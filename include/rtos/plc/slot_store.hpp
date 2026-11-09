// SPDX-License-Identifier: MIT
//
// A/B program slots for the PLC in the MCU's own flash (docs/plc.md "Program store"): the image
// and a small device configuration survive a power loss at any moment - after it, the old or the
// new program is there, never half of one.
//
//   flash region:  | slot A                                  | slot B                          |
//   slot:          | header | image ...          | journal  |
//
// - commit(image, config) writes the INACTIVE slot: erase, image, then the header with a
//   sequence number one above the active slot's and the CRCs. The header is the last thing
//   written, so a slot is valid only when it is complete. mount() picks the valid slot with the
//   highest sequence number; the other one stays as the previous program.
// - The configuration (control source, autostart, ...) changes without a new image: set_config()
//   appends a record to the active slot's journal (one program unit each; the newest valid record
//   wins). A full journal is moved: the image is copied into the other slot with a fresh journal.
// - A slot with an image length of 0 holds a configuration only (no program loaded yet).
//
// Flash: anything satisfying rtos::storage::FlashMemory (port flash::Region of the STM32H5/H7, a
// RAM fake in the host tests); its blocks are split in two halves. The calls block while the flash
// erases and programs (an STM32H7 sector erase: up to seconds): call them from a thread that may
// wait, never from the network thread.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "rtos/plc/image.hpp"
#include "rtos/storage/flash_memory.hpp"

namespace rtos::plc {

/// The device configuration kept with the program (8 bytes).
struct SlotConfig {
    std::uint8_t control_source = 0;  ///< 0: fieldbus, 1: IEC 61131 (docs/plc-iec61131.md 6.2)
    std::uint8_t autostart = 0;       ///< 1: RUN after power-on
    std::uint8_t halt_safe = 0;       ///< 1: outputs to the safe state while halted (P3)
    std::uint8_t reserved = 0;
    std::uint32_t reserved2 = 0;
    friend bool operator==(const SlotConfig&, const SlotConfig&) = default;
};
static_assert(sizeof(SlotConfig) == 8);

enum class SlotError : std::uint8_t { none, too_large, flash, no_image };

template <storage::FlashMemory Flash>
class SlotStore {
    static constexpr std::uint32_t round_up(std::uint32_t n, std::uint32_t unit) { return (n + unit - 1) / unit * unit; }

public:
    static constexpr std::uint32_t program_bytes = Flash::program_bytes;
    static constexpr std::uint32_t slot_blocks = Flash::block_count / 2;
    static constexpr std::uint32_t slot_bytes = slot_blocks * Flash::block_bytes;
    static constexpr std::uint32_t header_bytes = round_up(32, program_bytes);
    static constexpr std::uint32_t record_bytes = round_up(16, program_bytes);
    static constexpr std::uint32_t journal_bytes = round_up(4096, record_bytes);
    static constexpr std::uint32_t journal_offset = slot_bytes - journal_bytes;
    static constexpr std::uint32_t image_offset = header_bytes;
    static constexpr std::uint32_t max_image_bytes = journal_offset - image_offset;
    static constexpr std::uint32_t journal_records = journal_bytes / record_bytes;
    static_assert(Flash::block_count >= 2 && Flash::block_count % 2 == 0, "two slots of whole blocks");
    static_assert(program_bytes >= 4 && program_bytes <= 32 && 32 % program_bytes == 0,
                  "program unit: 4 .. 32 bytes (STM32H5: 16, STM32H7: 32)");
    static_assert(journal_offset > image_offset + 1024, "a slot holds at least 1 KB of image");

    explicit SlotStore(Flash& flash) noexcept : flash_(flash) {}

    /// Finds the active slot and the current configuration. Call once at start (and in tests after
    /// a simulated power loss).
    void mount() noexcept {
        active_ = -1;
        for (int s = 0; s < 2; ++s) {
            Header h;
            if (!read_header(s, h)) continue;
            if (active_ < 0 || h.seq > header_.seq) {
                active_ = s;
                header_ = h;
            }
        }
        config_ = active_ >= 0 ? header_.config : SlotConfig{};
        journal_next_ = 0;
        if (active_ >= 0) scan_journal();
    }

    [[nodiscard]] bool has_slot() const noexcept { return active_ >= 0; }
    [[nodiscard]] bool has_image() const noexcept { return active_ >= 0 && header_.image_bytes > 0; }
    [[nodiscard]] std::uint32_t image_bytes() const noexcept { return active_ >= 0 ? header_.image_bytes : 0; }
    [[nodiscard]] std::uint32_t image_crc() const noexcept { return active_ >= 0 ? header_.image_crc : 0; }
    [[nodiscard]] std::uint32_t sequence() const noexcept { return active_ >= 0 ? header_.seq : 0; }
    [[nodiscard]] int active_slot() const noexcept { return active_; }
    [[nodiscard]] SlotConfig config() const noexcept { return config_; }

    /// Copies the active image into `out` (at least image_bytes()); checks its CRC once more.
    SlotError read_image(std::span<std::uint8_t> out) const noexcept {
        if (!has_image()) return SlotError::no_image;
        if (out.size() < header_.image_bytes) return SlotError::too_large;
        if (!read(active_, image_offset, out.first(header_.image_bytes))) return SlotError::flash;
        return crc32(out.first(header_.image_bytes)) == header_.image_crc ? SlotError::none : SlotError::flash;
    }

    /// Writes `image` (may be empty: configuration only) and `config` into the inactive slot and
    /// makes it the active one.
    SlotError commit(std::span<const std::uint8_t> image, SlotConfig config) noexcept {
        if (image.size() > max_image_bytes) return SlotError::too_large;
        const int target = active_ == 0 ? 1 : 0;
        if (!erase_slot(target)) return SlotError::flash;
        // the image in program units; the last one is padded with 0xFF
        std::uint32_t at = 0;
        while (at < image.size()) {
            std::array<std::uint8_t, chunk_bytes> buf;
            buf.fill(0xFF);
            const std::uint32_t n = std::min<std::uint32_t>(chunk_bytes, static_cast<std::uint32_t>(image.size()) - at);
            std::memcpy(buf.data(), image.data() + at, n);
            if (!program(target, image_offset + at, std::as_bytes(std::span(buf)).first(round_up(n, program_bytes)))) {
                return SlotError::flash;
            }
            at += n;
        }
        return finish(target, static_cast<std::uint32_t>(image.size()), crc32(image), config);
    }

    /// A new configuration for the active program: one journal record (or a move to the other slot
    /// when the journal is full).
    SlotError set_config(SlotConfig config) noexcept {
        if (active_ < 0) return commit({}, config);
        if (config == config_) return SlotError::none;
        if (journal_next_ < journal_records) {
            std::array<std::uint8_t, record_bytes> rec;
            encode_record(rec, static_cast<std::uint16_t>(journal_seq_ + 1), config);
            const bool ok = program(active_, journal_offset + journal_next_ * record_bytes, std::as_bytes(std::span(rec)));
            ++journal_next_;  // a failed unit is skipped next time either way
            if (!ok) return SlotError::flash;
            ++journal_seq_;
            config_ = config;
            return SlotError::none;
        }
        return move(config);
    }

private:
    static constexpr std::uint32_t chunk_bytes = round_up(256, program_bytes);
    static constexpr std::array<std::uint8_t, 8> header_magic{'F', 'C', 'U', 'S', 'L', 'O', 'T', '1'};
    static constexpr std::uint16_t record_magic = 0x5A31;

    struct Header {
        std::uint32_t seq = 0;
        std::uint32_t image_bytes = 0;
        std::uint32_t image_crc = 0;
        SlotConfig config{};
    };

    bool read(int slot, std::uint32_t offset, std::span<std::uint8_t> out) const noexcept {
        // a range may cross blocks
        std::uint32_t done = 0;
        while (done < out.size()) {
            const std::uint32_t pos = offset + done;
            const std::uint32_t block = static_cast<std::uint32_t>(slot) * slot_blocks + pos / Flash::block_bytes;
            const std::uint32_t in_block = pos % Flash::block_bytes;
            const std::uint32_t n = std::min<std::uint32_t>(Flash::block_bytes - in_block, static_cast<std::uint32_t>(out.size()) - done);
            if (!flash_.read(block, in_block, std::as_writable_bytes(out.subspan(done, n)))) return false;
            done += n;
        }
        return true;
    }

    bool program(int slot, std::uint32_t offset, std::span<const std::byte> data) noexcept {
        std::uint32_t done = 0;
        while (done < data.size()) {
            const std::uint32_t pos = offset + done;
            const std::uint32_t block = static_cast<std::uint32_t>(slot) * slot_blocks + pos / Flash::block_bytes;
            const std::uint32_t in_block = pos % Flash::block_bytes;
            const std::uint32_t n = std::min<std::uint32_t>(Flash::block_bytes - in_block, static_cast<std::uint32_t>(data.size()) - done);
            if (!flash_.program(block, in_block, data.subspan(done, n))) return false;
            done += n;
        }
        return true;
    }

    bool erase_slot(int slot) noexcept {
        for (std::uint32_t b = 0; b < slot_blocks; ++b) {
            if (!flash_.erase(static_cast<std::uint32_t>(slot) * slot_blocks + b)) return false;
        }
        return true;
    }

    static void put32(void* p, std::uint32_t v) noexcept { std::memcpy(p, &v, 4); }
    static std::uint32_t get32(const std::uint8_t* p) noexcept { return rd32(p); }

    /// The header last: the slot becomes valid with it.
    SlotError finish(int target, std::uint32_t image_bytes, std::uint32_t image_crc, SlotConfig config) noexcept {
        std::array<std::uint8_t, header_bytes> h;
        h.fill(0xFF);
        const std::uint32_t seq = active_ >= 0 ? header_.seq + 1 : 1;
        std::memcpy(h.data(), header_magic.data(), 8);
        put32(&h[8], seq);
        put32(&h[12], image_bytes);
        put32(&h[16], image_crc);
        std::memcpy(&h[20], &config, sizeof config);
        put32(&h[28], crc32(std::span(h).first(28)));
        if (!program(target, 0, std::as_bytes(std::span(h)))) return SlotError::flash;
        // the new slot is valid from here on
        active_ = target;
        header_ = {seq, image_bytes, image_crc, config};
        config_ = config;
        journal_next_ = 0;
        journal_seq_ = 0;
        return SlotError::none;
    }

    bool read_header(int slot, Header& out) const noexcept {
        std::array<std::uint8_t, 32> h;
        if (!read(slot, 0, h)) return false;
        if (std::memcmp(h.data(), header_magic.data(), 8) != 0) return false;
        if (crc32(std::span(h).first(28)) != get32(&h[28])) return false;
        out.seq = get32(&h[8]);
        out.image_bytes = get32(&h[12]);
        out.image_crc = get32(&h[16]);
        std::memcpy(&out.config, &h[20], sizeof out.config);
        if (out.image_bytes > max_image_bytes) return false;
        // the image must be intact too (the header is written last, but check anyway)
        if (out.image_bytes > 0) {
            std::uint32_t crc = 0;
            std::array<std::uint8_t, 256> buf;
            for (std::uint32_t at = 0; at < out.image_bytes;) {
                const std::uint32_t n = std::min<std::uint32_t>(buf.size(), out.image_bytes - at);
                if (!read(slot, image_offset + at, std::span(buf).first(n))) return false;
                crc = crc32(std::span(buf).first(n), crc);
                at += n;
            }
            if (crc != out.image_crc) return false;
        }
        return true;
    }

    static void encode_record(std::span<std::uint8_t, record_bytes> rec, std::uint16_t seq, SlotConfig c) noexcept {
        std::ranges::fill(rec, std::uint8_t{0xFF});
        std::memcpy(&rec[0], &record_magic, 2);
        std::memcpy(&rec[2], &seq, 2);
        std::memcpy(&rec[4], &c, sizeof c);
        put32(&rec[12], crc32(rec.first(12)));
    }

    void scan_journal() noexcept {
        journal_seq_ = 0;
        for (std::uint32_t i = 0; i < journal_records; ++i) {
            std::array<std::uint8_t, record_bytes> rec;
            if (!read(active_, journal_offset + i * record_bytes, rec)) return;
            if (std::all_of(rec.begin(), rec.end(), [](std::uint8_t b) { return b == 0xFF; })) {
                journal_next_ = i;
                return;
            }
            journal_next_ = i + 1;  // written (valid or a broken write): not usable again
            std::uint16_t magic, seq;
            std::memcpy(&magic, &rec[0], 2);
            std::memcpy(&seq, &rec[2], 2);
            if (magic != record_magic || crc32(std::span(rec).first(12)) != get32(&rec[12])) continue;
            std::memcpy(&config_, &rec[4], sizeof config_);
            journal_seq_ = seq;
        }
    }

    /// The journal is full: the image into the other slot, with `config` in its header.
    SlotError move(SlotConfig config) noexcept {
        const int target = active_ == 0 ? 1 : 0;
        if (!erase_slot(target)) return SlotError::flash;
        std::uint32_t crc = 0;
        for (std::uint32_t at = 0; at < header_.image_bytes;) {
            std::array<std::uint8_t, chunk_bytes> buf;
            buf.fill(0xFF);
            const std::uint32_t n = std::min<std::uint32_t>(chunk_bytes, header_.image_bytes - at);
            if (!read(active_, image_offset + at, std::span(buf).first(n))) return SlotError::flash;
            crc = crc32(std::span(buf).first(n), crc);
            if (!program(target, image_offset + at, std::as_bytes(std::span(buf)).first(round_up(n, program_bytes)))) {
                return SlotError::flash;
            }
            at += n;
        }
        if (crc != header_.image_crc) return SlotError::flash;  // the active image is damaged: keep it as it is
        return finish(target, header_.image_bytes, crc, config);
    }

    Flash& flash_;
    int active_ = -1;
    Header header_{};
    SlotConfig config_{};
    std::uint32_t journal_next_ = 0;
    std::uint16_t journal_seq_ = 0;
};

}  // namespace rtos::plc
