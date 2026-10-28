// SPDX-License-Identifier: MIT
//
// SD memory card protocol (SD Physical Layer Simplified Specification): initialisation,
// capacity, block reads and writes - over any host controller that satisfies SdHost.
//
//   SdCard<Sdmmc1> card{host};
//   if (card.init() == SdStatus::Ok) {
//       card.read(0, std::span(sector));          // 512-byte blocks, word aligned
//   }
//
// Initialisation (spec. 4.2): CMD0, CMD8 (v2 cards echo the check pattern; v1 cards don't
// answer), ACMD41 with HCS until the card is ready (CCS: block-addressed SDHC/SDXC, else
// byte-addressed SDSC), CMD2, CMD3 (relative address), CMD9 (CSD -> capacity), CMD7 (select),
// CMD16 (512-byte blocks, SDSC only), then the transfer clock. Multi-block transfers end with
// CMD12; after a write the card is polled with CMD13 until it is back in the transfer state.
//
// The protocol does not touch hardware: host tests drive it with a simulated card.
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos::storage {

enum class SdResponse : std::uint8_t {
    None,
    R1,   ///< card status (CRC checked)
    R1b,  ///< R1, then busy on D0 until the card is done
    R2,   ///< 136 bits: CID or CSD
    R3,   ///< OCR (no CRC)
    R6,   ///< relative card address
    R7,   ///< interface condition
};

enum class SdStatus : std::uint8_t { Ok, Timeout, CrcError, CardError, NoCard, Unsupported, NotReady };

struct SdCommandResult {
    SdStatus status = SdStatus::Ok;
    std::array<std::uint32_t, 4> response{};  ///< [0] = bits 127:96 (R2) or the short response
};

/// What the protocol needs from a host controller.
template <class H>
concept SdHost = requires(H& h, std::uint8_t index, std::uint32_t arg, SdResponse r, std::span<std::byte> in,
                          std::span<const std::byte> out, std::uint32_t ms) {
    { h.power_on() } noexcept;                          // clock on at the identification rate
    { h.set_transfer_clock() } noexcept;                // after selection: full speed
    { h.command(index, arg, r) } noexcept -> std::same_as<SdCommandResult>;
    { h.stop(r) } noexcept -> std::same_as<SdCommandResult>;   // CMD12 marked as a stop command
    { h.read_data(index, arg, in) } noexcept -> std::same_as<SdCommandResult>;   // in: n x 512
    { h.write_data(index, arg, out) } noexcept -> std::same_as<SdCommandResult>;
    { h.delay_ms(ms) } noexcept;
};

inline constexpr std::size_t sd_block_size = 512;

/// Bits [hi:lo] of a 128-bit register given as four words, [0] = bits 127:96.
[[nodiscard]] constexpr std::uint32_t sd_bits(const std::array<std::uint32_t, 4>& r, unsigned hi, unsigned lo) noexcept {
    std::uint32_t v = 0;
    for (unsigned b = hi + 1; b-- > lo;) {
        const std::uint32_t word = r[3 - b / 32];
        v = (v << 1) | ((word >> (b % 32)) & 1u);
    }
    return v;
}

/// Capacity in 512-byte blocks from the CSD (versions 1.0 and 2.0); 0 if unknown.
[[nodiscard]] constexpr std::uint64_t sd_csd_blocks(const std::array<std::uint32_t, 4>& csd) noexcept {
    switch (sd_bits(csd, 127, 126)) {
        case 0: {  // CSD 1.0 (SDSC)
            const std::uint64_t c_size = sd_bits(csd, 73, 62);
            const std::uint32_t mult = sd_bits(csd, 49, 47);
            const std::uint32_t read_bl_len = sd_bits(csd, 83, 80);
            return ((c_size + 1) << (mult + 2) << read_bl_len) / sd_block_size;
        }
        case 1:  // CSD 2.0 (SDHC/SDXC): (C_SIZE + 1) x 512 KB
            return (static_cast<std::uint64_t>(sd_bits(csd, 69, 48)) + 1) * 1024;
        default: return 0;
    }
}

template <SdHost Host>
class SdCard {
public:
    explicit SdCard(Host& host) noexcept : host_(host) {}

    SdStatus init() noexcept {
        ready_ = false;
        host_.power_on();
        (void)host_.command(0, 0, SdResponse::None);  // GO_IDLE_STATE
        const SdCommandResult if_cond = host_.command(8, 0x1AA, SdResponse::R7);  // 2.7-3.6 V, pattern AA
        bool v2 = false;
        if (if_cond.status == SdStatus::Ok) {
            if ((if_cond.response[0] & 0xFFFu) != 0x1AA) {
                return SdStatus::Unsupported;  // voltage not accepted or pattern wrong
            }
            v2 = true;
        } else if (if_cond.status != SdStatus::Timeout) {
            return if_cond.status;
        }
        // ACMD41 until the card leaves its power-up (bit 31), at most ~1 s (spec 4.2.3)
        std::uint32_t ocr = 0;
        for (int attempt = 0;; ++attempt) {
            if (const SdCommandResult app = host_.command(55, 0, SdResponse::R1); app.status != SdStatus::Ok) {
                return attempt == 0 && app.status == SdStatus::Timeout ? SdStatus::NoCard : app.status;
            }
            const SdCommandResult op = host_.command(41, (v2 ? 1u << 30 : 0u) | 0x00FF'8000u, SdResponse::R3);
            if (op.status != SdStatus::Ok) {
                return op.status;
            }
            ocr = op.response[0];
            if ((ocr & (1u << 31)) != 0) {
                break;
            }
            if (attempt >= 1000) {
                return SdStatus::Timeout;
            }
            host_.delay_ms(1);
        }
        block_addressed_ = (ocr & (1u << 30)) != 0;

        const SdCommandResult cid = host_.command(2, 0, SdResponse::R2);
        if (cid.status != SdStatus::Ok) {
            return cid.status;
        }
        cid_ = cid.response;
        const SdCommandResult rca = host_.command(3, 0, SdResponse::R6);
        if (rca.status != SdStatus::Ok) {
            return rca.status;
        }
        rca_ = rca.response[0] & 0xFFFF'0000u;
        const SdCommandResult csd = host_.command(9, rca_, SdResponse::R2);
        if (csd.status != SdStatus::Ok) {
            return csd.status;
        }
        csd_ = csd.response;
        blocks_ = sd_csd_blocks(csd_);
        if (blocks_ == 0) {
            return SdStatus::Unsupported;
        }
        if (const SdCommandResult sel = host_.command(7, rca_, SdResponse::R1b); sel.status != SdStatus::Ok) {
            return sel.status;
        }
        if (!block_addressed_) {
            if (const SdCommandResult len = host_.command(16, sd_block_size, SdResponse::R1); len.status != SdStatus::Ok) {
                return len.status;
            }
        }
        host_.set_transfer_clock();
        ready_ = true;
        return SdStatus::Ok;
    }

    /// Read whole blocks starting at `block` into `out` (a multiple of 512 bytes).
    SdStatus read(std::uint64_t block, std::span<std::byte> out) noexcept {
        if (const SdStatus s = check(block, out.size()); s != SdStatus::Ok) {
            return s;
        }
        const bool multi = out.size() > sd_block_size;
        const SdCommandResult r = host_.read_data(multi ? 18 : 17, address(block), out);
        if (multi) {
            const SdCommandResult stop = host_.stop(SdResponse::R1b);
            if (r.status == SdStatus::Ok && stop.status != SdStatus::Ok) {
                return stop.status;
            }
        }
        return r.status;
    }

    /// Write whole blocks starting at `block`; returns once the card has programmed them.
    SdStatus write(std::uint64_t block, std::span<const std::byte> data) noexcept {
        if (const SdStatus s = check(block, data.size()); s != SdStatus::Ok) {
            return s;
        }
        const bool multi = data.size() > sd_block_size;
        const SdCommandResult r = host_.write_data(multi ? 25 : 24, address(block), data);
        if (multi) {
            (void)host_.stop(SdResponse::R1b);
        }
        if (r.status != SdStatus::Ok) {
            return r.status;
        }
        return wait_transfer_state();
    }

    [[nodiscard]] bool ready() const noexcept { return ready_; }
    [[nodiscard]] std::uint64_t blocks() const noexcept { return blocks_; }
    [[nodiscard]] bool high_capacity() const noexcept { return block_addressed_; }
    [[nodiscard]] const std::array<std::uint32_t, 4>& cid() const noexcept { return cid_; }
    [[nodiscard]] const std::array<std::uint32_t, 4>& csd() const noexcept { return csd_; }

private:
    SdStatus check(std::uint64_t block, std::size_t bytes) const noexcept {
        if (!ready_) {
            return SdStatus::NotReady;
        }
        if (bytes == 0 || bytes % sd_block_size != 0 || block + bytes / sd_block_size > blocks_) {
            return SdStatus::CardError;
        }
        return SdStatus::Ok;
    }

    [[nodiscard]] std::uint32_t address(std::uint64_t block) const noexcept {
        return static_cast<std::uint32_t>(block_addressed_ ? block : block * sd_block_size);
    }

    /// CMD13 until CURRENT_STATE (bits 12:9) = tran (4) and READY_FOR_DATA (bit 8): up to ~1 s.
    SdStatus wait_transfer_state() noexcept {
        for (int i = 0; i < 1000; ++i) {
            const SdCommandResult s = host_.command(13, rca_, SdResponse::R1);
            if (s.status != SdStatus::Ok) {
                return s.status;
            }
            if (((s.response[0] >> 9) & 0xFu) == 4 && (s.response[0] & (1u << 8)) != 0) {
                return SdStatus::Ok;
            }
            host_.delay_ms(1);
        }
        return SdStatus::Timeout;
    }

    Host& host_;
    std::uint32_t rca_ = 0;
    std::uint64_t blocks_ = 0;
    bool block_addressed_ = false;
    bool ready_ = false;
    std::array<std::uint32_t, 4> cid_{};
    std::array<std::uint32_t, 4> csd_{};
};

}  // namespace rtos::storage
