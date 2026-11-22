// SPDX-License-Identifier: MIT
//
// A simulated ESC for the slave stack's host tests (ethercat_test.cpp): the ESC memory as the PDI
// sees it (EscPort), and a master side that does what a master's datagrams do - with the ESC's
// side effects that the slave relies on (LAN9253 DS00003421B 11.16, ETG.1000.4):
//
//   AL control      a master write sets AL event 0; a PDI read of 0120h clears it
//   EEPROM          a master command sets busy, the command and AL event 5 (emulation); the PDI's
//                   acknowledge (write of 0502h) clears busy and the event
//   mailbox         SM0: the master's write of the whole buffer sets "full" and AL event 8; the PDI's
//                   read of its last byte clears it. SM1: the PDI's write of its last byte sets
//                   "full" (AL event 9 cleared); the master's read clears it. The repeat request is
//                   SM1 activate bit 1, toggled by the master
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "rtos/ethercat/esc.hpp"

namespace ecat_sim {

namespace ec = rtos::ethercat;

class SimEsc {
public:
    SimEsc() {
        mem_.fill(std::byte{0});
        set16(ec::reg::al_status, 1);  // INIT
        set8(ec::reg::type, 0xC0), set8(ec::reg::fmmus, 3), set8(ec::reg::sync_managers, 8), set8(ec::reg::ram_kb, 8);
        set16(ec::reg::eeprom_status, ec::eeprom::emulation);
    }

    // ---- EscPort (the PDI) ------------------------------------------------------------------------
    void read(std::uint16_t address, std::span<std::byte> out) noexcept {
        for (std::size_t i = 0; i < out.size(); ++i) out[i] = mem_[(address + i) & mask];
        ++pdi_reads;
        if (overlaps(address, out.size(), ec::reg::al_control, 2)) clear_event(ec::al_event::al_control);
        if (const auto sm0 = sm_config(0); mailbox_enabled(sm0) && overlaps(address, out.size(), sm0.start + sm0.length - 1, 1)) {
            set8(ec::reg::sm_base + 5, get8(ec::reg::sm_base + 5) & ~ec::sm_status::mailbox_full);
            clear_event(ec::al_event::sm(0));
        }
    }
    void write(std::uint16_t address, std::span<const std::byte> in) noexcept {
        for (std::size_t i = 0; i < in.size(); ++i) mem_[(address + i) & mask] = in[i];
        ++pdi_writes;
        if (const auto sm1 = sm_config(1); mailbox_enabled(sm1) && overlaps(address, in.size(), sm1.start + sm1.length - 1, 1)) {
            set8(ec::reg::sm_base + ec::reg::sm_size + 5, get8(ec::reg::sm_base + ec::reg::sm_size + 5) | ec::sm_status::mailbox_full);
            ++sm1_writes;
        }
        if (overlaps(address, in.size(), ec::reg::eeprom_status, 2)) {  // the PDI acknowledges a command
            set16(ec::reg::eeprom_status, static_cast<std::uint16_t>(get16(ec::reg::eeprom_status) & ~ec::eeprom::busy) |
                                              ec::eeprom::emulation);
            clear_event(ec::al_event::eeprom);
        }
    }

    // ---- the master's side ------------------------------------------------------------------------
    void master_write(std::uint16_t address, std::span<const std::byte> in) {
        for (std::size_t i = 0; i < in.size(); ++i) mem_[(address + i) & mask] = in[i];
        if (overlaps(address, in.size(), ec::reg::al_control, 2)) set_event(ec::al_event::al_control);
    }
    void master_write16(std::uint16_t address, std::uint16_t v) {
        const std::array<std::byte, 2> b{std::byte(v & 0xFF), std::byte(v >> 8)};
        master_write(address, b);
    }
    [[nodiscard]] std::uint16_t master_read16(std::uint16_t address) const { return get16(address); }

    /// AL control: request a state (with the error acknowledge bit if `ack`).
    void request(std::uint8_t state, bool ack = false) { master_write16(ec::reg::al_control, state | (ack ? ec::al_error_flag : 0)); }
    [[nodiscard]] std::uint16_t al_status() const { return get16(ec::reg::al_status); }
    [[nodiscard]] std::uint16_t al_code() const { return get16(ec::reg::al_status_code); }

    /// An EEPROM command as the master issues it (0502h/0504h); the data is read back with
    /// eeprom_data() once the slave has acknowledged it.
    void eeprom_command(unsigned cmd, std::uint32_t word_address, bool eight_bytes = false) {
        set32(ec::reg::eeprom_address, word_address);
        set16(ec::reg::eeprom_status, static_cast<std::uint16_t>(ec::eeprom::busy | ec::eeprom::emulation | (cmd << 8) |
                                                                 (eight_bytes ? ec::eeprom::read_8_bytes : 0)));
        set_event(ec::al_event::eeprom);
    }
    [[nodiscard]] bool eeprom_busy() const { return (get16(ec::reg::eeprom_status) & ec::eeprom::busy) != 0; }
    [[nodiscard]] std::uint32_t eeprom_data(unsigned offset = 0) const { return get32(ec::reg::eeprom_data + offset); }
    void set_eeprom_write_data(std::uint16_t v) { set16(ec::reg::eeprom_data, v); }

    /// A SyncManager configured by the master.
    void configure_sm(unsigned n, std::uint16_t start, std::uint16_t length, std::uint8_t control, bool enable) {
        const auto base = static_cast<std::uint16_t>(ec::reg::sm_base + ec::reg::sm_size * n);
        set16(base, start), set16(base + 2, length), set8(base + 4, control), set8(base + 6, enable ? 1 : 0);
    }

    [[nodiscard]] std::uint32_t events() const { return get32(ec::reg::al_event_request); }

    // ---- mailbox, the master's side ----------------------------------------------------------------
    /// A mailbox message into SM0: header (length, address 0, channel 0, type, counter) and `data`.
    /// false: the mailbox is still full (the slave has not read the last one).
    bool mailbox_write(std::uint8_t type, std::uint8_t counter, std::span<const std::byte> data) {
        const auto sm0 = sm_config(0);
        if ((get8(ec::reg::sm_base + 5) & ec::sm_status::mailbox_full) != 0) return false;
        std::vector<std::byte> b(sm0.length, std::byte{0});
        b[0] = std::byte(data.size() & 0xFF), b[1] = std::byte(data.size() >> 8);
        b[5] = std::byte(type | (counter << 4));
        std::copy(data.begin(), data.end(), b.begin() + 6);
        master_write(sm0.start, b);
        set8(ec::reg::sm_base + 5, get8(ec::reg::sm_base + 5) | ec::sm_status::mailbox_full);
        set_event(ec::al_event::sm(0));
        return true;
    }
    /// The SM1 buffer if full (then empty again), else empty.
    std::vector<std::byte> mailbox_read() {
        const auto sm1 = sm_config(1);
        if ((get8(ec::reg::sm_base + ec::reg::sm_size + 5) & ec::sm_status::mailbox_full) == 0) return {};
        std::vector<std::byte> b(sm1.length);
        for (std::size_t i = 0; i < b.size(); ++i) b[i] = mem_[(sm1.start + i) & mask];
        set8(ec::reg::sm_base + ec::reg::sm_size + 5, get8(ec::reg::sm_base + ec::reg::sm_size + 5) & ~ec::sm_status::mailbox_full);
        return b;
    }
    /// The master asks for the last response again: SM1 activate bit 1 toggled.
    void toggle_repeat_request() { set8(ec::reg::sm_base + ec::reg::sm_size + 6, get8(ec::reg::sm_base + ec::reg::sm_size + 6) ^ 0x02); }
    [[nodiscard]] bool repeat_acknowledged() const {
        return ((get8(ec::reg::sm_base + ec::reg::sm_size + 6) ^ get8(ec::reg::sm_base + ec::reg::sm_size + 7)) & 0x02) == 0;
    }

    std::size_t pdi_reads = 0, pdi_writes = 0, sm1_writes = 0;

    void set8(std::uint16_t a, std::uint8_t v) { mem_[a & mask] = std::byte{v}; }
    void set16(std::uint16_t a, std::uint16_t v) { set8(a, static_cast<std::uint8_t>(v)), set8(a + 1, static_cast<std::uint8_t>(v >> 8)); }
    void set32(std::uint16_t a, std::uint32_t v) { set16(a, static_cast<std::uint16_t>(v)), set16(a + 2, static_cast<std::uint16_t>(v >> 16)); }
    [[nodiscard]] std::uint8_t get8(std::uint16_t a) const { return std::to_integer<std::uint8_t>(mem_[a & mask]); }
    [[nodiscard]] std::uint16_t get16(std::uint16_t a) const { return static_cast<std::uint16_t>(get8(a) | (get8(a + 1) << 8)); }
    [[nodiscard]] std::uint32_t get32(std::uint16_t a) const { return get16(a) | (std::uint32_t{get16(a + 2)} << 16); }

private:
    static constexpr std::size_t size = 0x4000, mask = size - 1;
    [[nodiscard]] ec::SyncManager sm_config(unsigned n) const {
        const auto base = static_cast<std::uint16_t>(ec::reg::sm_base + ec::reg::sm_size * n);
        return {.start = get16(base), .length = get16(base + 2), .control = get8(base + 4), .status = get8(base + 5),
                .activate = get8(base + 6), .pdi_control = get8(base + 7)};
    }
    static bool mailbox_enabled(const ec::SyncManager& sm) { return sm.enabled() && sm.length > 0 && (sm.control & 0x03) == 0x02; }
    static bool overlaps(std::uint16_t a, std::size_t n, std::uint16_t r, std::size_t rn) { return a < r + rn && r < a + n; }
    void set_event(std::uint32_t bit) { set32(ec::reg::al_event_request, get32(ec::reg::al_event_request) | bit); }
    void clear_event(std::uint32_t bit) { set32(ec::reg::al_event_request, get32(ec::reg::al_event_request) & ~bit); }

    std::array<std::byte, size> mem_{};
};

static_assert(ec::EscPort<SimEsc>);

}  // namespace ecat_sim
