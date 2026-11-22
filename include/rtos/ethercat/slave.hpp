// SPDX-License-Identifier: MIT
//
// EtherCAT slave: the PDI side of an ESC (EscPort) - what the application does so that a master
// (TwinCAT, SOEM, ...) finds, configures and runs the device.
//
//   EEPROM emulation  the ESC's EEPROM commands answered from the SII image in RAM (a board without
//                     EEPROM); also before the ESC is ready (the reload of the configuration area);
//                     the master's writes are kept in RAM until the reset
//   AL state machine  INIT -> PRE-OP -> SAFE-OP -> OP and back, each request checked (ETG.1000.6
//                     6.4): only the allowed transitions, the mailbox SyncManagers as the SII says
//                     (PRE-OP), the process data SyncManagers as the application's PDOs (SAFE-OP);
//                     a refused request sets the error flag and an AL status code, which the master
//                     acknowledges
//   diagnostics       every register a status page can show (read_diagnostics(): on request)
//
// Driven by poll(), from one thread: after the ESC's interrupt (AL event request), or periodically
// where the IRQ line is missing - every event is in the AL event request register, so polling only
// adds latency. The application (App) is told about state changes and EEPROM commands; it logs,
// drives LEDs and puts its outputs to the safe state.
//
// Pure (no hardware, no kernel): tests/ethercat_test.cpp against a simulated ESC.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/ethercat/esc.hpp"

namespace rtos::ethercat {

/// Where the mailbox and process data SyncManagers sit (the same as in the SII), and their sizes.
struct SlaveLayout {
    std::uint16_t mailbox_out_start = 0, mailbox_out_size = 0;  ///< SM0, master -> slave; size 0: no mailbox
    std::uint16_t mailbox_in_start = 0, mailbox_in_size = 0;    ///< SM1, slave -> master
    std::uint16_t outputs_start = 0, inputs_start = 0;          ///< SM2 (RxPDO), SM3 (TxPDO)
};

/// What the slave did, for the application's log and status page.
struct SlaveCounters {
    std::uint32_t eeprom_reads = 0, eeprom_writes = 0, eeprom_reloads = 0;
    std::uint32_t al_transitions = 0, al_refused = 0;
    std::uint16_t last_refused_code = 0;
    std::uint8_t last_refused_from = 0, last_refused_to = 0;
};

/// The ESC's registers for a status page (read_diagnostics()).
struct PortCounters {
    std::uint8_t invalid_frames = 0, rx_errors = 0, forwarded_errors = 0, lost_links = 0;
};
struct Diagnostics {
    std::uint8_t esc_type = 0, esc_revision = 0;
    std::uint16_t esc_build = 0;
    std::uint8_t fmmus = 0, sync_managers = 0, ram_kb = 0, port_descriptor = 0;
    std::uint16_t features = 0;
    std::uint16_t station_address = 0, station_alias = 0;
    std::uint16_t dl_status = 0, al_status = 0, al_status_code = 0;
    std::uint8_t pdi_control = 0, esc_config = 0;
    std::array<PortCounters, 2> ports{};
    std::uint8_t processing_unit_errors = 0, pdi_errors = 0;
    std::uint16_t watchdog_divider = 0, watchdog_time_process_data = 0;
    std::uint8_t watchdog_counter_process_data = 0, watchdog_counter_pdi = 0;
    std::uint16_t eeprom_status = 0;
    std::array<SyncManager, 4> sm{};
    std::array<Fmmu, 3> fmmu{};
    std::uint64_t system_time_ns = 0;
};

/// DL status bits (0110h): PDI operational, link and communication per port.
namespace dl_status {
inline constexpr std::uint16_t pdi_operational = 1u << 0;
[[nodiscard]] constexpr bool link(std::uint16_t dl, unsigned port) noexcept { return (dl & (1u << (4 + port))) != 0; }
[[nodiscard]] constexpr bool loop_closed(std::uint16_t dl, unsigned port) noexcept { return (dl & (1u << (8 + 2 * port))) != 0; }
[[nodiscard]] constexpr bool communication(std::uint16_t dl, unsigned port) noexcept { return (dl & (1u << (9 + 2 * port))) != 0; }
}  // namespace dl_status

/// The application side of a Slave. `from`/`to`: AL states (1, 2, 4, 8).
template <class A>
concept SlaveApp = requires(A& a, std::uint8_t from, std::uint8_t to, std::uint16_t code, unsigned cmd, std::uint32_t word) {
    { a.al_changed(from, to) } noexcept;            ///< after the AL status was written
    { a.al_refused(from, to, code) } noexcept;      ///< a request refused with an AL status code
    { a.eeprom_command(cmd, word) } noexcept;       ///< an EEPROM command served (log, rate-limited)
    { a.outputs_size() } noexcept -> std::convertible_to<std::uint16_t>;  ///< RxPDO bytes (0: none)
    { a.inputs_size() } noexcept -> std::convertible_to<std::uint16_t>;   ///< TxPDO bytes (0: none)
};

/// Whether an AL transition is allowed at all (ETG.1000.6 table 9): 0 or the AL status code.
[[nodiscard]] constexpr std::uint16_t transition_code(unsigned from, unsigned to) noexcept {
    if (to != 1 && to != 2 && to != 3 && to != 4 && to != 8) return al_code::unknown_state;
    if (to == 3) return al_code::bootstrap_not_supported;
    if (to <= from || to == 1) return al_code::none;  // down, or the same state (an acknowledge)
    const bool up_one = (from == 1 && to == 2) || (from == 2 && to == 4) || (from == 4 && to == 8);
    return up_one ? al_code::none : al_code::invalid_state_change;
}

/// A mailbox SyncManager as configured by the master against the layout: enabled, at the SII's
/// address and size, in mailbox mode, the direction `direction` (sm_control::write: master -> slave).
[[nodiscard]] constexpr bool mailbox_sm_ok(const SyncManager& sm, std::uint16_t start, std::uint16_t size,
                                           std::uint8_t direction) noexcept {
    return sm.enabled() && sm.start == start && sm.length == size &&
           (sm.control & sm_control::mode_mask) == sm_control::mailbox && (sm.control & sm_control::direction_mask) == direction;
}

/// A process data SyncManager: disabled (or of length 0) when there is no data; else enabled,
/// buffered, at the layout's address with the PDOs' size, the direction `direction`.
[[nodiscard]] constexpr bool process_sm_ok(const SyncManager& sm, std::uint16_t start, std::uint16_t size,
                                           std::uint8_t direction) noexcept {
    if (size == 0) return !sm.enabled() || sm.length == 0;
    return sm.enabled() && sm.start == start && sm.length == size &&
           (sm.control & sm_control::mode_mask) == sm_control::buffered && (sm.control & sm_control::direction_mask) == direction;
}

template <EscPort Esc, SlaveApp App, std::size_t SiiWords>
class Slave {
public:
    using Sii = std::array<std::uint16_t, SiiWords>;

    Slave(Esc& esc, const Sii& sii, const SlaveLayout& layout, App& app) noexcept
        : esc_(esc), sii_(sii), layout_(layout), app_(app) {}

    /// One pass. `ready`: the ESC has loaded its configuration (before, only the EEPROM emulation
    /// may touch the ESC - LAN9253 HW_CFG READY).
    void poll(bool ready) noexcept {
        for (int i = 0; i < 8; ++i) {  // a master reads the SII in bursts: several commands per pass
            if (!serve_eeprom()) break;
        }
        if (!ready) return;
        const std::uint32_t events = read32(esc_, reg::al_event_request);
        if ((events & al_event::al_control) != 0) serve_al_control();
    }

    /// The AL state the slave last wrote (1 INIT, 2 PRE-OP, 4 SAFE-OP, 8 OP).
    [[nodiscard]] std::uint8_t state() const noexcept { return state_; }
    [[nodiscard]] const SlaveCounters& counters() const noexcept { return counters_; }

    /// The ESC's registers now (about 150 bytes over the PDI). Not the watchdog status 0440h: reading
    /// it clears the watchdog event, which the process data handling takes.
    [[nodiscard]] Diagnostics read_diagnostics() noexcept {
        Diagnostics d{};
        std::array<std::byte, 10> id{};
        esc_.read(reg::type, id);
        const auto u = [&](std::size_t i) { return std::to_integer<std::uint8_t>(id[i]); };
        d.esc_type = u(0), d.esc_revision = u(1), d.esc_build = static_cast<std::uint16_t>(u(2) | (u(3) << 8));
        d.fmmus = u(4), d.sync_managers = u(5), d.ram_kb = u(6), d.port_descriptor = u(7);
        d.features = static_cast<std::uint16_t>(u(8) | (u(9) << 8));
        d.station_address = read16(esc_, reg::station_address);
        d.station_alias = read16(esc_, reg::station_alias);
        d.dl_status = read16(esc_, reg::dl_status);
        d.al_status = read16(esc_, reg::al_status);
        d.al_status_code = read16(esc_, reg::al_status_code);
        d.pdi_control = read8(esc_, reg::pdi_control);
        d.esc_config = read8(esc_, reg::esc_config);
        std::array<std::byte, 20> errors{};  // 0300h..0313h
        esc_.read(reg::rx_error_counters, errors);
        const auto e = [&](std::size_t i) { return std::to_integer<std::uint8_t>(errors[i]); };
        for (std::size_t p = 0; p < d.ports.size(); ++p) {
            d.ports[p] = {.invalid_frames = e(2 * p), .rx_errors = e(2 * p + 1), .forwarded_errors = e(8 + p), .lost_links = e(16 + p)};
        }
        d.processing_unit_errors = e(12), d.pdi_errors = e(13);
        d.watchdog_divider = read16(esc_, reg::watchdog_divider);
        d.watchdog_time_process_data = read16(esc_, reg::watchdog_time_process_data);
        d.watchdog_counter_process_data = read8(esc_, reg::watchdog_counter_process_data);
        d.watchdog_counter_pdi = read8(esc_, reg::watchdog_counter_pdi);
        d.eeprom_status = read16(esc_, reg::eeprom_status);
        for (unsigned n = 0; n < d.sm.size(); ++n) d.sm[n] = read_sm_config(n);
        for (unsigned n = 0; n < d.fmmu.size(); ++n) d.fmmu[n] = read_fmmu(esc_, n);
        std::array<std::byte, 8> t{};
        esc_.read(reg::system_time, t);
        for (int i = 7; i >= 0; --i) d.system_time_ns = (d.system_time_ns << 8) | std::to_integer<std::uint64_t>(t[static_cast<std::size_t>(i)]);
        return d;
    }

    /// Back to INIT from the application (e.g. the ESC was reset): state and counters stay coherent
    /// with the ESC, which starts in INIT.
    void reset() noexcept { state_ = 1; }

private:
    // ---- EEPROM emulation (ESC 0502h-050Fh) -----------------------------------------------------
    /// One pending command; false: none pending.
    bool serve_eeprom() noexcept {
        const std::uint16_t status = read16(esc_, reg::eeprom_status);
        if ((status & eeprom::busy) == 0 || (status & eeprom::emulation) == 0) return false;
        const unsigned cmd = (status >> 8) & 0x7;
        const std::uint32_t address = read32(esc_, reg::eeprom_address);  // word address
        const auto word = [this](std::uint32_t a) -> std::uint32_t { return a < sii_.size() ? sii_[a] : 0xFFFF; };
        if (cmd == eeprom::cmd_read || cmd == eeprom::cmd_reload) {
            write32(esc_, reg::eeprom_data, word(address) | (word(address + 1) << 16));
            if ((status & eeprom::read_8_bytes) != 0) {
                write32(esc_, reg::eeprom_data + 4, word(address + 2) | (word(address + 3) << 16));
            }
            ++(cmd == eeprom::cmd_read ? counters_.eeprom_reads : counters_.eeprom_reloads);
        } else if (cmd == eeprom::cmd_write) {
            // kept in RAM until the reset: a master reads back what it wrote (e.g. the station alias)
            if (address < sii_.size()) sii_[address] = read16(esc_, reg::eeprom_data);
            ++counters_.eeprom_writes;
        }
        write16(esc_, reg::eeprom_status, static_cast<std::uint16_t>(cmd << 8));  // acknowledge: executed
        app_.eeprom_command(cmd, address);
        return true;
    }

    // ---- AL state machine (ETG.1000.6 6.4) ----------------------------------------------------------
    void serve_al_control() noexcept {
        const std::uint16_t control = read16(esc_, reg::al_control);  // the read clears the event
        const std::uint16_t status = read16(esc_, reg::al_status);
        const auto from = static_cast<std::uint8_t>(status & 0x0F);
        const auto to = static_cast<std::uint8_t>(control & 0x0F);
        const bool error = (status & al_error_flag) != 0, acknowledge = (control & al_error_flag) != 0;
        if (error && !acknowledge) return;  // an indicated error stays until the master acknowledges it
        if (!error && to == from) return;
        std::uint16_t code = transition_code(from, to);
        if (code == al_code::none && from == 1 && to == 2) code = check_mailbox();
        if (code == al_code::none && from == 2 && to == 4) code = check_process_data();
        if (code != al_code::none) {
            write16(esc_, reg::al_status_code, code);
            write16(esc_, reg::al_status, static_cast<std::uint16_t>(from | al_error_flag));
            ++counters_.al_refused;
            counters_.last_refused_code = code, counters_.last_refused_from = from, counters_.last_refused_to = to;
            app_.al_refused(from, to, code);
            return;
        }
        write16(esc_, reg::al_status_code, al_code::none);
        write16(esc_, reg::al_status, to);
        if (to != from) ++counters_.al_transitions;
        state_ = to;
        app_.al_changed(from, to);
    }

    [[nodiscard]] std::uint16_t check_mailbox() noexcept {
        if (layout_.mailbox_out_size == 0) return al_code::none;
        const bool ok = mailbox_sm_ok(read_sm_config(0), layout_.mailbox_out_start, layout_.mailbox_out_size, sm_control::write) &&
                        mailbox_sm_ok(read_sm_config(1), layout_.mailbox_in_start, layout_.mailbox_in_size, sm_control::read);
        return ok ? al_code::none : al_code::invalid_mailbox_config;
    }

    [[nodiscard]] std::uint16_t check_process_data() noexcept {
        if (!process_sm_ok(read_sm_config(2), layout_.outputs_start, app_.outputs_size(), sm_control::write)) {
            return al_code::invalid_output_config;
        }
        if (!process_sm_ok(read_sm_config(3), layout_.inputs_start, app_.inputs_size(), sm_control::read)) {
            return al_code::invalid_input_config;
        }
        return al_code::none;
    }

    /// A SyncManager's registers; reading the activate byte also clears the "activation changed"
    /// event (nothing waits for it: the configuration is checked at the transitions).
    [[nodiscard]] SyncManager read_sm_config(unsigned n) noexcept { return read_sm(esc_, n); }

    Esc& esc_;
    Sii sii_;  ///< a copy: the master's EEPROM writes go here
    SlaveLayout layout_;
    App& app_;
    SlaveCounters counters_{};
    std::uint8_t state_ = 1;
};

}  // namespace rtos::ethercat
