// SPDX-License-Identifier: MIT
//
// EtherCAT slave controller (ESC) access for the slave stack (rtos::ethercat): the ESC registers
// as the PDI (the host side) sees them, and the EscPort concept a driver satisfies.
//
// The registers are those of ETG.1000.4 / ETG.1000.6, as implemented by the LAN9253 (DS00003421B
// 11.16: 8 SyncManagers, 3 FMMUs, process RAM 1000h-2FFFh). Multi-byte registers are little endian.
//
//   rtos::devices::Lan9253<Spi> lan;          // satisfies EscPort
//   const std::uint16_t al = rtos::ethercat::read16(lan, rtos::ethercat::reg::al_status);
//
// An EscPort reads and writes a run of bytes at an ESC address (registers and process RAM alike);
// the driver handles the bus. Host tests use a simulated ESC (tests/ethercat_sim.hpp).
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace rtos::ethercat {

template <class E>
concept EscPort = requires(E& e, std::uint16_t address, std::span<std::byte> out, std::span<const std::byte> in) {
    { e.read(address, out) } noexcept -> std::same_as<void>;
    { e.write(address, in) } noexcept -> std::same_as<void>;
};

// ---- registers (byte addresses) ----------------------------------------------------------------------
namespace reg {
inline constexpr std::uint16_t type = 0x0000, revision = 0x0001, build = 0x0002, fmmus = 0x0004, sync_managers = 0x0005,
                               ram_kb = 0x0006, port_descriptor = 0x0007, features = 0x0008;
inline constexpr std::uint16_t station_address = 0x0010, station_alias = 0x0012;
inline constexpr std::uint16_t dl_control = 0x0100, dl_status = 0x0110;
inline constexpr std::uint16_t al_control = 0x0120, al_status = 0x0130, al_status_code = 0x0134, run_led_override = 0x0138,
                               err_led_override = 0x0139;
inline constexpr std::uint16_t pdi_control = 0x0140, esc_config = 0x0141, pdi_config = 0x0150;
inline constexpr std::uint16_t al_event_mask = 0x0204, al_event_request = 0x0220;
inline constexpr std::uint16_t rx_error_counters = 0x0300;  ///< 4 ports x (invalid frame, RX error), 1 byte each
inline constexpr std::uint16_t forwarded_rx_errors = 0x0308, processing_unit_errors = 0x030C, pdi_errors = 0x030D,
                               lost_link_counters = 0x0310;
inline constexpr std::uint16_t watchdog_divider = 0x0400, watchdog_time_pdi = 0x0410, watchdog_time_process_data = 0x0420,
                               watchdog_status_process_data = 0x0440, watchdog_counter_process_data = 0x0442,
                               watchdog_counter_pdi = 0x0443;
inline constexpr std::uint16_t eeprom_config = 0x0500, eeprom_pdi_access = 0x0501, eeprom_status = 0x0502,
                               eeprom_address = 0x0504, eeprom_data = 0x0508;
inline constexpr std::uint16_t mii_control = 0x0510, mii_phy_address = 0x0512, mii_register = 0x0513, mii_data = 0x0514,
                               mii_ecat_access = 0x0516, mii_pdi_access = 0x0517;
inline constexpr std::uint16_t fmmu_base = 0x0600, fmmu_size = 16;
inline constexpr std::uint16_t sm_base = 0x0800, sm_size = 8;
inline constexpr std::uint16_t system_time = 0x0910;
inline constexpr std::uint16_t process_ram = 0x1000, process_ram_end = 0x3000;  ///< LAN9253: 8 KB
}  // namespace reg

// ---- AL states, status codes, events --------------------------------------------------------------
enum class AlState : std::uint8_t { init = 1, preop = 2, boot = 3, safeop = 4, op = 8 };

[[nodiscard]] constexpr std::string_view state_name(unsigned s) noexcept {
    switch (s & 0x0F) {
        case 1: return "INIT";
        case 2: return "PRE-OP";
        case 3: return "BOOT";
        case 4: return "SAFE-OP";
        case 8: return "OP";
        default: return "?";
    }
}

inline constexpr std::uint16_t al_error_flag = 0x10;  ///< AL status: error indicated; AL control: error acknowledge

/// AL status codes (ETG.1000.6 table 11).
namespace al_code {
inline constexpr std::uint16_t none = 0x0000, unspecified = 0x0001, invalid_state_change = 0x0011, unknown_state = 0x0012,
                               bootstrap_not_supported = 0x0013, invalid_mailbox_config = 0x0016,
                               invalid_sm_config = 0x0017, sm_watchdog = 0x001B, invalid_output_config = 0x001D,
                               invalid_input_config = 0x001E;
[[nodiscard]] constexpr std::string_view text(std::uint16_t code) noexcept {
    switch (code) {
        case none: return "no error";
        case unspecified: return "unspecified error";
        case invalid_state_change: return "invalid requested state change";
        case unknown_state: return "unknown requested state";
        case bootstrap_not_supported: return "bootstrap not supported";
        case invalid_mailbox_config: return "invalid mailbox configuration";
        case invalid_sm_config: return "invalid sync manager configuration";
        case sm_watchdog: return "sync manager watchdog";
        case invalid_output_config: return "invalid output configuration";
        case invalid_input_config: return "invalid input configuration";
        default: return "?";
    }
}
}  // namespace al_code

/// AL event request bits (0220h, LAN9253 11.16.35).
namespace al_event {
inline constexpr std::uint32_t al_control = 1u << 0, sm_activation = 1u << 4, eeprom = 1u << 5, watchdog = 1u << 6;
[[nodiscard]] constexpr std::uint32_t sm(unsigned n) noexcept { return 1u << (8 + n); }
}  // namespace al_event

/// EEPROM control/status (0502h): busy, emulation by the PDI, 8-byte reads, the command (bits 10:8).
namespace eeprom {
inline constexpr std::uint16_t busy = 1u << 15, emulation = 1u << 5, read_8_bytes = 1u << 6, checksum_error = 1u << 11,
                               not_loaded = 1u << 12;
inline constexpr unsigned cmd_read = 1, cmd_write = 2, cmd_reload = 4;
}  // namespace eeprom

// ---- typed access --------------------------------------------------------------------------------
template <EscPort E>
[[nodiscard]] std::uint8_t read8(E& esc, std::uint16_t address) noexcept {
    std::array<std::byte, 1> b{};
    esc.read(address, b);
    return std::to_integer<std::uint8_t>(b[0]);
}
template <EscPort E>
[[nodiscard]] std::uint16_t read16(E& esc, std::uint16_t address) noexcept {
    std::array<std::byte, 2> b{};
    esc.read(address, b);
    return static_cast<std::uint16_t>(std::to_integer<unsigned>(b[0]) | (std::to_integer<unsigned>(b[1]) << 8));
}
template <EscPort E>
[[nodiscard]] std::uint32_t read32(E& esc, std::uint16_t address) noexcept {
    std::array<std::byte, 4> b{};
    esc.read(address, b);
    std::uint32_t v = 0;
    for (int i = 3; i >= 0; --i) v = (v << 8) | std::to_integer<std::uint32_t>(b[static_cast<std::size_t>(i)]);
    return v;
}
template <EscPort E>
void write8(E& esc, std::uint16_t address, std::uint8_t value) noexcept {
    const std::array<std::byte, 1> b{std::byte{value}};
    esc.write(address, b);
}
template <EscPort E>
void write16(E& esc, std::uint16_t address, std::uint16_t value) noexcept {
    const std::array<std::byte, 2> b{std::byte(value & 0xFF), std::byte(value >> 8)};
    esc.write(address, b);
}
template <EscPort E>
void write32(E& esc, std::uint16_t address, std::uint32_t value) noexcept {
    const std::array<std::byte, 4> b{std::byte(value & 0xFF), std::byte((value >> 8) & 0xFF), std::byte((value >> 16) & 0xFF),
                                     std::byte(value >> 24)};
    esc.write(address, b);
}

// ---- SyncManagers, FMMUs --------------------------------------------------------------------------
/// One SyncManager's registers (0800h + 8n).
struct SyncManager {
    std::uint16_t start = 0, length = 0;
    std::uint8_t control = 0, status = 0, activate = 0, pdi_control = 0;

    [[nodiscard]] constexpr bool enabled() const noexcept { return (activate & 1) != 0; }
    friend constexpr bool operator==(const SyncManager&, const SyncManager&) = default;
};

/// SyncManager control byte: operation mode (bits 1:0), direction (3:2), interrupts, watchdog.
namespace sm_control {
inline constexpr std::uint8_t buffered = 0x00, mailbox = 0x02, mode_mask = 0x03;
inline constexpr std::uint8_t read = 0x00, write = 0x04, direction_mask = 0x0C;  ///< seen from the master
inline constexpr std::uint8_t ecat_irq = 0x10, pdi_irq = 0x20, watchdog = 0x40;
}  // namespace sm_control
/// SyncManager status byte.
namespace sm_status {
inline constexpr std::uint8_t mailbox_full = 0x08, irq_write = 0x01, irq_read = 0x02;
}

template <EscPort E>
[[nodiscard]] SyncManager read_sm(E& esc, unsigned n) noexcept {
    std::array<std::byte, reg::sm_size> b{};
    esc.read(static_cast<std::uint16_t>(reg::sm_base + reg::sm_size * n), b);
    const auto u = [&](std::size_t i) { return std::to_integer<std::uint8_t>(b[i]); };
    return {.start = static_cast<std::uint16_t>(u(0) | (u(1) << 8)),
            .length = static_cast<std::uint16_t>(u(2) | (u(3) << 8)),
            .control = u(4),
            .status = u(5),
            .activate = u(6),
            .pdi_control = u(7)};
}

/// One FMMU's registers (0600h + 16n).
struct Fmmu {
    std::uint32_t logical_start = 0;
    std::uint16_t length = 0, physical_start = 0;
    std::uint8_t type = 0;  ///< bit 0 read, bit 1 write (seen from the master)
    bool active = false;
};

template <EscPort E>
[[nodiscard]] Fmmu read_fmmu(E& esc, unsigned n) noexcept {
    std::array<std::byte, reg::fmmu_size> b{};
    esc.read(static_cast<std::uint16_t>(reg::fmmu_base + reg::fmmu_size * n), b);
    const auto u = [&](std::size_t i) { return std::to_integer<std::uint32_t>(b[i]); };
    return {.logical_start = u(0) | (u(1) << 8) | (u(2) << 16) | (u(3) << 24),
            .length = static_cast<std::uint16_t>(u(4) | (u(5) << 8)),
            .physical_start = static_cast<std::uint16_t>(u(8) | (u(9) << 8)),
            .type = static_cast<std::uint8_t>(u(11)),
            .active = (u(12) & 1) != 0};
}

// ---- PHY management through the ESC (MII management, 0510h-0517h) -------------------------------
/// The PDI takes the MII management, then reads and writes PHY registers (IEEE 802.3 clause 22).
/// Blocking for a few microseconds per access (the busy bit).
template <EscPort E>
class Mii {
public:
    explicit Mii(E& esc) noexcept : esc_(esc) {}

    /// Takes the MII management for the PDI; false: the master holds it.
    [[nodiscard]] bool claim() noexcept {
        write8(esc_, reg::mii_pdi_access, 1);
        return (read8(esc_, reg::mii_pdi_access) & 1) != 0;
    }

    /// A PHY register; nullopt: busy, invalid command or read error (no PHY at that address).
    [[nodiscard]] std::optional<std::uint16_t> read(std::uint8_t phy, std::uint8_t reg) noexcept {
        if (!idle()) return std::nullopt;
        write8(esc_, reg::mii_phy_address, phy);
        write8(esc_, reg::mii_register, reg);
        write16(esc_, reg::mii_control, cmd_read);
        if (!idle()) return std::nullopt;
        if ((read16(esc_, reg::mii_control) & (command_error | read_error)) != 0) {
            write16(esc_, reg::mii_control, 0);  // command 00: clears the error bits
            return std::nullopt;
        }
        return read16(esc_, reg::mii_data);
    }

    bool write(std::uint8_t phy, std::uint8_t reg, std::uint16_t value) noexcept {
        if (!idle()) return false;
        write8(esc_, reg::mii_phy_address, phy);
        write8(esc_, reg::mii_register, reg);
        write16(esc_, reg::mii_data, value);
        write16(esc_, reg::mii_control, cmd_write);
        if (!idle()) return false;
        if ((read16(esc_, reg::mii_control) & command_error) != 0) {
            write16(esc_, reg::mii_control, 0);
            return false;
        }
        return true;
    }

private:
    static constexpr std::uint16_t busy = 1u << 15, command_error = 1u << 14, read_error = 1u << 13, cmd_read = 0x0100,
                                   cmd_write = 0x0200;
    bool idle() noexcept {
        for (int i = 0; i < 1000; ++i) {
            if ((read16(esc_, reg::mii_control) & busy) == 0) return true;
        }
        return false;
    }
    E& esc_;
};

}  // namespace rtos::ethercat
