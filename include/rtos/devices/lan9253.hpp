// SPDX-License-Identifier: MIT
//
// Microchip LAN9253 EtherCAT slave controller with two internal PHYs (data sheet DS00003421B), on
// SPI in "EtherCAT Direct Mapped" mode (PDI 0x82, EE_EMUL straps 100, 9.3): the EtherCAT core
// registers and the process RAM at their own addresses 0000h-2FFFh, the system CSRs at 3000h + offset.
// Satisfies rtos::ethercat::EscPort (read/write any ESC address).
//
//   rtos::devices::Lan9253<GpioSpi> esc;
//   if (esc.probe() == Lan9253Mode::direct_mapped) ... esc.ready() ... slave.poll(esc.ready());
//
// The bus is a template argument (Lan9253Bus): select, one byte full duplex (SPI mode 0, READ 03h /
// WRITE 02h), and a delay in nanoseconds. Without WAIT_ACK the host keeps the worst-case timing of
// 9.3.2: 370 ns from the address to each data byte of a core READ, 250 ns per byte of a core WRITE and
// 250 ns + 80 ns per byte after it before the next command. System CSRs need no waits.
//
// The LAN9252-compatible map (PDI 0x80, straps 000: core registers only through ECAT_CSR_CMD, process
// RAM through FIFOs) is recognised by probe() and reported, not driven.
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "rtos/ethercat/esc.hpp"

namespace rtos::devices {

template <class B>
concept Lan9253Bus = requires(bool on, std::uint8_t out, std::uint32_t ns) {
    { B::select(on) } noexcept;
    { B::transfer(out) } noexcept -> std::same_as<std::uint8_t>;
    { B::wait_ns(ns) } noexcept;
};

enum class Lan9253Mode : std::uint8_t { none, compatible, direct_mapped };

namespace lan9253 {
/// System CSR offsets (table 5-1); in Direct Mapped mode at 3000h + offset.
inline constexpr std::uint16_t id_rev = 0x050, irq_cfg = 0x054, int_sts = 0x058, byte_test = 0x064, hw_cfg = 0x074,
                               int_en = 0x1F8;
inline constexpr std::uint16_t direct_system_base = 0x3000;
inline constexpr std::uint32_t byte_test_value = 0x8765'4321, hw_cfg_ready = 1u << 27;
inline constexpr std::uint32_t irq_enable = 1u << 8;  ///< IRQ_CFG: output on; open drain, active low (7.3.1)
inline constexpr std::uint32_t ecat_int = 1u << 0;    ///< INT_EN / INT_STS: EtherCAT interrupt (AL event request)
inline constexpr std::uint8_t cmd_read = 0x03, cmd_write = 0x02;
inline constexpr std::uint16_t increment = 0x4000;  ///< address bits 15:14 = 01: auto-increment (9.3.2)

/// PHY registers of the internal PHYs (10.2.18): A at address 0 or 2 (device mode), B at 1 (10.1.1).
inline constexpr std::uint8_t phy_basic_control = 0, phy_basic_status = 1, phy_id_msb = 2, phy_an_advertisement = 4,
                              phy_mode_control_status = 17, phy_special_control_status_ind = 27;
inline constexpr std::uint16_t bmsr_link = 1u << 2, bmsr_an_complete = 1u << 5, mode_energy_on = 1u << 1,
                               scsi_auto_mdix = 0xC000;
}  // namespace lan9253

/// A PHY's state for a status page.
struct Lan9253Phy {
    std::uint8_t address = 0;
    bool present = false, link = false, autonegotiation_complete = false, energy = false, auto_mdix = false;
    std::uint16_t basic_control = 0, basic_status = 0;
};

template <Lan9253Bus Bus>
class Lan9253 {
public:
    /// Finds the register map by BYTE_TEST (87654321h at 064h or 3064h); the first reads after the
    /// reset may not answer - the caller repeats with a short sleep.
    [[nodiscard]] Lan9253Mode probe() noexcept {
        if (system_read(lan9253::direct_system_base + lan9253::byte_test) == lan9253::byte_test_value) {
            mode_ = Lan9253Mode::direct_mapped;
        } else if (system_read(lan9253::byte_test) == lan9253::byte_test_value) {
            mode_ = Lan9253Mode::compatible;
        } else {
            mode_ = Lan9253Mode::none;
        }
        return mode_;
    }
    [[nodiscard]] Lan9253Mode mode() const noexcept { return mode_; }

    /// A system CSR (Direct Mapped mode).
    [[nodiscard]] std::uint32_t csr(std::uint16_t offset) noexcept {
        return system_read(static_cast<std::uint16_t>(lan9253::direct_system_base + offset));
    }
    void set_csr(std::uint16_t offset, std::uint32_t value) noexcept {
        system_write(static_cast<std::uint16_t>(lan9253::direct_system_base + offset), value);
    }

    /// HW_CFG READY: the configuration area is loaded (before it only the EEPROM emulation).
    [[nodiscard]] bool ready() noexcept { return (csr(lan9253::hw_cfg) & lan9253::hw_cfg_ready) != 0; }

    /// The IRQ pin for AL events in `mask` (AL event mask 0204h): open drain, active low.
    void enable_irq(std::uint32_t mask) noexcept {
        rtos::ethercat::write32(*this, rtos::ethercat::reg::al_event_mask, mask);
        set_csr(lan9253::int_en, lan9253::ecat_int);
        set_csr(lan9253::irq_cfg, lan9253::irq_enable);
    }

    // ---- EscPort: core registers and process RAM ------------------------------------------------
    void read(std::uint16_t address, std::span<std::byte> out) noexcept {
        Bus::select(true);
        (void)Bus::transfer(lan9253::cmd_read);
        (void)Bus::transfer(static_cast<std::uint8_t>((address | lan9253::increment) >> 8));
        (void)Bus::transfer(static_cast<std::uint8_t>(address));
        for (std::size_t i = 0; i < out.size(); ++i) {
            Bus::wait_ns(400);  // 370 ns until the core has delivered the byte
            // SI high on the last byte: no prefetch past it (9.2.5.1) - a read past a mailbox
            // buffer's end would count as reading it
            out[i] = std::byte{Bus::transfer(i + 1 == out.size() ? 0xFF : 0x00)};
        }
        Bus::select(false);
    }

    void write(std::uint16_t address, std::span<const std::byte> in) noexcept {
        Bus::select(true);
        (void)Bus::transfer(lan9253::cmd_write);
        (void)Bus::transfer(static_cast<std::uint8_t>((address | lan9253::increment) >> 8));
        (void)Bus::transfer(static_cast<std::uint8_t>(address));
        for (const std::byte b : in) {
            (void)Bus::transfer(std::to_integer<std::uint8_t>(b));
            Bus::wait_ns(250);  // the core takes 250 ns per byte
        }
        Bus::select(false);
        Bus::wait_ns(static_cast<std::uint32_t>(250 + 80 * in.size()));  // no command while the write is pending
    }

    // ---- the internal PHYs ----------------------------------------------------------------------
    /// Takes the MII management and sets every PHY found at 0..3 to autonegotiation advertising only
    /// 100BASE-TX full duplex (EtherCAT) with Auto-MDIX - whatever the straps say (100FD_B fixes PHY B
    /// at 100 full duplex without Auto-MDIX: a negotiating PC adapter then gets no stable link).
    /// Returns the PHYs found.
    std::size_t configure_phys() noexcept {
        rtos::ethercat::Mii mii(*this);
        phy_count_ = 0;
        if (!mii.claim()) return 0;
        for (std::uint8_t a = 0; a < 4 && phy_count_ < phys_.size(); ++a) {
            const auto id = mii.read(a, lan9253::phy_id_msb);
            if (!id || *id == 0xFFFF) continue;
            (void)mii.write(a, lan9253::phy_an_advertisement, 0x0101);             // 100BASE-TX FD, selector 802.3
            (void)mii.write(a, lan9253::phy_special_control_status_ind, lan9253::scsi_auto_mdix);
            (void)mii.write(a, lan9253::phy_basic_control, 0x3300);                // 100 Mbit/s, AN on + restart, FD
            phys_[phy_count_++] = a;
        }
        return phy_count_;
    }

    /// The PHYs configured, their state now (basic status read twice: the link bit latches low).
    [[nodiscard]] std::size_t phy_states(std::span<Lan9253Phy> out) noexcept {
        rtos::ethercat::Mii mii(*this);
        std::size_t n = 0;
        for (std::size_t i = 0; i < phy_count_ && n < out.size(); ++i) {
            Lan9253Phy p{.address = phys_[i]};
            (void)mii.read(p.address, lan9253::phy_basic_status);
            const auto bmsr = mii.read(p.address, lan9253::phy_basic_status);
            const auto bmcr = mii.read(p.address, lan9253::phy_basic_control);
            const auto mode = mii.read(p.address, lan9253::phy_mode_control_status);
            const auto scsi = mii.read(p.address, lan9253::phy_special_control_status_ind);
            p.present = bmsr && bmcr && mode && scsi;
            if (p.present) {
                p.basic_status = *bmsr, p.basic_control = *bmcr;
                p.link = (*bmsr & lan9253::bmsr_link) != 0;
                p.autonegotiation_complete = (*bmsr & lan9253::bmsr_an_complete) != 0;
                p.energy = (*mode & lan9253::mode_energy_on) != 0;
                p.auto_mdix = (*scsi & 0x8000) != 0;
            }
            out[n++] = p;
        }
        return n;
    }

private:
    /// A system CSR, 4 bytes LSB first (no waits; SI high on the last byte: no prefetch).
    static std::uint32_t system_read(std::uint16_t address) noexcept {
        Bus::select(true);
        (void)Bus::transfer(lan9253::cmd_read);
        (void)Bus::transfer(static_cast<std::uint8_t>(address >> 8));
        (void)Bus::transfer(static_cast<std::uint8_t>(address));
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= std::uint32_t{Bus::transfer(i == 3 ? 0xFF : 0x00)} << (8 * i);
        Bus::select(false);
        return v;
    }
    static void system_write(std::uint16_t address, std::uint32_t value) noexcept {
        Bus::select(true);
        (void)Bus::transfer(lan9253::cmd_write);
        (void)Bus::transfer(static_cast<std::uint8_t>(address >> 8));
        (void)Bus::transfer(static_cast<std::uint8_t>(address));
        for (int i = 0; i < 4; ++i) (void)Bus::transfer(static_cast<std::uint8_t>(value >> (8 * i)));
        Bus::select(false);
    }

    Lan9253Mode mode_ = Lan9253Mode::none;
    std::array<std::uint8_t, 4> phys_{};
    std::size_t phy_count_ = 0;
};

}  // namespace rtos::devices
