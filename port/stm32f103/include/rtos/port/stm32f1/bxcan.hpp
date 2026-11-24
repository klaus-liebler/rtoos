// SPDX-License-Identifier: MIT
//
// STM32F103 bxCAN (RM0008 rev 21 chapter 24) in classic CAN mode. Same frame and status types and
// the same calls as the FDCAN driver of the newer chips (port/stm32/fdcan.hpp), so code above it
// does not care which controller it has:
//
//   using Can = BxCan<{.kernel_hz = clocks.pclk1_hz(), .cpu_hz = clocks.sysclk_hz(), .bitrate = 125'000}>;
//   Can::init();                          // bit timing, accept all, join the bus
//   Can::send({.id = 0x123, .extended = true, .length = 2, .data = {1, 2}});
//   CanFrame f;  while (Can::receive(f)) { ... }
//   CanStatus s = Can::status();           // error counters, error passive, bus off, last error
//
// The configuration is a template argument: the bit timing is computed at compile time - the
// most time quanta per bit (25 .. 8) with which PCLK1 divides exactly into the bit rate and the
// segments fit the bxCAN (BRP 1..1024, TS1 1..16, TS2 1..8), the sample point nearest to
// sample_permille (CiA: 875), SJW = min(TS2, 4) - and checked. Every standard and extended
// frame goes into FIFO 0 (3 deep, filter bank 0 as a 32-bit mask that matches everything);
// frames leave through the three mailboxes in the order they were queued (TXFP).
//
// Bus off: with auto_bus_off_recovery (ABOM) the controller rejoins by itself after 128 x 11
// recessive bits - a node behind a noisy cable must not stay silent until someone resets it.
//
// The bxCAN and the USB device share 512 bytes of SRAM on the F103: they cannot run together.
// Erratum ES096 2.11.1 "time-triggered communication mode not supported": TTCM stays 0.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32/fdcan.hpp"
#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/rcc.hpp"

namespace rtos::port::stm32f1 {

using stm32::CanFrame, stm32::CanStatus, stm32::CanBitTiming;

struct BxCanConfig {
    std::uint32_t kernel_hz;               ///< PCLK1
    std::uint32_t cpu_hz;                  ///< core clock: timeouts
    std::uint32_t bitrate = 500'000;
    std::uint32_t sample_permille = 875;
    bool auto_retransmission = true;
    bool auto_bus_off_recovery = true;     ///< ABOM
    bool internal_loopback = false;        ///< test mode (LBKM + SILM): frames come back, the bus stays recessive
};

/// Bit timing for the bxCAN's segment ranges (RM0008 24.7.7 CAN_BTR).
[[nodiscard]] constexpr CanBitTiming bxcan_bit_timing(std::uint32_t kernel_hz, std::uint32_t bitrate,
                                                      std::uint32_t sample_permille) noexcept {
    for (std::uint32_t n = 25; n >= 8; --n) {
        const std::uint64_t per_bit = static_cast<std::uint64_t>(bitrate) * n;
        if (kernel_hz % per_bit != 0) {
            continue;
        }
        const auto prescaler = static_cast<std::uint32_t>(kernel_hz / per_bit);
        if (prescaler < 1 || prescaler > 1024) {
            continue;
        }
        const std::uint32_t sample = (n * sample_permille + 500) / 1000;  // quanta up to the sample point
        const std::uint32_t tseg1 = sample - 1;
        const std::uint32_t tseg2 = n - sample;
        if (tseg1 < 1 || tseg1 > 16 || tseg2 < 1 || tseg2 > 8) {
            continue;
        }
        return {prescaler, tseg1, tseg2, tseg2 < 4 ? tseg2 : 4, true};
    }
    return {};
}

/// CAN_BTR of a bit timing (without the test mode bits).
[[nodiscard]] constexpr std::uint32_t bxcan_btr(const CanBitTiming& t) noexcept {
    return ((t.sjw - 1) << 24) | ((t.tseg2 - 1) << 20) | ((t.tseg1 - 1) << 16) | (t.prescaler - 1);
}

namespace bxcan_detail {
inline constexpr std::uint32_t mcr_inrq = 1u << 0, mcr_sleep = 1u << 1, mcr_txfp = 1u << 2, mcr_nart = 1u << 4,
                               mcr_abom = 1u << 6;
inline constexpr std::uint32_t msr_inak = 1u << 0, msr_slak = 1u << 1;
inline constexpr std::uint32_t tsr_tme_pos = 26, tsr_code_pos = 24;
inline constexpr std::uint32_t rfr_fmp_msk = 3u, rfr_fovr = 1u << 4, rfr_rfom = 1u << 5;
inline constexpr std::uint32_t ier_fmpie0 = 1u << 1;
inline constexpr std::uint32_t esr_epvf = 1u << 1, esr_boff = 1u << 2;
inline constexpr std::uint32_t btr_lbkm = 1u << 30, btr_silm = 1u << 31;
inline constexpr std::uint32_t tir_txrq = 1u << 0, ir_rtr = 1u << 1, ir_ide = 1u << 2;
inline constexpr std::uint32_t fmr_finit = 1u << 0;
}  // namespace bxcan_detail

template <BxCanConfig Config>
class BxCan {
    static constexpr CanBitTiming timing = bxcan_bit_timing(Config.kernel_hz, Config.bitrate, Config.sample_permille);
    static_assert(timing.valid, "no bit timing: PCLK1 does not divide into the bit rate (8..25 quanta, bxCAN segments)");
    static_assert(timing.sample_permille() + 50 >= Config.sample_permille &&
                      timing.sample_permille() <= Config.sample_permille + 50,
                  "sample point more than 5 % away from the requested one");
    static constexpr std::uint32_t cycles_per_ms = Config.cpu_hz / 1000;
    static constexpr std::uintptr_t base = periph_base + 0x0000'6400;  // CAN1

public:
    static constexpr std::uint8_t irq_tx = static_cast<std::uint8_t>(Irq::USB_HP_CAN1_TX),
                                  irq_rx0 = static_cast<std::uint8_t>(Irq::USB_LP_CAN1_RX0),
                                  irq_sce = static_cast<std::uint8_t>(Irq::CAN1_SCE);
    static constexpr CanBitTiming bit_timing = timing;
    static constexpr std::uint32_t btr = bxcan_btr(timing) |
                                         (Config.internal_loopback ? bxcan_detail::btr_lbkm | bxcan_detail::btr_silm : 0u);

    /// Clock, bit timing, filter (accept all into FIFO 0); the controller joins the bus (after
    /// 11 recessive bits). False if it does not leave sleep/initialization mode within 10 ms.
    static bool init() noexcept {
        using namespace bxcan_detail;
        rcc::enable(rcc::gate::can1);
        MCR::write(mcr_inrq);  // out of sleep, into initialization
        if (!wait([] { return (MSR::read() & (msr_inak | msr_slak)) == msr_inak; })) {
            return false;
        }
        MCR::write(mcr_inrq | mcr_txfp | (Config.auto_retransmission ? 0u : mcr_nart) |
                   (Config.auto_bus_off_recovery ? mcr_abom : 0u));  // TTCM = 0 (ES096 2.11.1)
        BTR::write(btr);
        FMR::write(FMR::read() | fmr_finit);
        FA1R::write(FA1R::read() & ~1u);
        FS1R::write(FS1R::read() | 1u);   // bank 0: one 32-bit filter
        FM1R::write(FM1R::read() & ~1u);  // identifier mask mode
        FFA1R::write(FFA1R::read() & ~1u);  // -> FIFO 0
        F0R1::write(0);  // identifier
        F0R2::write(0);  // mask: no bit must match
        FA1R::write(FA1R::read() | 1u);
        FMR::write(FMR::read() & ~fmr_finit);
        MCR::write(MCR::read() & ~mcr_inrq);
        return wait([] { return (MSR::read() & msr_inak) == 0; });
    }

    /// Queue a frame in a free mailbox; false if all three are busy.
    static bool send(const CanFrame& f) noexcept {
        using namespace bxcan_detail;
        const std::uint32_t tsr = TSR::read();
        if (((tsr >> tsr_tme_pos) & 7u) == 0) {
            return false;
        }
        const std::uint32_t box = (tsr >> tsr_code_pos) & 3u;
        const std::uintptr_t mb = base + 0x180 + 0x10 * box;
        reg(mb + 0x4) = f.length > 8 ? 8u : f.length;  // TDTxR: DLC
        reg(mb + 0x8) = word(f.data, 0);
        reg(mb + 0xC) = word(f.data, 4);
        reg(mb + 0x0) = (f.extended ? ((f.id & 0x1FFF'FFFFu) << 3) | ir_ide : (f.id & 0x7FFu) << 21) | tir_txrq;
        return true;
    }

    /// Take the oldest received frame; false if none waits. (Remote frames come through with
    /// length 0: the filter does not reject them - RTR is not part of CanFrame.)
    static bool receive(CanFrame& f) noexcept {
        using namespace bxcan_detail;
        if ((RF0R::read() & rfr_fmp_msk) == 0) {
            return false;
        }
        const std::uint32_t rir = RI0R::read();
        f.extended = (rir & ir_ide) != 0;
        f.id = f.extended ? (rir >> 3) & 0x1FFF'FFFFu : (rir >> 21) & 0x7FFu;
        const std::uint32_t dlc = RDT0R::read() & 0xFu;
        f.length = static_cast<std::uint8_t>((rir & ir_rtr) != 0 ? 0 : (dlc > 8 ? 8 : dlc));
        const std::uint32_t d0 = RDL0R::read(), d1 = RDH0R::read();
        for (std::size_t i = 0; i < 4; ++i) {
            f.data[i] = static_cast<std::uint8_t>(d0 >> (8 * i));
            f.data[4 + i] = static_cast<std::uint8_t>(d1 >> (8 * i));
        }
        RF0R::write(rfr_rfom);  // release the FIFO slot
        return true;
    }

    /// Frames lost because FIFO 0 was full (since the last call).
    [[nodiscard]] static bool rx_overflowed() noexcept {
        const bool lost = (RF0R::read() & bxcan_detail::rfr_fovr) != 0;
        if (lost) {
            RF0R::write(bxcan_detail::rfr_fovr);
        }
        return lost;
    }

    /// Interrupt on a received frame (USB_LP_CAN1_RX0): the caller sets the NVIC priority and
    /// enables irq_rx0; the handler calls receive() until it returns false.
    static void enable_rx_irq() noexcept { IER::write(IER::read() | bxcan_detail::ier_fmpie0); }

    [[nodiscard]] static CanStatus status() noexcept {
        const std::uint32_t esr = ESR::read();
        return {static_cast<std::uint8_t>((esr >> 16) & 0xFFu), static_cast<std::uint8_t>((esr >> 24) & 0xFFu),
                (esr & bxcan_detail::esr_epvf) != 0, (esr & bxcan_detail::esr_boff) != 0,
                static_cast<std::uint8_t>((esr >> 4) & 7u)};
    }

    /// No frame waiting in a mailbox.
    [[nodiscard]] static bool tx_idle() noexcept { return ((TSR::read() >> bxcan_detail::tsr_tme_pos) & 7u) == 7u; }

    /// Leave bus-off by hand (without auto_bus_off_recovery): initialization mode and back -
    /// the controller rejoins after 128 x 11 recessive bits.
    static void recover() noexcept {
        MCR::write(MCR::read() | bxcan_detail::mcr_inrq);
        (void)wait([] { return (MSR::read() & bxcan_detail::msr_inak) != 0; });
        MCR::write(MCR::read() & ~bxcan_detail::mcr_inrq);
    }

private:
    static volatile std::uint32_t& reg(std::uintptr_t a) noexcept { return *reinterpret_cast<volatile std::uint32_t*>(a); }
    static constexpr std::uint32_t word(const std::array<std::uint8_t, 8>& d, std::size_t at) noexcept {
        return d[at] | (std::uint32_t{d[at + 1]} << 8) | (std::uint32_t{d[at + 2]} << 16) |
               (std::uint32_t{d[at + 3]} << 24);
    }

    template <class Done>
    static bool wait(Done done) noexcept {
        const std::uint32_t start = cortex_m::CycleCounter::now();
        while (!done()) {
            if (cortex_m::CycleCounter::now() - start > 10 * cycles_per_ms) {
                return false;
            }
        }
        return true;
    }

    using MCR = cortex_m::Reg<base + 0x000>;
    using MSR = cortex_m::Reg<base + 0x004>;
    using TSR = cortex_m::Reg<base + 0x008>;
    using RF0R = cortex_m::Reg<base + 0x00C>;
    using IER = cortex_m::Reg<base + 0x014>;
    using ESR = cortex_m::Reg<base + 0x018>;
    using BTR = cortex_m::Reg<base + 0x01C>;
    using RI0R = cortex_m::Reg<base + 0x1B0>;
    using RDT0R = cortex_m::Reg<base + 0x1B4>;
    using RDL0R = cortex_m::Reg<base + 0x1B8>;
    using RDH0R = cortex_m::Reg<base + 0x1BC>;
    using FMR = cortex_m::Reg<base + 0x200>;
    using FM1R = cortex_m::Reg<base + 0x204>;
    using FS1R = cortex_m::Reg<base + 0x20C>;
    using FFA1R = cortex_m::Reg<base + 0x214>;
    using FA1R = cortex_m::Reg<base + 0x21C>;
    using F0R1 = cortex_m::Reg<base + 0x240>;
    using F0R2 = cortex_m::Reg<base + 0x244>;
};

}  // namespace rtos::port::stm32f1
