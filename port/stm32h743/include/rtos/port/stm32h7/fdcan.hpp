// SPDX-License-Identifier: MIT
//
// STM32H743 FDCAN1 in classic CAN mode (RM0433 chapter FDCAN; registers, bits: SVD v2.4; message
// RAM at SRAMCAN_BASE and the clock calibration unit: ST's stm32h743xx.h, stm32h7xx_hal_fdcan.c).
//
//   using Can = Fdcan<{.kernel_hz = clocks.pll1_q_hz(), .cpu_hz = clocks.cpu_hz(), .bitrate = 500'000}>;
//   Can::init();                         // bit timing, accept all, start
//   Can::send({.id = 0x123, .length = 2, .data = {1, 2}});
//   CanFrame f;  while (Can::receive(f)) { ... }
//   CanStatus s = Can::status();
//
// The interface and the bit timing of the STM32H5/G4 driver (stm32/fdcan.hpp: CanConfig,
// can_bit_timing, CanFrame, CanStatus), but not its register code: the H7 has the full M_CAN,
// whose message RAM (10 KB, shared by FDCAN1/2) the driver lays out itself - no filter
// elements, RX FIFO 0 of 16 frames at byte 0, TX FIFO of 8 frames at byte 256, elements of
// 8 data bytes (16 bytes). The IR, RXF0S, TXFQS and TXBAR fields differ from the H5's in place.
// Every standard and extended frame goes into RX FIFO 0, remote frames are rejected. The clock
// calibration unit is bypassed (CCFG.BCC = 1, divider 1, as ST's HAL does with
// FDCAN_CLOCK_CALIBRATION_DISABLE). Kernel clock: pll1_q (RCC_D2CCIP1R.FDCANSEL = 01; the
// reset choice is HSE).
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/stm32/fdcan.hpp"
#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::can_bit_timing, stm32::CanBitTiming, stm32::CanConfig, stm32::CanFrame, stm32::CanStatus;

namespace fdcan_detail {
inline constexpr std::uintptr_t base = 0x4000'A000, ccu = 0x4000'A800, sram = 0x4000'AC00;
inline constexpr std::uint32_t rx_fifo0 = 0x000, rx_fifo0_elements = 16;
inline constexpr std::uint32_t tx_fifo = 0x100, tx_fifo_elements = 8;
inline constexpr std::uint32_t element_bytes = 16, used_bytes = tx_fifo + tx_fifo_elements * element_bytes;
inline constexpr std::uint32_t element_xtd = 1u << 30;
inline constexpr std::uint32_t cccr_init = 1u << 0, cccr_cce = 1u << 1, cccr_mon = 1u << 5, cccr_dar = 1u << 6,
                               cccr_test = 1u << 7;
inline constexpr std::uint32_t test_lbck = 1u << 4;
inline constexpr std::uint32_t gfc_rrfe = 1u << 0, gfc_rrfs = 1u << 1;  // ANFS/ANFE = 00: into RX FIFO 0
inline constexpr std::uint32_t rxf0s_rf0l = 1u << 25, txfqs_tfqf = 1u << 21;
inline constexpr std::uint32_t psr_ep = 1u << 5, psr_bo = 1u << 7;
inline constexpr std::uint32_t ir_rf0l = 1u << 3;
inline constexpr std::uint32_t ccfg_bcc = 1u << 6;  // CDIV (19:16) = 0: divide by 1
/// RXF0C: start (byte offset, bits 15:2) | size; TXBC: start | TFQS, TFQM = 0 (FIFO)
inline constexpr std::uint32_t rxf0c = rx_fifo0 | (rx_fifo0_elements << 16);
inline constexpr std::uint32_t txbc = tx_fifo | (tx_fifo_elements << 24);
/// RCC_D2CCIP1R.FDCANSEL (29:28) = 01: pll1_q (ST's LL_RCC_FDCAN_CLKSOURCE_PLL1Q).
inline constexpr stm32::rcc::ClockSelection kernel_clock{{rcc::base + 0x050, 28, 2}, 1};
}  // namespace fdcan_detail

template <CanConfig Config>
class Fdcan {
    static constexpr CanBitTiming timing = can_bit_timing(Config.kernel_hz, Config.bitrate, Config.sample_permille);
    static_assert(timing.valid, "no bit timing: the kernel clock does not divide into the bit rate (8..25 quanta)");
    static_assert(timing.sample_permille() + 50 >= Config.sample_permille &&
                      timing.sample_permille() <= Config.sample_permille + 50,
                  "sample point more than 5 % away from the requested one");
    static constexpr std::uint32_t cycles_per_ms = Config.cpu_hz / 1000;

public:
    static constexpr std::uint8_t irq0 = 19, irq1 = 21;  // FDCAN1_IT0, FDCAN1_IT1
    static constexpr CanBitTiming bit_timing = timing;

    /// Clock, bit timing, message RAM, filters (accept all into FIFO 0); the controller joins the bus.
    static bool init() noexcept {
        using namespace fdcan_detail;
        stm32::rcc::select(kernel_clock);
        stm32::rcc::enable(rcc::gate::fdcan);
        CCCR::write(cccr_init);
        if (!wait([] { return (CCCR::read() & cccr_init) != 0; })) {
            return false;
        }
        CCCR::write(cccr_init | cccr_cce);
        CCFG::write(ccfg_bcc);
        NBTP::write(timing.nbtp());
        CCCR::write(cccr_init | cccr_cce | (Config.auto_retransmission ? 0u : cccr_dar) |
                    (Config.internal_loopback ? cccr_test | cccr_mon : 0u));
        if constexpr (Config.internal_loopback) {
            TEST::write(test_lbck);
        }
        GFC::write(gfc_rrfs | gfc_rrfe);
        SIDFC::write(0);
        XIDFC::write(0);
        RXF0C::write(rxf0c);
        RXESC::write(0);  // 8-byte data fields
        TXBC::write(txbc);
        TXESC::write(0);
        for (std::uintptr_t a = 0; a < used_bytes; a += 4) {
            *reinterpret_cast<volatile std::uint32_t*>(sram + a) = 0;
        }
        CCCR::write(CCCR::read() & ~(cccr_init | cccr_cce));
        return wait([] { return (CCCR::read() & cccr_init) == 0; });
    }

    /// Queue a frame; false if the TX FIFO is full.
    static bool send(const CanFrame& f) noexcept {
        using namespace fdcan_detail;
        const std::uint32_t txfqs = TXFQS::read();
        if ((txfqs & txfqs_tfqf) != 0) {
            return false;
        }
        const std::uint32_t index = (txfqs >> 16) & 0x1Fu;  // TFQPI
        auto* e = reinterpret_cast<volatile std::uint32_t*>(sram + tx_fifo + index * element_bytes);
        e[0] = f.extended ? (f.id & 0x1FFF'FFFFu) | element_xtd : (f.id & 0x7FFu) << 18;
        e[1] = static_cast<std::uint32_t>(f.length > 8 ? 8 : f.length) << 16;
        e[2] = word(f, 0);
        e[3] = word(f, 4);
        TXBAR::write(1u << index);
        return true;
    }

    /// Take the oldest received frame; false if none waits.
    static bool receive(CanFrame& f) noexcept {
        using namespace fdcan_detail;
        const std::uint32_t rxf0s = RXF0S::read();
        if ((rxf0s & 0x7Fu) == 0) {
            return false;
        }
        const std::uint32_t index = (rxf0s >> 8) & 0x3Fu;  // F0G
        const auto* e = reinterpret_cast<const volatile std::uint32_t*>(sram + rx_fifo0 + index * element_bytes);
        const std::uint32_t w0 = e[0], w1 = e[1], d0 = e[2], d1 = e[3];
        f.extended = (w0 & element_xtd) != 0;
        f.id = f.extended ? (w0 & 0x1FFF'FFFFu) : ((w0 >> 18) & 0x7FFu);
        const std::uint32_t dlc = (w1 >> 16) & 0xFu;
        f.length = static_cast<std::uint8_t>(dlc > 8 ? 8 : dlc);
        for (std::size_t i = 0; i < 4; ++i) {
            f.data[i] = static_cast<std::uint8_t>(d0 >> (8 * i));
            f.data[4 + i] = static_cast<std::uint8_t>(d1 >> (8 * i));
        }
        RXF0A::write(index);
        return true;
    }

    /// Frames lost because RX FIFO 0 was full (since the last call).
    [[nodiscard]] static bool rx_overflowed() noexcept {
        const bool lost = (RXF0S::read() & fdcan_detail::rxf0s_rf0l) != 0;
        if (lost) {
            IR::write(fdcan_detail::ir_rf0l);
        }
        return lost;
    }

    [[nodiscard]] static CanStatus status() noexcept {
        const std::uint32_t ecr = ECR::read();
        const std::uint32_t psr = PSR::read();  // reading clears LEC
        return {static_cast<std::uint8_t>(ecr & 0xFFu), static_cast<std::uint8_t>((ecr >> 8) & 0x7Fu),
                (psr & fdcan_detail::psr_ep) != 0, (psr & fdcan_detail::psr_bo) != 0,
                static_cast<std::uint8_t>(psr & 7u)};
    }

    /// Leave bus-off: back to normal operation after 128 x 11 recessive bits.
    static void recover() noexcept { CCCR::write(CCCR::read() & ~fdcan_detail::cccr_init); }

private:
    static constexpr std::uint32_t word(const CanFrame& f, std::size_t at) noexcept {
        return f.data[at] | (std::uint32_t{f.data[at + 1]} << 8) | (std::uint32_t{f.data[at + 2]} << 16) |
               (std::uint32_t{f.data[at + 3]} << 24);
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

    static constexpr std::uintptr_t b = fdcan_detail::base;
    using TEST = Reg<b + 0x10>;
    using CCCR = Reg<b + 0x18>;
    using NBTP = Reg<b + 0x1C>;
    using ECR = Reg<b + 0x40>;
    using PSR = Reg<b + 0x44>;
    using IR = Reg<b + 0x50>;
    using GFC = Reg<b + 0x80>;
    using SIDFC = Reg<b + 0x84>;
    using XIDFC = Reg<b + 0x88>;
    using RXF0C = Reg<b + 0xA0>;
    using RXF0S = Reg<b + 0xA4>;
    using RXF0A = Reg<b + 0xA8>;
    using RXESC = Reg<b + 0xBC>;
    using TXBC = Reg<b + 0xC0>;
    using TXFQS = Reg<b + 0xC4>;
    using TXESC = Reg<b + 0xC8>;
    using TXBAR = Reg<b + 0xD0>;
    using CCFG = Reg<fdcan_detail::ccu + 0x04>;
};

}  // namespace rtos::port::stm32h7
