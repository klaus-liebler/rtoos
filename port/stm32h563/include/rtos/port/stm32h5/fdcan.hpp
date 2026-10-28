// SPDX-License-Identifier: MIT
//
// STM32H563 FDCAN1 in classic CAN mode (RM0481 chapter FDCAN; registers, bits, IRQs: SVD
// v1.8; message RAM layout: ST's stm32h5xx_hal_fdcan.c, SRAMCAN_*).
//
//   using Can = Fdcan<{.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz, .bitrate = 500'000}>;
//   Can::init();                         // bit timing, accept all, start
//   Can::send({.id = 0x123, .length = 2, .data = {1, 2}});
//   CanFrame f;  while (Can::receive(f)) { ... }
//   CanStatus s = Can::status();          // error counters, error passive, bus off, last error
//
// The configuration is a template argument: the bit timing is computed at compile time - the
// most time quanta per bit (25 .. 8) with which the kernel clock divides exactly into the bit
// rate, the sample point nearest to sample_permille (CiA: 875), SJW = TSEG2 - and checked
// against the register ranges. Every standard and extended frame goes into RX FIFO 0 (3
// frames deep), remote frames are rejected; frames leave through the 3-deep TX FIFO.
//
// Kernel clock: RCC_CCIPR5.FDCAN12SEL = pll1_q (init() selects it; the reset choice HSE does
// not exist on the FCU).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

struct CanConfig {
    std::uint32_t kernel_hz;             ///< FDCAN kernel clock (pll1_q)
    std::uint32_t cpu_hz;                ///< core clock: timeouts
    std::uint32_t bitrate = 500'000;
    std::uint32_t sample_permille = 875;
    bool auto_retransmission = true;
    bool internal_loopback = false;      ///< test mode: frames come back, the bus stays recessive
};

struct CanBitTiming {
    std::uint32_t prescaler = 0, tseg1 = 0, tseg2 = 0, sjw = 0;  // in time quanta (not register values)
    bool valid = false;

    [[nodiscard]] constexpr std::uint32_t quanta() const noexcept { return 1 + tseg1 + tseg2; }
    [[nodiscard]] constexpr std::uint32_t sample_permille() const noexcept { return 1000 * (1 + tseg1) / quanta(); }
    [[nodiscard]] constexpr std::uint32_t nbtp() const noexcept {
        return ((sjw - 1) << 25) | ((prescaler - 1) << 16) | ((tseg1 - 1) << 8) | (tseg2 - 1);
    }
};

[[nodiscard]] constexpr CanBitTiming can_bit_timing(std::uint32_t kernel_hz, std::uint32_t bitrate,
                                                    std::uint32_t sample_permille) noexcept {
    for (std::uint32_t n = 25; n >= 8; --n) {
        const std::uint64_t per_bit = static_cast<std::uint64_t>(bitrate) * n;
        if (kernel_hz % per_bit != 0) {
            continue;
        }
        const std::uint32_t prescaler = static_cast<std::uint32_t>(kernel_hz / per_bit);
        if (prescaler < 1 || prescaler > 512) {
            continue;
        }
        const std::uint32_t sample = (n * sample_permille + 500) / 1000;  // quanta up to the sample point
        const std::uint32_t tseg1 = sample - 1;
        const std::uint32_t tseg2 = n - sample;
        if (tseg1 < 2 || tseg1 > 256 || tseg2 < 1 || tseg2 > 128) {
            continue;
        }
        return {prescaler, tseg1, tseg2, tseg2, true};
    }
    return {};
}

struct CanFrame {
    std::uint32_t id = 0;         ///< 11 bit, or 29 bit with extended
    bool extended = false;
    std::uint8_t length = 0;      ///< 0..8
    std::array<std::uint8_t, 8> data{};
};

struct CanStatus {
    std::uint8_t tx_errors = 0;   ///< TEC
    std::uint8_t rx_errors = 0;   ///< REC
    bool error_passive = false;
    bool bus_off = false;
    std::uint8_t last_error = 0;  ///< PSR.LEC: 0 none, 1 stuff, 2 form, 3 ack, 4 bit1, 5 bit0, 6 CRC, 7 no change
};

template <CanConfig Config>
class Fdcan {
    static constexpr CanBitTiming timing = can_bit_timing(Config.kernel_hz, Config.bitrate, Config.sample_permille);
    static_assert(timing.valid, "no bit timing: the kernel clock does not divide into the bit rate (8..25 quanta)");
    static_assert(timing.sample_permille() + 50 >= Config.sample_permille &&
                      timing.sample_permille() <= Config.sample_permille + 50,
                  "sample point more than 5 % away from the requested one");
    static constexpr std::uint32_t cycles_per_ms = Config.cpu_hz / 1000;

public:
    static constexpr std::uint8_t irq0 = 39, irq1 = 40;
    static constexpr CanBitTiming bit_timing = timing;

    /// Clock, bit timing, filters (accept all into FIFO 0); the controller joins the bus.
    static bool init() noexcept {
        rcc::enable(rcc::gate::fdcan12);
        rcc::select(rcc::kernel_clock::fdcan12, rcc::kernel_clock::fdcan_pll1_q);
        CCCR::write(cccr_init);
        if (!wait([] { return (CCCR::read() & cccr_init) != 0; })) {
            return false;
        }
        CCCR::write(cccr_init | cccr_cce);
        CKDIV::write(0);
        NBTP::write(timing.nbtp());
        // test mode for the internal loopback: TEST is writable once CCCR.TEST is set
        CCCR::write(cccr_init | cccr_cce | (Config.auto_retransmission ? 0u : cccr_dar) |
                    (Config.internal_loopback ? cccr_test | cccr_mon : 0u));
        if constexpr (Config.internal_loopback) {
            TEST::write(test_lbck);
        }
        RXGFC::write(rxgfc_rrfs | rxgfc_rrfe);  // no filter elements; non-matching -> FIFO 0, no remote
        TXBC::write(0);                         // TX FIFO mode
        for (std::uintptr_t a = 0; a < sram_bytes; a += 4) {
            *reinterpret_cast<volatile std::uint32_t*>(sram + a) = 0;
        }
        CCCR::write(CCCR::read() & ~(cccr_init | cccr_cce));
        return wait([] { return (CCCR::read() & cccr_init) == 0; });
    }

    /// Queue a frame; false if the TX FIFO is full.
    static bool send(const CanFrame& f) noexcept {
        const std::uint32_t txfqs = TXFQS::read();
        if ((txfqs & txfqs_tfqf) != 0) {
            return false;
        }
        const std::uint32_t index = (txfqs >> 16) & 3u;  // TFQPI
        auto* e = reinterpret_cast<volatile std::uint32_t*>(sram + tx_fifo + index * element_bytes);
        e[0] = f.extended ? (f.id & 0x1FFF'FFFFu) | element_xtd : (f.id & 0x7FFu) << 18;
        e[1] = static_cast<std::uint32_t>(f.length > 8 ? 8 : f.length) << 16;
        e[2] = word(f.data, 0);
        e[3] = word(f.data, 4);
        TXBAR::write(1u << index);
        return true;
    }

    /// Take the oldest received frame; false if none waits.
    static bool receive(CanFrame& f) noexcept {
        const std::uint32_t rxf0s = RXF0S::read();
        if ((rxf0s & 0xFu) == 0) {
            return false;
        }
        const std::uint32_t index = (rxf0s >> 8) & 3u;  // F0GI
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
        const bool lost = (RXF0S::read() & rxf0s_rf0l) != 0;
        if (lost) {
            IR::write(ir_rf0l);
        }
        return lost;
    }

    [[nodiscard]] static CanStatus status() noexcept {
        const std::uint32_t ecr = ECR::read();
        const std::uint32_t psr = PSR::read();  // reading clears LEC
        return {static_cast<std::uint8_t>(ecr & 0xFFu), static_cast<std::uint8_t>((ecr >> 8) & 0x7Fu),
                (psr & psr_ep) != 0, (psr & psr_bo) != 0, static_cast<std::uint8_t>(psr & 7u)};
    }

    /// Leave bus-off: back to normal operation after 128 x 11 recessive bits.
    static void recover() noexcept { CCCR::write(CCCR::read() & ~cccr_init); }

private:
    static constexpr std::uint32_t word(const std::array<std::uint8_t, 8>& d, std::size_t at) noexcept {
        return d[at] | (std::uint32_t{d[at + 1]} << 8) | (std::uint32_t{d[at + 2]} << 16) |
               (std::uint32_t{d[at + 3]} << 24);
    }

    template <class Done>
    static bool wait(Done done) noexcept {
        const std::uint32_t start = cm33::CycleCounter::now();
        while (!done()) {
            if (cm33::CycleCounter::now() - start > 10 * cycles_per_ms) {
                return false;
            }
        }
        return true;
    }

    static constexpr std::uintptr_t offset = 0x0000'A400;
    static constexpr std::uintptr_t sram = periph_alias + 0x0000'AC00;  // SRAMCAN, FDCAN1 first
    // message RAM (SRAMCAN_*): 28 standard and 8 extended filters, 3 x RX FIFO 0/1, 3 TX events,
    // 3 x TX FIFO; elements of 18 words
    static constexpr std::uintptr_t rx_fifo0 = 0x0B0, tx_fifo = 0x278, sram_bytes = 0x350;
    static constexpr std::uintptr_t element_bytes = 18 * 4;
    static constexpr std::uint32_t element_xtd = 1u << 30;

    static constexpr std::uint32_t cccr_init = 1u << 0, cccr_cce = 1u << 1, cccr_mon = 1u << 5, cccr_dar = 1u << 6,
                                   cccr_test = 1u << 7;
    static constexpr std::uint32_t test_lbck = 1u << 4;
    static constexpr std::uint32_t rxgfc_rrfe = 1u << 0, rxgfc_rrfs = 1u << 1;
    static constexpr std::uint32_t rxf0s_rf0l = 1u << 25;
    static constexpr std::uint32_t txfqs_tfqf = 1u << 21;
    static constexpr std::uint32_t psr_ep = 1u << 5, psr_bo = 1u << 7;
    static constexpr std::uint32_t ir_rf0l = 1u << 2;

    using TEST = PReg<offset + 0x10>;
    using CCCR = PReg<offset + 0x18>;
    using NBTP = PReg<offset + 0x1C>;
    using ECR = PReg<offset + 0x40>;
    using PSR = PReg<offset + 0x44>;
    using IR = PReg<offset + 0x50>;
    using RXGFC = PReg<offset + 0x80>;
    using RXF0S = PReg<offset + 0x90>;
    using RXF0A = PReg<offset + 0x94>;
    using TXBC = PReg<offset + 0xC0>;
    using TXFQS = PReg<offset + 0xC4>;
    using TXBAR = PReg<offset + 0xCC>;
    using CKDIV = PReg<offset + 0x100>;
};

}  // namespace rtos::port::stm32h5
