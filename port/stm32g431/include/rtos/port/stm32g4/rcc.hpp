// SPDX-License-Identifier: MIT
//
// STM32G431 reset and clock control: oscillators (HSI16, HSE, HSI48), the main PLL, peripheral
// clock gates and kernel clock selections (RM0440 "RCC"; registers, bits: SVD STM32G431 v1.8;
// CCIPR encodings and the PLL parameter ranges: ST's stm32g4xx_ll_rcc.h / stm32g4xx_hal_rcc.h).
#pragma once

#include <cstdint>

#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32g4/chip.hpp"

namespace rtos::port::stm32g4::rcc {

inline constexpr std::uintptr_t base = periph_base + 0x0002'1000;

using CR = PReg<0x0002'1000 + 0x00>;       // reset 0x0000'0063: HSI16 on
using CFGR = PReg<0x0002'1000 + 0x08>;     // reset 0x0000'0005: SYSCLK = HSI16
using PLLCFGR = PReg<0x0002'1000 + 0x0C>;
using AHB1ENR = PReg<0x0002'1000 + 0x48>;
using AHB2ENR = PReg<0x0002'1000 + 0x4C>;
using APB1ENR1 = PReg<0x0002'1000 + 0x58>;
using APB1ENR2 = PReg<0x0002'1000 + 0x5C>;
using APB2ENR = PReg<0x0002'1000 + 0x60>;
using CCIPR = PReg<0x0002'1000 + 0x88>;
using CRRCR = PReg<0x0002'1000 + 0x98>;

inline constexpr std::uint32_t cr_hsion = 1u << 8, cr_hsirdy = 1u << 10;
inline constexpr std::uint32_t cr_hseon = 1u << 16, cr_hserdy = 1u << 17, cr_hsebyp = 1u << 18;
inline constexpr std::uint32_t cr_pllon = 1u << 24, cr_pllrdy = 1u << 25;
inline constexpr std::uint32_t cfgr_sw_msk = 3u << 0, cfgr_sws_pos = 2, cfgr_sws_msk = 3u << cfgr_sws_pos;
inline constexpr std::uint32_t cfgr_hpre_pos = 4, cfgr_hpre_msk = 0xFu << cfgr_hpre_pos;
inline constexpr std::uint32_t sw_hsi16 = 1, sw_hse = 2, sw_pll = 3;
inline constexpr std::uint32_t pllcfgr_src_hsi16 = 2, pllcfgr_src_hse = 3;
inline constexpr std::uint32_t pllcfgr_m_pos = 4, pllcfgr_n_pos = 8, pllcfgr_pen = 1u << 16, pllcfgr_qen = 1u << 20,
                               pllcfgr_q_pos = 21, pllcfgr_ren = 1u << 24, pllcfgr_r_pos = 25, pllcfgr_pdiv_pos = 27;
inline constexpr std::uint32_t crrcr_hsi48on = 1u << 0, crrcr_hsi48rdy = 1u << 1;

/// Run SYSCLK from HSI16 (the reset state; also the safe state to reconfigure the PLL from).
inline void use_hsi16() noexcept {
    CR::set(cr_hsion);
    while ((CR::read() & cr_hsirdy) == 0) {
    }
    CFGR::write((CFGR::read() & ~cfgr_sw_msk) | sw_hsi16);
    while ((CFGR::read() & cfgr_sws_msk) != (sw_hsi16 << cfgr_sws_pos)) {
    }
}

/// Start the HSE: a crystal, or with `bypass` an external clock on OSC_IN.
inline void enable_hse(bool bypass) noexcept {
    CR::write((CR::read() & ~cr_hsebyp) | (bypass ? cr_hsebyp : 0u));
    CR::set(cr_hseon);
    while ((CR::read() & cr_hserdy) == 0) {
    }
}

/// 48 MHz internal oscillator (clock of the RNG and the USB, RCC_CCIPR.CLK48SEL reset value).
inline void enable_hsi48() noexcept {
    CRRCR::set(crrcr_hsi48on);
    while ((CRRCR::read() & crrcr_hsi48rdy) == 0) {
    }
}

enum class PllSource : std::uint8_t { Hsi16, Hse };

/// Main PLL: f_vco = f_in / m * n; R (SYSCLK) = f_vco / r, Q (USB, FDCAN) = f_vco / q,
/// P (ADC) = f_vco / p.
struct PllConfig {
    PllSource source = PllSource::Hsi16;
    std::uint32_t input_hz = hsi16_hz;  ///< HSI16, or the board's HSE frequency
    std::uint32_t m;                    ///< 1..16
    std::uint32_t n;                    ///< 8..127
    std::uint32_t r;                    ///< 2, 4, 6, 8
    std::uint32_t q = 0;                ///< 2, 4, 6, 8, or 0: Q output off
    std::uint32_t p = 0;                ///< 2..31, or 0: P output off

    [[nodiscard]] constexpr std::uint32_t reference_hz() const noexcept { return input_hz / m; }
    [[nodiscard]] constexpr std::uint32_t vco_hz() const noexcept { return reference_hz() * n; }
    [[nodiscard]] constexpr std::uint32_t output_hz() const noexcept { return vco_hz() / r; }
    [[nodiscard]] constexpr std::uint32_t q_hz() const noexcept { return q == 0 ? 0 : vco_hz() / q; }
    [[nodiscard]] constexpr std::uint32_t p_hz() const noexcept { return p == 0 ? 0 : vco_hz() / p; }

    [[nodiscard]] static constexpr bool even_2_to_8(std::uint32_t d) noexcept {
        return d == 2 || d == 4 || d == 6 || d == 8;
    }
    /// Register ranges (stm32g4xx_hal_rcc.h IS_RCC_PLL*_VALUE) and the PLL's operating limits
    /// (STM32CubeMX db IP/RCC-STM32G4_rcc_v1_0_Modes.xml: VCO input 2.66-16 MHz, VCO output
    /// 96-344 MHz, R/Q/P outputs <= 170 MHz).
    [[nodiscard]] constexpr bool valid() const noexcept {
        return m >= 1 && m <= 16 && n >= 8 && n <= 127 && even_2_to_8(r) && (q == 0 || even_2_to_8(q)) &&
               (p == 0 || (p >= 2 && p <= 31)) && reference_hz() >= 2'660'000 && reference_hz() <= 16'000'000 &&
               vco_hz() >= 96'000'000 && vco_hz() <= 344'000'000 && output_hz() <= 170'000'000 &&
               q_hz() <= 170'000'000 && p_hz() <= 170'000'000;
    }

    /// RCC_PLLCFGR (PLLR/PLLQ: 00 /2, 01 /4, 10 /6, 11 /8; PLLPDIV = p).
    [[nodiscard]] constexpr std::uint32_t pllcfgr() const noexcept {
        return (source == PllSource::Hse ? pllcfgr_src_hse : pllcfgr_src_hsi16) | ((m - 1) << pllcfgr_m_pos) |
               (n << pllcfgr_n_pos) | pllcfgr_ren | ((r / 2 - 1) << pllcfgr_r_pos) |
               (q != 0 ? pllcfgr_qen | ((q / 2 - 1) << pllcfgr_q_pos) : 0u) |
               (p != 0 ? pllcfgr_pen | (p << pllcfgr_pdiv_pos) : 0u);
    }
};

/// Configure and start the PLL (it must not be the system clock: use_hsi16() first).
template <PllConfig Cfg>
inline void start_pll() noexcept {
    static_assert(Cfg.valid(), "PLL configuration outside the RM0440 / CubeMX limits");
    CR::clear(cr_pllon);
    while ((CR::read() & cr_pllrdy) != 0) {
    }
    PLLCFGR::write(Cfg.pllcfgr());
    CR::set(cr_pllon);
    while ((CR::read() & cr_pllrdy) == 0) {
    }
}

inline void use_pll() noexcept {
    CFGR::write((CFGR::read() & ~cfgr_sw_msk) | sw_pll);
    while ((CFGR::read() & cfgr_sws_msk) != (sw_pll << cfgr_sws_pos)) {
    }
}

// ---- peripheral clock gates (SVD v1.8, RCC_xxxENR; RCC_xxxRSTR = ENR - 0x20) --------------

using stm32::rcc::ClockSelection, stm32::rcc::Gate, stm32::rcc::KernelClock;
using stm32::rcc::disable, stm32::rcc::enable, stm32::rcc::reset, stm32::rcc::select;

namespace detail {
inline constexpr std::uint16_t ahb1 = 0x48, ahb2 = 0x4C, apb1_1 = 0x58, apb1_2 = 0x5C, apb2 = 0x60;
inline constexpr std::uint16_t enr_to_rstr = 0x20;
[[nodiscard]] constexpr Gate gate(std::uint16_t enr, std::uint8_t bit) noexcept {
    return {base + enr, base + enr - enr_to_rstr, bit};
}
}  // namespace detail

namespace gate {
using detail::ahb1, detail::ahb2, detail::apb1_1, detail::apb1_2, detail::apb2, detail::gate;
inline constexpr Gate dma1 = gate(ahb1, 0), dma2 = gate(ahb1, 1), dmamux1 = gate(ahb1, 2), crc = gate(ahb1, 12);
inline constexpr Gate adc12 = gate(ahb2, 13), dac1 = gate(ahb2, 16), dac3 = gate(ahb2, 18), rng = gate(ahb2, 26);
inline constexpr Gate tim2 = gate(apb1_1, 0), tim3 = gate(apb1_1, 1), tim4 = gate(apb1_1, 2), tim6 = gate(apb1_1, 4),
                      tim7 = gate(apb1_1, 5), crs = gate(apb1_1, 8), spi2 = gate(apb1_1, 14), spi3 = gate(apb1_1, 15),
                      usart2 = gate(apb1_1, 17), usart3 = gate(apb1_1, 18), uart4 = gate(apb1_1, 19),
                      i2c1 = gate(apb1_1, 21), i2c2 = gate(apb1_1, 22), usb = gate(apb1_1, 23),
                      fdcan = gate(apb1_1, 25), pwr = gate(apb1_1, 28), i2c3 = gate(apb1_1, 30);
inline constexpr Gate lpuart1 = gate(apb1_2, 0), ucpd1 = gate(apb1_2, 8);
inline constexpr Gate syscfg = gate(apb2, 0), tim1 = gate(apb2, 11), spi1 = gate(apb2, 12), tim8 = gate(apb2, 13),
                      usart1 = gate(apb2, 14), tim15 = gate(apb2, 16), tim16 = gate(apb2, 17), tim17 = gate(apb2, 18),
                      sai1 = gate(apb2, 21);
}  // namespace gate

// ---- kernel clock selection (RCC_CCIPR; encodings: stm32g4xx_ll_rcc.h) -------------------

namespace kernel_clock {
inline constexpr KernelClock fdcan{base + 0x88, 24, 2};   ///< 0: HSE (reset), 1: PLL Q, 2: PCLK1
inline constexpr KernelClock clk48{base + 0x88, 26, 2};   ///< 0: HSI48 (reset), 2: PLL Q - USB, RNG
inline constexpr KernelClock adc12{base + 0x88, 28, 2};   ///< 0: none (reset), 1: PLL P, 2: SYSCLK
inline constexpr std::uint32_t fdcan_pll_q = 1, fdcan_pclk1 = 2;
inline constexpr std::uint32_t adc12_pll_p = 1, adc12_sysclk = 2;
}  // namespace kernel_clock

using GpioPort = stm32::GpioPort;

inline void enable(GpioPort port) noexcept {
    AHB2ENR::set(1u << static_cast<std::uint32_t>(port));
    (void)AHB2ENR::read();  // RM0440: wait 2 cycles after enabling a clock
}

}  // namespace rtos::port::stm32g4::rcc
