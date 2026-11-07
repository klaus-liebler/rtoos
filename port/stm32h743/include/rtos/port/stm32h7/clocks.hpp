// SPDX-License-Identifier: MIT
//
// The clock tree as one compile-time value. Every frequency a driver needs (CPU, HCLK, PCLK1-4,
// timer clocks, PLL1 Q) is derived from it at compile time, every limit is checked there
// (static_assert), and configure_clocks<Tree>() only writes the precomputed register values in
// the order ST's HAL uses (HAL_PWREx_ConfigSupply, HAL_PWREx_ControlVoltageScaling,
// HAL_RCC_OscConfig, HAL_RCC_ClockConfig): supply, voltage scale, oscillators, flash wait
// states, bus prescalers, PLL1, switch-over.
//
//   // HSI 64 MHz / 4 = 16 MHz reference, x 50 = 800 MHz VCO; P / 2 = 400 MHz CPU,
//   // HCLK = AXI = 200 MHz (2 wait states), APB1-4 = 100 MHz, Q / 8 = 100 MHz
//   inline constexpr rcc::ClockTree clocks{.pll1 = {.m = 4, .n = 50, .p = 2, .q = 8}};
//   configure_clocks<clocks>();
//   constexpr UartConfig console{.kernel_hz = clocks.pclk1_hz(), .baud = 115'200};
//
// Clock domains (RM0433 "RCC"): SYSCLK -> D1CPRE (= 1 here) -> CPU; -> HPRE -> HCLK (AXI, AHB1-4);
// HCLK -> D1PPRE (APB3), D2PPRE1 (APB1), D2PPRE2 (APB2), D3PPRE (APB4). Prescaler encodings:
// stm32h743xx.h RCC_D1CFGR_HPRE_DIVx (the same as the STM32H5's HPRE), RCC_D1CFGR_D1PPRE_DIVx.
// Voltage scale VOS1 only: CPU <= 400 MHz, HCLK <= 200 MHz, PCLK <= 100 MHz.
#pragma once

#include <cstdint>

#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/flash.hpp"
#include "rtos/port/stm32h7/pwr.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7::rcc {

struct ClockTree {
    PllConfig pll1;
    std::uint32_t ahb_div = 2;   ///< HPRE: 1, 2, 4, 8, 16, 64, 128, 256, 512
    std::uint32_t apb1_div = 2;  ///< D2PPRE1: 1, 2, 4, 8, 16
    std::uint32_t apb2_div = 2;  ///< D2PPRE2
    std::uint32_t apb3_div = 2;  ///< D1PPRE
    std::uint32_t apb4_div = 2;  ///< D3PPRE
    bool hse_bypass = false;     ///< HSE from an external clock on OSC_IN instead of a crystal
    bool hsi48 = false;          ///< 48 MHz oscillator (RNG, USB)

    [[nodiscard]] constexpr std::uint32_t sysclk_hz() const noexcept { return pll1.p_hz(); }
    /// The core clock (D1CPRE = 1): SysTick and the DWT cycle counter.
    [[nodiscard]] constexpr std::uint32_t cpu_hz() const noexcept { return sysclk_hz(); }
    [[nodiscard]] constexpr std::uint32_t hclk_hz() const noexcept { return cpu_hz() / ahb_div; }
    [[nodiscard]] constexpr std::uint32_t pclk1_hz() const noexcept { return hclk_hz() / apb1_div; }
    [[nodiscard]] constexpr std::uint32_t pclk2_hz() const noexcept { return hclk_hz() / apb2_div; }
    [[nodiscard]] constexpr std::uint32_t pclk3_hz() const noexcept { return hclk_hz() / apb3_div; }
    [[nodiscard]] constexpr std::uint32_t pclk4_hz() const noexcept { return hclk_hz() / apb4_div; }
    /// Timer kernel clocks (TIMPRE = 0): HCLK if the APB prescaler is 1 or 2, else 2 x PCLK -
    /// i.e. PCLK for prescaler 1, else 2 x PCLK.
    [[nodiscard]] constexpr std::uint32_t timer1_hz() const noexcept { return apb1_div == 1 ? pclk1_hz() : 2 * pclk1_hz(); }
    [[nodiscard]] constexpr std::uint32_t timer2_hz() const noexcept { return apb2_div == 1 ? pclk2_hz() : 2 * pclk2_hz(); }
    [[nodiscard]] constexpr std::uint32_t pll1_q_hz() const noexcept { return pll1.q_hz(); }

    [[nodiscard]] static constexpr std::uint32_t hpre(std::uint32_t div) noexcept {
        switch (div) {
            case 1: return 0x0;
            case 2: return 0x8;
            case 4: return 0x9;
            case 8: return 0xA;
            case 16: return 0xB;
            case 64: return 0xC;
            case 128: return 0xD;
            case 256: return 0xE;
            case 512: return 0xF;
            default: return 0xFF;  // invalid
        }
    }
    [[nodiscard]] static constexpr std::uint32_t ppre(std::uint32_t div) noexcept {
        switch (div) {
            case 1: return 0x0;
            case 2: return 0x4;
            case 4: return 0x5;
            case 8: return 0x6;
            case 16: return 0x7;
            default: return 0xFF;  // invalid
        }
    }
    /// RCC_D1CFGR: HPRE 3:0, D1PPRE 6:4, D1CPRE 11:8 (= 0, divide by 1).
    [[nodiscard]] constexpr std::uint32_t d1cfgr() const noexcept { return hpre(ahb_div) | (ppre(apb3_div) << 4); }
    /// RCC_D2CFGR: D2PPRE1 6:4, D2PPRE2 10:8.
    [[nodiscard]] constexpr std::uint32_t d2cfgr() const noexcept { return (ppre(apb1_div) << 4) | (ppre(apb2_div) << 8); }
    /// RCC_D3CFGR: D3PPRE 6:4.
    [[nodiscard]] constexpr std::uint32_t d3cfgr() const noexcept { return ppre(apb4_div) << 4; }
    [[nodiscard]] constexpr std::uint32_t wait_states() const noexcept { return flash::vos1_wait_states(hclk_hz()); }
};

/// Set up the clock tree `T` (from reset or from HSI). Everything checked at compile time.
template <ClockTree T>
void configure_clocks() noexcept {
    static_assert(T.pll1.valid(), "PLL1 configuration outside the RM0433 / ST HAL limits");
    static_assert(T.pll1.source == PllSource::Hse || T.pll1.input_hz == hsi_hz, "the HSI runs at 64 MHz");
    static_assert(T.pll1.source == PllSource::Hsi || (T.pll1.input_hz >= 4'000'000 && T.pll1.input_hz <= 48'000'000),
                  "HSE outside 4..48 MHz (ST's UTILS_HSE_FREQUENCY_MIN/MAX)");
    static_assert(ClockTree::hpre(T.ahb_div) != 0xFF, "AHB prescaler must be 1, 2, 4, 8, 16, 64, 128, 256 or 512");
    static_assert(ClockTree::ppre(T.apb1_div) != 0xFF && ClockTree::ppre(T.apb2_div) != 0xFF &&
                      ClockTree::ppre(T.apb3_div) != 0xFF && ClockTree::ppre(T.apb4_div) != 0xFF,
                  "APB prescalers must be 1, 2, 4, 8 or 16");
    static_assert(T.cpu_hz() <= pwr::vos1_max_cpu_hz, "CPU clock above 400 MHz (VOS1)");
    static_assert(T.hclk_hz() <= pwr::vos1_max_hclk_hz, "HCLK (AXI) above 200 MHz (VOS1)");
    static_assert(T.pclk1_hz() <= pwr::vos1_max_pclk_hz && T.pclk2_hz() <= pwr::vos1_max_pclk_hz &&
                      T.pclk3_hz() <= pwr::vos1_max_pclk_hz && T.pclk4_hz() <= pwr::vos1_max_pclk_hz,
                  "an APB clock above 100 MHz (VOS1)");
    static_assert(T.pll1.q_hz() <= pwr::vos1_max_cpu_hz && T.pll1.r_hz() <= pwr::vos1_max_cpu_hz,
                  "PLL1 Q/R above 400 MHz");
    static_assert(T.wait_states() <= 7, "no flash latency for this HCLK");
    constexpr std::uint32_t wait_states = T.wait_states();
    constexpr std::uint32_t delay = flash::programming_delay(T.hclk_hz());

    pwr::use_ldo();
    pwr::set_scale(pwr::Scale::Vos1);
    use_hsi();
    if constexpr (T.pll1.source == PllSource::Hse) {
        enable_hse(T.hse_bypass);
    }
    flash::set_latency(wait_states, delay);
    // Prescalers first (the HSI's 64 MHz divided by them is safe), then PLL1 as SYSCLK.
    D1CFGR::write(T.d1cfgr());
    D2CFGR::write(T.d2cfgr());
    D3CFGR::write(T.d3cfgr());
    start_pll1<T.pll1>();
    use_pll1();
    if constexpr (T.hsi48) {
        enable_hsi48();
    }
}

}  // namespace rtos::port::stm32h7::rcc
