// SPDX-License-Identifier: MIT
//
// The clock tree as one compile-time value. Every frequency a driver needs (HCLK, PCLK1/2,
// timer clocks, PLL Q/P) is derived from it at compile time, every limit is checked there
// (static_assert), and configure_clocks<Tree>() only writes the precomputed register values
// in the order RM0440 requires: PWR clock, range 1 (boost above 150 MHz), flash wait states,
// PLL, switch-over, bus prescalers.
//
//   // HSE 8 MHz / 2 x 85 = 340 MHz VCO, R / 2 = 170 MHz SYSCLK, Q / 8 = 42.5 MHz
//   inline constexpr rcc::ClockTree clocks{
//       .pll = {.source = rcc::PllSource::Hse, .input_hz = 8'000'000, .m = 2, .n = 85, .r = 2, .q = 8}};
//   configure_clocks<clocks>();
//   constexpr UartConfig console{.kernel_hz = clocks.pclk2_hz(), .baud = 115'200};
//
// Prescaler encodings: SVD v1.8 RCC_CFGR (the same as the STM32H5's HPRE/PPRE); limits: RM0440,
// ST's stm32g4xx_ll_utils.c (170 MHz in boost mode, 150 MHz in normal range 1).
#pragma once

#include <cstdint>

#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/flash.hpp"
#include "rtos/port/stm32g4/pwr.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4::rcc {

struct ClockTree {
    PllConfig pll;
    std::uint32_t ahb_div = 1;   ///< 1, 2, 4, 8, 16, 64, 128, 256, 512
    std::uint32_t apb1_div = 1;  ///< 1, 2, 4, 8, 16
    std::uint32_t apb2_div = 1;
    bool hse_bypass = false;     ///< HSE from an external clock on OSC_IN instead of a crystal
    bool hsi48 = false;          ///< 48 MHz oscillator (RNG, USB)

    [[nodiscard]] constexpr std::uint32_t sysclk_hz() const noexcept { return pll.output_hz(); }
    [[nodiscard]] constexpr std::uint32_t hclk_hz() const noexcept { return sysclk_hz() / ahb_div; }
    [[nodiscard]] constexpr std::uint32_t pclk1_hz() const noexcept { return hclk_hz() / apb1_div; }
    [[nodiscard]] constexpr std::uint32_t pclk2_hz() const noexcept { return hclk_hz() / apb2_div; }
    /// Timer kernel clocks: PCLK if its prescaler is 1, else 2 x PCLK.
    [[nodiscard]] constexpr std::uint32_t timer1_hz() const noexcept { return apb1_div == 1 ? pclk1_hz() : 2 * pclk1_hz(); }
    [[nodiscard]] constexpr std::uint32_t timer2_hz() const noexcept { return apb2_div == 1 ? pclk2_hz() : 2 * pclk2_hz(); }
    [[nodiscard]] constexpr std::uint32_t pll_q_hz() const noexcept { return pll.q_hz(); }
    [[nodiscard]] constexpr std::uint32_t pll_p_hz() const noexcept { return pll.p_hz(); }
    /// UCPD kernel clock: HSI16 (it keeps running: the UCPD needs it).
    [[nodiscard]] constexpr std::uint32_t ucpd_hz() const noexcept { return hsi16_hz; }
    /// Range 1 boost mode (PWR_CR5.R1MODE = 0) above 150 MHz.
    [[nodiscard]] constexpr bool boost() const noexcept { return sysclk_hz() > pwr::max_hz_normal; }

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
    /// RCC_CFGR without SW: HPRE 7:4, PPRE1 10:8, PPRE2 13:11.
    [[nodiscard]] constexpr std::uint32_t cfgr() const noexcept {
        return (hpre(ahb_div) << 4) | (ppre(apb1_div) << 8) | (ppre(apb2_div) << 11);
    }
    [[nodiscard]] constexpr std::uint32_t wait_states() const noexcept {
        return flash::range1_wait_states(hclk_hz(), boost());
    }
};

/// Set up the clock tree `T` (from reset or from HSI16). Everything checked at compile time.
template <ClockTree T>
void configure_clocks() noexcept {
    static_assert(T.pll.valid(), "PLL configuration outside the RM0440 / CubeMX limits");
    static_assert(T.sysclk_hz() <= pwr::max_hz_boost, "SYSCLK above 170 MHz (range 1 boost limit)");
    static_assert(T.pll.source == PllSource::Hse || T.pll.input_hz == hsi16_hz, "the HSI16 runs at 16 MHz");
    static_assert(T.pll.source == PllSource::Hsi16 || (T.pll.input_hz >= 4'000'000 && T.pll.input_hz <= 48'000'000),
                  "HSE outside 4..48 MHz (ST's UTILS_HSE_FREQUENCY_MIN/MAX)");
    static_assert(ClockTree::hpre(T.ahb_div) != 0xFF, "AHB prescaler must be 1, 2, 4, 8, 16, 64, 128, 256 or 512");
    static_assert(ClockTree::ppre(T.apb1_div) != 0xFF && ClockTree::ppre(T.apb2_div) != 0xFF,
                  "APB prescalers must be 1, 2, 4, 8 or 16");
    static_assert(T.wait_states() <= 4, "flash wait states above 4");
    constexpr std::uint32_t cfgr = T.cfgr();
    constexpr std::uint32_t wait_states = T.wait_states();

    pwr::enable_clock();
    use_hsi16();
    if constexpr (T.pll.source == PllSource::Hse) {
        enable_hse(T.hse_bypass);
    }
    pwr::set_range1(T.boost());
    flash::set_latency(wait_states);
    start_pll<T.pll>();
    if constexpr (T.sysclk_hz() > 80'000'000 && T.ahb_div == 1) {
        // To a PLL above 80 MHz the switch-over goes through HCLK = SYSCLK / 2 for 1 us
        // (RM0440, ST's HAL_RCC_ClockConfig): no current step on the core supply.
        CFGR::write((CFGR::read() & cfgr_sw_msk) | (cfgr & ~cfgr_hpre_msk) | (ClockTree::hpre(2) << cfgr_hpre_pos));
        use_pll();
        for (std::uint32_t i = 0; i < T.sysclk_hz() / 2 / 1'000'000; ++i) {  // >= 1 cycle per iteration
            __asm volatile("" ::: "memory");
        }
    } else {
        CFGR::write((CFGR::read() & cfgr_sw_msk) | cfgr);
        use_pll();
    }
    CFGR::write((CFGR::read() & cfgr_sw_msk) | cfgr);
    if constexpr (T.hsi48) {
        enable_hsi48();
    }
}

}  // namespace rtos::port::stm32g4::rcc
