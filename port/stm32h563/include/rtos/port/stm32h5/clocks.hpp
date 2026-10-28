// SPDX-License-Identifier: MIT
//
// The clock tree as one compile-time value. Every frequency a driver needs (HCLK, PCLK1-3,
// timer clocks, PLL1 Q) is derived from it at compile time, every limit is checked there
// (static_assert), and configure_clocks<Tree>() only writes the precomputed register values
// in the order RM0481 requires: voltage scale, flash wait states, bus prescalers, PLL1,
// switch-over.
//
//   inline constexpr rcc::ClockTree clocks{.pll1 = {.m = 32, .n = 250, .p = 2, .q = 5}};
//   configure_clocks<clocks>();
//   constexpr UartConfig console{.kernel_hz = clocks.pclk1_hz(), .baud = 115'200};
//
// The core runs in voltage scale VOS0 (the wait-state table below is the VOS0 one), from
// PLL1 fed by HSI 64 MHz. Prescaler encodings: SVD v1.8 RCC_CFGR2; limits: RM0481.
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/flash.hpp"
#include "rtos/port/stm32h5/pwr.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5::rcc {

struct ClockTree {
    Pll1Config pll1;
    std::uint32_t ahb_div = 1;   ///< 1, 2, 4, 8, 16, 64, 128, 256, 512
    std::uint32_t apb1_div = 1;  ///< 1, 2, 4, 8, 16
    std::uint32_t apb2_div = 1;
    std::uint32_t apb3_div = 1;
    bool hsi48 = false;          ///< 48 MHz oscillator (RNG, USB)

    [[nodiscard]] constexpr std::uint32_t sysclk_hz() const noexcept { return pll1.output_hz(); }
    [[nodiscard]] constexpr std::uint32_t hclk_hz() const noexcept { return sysclk_hz() / ahb_div; }
    [[nodiscard]] constexpr std::uint32_t pclk1_hz() const noexcept { return hclk_hz() / apb1_div; }
    [[nodiscard]] constexpr std::uint32_t pclk2_hz() const noexcept { return hclk_hz() / apb2_div; }
    [[nodiscard]] constexpr std::uint32_t pclk3_hz() const noexcept { return hclk_hz() / apb3_div; }
    /// Timer kernel clocks (TIMPRE = 0): PCLK if its prescaler is 1, else 2 x PCLK.
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
    [[nodiscard]] constexpr std::uint32_t cfgr2() const noexcept {
        return hpre(ahb_div) | (ppre(apb1_div) << 4) | (ppre(apb2_div) << 8) | (ppre(apb3_div) << 12);
    }
    [[nodiscard]] constexpr std::uint32_t wait_states() const noexcept { return flash::vos0_wait_states(hclk_hz()); }
};

/// Set up the clock tree `T` (from reset or from HSI). Everything checked at compile time.
template <ClockTree T>
void configure_clocks() noexcept {
    static_assert(T.pll1.valid(), "PLL1 configuration outside the RM0481 limits");
    static_assert(T.sysclk_hz() <= 250'000'000, "SYSCLK above 250 MHz (VOS0 limit)");
    static_assert(ClockTree::hpre(T.ahb_div) != 0xFF, "AHB prescaler must be 1, 2, 4, 8, 16, 64, 128, 256 or 512");
    static_assert(ClockTree::ppre(T.apb1_div) != 0xFF && ClockTree::ppre(T.apb2_div) != 0xFF &&
                      ClockTree::ppre(T.apb3_div) != 0xFF,
                  "APB prescalers must be 1, 2, 4, 8 or 16");
    static_assert(T.wait_states() <= 5, "flash wait states above 5");
    static_assert(T.pll1.q == 0 || T.pll1_q_hz() <= 250'000'000, "PLL1 Q above 250 MHz");
    constexpr std::uint32_t cfgr2 = T.cfgr2();
    constexpr std::uint32_t wait_states = T.wait_states();
    constexpr std::uint32_t delay = flash::programming_delay(T.hclk_hz());

    use_hsi(HsiDiv::Div1);
    pwr::set_voltage_scale(pwr::VoltageScale::Vos0);
    flash::set_latency(wait_states, delay);
    CFGR2::write(cfgr2);
    use_pll1<T.pll1>();
    if constexpr (T.hsi48) {
        enable_hsi48();
    }
}

}  // namespace rtos::port::stm32h5::rcc
