// SPDX-License-Identifier: MIT
//
// The clock tree as one compile-time value. Every frequency a driver needs (HCLK, PCLK1/2,
// timer clocks, ADC clock) is derived from it at compile time, every limit is checked there
// (static_assert), and configure_clocks<Tree>() only writes the precomputed register values in
// the order RM0008 requires: flash wait states, HSE, PLL, switch-over.
//
//   // HSE 12 MHz from an oscillator (bypass) x 6 = 72 MHz SYSCLK, APB1 36 MHz
//   inline constexpr rcc::ClockTree clocks{.pll_source = rcc::PllSource::Hse, .hse_hz = 12'000'000,
//                                          .hse_bypass = true, .pll_mul = 6};
//   if (!rcc::configure_clocks<clocks>()) { ... HSE did not start ... }
//   constexpr UartConfig console{.kernel_hz = clocks.pclk2_hz(), .baud = 115'200};
//
// Limits: RM0008 rev 21 chapter 7 (PLL output <= 72 MHz, APB1 <= 36 MHz, ADC <= 14 MHz) and the
// datasheet DS5319 (HSE crystal 4..16 MHz, external clock 1..25 MHz, PLL input 1..25 MHz).
//
// Silicon erratum ES096 2.2.1 "Flash memory read after WFI/WFE instruction": with prefetch on
// and 2 wait states the instruction fetch after a very short Sleep can be corrupted when the
// FLITF clock stops in Sleep mode. The kernel's idle thread sleeps with WFI, so
// configure_clocks() keeps RCC_AHBENR.FLITFEN set (its reset value) - never clear it.
#pragma once

#include <cstdint>

#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/flash.hpp"
#include "rtos/port/stm32f1/rcc.hpp"

namespace rtos::port::stm32f1::rcc {

enum class SysclkSource : std::uint8_t { Hsi, Hse, Pll };
enum class PllSource : std::uint8_t { HsiDiv2, Hse, HseDiv2 };

struct ClockTree {
    SysclkSource sysclk = SysclkSource::Pll;
    PllSource pll_source = PllSource::HsiDiv2;
    std::uint32_t hse_hz = 0;      ///< the board's crystal or oscillator (0: none)
    bool hse_bypass = false;       ///< HSE from an external clock on OSC_IN instead of a crystal
    std::uint32_t pll_mul = 9;     ///< 2..16
    std::uint32_t ahb_div = 1;     ///< 1, 2, 4, 8, 16, 64, 128, 256, 512
    std::uint32_t apb1_div = 2;    ///< 1, 2, 4, 8, 16 (PCLK1 <= 36 MHz)
    std::uint32_t apb2_div = 1;
    std::uint32_t adc_div = 6;     ///< 2, 4, 6, 8 (ADC clock <= 14 MHz)

    [[nodiscard]] constexpr bool uses_hse() const noexcept {
        return sysclk == SysclkSource::Hse || (sysclk == SysclkSource::Pll && pll_source != PllSource::HsiDiv2);
    }
    [[nodiscard]] constexpr std::uint32_t pll_input_hz() const noexcept {
        switch (pll_source) {
            case PllSource::HsiDiv2: return hsi_hz / 2;
            case PllSource::Hse: return hse_hz;
            case PllSource::HseDiv2: return hse_hz / 2;
        }
        return 0;
    }
    [[nodiscard]] constexpr std::uint32_t pll_hz() const noexcept { return pll_input_hz() * pll_mul; }
    [[nodiscard]] constexpr std::uint32_t sysclk_hz() const noexcept {
        switch (sysclk) {
            case SysclkSource::Hsi: return hsi_hz;
            case SysclkSource::Hse: return hse_hz;
            case SysclkSource::Pll: return pll_hz();
        }
        return 0;
    }
    [[nodiscard]] constexpr std::uint32_t hclk_hz() const noexcept { return sysclk_hz() / ahb_div; }
    [[nodiscard]] constexpr std::uint32_t pclk1_hz() const noexcept { return hclk_hz() / apb1_div; }
    [[nodiscard]] constexpr std::uint32_t pclk2_hz() const noexcept { return hclk_hz() / apb2_div; }
    /// Timer clocks: PCLK if its prescaler is 1, else 2 x PCLK. timer1_hz: TIM2-4 (APB1),
    /// timer2_hz: TIM1 (APB2).
    [[nodiscard]] constexpr std::uint32_t timer1_hz() const noexcept { return apb1_div == 1 ? pclk1_hz() : 2 * pclk1_hz(); }
    [[nodiscard]] constexpr std::uint32_t timer2_hz() const noexcept { return apb2_div == 1 ? pclk2_hz() : 2 * pclk2_hz(); }
    [[nodiscard]] constexpr std::uint32_t adc_hz() const noexcept { return pclk2_hz() / adc_div; }
    [[nodiscard]] constexpr std::uint32_t wait_states() const noexcept { return flash::wait_states(sysclk_hz()); }

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
    [[nodiscard]] static constexpr std::uint32_t adcpre(std::uint32_t div) noexcept {
        return div == 2 || div == 4 || div == 6 || div == 8 ? div / 2 - 1 : 0xFF;
    }

    /// The limits (RM0008, DS5319) as one predicate - the host tests check it case by case.
    [[nodiscard]] constexpr bool valid() const noexcept {
        const bool hse_ok = !uses_hse() || (hse_bypass ? hse_hz >= 1'000'000 && hse_hz <= 25'000'000
                                                       : hse_hz >= 4'000'000 && hse_hz <= 16'000'000);
        const bool pll_ok = sysclk != SysclkSource::Pll ||
                            (pll_mul >= 2 && pll_mul <= 16 && pll_input_hz() >= 1'000'000 &&
                             pll_input_hz() <= 25'000'000 && pll_hz() <= 72'000'000);
        return hse_ok && pll_ok && sysclk_hz() <= 72'000'000 && hpre(ahb_div) != 0xFF && ppre(apb1_div) != 0xFF &&
               ppre(apb2_div) != 0xFF && adcpre(adc_div) != 0xFF && pclk1_hz() <= 36'000'000 &&
               pclk2_hz() <= 72'000'000 && adc_hz() <= 14'000'000;
    }

    /// RCC_CFGR without SW: HPRE 7:4, PPRE1 10:8, PPRE2 13:11, ADCPRE 15:14, PLLSRC 16,
    /// PLLXTPRE 17, PLLMUL 21:18.
    [[nodiscard]] constexpr std::uint32_t cfgr() const noexcept {
        std::uint32_t v = (hpre(ahb_div) << cfgr_hpre_pos) | (ppre(apb1_div) << cfgr_ppre1_pos) |
                          (ppre(apb2_div) << cfgr_ppre2_pos) | (adcpre(adc_div) << cfgr_adcpre_pos);
        if (sysclk == SysclkSource::Pll) {
            v |= (pll_mul - 2) << cfgr_pllmul_pos;
            if (pll_source != PllSource::HsiDiv2) v |= cfgr_pllsrc;
            if (pll_source == PllSource::HseDiv2) v |= cfgr_pllxtpre;
        }
        return v;
    }
};

/// How long configure_clocks() waits for the HSE: polls of HSERDY at the 8 MHz HSI, several
/// times the crystal start-up time (DS5319: typically 2 ms); an oscillator (bypass) is at once.
inline constexpr std::uint32_t hse_start_polls = 200'000;

/// Set up the clock tree `T` (from reset or from the HSI). Everything checked at compile time.
/// False if the HSE did not start: SYSCLK then stays on the HSI (8 MHz), nothing else changed.
template <ClockTree T>
[[nodiscard]] bool configure_clocks() noexcept {
    static_assert(T.valid(), "clock tree outside the RM0008 / DS5319 limits");
    static_assert(!T.uses_hse() || T.hse_hz != 0, "an HSE clock tree needs hse_hz");
    static_assert(T.sysclk != SysclkSource::Pll || T.pll_source != PllSource::HsiDiv2 || T.pll_hz() <= 64'000'000,
                  "from the HSI the PLL reaches at most 4 MHz x 16 = 64 MHz");
    constexpr std::uint32_t cfgr = T.cfgr();

    use_hsi();
    AHBENR::set(ahbenr_flitfen);  // ES096 2.2.1: the FLITF clock keeps running in Sleep mode
    if constexpr (T.uses_hse()) {
        if (!enable_hse(T.hse_bypass, hse_start_polls)) {
            return false;
        }
    }
    flash::set_latency(T.wait_states() > flash::wait_states(hsi_hz) ? T.wait_states() : flash::wait_states(hsi_hz));
    if constexpr (T.sysclk == SysclkSource::Pll) {
        CR::clear(cr_pllon);
        while ((CR::read() & cr_pllrdy) != 0) {
        }
        CFGR::write(cfgr | sw_hsi);
        CR::set(cr_pllon);
        while ((CR::read() & cr_pllrdy) == 0) {
        }
        use_pll();
    } else {
        CFGR::write(cfgr | (T.sysclk == SysclkSource::Hse ? sw_hse : sw_hsi));
        while ((CFGR::read() & cfgr_sws_msk) != ((CFGR::read() & cfgr_sw_msk) << cfgr_sws_pos)) {
        }
    }
    flash::set_latency(T.wait_states());
    return true;
}

}  // namespace rtos::port::stm32f1::rcc
