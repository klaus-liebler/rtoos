// SPDX-License-Identifier: MIT
//
// STM32H743 reset and clock control: oscillators (HSI, HSE, HSI48), PLL1, the domain
// prescalers, peripheral clock gates (RM0433 "RCC"; registers, bits: SVD STM32H743 v2.4;
// encodings: ST's stm32h743xx.h; PLL parameter ranges: ST's stm32h7xx_hal_rcc.h IS_RCC_PLL*_VALUE
// and stm32h7xx_ll_utils.c UTILS_PLLVCO_* for the three-domain parts).
//
// Only PLL1 so far (SYSCLK, and Q for the peripheral kernel clocks); PLL2/PLL3 stay off.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32h7/chip.hpp"

namespace rtos::port::stm32h7::rcc {

inline constexpr std::uintptr_t base = 0x5802'4400;

using CR = Reg<base + 0x000>;
using CFGR = Reg<base + 0x010>;       // reset 0: SYSCLK = HSI
using D1CFGR = Reg<base + 0x018>;     // D1CPRE, HPRE, D1PPRE (APB3)
using D2CFGR = Reg<base + 0x01C>;     // D2PPRE1 (APB1), D2PPRE2 (APB2)
using D3CFGR = Reg<base + 0x020>;     // D3PPRE (APB4)
using PLLCKSELR = Reg<base + 0x028>;  // reset 0x0202'0200: HSI, DIVM1..3 = 32
using PLLCFGR = Reg<base + 0x02C>;    // reset 0x01FF'0000
using PLL1DIVR = Reg<base + 0x030>;   // reset 0x0101'0280
using PLL1FRACR = Reg<base + 0x034>;
using AHB4ENR = Reg<base + 0x0E0>;

inline constexpr std::uint32_t cr_hsion = 1u << 0, cr_hsirdy = 1u << 2, cr_hsidiv_pos = 3,
                               cr_hsidiv_msk = 3u << cr_hsidiv_pos, cr_hsidivf = 1u << 5;
inline constexpr std::uint32_t cr_rc48on = 1u << 12, cr_rc48rdy = 1u << 13;
inline constexpr std::uint32_t cr_hseon = 1u << 16, cr_hserdy = 1u << 17, cr_hsebyp = 1u << 18;
inline constexpr std::uint32_t cr_pll1on = 1u << 24, cr_pll1rdy = 1u << 25;
inline constexpr std::uint32_t cfgr_sw_msk = 7u << 0, cfgr_sws_pos = 3, cfgr_sws_msk = 7u << cfgr_sws_pos;
inline constexpr std::uint32_t sw_hsi = 0, sw_hse = 2, sw_pll1 = 3;
inline constexpr std::uint32_t pllckselr_src_msk = 3u << 0, pllckselr_src_hsi = 0, pllckselr_src_hse = 2,
                               pllckselr_divm1_pos = 4, pllckselr_divm1_msk = 0x3Fu << pllckselr_divm1_pos;
inline constexpr std::uint32_t pllcfgr_pll1fracen = 1u << 0, pllcfgr_pll1vcosel = 1u << 1, pllcfgr_pll1rge_pos = 2,
                               pllcfgr_pll1_msk = 0xFu << 0, pllcfgr_divp1en = 1u << 16, pllcfgr_divq1en = 1u << 17,
                               pllcfgr_divr1en = 1u << 18, pllcfgr_div1en_msk = 7u << 16;
inline constexpr std::uint32_t pll1divr_n_pos = 0, pll1divr_p_pos = 9, pll1divr_q_pos = 16, pll1divr_r_pos = 24;

/// Run SYSCLK from the HSI at 64 MHz (HSIDIV = 1): the reset state, and the safe state to
/// reconfigure PLL1 from.
inline void use_hsi() noexcept {
    CR::set(cr_hsion);
    while ((CR::read() & cr_hsirdy) == 0) {
    }
    CR::write(CR::read() & ~cr_hsidiv_msk);  // HSIDIV = 1
    while ((CR::read() & cr_hsidivf) == 0) {
    }
    CFGR::write((CFGR::read() & ~cfgr_sw_msk) | sw_hsi);
    while ((CFGR::read() & cfgr_sws_msk) != (sw_hsi << cfgr_sws_pos)) {
    }
}

/// Start the HSE: a crystal, or with `bypass` an external clock on OSC_IN.
inline void enable_hse(bool bypass) noexcept {
    CR::write((CR::read() & ~cr_hsebyp) | (bypass ? cr_hsebyp : 0u));
    CR::set(cr_hseon);
    while ((CR::read() & cr_hserdy) == 0) {
    }
}

/// 48 MHz internal oscillator (RC48: kernel clock of the RNG and the USB OTG, trimmed by the
/// CRS from the USB SOF).
inline void enable_hsi48() noexcept {
    CR::set(cr_rc48on);
    while ((CR::read() & cr_rc48rdy) == 0) {
    }
}

enum class PllSource : std::uint8_t { Hsi, Hse };

/// PLL1: reference = f_in / m, f_vco = reference * n; P (SYSCLK) = f_vco / p, Q = f_vco / q
/// (peripheral kernel clocks), R = f_vco / r. Integer mode (no FRACN).
struct PllConfig {
    PllSource source = PllSource::Hsi;
    std::uint32_t input_hz = hsi_hz;  ///< HSI 64 MHz, or the board's HSE frequency
    std::uint32_t m;                  ///< 1..63
    std::uint32_t n;                  ///< 4..512
    std::uint32_t p;                  ///< 2..128, even
    std::uint32_t q = 0;              ///< 1..128, or 0: Q output off
    std::uint32_t r = 0;              ///< 1..128, or 0: R output off

    [[nodiscard]] constexpr std::uint32_t reference_hz() const noexcept { return input_hz / m; }
    [[nodiscard]] constexpr std::uint32_t vco_hz() const noexcept { return reference_hz() * n; }
    [[nodiscard]] constexpr std::uint32_t p_hz() const noexcept { return vco_hz() / p; }
    [[nodiscard]] constexpr std::uint32_t q_hz() const noexcept { return q == 0 ? 0 : vco_hz() / q; }
    [[nodiscard]] constexpr std::uint32_t r_hz() const noexcept { return r == 0 ? 0 : vco_hz() / r; }

    /// PLL1RGE: 0 = 1-2 MHz, 1 = 2-4 MHz, 2 = 4-8 MHz, 3 = 8-16 MHz reference.
    [[nodiscard]] constexpr std::uint32_t range() const noexcept {
        const std::uint32_t ref = reference_hz();
        return ref < 2'000'000 ? 0 : (ref < 4'000'000 ? 1 : (ref < 8'000'000 ? 2 : 3));
    }
    /// The medium VCO (150-420 MHz) for a 1-2 MHz reference, else the wide VCO (192-836 MHz).
    [[nodiscard]] constexpr bool medium_vco() const noexcept { return range() == 0; }

    /// Register ranges (stm32h7xx_hal_rcc.h; P even only - stricter than the HAL's 1..128,
    /// P = 2 as in ST's examples) and the operating limits (stm32h7xx_ll_utils.c, three
    /// power domains: reference 1-16 MHz, VCO medium 150-420 MHz, wide 192-836 MHz).
    [[nodiscard]] constexpr bool valid() const noexcept {
        const std::uint32_t ref = reference_hz(), vco = vco_hz();
        return m >= 1 && m <= 63 && n >= 4 && n <= 512 && p >= 2 && p <= 128 && p % 2 == 0 && q <= 128 &&
               r <= 128 && ref >= 1'000'000 && ref <= 16'000'000 &&
               (medium_vco() ? vco >= 150'000'000 && vco <= 420'000'000 : vco >= 192'000'000 && vco <= 836'000'000);
    }

    [[nodiscard]] constexpr std::uint32_t pllckselr() const noexcept {
        return (source == PllSource::Hse ? pllckselr_src_hse : pllckselr_src_hsi) | (m << pllckselr_divm1_pos);
    }
    /// PLL1 bits of RCC_PLLCFGR: integer mode, range, VCO, the enabled outputs.
    [[nodiscard]] constexpr std::uint32_t pllcfgr() const noexcept {
        return (range() << pllcfgr_pll1rge_pos) | (medium_vco() ? pllcfgr_pll1vcosel : 0u) | pllcfgr_divp1en |
               (q != 0 ? pllcfgr_divq1en : 0u) | (r != 0 ? pllcfgr_divr1en : 0u);
    }
    /// RCC_PLL1DIVR: every factor minus one (an unused output keeps divider 2).
    [[nodiscard]] constexpr std::uint32_t pll1divr() const noexcept {
        return ((n - 1) << pll1divr_n_pos) | ((p - 1) << pll1divr_p_pos) | (((q != 0 ? q : 2) - 1) << pll1divr_q_pos) |
               (((r != 0 ? r : 2) - 1) << pll1divr_r_pos);
    }
};

/// Configure and start PLL1 (it must not be the system clock: use_hsi() first).
template <PllConfig Cfg>
inline void start_pll1() noexcept {
    static_assert(Cfg.valid(), "PLL1 configuration outside the RM0433 / ST HAL limits");
    CR::clear(cr_pll1on);
    while ((CR::read() & cr_pll1rdy) != 0) {
    }
    PLLCKSELR::write((PLLCKSELR::read() & ~(pllckselr_src_msk | pllckselr_divm1_msk)) | Cfg.pllckselr());
    PLL1DIVR::write(Cfg.pll1divr());
    PLL1FRACR::write(0);
    PLLCFGR::write((PLLCFGR::read() & ~(pllcfgr_pll1_msk | pllcfgr_div1en_msk)) | Cfg.pllcfgr());
    CR::set(cr_pll1on);
    while ((CR::read() & cr_pll1rdy) == 0) {
    }
}

inline void use_pll1() noexcept {
    CFGR::write((CFGR::read() & ~cfgr_sw_msk) | sw_pll1);
    while ((CFGR::read() & cfgr_sws_msk) != (sw_pll1 << cfgr_sws_pos)) {
    }
}

// ---- peripheral clock gates (SVD v2.4, RCC_xxxENR; RCC_xxxRSTR = ENR - 0x58) --------------

using stm32::rcc::ClockSelection, stm32::rcc::Gate, stm32::rcc::KernelClock;
using stm32::rcc::disable, stm32::rcc::enable, stm32::rcc::reset, stm32::rcc::select;

namespace detail {
inline constexpr std::uint16_t ahb3 = 0xD4, ahb1 = 0xD8, ahb2 = 0xDC, ahb4 = 0xE0, apb3 = 0xE4, apb1l = 0xE8,
                               apb1h = 0xEC, apb2 = 0xF0, apb4 = 0xF4;
inline constexpr std::uint16_t enr_to_rstr = 0x58;
[[nodiscard]] constexpr Gate gate(std::uint16_t enr, std::uint8_t bit) noexcept {
    return {base + enr, base + enr - enr_to_rstr, bit};
}
}  // namespace detail

namespace gate {
using detail::ahb2, detail::ahb3, detail::apb1l, detail::apb2, detail::apb4, detail::gate;
inline constexpr Gate sdmmc1 = gate(ahb3, 16);
inline constexpr Gate rng = gate(ahb2, 6), sdmmc2 = gate(ahb2, 9);
inline constexpr Gate tim2 = gate(apb1l, 0), tim3 = gate(apb1l, 1), tim4 = gate(apb1l, 2), tim5 = gate(apb1l, 3),
                      tim6 = gate(apb1l, 4), tim7 = gate(apb1l, 5), tim12 = gate(apb1l, 6), tim13 = gate(apb1l, 7),
                      tim14 = gate(apb1l, 8), usart2 = gate(apb1l, 17), usart3 = gate(apb1l, 18),
                      spi2 = gate(apb1l, 14), spi3 = gate(apb1l, 15),
                      uart4 = gate(apb1l, 19), uart5 = gate(apb1l, 20), i2c1 = gate(apb1l, 21),
                      i2c2 = gate(apb1l, 22), i2c3 = gate(apb1l, 23), uart7 = gate(apb1l, 30),
                      uart8 = gate(apb1l, 31);
inline constexpr Gate tim1 = gate(apb2, 0), tim8 = gate(apb2, 1), usart1 = gate(apb2, 4), usart6 = gate(apb2, 5),
                      spi1 = gate(apb2, 12), spi4 = gate(apb2, 13), tim15 = gate(apb2, 16), tim16 = gate(apb2, 17),
                      tim17 = gate(apb2, 18), spi5 = gate(apb2, 20);
inline constexpr Gate syscfg = gate(apb4, 1), spi6 = gate(apb4, 5), i2c4 = gate(apb4, 7);
}  // namespace gate

using GpioPort = stm32::GpioPort;

/// GPIOx clock: RCC_AHB4ENR bit x.
inline constexpr std::uintptr_t gpio_clock_enr = base + 0x0E0;

}  // namespace rtos::port::stm32h7::rcc
