// SPDX-License-Identifier: MIT
//
// STM32H563 reset and clock control: system clock from HSI or PLL1, peripheral
// clock gates.
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"

namespace rtos::port::stm32h5::rcc {

using CR = PReg<0x0402'0C00 + 0x00>;
using CFGR1 = PReg<0x0402'0C00 + 0x1C>;
using CFGR2 = PReg<0x0402'0C00 + 0x20>;
using PLL1CFGR = PReg<0x0402'0C00 + 0x28>;
using PLL1DIVR = PReg<0x0402'0C00 + 0x34>;
using AHB1ENR = PReg<0x0402'0C00 + 0x88>;
using AHB2ENR = PReg<0x0402'0C00 + 0x8C>;
using APB1LENR = PReg<0x0402'0C00 + 0x9C>;
using APB2ENR = PReg<0x0402'0C00 + 0xA4>;
using APB3ENR = PReg<0x0402'0C00 + 0xA8>;

inline constexpr std::uint32_t cr_hsion = 1u << 0;
inline constexpr std::uint32_t cr_hsirdy = 1u << 1;
inline constexpr std::uint32_t cr_hsidiv_pos = 3;
inline constexpr std::uint32_t cr_hsidiv_msk = 3u << cr_hsidiv_pos;
inline constexpr std::uint32_t cr_hsidivf = 1u << 5;
inline constexpr std::uint32_t cr_hsi48on = 1u << 12;
inline constexpr std::uint32_t cr_hsi48rdy = 1u << 13;
inline constexpr std::uint32_t cr_pll1on = 1u << 24;
inline constexpr std::uint32_t cr_pll1rdy = 1u << 25;
inline constexpr std::uint32_t cfgr1_sw_msk = 3u << 0;
inline constexpr std::uint32_t cfgr1_sws_pos = 3;
inline constexpr std::uint32_t cfgr1_sws_msk = 3u << cfgr1_sws_pos;
inline constexpr std::uint32_t cfgr1_sw_pll1 = 3;
inline constexpr std::uint32_t pll1cfgr_src_hsi = 1u << 0;
inline constexpr std::uint32_t pll1cfgr_rge_pos = 2;
inline constexpr std::uint32_t pll1cfgr_m_pos = 8;
inline constexpr std::uint32_t pll1cfgr_pen = 1u << 16;
inline constexpr std::uint32_t pll1divr_p_pos = 9;

enum class HsiDiv : std::uint32_t { Div1 = 0, Div2 = 1, Div4 = 2, Div8 = 3 };

[[nodiscard]] constexpr std::uint32_t hsi_frequency(HsiDiv div) noexcept {
    return hsi_hz >> static_cast<std::uint32_t>(div);
}

/// Run SYSCLK from HSI with the given divider (the reset value of HSIDIV differs
/// between RM0481 and the SVD, so the board sets it explicitly).
inline void use_hsi(HsiDiv div) noexcept {
    CR::set(cr_hsion);
    while ((CR::read() & cr_hsirdy) == 0) {
    }
    CR::write((CR::read() & ~cr_hsidiv_msk) | (static_cast<std::uint32_t>(div) << cr_hsidiv_pos));
    while ((CR::read() & cr_hsidivf) == 0) {  // new divider applied
    }
    CFGR1::clear(cfgr1_sw_msk);  // SW = 000: HSI
    while ((CFGR1::read() & cfgr1_sws_msk) != 0) {
    }
}

/// PLL1 from HSI (integer mode): f_vco = HSI / m * n, SYSCLK = f_vco / p.
struct Pll1Config {
    std::uint32_t m;  ///< 1..63; reference HSI / m must be 1..16 MHz
    std::uint32_t n;  ///< 4..512
    std::uint32_t p;  ///< 2..128 (even)

    [[nodiscard]] constexpr std::uint32_t reference_hz() const noexcept { return hsi_hz / m; }
    [[nodiscard]] constexpr std::uint32_t vco_hz() const noexcept { return reference_hz() * n; }
    [[nodiscard]] constexpr std::uint32_t output_hz() const noexcept { return vco_hz() / p; }
    /// PLL1RGE: input range 1-2 / 2-4 / 4-8 / 8-16 MHz.
    [[nodiscard]] constexpr std::uint32_t input_range() const noexcept {
        const std::uint32_t ref = reference_hz();
        return ref < 2'000'000 ? 0 : (ref < 4'000'000 ? 1 : (ref < 8'000'000 ? 2 : 3));
    }
    /// Wide VCO range (PLL1VCOSEL = 0): 192..836 MHz, reference >= 2 MHz.
    [[nodiscard]] constexpr bool valid() const noexcept {
        return m >= 1 && m <= 63 && n >= 4 && n <= 512 && p >= 2 && p <= 128 && p % 2 == 0 &&
               reference_hz() >= 2'000'000 && reference_hz() <= 16'000'000 && vco_hz() >= 192'000'000 &&
               vco_hz() <= 836'000'000;
    }
};

/// Run SYSCLK from PLL1 (P output). HSI must be running with HSIDIV = 1; voltage scale
/// and flash latency must already allow the target frequency. AHB/APB prescalers stay /1.
template <Pll1Config Cfg>
inline void use_pll1() noexcept {
    static_assert(Cfg.valid(), "PLL1 configuration outside the RM0481 limits");
    CR::clear(cr_pll1on);
    while ((CR::read() & cr_pll1rdy) != 0) {
    }
    PLL1CFGR::write(pll1cfgr_src_hsi | (Cfg.input_range() << pll1cfgr_rge_pos) | (Cfg.m << pll1cfgr_m_pos) |
                    pll1cfgr_pen);  // VCOSEL = 0 (wide), FRACEN = 0, Q/R outputs off
    PLL1DIVR::write((Cfg.n - 1u) | ((Cfg.p - 1u) << pll1divr_p_pos) | (1u << 16) | (1u << 24));  // Q = R = 2 (unused)
    CR::set(cr_pll1on);
    while ((CR::read() & cr_pll1rdy) == 0) {
    }
    CFGR2::write(0);  // HPRE = PPRE1/2/3 = 1
    CFGR1::write((CFGR1::read() & ~cfgr1_sw_msk) | cfgr1_sw_pll1);
    while ((CFGR1::read() & cfgr1_sws_msk) != (cfgr1_sw_pll1 << cfgr1_sws_pos)) {
    }
}

/// 48 MHz internal oscillator (clock of the RNG, RCC_CCIPR5.RNGSEL reset value).
inline void enable_hsi48() noexcept {
    CR::set(cr_hsi48on);
    while ((CR::read() & cr_hsi48rdy) == 0) {
    }
}

enum class GpioPort : std::uint8_t { A, B, C, D, E, F, G, H, I };

inline void enable(GpioPort port) noexcept {
    AHB2ENR::set(1u << static_cast<std::uint32_t>(port));
    (void)AHB2ENR::read();  // RM0481: wait 2 cycles after enabling a clock
}

inline constexpr std::uint32_t ahb1_eth = 1u << 19;
inline constexpr std::uint32_t ahb1_ethtx = 1u << 20;
inline constexpr std::uint32_t ahb1_ethrx = 1u << 21;
inline constexpr std::uint32_t ahb2_rng = 1u << 18;
inline constexpr std::uint32_t apb3_sbs = 1u << 1;
inline constexpr std::uint32_t apb2_usart1 = 1u << 14;
inline constexpr std::uint32_t apb1l_usart2 = 1u << 17;
inline constexpr std::uint32_t apb1l_usart3 = 1u << 18;

}  // namespace rtos::port::stm32h5::rcc
