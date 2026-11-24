// SPDX-License-Identifier: MIT
//
// STM32F103 reset and clock control: oscillators (HSI, HSE), the PLL, peripheral clock gates
// (RM0008 rev 21 chapter 7 "Connectivity line devices" excluded: 7.3 RCC registers of the
// low-, medium-, high- and XL-density devices; bits: ST's stm32f103xb.h).
//
// Unlike the newer STM32 the F1 has no kernel clock multiplexers: every peripheral runs from its
// bus clock (PCLK1, PCLK2), the timers from 2 x PCLK when that bus is divided.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/rcc.hpp"
#include "rtos/port/stm32f1/chip.hpp"

namespace rtos::port::stm32f1::rcc {

inline constexpr std::uintptr_t base = periph_base + 0x0002'1000;

using CR = PReg<0x0002'1000 + 0x00>;       // reset 0x0000'XX83: HSI on
using CFGR = PReg<0x0002'1000 + 0x04>;     // reset 0: SYSCLK = HSI, no prescalers
using CIR = PReg<0x0002'1000 + 0x08>;
using APB2RSTR = PReg<0x0002'1000 + 0x0C>;
using APB1RSTR = PReg<0x0002'1000 + 0x10>;
using AHBENR = PReg<0x0002'1000 + 0x14>;   // reset 0x14: SRAM and FLITF clocks on in Sleep mode
using APB2ENR = PReg<0x0002'1000 + 0x18>;
using APB1ENR = PReg<0x0002'1000 + 0x1C>;
using CSR = PReg<0x0002'1000 + 0x24>;

inline constexpr std::uint32_t cr_hsion = 1u << 0, cr_hsirdy = 1u << 1;
inline constexpr std::uint32_t cr_hseon = 1u << 16, cr_hserdy = 1u << 17, cr_hsebyp = 1u << 18;
inline constexpr std::uint32_t cr_pllon = 1u << 24, cr_pllrdy = 1u << 25;
inline constexpr std::uint32_t cfgr_sw_msk = 3u << 0, cfgr_sws_pos = 2, cfgr_sws_msk = 3u << cfgr_sws_pos;
inline constexpr std::uint32_t sw_hsi = 0, sw_hse = 1, sw_pll = 2;
inline constexpr std::uint32_t cfgr_hpre_pos = 4, cfgr_ppre1_pos = 8, cfgr_ppre2_pos = 11, cfgr_adcpre_pos = 14;
inline constexpr std::uint32_t cfgr_pllsrc = 1u << 16, cfgr_pllxtpre = 1u << 17, cfgr_pllmul_pos = 18,
                               cfgr_usbpre = 1u << 22;
inline constexpr std::uint32_t ahbenr_flitfen = 1u << 4, ahbenr_sramen = 1u << 2;
inline constexpr std::uint32_t csr_lsion = 1u << 0, csr_lsirdy = 1u << 1, csr_rmvf = 1u << 24;
/// Reset flags in RCC_CSR (cleared by RMVF): pin, power-on, software, IWDG, WWDG, low power.
inline constexpr std::uint32_t csr_pinrstf = 1u << 26, csr_porrstf = 1u << 27, csr_sftrstf = 1u << 28,
                               csr_iwdgrstf = 1u << 29, csr_wwdgrstf = 1u << 30, csr_lpwrrstf = 1u << 31;

/// Run SYSCLK from the HSI (the reset state; also the safe state to reconfigure the PLL from).
inline void use_hsi() noexcept {
    CR::set(cr_hsion);
    while ((CR::read() & cr_hsirdy) == 0) {
    }
    CFGR::write((CFGR::read() & ~cfgr_sw_msk) | sw_hsi);
    while ((CFGR::read() & cfgr_sws_msk) != (sw_hsi << cfgr_sws_pos)) {
    }
}

/// Start the HSE: a crystal, or with `bypass` an external clock on OSC_IN. False if it does
/// not become ready within `max_polls` reads (no crystal, oscillator dead): the caller falls
/// back to the HSI. RM0008 7.2.1: HSERDY within 6 HSE cycles after the start-up time.
inline bool enable_hse(bool bypass, std::uint32_t max_polls) noexcept {
    CR::clear(cr_hseon);
    CR::write((CR::read() & ~cr_hsebyp) | (bypass ? cr_hsebyp : 0u));
    CR::set(cr_hseon);
    for (std::uint32_t i = 0; i < max_polls; ++i) {
        if ((CR::read() & cr_hserdy) != 0) {
            return true;
        }
    }
    CR::clear(cr_hseon);
    return false;
}

inline void use_pll() noexcept {
    CFGR::write((CFGR::read() & ~cfgr_sw_msk) | sw_pll);
    while ((CFGR::read() & cfgr_sws_msk) != (sw_pll << cfgr_sws_pos)) {
    }
}

/// The LSI (IWDG clock).
inline void enable_lsi() noexcept {
    CSR::set(csr_lsion);
    while ((CSR::read() & csr_lsirdy) == 0) {
    }
}

/// The reset flags (RCC_CSR bits 26..31), then cleared - why the chip started.
[[nodiscard]] inline std::uint32_t take_reset_flags() noexcept {
    const std::uint32_t flags = CSR::read() & 0xFC00'0000u;
    CSR::set(csr_rmvf);
    return flags;
}

// ---- peripheral clock gates (RCC_APB1ENR/APB2ENR, reset registers APB1RSTR/APB2RSTR) --------

using stm32::rcc::Gate;
using stm32::rcc::disable, stm32::rcc::enable, stm32::rcc::reset;

namespace detail {
inline constexpr std::uintptr_t apb2_enr = base + 0x18, apb2_rstr = base + 0x0C;
inline constexpr std::uintptr_t apb1_enr = base + 0x1C, apb1_rstr = base + 0x10;
[[nodiscard]] constexpr Gate apb1(std::uint8_t bit) noexcept { return {apb1_enr, apb1_rstr, bit}; }
[[nodiscard]] constexpr Gate apb2(std::uint8_t bit) noexcept { return {apb2_enr, apb2_rstr, bit}; }
}  // namespace detail

/// The AHB peripherals (DMA1, CRC) have a clock enable but no reset register on the medium
/// density parts: they have no Gate (rcc::reset() would hit a wrong register).
namespace gate {
using detail::apb1, detail::apb2;
inline constexpr Gate afio = apb2(0), iopa = apb2(2), iopb = apb2(3), iopc = apb2(4), iopd = apb2(5),
                      iope = apb2(6), adc1 = apb2(9), adc2 = apb2(10), tim1 = apb2(11), spi1 = apb2(12),
                      usart1 = apb2(14);
inline constexpr Gate tim2 = apb1(0), tim3 = apb1(1), tim4 = apb1(2), wwdg = apb1(11), spi2 = apb1(14),
                      usart2 = apb1(17), usart3 = apb1(18), i2c1 = apb1(21), i2c2 = apb1(22), usb = apb1(23),
                      can1 = apb1(25), bkp = apb1(27), pwr = apb1(28);
}  // namespace gate

}  // namespace rtos::port::stm32f1::rcc
