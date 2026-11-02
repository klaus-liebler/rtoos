// SPDX-License-Identifier: MIT
//
// STM32H563 reset and clock control: system clock from HSI or PLL1, peripheral
// clock gates.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/rcc.hpp"
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
inline constexpr std::uint32_t pll1cfgr_qen = 1u << 17;
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
    std::uint32_t q = 0;  ///< 1..128, or 0: Q output off (kernel clock of SPI1-3, FDCAN, USB, SDMMC)

    [[nodiscard]] constexpr std::uint32_t reference_hz() const noexcept { return hsi_hz / m; }
    [[nodiscard]] constexpr std::uint32_t vco_hz() const noexcept { return reference_hz() * n; }
    [[nodiscard]] constexpr std::uint32_t output_hz() const noexcept { return vco_hz() / p; }
    [[nodiscard]] constexpr std::uint32_t q_hz() const noexcept { return q == 0 ? 0 : vco_hz() / q; }
    /// PLL1RGE: input range 1-2 / 2-4 / 4-8 / 8-16 MHz.
    [[nodiscard]] constexpr std::uint32_t input_range() const noexcept {
        const std::uint32_t ref = reference_hz();
        return ref < 2'000'000 ? 0 : (ref < 4'000'000 ? 1 : (ref < 8'000'000 ? 2 : 3));
    }
    /// Wide VCO range (PLL1VCOSEL = 0): 192..836 MHz, reference >= 2 MHz.
    [[nodiscard]] constexpr bool valid() const noexcept {
        return m >= 1 && m <= 63 && n >= 4 && n <= 512 && p >= 2 && p <= 128 && p % 2 == 0 && q <= 128 &&
               reference_hz() >= 2'000'000 && reference_hz() <= 16'000'000 && vco_hz() >= 192'000'000 &&
               vco_hz() <= 836'000'000;
    }
};

/// Run SYSCLK from PLL1 (P output). HSI must be running with HSIDIV = 1; voltage scale,
/// flash latency and bus prescalers must already allow the target frequency (clocks.hpp:
/// configure_clocks() does all of it).
template <Pll1Config Cfg>
inline void use_pll1() noexcept {
    static_assert(Cfg.valid(), "PLL1 configuration outside the RM0481 limits");
    CR::clear(cr_pll1on);
    while ((CR::read() & cr_pll1rdy) != 0) {
    }
    PLL1CFGR::write(pll1cfgr_src_hsi | (Cfg.input_range() << pll1cfgr_rge_pos) | (Cfg.m << pll1cfgr_m_pos) |
                    pll1cfgr_pen | (Cfg.q != 0 ? pll1cfgr_qen : 0u));  // VCOSEL = 0 (wide), FRACEN = 0, R off
    const std::uint32_t q = Cfg.q != 0 ? Cfg.q : 2u;
    PLL1DIVR::write((Cfg.n - 1u) | ((Cfg.p - 1u) << pll1divr_p_pos) | ((q - 1u) << 16) | (1u << 24));  // R = 2 (off)
    CR::set(cr_pll1on);
    while ((CR::read() & cr_pll1rdy) == 0) {
    }
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

// ---- peripheral clock gates (SVD v1.8, RCC_xxxENR; RCC_xxxRSTR = ENR - 0x28) --------------

using stm32::rcc::ClockSelection, stm32::rcc::Gate, stm32::rcc::KernelClock;
using stm32::rcc::disable, stm32::rcc::enable, stm32::rcc::reset, stm32::rcc::select;

namespace detail {
inline constexpr std::uintptr_t base = periph_alias + 0x0402'0C00;
inline constexpr std::uint16_t ahb1 = 0x88, ahb2 = 0x8C, ahb4 = 0x94, apb1l = 0x9C, apb1h = 0xA0, apb2 = 0xA4, apb3 = 0xA8;
inline constexpr std::uint16_t enr_to_rstr = 0x28;
[[nodiscard]] constexpr Gate gate(std::uint16_t enr, std::uint8_t bit) noexcept {
    return {base + enr, base + enr - enr_to_rstr, bit};
}
}  // namespace detail

namespace gate {
using detail::ahb1, detail::ahb2, detail::ahb4, detail::apb1l, detail::apb1h, detail::apb2, detail::apb3, detail::gate;
inline constexpr Gate gpdma1 = gate(ahb1, 0), gpdma2 = gate(ahb1, 1), crc = gate(ahb1, 12);
inline constexpr Gate adc12 = gate(ahb2, 10), rng = gate(ahb2, 18);
inline constexpr Gate sdmmc1 = gate(ahb4, 11), sdmmc2 = gate(ahb4, 12);
inline constexpr Gate tim2 = gate(apb1l, 0), tim3 = gate(apb1l, 1), tim4 = gate(apb1l, 2), tim5 = gate(apb1l, 3),
                      tim6 = gate(apb1l, 4), tim7 = gate(apb1l, 5);
inline constexpr Gate crs = gate(apb1l, 24);
inline constexpr Gate spi2 = gate(apb1l, 14), spi3 = gate(apb1l, 15), usart2 = gate(apb1l, 17),
                      usart3 = gate(apb1l, 18), uart4 = gate(apb1l, 19), uart5 = gate(apb1l, 20),
                      i2c1 = gate(apb1l, 21), i2c2 = gate(apb1l, 22), uart7 = gate(apb1l, 30), uart8 = gate(apb1l, 31);
inline constexpr Gate uart9 = gate(apb1h, 0), uart12 = gate(apb1h, 1), dts = gate(apb1h, 3), fdcan12 = gate(apb1h, 9),
                      ucpd1 = gate(apb1h, 23);
inline constexpr Gate tim1 = gate(apb2, 11), spi1 = gate(apb2, 12), tim8 = gate(apb2, 13), usart1 = gate(apb2, 14),
                      tim15 = gate(apb2, 16), tim16 = gate(apb2, 17), tim17 = gate(apb2, 18), spi4 = gate(apb2, 19),
                      usbfs = gate(apb2, 24);
inline constexpr Gate sbs = gate(apb3, 1), i2c3 = gate(apb3, 7), i2c4 = gate(apb3, 8);
}  // namespace gate

// ---- kernel clock selection (RCC_CCIPRx) ---------------------------------------------------

namespace kernel_clock {
inline constexpr KernelClock spi2{detail::base + 0xE0, 3, 3};       ///< 0: pll1_q (reset), 4: per_ck
inline constexpr KernelClock usbfs{detail::base + 0xE4, 4, 2};      ///< 0: none (reset), 1: pll1_q, 3: hsi48
inline constexpr KernelClock adcdac{detail::base + 0xE8, 0, 3};     ///< 0: hclk (reset), 1: sys_ck, 4: hsi_ker
inline constexpr KernelClock fdcan12{detail::base + 0xE8, 8, 2};    ///< 0: hse (reset), 1: pll1_q
inline constexpr std::uint32_t usbfs_hsi48 = 3, fdcan_pll1_q = 1;
}  // namespace kernel_clock

using GpioPort = stm32::GpioPort;

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
