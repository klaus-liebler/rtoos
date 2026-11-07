// SPDX-License-Identifier: MIT
//
// Compile-time configuration of the STM32H743 chip drivers: the register values the drivers
// write are computed here exactly as on the target (they are constexpr), so their limits can be
// checked against RM0433 and ST's tables on the host. Expected encodings: ST's stm32h743xx.h
// (RCC fields, prescalers), stm32h7xx_hal_rcc.h / stm32h7xx_ll_utils.c (PLL ranges),
// STM32CubeMX db (wait states, pin database), stm32h7xx_hal_flash_ex.h (programming delay).
#include <gtest/gtest.h>

#include "rtos/port/stm32h7/clocks.hpp"
#include "rtos/port/stm32h7/gpio.hpp"
#include "rtos/port/stm32h7/i2c.hpp"
#include "rtos/port/stm32h7/sdmmc.hpp"
#include "rtos/port/stm32h7/spi.hpp"
#include "rtos/port/stm32h7/timer.hpp"
#include "rtos/port/stm32h7/usart.hpp"

namespace {

namespace h7 = rtos::port::stm32h7;

TEST(H7Rcc, Pll1FromHsiAt400MHz) {
    constexpr h7::rcc::PllConfig pll{.m = 4, .n = 50, .p = 2, .q = 8};
    static_assert(pll.valid());
    EXPECT_EQ(pll.reference_hz(), 16'000'000u);
    EXPECT_EQ(pll.vco_hz(), 800'000'000u);
    EXPECT_EQ(pll.p_hz(), 400'000'000u);
    EXPECT_EQ(pll.q_hz(), 100'000'000u);
    EXPECT_EQ(pll.range(), 3u) << "8-16 MHz reference";
    EXPECT_FALSE(pll.medium_vco());
    // PLLSRC = 00 (HSI), DIVM1 = m (not m - 1: 0 switches the prescaler off)
    EXPECT_EQ(pll.pllckselr(), 4u << 4);
    // PLL1RGE = 11, VCOSEL = 0 (wide), integer mode, DIVP1EN, DIVQ1EN
    EXPECT_EQ(pll.pllcfgr(), (3u << 2) | (1u << 16) | (1u << 17));
    // DIVN1 = n - 1, DIVP1 = p - 1, DIVQ1 = q - 1, DIVR1 = 2 - 1 (R unused)
    EXPECT_EQ(pll.pll1divr(), 49u | (1u << 9) | (7u << 16) | (1u << 24));
}

TEST(H7Rcc, Pll1FromHseAndTheMediumVco) {
    constexpr h7::rcc::PllConfig hse{.source = h7::rcc::PllSource::Hse, .input_hz = 25'000'000, .m = 5, .n = 160,
                                     .p = 2, .q = 8, .r = 4};
    static_assert(hse.valid());
    EXPECT_EQ(hse.range(), 2u) << "5 MHz: 4-8 MHz range";
    EXPECT_EQ(hse.pllckselr(), 2u | (5u << 4)) << "PLLSRC = 10 (HSE)";
    EXPECT_EQ(hse.pllcfgr(), (2u << 2) | (1u << 16) | (1u << 17) | (1u << 18));
    EXPECT_EQ(hse.r_hz(), 200'000'000u);
    // a 1-2 MHz reference needs the medium VCO (ST: IS_LL_UTILS_CHECK_VCO_RANGES)
    constexpr h7::rcc::PllConfig medium{.m = 40, .n = 200, .p = 2};
    static_assert(medium.valid());
    EXPECT_EQ(medium.reference_hz(), 1'600'000u);
    EXPECT_TRUE(medium.medium_vco());
    EXPECT_EQ(medium.pllcfgr() & 0xFu, 1u << 1) << "PLL1RGE = 00, VCOSEL = 1";
}

TEST(H7Rcc, Pll1Limits) {
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 56, .p = 2}.valid())) << "VCO 896 MHz > 836 MHz (wide)";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 11, .p = 2}.valid())) << "VCO 176 MHz < 192 MHz (wide)";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 40, .n = 300, .p = 2}.valid())) << "VCO 480 MHz > 420 MHz (medium)";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 3, .n = 30, .p = 2}.valid())) << "reference 21.3 MHz > 16 MHz";
    EXPECT_TRUE((h7::rcc::PllConfig{.m = 63, .n = 300, .p = 2}.valid())) << "reference 1.02 MHz, medium VCO 305 MHz";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 63, .n = 100, .p = 2}.valid())) << "medium VCO 102 MHz < 150 MHz";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 50, .p = 3}.valid())) << "P even only";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 50, .p = 1}.valid())) << "P >= 2";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 64, .n = 200, .p = 2}.valid())) << "M is 1..63";
    EXPECT_FALSE((h7::rcc::PllConfig{.m = 4, .n = 50, .p = 2, .q = 129}.valid())) << "Q is 1..128";
}

TEST(H7Clocks, TreeDerivesFrequenciesAndPrescalers) {
    constexpr h7::rcc::ClockTree t{.pll1 = {.m = 4, .n = 50, .p = 2, .q = 8}};
    EXPECT_EQ(t.cpu_hz(), 400'000'000u);
    EXPECT_EQ(t.hclk_hz(), 200'000'000u);
    EXPECT_EQ(t.pclk1_hz(), 100'000'000u);
    EXPECT_EQ(t.pclk2_hz(), 100'000'000u);
    EXPECT_EQ(t.pclk3_hz(), 100'000'000u);
    EXPECT_EQ(t.pclk4_hz(), 100'000'000u);
    EXPECT_EQ(t.timer1_hz(), 200'000'000u) << "APB prescaler 2: the timers run at HCLK";
    EXPECT_EQ(t.wait_states(), 2u);
    // HPRE 1000 (/2), D1PPRE 100 (/2), D1CPRE 0000 (/1)
    EXPECT_EQ(t.d1cfgr(), 0x8u | (0x4u << 4));
    EXPECT_EQ(t.d2cfgr(), (0x4u << 4) | (0x4u << 8));
    EXPECT_EQ(t.d3cfgr(), 0x4u << 4);
    constexpr h7::rcc::ClockTree slow{.pll1 = {.m = 4, .n = 50, .p = 2}, .ahb_div = 4, .apb1_div = 4,
                                      .apb2_div = 1, .apb3_div = 1, .apb4_div = 16};
    EXPECT_EQ(slow.hclk_hz(), 100'000'000u);
    EXPECT_EQ(slow.d1cfgr(), 0x9u);
    EXPECT_EQ(slow.d2cfgr(), 0x5u << 4);
    EXPECT_EQ(slow.d3cfgr(), 0x7u << 4);
    EXPECT_EQ(slow.timer1_hz(), 50'000'000u) << "2 x PCLK1 (25 MHz)";
    EXPECT_EQ(slow.timer2_hz(), 100'000'000u) << "prescaler 1: PCLK2";
    EXPECT_EQ(slow.wait_states(), 1u);
}

TEST(H7Flash, WaitStatesMatchCubeMx) {
    // STM32CubeMX db IP/RCC-STM32H7_rcc_v1_0_Modes.xml, FLatency, scale1: 70/140/210/225 MHz
    const std::array<std::uint32_t, 4> limits{70'000'000u, 140'000'000u, 210'000'000u, 225'000'000u};
    for (std::uint32_t ws = 0; ws < limits.size(); ++ws) {
        EXPECT_EQ(h7::flash::vos1_wait_states(limits[ws]), ws) << limits[ws];
        EXPECT_EQ(h7::flash::vos1_wait_states(limits[ws] + 1), ws + 1 < limits.size() ? ws + 1 : 0xFFu) << limits[ws];
    }
    // stm32h7xx_hal_flash_ex.h: FLASH_PROGRAMMING_DELAY_0/1/2 up to 70/185/225 MHz
    EXPECT_EQ(h7::flash::programming_delay(70'000'000), 0u);
    EXPECT_EQ(h7::flash::programming_delay(70'000'001), 1u);
    EXPECT_EQ(h7::flash::programming_delay(185'000'001), 2u);
}

TEST(H7Gpio, PinDatabaseOfTheLqfp100) {
    using h7::gpio::GpioPort, h7::gpio::Signal;
    EXPECT_EQ(h7::gpio::package_pins.size(), 82u) << "LQFP100: 82 I/O pins";
    // the FCU revision 3's pins (pcb_factory_control_unit, U17)
    static_assert(h7::gpio::function_af({GpioPort::D, 8}, Signal::USART3_TX) == 7);
    static_assert(h7::gpio::function_af({GpioPort::D, 9}, Signal::USART3_RX) == 7);
    static_assert(h7::gpio::function_af({GpioPort::B, 11}, Signal::ETH_TX_EN) == 11);
    static_assert(h7::gpio::function_af({GpioPort::B, 12}, Signal::ETH_TXD0) == 11);
    static_assert(h7::gpio::function_af({GpioPort::D, 12}, Signal::I2C4_SCL) == 4);
    static_assert(h7::gpio::function_af({GpioPort::A, 8}, Signal::I2C3_SCL) == 4);
    static_assert(h7::gpio::function_af({GpioPort::E, 5}, Signal::TIM15_CH1) == 4);
    static_assert(h7::gpio::function_af({GpioPort::C, 11}, Signal::QUADSPI_BK2_NCS) == 9);
    static_assert(h7::gpio::function_af({GpioPort::C, 15}, Signal::QUADSPI_BK2_NCS) == -1,
                  "PC15 has no QUADSPI function (the revision 3 schematic had QSPI_CS there)");
    static_assert(h7::gpio::function_af({GpioPort::A, 4}, Signal::ADC1_INP18) == h7::gpio::additional);
    EXPECT_EQ(h7::gpio::parse_pin("PE1"), (h7::gpio::Pin{GpioPort::E, 1}));
    EXPECT_FALSE(h7::gpio::parse_pin("PF0")) << "port F is not bonded out in the LQFP100";
}

TEST(H7Gpio, PinListFoldsIntoOneValuePerPort) {
    using h7::gpio::GpioPort, h7::gpio::Signal;
    constexpr std::array pins{
        h7::gpio::output({GpioPort::E, 1}, true),
        h7::gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX),
        h7::gpio::alternate({GpioPort::D, 9}, Signal::USART3_RX, h7::gpio::Speed::Low, h7::gpio::Pull::Up),
    };
    constexpr auto plans = h7::gpio::plan(pins);
    static_assert(plans.size() == 8);
    EXPECT_FALSE(plans[0].used);
    EXPECT_EQ(plans[3].mode, (2u << 16) | (2u << 18));
    EXPECT_EQ(plans[3].afh, 7u | (7u << 4));
    EXPECT_EQ(plans[3].pull, 1u << 18);
    EXPECT_EQ(plans[4].mode, 1u << 2);
    EXPECT_EQ(plans[4].bsrr, 1u << 1);
}

TEST(H7Usart, InstancesAndConsoleBaudRate) {
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).base, 0x4000'4800u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).irq, 39u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart1).base, 0x4001'1000u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Uart8).irq, 83u);
    // RCC_APB1LENR.USART3EN = bit 18, reset register 0x58 below
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).gate.enr, 0x5802'44E8u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).gate.rstr, 0x5802'4490u);
    EXPECT_EQ(h7::usart_hw(h7::UsartId::Usart3).gate.bit, 18u);
    EXPECT_EQ(h7::uart_brr(100'000'000, 115'200), 868u) << "100 MHz / 115200, rounded";
}

TEST(H7Timer, DmaRegistersSitAt0x48) {
    // SVD v2.4: TIMx_DCR at +0x048, DMAR at +0x04C (the H5 and G4: +0x3DC/+0x3E0)
    using Leds = h7::Timer<h7::TimerId::Tim15, {.kernel_hz = 200'000'000, .update_hz = 800'000}>;
    EXPECT_EQ(Leds::dmar_address(), 0x4001'4000u + 0x4Cu);
    EXPECT_EQ(Leds::compare_address<1>(), 0x4001'4034u);
    EXPECT_EQ(Leds::period_ticks, 250u) << "200 MHz / 800 kHz";
    using Steps = h7::Timer<h7::TimerId::Tim2, {.kernel_hz = 200'000'000, .tick_hz = 1'000'000}>;
    EXPECT_EQ(Steps::arr_address(), 0x4000'002Cu);
    EXPECT_EQ(Steps::max_period, 0xFFFF'FFFFu) << "TIM2 counts 32 bit";
    EXPECT_EQ(h7::timer_hw(h7::TimerId::Tim15).irq, 116u);
    EXPECT_FALSE(h7::timer_hw(h7::TimerId::Tim15).burst_source) << "no TIMx_DCR.DBSS on the H7";
}

TEST(H7Buses, I2cSpiSdmmcInstances) {
    EXPECT_EQ(h7::i2c_hw(h7::I2cId::I2c4).base, 0x5800'1C00u);
    EXPECT_EQ(h7::i2c_hw(h7::I2cId::I2c4).irq_event, 95u);
    EXPECT_EQ(h7::i2c_hw(h7::I2cId::I2c3).irq_error, 73u);
    EXPECT_TRUE((h7::checked_i2c_timing<{.kernel_hz = 100'000'000, .cpu_hz = 400'000'000}>().valid));
    EXPECT_EQ(h7::spi_hw(h7::SpiId::Spi2).fifo_bytes, 16u) << "SPI1-3: 16-byte FIFO (HX711)";
    EXPECT_EQ(h7::spi_hw(h7::SpiId::Spi4).fifo_bytes, 8u);
    EXPECT_EQ(h7::spi_mbr({.kernel_hz = 100'000'000, .max_bus_hz = 1'000'000}), 6u) << "/128 -> 781 kHz";
    EXPECT_EQ(h7::sdmmc_hw(h7::SdmmcId::Sdmmc1).base, 0x5200'7000u);
    EXPECT_EQ(h7::sdmmc_hw(h7::SdmmcId::Sdmmc1).irq, 49u);
    EXPECT_EQ(h7::sdmmc_clkdiv(100'000'000, 400'000), 125u);
}

}  // namespace
