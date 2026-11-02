// SPDX-License-Identifier: MIT
//
// Compile-time configuration of the STM32G431 chip drivers: the register values the drivers
// write are computed here exactly as on the target (they are constexpr), so their limits can be
// checked against RM0440 and ST's tables on the host. Expected encodings: ST's
// stm32g4xx_ll_rcc.h (PLL dividers), stm32g4xx_ll_utils.c (wait states), CubeMX pin database
// (alternate functions, packages), stm32g4xx_ll_dmamux.h / stm32g431xx.h (DMA).
#include <gtest/gtest.h>

#include <algorithm>

#include "rtos/port/stm32g4/adc.hpp"
#include "rtos/port/stm32g4/clocks.hpp"
#include "rtos/port/stm32g4/dma.hpp"
#include "rtos/port/stm32g4/fdcan.hpp"
#include "rtos/port/stm32g4/gpio.hpp"
#include "rtos/port/stm32g4/i2c.hpp"
#include "rtos/port/stm32g4/spi.hpp"
#include "rtos/port/stm32g4/timer.hpp"
#include "rtos/port/stm32g4/ucpd.hpp"
#include "rtos/port/stm32g4/usart.hpp"

namespace {

namespace g4 = rtos::port::stm32g4;

TEST(G4Rcc, PllFromHsi16At170MHz) {
    constexpr g4::rcc::PllConfig pll{.m = 4, .n = 85, .r = 2};
    static_assert(pll.valid());
    EXPECT_EQ(pll.reference_hz(), 4'000'000u);
    EXPECT_EQ(pll.vco_hz(), 340'000'000u);
    EXPECT_EQ(pll.output_hz(), 170'000'000u);
    // PLLSRC = 10 (HSI16), PLLM = m - 1, PLLN = n, PLLREN, PLLR = 00 (/2)
    EXPECT_EQ(pll.pllcfgr(), 2u | (3u << 4) | (85u << 8) | (1u << 24));
}

TEST(G4Rcc, PllDividerEncodings) {
    // HSE 8 MHz / 2 x 85: R /2, Q /8 (PLLQ = 11), P /8 (PLLPDIV = 8)
    constexpr g4::rcc::PllConfig pll{.source = g4::rcc::PllSource::Hse, .input_hz = 8'000'000,
                                     .m = 2, .n = 85, .r = 2, .q = 8, .p = 8};
    static_assert(pll.valid());
    EXPECT_EQ(pll.q_hz(), 42'500'000u);
    EXPECT_EQ(pll.pllcfgr(), 3u | (1u << 4) | (85u << 8) | (1u << 24) | (1u << 20) | (3u << 21) | (1u << 16) |
                                 (8u << 27));
    constexpr g4::rcc::PllConfig r6{.m = 4, .n = 85, .r = 6, .q = 4};
    EXPECT_EQ((r6.pllcfgr() >> 25) & 3u, 2u) << "PLLR 10 = /6";
    EXPECT_EQ((r6.pllcfgr() >> 21) & 3u, 1u) << "PLLQ 01 = /4";
}

TEST(G4Rcc, PllLimits) {
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 8, .n = 85, .r = 2}.valid())) << "reference 2 MHz < 2.66 MHz";
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 1, .n = 30, .r = 2}.valid())) << "VCO 480 MHz > 344 MHz";
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 4, .n = 20, .r = 2}.valid())) << "VCO 80 MHz < 96 MHz";
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 4, .n = 86, .r = 2}.valid())) << "SYSCLK 172 MHz > 170 MHz";
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 4, .n = 85, .r = 3}.valid())) << "R is 2, 4, 6 or 8";
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 4, .n = 85, .r = 2, .p = 1}.valid())) << "P is 2..31";
    EXPECT_FALSE((g4::rcc::PllConfig{.m = 17, .n = 85, .r = 2}.valid())) << "M is 1..16";
}

TEST(G4Clocks, TreeDerivesFrequenciesWaitStatesAndBoost) {
    constexpr g4::rcc::ClockTree full{.pll = {.m = 4, .n = 85, .r = 2}};
    static_assert(full.sysclk_hz() == 170'000'000 && full.boost());
    EXPECT_EQ(full.wait_states(), 4u);
    EXPECT_EQ(full.cfgr(), 0u);
    EXPECT_EQ(full.ucpd_hz(), 16'000'000u);
    // 150 MHz: normal range 1 (no boost), 30 MHz per wait state -> 4
    constexpr g4::rcc::ClockTree normal{.pll = {.m = 4, .n = 75, .r = 2}, .apb1_div = 2};
    static_assert(normal.sysclk_hz() == 150'000'000 && !normal.boost());
    EXPECT_EQ(normal.wait_states(), 4u);
    EXPECT_EQ(normal.pclk1_hz(), 75'000'000u);
    EXPECT_EQ(normal.timer1_hz(), 150'000'000u) << "timers run at 2 x PCLK when the APB prescaler is not 1";
    EXPECT_EQ(normal.cfgr(), 0x4u << 8) << "PPRE1 = 100 (/2)";
    constexpr g4::rcc::ClockTree slow{.pll = {.m = 4, .n = 85, .r = 2}, .ahb_div = 4, .apb2_div = 16};
    EXPECT_EQ(slow.hclk_hz(), 42'500'000u);
    EXPECT_EQ(slow.cfgr(), (0x9u << 4) | (0x7u << 11));
    EXPECT_EQ(slow.wait_states(), 1u) << "boost: 34 MHz per wait state";
}

TEST(G4Flash, WaitStatesMatchSTsTables) {
    // stm32g4xx_ll_utils.c UTILS_SCALE1_LATENCYx_BOOST_FREQ / UTILS_SCALE1_LATENCYx_FREQ
    for (const std::uint32_t limit : {34'000'000u, 68'000'000u, 102'000'000u, 136'000'000u, 170'000'000u}) {
        const std::uint32_t ws = limit / 34'000'000u - 1;
        EXPECT_EQ(g4::flash::range1_wait_states(limit, true), ws) << limit;
        EXPECT_EQ(g4::flash::range1_wait_states(limit + 1, true), ws + 1) << limit;
    }
    for (const std::uint32_t limit : {30'000'000u, 60'000'000u, 90'000'000u, 120'000'000u, 150'000'000u}) {
        const std::uint32_t ws = limit / 30'000'000u - 1;
        EXPECT_EQ(g4::flash::range1_wait_states(limit, false), ws) << limit;
        EXPECT_EQ(g4::flash::range1_wait_states(limit + 1, false), ws + 1) << limit;
    }
}

TEST(G4Gpio, PinDatabaseOfThePackage) {
    using g4::gpio::GpioPort, g4::gpio::Signal;
    static_assert(g4::gpio::package == g4::gpio::Package::UFQFPN48);  // the tests' build setting
    static_assert(g4::gpio::function_af({GpioPort::C, 4}, Signal::USART1_TX) == 7);
    static_assert(g4::gpio::function_af({GpioPort::B, 7}, Signal::USART1_RX) == 7);
    static_assert(g4::gpio::function_af({GpioPort::A, 8}, Signal::I2C2_SDA) == 4);
    static_assert(g4::gpio::function_af({GpioPort::A, 9}, Signal::I2C2_SCL) == 4);
    static_assert(g4::gpio::function_af({GpioPort::B, 6}, Signal::UCPD1_CC1) == g4::gpio::additional);
    static_assert(g4::gpio::function_af({GpioPort::B, 4}, Signal::UCPD1_CC2) == g4::gpio::additional);
    static_assert(g4::gpio::function_af({GpioPort::A, 12}, Signal::USB_DP) == g4::gpio::additional);
    EXPECT_EQ(g4::gpio::package_pins.size(), 42u) << "UFQFPN48: 42 I/O pins";
    constexpr auto rx = g4::gpio::alternate({GpioPort::B, 7}, Signal::USART1_RX);
    EXPECT_EQ(rx.af, 7u);
}

TEST(G4Gpio, PackagesDifferInPinsAndFunctions) {
    using g4::gpio::GpioPort, g4::gpio::Package, g4::gpio::Signal;
    constexpr auto lqfp48_pins = g4::gpio::detail::pins_of<Package::LQFP48>();
    constexpr auto ufqfpn48_pins = g4::gpio::detail::pins_of<Package::UFQFPN48>();
    const auto has = [](const auto& pins, GpioPort port, std::uint8_t number) {
        return std::ranges::find(pins, std::pair{port, number}) != pins.end();
    };
    EXPECT_EQ(lqfp48_pins.size(), 38u);
    for (const std::uint8_t n : {4, 6, 10, 11}) {  // CubeMX: C(6-8-B)Tx lacks PC4, PC6, PC10, PC11
        EXPECT_FALSE(has(lqfp48_pins, GpioPort::C, n)) << "PC" << int{n};
        EXPECT_TRUE(has(ufqfpn48_pins, GpioPort::C, n)) << "PC" << int{n};
    }
    EXPECT_EQ((g4::gpio::detail::pins_of<Package::LQFP100>().size()), 86u);
    EXPECT_EQ((g4::gpio::detail::pins_of<Package::LQFP32>().size()), 26u);
    // a function the smaller package does not bring out on a pin it has
    constexpr auto lqfp48 = g4::gpio::detail::functions_of<Package::LQFP48>();
    constexpr auto lqfp100 = g4::gpio::detail::functions_of<Package::LQFP100>();
    EXPECT_EQ(rtos::port::stm32::gpio::function_af(lqfp48, {GpioPort::B, 7}, Signal::UART4_CTS), -1);
    EXPECT_GE(rtos::port::stm32::gpio::function_af(lqfp100, {GpioPort::B, 7}, Signal::UART4_CTS), 0);
    // g4::gpio::alternate({GpioPort::F, 3}, ...) - a pin no G431 package has - does not compile
}

TEST(G4Gpio, PinListFoldsIntoOneValuePerPort) {
    using g4::gpio::GpioPort, g4::gpio::Signal;
    constexpr std::array pins{
        g4::gpio::output({GpioPort::C, 6}, true),
        g4::gpio::alternate({GpioPort::C, 4}, Signal::USART1_TX),
        g4::gpio::analog({GpioPort::B, 6}, Signal::UCPD1_CC1),
    };
    constexpr auto plans = g4::gpio::plan(pins);
    static_assert(plans.size() == 7);
    EXPECT_FALSE(plans[0].used);
    EXPECT_EQ(plans[1].mode, 3u << 12);
    EXPECT_EQ(plans[2].mode, (1u << 12) | (2u << 8));
    EXPECT_EQ(plans[2].afl, 7u << 16);
    EXPECT_EQ(plans[2].bsrr, 1u << 6);
}

TEST(G4Dma, ChannelRegistersAndRequests) {
    EXPECT_EQ(g4::dmamux_channel_offset(1, 1), 0x0002'0800u);
    EXPECT_EQ(g4::dmamux_channel_offset(1, 6), 0x0002'0814u);
    EXPECT_EQ(g4::dmamux_channel_offset(2, 1), 0x0002'0820u) << "stm32g431xx.h DMAMUX1_Channel6_BASE";
    EXPECT_EQ(g4::dmamux_channel_offset(2, 6), 0x0002'0834u);
    constexpr g4::DmaConfig to_timer{.channel = 3, .request = g4::DmaRequest::Tim16Ch1,
                                     .direction = g4::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                                     .peripheral_bytes = 4, .priority = 2};
    // DIR, MINC, PSIZE 10 (32 bit), MSIZE 01 (16 bit), PL 10
    EXPECT_EQ(g4::dma_ccr(to_timer), (1u << 4) | (1u << 7) | (2u << 8) | (1u << 10) | (2u << 12));
    EXPECT_EQ(static_cast<unsigned>(g4::DmaRequest::Ucpd1Rx), 0x72u);
    using Ch1 = g4::DmaChannel<{.channel = 1, .request = g4::DmaRequest::Ucpd1Rx,
                                .direction = g4::DmaDirection::PeripheralToMemory}>;
    using Dma2Ch5 = g4::DmaChannel<{.controller = 2, .channel = 5, .request = g4::DmaRequest::Ucpd1Tx,
                                    .direction = g4::DmaDirection::MemoryToPeripheral}>;
    using Dma2Ch6 = g4::DmaChannel<{.controller = 2, .channel = 6, .request = g4::DmaRequest::Ucpd1Tx,
                                    .direction = g4::DmaDirection::MemoryToPeripheral}>;
    EXPECT_EQ(Ch1::irq, 11u);
    EXPECT_EQ(Dma2Ch5::irq, 60u);
    EXPECT_EQ(Dma2Ch6::irq, 97u);
}

TEST(G4Spi, PrescalerAndControlRegisters) {
    constexpr g4::SpiConfig c{.kernel_hz = 170'000'000, .max_bus_hz = 1'000'000, .mode = 3};
    EXPECT_EQ(g4::spi_br(c), 7u) << "/256 -> 664 kHz";
    EXPECT_LE(g4::spi_bus_hz(c), 1'000'000u);
    EXPECT_EQ(g4::spi_br({.kernel_hz = 170'000'000, .max_bus_hz = 100'000}), 8u) << "below /256: rejected";
    EXPECT_EQ(g4::spi_br({.kernel_hz = 42'500'000, .max_bus_hz = 10'000'000}), 2u) << "/8 -> 5.3 MHz";
    // MSTR, SSM, SSI, BR, CPHA, CPOL
    EXPECT_EQ(g4::spi_cr1(c), (1u << 2) | (1u << 9) | (1u << 8) | (7u << 3) | 3u);
    // DS = 0111 (8 bit), FRXTH
    EXPECT_EQ(g4::spi_cr2(c), (7u << 8) | (1u << 12));
}

TEST(G4Peripherals, SharedDriversAtTheG431Clocks) {
    constexpr g4::rcc::ClockTree clocks{.pll = {.m = 4, .n = 85, .r = 2}};
    const g4::I2cTiming i2c = g4::i2c_timing(clocks.pclk1_hz(), 400'000);
    EXPECT_TRUE(i2c.valid);
    EXPECT_LE(g4::i2c_frequency(i2c, clocks.pclk1_hz()), 400'000u);
    EXPECT_LE(g4::uart_baud_error({.kernel_hz = clocks.pclk2_hz(), .baud = 1'000'000}), 200u);
    const g4::UcpdTiming pd = g4::ucpd_timing({.kernel_hz = clocks.ucpd_hz()});
    EXPECT_TRUE(pd.valid) << "HSI16 / 2 = 8 MHz ucpd_clk";
    EXPECT_EQ(pd.ucpd_hz, 8'000'000u);
    const g4::CanBitTiming can = g4::can_bit_timing(clocks.pclk1_hz(), 500'000, 875);
    EXPECT_TRUE(can.valid);
    EXPECT_EQ(170'000'000u % (500'000u * can.quanta()), 0u);
    EXPECT_EQ(g4::adc_clock_hz({.kernel_hz = clocks.sysclk_hz(), .cpu_hz = clocks.sysclk_hz(), .channel = 12}),
              170'000'000u / 6) << "smallest prescaler at or below 40 MHz: /6";
    EXPECT_TRUE((g4::timer_registers({.kernel_hz = clocks.timer1_hz(), .update_hz = 20'000, .min_steps = 1000},
                                     0xFFFF).valid));
}

}  // namespace
