// SPDX-License-Identifier: MIT
//
// Compile-time configuration of the STM32H5 chip drivers: the register values the drivers
// write are computed here exactly as on the target (they are constexpr), so their limits
// can be checked against the specifications on the host.
#include <gtest/gtest.h>

#include "rtos/port/stm32h5/clocks.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/rcc.hpp"
#include "rtos/port/stm32h5/usart.hpp"

namespace {

namespace h5 = rtos::port::stm32h5;

struct I2cCase {
    std::uint32_t kernel_hz, bus_hz;
    std::uint32_t low_ns, high_ns, setup_ns;  // UM10204 minimum
};

class I2cTimingTest : public ::testing::TestWithParam<I2cCase> {};

TEST_P(I2cTimingTest, MeetsTheBusSpecification) {
    const I2cCase c = GetParam();
    const h5::I2cTiming t = h5::i2c_timing(c.kernel_hz, c.bus_hz);
    ASSERT_TRUE(t.valid);
    EXPECT_LE(t.presc, 15u);
    EXPECT_LE(t.scldel, 15u);
    EXPECT_LE(t.scll, 255u);
    EXPECT_LE(t.sclh, 255u);
    const double tp_ns = 1e9 / c.kernel_hz * (t.presc + 1);
    EXPECT_GE((t.scll + 1) * tp_ns, c.low_ns);
    EXPECT_GE((t.sclh + 1) * tp_ns, c.high_ns);
    EXPECT_GE((t.scldel + 1) * tp_ns, c.setup_ns);
    const std::uint32_t f = h5::i2c_frequency(t, c.kernel_hz);
    EXPECT_LE(f, c.bus_hz);
    EXPECT_GE(f, c.bus_hz / 10 * 9);
}

INSTANTIATE_TEST_SUITE_P(KernelClocksAndSpeeds, I2cTimingTest,
                         ::testing::Values(I2cCase{250'000'000, 100'000, 4700, 4000, 250},
                                           I2cCase{250'000'000, 400'000, 1300, 600, 100},
                                           I2cCase{250'000'000, 1'000'000, 500, 260, 50},
                                           I2cCase{80'000'000, 100'000, 4700, 4000, 250},
                                           I2cCase{160'000'000, 400'000, 1300, 600, 100},
                                           I2cCase{64'000'000, 100'000, 4700, 4000, 250}));

// The product's CubeMX value for 80 MHz / 100 kHz was 0x10909CEC (PRESC 1, SCLDEL 9, SCLH 156,
// SCLL 236): the computed timing uses the same prescaler and a comparable split.
TEST(I2cTiming, ComparableToTheVendorToolAt80MHz) {
    const h5::I2cTiming t = h5::i2c_timing(80'000'000, 100'000);
    EXPECT_EQ(t.presc, 1u);
    EXPECT_NEAR(static_cast<double>(t.scll), 236.0, 40.0);
    EXPECT_NEAR(static_cast<double>(t.sclh), 156.0, 40.0);
}

// The checks the drivers run at compile time (they would stop the build).
static_assert(h5::checked_i2c_timing<h5::I2cConfig{.kernel_hz = 250'000'000, .cpu_hz = 250'000'000}>().valid);
static_assert(h5::checked_uart_registers<h5::UartConfig{.kernel_hz = 250'000'000, .baud = 115'200}>().brr == 2170);

TEST(UartConfig, RegistersFromTheConfiguration) {
    constexpr h5::UartRegisters r = h5::uart_registers(
        {.kernel_hz = 250'000'000, .baud = 1'000'000, .parity = h5::Parity::Even, .half_duplex = true});
    EXPECT_EQ(r.brr, 250u);
    EXPECT_EQ(r.cr1 & (1u << 10), 1u << 10) << "PCE";
    EXPECT_EQ(r.cr1 & (1u << 12), 1u << 12) << "M0: 9 bits with parity";
    EXPECT_EQ(r.cr1 & (1u << 9), 0u) << "even parity";
    EXPECT_EQ(r.cr3, 1u << 3) << "HDSEL";
    EXPECT_EQ(r.cr1 & 0xDu, 0xDu) << "UE, RE, TE";
}

TEST(UartConfig, BaudRateError) {
    EXPECT_EQ(h5::uart_baud_error({.kernel_hz = 250'000'000, .baud = 115'200}), 0u);
    // 16 MHz / 1 Mbaud: BRR 16, exact; 16 MHz / 921600: BRR 17 -> 941176 baud, 2.1 %
    EXPECT_EQ(h5::uart_baud_error({.kernel_hz = 16'000'000, .baud = 1'000'000}), 0u);
    EXPECT_GT(h5::uart_baud_error({.kernel_hz = 16'000'000, .baud = 921'600}), 200u);
}

TEST(RccConfig, Pll1WithQOutput) {
    constexpr h5::rcc::Pll1Config fcu{.m = 32, .n = 250, .p = 2, .q = 5};
    static_assert(fcu.valid());
    EXPECT_EQ(fcu.output_hz(), 250'000'000u);
    EXPECT_EQ(fcu.q_hz(), 100'000'000u);
    constexpr h5::rcc::Pll1Config bad{.m = 32, .n = 250, .p = 2, .q = 200};
    EXPECT_FALSE(bad.valid());
}

TEST(RccConfig, ClockTreeDerivesFrequenciesAndRegisters) {
    constexpr h5::rcc::ClockTree fcu{.pll1 = {.m = 32, .n = 250, .p = 2, .q = 5}};
    static_assert(fcu.sysclk_hz() == 250'000'000 && fcu.pclk1_hz() == 250'000'000 && fcu.timer1_hz() == 250'000'000);
    EXPECT_EQ(fcu.cfgr2(), 0u);
    EXPECT_EQ(fcu.wait_states(), 5u);
    // the vendor firmware's tree: 160 MHz, APB1/APB2 / 2
    constexpr h5::rcc::ClockTree cube{.pll1 = {.m = 4, .n = 20, .p = 2, .q = 4}, .apb1_div = 2, .apb2_div = 2};
    static_assert(cube.pll1.valid());
    EXPECT_EQ(cube.sysclk_hz(), 160'000'000u);
    EXPECT_EQ(cube.pclk1_hz(), 80'000'000u);
    EXPECT_EQ(cube.timer1_hz(), 160'000'000u) << "timers run at 2 x PCLK when the APB prescaler is not 1";
    EXPECT_EQ(cube.pclk3_hz(), 160'000'000u);
    EXPECT_EQ(cube.pll1_q_hz(), 80'000'000u);
    EXPECT_EQ(cube.cfgr2(), (0x4u << 4) | (0x4u << 8));
    EXPECT_EQ(cube.wait_states(), 3u);
    EXPECT_EQ(h5::rcc::ClockTree::hpre(32), 0xFFu) << "HPRE has no /32";
}

}  // namespace
