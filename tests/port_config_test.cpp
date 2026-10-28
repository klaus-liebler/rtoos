// SPDX-License-Identifier: MIT
//
// Compile-time configuration of the STM32H5 chip drivers: the register values the drivers
// write are computed here exactly as on the target (they are constexpr), so their limits
// can be checked against the specifications on the host.
#include <gtest/gtest.h>

#include "rtos/port/stm32h5/adc.hpp"
#include "rtos/port/stm32h5/clocks.hpp"
#include "rtos/port/stm32h5/dts.hpp"
#include "rtos/port/stm32h5/gpio.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/rcc.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
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

TEST(GpioConfig, PinListFoldsIntoOneValuePerPortAndRegister) {
    namespace gpio = h5::gpio;
    using gpio::GpioPort;
    constexpr std::array pins{
        gpio::output({GpioPort::H, 1}, true),
        gpio::alternate({GpioPort::D, 8}, 7),
        gpio::alternate({GpioPort::D, 9}, 7, gpio::Speed::Low, gpio::Pull::Up),
        gpio::alternate({GpioPort::C, 10}, 11, gpio::Speed::VeryHigh),
        gpio::output({GpioPort::A, 0}, false, gpio::Speed::Low, true),
        gpio::analog({GpioPort::A, 3}),
    };
    constexpr auto plans = gpio::plan(pins);
    const auto& a = plans[0];
    const auto& c = plans[2];
    const auto& d = plans[3];
    const auto& h = plans[7];
    EXPECT_FALSE(plans[1].used) << "port B untouched";
    EXPECT_EQ(h.mode_mask, 3u << 2);
    EXPECT_EQ(h.mode, 1u << 2);
    EXPECT_EQ(h.bsrr, 1u << 1) << "LED starts high";
    EXPECT_EQ(d.mode, (2u << 16) | (2u << 18));
    EXPECT_EQ(d.afh_mask, 0xFFu);
    EXPECT_EQ(d.afh, 0x77u);
    EXPECT_EQ(d.pull, 1u << 18);
    EXPECT_EQ(d.speed, 2u << 16) << "TX High, RX Low";
    EXPECT_EQ(c.afh, 11u << 8);
    EXPECT_EQ(c.speed, 3u << 20);
    EXPECT_EQ(a.bsrr, 1u << 16) << "PA0 starts low";
    EXPECT_EQ(a.type, 1u) << "PA0 open drain";
    EXPECT_EQ(a.mode, (1u << 0) | (3u << 6));
    EXPECT_EQ(a.afl_mask, 0u) << "no alternate function on port A";
}

TEST(SpiConfig, PrescalerIsTheSmallestWithinTheBusLimit) {
    // HX711 behind SPI2 at PLL1 Q = 100 MHz, at most 1 MHz: / 128 -> 781 kHz
    EXPECT_EQ(h5::spi_mbr({.kernel_hz = 100'000'000, .max_bus_hz = 1'000'000}), 6u);
    EXPECT_EQ(h5::spi_bus_hz({.kernel_hz = 100'000'000, .max_bus_hz = 1'000'000}), 781'250u);
    EXPECT_EQ(h5::spi_mbr({.kernel_hz = 100'000'000, .max_bus_hz = 50'000'000}), 0u) << "/2";
    EXPECT_EQ(h5::spi_mbr({.kernel_hz = 100'000'000, .max_bus_hz = 49'999'999}), 1u) << "/4";
    EXPECT_EQ(h5::spi_mbr({.kernel_hz = 250'000'000, .max_bus_hz = 100'000}), 8u) << "below / 256: invalid";
}

TEST(TimerConfig, PwmGetsTheFinestPrescalerThatFits) {
    // 20 kHz at 250 MHz: 12500 ticks without prescaler
    const auto r = h5::timer_registers({.kernel_hz = 250'000'000, .update_hz = 20'000, .min_steps = 1000}, 0xFFFF);
    ASSERT_TRUE(r.valid);
    EXPECT_EQ(r.psc, 0u);
    EXPECT_EQ(r.arr, 12'499u);
    // 50 Hz needs a prescaler on a 16-bit counter: 5'000'000 ticks / 77 -> 64935 per period
    const auto slow = h5::timer_registers({.kernel_hz = 250'000'000, .update_hz = 50}, 0xFFFF);
    ASSERT_TRUE(slow.valid);
    EXPECT_LE(slow.arr, 0xFFFFu);
    EXPECT_NEAR(static_cast<double>(h5::timer_update_mhz(slow, 250'000'000)), 50'000.0, 500.0);
    EXPECT_EQ(h5::timer_registers({.kernel_hz = 250'000'000, .update_hz = 50}, 0xFFFF'FFFF).psc, 0u) << "32 bit: none";
}

TEST(TimerConfig, TickRateMustDivideTheKernelClock) {
    EXPECT_TRUE(h5::timer_registers({.kernel_hz = 250'000'000, .tick_hz = 1'000'000}, 0xFFFF).valid);
    EXPECT_EQ(h5::timer_registers({.kernel_hz = 250'000'000, .tick_hz = 1'000'000}, 0xFFFF).psc, 249u);
    EXPECT_FALSE(h5::timer_registers({.kernel_hz = 250'000'000, .tick_hz = 3'000'000}, 0xFFFF).valid);
    EXPECT_FALSE(h5::timer_registers({.kernel_hz = 250'000'000, .tick_hz = 1'000}, 0xFFFF).valid) << "PSC > 65535";
    EXPECT_FALSE(h5::timer_registers({.kernel_hz = 250'000'000, .update_hz = 20'000'000, .min_steps = 100}, 0xFFFF).valid)
        << "too few steps";
}

TEST(AdcConfig, PrescalerKeepsTheAdcClockWithinItsLimit) {
    // HCLK 250 MHz, at most 40 MHz: / 8 -> 31.25 MHz (/ 6 would give 41.7 MHz)
    EXPECT_EQ(h5::adc_presc({.kernel_hz = 250'000'000, .cpu_hz = 250'000'000, .channel = 18}), 4u);
    EXPECT_EQ(h5::adc_clock_hz({.kernel_hz = 250'000'000, .cpu_hz = 250'000'000, .channel = 18}), 31'250'000u);
    // the vendor firmware: HCLK 160 MHz / 4 = 40 MHz
    EXPECT_EQ(h5::adc_clock_hz({.kernel_hz = 160'000'000, .cpu_hz = 160'000'000, .channel = 18}), 40'000'000u);
}

TEST(DtsConfig, CalibrationClockBelowOneMegahertz) {
    EXPECT_EQ(h5::dts_hsref_div(125'000'000), 126u);
    EXPECT_LT(125'000'000 / h5::dts_hsref_div(125'000'000), 1'000'000u);
    EXPECT_GT(h5::dts_hsref_div(250'000'000), 127u) << "PCLK1 250 MHz is too fast: the board runs APB1 / 2";
    static_assert(h5::rcc::ClockTree{.pll1 = {.m = 32, .n = 250, .p = 2, .q = 5}, .apb1_div = 2}.pclk1_hz() == 125'000'000);
}

TEST(DtsConfig, TemperatureFromCounterAndFactoryValues) {
    // factory values: T0 = 30 degC (code 0), FMT0 = 300.0 kHz, ramp 1000 Hz/degC
    constexpr std::uint32_t t0valr1 = 3000;  // 0.1 kHz units
    constexpr std::uint32_t rampvalr = 1000;
    // FM = 125 MHz * 7 / mfreq; FM = 325 kHz -> 25 degC above T0
    const std::uint32_t mfreq = 125'000'000u * 7 / 325'000;
    const auto t = h5::dts_centi_celsius(125'000'000, 7, mfreq, t0valr1, rampvalr);
    ASSERT_TRUE(t.has_value());
    EXPECT_NEAR(*t, 5500, 15) << "one counter step is ~0.12 degC here (mfreq ~2692)";
    EXPECT_EQ(*h5::dts_centi_celsius(125'000'000, 7, 125'000'000u * 7 / 300'000, t0valr1 | (1u << 16), rampvalr) / 100,
              130) << "T0 code 1: 130 degC";
    EXPECT_FALSE(h5::dts_centi_celsius(125'000'000, 7, 0, t0valr1, rampvalr).has_value());
    EXPECT_FALSE(h5::dts_centi_celsius(125'000'000, 7, mfreq, t0valr1, 0).has_value()) << "unprogrammed ramp";
}

}  // namespace
