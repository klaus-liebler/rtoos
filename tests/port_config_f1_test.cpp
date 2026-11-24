// SPDX-License-Identifier: MIT
//
// Compile-time configuration of the STM32F103 chip drivers: the register values the drivers
// write are computed here exactly as on the target (they are constexpr), so their limits can be
// checked against RM0008 and the datasheet on the host. Expected encodings: RM0008 rev 21
// (RCC_CFGR, I2C_CCR/TRISE, USART_BRR, CAN_BTR, AFIO_MAPR, GPIOx_CRL/CRH, IWDG), CubeMX pin
// database (remaps).
#include <gtest/gtest.h>

#include <array>

#include "rtos/port/stm32f1/bxcan.hpp"
#include "rtos/port/stm32f1/clocks.hpp"
#include "rtos/port/stm32f1/gpio.hpp"
#include "rtos/port/stm32f1/i2c.hpp"
#include "rtos/port/stm32f1/iwdg.hpp"
#include "rtos/port/stm32f1/timer.hpp"
#include "rtos/port/stm32f1/usart.hpp"

namespace {

namespace f1 = rtos::port::stm32f1;
using f1::gpio::GpioPort, f1::gpio::Signal, f1::gpio::Pull, f1::gpio::Speed;

TEST(F1Clocks, HseOscillatorTo72MHz) {
    constexpr f1::rcc::ClockTree t{.pll_source = f1::rcc::PllSource::Hse, .hse_hz = 12'000'000, .hse_bypass = true,
                                   .pll_mul = 6};
    static_assert(t.valid());
    EXPECT_EQ(t.sysclk_hz(), 72'000'000u);
    EXPECT_EQ(t.pclk1_hz(), 36'000'000u);
    EXPECT_EQ(t.pclk2_hz(), 72'000'000u);
    EXPECT_EQ(t.timer1_hz(), 72'000'000u) << "APB1 divided: timers at 2 x PCLK1";
    EXPECT_EQ(t.timer2_hz(), 72'000'000u);
    EXPECT_EQ(t.adc_hz(), 12'000'000u);
    EXPECT_EQ(t.wait_states(), 2u);
    // PPRE1 = 100 (/2), ADCPRE = 10 (/6), PLLSRC = HSE, PLLMUL = 0100 (x6)
    EXPECT_EQ(t.cfgr(), (4u << 8) | (2u << 14) | (1u << 16) | (4u << 18));
}

TEST(F1Clocks, HsiPllAndDividedHse) {
    constexpr f1::rcc::ClockTree hsi{.pll_mul = 16};
    static_assert(hsi.valid() && !hsi.uses_hse());
    EXPECT_EQ(hsi.sysclk_hz(), 64'000'000u);
    EXPECT_EQ(hsi.cfgr() & (1u << 16), 0u) << "PLLSRC = HSI / 2";
    constexpr f1::rcc::ClockTree hse_div2{.pll_source = f1::rcc::PllSource::HseDiv2, .hse_hz = 16'000'000, .pll_mul = 9};
    static_assert(hse_div2.valid());
    EXPECT_EQ(hse_div2.sysclk_hz(), 72'000'000u);
    EXPECT_NE(hse_div2.cfgr() & (1u << 17), 0u) << "PLLXTPRE";
    EXPECT_EQ(f1::flash::wait_states(24'000'000), 0u);
    EXPECT_EQ(f1::flash::wait_states(48'000'000), 1u);
    EXPECT_EQ(f1::flash::wait_states(48'000'001), 2u);
}

TEST(F1Clocks, Limits) {
    EXPECT_FALSE((f1::rcc::ClockTree{.pll_source = f1::rcc::PllSource::Hse, .hse_hz = 8'000'000, .pll_mul = 10}.valid()))
        << "PLL 80 MHz > 72 MHz";
    EXPECT_FALSE((f1::rcc::ClockTree{.pll_mul = 16, .apb1_div = 1}.valid())) << "PCLK1 64 MHz > 36 MHz";
    EXPECT_TRUE((f1::rcc::ClockTree{.pll_mul = 9, .apb1_div = 1}.valid())) << "PCLK1 36 MHz is the limit";
    EXPECT_FALSE((f1::rcc::ClockTree{.pll_source = f1::rcc::PllSource::Hse, .hse_hz = 25'000'000, .pll_mul = 2}.valid()))
        << "a crystal is 4..16 MHz";
    EXPECT_TRUE((f1::rcc::ClockTree{.pll_source = f1::rcc::PllSource::Hse, .hse_hz = 24'000'000, .hse_bypass = true,
                                    .pll_mul = 3}.valid()))
        << "an external clock up to 25 MHz";
    EXPECT_FALSE((f1::rcc::ClockTree{.pll_mul = 16, .adc_div = 2}.valid())) << "ADC 32 MHz > 14 MHz";
    EXPECT_FALSE((f1::rcc::ClockTree{.pll_mul = 17}.valid())) << "PLLMUL is 2..16";
}

TEST(F1I2c, StandardAndFastModeRegisters) {
    // 36 MHz, 50 kHz: CCR = 36 MHz / (2 x 50 kHz) = 360, TRISE = 1000 ns x 36 MHz + 1
    constexpr f1::I2cTiming slow = f1::i2c_timing(36'000'000, 50'000);
    ASSERT_TRUE(slow.valid);
    EXPECT_EQ(slow.freq, 36u);
    EXPECT_EQ(slow.ccr, 360u);
    EXPECT_EQ(slow.trise, 37u);
    EXPECT_EQ(slow.scl_hz(36'000'000), 50'000u);
    // 88 kHz: rounded up to CCR 205 -> SCL 87.8 kHz, not faster than asked (erratum 2.8.5 limit)
    constexpr f1::I2cTiming limit = f1::i2c_timing(36'000'000, 88'000);
    EXPECT_EQ(limit.ccr, 205u);
    EXPECT_LE(limit.scl_hz(36'000'000), 88'000u);
    // 400 kHz: F/S, DUTY 0 (2:1), CCR = 36 MHz / 1.2 MHz = 30, TRISE = 300 ns x 36 MHz + 1
    constexpr f1::I2cTiming fast = f1::i2c_timing(36'000'000, 400'000);
    EXPECT_EQ(fast.ccr, (1u << 15) | 30u);
    EXPECT_EQ(fast.trise, 11u);
    EXPECT_EQ(fast.scl_hz(36'000'000), 400'000u);
    EXPECT_FALSE(f1::i2c_timing(36'500'000, 50'000).valid) << "FREQ is whole MHz";
    EXPECT_FALSE(f1::i2c_timing(1'000'000, 50'000).valid) << "FREQ 2 MHz at least";
}

TEST(F1Usart, BaudRateRegister) {
    constexpr f1::UartConfig c72{.kernel_hz = 72'000'000, .baud = 115'200};
    EXPECT_EQ(f1::uart_registers(c72).brr, 625u);  // 39.0625 -> mantissa 39, fraction 1
    EXPECT_EQ(f1::uart_baud_error(c72), 0u);
    constexpr f1::UartConfig c36{.kernel_hz = 36'000'000, .baud = 115'200, .parity = f1::Parity::Even};
    EXPECT_EQ(f1::uart_registers(c36).brr, 313u);
    EXPECT_LE(f1::uart_baud_error(c36), 20u);
    // UE, TE, RE; parity: PCE + M (9 bits)
    EXPECT_EQ(f1::uart_registers(c36).cr1, (1u << 13) | (1u << 3) | (1u << 2) | (1u << 10) | (1u << 12));
}

TEST(F1BxCan, BitTimingWithinTheSegmentRanges) {
    // 36 MHz, 125 kbit/s: 24 quanta would need TS1 = 20 (> 16) - 18 quanta, BRP 16, TS1 15, TS2 2
    constexpr auto t = f1::bxcan_bit_timing(36'000'000, 125'000, 875);
    ASSERT_TRUE(t.valid);
    EXPECT_EQ(t.prescaler, 16u);
    EXPECT_EQ(t.quanta(), 18u);
    EXPECT_EQ(t.tseg1, 15u);
    EXPECT_EQ(t.tseg2, 2u);
    EXPECT_EQ(t.sjw, 2u);
    EXPECT_EQ(t.sample_permille(), 888u);
    // CAN_BTR: SJW-1 at 24, TS2-1 at 20, TS1-1 at 16, BRP-1
    EXPECT_EQ(f1::bxcan_btr(t), (1u << 24) | (1u << 20) | (14u << 16) | 15u);
    // 32 MHz (HSI PLL 64 MHz): 16 quanta, BRP 16
    constexpr auto hsi = f1::bxcan_bit_timing(32'000'000, 125'000, 875);
    EXPECT_EQ(hsi.prescaler, 16u);
    EXPECT_EQ(hsi.quanta(), 16u);
    EXPECT_EQ(hsi.tseg2, 2u);
    EXPECT_FALSE(f1::bxcan_bit_timing(36'000'000, 1'000'003, 875).valid) << "no exact divider";
}

TEST(F1Gpio, ConfigurationNibbles) {
    using f1::gpio::nibble;
    EXPECT_EQ(nibble(f1::gpio::input({GpioPort::B, 0}, Pull::Up)), 0b1000u) << "CNF 10: pull-up/down";
    EXPECT_EQ(nibble(f1::gpio::input({GpioPort::B, 0})), 0b0100u) << "CNF 01: floating";
    EXPECT_EQ(nibble(f1::gpio::output({GpioPort::B, 12})), 0b0010u) << "push-pull, 2 MHz";
    EXPECT_EQ(nibble(f1::gpio::output({GpioPort::B, 12}, false, Speed::High, true)), 0b0111u) << "open-drain, 50 MHz";
    EXPECT_EQ(nibble(f1::gpio::alternate({GpioPort::B, 10}, Signal::I2C2_SCL, Speed::Low, true)), 0b1110u)
        << "alternate open-drain, 2 MHz";
    EXPECT_EQ(nibble(f1::gpio::alternate({GpioPort::A, 9}, Signal::USART1_TX)), 0b1011u) << "alternate push-pull, 50 MHz";
    EXPECT_EQ(nibble(f1::gpio::analog({GpioPort::A, 2}, Signal::ADC1_IN2)), 0u);
}

TEST(F1Gpio, PlanFoldsPinsRemapsAndDebugPort) {
    constexpr std::array pins{
        f1::gpio::input({GpioPort::B, 0}, Pull::Up),
        f1::gpio::output({GpioPort::B, 12}, true),
        f1::gpio::peripheral_input({GpioPort::B, 4}, Signal::TIM3_CH1, Pull::Up),
        f1::gpio::peripheral_input({GpioPort::B, 5}, Signal::TIM3_CH2, Pull::Up),
        f1::gpio::peripheral_input({GpioPort::A, 0}, Signal::TIM2_CH1),
    };
    constexpr f1::gpio::Plan p = f1::gpio::plan(pins);
    const auto& b = p.ports[1];
    EXPECT_TRUE(b.used);
    EXPECT_EQ(b.crl_mask, 0x00FF'000Fu);
    EXPECT_EQ(b.crl, 0x0088'0008u);
    EXPECT_EQ(b.crh_mask, 0x000F'0000u);
    EXPECT_EQ(b.crh, 0x0002'0000u);
    // pull-ups PB0/PB4/PB5 and the LED high: BSRR set bits
    EXPECT_EQ(b.bsrr, (1u << 0) | (1u << 4) | (1u << 5) | (1u << 12));
    EXPECT_EQ(p.ports[0].crl, 0x4u) << "PA0 floating input";
    // TIM3 partial remap (10b at bits 11:10); TIM2 allows 00 and 10 on PA0 - the lowest is taken
    EXPECT_EQ(p.mapr_mask, (3u << 10) | (3u << 8));
    EXPECT_EQ(p.mapr, 2u << 10);
    EXPECT_EQ(p.swj, f1::gpio::Swj::NoJtag) << "PB4 is NJTRST after reset";

    constexpr std::array swd{f1::gpio::output({GpioPort::A, 13})};
    EXPECT_EQ(f1::gpio::plan(swd).swj, f1::gpio::Swj::Off);
    constexpr std::array plain{f1::gpio::output({GpioPort::C, 13})};
    EXPECT_EQ(f1::gpio::plan(plain).swj, f1::gpio::Swj::Full);
    static_assert(f1::gpio::pin_lists_compatible(pins, plain));
}

TEST(F1Gpio, RemapIntersectionAcrossPins) {
    // PA15 carries TIM2_CH1 only with TIM2_REMAP 01 or 11, PB3 TIM2_CH2 with 01 or 11, PB10 TIM2_CH3
    // with 10 or 11: together only 11 (full remap)
    constexpr std::array pins{
        f1::gpio::peripheral_input({GpioPort::A, 15}, Signal::TIM2_CH1),
        f1::gpio::peripheral_input({GpioPort::B, 3}, Signal::TIM2_CH2),
        f1::gpio::peripheral_input({GpioPort::B, 10}, Signal::TIM2_CH3),
    };
    constexpr f1::gpio::Plan p = f1::gpio::plan(pins);
    EXPECT_EQ(p.mapr, 3u << 8);
}

TEST(F1Iwdg, TimeoutNeverEarlierThanAsked) {
    // 2 s at 60 kHz: /32, 3750 ticks; at 30 kHz the same counts last 4 s
    constexpr f1::IwdgRegisters r = f1::iwdg_registers(2'000);
    ASSERT_TRUE(r.valid);
    EXPECT_EQ(r.pr, 3u);
    EXPECT_EQ(r.rlr, 3749u);
    EXPECT_EQ(r.earliest_ms(), 2'000u);
    EXPECT_EQ(r.latest_ms(), 4'000u);
    EXPECT_FALSE(f1::iwdg_registers(20'000).valid) << "above 17 s";
}

TEST(F1Timer, EncoderInputConfiguration) {
    // CC1S = CC2S = 01, IC1F = IC2F = 1111
    EXPECT_EQ(f1::timer_encoder_ccmr1(15), 0xF1F1u);
    EXPECT_EQ(f1::timer_encoder_ccmr1(0), 0x0101u);
    constexpr auto pwm = f1::timer_registers({.kernel_hz = 72'000'000, .update_hz = 1'000, .min_steps = 1000}, 0xFFFF);
    ASSERT_TRUE(pwm.valid);
    EXPECT_EQ((pwm.psc + 1) * (pwm.arr + 1), 72'000u);
}

}  // namespace
