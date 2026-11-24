// SPDX-License-Identifier: MIT
//
// Compile check of the STM32F103 chip drivers: instantiates every driver template with a 72 MHz
// clock tree (HSE 12 MHz from an oscillator, the sensactUp board's), so the compile-time
// configuration checks and the register code are built even before a board uses them. Not
// linked into any firmware.
#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/port/stm32f1/bxcan.hpp"
#include "rtos/devices/ws2812.hpp"
#include "rtos/port/stm32f1/clocks.hpp"
#include "rtos/port/stm32f1/dma.hpp"
#include "rtos/port/stm32f1/gpio.hpp"
#include "rtos/port/stm32f1/i2c.hpp"
#include "rtos/port/stm32f1/iwdg.hpp"
#include "rtos/port/stm32f1/timer.hpp"
#include "rtos/port/stm32f1/usart.hpp"

namespace f1 = rtos::port::stm32f1;

namespace {
using f1::gpio::GpioPort, f1::gpio::Signal, f1::gpio::Pull, f1::gpio::Speed;

// HSE 12 MHz (bypass) x 6 = 72 MHz, APB1 36 MHz, APB2 72 MHz
constexpr f1::rcc::ClockTree clocks{.pll_source = f1::rcc::PllSource::Hse, .hse_hz = 12'000'000,
                                    .hse_bypass = true, .pll_mul = 6};
static_assert(clocks.sysclk_hz() == 72'000'000 && clocks.pclk1_hz() == 36'000'000 && clocks.timer1_hz() == 72'000'000);
static_assert(clocks.wait_states() == 2 && clocks.adc_hz() == 12'000'000);
constexpr std::uint32_t cpu_hz = clocks.sysclk_hz();

constexpr std::array pins{
    f1::gpio::output({GpioPort::B, 12}, true),
    f1::gpio::input({GpioPort::B, 0}, Pull::Up),
    f1::gpio::alternate({GpioPort::A, 9}, Signal::USART1_TX),
    f1::gpio::peripheral_input({GpioPort::A, 10}, Signal::USART1_RX, Pull::Up),
    f1::gpio::alternate({GpioPort::B, 10}, Signal::I2C2_SCL, Speed::Low, true),
    f1::gpio::alternate({GpioPort::B, 11}, Signal::I2C2_SDA, Speed::Low, true),
    f1::gpio::alternate({GpioPort::A, 12}, Signal::CAN_TX),
    f1::gpio::peripheral_input({GpioPort::A, 11}, Signal::CAN_RX, Pull::Up),
    f1::gpio::peripheral_input({GpioPort::A, 0}, Signal::TIM2_CH1),
    f1::gpio::peripheral_input({GpioPort::A, 1}, Signal::TIM2_CH2),
    f1::gpio::peripheral_input({GpioPort::B, 4}, Signal::TIM3_CH1, Pull::Up),  // partial remap, JTAG off
    f1::gpio::peripheral_input({GpioPort::B, 5}, Signal::TIM3_CH2, Pull::Up),
    f1::gpio::alternate({GpioPort::B, 6}, Signal::TIM4_CH1),
    f1::gpio::alternate({GpioPort::A, 8}, Signal::TIM1_CH1),
    f1::gpio::analog({GpioPort::A, 2}, Signal::ADC1_IN2),
};
constexpr f1::gpio::Plan plan = f1::gpio::plan(pins);
static_assert(plan.swj == f1::gpio::Swj::NoJtag, "PB4 takes NJTRST: JTAG off, SWD on");
static_assert(plan.mapr_mask == ((3u << 10) | (3u << 8) | (1u << 12) | (3u << 13) | (3u << 6) | (1u << 2)) &&
                  plan.mapr == (2u << 10),
              "TIM3 partial remap, everything else at its default");

f1::BufferedUart<f1::UsartId::Usart1, {.kernel_hz = clocks.pclk2_hz(), .baud = 115'200}> console;
using Console = f1::Usart<f1::UsartId::Usart1>;
using I2c2 = f1::I2cMaster<f1::I2cId::I2c2, {.pclk1_hz = clocks.pclk1_hz(), .cpu_hz = cpu_hz, .bus_hz = 50'000,
                                             .scl = {GpioPort::B, 10}, .sda = {GpioPort::B, 11}}>;
using I2c1Fast = f1::I2cMaster<f1::I2cId::I2c1, {.pclk1_hz = clocks.pclk1_hz(), .cpu_hz = cpu_hz, .bus_hz = 400'000,
                                                 .scl = {GpioPort::B, 8}, .sda = {GpioPort::B, 9}}>;
using Can = f1::BxCan<{.kernel_hz = clocks.pclk1_hz(), .cpu_hz = cpu_hz, .bitrate = 125'000}>;
static_assert(Can::bit_timing.prescaler == 16 && Can::bit_timing.quanta() == 18);
using Pwm = f1::Timer<f1::TimerId::Tim4, {.kernel_hz = clocks.timer1_hz(), .update_hz = 1'000, .min_steps = 1000}>;
using Knob = f1::Timer<f1::TimerId::Tim2, {.kernel_hz = clocks.timer1_hz(), .tick_hz = clocks.timer1_hz()}>;
using Leds = f1::Timer<f1::TimerId::Tim1, {.kernel_hz = clocks.timer2_hz(), .update_hz = 800'000}>;
// WS2812 on TIM1_CH1: 800 kHz, the next compare value per update event by DMA (channel 5)
using LedDma = f1::DmaChannel<{.request = f1::DmaRequest::Tim1Up, .direction = f1::DmaDirection::MemoryToPeripheral,
                               .memory_bytes = 2, .peripheral_bytes = 2, .priority = 2}>;
static_assert(LedDma::channel_number == 5 && LedDma::irq == 15);
using UartRxDma = f1::DmaChannel<{.request = f1::DmaRequest::Usart1Rx, .direction = f1::DmaDirection::PeripheralToMemory}>;
std::array<std::uint16_t, rtos::devices::ws2812_slots(5)> led_slots{};
using Watchdog = f1::Iwdg<{.pclk1_hz = clocks.pclk1_hz(), .timeout_ms = 2'000}>;
static_assert(Watchdog::registers.earliest_ms() >= 2'000 && Watchdog::registers.latest_ms() <= 4'100);
}  // namespace

void drivers_check() noexcept {
    if (!f1::rcc::configure_clocks<clocks>()) {
        return;
    }
    f1::gpio::configure<pins>();
    f1::gpio::toggle({GpioPort::B, 12});
    Console::configure<{.kernel_hz = clocks.pclk2_hz()}>();
    Console::write("x\n");
    Console::flush();
    console.start();
    std::byte b{0x55};
    (void)console.write(std::span(&b, 1));
    console.on_irq();
    I2c2::init();
    std::array<std::byte, 3> rx{};
    (void)I2c2::read_register(0x40, 0x00, rx);
    (void)I2c2::read_register(0x40, 0x00, std::span(rx.data(), 1));
    (void)I2c2::read_register(0x40, 0x00, std::span(rx.data(), 2));
    (void)I2c2::write_register(0x40, 0x06, rx);
    (void)I2c2::probe(0x41);
    I2c2::recover();
    I2c1Fast::init();
    (void)Can::init();
    Can::enable_rx_irq();
    (void)Can::send({.id = 0x1234, .extended = true, .length = 2, .data = {1, 2}});
    f1::CanFrame frame;
    (void)Can::receive(frame);
    (void)Can::status();
    (void)Can::rx_overflowed();
    Pwm::init();
    Pwm::configure_pwm<1>();
    Pwm::start();
    Pwm::set_duty_permille<1>(500);
    Knob::init();
    Knob::configure_encoder<f1::EncoderMode::Ti1, 15, true>();
    Knob::start();
    (void)Knob::counter();
    Leds::init();
    Leds::configure_pwm<1>();
    constexpr auto timing = rtos::devices::ws2812_timing(clocks.timer2_hz());
    static_assert(timing.period_ticks == 90);
    const std::array<std::uint32_t, 5> colours{0xFF0000, 0x00FF00, 0x0000FF, 0, 0xFFFFFF};
    rtos::devices::ws2812_encode(colours, led_slots, timing);
    LedDma::init();
    Leds::enable_update_dma();
    (void)LedDma::start(Leds::compare_address<1>(), led_slots.data(), led_slots.size());
    while (LedDma::busy()) {
    }
    (void)LedDma::done();
    (void)LedDma::failed();
    LedDma::clear();
    UartRxDma::init();
    UartRxDma::stop();
    Watchdog::start();
    Watchdog::kick();
}
