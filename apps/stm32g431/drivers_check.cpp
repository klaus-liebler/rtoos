// SPDX-License-Identifier: MIT
//
// Compile check of the STM32G431 chip drivers: instantiates every driver template with a
// 170 MHz clock tree (HSE 8 MHz), so the compile-time configuration checks and the register
// code are built even before a board uses them. Not linked into any firmware.
#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/devices/ina226.hpp"
#include "rtos/port/stm32g4/adc.hpp"
#include "rtos/port/stm32g4/clocks.hpp"
#include "rtos/port/stm32g4/dma.hpp"
#include "rtos/port/stm32g4/fdcan.hpp"
#include "rtos/port/stm32g4/gpio.hpp"
#include "rtos/port/stm32g4/i2c.hpp"
#include "rtos/port/stm32g4/rng.hpp"
#include "rtos/port/stm32g4/spi.hpp"
#include "rtos/port/stm32g4/timer.hpp"
#include "rtos/port/stm32g4/ucpd.hpp"
#include "rtos/port/stm32g4/usart.hpp"
#include "rtos/port/stm32g4/usb_device.hpp"
#include "rtos/usb_pd/sink.hpp"

namespace g4 = rtos::port::stm32g4;

namespace {
// HSE 8 MHz / 2 x 85 = 340 MHz VCO; R / 2 = 170 MHz, Q / 8 = 42.5 MHz, P / 8 = 42.5 MHz
constexpr g4::rcc::ClockTree clocks{
    .pll = {.source = g4::rcc::PllSource::Hse, .input_hz = 8'000'000, .m = 2, .n = 85, .r = 2, .q = 8, .p = 8},
    .hsi48 = true};
static_assert(clocks.sysclk_hz() == 170'000'000 && clocks.boost() && clocks.wait_states() == 4);
constexpr std::uint32_t cpu_hz = clocks.sysclk_hz();

constexpr std::array pins{
    g4::gpio::output({g4::gpio::GpioPort::C, 6}),
    g4::gpio::alternate({g4::gpio::GpioPort::C, 4}, g4::gpio::Signal::USART1_TX),
    g4::gpio::alternate({g4::gpio::GpioPort::B, 7}, g4::gpio::Signal::USART1_RX, g4::gpio::Speed::Low,
                        g4::gpio::Pull::Up),
    g4::gpio::alternate({g4::gpio::GpioPort::A, 8}, g4::gpio::Signal::I2C2_SDA, g4::gpio::Speed::Low,
                        g4::gpio::Pull::None, true),
    g4::gpio::alternate({g4::gpio::GpioPort::A, 9}, g4::gpio::Signal::I2C2_SCL, g4::gpio::Speed::Low,
                        g4::gpio::Pull::None, true),
    g4::gpio::analog({g4::gpio::GpioPort::B, 6}, g4::gpio::Signal::UCPD1_CC1),
    g4::gpio::analog({g4::gpio::GpioPort::B, 4}, g4::gpio::Signal::UCPD1_CC2),
    g4::gpio::analog({g4::gpio::GpioPort::B, 2}, g4::gpio::Signal::ADC2_IN12),
};

g4::BufferedUart<g4::UsartId::Usart2, {.kernel_hz = clocks.pclk1_hz(), .baud = 1'000'000, .half_duplex = true}> servo;
using Console = g4::Usart<g4::UsartId::Usart1>;
using I2c2 = g4::I2cMaster<g4::I2cId::I2c2, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = cpu_hz, .bus_hz = 400'000}>;
using Spi2 = g4::SpiMaster<g4::SpiId::Spi2, {.kernel_hz = clocks.pclk1_hz(), .max_bus_hz = 1'000'000, .cpu_hz = cpu_hz}>;
using Pwm = g4::Timer<g4::TimerId::Tim3, {.kernel_hz = clocks.timer1_hz(), .update_hz = 20'000, .min_steps = 1000}>;
using Leds = g4::Timer<g4::TimerId::Tim16, {.kernel_hz = clocks.timer2_hz(), .update_hz = 800'000}>;
using Supply = g4::Adc<g4::AdcId::Adc2, {.kernel_hz = clocks.sysclk_hz(), .cpu_hz = cpu_hz, .channel = 12}>;
using Can = g4::Fdcan<{.kernel_hz = clocks.pclk1_hz(), .cpu_hz = cpu_hz, .bitrate = 500'000}>;
using LedDma = g4::DmaChannel<{.channel = 3, .request = g4::DmaRequest::Tim16Ch1,
                               .direction = g4::DmaDirection::MemoryToPeripheral, .memory_bytes = 2,
                               .peripheral_bytes = 2}>;
using Dma2 = g4::DmaChannel<{.controller = 2, .channel = 6, .request = g4::DmaRequest::Spi2Rx,
                             .direction = g4::DmaDirection::PeripheralToMemory}>;
static_assert(LedDma::irq == 13 && Dma2::irq == 97);
std::array<std::uint16_t, 24> led_bits{};
g4::UsbFs<> usb;
using PdClock = g4::MicrosecondTimer<g4::TimerId::Tim2, clocks.timer1_hz()>;
PdClock pd_clock;
g4::Ucpd<{.kernel_hz = clocks.ucpd_hz(), .rx_dma_channel = 1, .tx_dma_channel = 2}> ucpd;
rtos::usb_pd::Sink<decltype(ucpd), PdClock> pd_sink{ucpd, pd_clock};
using Current = rtos::devices::Ina226<I2c2, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;
// No HX711 on the G4: the driver queues its 8 clock bytes in the SPI TxFIFO before the start
// (Spi::fifo_frames >= 8: a pause with PD_SCK high powers the HX711 down), and the G4's SPI
// does not offer that (no fifo_frames). The H5 drivers check instantiates it.
}  // namespace

void drivers_check() noexcept {
    g4::rcc::configure_clocks<clocks>();
    g4::gpio::configure<pins>();
    g4::gpio::toggle({g4::gpio::GpioPort::C, 6});
    Console::configure<{.kernel_hz = clocks.pclk2_hz()}>();
    Console::write("x\n");

    servo.start();
    servo.set_baud<115'200>();
    std::array<std::byte, 4> b{};
    (void)servo.write(b);
    (void)servo.read(b);
    servo.on_irq();

    I2c2::init();
    (void)I2c2::read_register(0x40, 0x02, b);
    I2c2::recover();
    Spi2::init();
    (void)Spi2::transfer(b, b);
    (void)Spi2::read(b);

    Pwm::init();
    Pwm::configure_pwm<1>();
    Pwm::set_duty_permille<1>(500);
    Pwm::start();
    Leds::init();
    Leds::configure_pwm<1>();
    Leds::enable_compare_dma<1>();
    Leds::configure_dma_burst(0x34, 1);
    LedDma::init();
    (void)LedDma::start(Leds::compare_address<1>(), led_bits.data(), led_bits.size());
    (void)LedDma::done();
    Dma2::init();

    (void)Supply::init();
    Supply::start_continuous();
    (void)Supply::read();

    (void)Can::init();
    (void)Can::send({.id = 0x123, .length = 1, .data = {1}});
    g4::CanFrame f;
    (void)Can::receive(f);

    g4::Rng::init();
    (void)g4::Rng::next();

    g4::UsbHardware::enable_clocks();
    usb.start();
    usb.connect();
    usb.on_irq();

    PdClock::init();
    ucpd.start();
    (void)pd_sink.request_power(9'000);
    ucpd.on_irq(pd_sink);
    if (PdClock::take()) {
        pd_sink.on_timer();
    }
    pd_sink.poll([](rtos::usb_pd::SinkEvent) {});

    if (Current::probe() && Current::init()) {
        (void)Current::read();
    }
}
