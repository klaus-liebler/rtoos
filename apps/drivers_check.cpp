// SPDX-License-Identifier: MIT
//
// Compile check of the STM32H5 chip drivers: instantiates every driver template with the
// FCU's clocks, so the compile-time configuration checks and the register code are built
// even before an application uses them. Not linked into any firmware.
#include <array>
#include <cstddef>

#include "board/fcu.hpp"
#include "rtos/port/stm32h5/adc.hpp"
#include "rtos/port/stm32h5/dts.hpp"
#include "rtos/port/stm32h5/fdcan.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
#include "rtos/port/stm32h5/usart.hpp"
#include "rtos/port/stm32h5/usb_device.hpp"

namespace h5 = rtos::port::stm32h5;

namespace {
constexpr h5::UartConfig single_wire{.kernel_hz = board::clocks.pclk1_hz(), .baud = 115'200, .half_duplex = true};
h5::BufferedUart<h5::UsartId::Uart5, single_wire> uart5;

using I2c4 = h5::I2cMaster<h5::I2cId::I2c4, {.kernel_hz = board::clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz}>;
using I2c1 = h5::I2cMaster<h5::I2cId::I2c1,
                           {.kernel_hz = board::clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz, .bus_hz = 400'000}>;
using Spi2 = h5::SpiMaster<h5::SpiId::Spi2, {.kernel_hz = board::pll1_q_hz, .max_bus_hz = 1'000'000,
                                             .cpu_hz = board::sysclk_hz}>;
using Pwm = h5::Timer<h5::TimerId::Tim4, {.kernel_hz = board::clocks.timer1_hz(), .update_hz = 20'000, .min_steps = 1000}>;
using Steps = h5::Timer<h5::TimerId::Tim16, {.kernel_hz = board::clocks.timer2_hz(), .tick_hz = 1'000'000}>;
using Leds = h5::Timer<h5::TimerId::Tim15, {.kernel_hz = board::clocks.timer2_hz(), .update_hz = 800'000}>;
using Pressure = h5::Adc<h5::AdcId::Adc1, {.kernel_hz = board::clocks.hclk_hz(), .cpu_hz = board::sysclk_hz,
                                           .channel = 18, .sample = h5::AdcSample::Cycles2_5}>;
using CpuTemperature = h5::Dts<{.pclk_hz = board::clocks.pclk1_hz()}>;
using Can = h5::Fdcan<{.kernel_hz = board::pll1_q_hz, .cpu_hz = board::sysclk_hz, .bitrate = 500'000}>;
using Wide = h5::Timer<h5::TimerId::Tim2, {.kernel_hz = board::clocks.timer1_hz(), .tick_hz = 250'000'000}>;
h5::UsbDrdFs<> usb_fs;
using LedDma = h5::DmaChannel<{.channel = 2, .request = h5::DmaRequest::Tim15Ch1,
                                .direction = h5::DmaDirection::MemoryToPeripheral, .memory_bytes = 2, .peripheral_bytes = 2}>;
std::array<std::uint16_t, 24> led_bits{};
}  // namespace

void drivers_check() noexcept {
    uart5.start();
    uart5.set_baud<1'000'000>();
    std::array<std::byte, 4> b{};
    (void)uart5.write(b);
    (void)uart5.read(b);
    uart5.on_irq();
    (void)uart5.tx_idle();

    I2c4::init();
    (void)I2c4::read_register(0x40, 0x02, b);
    (void)I2c4::write_register(0x40, 0x00, b);
    (void)I2c4::probe(0x29);
    I2c4::recover();
    I2c1::init();

    Pwm::init();
    Pwm::configure_pwm<3>();
    Pwm::configure_pwm<4>(true);
    Pwm::set_duty_permille<3>(500);
    Pwm::start();
    Steps::init();
    Steps::set_period(250);
    Steps::enable_update_irq();
    (void)Steps::take_update();
    Leds::init();
    Leds::configure_pwm<2>();
    Leds::enable_update_dma();
    Leds::configure_dma_burst(0x38, 1, 1);
    Wide::init();
    Wide::set_period(1'000'000'000);

    (void)Pressure::init();
    Pressure::start_continuous();
    (void)Pressure::read();
    Pressure::stop();
    std::uint16_t v = 0;
    (void)Pressure::convert(v);
    CpuTemperature::init();
    if (CpuTemperature::ready()) {
        CpuTemperature::start();
    }
    (void)CpuTemperature::centi_celsius();

    (void)Can::init();
    (void)Can::send({.id = 0x123, .length = 2, .data = {1, 2}});
    h5::CanFrame frame;
    (void)Can::receive(frame);
    (void)Can::status();
    (void)Can::rx_overflowed();

    h5::UsbHardware::enable_clocks();
    usb_fs.start();
    usb_fs.connect();
    usb_fs.on_irq();
    (void)usb_fs.poll();
    (void)usb_fs.open({0x81, rtos::usb::TransferType::Bulk, 64});
    (void)usb_fs.transfer(0x81, b, true);
    usb_fs.stall(0x81);
    usb_fs.clear_stall(0x81);
    usb_fs.set_address(5);
    usb_fs.close_all();

    LedDma::init();
    (void)LedDma::start(Leds::compare_address<1>(), led_bits.data(), led_bits.size());
    (void)LedDma::done();
    LedDma::stop();

    Spi2::init();
    (void)Spi2::transfer(b, b);
    (void)Spi2::write(b);
    (void)Spi2::read(b);
}
