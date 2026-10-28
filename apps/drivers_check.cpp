// SPDX-License-Identifier: MIT
//
// Compile check of the STM32H5 chip drivers: instantiates every driver template with the
// FCU's clocks, so the compile-time configuration checks and the register code are built
// even before an application uses them. Not linked into any firmware.
#include <array>
#include <cstddef>

#include "board/fcu.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
#include "rtos/port/stm32h5/usart.hpp"

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
using Wide = h5::Timer<h5::TimerId::Tim2, {.kernel_hz = board::clocks.timer1_hz(), .tick_hz = 250'000'000}>;
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

    Spi2::init();
    (void)Spi2::transfer(b, b);
    (void)Spi2::write(b);
    (void)Spi2::read(b);
}
