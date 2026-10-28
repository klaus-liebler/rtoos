// SPDX-License-Identifier: MIT
//
// Compile check of the STM32H5 chip drivers: instantiates every driver template with the
// FCU's clocks, so the compile-time configuration checks and the register code are built
// even before an application uses them. Not linked into any firmware.
#include <array>
#include <cstddef>

#include "board/fcu.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/port/stm32h5/usart.hpp"

namespace h5 = rtos::port::stm32h5;

namespace {
constexpr h5::UartConfig single_wire{.kernel_hz = board::clocks.pclk1_hz(), .baud = 115'200, .half_duplex = true};
h5::BufferedUart<h5::UsartId::Uart5, single_wire> uart5;

using I2c4 = h5::I2cMaster<h5::I2cId::I2c4, {.kernel_hz = board::clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz}>;
using I2c1 = h5::I2cMaster<h5::I2cId::I2c1,
                           {.kernel_hz = board::clocks.pclk1_hz(), .cpu_hz = board::sysclk_hz, .bus_hz = 400'000}>;
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
}
