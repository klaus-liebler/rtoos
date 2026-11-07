// SPDX-License-Identifier: MIT
//
// Compile check of the STM32H743 chip drivers: instantiates every driver template with a
// 400 MHz clock tree (HSE 25 MHz, the HSI48 on), so the compile-time configuration checks and
// the register code are built even before a board uses them. Not linked into any firmware.
// The chip package is still growing (clocks, GPIO, USART, caches so far); every driver it gets
// is instantiated here.
#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/port/cm7/cache.hpp"
#include "rtos/port/stm32h7/clocks.hpp"
#include "rtos/port/stm32h7/gpio.hpp"
#include "rtos/port/stm32h7/usart.hpp"

namespace h7 = rtos::port::stm32h7;

namespace {
// HSE 25 MHz / 5 = 5 MHz reference, x 160 = 800 MHz VCO; P / 2 = 400 MHz, Q / 8 = 100 MHz,
// R / 4 = 200 MHz
constexpr h7::rcc::ClockTree clocks{
    .pll1 = {.source = h7::rcc::PllSource::Hse, .input_hz = 25'000'000, .m = 5, .n = 160, .p = 2, .q = 8, .r = 4},
    .hsi48 = true};
static_assert(clocks.cpu_hz() == 400'000'000 && clocks.hclk_hz() == 200'000'000 && clocks.wait_states() == 2 &&
              clocks.pll1_q_hz() == 100'000'000 && clocks.timer1_hz() == 200'000'000);

constexpr std::array pins{
    h7::gpio::output({h7::gpio::GpioPort::E, 1}, /*high*/ true),
    h7::gpio::alternate({h7::gpio::GpioPort::D, 8}, h7::gpio::Signal::USART3_TX),
    h7::gpio::alternate({h7::gpio::GpioPort::D, 9}, h7::gpio::Signal::USART3_RX, h7::gpio::Speed::Low,
                        h7::gpio::Pull::Up),
    h7::gpio::alternate({h7::gpio::GpioPort::B, 14}, h7::gpio::Signal::USART1_TX),
    h7::gpio::analog({h7::gpio::GpioPort::A, 4}, h7::gpio::Signal::ADC1_INP18),
};

h7::BufferedUart<h7::UsartId::Usart1, {.kernel_hz = clocks.pclk2_hz(), .baud = 1'000'000, .half_duplex = true}> servo;
using Console = h7::Usart<h7::UsartId::Usart3>;
alignas(rtos::port::cm7::cache::line_bytes) std::array<std::byte, 64> dma_buffer{};
}  // namespace

void drivers_check() noexcept;
void drivers_check() noexcept {
    rtos::port::cm7::cache::enable_icache();
    rtos::port::cm7::cache::enable_dcache();
    rtos::port::cm7::cache::clean(dma_buffer.data(), dma_buffer.size());
    rtos::port::cm7::cache::invalidate(dma_buffer.data(), dma_buffer.size());
    rtos::port::cm7::cache::clean_invalidate(dma_buffer.data(), dma_buffer.size());

    h7::rcc::configure_clocks<clocks>();
    h7::gpio::configure<pins>();
    h7::gpio::toggle({h7::gpio::GpioPort::E, 1});
    (void)h7::gpio::parse_pin("PD8");
    Console::configure<{.kernel_hz = clocks.pclk1_hz()}>();
    Console::write("x\n");

    servo.start();
    servo.set_baud<115'200>();
    std::array<std::byte, 4> b{};
    (void)servo.write(b);
    (void)servo.read(b);
    servo.on_irq();
}
