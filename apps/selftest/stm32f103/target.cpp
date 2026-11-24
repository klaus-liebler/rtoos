// SPDX-License-Identifier: MIT
//
// Self-test, the STM32F103's part: kernel binding, clock and console, interrupt vectors, fault
// handlers. No tests of its own: the Cortex-M3 has no stack limit register, and the chip drivers
// the self-test could check are covered by the host tests and drivers_check.
#include <array>
#include <cstdint>

#include "rtos/port/cm3/port.hpp"
#include "rtos/port/stm32f1/gpio.hpp"
#include "rtos/port/stm32f1/usart.hpp"
#include "selftest.hpp"

RTOS_CM3_BIND_KERNEL(kernel)

namespace {
namespace f1 = rtos::port::stm32f1;
using Console = f1::Usart<f1::UsartId::Usart1>;
using f1::gpio::GpioPort, f1::gpio::Signal;

constexpr std::array console_pins{
    f1::gpio::alternate({GpioPort::A, 9}, Signal::USART1_TX),
    f1::gpio::peripheral_input({GpioPort::A, 10}, Signal::USART1_RX, f1::gpio::Pull::Up),
};
}  // namespace

namespace target {

void init() noexcept {
    (void)f1::rcc::configure_clocks<clocks>();  // HSI only: cannot fail
    f1::gpio::configure<console_pins>();
    Console::configure<f1::UartConfig{.kernel_hz = clocks.pclk2_hz(), .baud = 115'200}>();
}

void print(std::string_view text) noexcept { Console::write(text); }

void print_dec(std::uint32_t value) noexcept {
    std::array<char, 10> digits{};
    std::size_t n = 0;
    do {
        digits[n++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value != 0);
    while (n != 0) {
        Console::put(digits[--n]);
    }
}

void print_hex(std::uint32_t value) noexcept {
    Console::write("0x");
    for (int shift = 28; shift >= 0; shift -= 4) {
        Console::put("0123456789ABCDEF"[(value >> shift) & 0xFu]);
    }
}

void flush() noexcept { Console::flush(); }

}  // namespace target

void selftest::target_tests() {}

extern "C" {
void TIM2_IRQHandler() noexcept { selftest::spare_irq(0); }
void TIM3_IRQHandler() noexcept { selftest::spare_irq(1); }
void TIM4_IRQHandler() noexcept { selftest::spare_irq(2); }
void TIM1_UP_IRQHandler() noexcept { selftest::spare_irq(3); }

void UsageFault_Handler() noexcept { selftest::fatal("UsageFault"); }
void HardFault_Handler() noexcept { selftest::fatal("HardFault"); }
void MemManage_Handler() noexcept { selftest::fatal("MemManage"); }
void BusFault_Handler() noexcept { selftest::fatal("BusFault"); }
}
