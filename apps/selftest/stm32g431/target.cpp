// SPDX-License-Identifier: MIT
//
// Self-test, the STM32G431's part: kernel binding, clock and console, interrupt vectors, fault
// handlers. No tests of its own: the Cortex-M4 has no stack limit register, and the chip
// drivers the self-test could check are covered by the host tests and drivers_check.
#include <array>
#include <cstdint>

#include "rtos/port/cm4/port.hpp"
#include "rtos/port/stm32g4/gpio.hpp"
#include "rtos/port/stm32g4/usart.hpp"
#include "selftest.hpp"

RTOS_CM4_BIND_KERNEL(kernel)

namespace {
namespace g4 = rtos::port::stm32g4;
using Console = g4::Usart<g4::UsartId::Usart1>;
using g4::gpio::GpioPort, g4::gpio::Signal;

constexpr std::array console_pins{
    g4::gpio::alternate({GpioPort::C, 4}, Signal::USART1_TX),
    g4::gpio::alternate({GpioPort::B, 7}, Signal::USART1_RX, g4::gpio::Speed::Low, g4::gpio::Pull::Up),
};
}  // namespace

namespace target {

void init() noexcept {
    g4::rcc::configure_clocks<clocks>();
    g4::gpio::configure<console_pins>();
    Console::configure<g4::UartConfig{.kernel_hz = clocks.pclk2_hz(), .baud = 115'200, .fifo = false}>();
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
void TIM7_IRQHandler() noexcept { selftest::spare_irq(3); }

void UsageFault_Handler() noexcept { selftest::fatal("UsageFault"); }
void HardFault_Handler() noexcept { selftest::fatal("HardFault"); }
void MemManage_Handler() noexcept { selftest::fatal("MemManage"); }
void BusFault_Handler() noexcept { selftest::fatal("BusFault"); }
}
