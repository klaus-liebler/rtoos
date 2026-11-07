// SPDX-License-Identifier: MIT
//
// Self-test, the STM32H743's part: kernel binding, clock, caches and console, interrupt vectors,
// fault handlers. No tests of its own yet: the Cortex-M7 has no stack limit register, and the
// chip drivers the self-test could check are covered by the host tests and drivers_check.
#include <array>
#include <cstdint>

#include "rtos/port/cm7/cache.hpp"
#include "rtos/port/cm7/port.hpp"
#include "rtos/port/stm32h7/gpio.hpp"
#include "rtos/port/stm32h7/mpu.hpp"
#include "rtos/port/stm32h7/usart.hpp"
#include "selftest.hpp"

RTOS_CM7_BIND_KERNEL(kernel)

namespace {
namespace h7 = rtos::port::stm32h7;
namespace cache = rtos::port::cm7::cache;
using Console = h7::Usart<h7::UsartId::Usart3>;
using h7::gpio::GpioPort, h7::gpio::Signal;

constexpr std::array console_pins{
    h7::gpio::alternate({GpioPort::D, 8}, Signal::USART3_TX),
    h7::gpio::alternate({GpioPort::D, 9}, Signal::USART3_RX, h7::gpio::Speed::Low, h7::gpio::Pull::Up),
};
}  // namespace

namespace target {

void init() noexcept {
    // The MPU before the caches (it decides what is cacheable: SRAM1/2 not), then the caches:
    // the D-cache enable invalidates every line, and nothing that must survive has been written
    // through it yet (.data/.bss are written before main(), cache off).
    h7::configure_mpu();
    cache::enable_icache();
    cache::enable_dcache();
    h7::rcc::configure_clocks<clocks>();
    h7::gpio::configure<console_pins>();
    Console::configure<h7::UartConfig{.kernel_hz = clocks.pclk1_hz(), .baud = 115'200, .fifo = false}>();
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
