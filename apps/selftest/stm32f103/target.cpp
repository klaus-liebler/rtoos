// SPDX-License-Identifier: MIT
//
// Self-test, the STM32F103's part: kernel binding, clock and console, interrupt vectors, fault
// handlers, and the one chip mechanism the host tests cannot cover: a WS2812 bit stream on TIM1_CH1
// fed item by item by DMA1 channel 5 on the update event (timer + DMA request pacing). The Cortex-M3
// has no stack limit register; the other drivers are covered by the host tests and drivers_check.
#include <array>
#include <cstdint>

#include "rtos/devices/ws2812.hpp"
#include "rtos/port/cm3/port.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/stm32f1/dma.hpp"
#include "rtos/port/stm32f1/timer.hpp"
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

namespace {
using LedTimer = f1::Timer<f1::TimerId::Tim1, {.kernel_hz = target::clocks.timer2_hz(), .update_hz = 800'000}>;
using LedDma = f1::DmaChannel<{.request = f1::DmaRequest::Tim1Up, .direction = f1::DmaDirection::MemoryToPeripheral,
                               .memory_bytes = 2, .peripheral_bytes = 2, .priority = 2}>;
std::array<std::uint16_t, rtos::devices::ws2812_slots(5)> slots{};

// The bits leave one per timer period (1.25 us): right after the start nearly all are still to go,
// after 121 periods none; the last slot (0) leaves CCR1 at 0, the line low (WS2812 reset).
void test_ws2812_dma() {
    constexpr auto timing = rtos::devices::ws2812_timing(target::clocks.timer2_hz());
    const std::array<std::uint32_t, 5> colours{0xFF0000, 0x00FF00, 0x0000FF, 0x123456, 0xFFFFFF};
    rtos::devices::ws2812_encode(colours, slots, timing);
    LedTimer::init();
    LedTimer::configure_pwm<1>();
    LedTimer::set_compare<1>(0);
    LedDma::init();
    LedTimer::enable_update_dma();
    LedTimer::start();
    (void)LedDma::start(LedTimer::compare_address<1>(), slots.data(), slots.size());
    const std::size_t right_after = LedDma::remaining();
    const std::uint32_t t0 = rtos::port::cortex_m::CycleCounter::now();
    while (!LedDma::done() && rtos::port::cortex_m::CycleCounter::now() - t0 < target::sysclk_hz / 100) {
    }
    const bool done = LedDma::done();
    const std::size_t left = LedDma::remaining();
    const auto ccr1 = *reinterpret_cast<volatile std::uint32_t*>(LedTimer::compare_address<1>());
    LedDma::stop();
    LedTimer::stop();
    // no TIM1/DMA model: nothing moves (the counter keeps its value), or the registers read 0 (QEMU)
    if (!done && (left == slots.size() || right_after == 0)) {
        selftest::report_unsupported("WS2812 over TIM1/DMA (no TIM1/DMA model)");
        return;
    }
    selftest::report("WS2812 over TIM1/DMA: paced by the update event", done && left == 0 && right_after > slots.size() / 2 && ccr1 == 0);
}
}  // namespace

void selftest::target_tests() { test_ws2812_dma(); }

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
