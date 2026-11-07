// SPDX-License-Identifier: MIT
//
// Self-test target: bare STM32H743 (Cortex-M7), no board package - the self-test sets up what it
// needs itself: CPU 400 MHz from the HSI (no HSE needed), the default MPU map (stm32h7/mpu.hpp),
// I- and D-cache on, console USART3 on PD8 (TX) / PD9 (RX) at 115200 baud (the FCU revision 3's
// console). Spare interrupts: TIM2, TIM3, TIM4, TIM7 (software-triggered).
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "rtos/port/cm7/port.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/clocks.hpp"

namespace target {

namespace h7 = rtos::port::stm32h7;

/// HSI 64 MHz / 4 = 16 MHz reference, x 50 = 800 MHz VCO; P / 2 = 400 MHz CPU (VOS1),
/// HCLK 200 MHz (2 wait states), APB1-4 100 MHz.
inline constexpr h7::rcc::ClockTree clocks{.pll1 = {.m = 4, .n = 50, .p = 2, .q = 8}};
static_assert(clocks.cpu_hz() == 400'000'000 && clocks.hclk_hz() == 200'000'000 && clocks.pclk1_hz() == 100'000'000);

using Port = rtos::port::cm7::Cm7Port;
inline constexpr std::uint32_t sysclk_hz = clocks.cpu_hz();
inline constexpr std::string_view name = "STM32H743 (bare chip)";

inline constexpr std::array<unsigned, 4> spare_irqs{
    static_cast<unsigned>(h7::Irq::TIM2), static_cast<unsigned>(h7::Irq::TIM3),
    static_cast<unsigned>(h7::Irq::TIM4), static_cast<unsigned>(h7::Irq::TIM7)};

void init() noexcept;
void print(std::string_view text) noexcept;
void print_dec(std::uint32_t value) noexcept;
void print_hex(std::uint32_t value) noexcept;
void flush() noexcept;
inline void led_toggle() noexcept {}

}  // namespace target
