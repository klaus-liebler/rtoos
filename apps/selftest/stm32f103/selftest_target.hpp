// SPDX-License-Identifier: MIT
//
// Self-test target: bare STM32F103xB (Cortex-M3), no board package - the self-test sets up what it
// needs itself: SYSCLK 64 MHz from the HSI (no HSE needed: the emulators and any board run it),
// console USART1 on PA9 (TX) / PA10 (RX) at 115200 baud. Spare interrupts: TIM2, TIM3, TIM4,
// TIM1_UP (software-triggered).
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "rtos/port/cm3/port.hpp"
#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/clocks.hpp"

namespace target {

namespace f1 = rtos::port::stm32f1;

/// HSI / 2 = 4 MHz x 16 = 64 MHz (2 wait states), APB1 32 MHz.
inline constexpr f1::rcc::ClockTree clocks{.pll_mul = 16};
static_assert(clocks.sysclk_hz() == 64'000'000 && clocks.pclk1_hz() == 32'000'000);

using Port = rtos::port::cm3::Cm3Port;
inline constexpr std::uint32_t sysclk_hz = clocks.sysclk_hz();
inline constexpr std::string_view name = "STM32F103 (bare chip)";

inline constexpr std::array<unsigned, 4> spare_irqs{
    static_cast<unsigned>(f1::Irq::TIM2), static_cast<unsigned>(f1::Irq::TIM3),
    static_cast<unsigned>(f1::Irq::TIM4), static_cast<unsigned>(f1::Irq::TIM1_UP)};

void init() noexcept;
void print(std::string_view text) noexcept;
void print_dec(std::uint32_t value) noexcept;
void print_hex(std::uint32_t value) noexcept;
void flush() noexcept;
inline void led_toggle() noexcept {}

}  // namespace target
