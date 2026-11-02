// SPDX-License-Identifier: MIT
//
// Self-test target: bare STM32G431 (Cortex-M4), no board package - the self-test sets up what it
// needs itself: SYSCLK 170 MHz from HSI16 (no HSE needed), console USART1 on PC4 (TX) / PB7 (RX)
// at 115200 baud. Spare interrupts: TIM2, TIM3, TIM4, TIM7 (software-triggered).
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "rtos/port/cm4/port.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/clocks.hpp"

namespace target {

namespace g4 = rtos::port::stm32g4;

/// HSI16 / 4 = 4 MHz reference, x 85 = 340 MHz VCO, R / 2 = 170 MHz (range 1 boost, 4 wait states).
inline constexpr g4::rcc::ClockTree clocks{.pll = {.m = 4, .n = 85, .r = 2}};
static_assert(clocks.sysclk_hz() == 170'000'000);

using Port = rtos::port::cm4::Cm4Port;
inline constexpr std::uint32_t sysclk_hz = clocks.sysclk_hz();
inline constexpr std::string_view name = "STM32G431 (bare chip)";

inline constexpr std::array<unsigned, 4> spare_irqs{
    static_cast<unsigned>(g4::Irq::TIM2), static_cast<unsigned>(g4::Irq::TIM3),
    static_cast<unsigned>(g4::Irq::TIM4), static_cast<unsigned>(g4::Irq::TIM7)};

void init() noexcept;
void print(std::string_view text) noexcept;
void print_dec(std::uint32_t value) noexcept;
void print_hex(std::uint32_t value) noexcept;
void flush() noexcept;
inline void led_toggle() noexcept {}

}  // namespace target
