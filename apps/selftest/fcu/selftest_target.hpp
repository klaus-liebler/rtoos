// SPDX-License-Identifier: MIT
//
// Self-test target: Factory Control Unit (STM32H563, Cortex-M33). Console: board (USART3).
// Spare interrupts: TIM2..TIM5 (software-triggered; the timers themselves stay off).
#pragma once

#include <array>
#include <cstdint>
#include <string_view>

#include "board/fcu.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/chip.hpp"

namespace target {

using Port = rtos::port::cm33::Cm33Port;
inline constexpr std::uint32_t sysclk_hz = board::sysclk_hz;
inline constexpr std::string_view name = "STM32H563 / Factory Control Unit";

namespace h5 = rtos::port::stm32h5;
inline constexpr std::array<unsigned, 4> spare_irqs{
    static_cast<unsigned>(h5::Irq::TIM2), static_cast<unsigned>(h5::Irq::TIM3),
    static_cast<unsigned>(h5::Irq::TIM4), static_cast<unsigned>(h5::Irq::TIM5)};

inline void init() noexcept { board::init(); }
inline void print(std::string_view text) noexcept { board::print(text); }
inline void print_dec(std::uint32_t value) noexcept { board::print_dec(value); }
inline void print_hex(std::uint32_t value) noexcept { board::print_hex(value); }
inline void flush() noexcept { board::flush(); }
inline void led_toggle() noexcept { board::led_info_toggle(); }

}  // namespace target
