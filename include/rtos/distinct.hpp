// SPDX-License-Identifier: MIT
//
// Compile-time resource checks: no pin, EXTI line, peripheral instance, DMA channel or bus address
// is used twice. The drivers name what they occupy - the peripheral's base address (Timer::base,
// Usart::base, I2cMaster::base, SpiMaster::base, Ucpd::base), DmaChannel::id, ExtiLine::line,
// a device's ::bus and ::address -; a firmware lists its resources once and lets the compiler
// compare them:
//
//   static_assert(rtos::distinct(std::array{hw::Barrier1::line, hw::Diag1::line}), "two EXTI interrupts on one line");
//   static_assert(rtos::distinct(std::array{hw::MotorPwm::base, hw::StepTimer::base}), "a timer used twice");
//
// Every resource has exactly one owner; nothing here models sharing. Pins across several
// configuration lists: gpio::pins_distinct() (rtos/port/stm32/gpio.hpp).
#pragma once

#include <array>
#include <cstddef>

namespace rtos {

/// No value twice.
template <class T, std::size_t N>
[[nodiscard]] consteval bool distinct(const std::array<T, N>& values) {
    for (std::size_t i = 0; i < N; ++i) {
        for (std::size_t j = i + 1; j < N; ++j) {
            if (values[i] == values[j]) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace rtos
