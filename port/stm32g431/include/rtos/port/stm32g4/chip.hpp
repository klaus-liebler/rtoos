// SPDX-License-Identifier: MIT
//
// STM32G431 chip constants. Addresses and bit positions: ST SVD STM32G431 v1.8 (the STM32G4
// superset; which peripherals the G431 has: ST's stm32g431xx.h), cross-checked against RM0440.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"

namespace rtos::port::stm32g4 {

/// Peripheral register at `periph_base + offset` (APB1 0x4000'0000, APB2 0x4001'0000,
/// AHB1 0x4002'0000, AHB2 0x4800'0000 / 0x5000'0000).
inline constexpr std::uintptr_t periph_base = 0x4000'0000;
template <std::uintptr_t Offset>
using PReg = cortex_m::Reg<periph_base + Offset>;

inline constexpr std::uint32_t hsi16_hz = 16'000'000;
inline constexpr std::uint32_t hsi48_hz = 48'000'000;

inline constexpr std::size_t flash_bytes = 128 * 1024;  // STM32G431xB
inline constexpr std::uintptr_t flash_base = 0x0800'0000;
/// SRAM1 16 KB + SRAM2 6 KB + CCM SRAM 10 KB (aliased at 0x2000'5800), contiguous.
inline constexpr std::uintptr_t sram_base = 0x2000'0000;
inline constexpr std::size_t sram_bytes = 32 * 1024;
/// 96-bit unique device ID (stm32g431xx.h UID_BASE).
inline constexpr std::uintptr_t uid_base = 0x1FFF'7590;

/// NVIC interrupt numbers used by the kernel ports/boards (full list: vectors.cpp).
enum class Irq : std::uint8_t {
    TIM2 = 28,
    TIM3 = 29,
    TIM4 = 30,
    USART1 = 37,
    USART2 = 38,
    USART3 = 39,
    TIM7 = 55,
};

}  // namespace rtos::port::stm32g4
