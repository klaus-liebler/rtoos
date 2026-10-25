// SPDX-License-Identifier: MIT
//
// STM32H563 chip constants. Addresses and bit positions: ST SVD STM32H563 v1.8,
// cross-checked against RM0481.
#pragma once

#include <cstdint>

#include "rtos/port/cm33/registers.hpp"

namespace rtos::port::stm32h5 {

#if defined(__ARM_FEATURE_CMSE) && (__ARM_FEATURE_CMSE == 3)
inline constexpr std::uintptr_t periph_alias = 0x5000'0000;  // Secure aliases
#else
inline constexpr std::uintptr_t periph_alias = 0x4000'0000;  // Non-secure aliases
#endif

/// Peripheral register at `periph_alias + offset`.
template <std::uintptr_t Offset>
using PReg = cm33::Reg<periph_alias + Offset>;

inline constexpr std::uint32_t hsi_hz = 64'000'000;

inline constexpr std::size_t flash_bytes = 2 * 1024 * 1024;  // STM32H563xI
inline constexpr std::uintptr_t flash_base = 0x0800'0000;
inline constexpr std::uintptr_t sram_base = 0x2000'0000;     // SRAM1|SRAM2|SRAM3 contiguous
inline constexpr std::size_t sram_bytes = 640 * 1024;

/// NVIC interrupt numbers used by the kernel ports/boards (full list: vectors.cpp).
enum class Irq : std::uint8_t {
    TIM2 = 45,
    TIM3 = 46,
    TIM4 = 47,
    TIM5 = 48,
    USART1 = 58,
    USART2 = 59,
    USART3 = 60,
};

}  // namespace rtos::port::stm32h5
