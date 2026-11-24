// SPDX-License-Identifier: MIT
//
// STM32F103x8/xB (medium density) chip constants. Addresses, bit positions and IRQ numbers:
// RM0008 rev 21 and ST's CMSIS device header stm32f103xb.h; memory sizes: datasheet DS5319.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"

namespace rtos::port::stm32f1 {

/// Peripheral register at `periph_base + offset` (APB1 0x4000'0000, APB2 0x4001'0000,
/// AHB 0x4001'8000 / 0x4002'0000).
inline constexpr std::uintptr_t periph_base = 0x4000'0000;
template <std::uintptr_t Offset>
using PReg = cortex_m::Reg<periph_base + Offset>;

inline constexpr std::uint32_t hsi_hz = 8'000'000;
/// LSI: 30..60 kHz (DS5319 table "LSI oscillator characteristics"), typically 40 kHz.
inline constexpr std::uint32_t lsi_min_hz = 30'000, lsi_typ_hz = 40'000, lsi_max_hz = 60'000;

inline constexpr std::size_t flash_bytes = 128 * 1024;  // STM32F103xB (the x8 has 64 KB)
inline constexpr std::uintptr_t flash_base = 0x0800'0000;
inline constexpr std::size_t flash_page_bytes = 1024;   // medium density
inline constexpr std::uintptr_t sram_base = 0x2000'0000;
inline constexpr std::size_t sram_bytes = 20 * 1024;
/// 96-bit unique device ID (RM0008 30.2) and the flash size in KB (30.1).
inline constexpr std::uintptr_t uid_base = 0x1FFF'F7E8;
inline constexpr std::uintptr_t flash_size_kb_address = 0x1FFF'F7E0;

/// NVIC interrupt numbers used by drivers and boards (full list: vectors.cpp). The NVIC
/// implements 4 priority bits.
enum class Irq : std::uint8_t {
    USB_HP_CAN1_TX = 19,
    USB_LP_CAN1_RX0 = 20,
    CAN1_RX1 = 21,
    CAN1_SCE = 22,
    TIM1_UP = 25,
    TIM1_CC = 27,
    TIM2 = 28,
    TIM3 = 29,
    TIM4 = 30,
    I2C1_EV = 31,
    I2C1_ER = 32,
    I2C2_EV = 33,
    I2C2_ER = 34,
    USART1 = 37,
    USART2 = 38,
    USART3 = 39,
};

}  // namespace rtos::port::stm32f1
