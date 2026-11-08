// SPDX-License-Identifier: MIT
//
// STM32H743xI chip constants. Addresses and bit positions: ST SVD STM32H743 v2.4 (from
// STM32CubeProgrammer); memory map, interrupt numbers and core configuration: ST's
// stm32h743xx.h (cmsis_device_h7); reference manual RM0433.
#pragma once

#include <cstddef>
#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"

namespace rtos::port::stm32h7 {

/// Peripheral register at an absolute address (the H7's buses are spread over 0x4000'0000 ..
/// 0x5C00'0000: D2 APB1 0x4000'0000, D2 APB2 0x4001'0000, D1 AHB 0x5200'0000, D3 0x5800'0000).
template <std::uintptr_t Address>
using Reg = cortex_m::Reg<Address>;

inline constexpr std::uint32_t hsi_hz = 64'000'000;  ///< HSI, HSIDIV = 1
inline constexpr std::uint32_t hsi48_hz = 48'000'000;

inline constexpr std::uintptr_t flash_base = 0x0800'0000;
inline constexpr std::size_t flash_bytes = 2 * 1024 * 1024;  // STM32H743xI: two banks of 1 MB
inline constexpr std::size_t flash_sector_bytes = 128 * 1024;
/// RAM (stm32h743xx.h): ITCM 64 KB, DTCM 128 KB (CPU only, no DMA1/2 access), AXI SRAM 512 KB
/// (D1), SRAM1-3 128 + 128 + 32 KB (D2), SRAM4 64 KB (D3).
inline constexpr std::uintptr_t itcm_base = 0x0000'0000, dtcm_base = 0x2000'0000, axi_sram_base = 0x2400'0000,
                                sram1_base = 0x3000'0000, sram4_base = 0x3800'0000;
inline constexpr std::size_t itcm_bytes = 64 * 1024, dtcm_bytes = 128 * 1024, axi_sram_bytes = 512 * 1024,
                             d2_sram_bytes = 288 * 1024, sram4_bytes = 64 * 1024;
/// SDMMC1's buffers: SDMMC1 is a D1 bus master and its IDMA reaches the AXI SRAM, QUADSPI and FMC
/// only - not the DTCM, not SRAM1-4 (ST AN5200). The first 32 KB of the AXI SRAM are kept for it,
/// non-cacheable (stm32h743xi.ld RAM_SDMMC, .sdmmc_buffer; mpu.hpp region R4).
inline constexpr std::uintptr_t sdmmc_buffer_base = axi_sram_base;
inline constexpr std::size_t sdmmc_buffer_bytes = 32 * 1024;
/// 96-bit unique device ID (stm32h743xx.h UID_BASE).
inline constexpr std::uintptr_t uid_base = 0x1FF1'E800;

/// NVIC interrupt numbers used by the kernel ports/boards (full list: vectors.cpp).
enum class Irq : std::uint8_t {
    TIM2 = 28,
    TIM3 = 29,
    TIM4 = 30,
    USART1 = 37,
    USART2 = 38,
    USART3 = 39,
    TIM5 = 50,
    TIM7 = 55,
};

}  // namespace rtos::port::stm32h7
