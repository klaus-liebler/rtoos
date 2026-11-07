// SPDX-License-Identifier: MIT
//
// STM32H743 flash interface: read latency and programming delay (RM0433 "Embedded flash
// memory", SVD STM32H743 v2.4 FLASH_ACR; wait states per AXI clock: STM32CubeMX db
// IP/RCC-STM32H7_rcc_v1_0_Modes.xml, FLatency for scale1 - stricter above 140 MHz than ST's
// stm32h7xx_ll_utils.c, which allows 2 wait states up to 240 MHz; programming delay:
// stm32h7xx_hal_flash_ex.h FLASH_PROGRAMMING_DELAY_x).
#pragma once

#include <cstdint>

#include "rtos/port/stm32h7/chip.hpp"

namespace rtos::port::stm32h7::flash {

inline constexpr std::uintptr_t base = 0x5200'2000;
using ACR = Reg<base + 0x000>;  // reset 0x0000'0037: 7 wait states, WRHIGHFREQ 3

inline constexpr std::uint32_t acr_latency_msk = 0xFu << 0, acr_wrhighfreq_pos = 4, acr_wrhighfreq_msk = 3u << 4;

/// Wait states for the AXI clock (HCLK) in VOS1: 0 up to 70 MHz, 1 up to 140, 2 up to 210,
/// 3 up to 225 MHz; 0xFF above (not allowed).
[[nodiscard]] constexpr std::uint32_t vos1_wait_states(std::uint32_t hclk_hz) noexcept {
    return hclk_hz <= 70'000'000 ? 0 : hclk_hz <= 140'000'000 ? 1 : hclk_hz <= 210'000'000 ? 2 : hclk_hz <= 225'000'000 ? 3 : 0xFF;
}
/// WRHIGHFREQ for the AXI clock: 0 up to 70 MHz, 1 up to 185 MHz, 2 up to 225 MHz.
[[nodiscard]] constexpr std::uint32_t programming_delay(std::uint32_t hclk_hz) noexcept {
    return hclk_hz <= 70'000'000 ? 0 : hclk_hz <= 185'000'000 ? 1 : 2;
}

static_assert(vos1_wait_states(200'000'000) == 2 && vos1_wait_states(210'000'001) == 3 &&
              vos1_wait_states(140'000'000) == 1 && vos1_wait_states(70'000'000) == 0 &&
              vos1_wait_states(225'000'001) == 0xFF && programming_delay(200'000'000) == 2 &&
              programming_delay(185'000'000) == 1);

/// Set wait states and programming delay; returns once the latency is active. Call before
/// raising the clock (and after lowering it).
inline void set_latency(std::uint32_t wait_states, std::uint32_t delay) noexcept {
    ACR::write((ACR::read() & ~(acr_latency_msk | acr_wrhighfreq_msk)) | wait_states | (delay << acr_wrhighfreq_pos));
    while ((ACR::read() & acr_latency_msk) != wait_states) {
    }
}

}  // namespace rtos::port::stm32h7::flash
