// SPDX-License-Identifier: MIT
//
// STM32H563 flash interface: read latency and programming delay
// (RM0481 "FLASH", SVD STM32H563 v1.8).
#pragma once

#include <cstdint>

#include "rtos/port/stm32h5/chip.hpp"

namespace rtos::port::stm32h5::flash {

using ACR = PReg<0x0002'2000 + 0x00>;  // reset 0x0000'0013: 3 wait states, WRHIGHFREQ 1

inline constexpr std::uint32_t acr_latency_msk = 0xFu << 0;
inline constexpr std::uint32_t acr_wrhighfreq_pos = 4;
inline constexpr std::uint32_t acr_wrhighfreq_msk = 3u << acr_wrhighfreq_pos;

/// Wait states for HCLK in voltage scale VOS0 (RM0481 table "Number of wait states
/// according to flash clock frequency"; same limits as UTILS_SCALE0_LATENCYx_FREQ in
/// ST's stm32h5xx_ll_utils.c): 42/84/126/168/210/250 MHz for 0..5 wait states.
[[nodiscard]] constexpr std::uint32_t vos0_wait_states(std::uint32_t hclk_hz) noexcept {
    return hclk_hz <= 42'000'000 ? 0 : (hclk_hz + 41'999'999) / 42'000'000 - 1;
}

/// Programming delay WRHIGHFREQ: 0 up to 84 MHz, 1 up to 168 MHz, 2 up to 250 MHz.
[[nodiscard]] constexpr std::uint32_t programming_delay(std::uint32_t hclk_hz) noexcept {
    return hclk_hz <= 84'000'000 ? 0 : (hclk_hz <= 168'000'000 ? 1 : 2);
}

static_assert(vos0_wait_states(250'000'000) == 5 && vos0_wait_states(210'000'000) == 4 &&
              vos0_wait_states(42'000'000) == 0 && vos0_wait_states(42'000'001) == 1);

/// Set wait states and programming delay; returns once the new latency is active.
/// Call before raising the clock (and after lowering it).
inline void set_latency(std::uint32_t wait_states, std::uint32_t delay) noexcept {
    ACR::write((ACR::read() & ~(acr_latency_msk | acr_wrhighfreq_msk)) | wait_states | (delay << acr_wrhighfreq_pos));
    while ((ACR::read() & acr_latency_msk) != wait_states) {
    }
}

}  // namespace rtos::port::stm32h5::flash
