// SPDX-License-Identifier: MIT
//
// STM32G431 flash interface: read latency, prefetch and the ART caches (RM0440 "FLASH",
// SVD STM32G431 v1.8; wait-state limits: ST's stm32g4xx_ll_utils.c, UTILS_SCALE1_LATENCYx_FREQ
// and UTILS_SCALE1_LATENCYx_BOOST_FREQ).
#pragma once

#include <cstdint>

#include "rtos/port/stm32g4/chip.hpp"

namespace rtos::port::stm32g4::flash {

using ACR = PReg<0x0002'2000 + 0x00>;  // reset 0x0004'0601: 1 wait state, ICEN, DCEN

inline constexpr std::uint32_t acr_latency_msk = 0xFu << 0;
inline constexpr std::uint32_t acr_prften = 1u << 8, acr_icen = 1u << 9, acr_dcen = 1u << 10;

/// Wait states for HCLK in range 1: boost mode 34/68/102/136/170 MHz, normal mode
/// 30/60/90/120/150 MHz for 0..4 wait states.
[[nodiscard]] constexpr std::uint32_t range1_wait_states(std::uint32_t hclk_hz, bool boost) noexcept {
    const std::uint32_t step = boost ? 34'000'000 : 30'000'000;
    return hclk_hz <= step ? 0 : (hclk_hz + step - 1) / step - 1;
}

static_assert(range1_wait_states(170'000'000, true) == 4 && range1_wait_states(136'000'001, true) == 4 &&
              range1_wait_states(136'000'000, true) == 3 && range1_wait_states(34'000'000, true) == 0 &&
              range1_wait_states(150'000'000, false) == 4 && range1_wait_states(30'000'001, false) == 1 &&
              range1_wait_states(16'000'000, false) == 0);

/// Set the wait states (with prefetch and both caches on); returns once the latency is
/// active. Call before raising the clock (and after lowering it).
inline void set_latency(std::uint32_t wait_states) noexcept {
    ACR::write((ACR::read() & ~acr_latency_msk) | wait_states | acr_prften | acr_icen | acr_dcen);
    while ((ACR::read() & acr_latency_msk) != wait_states) {
    }
}

}  // namespace rtos::port::stm32g4::flash
