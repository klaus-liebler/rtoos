// SPDX-License-Identifier: MIT
//
// STM32F103 flash interface: access control (wait states, prefetch). RM0008 rev 21, 3.3.3
// "Embedded flash memory" and FLASH_ACR (PM0075).
#pragma once

#include <cstdint>

#include "rtos/port/stm32f1/chip.hpp"

namespace rtos::port::stm32f1::flash {

using ACR = PReg<0x0002'2000 + 0x00>;  // reset 0x30: prefetch on, 0 wait states

inline constexpr std::uint32_t acr_latency_msk = 7u << 0, acr_hlfcya = 1u << 3, acr_prftbe = 1u << 4,
                               acr_prftbs = 1u << 5;

/// Wait states for SYSCLK (RM0008 3.3.3): 0 up to 24 MHz, 1 up to 48 MHz, 2 up to 72 MHz.
[[nodiscard]] constexpr std::uint32_t wait_states(std::uint32_t sysclk_hz) noexcept {
    return sysclk_hz <= 24'000'000 ? 0 : (sysclk_hz <= 48'000'000 ? 1 : 2);
}

/// Wait states and prefetch buffer on (RM0008: the prefetch buffer must stay on when the AHB
/// prescaler is not 1; on is also the fast setting). Set before raising, after lowering SYSCLK.
inline void set_latency(std::uint32_t wait_states) noexcept {
    ACR::write((ACR::read() & ~(acr_latency_msk | acr_hlfcya)) | acr_prftbe | wait_states);
    while ((ACR::read() & acr_latency_msk) != wait_states) {
    }
}

}  // namespace rtos::port::stm32f1::flash
