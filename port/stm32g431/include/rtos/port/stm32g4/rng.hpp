// SPDX-License-Identifier: MIT
//
// STM32G431 true random number generator: the shared STM32 driver (stm32/rng.hpp) with the G431's
// RNG at 0x50060800 (SVD v1.8). Kernel clock: HSI48 (RCC_CCIPR.CLK48SEL reset value 00).
//
//   Rng::init();  std::uint32_t r = Rng::next();
#pragma once

#include "rtos/port/stm32/rng.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

inline constexpr stm32::RngHw rng_hw{.base = 0x5006'0800, .gate = rcc::gate::rng, .clock_on = &rcc::enable_hsi48};

using Rng = stm32::Rng<rng_hw>;

}  // namespace rtos::port::stm32g4
