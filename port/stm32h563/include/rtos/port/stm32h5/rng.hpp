// SPDX-License-Identifier: MIT
//
// STM32H5 true random number generator: the shared STM32 driver (stm32/rng.hpp) with the H563's
// RNG at 0x420C0800 (ST SVD STM32H563 v1.8). Kernel clock: HSI48 (RCC_CCIPR5.RNGSEL reset value 00).
//
//   Rng::init();  std::uint32_t r = Rng::next();
#pragma once

#include "rtos/port/stm32/rng.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

inline constexpr stm32::RngHw rng_hw{.base = periph_alias + 0x020C'0800, .gate = rcc::gate::rng, .clock_on = &rcc::enable_hsi48};

using Rng = stm32::Rng<rng_hw>;

}  // namespace rtos::port::stm32h5
