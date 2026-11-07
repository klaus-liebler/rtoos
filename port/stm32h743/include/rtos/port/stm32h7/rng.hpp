// SPDX-License-Identifier: MIT
//
// STM32H743 true random number generator: the shared STM32 driver (stm32/rng.hpp) with the
// H743's RNG at 0x4802'1800 (SVD v2.4: CR, SR, DR only - the H5's configuration fields do not
// exist). Kernel clock: HSI48 (RCC_D2CCIP2R.RNGSEL reset value 00, ST's stm32h7xx_ll_rcc.h).
//
//   Rng::init();  std::uint32_t r = Rng::next();
#pragma once

#include "rtos/port/stm32/rng.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

inline constexpr stm32::RngHw rng_hw{.base = 0x4802'1800, .gate = rcc::gate::rng, .clock_on = &rcc::enable_hsi48};

using Rng = stm32::Rng<rng_hw>;

}  // namespace rtos::port::stm32h7
