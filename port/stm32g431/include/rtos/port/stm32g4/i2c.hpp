// SPDX-License-Identifier: MIT
//
// STM32G431 I2C master: the shared STM32 driver (stm32/i2c.hpp) with the G431's instances
// (RM0440 chapter I2C; addresses, clock gates: SVD v1.8; IRQ numbers: ST's stm32g431xx.h).
//
//   using Bus = I2cMaster<I2cId::I2c2, {.kernel_hz = clocks.pclk1_hz(), .cpu_hz = clocks.sysclk_hz()}>;
//   Bus::init();
//
// Kernel clock: the RCC_CCIPR reset selection, PCLK1 (I2C1-3).
#pragma once

#include <cstdint>

#include "rtos/port/stm32/i2c.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::checked_i2c_timing, stm32::i2c_frequency, stm32::i2c_timing;
using stm32::I2cConfig, stm32::I2cResult, stm32::I2cTiming;

enum class I2cId : std::uint8_t { I2c1, I2c2, I2c3 };

[[nodiscard]] constexpr stm32::I2cHw i2c_hw(I2cId id) noexcept {
    switch (id) {
        case I2cId::I2c1: return {periph_base + 0x0000'5400, rcc::gate::i2c1, 31, 32};
        case I2cId::I2c2: return {periph_base + 0x0000'5800, rcc::gate::i2c2, 33, 34};
        case I2cId::I2c3: return {periph_base + 0x0000'7800, rcc::gate::i2c3, 92, 93};
    }
    return {};
}

template <I2cId Id, I2cConfig Config>
using I2cMaster = stm32::I2cMaster<i2c_hw(Id), Config>;

}  // namespace rtos::port::stm32g4
