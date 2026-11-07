// SPDX-License-Identifier: MIT
//
// STM32H743 I2C master: the shared STM32 driver (stm32/i2c.hpp) with the H743's instances
// (RM0433 chapter I2C; addresses, clock gates: SVD v2.4; IRQ numbers: ST's stm32h743xx.h).
//
//   using Bus = I2cMaster<I2cId::I2c4, {.kernel_hz = clocks.pclk4_hz(), .cpu_hz = clocks.cpu_hz(),
//                                       .bus_hz = 100'000}>;
//   Bus::init();
//
// Kernel clock: the reset selection (ST's stm32h7xx_ll_rcc.h): PCLK1 for I2C1-3
// (RCC_D2CCIP2R.I2C123SEL), PCLK4 for I2C4 (RCC_D3CCIPR.I2C4SEL).
#pragma once

#include <cstdint>

#include "rtos/port/stm32/i2c.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::checked_i2c_timing, stm32::i2c_frequency, stm32::i2c_timing;
using stm32::I2cConfig, stm32::I2cPolled, stm32::I2cResult, stm32::I2cTiming, stm32::I2cWait;

enum class I2cId : std::uint8_t { I2c1, I2c2, I2c3, I2c4 };

[[nodiscard]] constexpr stm32::I2cHw i2c_hw(I2cId id) noexcept {
    switch (id) {
        case I2cId::I2c1: return {0x4000'5400, rcc::gate::i2c1, 31, 32};
        case I2cId::I2c2: return {0x4000'5800, rcc::gate::i2c2, 33, 34};
        case I2cId::I2c3: return {0x4000'5C00, rcc::gate::i2c3, 72, 73};
        case I2cId::I2c4: return {0x5800'1C00, rcc::gate::i2c4, 95, 96};
    }
    return {};
}

template <I2cId Id, I2cConfig Config, class Wait = I2cPolled>
using I2cMaster = stm32::I2cMaster<i2c_hw(Id), Config, Wait>;

}  // namespace rtos::port::stm32h7
