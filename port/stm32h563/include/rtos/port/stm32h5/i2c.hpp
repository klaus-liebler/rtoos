// SPDX-License-Identifier: MIT
//
// STM32H563 I2C master: the shared STM32 driver (stm32/i2c.hpp) with the H563's instances
// (RM0481 chapter I2C; addresses, clock gates, IRQ numbers: SVD v1.8).
//
//   using Bus = I2cMaster<I2cId::I2c4, {.kernel_hz = board::clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz,
//                                       .bus_hz = 100'000}>;
//   Bus::init();
//
// Kernel clock: the RCC_CCIPR4 reset selection, PCLK1 (I2C1/2) or PCLK3 (I2C3/4).
#pragma once

#include <cstdint>

#include "rtos/port/stm32/i2c.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::checked_i2c_timing, stm32::i2c_frequency, stm32::i2c_timing;
using stm32::I2cConfig, stm32::I2cPolled, stm32::I2cResult, stm32::I2cTiming, stm32::I2cWait;

enum class I2cId : std::uint8_t { I2c1, I2c2, I2c3, I2c4 };

[[nodiscard]] constexpr stm32::I2cHw i2c_hw(I2cId id) noexcept {
    switch (id) {
        case I2cId::I2c1: return {periph_alias + 0x0000'5400, rcc::gate::i2c1, 51, 52};
        case I2cId::I2c2: return {periph_alias + 0x0000'5800, rcc::gate::i2c2, 53, 54};
        case I2cId::I2c3: return {periph_alias + 0x0400'2800, rcc::gate::i2c3, 80, 81};
        case I2cId::I2c4: return {periph_alias + 0x0400'2C00, rcc::gate::i2c4, 125, 126};
    }
    return {};
}

template <I2cId Id, I2cConfig Config, class Wait = I2cPolled>
using I2cMaster = stm32::I2cMaster<i2c_hw(Id), Config, Wait>;

}  // namespace rtos::port::stm32h5
