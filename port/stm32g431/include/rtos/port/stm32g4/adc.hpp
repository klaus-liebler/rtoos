// SPDX-License-Identifier: MIT
//
// STM32G431 ADC1/ADC2, one regular channel: the shared STM32 driver (stm32/adc.hpp) with the
// G431's ADCs at 0x50000000/0x50000100, common registers at 0x50000300, IRQ ADC1_2 = 18 (RM0440
// chapter ADC, SVD v1.8, ST's stm32g431xx.h).
//
//   constexpr AdcConfig supply{.kernel_hz = clocks.sysclk_hz(), .cpu_hz = clocks.sysclk_hz(), .channel = 12};
//   using Supply = Adc<AdcId::Adc2, supply>;
//   Supply::init();             // kernel clock, power up, calibrate, enable
//
// Kernel clock: init() selects RCC_CCIPR.ADC12SEL = SYSCLK (the reset choice is no clock);
// the ADC clock prescaler keeps it at or below AdcConfig::max_adc_hz. Channels ADC_INP0..18.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/adc.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::adc_clock_hz, stm32::adc_presc;
using stm32::AdcConfig, stm32::AdcSample;

enum class AdcId : std::uint8_t { Adc1, Adc2 };

[[nodiscard]] consteval stm32::AdcHw adc_hw(AdcId id) {
    return {.base = id == AdcId::Adc1 ? 0x5000'0000u : 0x5000'0100u,
            .common = 0x5000'0300,
            .gate = rcc::gate::adc12,
            .irq = 18,
            .max_channel = 18,
            .select_kernel_clock = true,
            .kernel_clock = {rcc::kernel_clock::adc12, rcc::kernel_clock::adc12_sysclk}};
}

template <AdcId Id, AdcConfig Config>
using Adc = stm32::Adc<adc_hw(Id), Config>;

}  // namespace rtos::port::stm32g4
