// SPDX-License-Identifier: MIT
//
// STM32H563 ADC1/ADC2, one regular channel: the shared STM32 driver (stm32/adc.hpp) with the
// H563's ADCs (RM0481 chapter ADC; addresses, clock gate, IRQ numbers: SVD v1.8).
//
//   constexpr AdcConfig pressure{.kernel_hz = board::clocks.hclk_hz(), .cpu_hz = board::sysclk_hz,
//                                .channel = 18, .sample = AdcSample::Cycles2_5};
//   using Pressure = Adc<AdcId::Adc1, pressure>;
//   Pressure::init();             // power up, calibrate, enable
//
// Kernel clock: RCC_CCIPR5.ADCDACSEL reset selection, HCLK. Channels ADC_INP0..19.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/adc.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::adc_clock_hz, stm32::adc_presc;
using stm32::AdcConfig, stm32::AdcSample;

enum class AdcId : std::uint8_t { Adc1, Adc2 };

[[nodiscard]] consteval stm32::AdcHw adc_hw(AdcId id) {
    return {.base = periph_alias + (id == AdcId::Adc1 ? 0x0202'8000u : 0x0202'8100u),
            .common = periph_alias + 0x0202'8300,
            .gate = rcc::gate::adc12,
            .irq = static_cast<std::uint8_t>(id == AdcId::Adc1 ? 37 : 69),
            .max_channel = 19,
            .select_kernel_clock = false};
}

template <AdcId Id, AdcConfig Config>
using Adc = stm32::Adc<adc_hw(Id), Config>;

}  // namespace rtos::port::stm32h5
