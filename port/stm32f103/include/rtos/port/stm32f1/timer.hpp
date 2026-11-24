// SPDX-License-Identifier: MIT
//
// STM32F103 timers TIM1-4: the shared STM32 driver (stm32/timer.hpp) with the F103's instances
// (RM0008 rev 21 chapters 14/15; addresses, clock gates, IRQ numbers: ST's stm32f103xb.h).
//
//   using Pwm = Timer<TimerId::Tim4, {.kernel_hz = clocks.timer1_hz(), .update_hz = 1'000, .min_steps = 1000}>;
//   Pwm::init();  Pwm::configure_pwm<1>();  Pwm::start();  Pwm::set_duty_permille<1>(400);
//
//   using Knob = Timer<TimerId::Tim2, {.kernel_hz = clocks.timer1_hz(), .tick_hz = clocks.timer1_hz()}>;
//   Knob::init();  Knob::configure_encoder<EncoderMode::Ti1, 15, true>();  Knob::start();
//
// Kernel clock: ClockTree::timer1_hz() for TIM2-4 (APB1), timer2_hz() for TIM1 (APB2). All count
// 16 bit. TIMx_DCR/DMAR at 0x48/0x4C (like the H7). TIM1 is the advanced timer (BDTR.MOE).
//
// Errata (ES096 2.6.x): break/OCREF_CLR, TRGO before the slave clock, compare toggle at ARR/0,
// overcapture flag - the driver uses none of these features.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/timer.hpp"
#include "rtos/port/stm32f1/chip.hpp"
#include "rtos/port/stm32f1/rcc.hpp"

namespace rtos::port::stm32f1 {

using stm32::timer_encoder_ccmr1, stm32::timer_registers, stm32::timer_update_mhz, stm32::timer_update_within_1_percent;
using stm32::EncoderMode, stm32::TimerConfig, stm32::TimerRegisters;

enum class TimerId : std::uint8_t { Tim1, Tim2, Tim3, Tim4 };

[[nodiscard]] consteval stm32::TimerHw timer_hw(TimerId id) {
    // base, gate, update IRQ, channels, 32 bit, advanced (BDTR), DBSS, DCR offset
    switch (id) {
        case TimerId::Tim1: return {periph_base + 0x0001'2C00, rcc::gate::tim1, 25, 4, false, true, false, 0x048};
        case TimerId::Tim2: return {periph_base + 0x0000'0000, rcc::gate::tim2, 28, 4, false, false, false, 0x048};
        case TimerId::Tim3: return {periph_base + 0x0000'0400, rcc::gate::tim3, 29, 4, false, false, false, 0x048};
        case TimerId::Tim4: return {periph_base + 0x0000'0800, rcc::gate::tim4, 30, 4, false, false, false, 0x048};
    }
    return {};
}

template <TimerId Id, TimerConfig Config>
using Timer = stm32::Timer<timer_hw(Id), Config>;

}  // namespace rtos::port::stm32f1
