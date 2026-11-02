// SPDX-License-Identifier: MIT
//
// STM32H563 timers TIM1-8, TIM15-17: the shared STM32 driver (stm32/timer.hpp) with the H563's
// instances (RM0481 chapters TIM; addresses, clock gates, IRQ numbers: SVD v1.8).
//
//   // PWM: 20 kHz, at least 1000 steps of duty resolution
//   using Fan = Timer<TimerId::Tim4, {.kernel_hz = board::clocks.timer1_hz(), .update_hz = 20'000, .min_steps = 1000}>;
//   Fan::init();  Fan::configure_pwm<3>();  Fan::start();  Fan::set_duty_permille<3>(400);
//
// Kernel clock: ClockTree::timer1_hz() for TIM2-7 (APB1), timer2_hz() for TIM1/8/15-17 (APB2).
// TIM2 and TIM5 count 32 bit, the others 16 bit. TIMx_DCR.DBSS selects the DMA burst event.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/timer.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/port/stm32h5/rcc.hpp"

namespace rtos::port::stm32h5 {

using stm32::timer_registers, stm32::timer_update_mhz, stm32::timer_update_within_1_percent;
using stm32::TimerConfig, stm32::TimerRegisters;

enum class TimerId : std::uint8_t { Tim1, Tim2, Tim3, Tim4, Tim5, Tim6, Tim7, Tim8, Tim15, Tim16, Tim17 };

[[nodiscard]] consteval stm32::TimerHw timer_hw(TimerId id) {
    // base, gate, update IRQ, channels, 32 bit, advanced (BDTR), DBSS
    switch (id) {
        case TimerId::Tim1: return {periph_alias + 0x0001'2C00, rcc::gate::tim1, 42, 4, false, true, true};
        case TimerId::Tim2: return {periph_alias + 0x0000'0000, rcc::gate::tim2, 45, 4, true, false, true};
        case TimerId::Tim3: return {periph_alias + 0x0000'0400, rcc::gate::tim3, 46, 4, false, false, true};
        case TimerId::Tim4: return {periph_alias + 0x0000'0800, rcc::gate::tim4, 47, 4, false, false, true};
        case TimerId::Tim5: return {periph_alias + 0x0000'0C00, rcc::gate::tim5, 48, 4, true, false, true};
        case TimerId::Tim6: return {periph_alias + 0x0000'1000, rcc::gate::tim6, 49, 0, false, false, true};
        case TimerId::Tim7: return {periph_alias + 0x0000'1400, rcc::gate::tim7, 50, 0, false, false, true};
        case TimerId::Tim8: return {periph_alias + 0x0001'3400, rcc::gate::tim8, 66, 4, false, true, true};
        case TimerId::Tim15: return {periph_alias + 0x0001'4000, rcc::gate::tim15, 71, 2, false, true, true};
        case TimerId::Tim16: return {periph_alias + 0x0001'4400, rcc::gate::tim16, 72, 1, false, true, true};
        case TimerId::Tim17: return {periph_alias + 0x0001'4800, rcc::gate::tim17, 73, 1, false, true, true};
    }
    return {};
}

template <TimerId Id, TimerConfig Config>
using Timer = stm32::Timer<timer_hw(Id), Config>;

/// A free-running 1 MHz clock on a 32-bit timer (TIM2, TIM5) with one deadline; satisfies
/// usb_pd::DeadlineTimer (stm32/timer.hpp).
template <TimerId Id, std::uint32_t KernelHz, unsigned Ch = 1>
using MicrosecondTimer = stm32::MicrosecondTimer<timer_hw(Id), KernelHz, Ch>;

}  // namespace rtos::port::stm32h5
