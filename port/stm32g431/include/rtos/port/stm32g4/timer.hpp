// SPDX-License-Identifier: MIT
//
// STM32G431 timers TIM1-4, TIM6-8, TIM15-17: the shared STM32 driver (stm32/timer.hpp) with the
// G431's instances (RM0440 chapters TIM; addresses, clock gates: SVD v1.8; which timers exist and
// their IRQ numbers: ST's stm32g431xx.h - TIM15/16/17 share the TIM1 BRK/UP/TRG_COM vectors).
//
//   using Fan = Timer<TimerId::Tim3, {.kernel_hz = clocks.timer1_hz(), .update_hz = 20'000, .min_steps = 1000}>;
//   Fan::init();  Fan::configure_pwm<1>();  Fan::start();  Fan::set_duty_permille<1>(400);
//
// Kernel clock: ClockTree::timer1_hz() for TIM2-7 (APB1), timer2_hz() for TIM1/8/15-17 (APB2).
// TIM2 counts 32 bit, the others 16 bit. No TIMx_DCR.DBSS: a DMA burst follows the request of
// the DMA channel that serves DMAR.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/timer.hpp"
#include "rtos/port/stm32g4/chip.hpp"
#include "rtos/port/stm32g4/rcc.hpp"

namespace rtos::port::stm32g4 {

using stm32::timer_registers, stm32::timer_update_mhz, stm32::timer_update_within_1_percent;
using stm32::TimerConfig, stm32::TimerRegisters;

enum class TimerId : std::uint8_t { Tim1, Tim2, Tim3, Tim4, Tim6, Tim7, Tim8, Tim15, Tim16, Tim17 };

[[nodiscard]] consteval stm32::TimerHw timer_hw(TimerId id) {
    // base, gate, update IRQ, channels, 32 bit, advanced (BDTR), DBSS
    switch (id) {
        case TimerId::Tim1: return {periph_base + 0x0001'2C00, rcc::gate::tim1, 25, 4, false, true, false};
        case TimerId::Tim2: return {periph_base + 0x0000'0000, rcc::gate::tim2, 28, 4, true, false, false};
        case TimerId::Tim3: return {periph_base + 0x0000'0400, rcc::gate::tim3, 29, 4, false, false, false};
        case TimerId::Tim4: return {periph_base + 0x0000'0800, rcc::gate::tim4, 30, 4, false, false, false};
        case TimerId::Tim6: return {periph_base + 0x0000'1000, rcc::gate::tim6, 54, 0, false, false, false};
        case TimerId::Tim7: return {periph_base + 0x0000'1400, rcc::gate::tim7, 55, 0, false, false, false};
        case TimerId::Tim8: return {periph_base + 0x0001'3400, rcc::gate::tim8, 44, 4, false, true, false};
        case TimerId::Tim15: return {periph_base + 0x0001'4000, rcc::gate::tim15, 24, 2, false, true, false};
        case TimerId::Tim16: return {periph_base + 0x0001'4400, rcc::gate::tim16, 25, 1, false, true, false};
        case TimerId::Tim17: return {periph_base + 0x0001'4800, rcc::gate::tim17, 26, 1, false, true, false};
    }
    return {};
}

template <TimerId Id, TimerConfig Config>
using Timer = stm32::Timer<timer_hw(Id), Config>;

/// A free-running 1 MHz clock on TIM2 (the G431's only 32-bit timer) with one deadline;
/// satisfies usb_pd::DeadlineTimer (stm32/timer.hpp).
template <TimerId Id, std::uint32_t KernelHz, unsigned Ch = 1>
using MicrosecondTimer = stm32::MicrosecondTimer<timer_hw(Id), KernelHz, Ch>;

}  // namespace rtos::port::stm32g4
