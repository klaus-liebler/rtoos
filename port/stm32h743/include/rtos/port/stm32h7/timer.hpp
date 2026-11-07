// SPDX-License-Identifier: MIT
//
// STM32H743 timers TIM1-8, TIM12-17: the shared STM32 driver (stm32/timer.hpp) with the H743's
// instances (RM0433 chapters TIM; addresses, clock gates: SVD v2.4; IRQ numbers: ST's
// stm32h743xx.h).
//
//   // PWM: 1 kHz, at least 1000 steps of duty resolution
//   using Pwm = Timer<TimerId::Tim4, {.kernel_hz = clocks.timer1_hz(), .update_hz = 1'000, .min_steps = 1000}>;
//   Pwm::init();  Pwm::configure_pwm<3>();  Pwm::start();  Pwm::set_duty_permille<3>(400);
//
// Kernel clock: ClockTree::timer1_hz() for TIM2-7 and TIM12-14 (APB1), timer2_hz() for TIM1/8
// and TIM15-17 (APB2). TIM2 and TIM5 count 32 bit, the others 16 bit. TIMx_DCR/DMAR sit at
// 0x048/0x04C (not 0x3DC/0x3E0 as on the H5), without DBSS. The irq is the update (or global)
// interrupt: TIM1_UP, TIM8_UP_TIM13 (shared with TIM13), TIM8_BRK_TIM12, TIM8_TRG_COM_TIM14.
#pragma once

#include <cstdint>

#include "rtos/port/stm32/timer.hpp"
#include "rtos/port/stm32h7/chip.hpp"
#include "rtos/port/stm32h7/rcc.hpp"

namespace rtos::port::stm32h7 {

using stm32::timer_registers, stm32::timer_update_mhz, stm32::timer_update_within_1_percent;
using stm32::TimerConfig, stm32::TimerRegisters;

enum class TimerId : std::uint8_t {
    Tim1, Tim2, Tim3, Tim4, Tim5, Tim6, Tim7, Tim8, Tim12, Tim13, Tim14, Tim15, Tim16, Tim17
};

inline constexpr std::uint16_t timer_dcr_offset = 0x048;

[[nodiscard]] consteval stm32::TimerHw timer_hw(TimerId id) {
    // base, gate, update IRQ, channels, 32 bit, advanced (BDTR), DBSS, DCR
    switch (id) {
        case TimerId::Tim1: return {0x4001'0000, rcc::gate::tim1, 25, 4, false, true, false, timer_dcr_offset};
        case TimerId::Tim2: return {0x4000'0000, rcc::gate::tim2, 28, 4, true, false, false, timer_dcr_offset};
        case TimerId::Tim3: return {0x4000'0400, rcc::gate::tim3, 29, 4, false, false, false, timer_dcr_offset};
        case TimerId::Tim4: return {0x4000'0800, rcc::gate::tim4, 30, 4, false, false, false, timer_dcr_offset};
        case TimerId::Tim5: return {0x4000'0C00, rcc::gate::tim5, 50, 4, true, false, false, timer_dcr_offset};
        case TimerId::Tim6: return {0x4000'1000, rcc::gate::tim6, 54, 0, false, false, false, timer_dcr_offset};
        case TimerId::Tim7: return {0x4000'1400, rcc::gate::tim7, 55, 0, false, false, false, timer_dcr_offset};
        case TimerId::Tim8: return {0x4001'0400, rcc::gate::tim8, 44, 4, false, true, false, timer_dcr_offset};
        case TimerId::Tim12: return {0x4000'1800, rcc::gate::tim12, 43, 2, false, false, false, timer_dcr_offset};
        case TimerId::Tim13: return {0x4000'1C00, rcc::gate::tim13, 44, 1, false, false, false, timer_dcr_offset};
        case TimerId::Tim14: return {0x4000'2000, rcc::gate::tim14, 45, 1, false, false, false, timer_dcr_offset};
        case TimerId::Tim15: return {0x4001'4000, rcc::gate::tim15, 116, 2, false, true, false, timer_dcr_offset};
        case TimerId::Tim16: return {0x4001'4400, rcc::gate::tim16, 117, 1, false, true, false, timer_dcr_offset};
        case TimerId::Tim17: return {0x4001'4800, rcc::gate::tim17, 118, 1, false, true, false, timer_dcr_offset};
    }
    return {};
}

template <TimerId Id, TimerConfig Config>
using Timer = stm32::Timer<timer_hw(Id), Config>;

/// A free-running 1 MHz clock on a 32-bit timer (TIM2, TIM5) with one deadline (stm32/timer.hpp).
template <TimerId Id, std::uint32_t KernelHz, unsigned Ch = 1>
using MicrosecondTimer = stm32::MicrosecondTimer<timer_hw(Id), KernelHz, Ch>;

}  // namespace rtos::port::stm32h7
