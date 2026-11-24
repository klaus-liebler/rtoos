// SPDX-License-Identifier: MIT
//
// ARM Cortex-M3 (ARMv7-M without FP extension) implementation of rtos::CpuPort: the Cortex-M4
// port (cm4/port.hpp) compiled without an FPU. Everything it does is ARMv7-M - BASEPRI kernel
// lock, PendSV/SVC context switch, SysTick, the DWT cycle counter; its FP frame handling is
// under __ARM_FP and drops out, so a thread context is the 9-word software frame plus the
// 8-word hardware frame (68 bytes).
//
// Differences to the Cortex-M4 that matter here:
// * No FPU: CONTROL.FPCA does not exist (start_first_thread clears it: writing 0 to the
//   reserved bit is harmless), no CPACR to set (cortex_m startup skips it without __ARM_FP).
// * NVIC: the STM32F1 implements 4 priority bits like the G4 (RTOS_CM4_PRIO_BITS = 4).
//
//   using Port = rtos::port::cm3::Cm3Port;
//   RTOS_CM3_BIND_KERNEL(kernel)
#pragma once

#include "rtos/port/cm4/port.hpp"

#if defined(__ARM_FP)
#error "cm3/port.hpp is for a core without FPU (-mcpu=cortex-m3)"
#endif

namespace rtos::port::cm3 {

using namespace cortex_m;

using Cm3Port = cm4::Cm4Port;
inline constexpr std::uint32_t kernel_basepri = cm4::kernel_basepri;
inline constexpr std::uint32_t lowest_priority = cm4::lowest_priority;

static_assert(rtos::CpuPort<Cm3Port>);

}  // namespace rtos::port::cm3

/// Connects one kernel instance to the port's exception handlers (see RTOS_CM4_BIND_KERNEL).
#define RTOS_CM3_BIND_KERNEL(kernel_object) RTOS_CM4_BIND_KERNEL(kernel_object)
