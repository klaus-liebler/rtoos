// SPDX-License-Identifier: MIT
//
// ARM Cortex-M33 (ARMv8-M Mainline, FPv5-SP) implementation of rtos::CpuPort.
//
// Everything but the context switch is the Cortex-M core both ports share (cortex_m/core.hpp:
// BASEPRI kernel lock, PendSV, SysTick, lazy FPU stacking, the initial thread frame).
// What ARMv8-M adds:
// * Threads run on PSP with PSPLIM = bottom of their stack: hardware stack
//   overflow detection (UsageFault STKOF) at no run-time cost.
// * Security state agnostic: the EXC_RETURN value for new threads is derived at
//   run time from the start-up SVC, so the same binary works in Secure or
//   Non-secure state (TrustZone enabled or not).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/kernel/port.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/registers.hpp"
#include "rtos/port/cortex_m/core.hpp"

#ifndef RTOS_CM33_PRIO_BITS
#define RTOS_CM33_PRIO_BITS 4  // STM32H5: 16 NVIC priority levels
#endif
#ifndef RTOS_CM33_KERNEL_PRIORITY
#define RTOS_CM33_KERNEL_PRIORITY 5  // ISRs with priority 0..4 are never masked
#endif

extern "C" {
/// {initial PSP, PSPLIM} of the first thread, consumed by SVC_Handler.
extern std::uint32_t rtos_port_start_ctx[2];
}

namespace rtos::port::cm33 {

using Core = cortex_m::CoreServices<RTOS_CM33_PRIO_BITS, RTOS_CM33_KERNEL_PRIORITY>;
inline constexpr std::uint32_t kernel_basepri = Core::kernel_basepri;
inline constexpr std::uint32_t lowest_priority = Core::lowest_priority;

struct Cm33Port : Core {
    /// Software frame (r4-r11, EXC_RETURN) 36 B + extended HW frame 104 B
    /// + s16-s31 64 B = 204 B worst case per switch, plus margin for the entry call.
    static constexpr std::size_t min_stack_bytes = 256;

    static void start_first_thread(void* sp, std::uintptr_t limit) noexcept;
};

static_assert(rtos::CpuPort<Cm33Port>);

}  // namespace rtos::port::cm33

/// Connects one kernel instance to the port's exception handlers. Use exactly once
/// in the application, at namespace scope:  RTOS_CM33_BIND_KERNEL(my_kernel)
///
/// rtos_port_switch is called from the PendSV assembly; it returns the next thread's
/// PSP in r0 and its PSPLIM in r1 (AAPCS: 64-bit result in r0:r1).
#define RTOS_CM33_BIND_KERNEL(kernel_object)                                                      \
    extern "C" std::uint64_t rtos_port_switch(void* sp) noexcept {                                \
        auto& next = (kernel_object).switch_context(sp);                                          \
        return (static_cast<std::uint64_t>(next.stack_limit) << 32) |                             \
               static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(next.sp));             \
    }                                                                                             \
    extern "C" void SysTick_Handler() noexcept { (kernel_object).tick(); }
