// SPDX-License-Identifier: MIT
//
// ARM Cortex-M4 (ARMv7E-M, FPv4-SP) implementation of rtos::CpuPort. The Cortex-M7 runs it
// unchanged (cm7/port.hpp: same ARMv7-M exception model and FP frame).
//
// Everything but the context switch is the Cortex-M core both ports share (cortex_m/core.hpp:
// BASEPRI kernel lock, PendSV, SysTick, lazy FPU stacking, the initial thread frame).
// Differences to the Cortex-M33 port:
// * No stack limit register (PSPLIM is ARMv8-M): a thread stack overflow is not caught by the
//   hardware. The kernel checks the saved SP against the stack's lower end at every context
//   switch (contract), which catches overflows that are still there when the thread is switched
//   out - size the stacks from the deepest call path.
// * No security states: the EXC_RETURN of a fresh thread is the one the start-up SVC sees,
//   with PSP selected (0xFFFFFFFD).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/kernel/port.hpp"
#include "rtos/port/cortex_m/core.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"

#ifndef RTOS_CM4_PRIO_BITS
#define RTOS_CM4_PRIO_BITS 4  // STM32G4: 16 NVIC priority levels
#endif
#ifndef RTOS_CM4_KERNEL_PRIORITY
#define RTOS_CM4_KERNEL_PRIORITY 5  // ISRs with priority 0..4 are never masked
#endif

extern "C" {
/// Initial PSP of the first thread, consumed by SVC_Handler.
extern std::uint32_t rtos_port_start_sp;
}

namespace rtos::port::cm4 {

// The core registers and the cycle counter of the Cortex-M family (cm4::nvic, cm4::CycleCounter, ...).
using namespace cortex_m;

using Core = cortex_m::CoreServices<RTOS_CM4_PRIO_BITS, RTOS_CM4_KERNEL_PRIORITY>;
inline constexpr std::uint32_t kernel_basepri = Core::kernel_basepri;
inline constexpr std::uint32_t lowest_priority = Core::lowest_priority;

struct Cm4Port : Core {
    /// Software frame (r4-r11, EXC_RETURN) 36 B + extended HW frame 104 B
    /// + s16-s31 64 B = 204 B worst case per switch, plus margin for the entry call.
    static constexpr std::size_t min_stack_bytes = 256;

    /// `limit` is unused: ARMv7-M has no stack limit register.
    static void start_first_thread(void* sp, std::uintptr_t limit) noexcept;
};

static_assert(rtos::CpuPort<Cm4Port>);

}  // namespace rtos::port::cm4

/// Connects one kernel instance to the port's exception handlers. Use exactly once
/// in the application, at namespace scope:  RTOS_CM4_BIND_KERNEL(my_kernel)
///
/// rtos_port_switch is called from the PendSV assembly; it returns the next thread's PSP.
#define RTOS_CM4_BIND_KERNEL(kernel_object)                                                       \
    extern "C" void* rtos_port_switch(void* sp) noexcept { return (kernel_object).switch_context(sp).sp; } \
    extern "C" void SysTick_Handler() noexcept { (kernel_object).tick(); }
