// SPDX-License-Identifier: MIT
//
// ARM Cortex-M33 (ARMv8-M Mainline, FPv5-SP) implementation of rtos::CpuPort.
//
// Design decisions
// ----------------
// * Kernel lock = BASEPRI, never PRIMASK. Interrupts with a priority value below
//   `kernel_basepri` (i.e. more urgent) are never masked by the kernel and have
//   zero added latency. They may use only lock-free services (Kernel::notify,
//   BlockPool) - those never call lock().
// * PendSV (lowest priority) performs every context switch; SysTick runs at the
//   same lowest priority, so a tick never preempts a half-done switch.
// * Threads run on PSP with PSPLIM = bottom of their stack: hardware stack
//   overflow detection (UsageFault STKOF) at no run-time cost.
// * FPU: lazy stacking (ASPEN|LSPEN). s16-s31 are saved only for threads that have
//   actually used the FPU (EXC_RETURN.FType == 0).
// * Security state agnostic: the EXC_RETURN value for new threads is derived at
//   run time from the start-up SVC, so the same binary works in Secure or
//   Non-secure state (TrustZone enabled or not).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/kernel/port.hpp"
#include "rtos/port/cm33/registers.hpp"

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

inline constexpr std::uint32_t kernel_basepri = (RTOS_CM33_KERNEL_PRIORITY << (8 - RTOS_CM33_PRIO_BITS)) & 0xFFu;
/// Least urgent priority, expressed in the implemented (upper) priority bits.
inline constexpr std::uint32_t lowest_priority = (0xFFu << (8 - RTOS_CM33_PRIO_BITS)) & 0xFFu;

struct Cm33Port {
    using LockState = std::uint32_t;

    /// Software frame (r4-r11, EXC_RETURN) 36 B + extended HW frame 104 B
    /// + s16-s31 64 B = 204 B worst case per switch, plus margin for the entry call.
    static constexpr std::size_t min_stack_bytes = 256;

    static LockState lock() noexcept {
        std::uint32_t previous;
        // BASEPRI_MAX only ever raises the mask -> nesting is free.
        __asm volatile(
            "mrs %0, basepri      \n"
            "msr basepri_max, %1  \n"
            "isb                  \n"
            : "=&r"(previous)
            : "r"(kernel_basepri)
            : "memory");
        return previous;
    }

    static void unlock(LockState previous) noexcept {
        __asm volatile(
            "msr basepri, %0 \n"
            "isb             \n" ::"r"(previous)
            : "memory");
    }

    static void request_context_switch() noexcept {
        scb::ICSR::write(scb::icsr_pendsvset);
        // Make the pend visible before returning, so that an unlock() right after
        // takes the exception immediately.
        __asm volatile("dsb \n isb" ::: "memory");
    }

    static bool in_isr() noexcept {
        std::uint32_t ipsr;
        __asm volatile("mrs %0, ipsr" : "=r"(ipsr));
        return ipsr != 0;
    }

    /// Priority of the active exception (IPSR) compared with the kernel mask.
    /// Exception numbers: 1..3 Reset/NMI/HardFault (fixed negative priority),
    /// 4..15 system handlers (SHPR1..3 at 0xE000ED18 + n - 4), >= 16 IRQ n - 16.
    static bool may_lock() noexcept {
        std::uint32_t ipsr;
        __asm volatile("mrs %0, ipsr" : "=r"(ipsr));
        const std::uint32_t exception = ipsr & 0x1FFu;
        if (exception == 0) {
            return true;  // thread mode
        }
        if (exception < 4) {
            return false;
        }
        const std::uint32_t priority =
            exception < 16 ? *reinterpret_cast<volatile std::uint8_t*>(0xE000'ED18 + exception - 4)
                           : nvic::ipr(exception - 16);
        return priority >= kernel_basepri;
    }

    static void* init_stack(std::span<std::byte> stack, PortEntry entry, void* a0, void* a1) noexcept;
    static void start_first_thread(void* sp, std::uintptr_t limit) noexcept;

    static void idle_wait() noexcept { __asm volatile("dsb \n wfi" ::: "memory"); }

    static void park() noexcept {
        for (;;) {
            __asm volatile("wfi");
        }
    }

    /// Enable fault handlers, configure exception priorities, FPU lazy stacking and
    /// the SysTick timer.
    /// Call once from main() before Kernel::start().
    static void configure(std::uint32_t core_clock_hz, std::uint32_t tick_hz) noexcept;
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
