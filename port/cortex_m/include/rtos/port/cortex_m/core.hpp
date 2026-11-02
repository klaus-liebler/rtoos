// SPDX-License-Identifier: MIT
//
// The part of rtos::CpuPort that is the same on ARMv7-M (Cortex-M4) and ARMv8-M Mainline
// (Cortex-M33): the kernel lock on BASEPRI, PendSV pending, exception state queries, SysTick,
// the initial thread frame. The ports (cm4, cm33) derive from CoreServices and add what differs:
// the context switch and the start of the first thread (ARMv8-M also loads PSPLIM).
//
// Design decisions (both ports)
// -----------------------------
// * Kernel lock = BASEPRI, never PRIMASK. Interrupts with a priority value below
//   `kernel_basepri` (i.e. more urgent) are never masked by the kernel and have
//   zero added latency. They may use only lock-free services (Kernel::notify,
//   BlockPool) - those never call lock().
// * PendSV (lowest priority) performs every context switch; SysTick runs at the
//   same lowest priority, so a tick never preempts a half-done switch.
// * FPU: lazy stacking (ASPEN|LSPEN). s16-s31 are saved only for threads that have
//   actually used the FPU (EXC_RETURN.FType == 0).
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/kernel/port.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"

extern "C" {
/// Trap for a thread that returns through LR (never happens: the kernel trampoline
/// terminates threads). Lands here only on a corrupted stack. Defined by the port.
[[noreturn]] void rtos_port_thread_return_trap() noexcept;
}

namespace rtos::port::cortex_m {

/// `PrioBits`: implemented NVIC priority bits; `KernelPriority`: the most urgent priority the
/// kernel lock masks (ISRs with priority 0 .. KernelPriority - 1 are never masked).
template <unsigned PrioBits, unsigned KernelPriority>
struct CoreServices {
    static_assert(PrioBits >= 2 && PrioBits <= 8, "the NVIC implements 2..8 priority bits");
    static_assert(KernelPriority >= 1 && KernelPriority < (1u << PrioBits),
                  "kernel priority must leave room for zero-latency interrupts");

    using LockState = std::uint32_t;

    static constexpr std::uint32_t kernel_basepri = (KernelPriority << (8 - PrioBits)) & 0xFFu;
    /// Least urgent priority, expressed in the implemented (upper) priority bits.
    static constexpr std::uint32_t lowest_priority = (0xFFu << (8 - PrioBits)) & 0xFFu;

    /// NVIC/SHPR value of priority `level` (0 = most urgent .. 2^PrioBits - 1).
    [[nodiscard]] static constexpr std::uint8_t priority(unsigned level) noexcept {
        return static_cast<std::uint8_t>(level << (8 - PrioBits));
    }

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

    /// DWT_CYCCNT: per-thread CPU cycles (Kernel run_cycles) once CycleCounter::enable() ran.
    static std::uint32_t cycle_count() noexcept { return CycleCounter::now(); }

    /// Reset the whole chip (AIRCR.SYSRESETREQ; ARMv7-M and ARMv8-M Architecture Reference Manuals).
    [[noreturn]] static void system_reset() noexcept {
        __asm volatile("dsb" ::: "memory");
        scb::AIRCR::write(scb::aircr_vectkey | (scb::AIRCR::read() & scb::aircr_prigroup_mask) | scb::aircr_sysresetreq);
        __asm volatile("dsb" ::: "memory");
        for (;;) {
        }
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

    static void idle_wait() noexcept { __asm volatile("dsb \n wfi" ::: "memory"); }

    static void park() noexcept {
        for (;;) {
            __asm volatile("wfi");
        }
    }

    /// Initial context of a thread (the port's switch code unstacks it):
    ///
    ///   saved SP -> r4 r5 r6 r7 r8 r9 r10 r11 EXC_RETURN   (software frame, 9 words)
    ///               r0 r1 r2 r3 r12 lr pc xPSR              (hardware frame)
    ///
    /// EXC_RETURN = 0 is the sentinel of a thread that has never run: the switch code replaces
    /// it by the template the start-up SVC captured (Thread mode, PSP, no FP frame).
    static void* init_stack(std::span<std::byte> stack, PortEntry entry, void* a0, void* a1) noexcept {
        // AAPCS: 8-byte aligned stack at function entry.
        auto top = (reinterpret_cast<std::uintptr_t>(stack.data()) + stack.size()) & ~std::uintptr_t{7};
        auto* sp = reinterpret_cast<std::uint32_t*>(top);

        // Hardware exception frame, unstacked by the exception return.
        *--sp = 0x0100'0000u;  // xPSR: Thumb bit
        *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(entry)) & ~1u;  // PC
        *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&rtos_port_thread_return_trap));  // LR
        *--sp = 0;  // r12
        *--sp = 0;  // r3
        *--sp = 0;  // r2
        *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(a1));  // r1
        *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(a0));  // r0

        // Software frame: EXC_RETURN sentinel (highest address), then r11..r4.
        *--sp = 0;
        for (int r = 0; r < 8; ++r) {
            *--sp = 0;
        }
        return sp;
    }

    /// Enable the fault handlers, set the exception priorities (PendSV and SysTick lowest,
    /// SVC most urgent: only used at start), FPU lazy stacking, SysTick at `tick_hz` from the
    /// core clock. Call once from main() before Kernel::start().
    static void configure(std::uint32_t core_clock_hz, std::uint32_t tick_hz) noexcept {
        scb::SHCSR::set(scb::shcsr_memfaultena | scb::shcsr_busfaultena | scb::shcsr_usgfaultena);
        scb::SHPR3::write((scb::SHPR3::read() & 0x0000'FFFFu) | (lowest_priority << 16) | (lowest_priority << 24));
        scb::SHPR2::write(scb::SHPR2::read() & 0x00FF'FFFFu);
#if defined(__ARM_FP)
        fpu::FPCCR::set(fpu::fpccr_aspen | fpu::fpccr_lspen);
#endif
        systick::CTRL::write(0);
        systick::LOAD::write(core_clock_hz / tick_hz - 1u);
        systick::VAL::write(0);
        systick::CTRL::write(systick::ctrl_clksource_cpu | systick::ctrl_tickint | systick::ctrl_enable);
    }
};

}  // namespace rtos::port::cortex_m
