// SPDX-License-Identifier: MIT
//
// The CPU port interface. The kernel never touches a register directly; everything
// architecture-specific goes through a type satisfying `CpuPort`. All members are
// static, so the abstraction is resolved at compile time (zero cost, no vtable,
// no `this`).
//
// Implementations:
//   * rtos::port::cm33::Cm33Port  (port/cortex_m33)  - real hardware / emulator
//   * MockPort                     (tests/)            - host unit tests
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos {

/// Initial function executed on a fresh thread stack, receiving two pointer arguments.
using PortEntry = void (*)(void*, void*);

template <class P>
concept CpuPort = requires(typename P::LockState state, std::span<std::byte> stack, PortEntry entry,
                           void* arg, void* sp, std::uintptr_t limit) {
    /// Opaque value returned by lock() and handed back to unlock() (e.g. old BASEPRI).
    typename P::LockState;

    /// Smallest stack accepted by init_stack (worst-case context frame + margin).
    { P::min_stack_bytes } -> std::convertible_to<std::size_t>;

    /// Enter the kernel critical section. Nestable. Must mask every interrupt that is
    /// allowed to call locking kernel APIs (on Cortex-M: BASEPRI, never PRIMASK).
    { P::lock() } noexcept -> std::same_as<typename P::LockState>;
    { P::unlock(state) } noexcept -> std::same_as<void>;

    /// Request a deferred context switch (Cortex-M: pend PendSV). Must be callable
    /// from any context, including interrupts above the kernel mask.
    { P::request_context_switch() } noexcept -> std::same_as<void>;

    /// True when executing in interrupt context.
    { P::in_isr() } noexcept -> std::same_as<bool>;

    /// True if the current context may take the kernel lock: thread mode, or an
    /// interrupt whose priority is not above the kernel mask ("kernel-aware ISR").
    /// False in zero-latency interrupts (and NMI/HardFault): only lock-free kernel
    /// services are allowed there.
    { P::may_lock() } noexcept -> std::same_as<bool>;

    /// Build the initial context on `stack` so that the first switch to it calls
    /// `entry(arg, arg2)`. Returns the initial saved stack pointer.
    { P::init_stack(stack, entry, arg, arg) } noexcept -> std::same_as<void*>;

    /// Switch from the boot context to the first thread. Never returns on real hardware.
    { P::start_first_thread(sp, limit) } noexcept -> std::same_as<void>;

    /// Low-power wait used by the idle thread (WFI).
    { P::idle_wait() } noexcept -> std::same_as<void>;

    /// Called by a terminated thread after it has been removed from scheduling; on
    /// hardware the pending switch makes this unreachable.
    { P::park() } noexcept -> std::same_as<void>;
};

/// Optional: a free-running cycle counter. With it the kernel accumulates each thread's CPU
/// cycles (ThreadControlBlock::run_cycles), e.g. for a task monitor.
template <class P>
concept CycleCountingPort = requires {
    { P::cycle_count() } noexcept -> std::same_as<std::uint32_t>;
};

}  // namespace rtos
