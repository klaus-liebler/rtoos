// SPDX-License-Identifier: MIT
//
// Host mock of the CPU port. Records instead of acting:
//   * lock()/unlock()          -> nesting depth (tests assert it is balanced)
//   * request_context_switch() -> counter ("PendSV pending"); the test plays the role
//                                 of the hardware and calls Kernel::switch_context().
//   * init_stack()             -> stores {entry, a0, a1} at the stack top so a test
//                                 can execute a thread body synchronously via run().
#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <new>
#include <span>

#include "rtos/kernel/port.hpp"

struct MockPort {
    using LockState = std::uint32_t;
    static constexpr std::size_t min_stack_bytes = 64;

    struct Frame {
        rtos::PortEntry entry;
        void* a0;
        void* a1;
    };

    static inline std::uint32_t lock_depth = 0;
    static inline std::atomic<std::uint32_t> switch_requests{0};  // atomic: stress tests
    static inline bool isr = false;               ///< executing in an interrupt
    static inline bool zero_latency_isr = false;  ///< ...whose priority is above the kernel mask
    static inline void* started_sp = nullptr;

    static void reset() noexcept {
        lock_depth = 0;
        switch_requests = 0;
        isr = false;
        zero_latency_isr = false;
        started_sp = nullptr;
    }

    static LockState lock() noexcept { return lock_depth++; }
    static void unlock(LockState previous) noexcept { lock_depth = previous; }
    static void request_context_switch() noexcept { switch_requests.fetch_add(1); }
    static bool in_isr() noexcept { return isr; }
    static bool may_lock() noexcept { return !(isr && zero_latency_isr); }

    static void* init_stack(std::span<std::byte> stack, rtos::PortEntry entry, void* a0, void* a1) noexcept {
        auto top = reinterpret_cast<std::uintptr_t>(stack.data()) + stack.size() - sizeof(Frame);
        top &= ~(std::uintptr_t{alignof(Frame)} - 1);
        return ::new (reinterpret_cast<void*>(top)) Frame{entry, a0, a1};
    }

    static void start_first_thread(void* sp, std::uintptr_t) noexcept { started_sp = sp; }
    static void idle_wait() noexcept {}
    static void park() noexcept {}

    /// Execute the thread whose initial context is at `sp` until its entry returns.
    static void run(void* sp) {
        auto* f = static_cast<Frame*>(sp);
        f->entry(f->a0, f->a1);
    }
};

static_assert(rtos::CpuPort<MockPort>);
