// SPDX-License-Identifier: MIT
//
// Building blocks for blocking kernel objects.
//
// WaitQueue<Tcb>
//   FIFO of threads blocked on one object (ThreadX semantics: woken in arrival
//   order, O(1) block and O(1) wake). Only the kernel touches the list, always under
//   the kernel lock. `has_waiters()` is lock-free and may be read from any context,
//   e.g. by a zero-latency interrupt that must decide whether to defer a wake-up.
//
// Deferred
//   One unit of deferred kernel work, posted lock-free from any context (including
//   interrupts above the kernel mask) with Kernel::defer_from_isr() and executed
//   under the kernel lock at the next scheduling point (PendSV / tick). Posting an
//   already queued item is a no-op, so an item never needs storage per request.
#pragma once

#include <atomic>
#include <cstdint>

#include "rtos/kernel/intrusive_list.hpp"
#include "rtos/kernel/kernel_fwd.hpp"

namespace rtos {

template <class Tcb>
class WaitQueue {
public:
    constexpr WaitQueue() noexcept = default;
    WaitQueue(const WaitQueue&) = delete;
    WaitQueue& operator=(const WaitQueue&) = delete;

    /// True if at least one thread is blocked here. Lock-free, any context.
    [[nodiscard]] bool has_waiters() const noexcept { return count_.load(std::memory_order_seq_cst) != 0; }

private:
    template <CpuPort P, KernelConfig C>
    friend class Kernel;

    IntrusiveList<Tcb> waiters_{};
    std::atomic<std::uint16_t> count_{0};
};

class Deferred {
public:
    using Function = void (*)(void* context) noexcept;

    constexpr Deferred(Function function, void* context) noexcept : function_(function), context_(context) {}
    Deferred(const Deferred&) = delete;
    Deferred& operator=(const Deferred&) = delete;

private:
    template <CpuPort P, KernelConfig C>
    friend class Kernel;

    Function function_;
    void* context_;
    Deferred* next_ = nullptr;  // published by the CAS on the kernel's list head
    std::atomic<bool> queued_{false};
};

}  // namespace rtos
