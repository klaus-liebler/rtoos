// SPDX-License-Identifier: MIT
//
// Counting semaphore (binary: max = 1).
//
//   take(timeout)     thread       blocks while the count is 0
//   try_take()        thread       never blocks
//   give()            thread       hands the unit to the first waiter, else count++
//   give_from_isr()   kernel-aware ISR
//
// Waiters are served FIFO. A give() to a waiting thread hands the unit over
// directly (the count stays 0), so a woken thread can never lose it to a third
// thread that runs first. All operations O(1).
#pragma once

#include <contracts>
#include <cstdint>
#include <optional>

#include "rtos/config.hpp"
#include "rtos/kernel/thread.hpp"

namespace rtos {

template <class K>
class Semaphore {
public:
    Semaphore(K& kernel, std::uint32_t initial, std::uint32_t max) noexcept
        : kernel_(kernel), count_(initial), max_(max) {
        contract_assert(max >= 1);
        contract_assert(initial <= max);
    }

    Semaphore(const Semaphore&) = delete;
    Semaphore& operator=(const Semaphore&) = delete;

    /// Phase 1: true = taken, false = not available (only with no_wait),
    /// nullopt = blocked; then call K::wait_end() (Ok = taken).
    std::optional<bool> take_begin(Tick timeout) noexcept {
        contract_assert(!K::in_isr());
        typename K::Lock lock;
        if (count_ > 0) {
            --count_;
            return true;
        }
        if (timeout == no_wait) {
            return false;
        }
        kernel_.block_current_locked(waiters_, timeout);
        return std::nullopt;
    }

    /// Take one unit, waiting up to `timeout` ticks. Returns false on timeout.
    bool take(Tick timeout = wait_forever) noexcept {
        if (auto taken = take_begin(timeout)) {
            return *taken;
        }
        return kernel_.wait_end().result == WaitResult::Ok;
    }

    bool try_take() noexcept { return take(no_wait); }

    /// Release one unit. Returns false if the count is already at its maximum.
    bool give() noexcept {
        contract_assert(!K::in_isr());
        return give_locked();
    }

    /// give() for kernel-aware interrupts.
    bool give_from_isr() noexcept {
        contract_assert(K::in_isr());
        contract_assert(K::may_lock());
        return give_locked();
    }

    [[nodiscard]] std::uint32_t count() const noexcept {
        typename K::Lock lock;
        return count_;
    }

private:
    bool give_locked() noexcept {
        typename K::Lock lock;
        if (kernel_.wake_one_locked(waiters_) != nullptr) {
            return true;  // handed over directly
        }
        if (count_ == max_) {
            return false;
        }
        ++count_;
        return true;
    }

    K& kernel_;
    typename K::WaitQueue waiters_{};
    std::uint32_t count_;
    std::uint32_t max_;
};

}  // namespace rtos
