// SPDX-License-Identifier: MIT
//
// Hashed timing wheel for thread timeouts (same structure as ThreadX' timer list).
//
//   insert : O(1)  - bucket = wake_tick mod Slots
//   cancel : O(1)  - unlink
//   expire : O(k)  - k = threads hashed into the current bucket, bounded by max_threads
//
// Entries more than one revolution in the future simply stay in their bucket until
// `wake_tick == now`; the exact-equality test is wrap-around safe for any timeout
// below 2^32 ticks.
#pragma once

#include <array>
#include <bit>
#include <contracts>
#include <cstddef>

#include "rtos/config.hpp"
#include "rtos/kernel/intrusive_list.hpp"

namespace rtos {

template <class Tcb, std::size_t Slots>
    requires(std::has_single_bit(Slots))
class TimerWheel {
public:
    void insert(Tcb& t) noexcept {
        contract_assert(!t.timer_link.linked());
        buckets_[t.wake_tick & mask].push_back(t.timer_link);
    }

    static void cancel(Tcb& t) noexcept { t.timer_link.unlink(); }

    /// Remove every entry whose deadline is `now` and hand it to `on_expired`.
    template <class Fn>
    void expire(Tick now, Fn&& on_expired) noexcept {
        buckets_[now & mask].for_each_safe([&](Tcb& t) noexcept {
            if (t.wake_tick == now) {
                t.timer_link.unlink();
                on_expired(t);
            }
        });
    }

private:
    static constexpr Tick mask = static_cast<Tick>(Slots - 1);
    std::array<IntrusiveList<Tcb>, Slots> buckets_{};
};

}  // namespace rtos
