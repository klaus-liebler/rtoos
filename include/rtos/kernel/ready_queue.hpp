// SPDX-License-Identifier: MIT
//
// Ready queue: one FIFO per priority level plus a bitmap of non-empty levels.
// insert / remove / highest / front / rotate are all O(1).
#pragma once

#include <array>
#include <contracts>
#include <cstddef>

#include "rtos/kernel/intrusive_list.hpp"
#include "rtos/kernel/priority_bitmap.hpp"

namespace rtos {

template <class Tcb, std::size_t Levels>
    requires(Levels <= PriorityBitmap::levels)
class ReadyQueue {
public:
    void insert(Tcb& t) noexcept {
        contract_assert(t.priority < Levels);
        lists_[t.priority].push_back(t.sched_link);
        map_.set(t.priority);
    }

    /// Re-insert a thread at the head of its level (it keeps its round-robin turn).
    void insert_front(Tcb& t) noexcept {
        contract_assert(t.priority < Levels);
        lists_[t.priority].push_front(t.sched_link);
        map_.set(t.priority);
    }

    void remove(Tcb& t) noexcept {
        contract_assert(t.sched_link.linked());
        t.sched_link.unlink();
        if (lists_[t.priority].empty()) {
            map_.clear(t.priority);
        }
    }

    [[nodiscard]] bool empty() const noexcept { return map_.empty(); }
    [[nodiscard]] unsigned highest() const noexcept { return map_.highest(); }

    [[nodiscard]] Tcb* front(unsigned prio) const noexcept {
        contract_assert(prio < Levels);
        return lists_[prio].front();
    }

    [[nodiscard]] bool is_front(const Tcb& t) const noexcept {
        return lists_[t.priority].is_front(t.sched_link);
    }

    [[nodiscard]] bool has_peers(const Tcb& t) const noexcept {
        return lists_[t.priority].has_multiple();
    }

    void rotate(unsigned prio) noexcept {
        contract_assert(prio < Levels);
        lists_[prio].rotate();
    }

private:
    std::array<IntrusiveList<Tcb>, Levels> lists_{};
    PriorityBitmap map_{};
};

}  // namespace rtos
