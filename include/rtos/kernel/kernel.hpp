// SPDX-License-Identifier: MIT
//
// rtos::Kernel - the core scheduler.
//
// Scheduling model
// ----------------
// * Fixed-priority preemptive, 0 = most urgent, up to 32 levels, FIFO within a level.
// * Preemption threshold (ThreadX): a running thread T can only be preempted by a
//   thread whose priority is *more urgent than T.threshold*. Threads sharing a
//   resource get the same threshold ("ceiling") and therefore never preempt each
//   other -> no mutex, no priority inversion, fewer context switches.
//   A thread with threshold that *was* preempted is remembered in `preempted_` and
//   resumed in preference to ready threads that may not preempt it.
// * Optional round-robin time slicing per thread (disabled while a threshold is
//   active, like ThreadX).
// * The running thread stays in the ready queue (at the front of its level).
//
// Determinism
// -----------
// Every scheduling decision is O(1): bitmap + CTZ for the highest level, intrusive
// lists for queue manipulation. tick() is O(k) with k = threads in one wheel bucket.
// Waking threads from lock-free notifications is O(w) with w = wake requests.
//
// Execution contexts
// ------------------
//   thread              every service without the _from_isr suffix
//   kernel-aware ISR    every *_from_isr service; priority value >= the kernel mask
//                       (Cortex-M: RTOS_CM33_KERNEL_PRIORITY). The kernel lock
//                       masks these interrupts.
//   zero-latency ISR    priority above the kernel mask: never masked by the kernel,
//                       so only lock-free services: notify_from_isr(),
//                       defer_from_isr(), ObjectPool::try_allocate()/
//                       deallocate_from_isr(), BlockPool.
// The contracts check the context: Port::in_isr() and Port::may_lock().
// An ISR never needs a "higher priority task woken" flag: every service pends
// PendSV itself when the selection may have changed.
//
// Concurrency model
// -----------------
// * Kernel data is protected by `Port::lock()` (BASEPRI on Cortex-M).
// * Task notifications use only atomics; a notifier posts a wake request into a
//   lock-free bitmap and pends a context switch; the bitmap is drained under the
//   lock in switch_context()/tick().
// * Deferred work items (defer_from_isr) are pushed onto a lock-free list and run
//   under the lock at the same points. This is how zero-latency interrupts wake
//   threads blocked on objects (e.g. a pool block freed by a DMA-complete IRQ).
// * Blocking calls are split into a *begin* phase (state transition + switch
//   request) and an *end* phase (collect the result after resumption). On hardware
//   the switch happens between the two at `unlock()`; host tests drive both phases
//   explicitly.
//
// No heap: all TCBs and stacks are supplied by the application, the idle thread's
// are embedded in the kernel object.
#pragma once

#include <array>
#include <atomic>
#include <bit>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <inplace_vector>
#include <optional>
#include <span>

#include "rtos/config.hpp"
#include "rtos/kernel/kernel_fwd.hpp"
#include "rtos/kernel/port.hpp"
#include "rtos/kernel/priority_bitmap.hpp"
#include "rtos/kernel/ready_queue.hpp"
#include "rtos/kernel/thread.hpp"
#include "rtos/kernel/timer_wheel.hpp"
#include "rtos/kernel/wait_queue.hpp"

namespace rtos {

template <CpuPort Port, KernelConfig Cfg>
class Kernel {
public:
    using PortType = Port;
    using Thread = ThreadControlBlock<Cfg::notify_slots>;
    using WaitQueue = rtos::WaitQueue<Thread>;
    using Entry = void (*)(void*);

    static constexpr std::size_t priority_levels = Cfg::priority_levels;
    static constexpr std::size_t notify_slots = Cfg::notify_slots;
    static constexpr std::uint8_t idle_priority = static_cast<std::uint8_t>(priority_levels - 1);
    /// `preemption_threshold` value meaning "equal to priority" (threshold disabled).
    static constexpr std::uint8_t no_threshold = 0xFF;

    struct ThreadParams {
        const char* name = "";
        Entry entry = nullptr;
        void* arg = nullptr;
        std::span<std::byte> stack{};
        std::uint8_t priority = 0;
        std::uint8_t preemption_threshold = no_threshold;
        std::uint32_t time_slice = 0;
    };

    /// Result of an object wait, read by the woken thread with wait_end().
    struct WaitOutcome {
        WaitResult result;
        std::uintptr_t data;
    };

    /// RAII kernel lock (BASEPRI on Cortex-M). Nestable. Public for kernel objects
    /// (semaphores, queues, pools) built on the *_locked primitives below.
    class Lock {
    public:
        Lock() noexcept : state_(checked_lock()) {}
        ~Lock() { Port::unlock(state_); }
        Lock(const Lock&) = delete;
        Lock& operator=(const Lock&) = delete;

    private:
        static typename Port::LockState checked_lock() noexcept {
            contract_assert(Port::may_lock());  // not from a zero-latency ISR
            return Port::lock();
        }

        typename Port::LockState state_;
    };

    // The ISR paths rely on these being genuinely lock-free (LDREX/STREX on ARMv8-M).
    static_assert(std::atomic<std::uint32_t>::is_always_lock_free);
    static_assert(std::atomic<std::uint16_t>::is_always_lock_free);
    static_assert(std::atomic<std::uint8_t>::is_always_lock_free);
    static_assert(std::atomic<Deferred*>::is_always_lock_free);

    Kernel() noexcept {
        register_thread(idle_, ThreadParams{.name = "idle",
                                            .entry = &idle_entry,
                                            .arg = nullptr,
                                            .stack = idle_stack_,
                                            .priority = idle_priority});
    }

    Kernel(const Kernel&) = delete;
    Kernel& operator=(const Kernel&) = delete;

    [[nodiscard]] static bool in_isr() noexcept { return Port::in_isr(); }
    [[nodiscard]] static bool may_lock() noexcept { return Port::may_lock(); }

    // =====================================================================
    // Thread management
    // =====================================================================

    /// Create a thread and make it ready. May be called before or after start().
    void create_thread(Thread& t, const ThreadParams& p) noexcept {
        contract_assert(p.priority < idle_priority);
        contract_assert(p.preemption_threshold == no_threshold || p.preemption_threshold <= p.priority);
        contract_assert(!Port::in_isr());
        register_thread(t, p);
    }

    /// Change the preemption threshold of `t` (ThreadX tx_thread_preemption_change).
    void set_preemption_threshold(Thread& t, std::uint8_t threshold) noexcept {
        contract_assert(threshold <= t.priority);
        contract_assert(!Port::in_isr());
        Lock lock;
        t.threshold = threshold;
        if (threshold == t.priority) {
            forget_preempted(t);
        }
        reschedule();
    }

    /// Remove a *ready* thread from scheduling. Returns false if `t` is blocked or
    /// already suspended (suspension of blocked threads is not supported).
    bool suspend(Thread& t) noexcept {
        contract_assert(&t != &idle_);
        contract_assert(!Port::in_isr());
        Lock lock;
        if (t.state != ThreadState::Ready) {
            return false;
        }
        ready_.remove(t);
        forget_preempted(t);
        t.state = ThreadState::Suspended;
        reschedule();
        return true;
    }

    bool resume(Thread& t) noexcept {
        contract_assert(!Port::in_isr());
        return resume_locked(t);
    }

    /// resume() for kernel-aware interrupts (e.g. a button IRQ starting a worker).
    bool resume_from_isr(Thread& t) noexcept {
        contract_assert(Port::in_isr());
        contract_assert(Port::may_lock());
        return resume_locked(t);
    }

    // =====================================================================
    // Start-up and port hooks
    // =====================================================================

    /// Select the first thread to run. Separate from start() so host tests can drive it.
    Thread& prepare_start() noexcept {
        contract_assert(!started_);
        Lock lock;
        started_ = true;
        run_pending_work();
        current_ = &pick();
        current_->slice_left = current_->time_slice;
        ++current_->run_count;
        if constexpr (CycleCountingPort<Port>) {
            switched_in_cycles_ = Port::cycle_count();
        }
        return *current_;
    }

    /// Hand the CPU to the scheduler. Does not return on hardware.
    void start() noexcept {
        Thread& first = prepare_start();
        Port::start_first_thread(first.sp, first.stack_limit);
    }

    /// Context-switch hook, called by the port's PendSV handler with the outgoing
    /// thread's stack pointer. Returns the thread to resume.
    Thread& switch_context(void* sp) noexcept {
        contract_assert(started_);
        Lock lock;
        Thread* prev = current_;
        prev->sp = sp;
        // Software stack check complementing the hardware PSPLIM check (the register
        // spill done by the switch code itself is not covered by PSPLIM).
        contract_assert(reinterpret_cast<std::uintptr_t>(sp) >= prev->stack_limit);

        run_pending_work();
        Thread& next = pick();
        if (&next != prev) {
            // Running thread with an active threshold loses the CPU without yielding:
            // remember it so it is resumed before threads it would have blocked.
            if (prev->state == ThreadState::Ready && prev->threshold < prev->priority &&
                ready_.is_front(*prev)) {
                preempted_[prev->priority] = prev;
                preempted_map_.set(prev->priority);
            }
            forget_preempted(next);
            next.slice_left = next.time_slice;
            if constexpr (CycleCountingPort<Port>) {
                const std::uint32_t now = Port::cycle_count();
                prev->run_cycles += now - switched_in_cycles_;
                switched_in_cycles_ = now;
            }
            ++next.run_count;
            current_ = &next;
            ++context_switches_;
        }
        return next;
    }

    /// System tick, called from the tick interrupt (kernel priority).
    void tick() noexcept {
        Lock lock;
        const Tick now = now_.load(std::memory_order_relaxed) + 1;
        now_.store(now, std::memory_order_relaxed);
        if (now == 0) {
            epoch_.store(epoch_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        }

        run_pending_work();
        timers_.expire(now, [this](Thread& t) noexcept {
            // Sleeping: done. WaitingNotify: notify_wait_end() reports the timeout.
            // Waiting: leave the object's queue, wait_end() reports Timeout.
            if (t.state == ThreadState::Waiting) {
                detach_from_wait_queue(t);
                t.wait_result = WaitResult::Timeout;
            }
            make_ready(t);
        });

        Thread* cur = current_;
        if (cur != nullptr && cur->state == ThreadState::Ready && cur->time_slice != 0 &&
            cur->threshold == cur->priority && --cur->slice_left == 0) {
            cur->slice_left = cur->time_slice;
            if (ready_.is_front(*cur)) {
                ready_.rotate(cur->priority);
            }
        }
        reschedule();
    }

    // =====================================================================
    // Thread services (called by the current thread)
    // =====================================================================

    /// Give the CPU to the next ready thread of the same priority.
    void yield() noexcept {
        contract_assert(!Port::in_isr());
        Lock lock;
        if (ready_.is_front(*current_)) {
            ready_.rotate(current_->priority);
        }
        reschedule();
    }

    /// Block the current thread for `ticks` system ticks (0 = yield).
    void sleep(Tick ticks) noexcept {
        contract_assert(!Port::in_isr());
        contract_assert(ticks != wait_forever);
        contract_assert(current_ != &idle_);
        if (ticks == 0) {
            yield();
            return;
        }
        Lock lock;
        Thread& self = *current_;
        ready_.remove(self);
        self.state = ThreadState::Sleeping;
        self.wake_tick = now_.load(std::memory_order_relaxed) + ticks;
        timers_.insert(self);
        Port::request_context_switch();
    }  // <- on hardware PendSV runs here, when BASEPRI is restored

    // =====================================================================
    // Task notifications (lock-free)
    // =====================================================================

    /// Signal `t` on slot `index` from a thread.
    void notify(Thread& t, std::size_t index, std::uint32_t value,
                NotifyAction action = NotifyAction::SetBits) noexcept {
                    contract_assert(index < notify_slots);
                    contract_assert(!Port::in_isr());
        notify_lock_free(t, index, value, action);
    }

    /// Signal `t` on slot `index` from an interrupt of ANY priority, including
    /// zero-latency interrupts: only atomics, never the kernel lock.
    void notify_from_isr(Thread& t, std::size_t index, std::uint32_t value,
                         NotifyAction action = NotifyAction::SetBits) noexcept {
                             contract_assert(index < notify_slots);
                             contract_assert(Port::in_isr());
        notify_lock_free(t, index, value, action);
    }

    /// Phase 1 of a notification wait. Returns the value if a notification is already
    /// pending (consumes it). Otherwise blocks the current thread (unless `timeout`
    /// is `no_wait`) and returns nullopt; call notify_wait_end() after resumption.
    std::optional<std::uint32_t> notify_wait_begin(std::size_t index, Tick timeout) noexcept {
        contract_assert(index < notify_slots);
        contract_assert(!Port::in_isr());
        contract_assert(current_ != &idle_);
        Lock lock;
        Thread& self = *current_;
        auto& flags = self.notify_flags[index];

        // Announce the wait first, then look: a notifier racing with us either sees
        // `waiting` (and posts a wake request) or we see its `pending`.
        const std::uint8_t prev = flags.fetch_or(notify_flag::waiting, std::memory_order_acq_rel);
        if ((prev & notify_flag::pending) != 0) {
            flags.fetch_and(static_cast<std::uint8_t>(~notify_flag::waiting), std::memory_order_relaxed);
            return consume(self, index);
        }
        if (timeout == no_wait) {
            flags.fetch_and(static_cast<std::uint8_t>(~notify_flag::waiting), std::memory_order_relaxed);
            return std::nullopt;
        }

        ready_.remove(self);
        self.state = ThreadState::WaitingNotify;
        self.wait_index = static_cast<std::uint8_t>(index);
        if (timeout != wait_forever) {
            self.wake_tick = now_.load(std::memory_order_relaxed) + timeout;
            timers_.insert(self);
        }
        Port::request_context_switch();
        return std::nullopt;
    }

    /// Phase 2 of a notification wait: the value, or nullopt on timeout.
    std::optional<std::uint32_t> notify_wait_end(std::size_t index) noexcept {
        contract_assert(index < notify_slots);
        contract_assert(!Port::in_isr());
        Lock lock;
        Thread& self = *current_;
        const std::uint8_t prev = self.notify_flags[index].fetch_and(
            static_cast<std::uint8_t>(~notify_flag::waiting), std::memory_order_acq_rel);
        if ((prev & notify_flag::pending) != 0) {
            return consume(self, index);
        }
        return std::nullopt;
    }

    /// Blocking wait for a notification on `index` (FreeRTOS xTaskNotifyWaitIndexed
    /// with "clear all on exit"). Returns the accumulated value or nullopt on timeout.
    /// Note: a benign race (notifier between flag-clear and value-take) can yield 0
    /// for a consumed SetBits/Increment notification; no notification is ever lost.
    std::optional<std::uint32_t> notify_wait(std::size_t index, Tick timeout = wait_forever) noexcept {
        if (auto value = notify_wait_begin(index, timeout)) {
            return value;
        }
        if (timeout == no_wait) {
            return std::nullopt;
        }
        // On hardware the thread has been switched out and resumed at this point.
        return notify_wait_end(index);
    }

    // =====================================================================
    // Deferred work (lock-free posting from any context)
    // =====================================================================

    /// Queue `work` to run under the kernel lock at the next scheduling point.
    /// Lock-free: callable from interrupts of ANY priority. Posting an item that is
    /// already queued is a no-op.
    void defer_from_isr(Deferred& work) noexcept {
        contract_assert(Port::in_isr());
        defer_lock_free(work);
    }

    /// defer_from_isr() for thread context.
    void defer(Deferred& work) noexcept {
        contract_assert(!Port::in_isr());
        defer_lock_free(work);
    }

    // =====================================================================
    // Primitives for blocking kernel objects. *_locked: the caller holds Lock.
    // =====================================================================

    /// Block the current thread on `queue` (FIFO). `data` is handed to the waking
    /// side (e.g. a buffer address); the waker may replace it.
    void block_current_locked(WaitQueue& queue, Tick timeout, std::uintptr_t data = 0) noexcept {
        contract_assert(!Port::in_isr());
        contract_assert(timeout != no_wait);
        contract_assert(current_ != &idle_);
        Thread& self = *current_;
        ready_.remove(self);
        self.state = ThreadState::Waiting;
        self.wait_queue = &queue;
        self.wait_data = data;
        self.wait_result = WaitResult::Timeout;
        queue.waiters_.push_back(self.wait_link);
        queue.count_.fetch_add(1, std::memory_order_seq_cst);
        if (timeout != wait_forever) {
            self.wake_tick = now_.load(std::memory_order_relaxed) + timeout;
            timers_.insert(self);
        }
        Port::request_context_switch();
    }

    /// Undo block_current_locked() before the switch happened (the object became
    /// available in the meantime). The thread keeps its round-robin position.
    void cancel_wait_locked() noexcept {
        contract_assert(current_->state == ThreadState::Waiting);
        Thread& self = *current_;
        detach_from_wait_queue(self);
        TimerWheel<Thread, Cfg::timer_wheel_slots>::cancel(self);
        self.state = ThreadState::Ready;
        ready_.insert_front(self);
    }

    /// First thread blocked on `queue`, or nullptr. Its wait_data is readable.
    [[nodiscard]] static Thread* front_waiter_locked(const WaitQueue& queue) noexcept {
        return queue.waiters_.front();
    }

    /// Wake the first thread blocked on `queue` with WaitResult::Ok and `data`.
    /// Returns the woken thread or nullptr if none was waiting.
    Thread* wake_one_locked(WaitQueue& queue, std::uintptr_t data = 0) noexcept {
        Thread* t = queue.waiters_.front();
        if (t == nullptr) {
            return nullptr;
        }
        detach_from_wait_queue(*t);
        t->wait_result = WaitResult::Ok;
        t->wait_data = data;
        make_ready(*t);
        reschedule();
        return t;
    }

    /// Wake every thread blocked on `queue`. Returns the number woken.
    std::size_t wake_all_locked(WaitQueue& queue, std::uintptr_t data = 0) noexcept {
        std::size_t n = 0;
        while (wake_one_locked(queue, data) != nullptr) {
            ++n;
        }
        return n;
    }

    /// Phase 2 of an object wait: how the wait of the current thread ended.
    [[nodiscard]] WaitOutcome wait_end() const noexcept {
        contract_assert(!Port::in_isr());
        return {current_->wait_result, current_->wait_data};
    }

    // =====================================================================
    // Introspection
    // =====================================================================

    [[nodiscard]] Thread* current() const noexcept { return current_; }
    [[nodiscard]] Tick now() const noexcept { return now_.load(std::memory_order_relaxed); }
    /// Ticks since start as 64 bits: never wraps (now() wraps after 2^32 ticks, 49.7 days
    /// at 1 kHz). Lock-free: tick() updates both halves at kernel priority, so a reader that
    /// sees the same epoch before and after reading now() has a consistent pair.
    [[nodiscard]] std::uint64_t now64() const noexcept {
        for (;;) {
            const std::uint32_t epoch = epoch_.load(std::memory_order_acquire);
            const Tick low = now_.load(std::memory_order_acquire);
            if (epoch_.load(std::memory_order_acquire) == epoch) {
                return (std::uint64_t{epoch} << 32) | low;
            }
        }
    }
    [[nodiscard]] const Thread& idle_thread() const noexcept { return idle_; }
    [[nodiscard]] std::size_t thread_count() const noexcept { return registry_.size(); }
    [[nodiscard]] std::uint32_t context_switches() const noexcept { return context_switches_; }

private:
    static constexpr std::size_t wake_words = (Cfg::max_threads + 1 + 31) / 32;

    void register_thread(Thread& t, const ThreadParams& p) noexcept {
        contract_assert(p.entry != nullptr);
        contract_assert(p.stack.size() >= Port::min_stack_bytes);
        contract_assert(t.state == ThreadState::Inactive);
        Lock lock;
        const auto id = registry_.size();
        [[maybe_unused]] const bool registered = registry_.try_push_back(&t).has_value();
        contract_assert(registered);  // more than Cfg::max_threads threads

        t.id = static_cast<std::uint8_t>(id);
        t.name = p.name;
        t.entry = p.entry;
        t.arg = p.arg;
        t.priority = p.priority;
        t.threshold = p.preemption_threshold == no_threshold ? p.priority : p.preemption_threshold;
        t.time_slice = p.time_slice;
        t.slice_left = p.time_slice;
        // PSPLIM requires 8-byte alignment; round the stack bottom up.
        t.stack_limit = (reinterpret_cast<std::uintptr_t>(p.stack.data()) + 7u) & ~std::uintptr_t{7u};
        t.sp = Port::init_stack(p.stack, &thread_trampoline, this, &t);
        contract_assert(t.sp != nullptr);

        make_ready(t);
        reschedule();
    }

    bool resume_locked(Thread& t) noexcept {
        Lock lock;
        if (t.state != ThreadState::Suspended) {
            return false;
        }
        make_ready(t);
        reschedule();
        return true;
    }

    /// Selection rule. Pure: no side effects, so tick()/notify paths can use it to
    /// decide whether a switch is necessary at all.
    [[nodiscard]] Thread& pick() const noexcept {
        contract_assert(!ready_.empty());  // idle is always ready
        const unsigned top = ready_.highest();

        // 1. The running thread keeps the CPU unless someone beats its threshold.
        //    (`is_front` is false after a yield/time-slice rotation.)
        const Thread* cur = current_;
        if (cur != nullptr && cur->state == ThreadState::Ready && ready_.is_front(*cur) &&
            top >= cur->threshold) {
            return *current_;
        }
        // 2. A thread preempted while holding a threshold resumes before threads that
        //    could not have preempted it. The most urgent one is the innermost.
        if (!preempted_map_.empty()) {
            Thread* p = preempted_[preempted_map_.highest()];
            if (top >= p->threshold) {
                return *p;
            }
        }
        // 3. Plain highest-priority-first.
        return *ready_.front(top);
    }

    /// Pend a context switch if the selection changed.
    void reschedule() noexcept {
        if (started_ && &pick() != current_) {
            Port::request_context_switch();
        }
    }

    void make_ready(Thread& t) noexcept {
        TimerWheel<Thread, Cfg::timer_wheel_slots>::cancel(t);
        t.state = ThreadState::Ready;
        ready_.insert(t);
    }

    static void detach_from_wait_queue(Thread& t) noexcept {
        contract_assert(t.wait_queue != nullptr);
        t.wait_link.unlink();
        t.wait_queue->count_.fetch_sub(1, std::memory_order_seq_cst);
        t.wait_queue = nullptr;
    }

    void forget_preempted(const Thread& t) noexcept {
        if (preempted_map_.test(t.priority) && preempted_[t.priority] == &t) {
            preempted_map_.clear(t.priority);
            preempted_[t.priority] = nullptr;
        }
    }

    void notify_lock_free(Thread& t, std::size_t index, std::uint32_t value, NotifyAction action) noexcept {
        auto& slot = t.notify_value[index];
        switch (action) {
            case NotifyAction::None: break;
            case NotifyAction::SetBits: slot.fetch_or(value, std::memory_order_relaxed); break;
            case NotifyAction::Increment: slot.fetch_add(1, std::memory_order_relaxed); break;
            case NotifyAction::Overwrite: slot.store(value, std::memory_order_relaxed); break;
        }
        // Publish (release) the value update together with the pending flag.
        const std::uint8_t prev =
            t.notify_flags[index].fetch_or(notify_flag::pending, std::memory_order_acq_rel);
        if ((prev & notify_flag::waiting) != 0) {
            wake_requests_[t.id / 32].fetch_or(1u << (t.id % 32), std::memory_order_release);
            Port::request_context_switch();
        }
    }

    void defer_lock_free(Deferred& work) noexcept {
        if (work.queued_.exchange(true, std::memory_order_acq_rel)) {
            return;  // already pending
        }
        // Push-only Treiber stack; the consumer takes the whole list at once, so
        // there is no ABA problem.
        Deferred* head = deferred_.load(std::memory_order_relaxed);
        do {
            work.next_ = head;
        } while (!deferred_.compare_exchange_weak(head, &work, std::memory_order_release,
                                                  std::memory_order_relaxed));
        Port::request_context_switch();
    }

    /// Lock-free requests posted since the last scheduling point (lock held).
    void run_pending_work() noexcept {
        drain_wake_requests();
        run_deferred();
    }

    /// Turn lock-free wake requests posted by notify*() into ready threads.
    void drain_wake_requests() noexcept {
        for (std::size_t w = 0; w < wake_words; ++w) {
            std::uint32_t bits = wake_requests_[w].exchange(0, std::memory_order_acquire);
            while (bits != 0) {
                const auto bit = static_cast<std::size_t>(std::countr_zero(bits));
                bits &= bits - 1;
                Thread& t = *registry_[w * 32 + bit];
                // Ignore stale requests (thread already timed out / was resumed).
                if (t.state == ThreadState::WaitingNotify &&
                    (t.notify_flags[t.wait_index].load(std::memory_order_acquire) & notify_flag::pending) != 0) {
                    make_ready(t);
                }
            }
        }
    }

    /// Run deferred work in posting order (lock held).
    void run_deferred() noexcept {
        Deferred* stack = deferred_.exchange(nullptr, std::memory_order_acquire);
        Deferred* fifo = nullptr;
        while (stack != nullptr) {  // reverse: LIFO stack -> FIFO order
            Deferred* next = stack->next_;
            stack->next_ = fifo;
            fifo = stack;
            stack = next;
        }
        while (fifo != nullptr) {
            Deferred* next = fifo->next_;
            // Clear first: the work may be posted again while (or after) it runs.
            fifo->queued_.store(false, std::memory_order_release);
            fifo->function_(fifo->context_);
            fifo = next;
        }
    }

    static std::uint32_t consume(Thread& t, std::size_t index) noexcept {
        // Clear `pending` BEFORE taking the value: a concurrent notifier then re-arms
        // `pending`, so its update is never lost (worst case: a later wake with 0).
        t.notify_flags[index].fetch_and(static_cast<std::uint8_t>(~notify_flag::pending),
                                        std::memory_order_acq_rel);
        return t.notify_value[index].exchange(0, std::memory_order_acq_rel);
    }

    void exit_current() noexcept {
        contract_assert(current_ != &idle_);
        Lock lock;
        Thread& self = *current_;
        ready_.remove(self);
        forget_preempted(self);
        self.state = ThreadState::Terminated;
        Port::request_context_switch();
    }

    /// First code executed by every thread (set up by Port::init_stack).
    static void thread_trampoline(void* kernel, void* thread) {
        auto& self = *static_cast<Kernel*>(kernel);
        auto& t = *static_cast<Thread*>(thread);
        t.entry(t.arg);
        self.exit_current();
        Port::park();
    }

    static void idle_entry(void*) {
        for (;;) {
            Port::idle_wait();
        }
    }

    // ---- state -------------------------------------------------------------------
    ReadyQueue<Thread, priority_levels> ready_{};
    TimerWheel<Thread, Cfg::timer_wheel_slots> timers_{};
    std::inplace_vector<Thread*, Cfg::max_threads + 1> registry_{};  // +1: idle
    std::array<Thread*, priority_levels> preempted_{};
    PriorityBitmap preempted_map_{};
    std::array<std::atomic<std::uint32_t>, wake_words> wake_requests_{};
    std::atomic<Deferred*> deferred_{nullptr};
    std::atomic<Tick> now_{initial_tick_of<Cfg>()};
    std::atomic<std::uint32_t> epoch_{0};  ///< wraps of now_ (now64())
    Thread* current_ = nullptr;
    std::uint32_t context_switches_ = 0;
    std::uint32_t switched_in_cycles_ = 0;  ///< cycle count when current_ was switched in
    bool started_ = false;

    Thread idle_{};
    alignas(8) std::array<std::byte, Cfg::idle_stack_bytes> idle_stack_{};
};

}  // namespace rtos
