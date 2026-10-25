// SPDX-License-Identifier: MIT
//
// Host unit tests for the scheduler (fixture: kernel_fixture.hpp).
#include <gtest/gtest.h>

#include <thread>
#include <vector>

#include "kernel_fixture.hpp"

namespace {

using namespace test;
using rtos::NotifyAction;
using rtos::ThreadState;

// ---------------------------------------------------------------------------
// Basic priority scheduling
// ---------------------------------------------------------------------------

TEST_F(KernelTest, IdleRunsWhenNoThreadExists) {
    start();
    EXPECT_EQ(k.current(), &k.idle_thread());
}

TEST_F(KernelTest, StartSelectsMostUrgentThread) {
    create(a, 10);
    create(b, 5);
    create(c, 20);
    start();
    EXPECT_EQ(k.current(), &b.tcb);
}

TEST_F(KernelTest, CreatingMoreUrgentThreadPreempts) {
    create(a, 10);
    start();
    create(b, 5);
    EXPECT_TRUE(switch_pending());
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
}

TEST_F(KernelTest, CreatingLessUrgentThreadDoesNotSwitch) {
    create(a, 10);
    start();
    create(b, 15);
    EXPECT_FALSE(switch_pending());
}

// ---------------------------------------------------------------------------
// Preemption threshold (ThreadX semantics)
// ---------------------------------------------------------------------------

TEST_F(KernelTest, ThresholdBlocksPreemptionByIntermediatePriority) {
    create(a, 20, /*threshold=*/10);
    start();
    ASSERT_EQ(k.current(), &a.tcb);

    create(b, 15);  // more urgent than a's priority, but not than its threshold
    EXPECT_FALSE(switch_pending());
    EXPECT_EQ(k.current(), &a.tcb);
}

TEST_F(KernelTest, ThresholdAllowsPreemptionAboveIt_AndResumesPreemptedThreadFirst) {
    create(a, 20, /*threshold=*/10);
    start();
    create(b, 15);  // blocked by threshold
    create(c, 5);   // beats the threshold
    run_pendsv();
    ASSERT_EQ(k.current(), &c.tcb);

    // c blocks -> a must resume (it was preempted under threshold), NOT b,
    // although b has the more urgent priority.
    k.sleep(100);
    run_pendsv();
    EXPECT_EQ(k.current(), &a.tcb);

    // Once a blocks, b finally runs.
    k.sleep(100);
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
}

TEST_F(KernelTest, LoweringThresholdToPriorityAllowsPendingPreemption) {
    create(a, 20, /*threshold=*/10);
    start();
    create(b, 15);
    EXPECT_FALSE(switch_pending());

    k.set_preemption_threshold(a.tcb, 20);
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
}

// ---------------------------------------------------------------------------
// Time: sleep, timer wheel, round robin
// ---------------------------------------------------------------------------

TEST_F(KernelTest, SleepWakesExactlyAfterDelayAcrossWheelRevolutions) {
    create(a, 5);
    create(b, 10);
    start();
    k.sleep(20);  // wheel has 8 buckets -> 2.5 revolutions
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);
    EXPECT_EQ(a.tcb.state, ThreadState::Sleeping);

    tick(19);
    EXPECT_EQ(k.current(), &b.tcb);
    tick(1);
    EXPECT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.now(), 20u);
}

TEST_F(KernelTest, TimeSliceRotatesEqualPriorityThreads) {
    create(a, 10, Kernel::no_threshold, /*slice=*/2);
    create(b, 10, Kernel::no_threshold, /*slice=*/2);
    start();
    ASSERT_EQ(k.current(), &a.tcb);
    tick(1);
    EXPECT_EQ(k.current(), &a.tcb);
    tick(1);
    EXPECT_EQ(k.current(), &b.tcb);
    tick(2);
    EXPECT_EQ(k.current(), &a.tcb);
}

TEST_F(KernelTest, ActiveThresholdDisablesTimeSlicing) {
    create(a, 10, /*threshold=*/5, /*slice=*/1);
    create(b, 10, Kernel::no_threshold, /*slice=*/1);
    start();
    tick(5);
    EXPECT_EQ(k.current(), &a.tcb);
}

TEST_F(KernelTest, YieldRotatesOnlyWithinPriority) {
    create(a, 10);
    create(b, 10);
    create(c, 12);
    start();
    k.yield();
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
    k.yield();
    run_pendsv();
    EXPECT_EQ(k.current(), &a.tcb);
}

// ---------------------------------------------------------------------------
// Task notifications
// ---------------------------------------------------------------------------

TEST_F(KernelTest, PendingNotificationIsConsumedWithoutBlocking) {
    create(a, 5);
    start();
    k.notify(a.tcb, 0, 0x5, NotifyAction::SetBits);
    auto v = k.notify_wait_begin(0, rtos::wait_forever);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 0x5u);
    EXPECT_FALSE(switch_pending());
    EXPECT_EQ(a.tcb.state, ThreadState::Ready);
}

TEST_F(KernelTest, NotifyFromIsrWakesBlockedThreadAndPreempts) {
    create(a, 5);
    create(b, 10);
    start();
    EXPECT_FALSE(k.notify_wait_begin(1, rtos::wait_forever).has_value());
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);
    ASSERT_EQ(a.tcb.state, ThreadState::WaitingNotify);

    {
        ZeroLatencyIsrScope isr;  // above the kernel mask: lock-free path only
        const auto locks_before = MockPort::lock_depth;
        k.notify_from_isr(a.tcb, 1, 0x1, NotifyAction::SetBits);
        k.notify_from_isr(a.tcb, 1, 0x8, NotifyAction::SetBits);
        EXPECT_EQ(MockPort::lock_depth, locks_before);
    }

    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    auto v = k.notify_wait_end(1);
    ASSERT_TRUE(v.has_value());
    EXPECT_EQ(*v, 0x9u);
}

TEST_F(KernelTest, NotificationSlotsAreIndependent) {
    create(a, 5);
    create(b, 10);
    start();
    EXPECT_FALSE(k.notify_wait_begin(0, rtos::wait_forever).has_value());
    run_pendsv();
    k.notify(a.tcb, 1, 0xFF);  // other slot: must not wake
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
    EXPECT_EQ(a.tcb.state, ThreadState::WaitingNotify);
}

TEST_F(KernelTest, NotifyActions) {
    create(a, 5);
    start();
    for (int i = 0; i < 3; ++i) {
        k.notify(a.tcb, 0, 0, NotifyAction::Increment);
    }
    EXPECT_EQ(k.notify_wait_begin(0, rtos::no_wait), 3u);

    k.notify(a.tcb, 0, 7, NotifyAction::Overwrite);
    k.notify(a.tcb, 0, 42, NotifyAction::Overwrite);
    EXPECT_EQ(k.notify_wait_begin(0, rtos::no_wait), 42u);

    k.notify(a.tcb, 0, 0, NotifyAction::None);
    EXPECT_EQ(k.notify_wait_begin(0, rtos::no_wait), 0u);  // pending, value untouched
    EXPECT_FALSE(k.notify_wait_begin(0, rtos::no_wait).has_value());
}

TEST_F(KernelTest, NotifyWaitTimesOut) {
    create(a, 5);
    create(b, 10);
    start();
    EXPECT_FALSE(k.notify_wait_begin(0, 5).has_value());
    run_pendsv();
    tick(4);
    EXPECT_EQ(k.current(), &b.tcb);
    tick(1);
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_FALSE(k.notify_wait_end(0).has_value());
}

TEST_F(KernelTest, NotifyAfterTimeoutBeforeResumeIsDelivered) {
    create(a, 5);
    create(b, 10);
    start();
    (void)k.notify_wait_begin(0, 3);
    run_pendsv();
    tick(2);
    k.tick();  // a times out and becomes ready (switch pending, not yet taken)
    k.notify(a.tcb, 0, 0x2);  // stale wake request: a is already ready
    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.notify_wait_end(0), 0x2u);
}

TEST_F(KernelTest, ConcurrentLockFreeNotifiesAreNotLost) {
    create(a, 5);
    start();
    constexpr int producers = 4;
    constexpr int per_producer = 20000;
    std::vector<std::thread> threads;
    for (int p = 0; p < producers; ++p) {
        threads.emplace_back([&] {
            for (int i = 0; i < per_producer; ++i) {
                k.notify(a.tcb, 0, 0, NotifyAction::Increment);
            }
        });
    }
    for (auto& t : threads) {
        t.join();
    }
    EXPECT_EQ(k.notify_wait_begin(0, rtos::no_wait), std::uint32_t{producers * per_producer});
}

// ---------------------------------------------------------------------------
// Suspend / resume / exit
// ---------------------------------------------------------------------------

TEST_F(KernelTest, SuspendAndResume) {
    create(a, 5);
    create(b, 10);
    start();
    EXPECT_TRUE(k.suspend(a.tcb));
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
    EXPECT_FALSE(k.suspend(a.tcb));
    EXPECT_TRUE(k.resume(a.tcb));
    run_pendsv();
    EXPECT_EQ(k.current(), &a.tcb);
}

TEST_F(KernelTest, ReturningFromEntryTerminatesThread) {
    create(a, 5);
    create(b, 10);
    start();
    MockPort::run(a.tcb.sp);  // runs noop_entry, then the kernel's exit path
    EXPECT_EQ(a.tcb.state, ThreadState::Terminated);
    run_pendsv();
    EXPECT_EQ(k.current(), &b.tcb);
}

// ---------------------------------------------------------------------------
// Contracts
// ---------------------------------------------------------------------------

using KernelDeathTest = KernelTest;

TEST_F(KernelDeathTest, IdlePriorityIsReserved) {
    EXPECT_DEATH(create(a, Kernel::idle_priority), "assertion_kind: assert");
}

TEST_F(KernelDeathTest, ThresholdMustNotBeLessUrgentThanPriority) {
    EXPECT_DEATH(create(a, 10, /*threshold=*/11), "assertion_kind: assert");
}

TEST_F(KernelDeathTest, ThreadCannotBeCreatedTwice) {
    create(a, 10);
    EXPECT_DEATH(create(a, 10), "assertion_kind: assert");
}

TEST_F(KernelDeathTest, NotifySlotOutOfRange) {
    create(a, 10);
    EXPECT_DEATH(k.notify(a.tcb, TestConfig::notify_slots, 1), "assertion_kind: assert");
}

// ---------------------------------------------------------------------------
// Execution contexts: *_from_isr services
// ---------------------------------------------------------------------------

TEST_F(KernelTest, ResumeFromKernelAwareIsr) {
    create(a, 5);
    create(b, 10);
    start();
    ASSERT_TRUE(k.suspend(a.tcb));
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);
    {
        IsrScope isr;
        EXPECT_TRUE(k.resume_from_isr(a.tcb));
    }
    run_pendsv();
    EXPECT_EQ(k.current(), &a.tcb);
}

TEST_F(KernelDeathTest, ThreadServiceInIsrViolatesContract) {
    create(a, 10);
    IsrScope isr;
    EXPECT_DEATH(k.notify(a.tcb, 0, 1), "assertion_kind: assert");
}

TEST_F(KernelDeathTest, IsrServiceInThreadViolatesContract) {
    create(a, 10);
    EXPECT_DEATH(k.notify_from_isr(a.tcb, 0, 1), "assertion_kind: assert");
}

TEST_F(KernelDeathTest, LockingServiceInZeroLatencyIsrViolatesContract) {
    create(a, 10);
    start();
    ASSERT_TRUE(k.suspend(a.tcb));
    ZeroLatencyIsrScope isr;
    EXPECT_DEATH(k.resume_from_isr(a.tcb), "assertion_kind: assert");
}

// ---------------------------------------------------------------------------
// Deferred work
// ---------------------------------------------------------------------------

struct Counter {
    int runs = 0;
    static void run(void* self) noexcept { ++static_cast<Counter*>(self)->runs; }
};

TEST_F(KernelTest, DeferredWorkRunsOnceAtNextSchedulingPoint) {
    create(a, 5);
    start();
    Counter counter;
    rtos::Deferred work{&Counter::run, &counter};
    {
        ZeroLatencyIsrScope isr;
        const auto locks_before = MockPort::lock_depth;
        k.defer_from_isr(work);
        k.defer_from_isr(work);  // already queued: no-op
        EXPECT_EQ(MockPort::lock_depth, locks_before);
    }
    EXPECT_TRUE(switch_pending());
    EXPECT_EQ(counter.runs, 0);
    run_pendsv();
    EXPECT_EQ(counter.runs, 1);

    k.defer(work);  // re-arm after it ran
    tick();
    EXPECT_EQ(counter.runs, 2);
}

TEST_F(KernelTest, DeferredWorkRunsInPostingOrder) {
    start();
    static std::array<int, 3> order{};
    static int next = 0;
    next = 0;
    struct Tagged {
        int tag;
        static void run(void* self) noexcept { order[next++] = static_cast<Tagged*>(self)->tag; }
    };
    Tagged t1{1}, t2{2}, t3{3};
    rtos::Deferred d1{&Tagged::run, &t1}, d2{&Tagged::run, &t2}, d3{&Tagged::run, &t3};
    k.defer(d1);
    k.defer(d2);
    k.defer(d3);
    run_pendsv();
    EXPECT_EQ(order, (std::array<int, 3>{1, 2, 3}));
}

}  // namespace
