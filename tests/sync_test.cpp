// SPDX-License-Identifier: MIT
//
// Host unit tests for the blocking kernel objects (fixture: kernel_fixture.hpp).
// Two-phase pattern: *_begin() as the current thread, run_pendsv() to switch,
// later the thread is current again and reads kernel.wait_end().
#include <gtest/gtest.h>

#include <cstdint>

#include "kernel_fixture.hpp"
#include "rtos/sync/message_queue.hpp"
#include "rtos/sync/object_pool.hpp"
#include "rtos/sync/semaphore.hpp"

namespace {

using namespace test;
using rtos::ThreadState;
using rtos::WaitResult;

// ---------------------------------------------------------------------------
// Semaphore
// ---------------------------------------------------------------------------

class SemaphoreTest : public KernelTest {
protected:
    rtos::Semaphore<Kernel> sem{k, 1, 2};
};

TEST_F(SemaphoreTest, TakeAndGiveWithinCount) {
    create(a, 5);
    start();
    EXPECT_TRUE(sem.try_take());
    EXPECT_FALSE(sem.try_take());
    EXPECT_TRUE(sem.give());
    EXPECT_TRUE(sem.give());
    EXPECT_FALSE(sem.give());  // max = 2
    EXPECT_EQ(sem.count(), 2u);
}

TEST_F(SemaphoreTest, GiveHandsUnitToBlockedThread) {
    create(a, 5);
    create(b, 10);
    start();
    ASSERT_TRUE(sem.try_take());
    EXPECT_FALSE(sem.take_begin(rtos::wait_forever).has_value());  // a blocks
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);
    EXPECT_EQ(a.tcb.state, ThreadState::Waiting);

    EXPECT_TRUE(sem.give());  // b gives -> a (more urgent) preempts
    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Ok);
    EXPECT_EQ(sem.count(), 0u);  // handed over, not counted
}

TEST_F(SemaphoreTest, TakeTimesOut) {
    create(a, 5);
    create(b, 10);
    start();
    ASSERT_TRUE(sem.try_take());
    (void)sem.take_begin(3);
    run_pendsv();
    tick(3);
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Timeout);
    EXPECT_TRUE(sem.give());  // nobody waits any more: counted
    EXPECT_EQ(sem.count(), 1u);
}

TEST_F(SemaphoreTest, WaitersAreServedFifo) {
    create(a, 5);
    create(b, 5);
    create(c, 20);
    start();
    ASSERT_TRUE(sem.try_take());
    (void)sem.take_begin(rtos::wait_forever);  // a
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);
    (void)sem.take_begin(rtos::wait_forever);  // b
    run_pendsv();
    ASSERT_EQ(k.current(), &c.tcb);

    EXPECT_TRUE(sem.give());
    run_pendsv();
    EXPECT_EQ(k.current(), &a.tcb);  // first come, first served
    EXPECT_EQ(b.tcb.state, ThreadState::Waiting);
}

TEST_F(SemaphoreTest, GiveFromKernelAwareIsr) {
    create(a, 5);
    create(b, 10);
    start();
    ASSERT_TRUE(sem.try_take());
    (void)sem.take_begin(rtos::wait_forever);
    run_pendsv();
    {
        IsrScope isr;
        EXPECT_TRUE(sem.give_from_isr());
    }
    run_pendsv();
    EXPECT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Ok);
}

using SemaphoreDeathTest = SemaphoreTest;

TEST_F(SemaphoreDeathTest, GiveFromZeroLatencyIsrViolatesContract) {
    ZeroLatencyIsrScope isr;
    EXPECT_DEATH((void)sem.give_from_isr(), "assertion_kind: assert");
}

// ---------------------------------------------------------------------------
// MessageQueue
// ---------------------------------------------------------------------------

class MessageQueueTest : public KernelTest {
protected:
    rtos::MessageQueue<Kernel, std::uint32_t, 2> queue{k};
};

TEST_F(MessageQueueTest, FifoWithinCapacity) {
    create(a, 5);
    start();
    EXPECT_TRUE(queue.try_send(1));
    EXPECT_TRUE(queue.try_send(2));
    EXPECT_FALSE(queue.try_send(3));
    EXPECT_EQ(queue.try_receive(), 1u);
    EXPECT_EQ(queue.try_receive(), 2u);
    EXPECT_FALSE(queue.try_receive().has_value());
}

TEST_F(MessageQueueTest, SenderHandsMessageToBlockedReceiver) {
    create(a, 5);
    create(b, 10);
    start();
    std::uint32_t out = 0;
    EXPECT_FALSE(queue.receive_begin(out, rtos::wait_forever).has_value());  // a blocks
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);

    EXPECT_TRUE(queue.try_send(42));
    EXPECT_EQ(out, 42u);           // copied straight into the receiver's buffer
    EXPECT_EQ(queue.size(), 0u);   // ring untouched
    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Ok);
}

TEST_F(MessageQueueTest, BlockedSenderIsAppendedInOrder) {
    create(a, 5);
    create(b, 10);
    start();
    ASSERT_TRUE(queue.try_send(1));
    ASSERT_TRUE(queue.try_send(2));
    const std::uint32_t third = 3;
    EXPECT_FALSE(queue.send_begin(third, rtos::wait_forever).has_value());  // a blocks: full
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);

    EXPECT_EQ(queue.try_receive(), 1u);  // makes room -> a's message moves in
    EXPECT_EQ(queue.size(), 2u);
    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Ok);
    EXPECT_EQ(queue.try_receive(), 2u);
    EXPECT_EQ(queue.try_receive(), 3u);
}

TEST_F(MessageQueueTest, ReceiveTimesOut) {
    create(a, 5);
    create(b, 10);
    start();
    std::uint32_t out = 0;
    (void)queue.receive_begin(out, 4);
    run_pendsv();
    tick(4);
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Timeout);
}

TEST_F(MessageQueueTest, IsrSendWakesReceiver) {
    create(a, 5);
    create(b, 10);
    start();
    std::uint32_t out = 0;
    (void)queue.receive_begin(out, rtos::wait_forever);
    run_pendsv();
    {
        IsrScope isr;
        EXPECT_TRUE(queue.try_send_from_isr(7));
        EXPECT_TRUE(queue.try_send_from_isr(8));  // into the ring
        EXPECT_EQ(queue.try_receive_from_isr(), 8u);
    }
    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(out, 7u);
}

// ---------------------------------------------------------------------------
// ObjectPool
// ---------------------------------------------------------------------------

struct Message {
    std::uint32_t id;
    static inline int alive = 0;
    explicit Message(std::uint32_t i) noexcept : id(i) { ++alive; }
    ~Message() { --alive; }
};

class ObjectPoolTest : public KernelTest {
protected:
    void SetUp() override { Message::alive = 0; }
    rtos::ObjectPool<Kernel, Message, 2> pool{k};
};

TEST_F(ObjectPoolTest, MakeConstructsAndHandleDestroys) {
    create(a, 5);
    start();
    {
        auto m = pool.try_make(7u);
        ASSERT_TRUE(m);
        EXPECT_EQ(m->id, 7u);
        EXPECT_EQ(Message::alive, 1);
        EXPECT_EQ(pool.available(), 1u);
    }
    EXPECT_EQ(Message::alive, 0);
    EXPECT_EQ(pool.available(), 2u);
}

TEST_F(ObjectPoolTest, ExhaustedTryMakeFails) {
    create(a, 5);
    start();
    auto m1 = pool.try_make(1u);
    auto m2 = pool.try_make(2u);
    EXPECT_FALSE(pool.try_make(3u));
    EXPECT_FALSE(pool.make(rtos::no_wait, 3u));
}

TEST_F(ObjectPoolTest, FreeHandsBlockToBlockedAllocator) {
    create(a, 5);
    create(b, 10);
    start();
    void* b1 = pool.try_allocate();
    void* b2 = pool.try_allocate();
    ASSERT_NE(b2, nullptr);
    EXPECT_FALSE(pool.allocate_begin(rtos::wait_forever).has_value());  // a blocks
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);

    pool.deallocate(b1);
    EXPECT_EQ(pool.available(), 0u);  // went straight to a
    run_pendsv();
    ASSERT_EQ(k.current(), &a.tcb);
    const auto outcome = k.wait_end();
    EXPECT_EQ(outcome.result, WaitResult::Ok);
    EXPECT_EQ(reinterpret_cast<void*>(outcome.data), b1);
    pool.deallocate(b1);
    pool.deallocate(b2);
}

TEST_F(ObjectPoolTest, AllocateTimesOut) {
    create(a, 5);
    create(b, 10);
    start();
    void* b1 = pool.try_allocate();
    void* b2 = pool.try_allocate();
    (void)pool.allocate_begin(2);
    run_pendsv();
    tick(2);
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(k.wait_end().result, WaitResult::Timeout);
    pool.deallocate(b1);  // nobody waits: back to the free list
    EXPECT_EQ(pool.available(), 1u);
    pool.deallocate(b2);
}

TEST_F(ObjectPoolTest, ZeroLatencyIsrFreeWakesAllocatorViaDeferredRefill) {
    create(a, 5);
    create(b, 10);
    start();
    void* b1 = pool.try_allocate();
    void* b2 = pool.try_allocate();
    (void)pool.allocate_begin(rtos::wait_forever);
    run_pendsv();
    ASSERT_EQ(k.current(), &b.tcb);
    {
        ZeroLatencyIsrScope isr;
        const auto locks_before = MockPort::lock_depth;
        pool.deallocate_from_isr(b1);  // lock-free; refill deferred
        EXPECT_EQ(MockPort::lock_depth, locks_before);
    }
    EXPECT_EQ(a.tcb.state, ThreadState::Waiting);  // not yet: runs at PendSV
    run_pendsv();                                  // refill + switch to a
    ASSERT_EQ(k.current(), &a.tcb);
    EXPECT_EQ(reinterpret_cast<void*>(k.wait_end().data), b1);
    pool.deallocate(b1);
    pool.deallocate(b2);
}

TEST_F(ObjectPoolTest, KernelAwareIsrFreeHandsOverDirectly) {
    create(a, 5);
    create(b, 10);
    start();
    void* b1 = pool.try_allocate();
    void* b2 = pool.try_allocate();
    (void)pool.allocate_begin(rtos::wait_forever);
    run_pendsv();
    {
        IsrScope isr;
        pool.deallocate_from_isr(b2);
    }
    EXPECT_EQ(a.tcb.state, ThreadState::Ready);
    run_pendsv();
    EXPECT_EQ(reinterpret_cast<void*>(k.wait_end().data), b2);
    pool.deallocate(b1);
    pool.deallocate(b2);
}

TEST_F(ObjectPoolTest, TryMakeAndDestroyFromZeroLatencyIsr) {
    create(a, 5);
    start();
    ZeroLatencyIsrScope isr;
    auto m = pool.try_make(9u);
    ASSERT_TRUE(m);
    pool.destroy_from_isr(m.release());
    EXPECT_EQ(Message::alive, 0);
    EXPECT_EQ(pool.available(), 2u);
}

using ObjectPoolDeathTest = ObjectPoolTest;

TEST_F(ObjectPoolDeathTest, ForeignPointerViolatesContract) {
    int foreign = 0;
    EXPECT_DEATH(pool.deallocate(&foreign), "assertion_kind: assert");
}

}  // namespace
