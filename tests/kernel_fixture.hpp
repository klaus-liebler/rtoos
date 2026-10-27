// SPDX-License-Identifier: MIT
//
// Test fixture for kernel-level tests. The test acts as the "hardware":
//   * Calls made "as the current thread" are plain kernel calls.
//   * When the kernel requested a context switch, run_pendsv() performs it, exactly
//     like the PendSV exception does on the target.
//   * IsrScope / ZeroLatencyIsrScope put the mock port into interrupt context.
#pragma once

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "mock_port.hpp"
#include "rtos/kernel/kernel.hpp"

namespace test {

struct TestConfig {
    static constexpr std::size_t max_threads = 8;
    static constexpr std::size_t priority_levels = 32;
    static constexpr std::size_t notify_slots = 2;
    static constexpr std::size_t timer_wheel_slots = 8;  // small: exercises multi-revolution timeouts
    static constexpr std::size_t idle_stack_bytes = 256;
};

using Kernel = rtos::Kernel<MockPort, TestConfig>;

/// TestConfig starting 16 ticks before the 32-bit wrap of now().
struct WrapConfig : TestConfig {
    static constexpr rtos::Tick initial_tick = 0xFFFF'FFF0u;
};

inline void noop_entry(void*) {}

struct TestThread {
    Kernel::Thread tcb;
    alignas(8) std::array<std::byte, 256> stack{};
};

/// Kernel-aware interrupt context for the lifetime of the object.
struct IsrScope {
    explicit IsrScope(bool zero_latency = false) {
        MockPort::isr = true;
        MockPort::zero_latency_isr = zero_latency;
    }
    ~IsrScope() {
        MockPort::isr = false;
        MockPort::zero_latency_isr = false;
    }
};

/// Interrupt above the kernel mask: only lock-free services allowed.
struct ZeroLatencyIsrScope : IsrScope {
    ZeroLatencyIsrScope() : IsrScope(true) {}
};

// Resets the mock before the kernel member is constructed.
struct MockReset {
    MockReset() { MockPort::reset(); }
};

template <class Config>
class BasicKernelTest : public ::testing::Test, private MockReset {
protected:
    using Kernel = rtos::Kernel<MockPort, Config>;
    Kernel k;
    TestThread a, b, c;

    void create(TestThread& t, std::uint8_t prio, std::uint8_t threshold = Kernel::no_threshold,
                std::uint32_t slice = 0, Kernel::Entry entry = &noop_entry) {
        k.create_thread(t.tcb, {.name = "t",
                                .entry = entry,
                                .arg = nullptr,
                                .stack = t.stack,
                                .priority = prio,
                                .preemption_threshold = threshold,
                                .time_slice = slice});
    }

    bool switch_pending() const { return MockPort::switch_requests.load() != 0; }

    /// Hardware model: PendSV runs if (and only if) it was pended.
    void run_pendsv() {
        if (MockPort::switch_requests.exchange(0) != 0) {
            k.switch_context(k.current()->sp);
        }
    }

    void tick(int n = 1) {
        for (int i = 0; i < n; ++i) {
            k.tick();
            run_pendsv();
        }
    }

    void start() {
        k.prepare_start();
        MockPort::switch_requests = 0;
    }

    void TearDown() override { EXPECT_EQ(MockPort::lock_depth, 0u) << "unbalanced kernel lock"; }
};

using KernelTest = BasicKernelTest<TestConfig>;
using KernelWrapTest = BasicKernelTest<WrapConfig>;

}  // namespace test
