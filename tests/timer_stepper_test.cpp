// SPDX-License-Identifier: MIT
//
// TimerStepper against a tick-exact simulation of an STM32 timer (up-counting, ARR preload,
// one-pulse mode, toggle output on CCR, update DMA request) and a GPDMA channel. The interrupt
// handlers run a given number of ticks after their flag (interrupt latency): every edge must
// still come exactly at the planned time, the move must end after exactly N edges.
#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <vector>

#include "rtos/motion/timer_stepper.hpp"

namespace {

using rtos::motion::SigmoidRamp;
constexpr SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000, .ramp_us = 500'000};

struct DmaConfigMock {
    std::uint8_t memory_bytes;
    std::uint8_t peripheral_bytes;
};

struct Hw {
    // timer
    std::uint32_t cnt = 0, arr = 0xFFFF, arr_active = 0xFFFF, ccr = 0;
    bool cen = false, opm = false, ude = false, uie = false, uif = false, toggle = false, out = false;
    // DMA
    bool dma_enabled = false, tc = false, tc_ie = false;
    const std::uint8_t* src = nullptr;
    std::uint32_t bytes = 0;
    // log
    std::uint64_t now = 0;
    std::vector<std::uint64_t> edges;
    int dma_starts = 0;
};
Hw hw;

struct TimMock {
    static constexpr bool preload = true;
    static constexpr std::uint32_t tick_hz = 1'000'000;
    static constexpr std::uint32_t max_period = 0xFFFF;
    static void init() noexcept { hw = Hw{}; }
    template <unsigned>
    static void set_compare(std::uint32_t v) noexcept { hw.ccr = v; }
    template <unsigned>
    static void configure_toggle() noexcept { hw.toggle = true; }
    static void start() noexcept { hw.cen = true; }
    static void stop() noexcept { hw.cen = false; }
    static void set_one_pulse(bool on) noexcept { hw.opm = on; }
    static void set_period(std::uint32_t ticks) noexcept { hw.arr = ticks - 1; }  // preloaded
    static void generate_update() noexcept {
        hw.cnt = 0;
        hw.arr_active = hw.arr;
        hw.uif = true;
    }
    static void set_counter(std::uint32_t v) noexcept { hw.cnt = v; }
    static bool take_update() noexcept {
        const bool f = hw.uif;
        hw.uif = false;
        return f;
    }
    static void enable_update_dma() noexcept { hw.ude = true; }
    static void disable_update_dma() noexcept { hw.ude = false; }
    static void enable_update_irq() noexcept { hw.uie = true; }
    static void disable_update_irq() noexcept { hw.uie = false; }
    static constexpr std::uintptr_t arr_address() noexcept { return 0x4000'042C; }
};

template <std::uint8_t ItemBytes>
struct DmaMock {
    static constexpr DmaConfigMock config{.memory_bytes = ItemBytes, .peripheral_bytes = 4};
    static void init() noexcept {}
    static void stop() noexcept { hw.dma_enabled = false; }
    static bool start(std::uintptr_t, const void* memory, std::size_t items, bool interrupt) noexcept {
        if (hw.dma_enabled || items == 0) {
            return false;
        }
        hw.src = static_cast<const std::uint8_t*>(memory);
        hw.bytes = static_cast<std::uint32_t>(items * ItemBytes);
        hw.tc_ie = interrupt;
        hw.dma_enabled = true;
        ++hw.dma_starts;
        return true;
    }
    static void clear() noexcept { hw.tc = false; }
    static std::uint32_t remaining() noexcept { return hw.dma_enabled ? hw.bytes : 0; }
    /// One request: one item into ARR.
    static void request() noexcept {
        if (!hw.dma_enabled) {
            return;
        }
        std::uint32_t v = 0;
        std::memcpy(&v, hw.src, ItemBytes);
        hw.arr = v;
        hw.src += ItemBytes;
        hw.bytes -= ItemBytes;
        if (hw.bytes == 0) {
            hw.dma_enabled = false;
            hw.tc = true;
        }
    }
};

/// Runs the simulated hardware until the move ends (or `limit` ticks); the handlers run
/// `latency` ticks after their flag.
template <class Stepper, class Dma>
void run(Stepper& s, std::uint64_t latency, std::uint64_t limit = 5'000'000) {
    std::uint64_t tc_at = 0, uif_at = 0;
    bool tc_seen = false, uif_seen = false;
    for (std::uint64_t t = 0; t < limit && s.moving(); ++t) {
        ++hw.now;
        if (hw.cen) {
            if (hw.cnt >= hw.arr_active) {  // update event
                hw.cnt = 0;
                hw.arr_active = hw.arr;
                hw.uif = true;
                if (hw.ude) {
                    Dma::request();
                }
                if (hw.opm) {
                    hw.cen = false;
                }
            } else {
                ++hw.cnt;
                if (hw.cnt == hw.ccr && hw.toggle) {
                    hw.out = !hw.out;
                    hw.edges.push_back(hw.now);
                }
            }
        }
        if (hw.tc && hw.tc_ie && !tc_seen) {
            tc_seen = true;
            tc_at = hw.now + latency;
        }
        if (tc_seen && hw.now >= tc_at) {
            tc_seen = false;
            s.on_dma_irq();
        }
        if (hw.uif && hw.uie && !uif_seen) {
            uif_seen = true;
            uif_at = hw.now + latency;
        }
        if (uif_seen && hw.now >= uif_at) {
            uif_seen = false;
            (void)s.on_timer_irq();
        }
    }
}

template <std::uint8_t ItemBytes>
void check_move(std::int32_t steps, std::uint64_t latency) {
    using Dma = DmaMock<ItemBytes>;
    rtos::motion::TimerStepper<TimMock, 1, Dma, ramp, 32> stepper;
    stepper.init();
    stepper.set_position(1000);
    ASSERT_TRUE(stepper.start(steps));
    const auto& plan = stepper.plan();
    const std::uint32_t n = plan.total();
    run<decltype(stepper), Dma>(stepper, latency);
    ASSERT_FALSE(stepper.moving()) << "the move did not end";
    ASSERT_EQ(hw.edges.size(), n) << steps << " steps, latency " << latency;
    std::uint64_t t = 0;
    for (std::uint32_t k = 0; k < n; ++k) {
        t += plan.period(k);
        ASSERT_EQ(hw.edges[k], t) << "edge " << k + 1 << " of " << n << ", latency " << latency;
    }
    EXPECT_FALSE(hw.cen) << "the counter must stop after the last edge";
    EXPECT_EQ(stepper.position(), 1000 + steps);
    EXPECT_EQ(hw.dma_starts, static_cast<int>((n + 31) / 32));
}

TEST(TimerStepper, EveryEdgeAtItsPlannedTick) {
    for (std::int32_t steps : {1, 2, 3, 31, 32, 33, 64, 65, 101, 2000}) {
        check_move<2>(steps, 0);
    }
}

TEST(TimerStepper, InterruptLatencyBelowOneStepDoesNotMoveAnEdge) {
    // the cruise step lasts 1000 ticks: the DMA refill and the one-pulse switch may come late
    for (std::uint64_t latency : {1u, 100u, 900u}) {
        for (std::int32_t steps : {1, 33, 2000}) {
            check_move<2>(steps, latency);
        }
    }
}

TEST(TimerStepper, BackwardsAndThirtyTwoBitItems) {
    check_move<4>(-500, 50);
    check_move<4>(-1, 0);
}

TEST(TimerStepper, StopAbortsAndCountsTheStepsDone) {
    using Dma = DmaMock<2>;
    rtos::motion::TimerStepper<TimMock, 1, Dma, ramp, 32> stepper;
    stepper.init();
    ASSERT_TRUE(stepper.start(2000));
    run<decltype(stepper), Dma>(stepper, 0, 300'000);  // 0.3 s: still in the ramp
    ASSERT_TRUE(stepper.moving());
    const std::size_t edges = hw.edges.size();
    stepper.stop();
    EXPECT_FALSE(stepper.moving());
    EXPECT_FALSE(hw.cen);
    EXPECT_NEAR(stepper.position(), static_cast<double>(edges), 1.0);
    EXPECT_FALSE(stepper.start(0));
    EXPECT_TRUE(stepper.start(-5)) << "a new move after the abort";
}

// The ramp chosen at run time (ProfileStepper): the same edge-exact hardware path, periods computed
// during the move by the DMA refill.
TEST(ProfileTimerStepper, EveryEdgeAtItsPlannedTickWithBothShapes) {
    using Dma = DmaMock<2>;
    for (const auto shape : {rtos::motion::RampShape::Linear, rtos::motion::RampShape::Sigmoid}) {
        for (std::int32_t steps : {1, 33, 700, -3000}) {
            rtos::motion::ProfileTimerStepper<TimMock, 1, Dma, 32> stepper;
            stepper.init();
            ASSERT_NE(stepper.motion(), nullptr);
            ASSERT_EQ(stepper.motion()->configure({.start_hz = 200, .cruise_hz = 4000, .accel = 20'000, .shape = shape},
                                                  1'000'000, decltype(stepper)::max_ticks), nullptr);
            ASSERT_TRUE(stepper.start(steps));
            EXPECT_EQ(stepper.motion(), nullptr) << "no new profile while moving";
            const auto& plan = stepper.plan();
            const std::uint32_t n = plan.total();
            run<decltype(stepper), Dma>(stepper, 100);
            ASSERT_FALSE(stepper.moving());
            ASSERT_EQ(hw.edges.size(), n);
            std::uint64_t t = 0;
            for (std::uint32_t k = 0; k < n; ++k) {
                t += plan.period(k);
                ASSERT_EQ(hw.edges[k], t) << "edge " << k + 1 << " of " << n;
            }
            EXPECT_EQ(stepper.position(), steps);
        }
    }
}

TEST(ProfileTimerStepper, AProfileTheTimerCannotMakeIsRefused) {
    rtos::motion::ProfileTimerStepper<TimMock, 1, DmaMock<2>, 32> stepper;
    auto* m = stepper.motion();
    // 1 MHz ticks, 16-bit items: the first step at most 65535 ticks (15.3 steps/s), at least 10 ticks
    EXPECT_NE(m->configure({.start_hz = 10, .cruise_hz = 1000, .accel = 1000}, 1'000'000, decltype(stepper)::max_ticks), nullptr);
    EXPECT_NE(m->configure({.start_hz = 100, .cruise_hz = 200'000, .accel = 1000}, 1'000'000, decltype(stepper)::max_ticks), nullptr);
    EXPECT_NE(m->configure({.start_hz = 500, .cruise_hz = 400, .accel = 1000}, 1'000'000, decltype(stepper)::max_ticks), nullptr);
    EXPECT_EQ(m->configure({.start_hz = 16, .cruise_hz = 100'000, .accel = 1000}, 1'000'000, decltype(stepper)::max_ticks), nullptr);
}

}  // namespace
