// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "rtos/motion/sigmoid_stepper.hpp"

namespace {

using rtos::motion::SigmoidRamp;

// The FCU's sorter axis: 100 -> 1000 steps/s in 0.5 s, 1 MHz timer ticks.
constexpr SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000, .ramp_us = 500'000};
using Stepper = rtos::motion::SigmoidStepper<ramp>;
constexpr const auto& table = Stepper::ramp;

TEST(SigmoidRamp, ConstantExpExact) {
    for (double x : {-6.0, -1.0, 0.0, 0.3, 2.5, 6.0}) {
        EXPECT_NEAR(rtos::motion::detail::exp(x), std::exp(x), std::exp(x) * 1e-12) << x;
    }
}

TEST(SigmoidRamp, TableRunsFromStartToCruiseInTheRampTime) {
    EXPECT_EQ(table.front(), 10'000u) << "first step at 100 steps/s";
    EXPECT_NEAR(1e6 / table.back(), 1000.0, 15.0) << "last step near the cruise rate";
    double total_us = 0;
    for (auto t : table) {
        total_us += t;
    }
    EXPECT_GE(total_us, 500'000.0);
    EXPECT_LT(total_us, 500'000.0 + table.back()) << "the ramp ends with the step that crosses its time";
    // integral of the rate over 0.5 s, between 100 and 1000 steps/s: about 275 steps
    EXPECT_GT(table.size(), 250u);
    EXPECT_LT(table.size(), 300u);
}

TEST(SigmoidRamp, EveryStepFollowsTheSigmoidAndTheRateNeverFalls) {
    double t_us = 0;
    for (std::size_t i = 0; i < table.size(); ++i) {
        const double want = rtos::motion::sigmoid_rate(ramp, t_us);
        EXPECT_NEAR(1e6 / table[i], want, want * 0.001 + 1e6 / table[i] - 1e6 / (table[i] + 0.5)) << i;
        if (i > 0) {
            EXPECT_LE(table[i], table[i - 1]) << i;
        }
        t_us += table[i];
    }
}

TEST(SigmoidRamp, AccelerationPeaksInTheMiddleAndIsGentleAtBothEnds) {
    // acceleration between consecutive steps: dv / dt
    double t_us = 0, best = 0, best_at = 0;
    std::vector<std::pair<double, double>> accel;
    for (std::size_t i = 1; i < table.size(); ++i) {
        const double v0 = 1e6 / table[i - 1], v1 = 1e6 / table[i];
        t_us += table[i - 1];
        const double a = (v1 - v0) / (table[i - 1] * 1e-6);
        accel.push_back({t_us, a});
        if (a > best) {
            best = a;
            best_at = t_us;
        }
    }
    EXPECT_NEAR(best_at, 250'000.0, 50'000.0) << "strongest acceleration near the ramp's middle";
    EXPECT_LT(accel.front().second, best * 0.1) << "gentle start";
    EXPECT_LT(accel.back().second, best * 0.1) << "gentle end";
}

std::vector<std::uint32_t> run(Stepper& s, std::int32_t steps) {
    std::vector<std::uint32_t> periods;
    for (std::uint32_t p = s.begin(steps); p != 0; p = s.next()) {
        periods.push_back(p);
    }
    return periods;
}

TEST(SigmoidStepper, LongMoveAcceleratesCruisesAndDeceleratesSymmetrically) {
    Stepper s;
    const auto p = run(s, 2000);
    ASSERT_EQ(p.size(), 2000u);
    EXPECT_EQ(s.position(), 2000);
    EXPECT_FALSE(s.moving());
    for (std::size_t i = 0; i < table.size(); ++i) {
        EXPECT_EQ(p[i], table[i]);
        EXPECT_EQ(p[p.size() - 1 - i], table[i]) << "deceleration mirrors the ramp";
    }
    for (std::size_t i = table.size(); i < p.size() - table.size(); ++i) {
        EXPECT_EQ(p[i], Stepper::cruise_ticks);
    }
}

TEST(SigmoidStepper, ShortMoveIsASymmetricTriangleBelowCruise) {
    Stepper s;
    const auto p = run(s, 101);
    ASSERT_EQ(p.size(), 101u);
    for (std::size_t i = 0; i < p.size(); ++i) {
        EXPECT_EQ(p[i], p[p.size() - 1 - i]) << i;
    }
    EXPECT_GT(*std::min_element(p.begin(), p.end()), Stepper::cruise_ticks);
    EXPECT_EQ(p.front(), table.front());
}

TEST(SigmoidStepper, DirectionPositionAndEmptyMoves) {
    Stepper s;
    run(s, 10);
    run(s, -25);
    EXPECT_EQ(s.position(), -15);
    EXPECT_FALSE(s.forward());
    EXPECT_EQ(s.begin(0), 0u);
    EXPECT_EQ(s.next(), 0u);
    EXPECT_EQ(s.position(), -15);
    const auto one = run(s, 1);
    ASSERT_EQ(one.size(), 1u);
    EXPECT_EQ(one[0], table.front());
}

// The product's 16-bit timers: the slowest step must fit a 16-bit period.
static_assert(Stepper::max_ticks <= 0x10000);

}  // namespace
