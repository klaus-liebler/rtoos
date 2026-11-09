// SPDX-License-Identifier: MIT
// ProfileStepper: step periods of a ramp chosen at run time (linear or sigmoid acceleration).
#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "rtos/motion/profile_stepper.hpp"

namespace {

using rtos::motion::ProfileStepper;
using rtos::motion::RampShape;
using rtos::motion::StepProfile;

constexpr std::uint32_t tick_hz = 1'000'000;

ProfileStepper make(RampShape shape, std::uint32_t start = 200, std::uint32_t cruise = 4000, std::uint32_t accel = 20'000) {
    ProfileStepper p;
    EXPECT_EQ(p.configure({.start_hz = start, .cruise_hz = cruise, .accel = accel, .shape = shape}, tick_hz, 0xFFFF), nullptr);
    return p;
}

TEST(ProfileStepper, LinearRampHasConstantAcceleration) {
    auto p = make(RampShape::Linear);
    // S = (4000^2 - 200^2) / (2 * 20000) = 399 steps
    EXPECT_EQ(p.ramp_steps(), 399u);
    p.begin(2000);
    EXPECT_EQ(p.period(0), 5000u) << "first step at the start rate";
    // v(s) = sqrt(v0^2 + 2 a s): the rate at the start of each step
    for (std::uint32_t s : {1u, 50u, 200u, 398u}) {
        const double v = std::sqrt(200.0 * 200 + 2.0 * 20'000 * s);
        EXPECT_NEAR(p.period(s), 1e6 / v, 1.0) << s;
    }
    // the time over the ramp is (vc - v0) / a = 0.19 s
    double t = 0;
    for (std::uint32_t s = 0; s < p.ramp_steps(); ++s) t += p.period(s);
    EXPECT_NEAR(t / 1e6, 0.19, 0.01);
    EXPECT_EQ(p.period(1000), 250u) << "cruise at 4000 steps/s";
}

TEST(ProfileStepper, SigmoidIsGentleAtBothEndsAndCoversTheSameDistance) {
    auto lin = make(RampShape::Linear), sig = make(RampShape::Sigmoid);
    EXPECT_EQ(sig.ramp_steps(), lin.ramp_steps());
    sig.begin(2000);
    lin.begin(2000);
    // near the start the sigmoid accelerates more slowly than the linear ramp, in the middle faster
    EXPECT_GT(sig.period(20), lin.period(20));
    const std::uint32_t mid = sig.ramp_steps() / 2;
    EXPECT_GT(sig.rate(mid + 10) - sig.rate(mid - 10), lin.rate(mid + 10) - lin.rate(mid - 10));
    // the rate never falls during the ramp, and it reaches the cruise rate at its end
    for (std::uint32_t s = 1; s < sig.ramp_steps(); ++s) EXPECT_GE(sig.rate(s), sig.rate(s - 1)) << s;
    EXPECT_NEAR(sig.rate(0), 200.0f, 0.5f);
    EXPECT_NEAR(sig.rate(sig.ramp_steps() - 1), 4000.0f, 40.0f);
}

TEST(ProfileStepper, DecelerationMirrorsTheRampAndShortMovesAreTriangles) {
    for (const auto shape : {RampShape::Linear, RampShape::Sigmoid}) {
        auto p = make(shape);
        p.begin(2000);
        for (std::uint32_t k = 0; k < 2000; ++k) EXPECT_EQ(p.period(k), p.period(1999 - k)) << k;
        // 101 steps: 50 up, the middle step, 50 down - never at the cruise rate
        p.begin(-101);
        EXPECT_EQ(p.total(), 101u);
        for (std::uint32_t k = 0; k < 101; ++k) EXPECT_GT(p.period(k), 250u) << k;
        EXPECT_LE(p.period(50), p.period(49));
        EXPECT_EQ(p.begin(0), 0u);
    }
}

TEST(ProfileStepper, RefusesWhatTheTimerCannotMakeAndKeepsTheOldProfile) {
    auto p = make(RampShape::Linear);
    EXPECT_NE(p.configure({.start_hz = 0, .cruise_hz = 100, .accel = 10}, tick_hz, 0xFFFF), nullptr);
    EXPECT_NE(p.configure({.start_hz = 100, .cruise_hz = 50, .accel = 10}, tick_hz, 0xFFFF), nullptr);
    EXPECT_NE(p.configure({.start_hz = 100, .cruise_hz = 200, .accel = 0}, tick_hz, 0xFFFF), nullptr);
    EXPECT_NE(p.configure({.start_hz = 10, .cruise_hz = 200, .accel = 10}, tick_hz, 0xFFFF), nullptr) << "100000 ticks > 16 bit";
    EXPECT_NE(p.configure({.start_hz = 100, .cruise_hz = 150'000, .accel = 10}, tick_hz, 0xFFFF), nullptr) << "< 10 ticks per step";
    EXPECT_EQ(p.profile().cruise_hz, 4000u) << "the old profile stays";
}

}  // namespace
