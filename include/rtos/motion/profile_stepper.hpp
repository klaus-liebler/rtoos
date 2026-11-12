// SPDX-License-Identifier: MIT
//
// Stepper moves with a speed profile chosen at run time (SigmoidStepper: fixed by the compiler).
// The step rate depends on the step index within the ramp, so every period is computed when it
// is needed - by TimerStepper's DMA refill, a few dozen steps ahead - and no table is kept:
//
//   Linear  constant acceleration a (steps/s^2): v(s) = sqrt(v0^2 + 2 a s); the ramp ends at the
//           cruise rate after S = (vc^2 - v0^2) / (2 a) steps.
//   Sigmoid the same distance S, the rate along a logistic curve over the distance:
//           v(s) = v0 + (vc - v0) * (sig(k s/S - k/2) - sig(-k/2)) / (sig(k/2) - sig(-k/2)) -
//           gentle at both ends (no jerk at the start and at the cruise rate), the strongest
//           acceleration in the middle (about k/4 times the linear one at k = 10: ~2.5).
//
// A move of n steps accelerates over min(S, n/2) steps, cruises, and decelerates through the
// same periods in reverse: short moves get a symmetric triangle (the interface of SigmoidStepper
// that TimerStepper uses: begin(), period(), total()).
//
//   ProfileStepper p;
//   p.configure({.start_hz = 200, .cruise_hz = 8000, .accel = 20000, .shape = RampShape::Sigmoid},
//               1'000'000, 0xFFFF);   // timer ticks per second, longest period a timer/DMA item holds
//   p.begin(5000);  ... p.period(k) for k = 0 .. 4999
#pragma once

#include <cmath>
#include <cstdint>

namespace rtos::motion {

enum class RampShape : std::uint8_t { Linear = 0, Sigmoid = 1 };

struct StepProfile {
    std::uint32_t start_hz = 100;    ///< step rate at standstill (start and stop)
    std::uint32_t cruise_hz = 1000;  ///< step rate after the ramp
    std::uint32_t accel = 1800;      ///< steps/s^2 of the linear ramp (the sigmoid: the same distance)
    RampShape shape = RampShape::Sigmoid;
    std::uint32_t steepness = 10;    ///< k of the sigmoid (2..30)
};

class ProfileStepper {
public:
    /// Takes `p` for the next moves if the timer can make it: at least 10 ticks per step at the
    /// cruise rate, the first (slowest) step at most `max_ticks`. Returns why not (the old
    /// profile stays) or nullptr. Not while a move runs (the periods are computed on the way).
    const char* configure(const StepProfile& p, std::uint32_t tick_hz, std::uint32_t max_ticks) noexcept {
        if (p.start_hz == 0 || p.cruise_hz < p.start_hz) return "0 < start rate <= cruise rate";
        if (p.accel == 0) return "acceleration 0";
        if (p.steepness < 2 || p.steepness > 30) return "steepness 2..30";
        if (tick_hz / p.cruise_hz < 10) return "cruise rate too high for the step timer (fewer than 10 ticks per step)";
        if ((tick_hz + p.start_hz / 2) / p.start_hz > max_ticks) return "start rate too low for the step timer (the first step does not fit)";
        profile_ = p;
        tick_hz_ = tick_hz;
        const double v0 = p.start_hz, vc = p.cruise_hz;
        ramp_ = static_cast<std::uint32_t>(std::ceil((vc * vc - v0 * v0) / (2.0 * p.accel)));
        const float k = static_cast<float>(p.steepness);
        sig_lo_ = sigmoid(-k / 2);
        sig_span_ = sigmoid(k / 2) - sig_lo_;
        cruise_ticks_ = ticks(static_cast<float>(p.cruise_hz));
        return nullptr;
    }

    /// Plan a move of `steps` (sign = direction): the ticks until the first step, 0: nothing to do.
    std::uint32_t begin(std::int32_t steps) noexcept {
        if (tick_hz_ == 0) {  // never configured: no move
            total_ = 0;
            return 0;
        }
        total_ = static_cast<std::uint32_t>(steps >= 0 ? steps : -steps);
        ramp_steps_ = total_ / 2 < ramp_ ? total_ / 2 : ramp_;
        return total_ == 0 ? 0 : period(0);
    }

    /// Ticks of step k (0 .. total() - 1) of the planned move.
    [[nodiscard]] std::uint32_t period(std::uint32_t k) const noexcept {
        if (k < ramp_steps_) return ramp_period(k);                          // accelerate
        if (k >= total_ - ramp_steps_) return ramp_period(total_ - 1 - k);   // decelerate
        // cruise - or, in a triangle with an odd step count, the single middle step
        return ramp_steps_ < ramp_ ? ramp_period(ramp_steps_) : cruise_ticks_;
    }

    /// The step rate at ramp step s (0: the start rate; from the ramp length on: the cruise rate).
    [[nodiscard]] float rate(std::uint32_t s) const noexcept {
        const float v0 = static_cast<float>(profile_.start_hz), vc = static_cast<float>(profile_.cruise_hz);
        if (s >= ramp_) return vc;
        if (profile_.shape == RampShape::Linear) return std::sqrt(v0 * v0 + 2.0f * static_cast<float>(profile_.accel) * static_cast<float>(s));
        const float k = static_cast<float>(profile_.steepness);
        const float y = (sigmoid(k * static_cast<float>(s) / static_cast<float>(ramp_) - k / 2) - sig_lo_) / sig_span_;
        return v0 + (vc - v0) * y;
    }

    [[nodiscard]] std::uint32_t total() const noexcept { return total_; }
    /// Steps of a full ramp (start to cruise rate) with this profile.
    [[nodiscard]] std::uint32_t ramp_steps() const noexcept { return ramp_; }
    [[nodiscard]] const StepProfile& profile() const noexcept { return profile_; }

private:
    static float sigmoid(float x) noexcept { return 1.0f / (1.0f + std::exp(-x)); }
    [[nodiscard]] std::uint32_t ticks(float hz) const noexcept { return static_cast<std::uint32_t>(static_cast<float>(tick_hz_) / hz + 0.5f); }
    [[nodiscard]] std::uint32_t ramp_period(std::uint32_t s) const noexcept { return ticks(rate(s)); }

    // Every member starts at 0, "not configured" (begin() plans nothing until configure()): a
    // ProfileStepper inside a TimerStepper lives in a DMA section, which is zeroed at start-up but
    // not loaded - a start value other than 0 would never arrive (tools/check_noload.py).
    StepProfile profile_{.start_hz = 0, .cruise_hz = 0, .accel = 0, .shape = RampShape::Linear, .steepness = 0};
    std::uint32_t tick_hz_ = 0;
    std::uint32_t ramp_ = 0, ramp_steps_ = 0, total_ = 0, cruise_ticks_ = 0;
    float sig_lo_ = 0, sig_span_ = 0;
};

}  // namespace rtos::motion
