// SPDX-License-Identifier: MIT
//
// Stepper motion with a sigmoid speed ramp, computed by the compiler.
//
// The step rate follows a logistic curve over time - gentle at both ends of the ramp, the
// strongest acceleration in its middle - from start_hz (the rate a motor can start and stop
// at without a ramp) to cruise_hz within ramp_us:
//
//     v(t) = start + (cruise - start) * (s(k*t/T - k/2) - s(-k/2)) / (s(k/2) - s(-k/2)),
//     s(x) = 1 / (1 + e^-x), k = steepness
//
// SigmoidRamp is a template argument: the period of every step of the ramp (in timer ticks)
// is evaluated at compile time into a table - no generator script, no approximation, any
// timer clock. At run time the motion only reads the table: one array access per step.
//
//   constexpr SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000, .ramp_us = 500'000};
//   SigmoidStepper<ramp> motion;
//   Timer::set_period(motion.begin(2000));       // first period; start the timer
//   // update interrupt = one step edge:
//   if (Timer::take_update()) { gpio::toggle(step); if (auto p = motion.next()) Timer::set_period(p); else Timer::stop(); }
//
// A move of n steps accelerates over min(ramp length, n/2) steps, cruises, and decelerates
// through the same periods in reverse: short moves get a symmetric triangle. Each call of
// next() is one step (one edge on the STEP pin; a driver counting both edges - TMC2209 DEDGE -
// moves one microstep per edge).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

namespace rtos::motion {

struct SigmoidRamp {
    std::uint32_t tick_hz;          ///< counter frequency of the step timer
    std::uint32_t start_hz;         ///< step rate at standstill (start and stop)
    std::uint32_t cruise_hz;        ///< step rate after the ramp
    std::uint32_t ramp_us;          ///< duration of the ramp
    std::uint32_t steepness = 12;   ///< k: the sigmoid's x range -k/2 .. k/2
};

namespace detail {

/// e^x for constant evaluation (|x| up to ~20): halve until small, Taylor, square back.
constexpr double exp(double x) noexcept {
    int halvings = 0;
    while (x > 0.5 || x < -0.5) {
        x /= 2;
        ++halvings;
    }
    double term = 1, sum = 1;
    for (int n = 1; n < 20; ++n) {
        term *= x / n;
        sum += term;
    }
    while (halvings-- > 0) {
        sum *= sum;
    }
    return sum;
}

constexpr double sigmoid(double x) noexcept { return 1.0 / (1.0 + exp(-x)); }

}  // namespace detail

/// Step rate (steps/s) of the ramp `r` at time `t_us` after its start.
[[nodiscard]] constexpr double sigmoid_rate(const SigmoidRamp& r, double t_us) noexcept {
    const double k = r.steepness;
    const double x = (t_us >= r.ramp_us ? 1.0 : t_us / r.ramp_us) * k - k / 2;
    const double lo = detail::sigmoid(-k / 2), hi = detail::sigmoid(k / 2);
    const double y = (detail::sigmoid(x) - lo) / (hi - lo);
    return r.start_hz + (static_cast<double>(r.cruise_hz) - r.start_hz) * y;
}

/// Number of steps the ramp takes (until its time is used up).
[[nodiscard]] constexpr std::size_t sigmoid_ramp_steps(const SigmoidRamp& r) noexcept {
    double t_us = 0;
    std::size_t n = 0;
    while (t_us < r.ramp_us) {
        t_us += 1e6 / sigmoid_rate(r, t_us);
        ++n;
    }
    return n;
}

/// Ticks of one step at the step rate `hz` (rounded).
[[nodiscard]] constexpr std::uint32_t step_ticks(std::uint32_t tick_hz, double hz) noexcept {
    return static_cast<std::uint32_t>(tick_hz / hz + 0.5);
}

/// The period of every ramp step in timer ticks: step i lasts table[i], its rate is the
/// sigmoid at the time the step starts.
template <SigmoidRamp R>
inline constexpr auto sigmoid_ramp_table = [] {
    static_assert(R.tick_hz > 0 && R.start_hz > 0 && R.cruise_hz > R.start_hz && R.ramp_us > 0,
                  "ramp needs tick_hz > 0 and 0 < start_hz < cruise_hz");
    static_assert(R.steepness >= 2 && R.steepness <= 30, "steepness 2..30");
    static_assert(R.tick_hz / R.cruise_hz >= 10, "fewer than 10 ticks per step at cruise: too coarse");
    std::array<std::uint32_t, sigmoid_ramp_steps(R)> t{};
    double t_us = 0;
    for (std::size_t i = 0; i < t.size(); ++i) {
        const double rate = sigmoid_rate(R, t_us);
        t[i] = step_ticks(R.tick_hz, rate);
        t_us += t[i] * 1e6 / R.tick_hz;  // the time the table really takes
    }
    return t;
}();

template <SigmoidRamp R>
class SigmoidStepper {
public:
    static constexpr const auto& ramp = sigmoid_ramp_table<R>;
    static constexpr std::uint32_t cruise_ticks = step_ticks(R.tick_hz, R.cruise_hz);
    /// Longest step (the first): the timer must reach it.
    static constexpr std::uint32_t max_ticks = ramp[0];

    /// Plan a move of `steps` (sign = direction) and return the ticks until the first step;
    /// 0: nothing to do.
    std::uint32_t begin(std::int32_t steps) noexcept {
        forward_ = steps >= 0;
        total_ = static_cast<std::uint32_t>(forward_ ? steps : -steps);
        done_ = 0;
        ramp_steps_ = total_ / 2 < ramp.size() ? total_ / 2 : static_cast<std::uint32_t>(ramp.size());
        return total_ == 0 ? 0 : period(0);
    }

    /// One step has happened (call at the step edge). Returns the ticks until the next one,
    /// or 0 when the move is complete.
    std::uint32_t next() noexcept {
        if (done_ >= total_) {
            return 0;
        }
        ++done_;
        position_ += forward_ ? 1 : -1;
        return done_ < total_ ? period(done_) : 0;
    }

    [[nodiscard]] bool moving() const noexcept { return done_ < total_; }
    [[nodiscard]] bool forward() const noexcept { return forward_; }
    [[nodiscard]] std::int32_t position() const noexcept { return position_; }
    void set_position(std::int32_t p) noexcept { position_ = p; }
    /// Steps left in the current move.
    [[nodiscard]] std::uint32_t remaining() const noexcept { return total_ - done_; }

private:
    [[nodiscard]] std::uint32_t period(std::uint32_t k) const noexcept {
        if (k < ramp_steps_) {
            return ramp[k];  // accelerate
        }
        if (k >= total_ - ramp_steps_) {
            return ramp[total_ - 1 - k];  // decelerate: the ramp in reverse
        }
        // between them: cruise - or, in a triangle with an odd step count, the single middle
        // step, which continues the ramp by one step
        return ramp_steps_ < ramp.size() ? ramp[ramp_steps_] : cruise_ticks;
    }

    std::uint32_t total_ = 0;
    std::uint32_t done_ = 0;
    std::uint32_t ramp_steps_ = 0;
    std::int32_t position_ = 0;
    bool forward_ = true;
};

}  // namespace rtos::motion
