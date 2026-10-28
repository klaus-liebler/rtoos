# Motion: sigmoid stepper ramps

[include/rtos/motion/sigmoid_stepper.hpp](../include/rtos/motion/sigmoid_stepper.hpp) drives stepper motors with a sigmoid speed ramp. The step rate follows a logistic curve over time: gentle at both ends, with the strongest acceleration in the middle of the ramp. Neither the motor nor the mechanics see a jerk.

```
v(t) = start + (cruise - start) * (s(k·t/T - k/2) - s(-k/2)) / (s(k/2) - s(-k/2)),   s(x) = 1 / (1 + e^-x)
```

## Computed by the compiler

The ramp is a template argument. The compiler evaluates the period of every ramp step, in timer ticks, into a table:

```cpp
constexpr rtos::motion::SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000,
                                        .ramp_us = 500'000};          // steepness k = 12 by default
rtos::motion::SigmoidStepper<ramp> motion;                            // table: 274 steps, 1.1 KB flash
```

Each step lasts exactly as long as the sigmoid prescribes for the moment it starts, for any timer clock. There is no generator script and no piecewise-linear approximation. At run time each step costs one table access. The FCU firmware used to carry a Python-generated table for exactly one timer clock (160 MHz).

## Moves

A move of n steps accelerates over min(ramp length, n/2) steps, cruises, and decelerates through the same periods in reverse. A move too short for the full ramp becomes a symmetric triangle. With an odd step count, the single middle step continues the ramp.

```cpp
using StepTimer = h5::Timer<h5::TimerId::Tim16, {.kernel_hz = board::clocks.timer2_hz(),
                                                 .tick_hz = ramp.tick_hz, .preload = false}>;
static_assert(decltype(motion)::max_ticks <= StepTimer::max_period + 1);   // the slowest step fits

void move(std::int32_t steps) {
    gpio::write(dir_pin, steps >= 0);
    StepTimer::set_period(motion.begin(steps));
    StepTimer::set_counter(0);
    StepTimer::start();
}

extern "C" void TIM16_IRQHandler() noexcept {            // one update = one step edge
    if (!StepTimer::take_update()) return;
    gpio::toggle(step_pin);
    if (const auto ticks = motion.next()) StepTimer::set_period(ticks);
    else { StepTimer::stop(); /* notify the waiting thread */ }
}
```

Each call of `next()` is one edge on the STEP pin. A driver that steps on both edges (TMC2209 with DEDGE) moves one microstep per edge. Without ARR preload, the new period applies to the period that has just begun.

## Tests

- **Host** ([motion_test.cpp](../tests/motion_test.cpp)): the compile-time `exp`, ramp length and duration, every step against the sigmoid, the acceleration peak in the middle, long and short moves, direction and position.
- **Renode** ([stepper_test.py](../emulation/stepper_test.py), `ctest -R stepper_renode`): the firmware [`fcu_stepper`](../apps/stepper/main.cpp) runs two moves on TIM16. A probe on the STEP pin records every edge with its virtual time. The test computes the step rate as the reciprocal of each edge-to-edge interval and checks:
  - every ramp step against the sigmoid (within 2 %);
  - that the sigmoid fits far better than a linear ramp (RMS 0.9 against 138 steps/s);
  - the strongest acceleration near the middle of the ramp and gentle ends;
  - the cruise rate;
  - the mirrored deceleration;
  - the symmetric triangle of the short move.
