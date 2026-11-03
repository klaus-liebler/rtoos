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

A move of n steps accelerates over min(ramp length, n/2) steps, cruises, and decelerates through the same periods in reverse. A move too short for the full ramp becomes a symmetric triangle. With an odd step count, the single middle step continues the ramp. `period(k)` gives the ticks of step k of the planned move.

## Steps in hardware: TimerStepper

[include/rtos/motion/timer_stepper.hpp](../include/rtos/motion/timer_stepper.hpp) generates a move with a timer and a GPDMA channel. The CPU sets the move up and refills a buffer every 32 steps; no interrupt per step, so no interrupt latency in the edges:

- the STEP pin is the timer's channel output in toggle mode, compare value 1: one edge one tick after every update event. A driver that steps on both edges (TMC2209 with DEDGE, which `Tmc2209::init()` switches on) moves one microstep per edge;
- the GPDMA writes the next period into ARR at every update event (ARR preload: it applies from the next update);
- start: ARR = p0 active, ARR = p1 - 1 preloaded, CNT = 2 (past the compare value: no edge at the start) - edge 1 comes after p0 ticks, edge k + 1 one period p(k) after edge k;
- the DMA's transfer-complete interrupt of the last item switches on one-pulse mode; the next update event stops the counter after the last edge and raises the timer interrupt (the move is done).

```cpp
constexpr rtos::motion::SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000, .ramp_us = 500'000};
using StepTimer = h5::Timer<h5::TimerId::Tim3, {.kernel_hz = board::clocks.timer1_hz(), .tick_hz = ramp.tick_hz}>;
using StepDma = h5::DmaChannel<{.channel = 4, .request = h5::DmaRequest::Tim3Up,
                                .direction = h5::DmaDirection::MemoryToPeripheral,
                                .memory_bytes = 2, .peripheral_bytes = 4}>;
RTOS_DMA_BUFFER rtos::motion::TimerStepper<StepTimer, 1, StepDma, ramp> stepper;  // STEP = TIM3_CH1

stepper.init();
gpio::write(dir_pin, steps > 0);
stepper.start(steps);
extern "C" void GPDMA1_CH4_IRQHandler() noexcept { stepper.on_dma_irq(); }
extern "C" void TIM3_IRQHandler() noexcept { if (stepper.on_timer_irq()) { /* move done */ } }
```

Deadline: the DMA interrupt must start the next chunk, and switch on one-pulse mode, within one step period at cruise rate. `stop()` aborts a move and counts the steps done. Compile-time checks: ARR preload on, the timer's tick rate equal to the ramp's, the slowest step fits ARR and a DMA item.

## Tests

- **Host** ([motion_test.cpp](../tests/motion_test.cpp)): the compile-time `exp`, ramp length and duration, every step against the sigmoid, the acceleration peak in the middle, long and short moves, direction and position. [timer_stepper_test.cpp](../tests/timer_stepper_test.cpp): `TimerStepper` against a tick-exact simulation of the timer (preload, one-pulse mode, toggle output, update DMA request) and the GPDMA, with interrupt latencies up to 900 ticks: every edge at its planned tick, exactly N edges for 1, 2, 3, 31-33, 64, 65, 101 and 2000 steps, the counter stopped, abort.
- **Renode** ([stepper_test.py](../emulation/stepper_test.py), `ctest -R stepper_renode`): the firmware [`fcu_stepper`](../apps/stepper/main.cpp) runs four moves (2000, -101, 1, 2 steps) with `TimerStepper` on TIM3 and GPDMA1 (own models [STM32H5_TIM.cs](../emulation/renode/STM32H5_TIM.cs), [STM32H5_GPDMA.cs](../emulation/renode/STM32H5_GPDMA.cs)) while an interrupt load runs above the stepper's interrupts (200 us busy every 700 us). A probe on TIM3_CH1 records every edge with its virtual time. The test checks:
  - exactly as many edges as steps, every edge on a timer tick, each move exactly as long as planned;
  - every ramp step against the sigmoid (within 2 %), and that the sigmoid fits far better than a linear ramp;
  - the strongest acceleration near the middle of the ramp and gentle ends;
  - the cruise rate, the mirrored deceleration, the symmetric triangle of the short move.
