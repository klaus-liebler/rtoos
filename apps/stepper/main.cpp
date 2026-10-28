// SPDX-License-Identifier: MIT
//
// Stepper firmware for the Factory Control Unit: sigmoid moves on one axis.
//
//   STEP  PB4 (the FCU's elevator axis), toggled in the TIM16 update interrupt - one edge
//         per step
//   DIR   PD7
//
// Two moves: 2000 steps forward (sigmoid ramp up, cruise, ramp down), then 101 steps back
// (too short for the full ramp: a symmetric triangle). The console reports the ramp and the
// moves; emulation/stepper_test.py records the STEP edges in Renode and checks that the step
// rate rises and falls along the sigmoid.
#include <array>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/motion/sigmoid_stepper.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/gpio.hpp"
#include "rtos/port/stm32h5/timer.hpp"
#include "rtos/rtos.hpp"

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 2;
    static constexpr std::size_t notify_slots = 1;
};
using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, Config>;

}  // namespace

Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

namespace h5 = rtos::port::stm32h5;
namespace gpio = h5::gpio;
namespace cm33 = rtos::port::cm33;

constexpr gpio::Pin step_pin{gpio::GpioPort::B, 4};
constexpr gpio::Pin dir_pin{gpio::GpioPort::D, 7};
constexpr std::array pins{gpio::output(step_pin), gpio::output(dir_pin)};

// 1 MHz ticks: 10000 ticks per step at 100 steps/s still fit the 16-bit timer
constexpr rtos::motion::SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000, .ramp_us = 500'000};
using StepTimer =
    h5::Timer<h5::TimerId::Tim16, {.kernel_hz = board::clocks.timer2_hz(), .tick_hz = ramp.tick_hz, .preload = false}>;
using Motion = rtos::motion::SigmoidStepper<ramp>;
static_assert(Motion::max_ticks <= StepTimer::max_period + 1, "the slowest step does not fit the timer");

Motion motion;
Kernel::Thread main_tcb;
alignas(8) std::array<std::byte, 2048> main_stack;

/// Start a move; returns at once (the interrupt does the steps).
void move(std::int32_t steps) noexcept {
    const std::uint32_t first = motion.begin(steps);
    if (first == 0) {
        return;
    }
    gpio::write(dir_pin, motion.forward());
    StepTimer::set_period(first);
    StepTimer::set_counter(0);
    StepTimer::start();
}

void print_move(const char* what, std::int32_t steps) {
    board::print(what);
    board::print(steps < 0 ? "-" : "");
    board::print_dec(static_cast<std::uint32_t>(steps < 0 ? -steps : steps));
    board::print(" steps\n");
}

void main_thread(void*) {
    board::print("stepper: tick_hz ");
    board::print_dec(ramp.tick_hz);
    board::print(" start_hz ");
    board::print_dec(ramp.start_hz);
    board::print(" cruise_hz ");
    board::print_dec(ramp.cruise_hz);
    board::print(" ramp_us ");
    board::print_dec(ramp.ramp_us);
    board::print(" steepness ");
    board::print_dec(ramp.steepness);
    board::print(" ramp_steps ");
    board::print_dec(static_cast<std::uint32_t>(Motion::ramp.size()));
    board::print("\n");

    for (const std::int32_t steps : {2000, -101}) {
        kernel.sleep(200);  // a gap between the moves (the test splits the edges here)
        print_move("stepper: move ", steps);
        move(steps);
        (void)kernel.notify_wait(0, rtos::wait_forever);
        board::print("stepper: done at ");
        board::print_dec(static_cast<std::uint32_t>(motion.position() + 1'000'000));
        board::print(" (position + 1000000)\n");
    }
    board::print("STEPPER RESULT: DONE\n");
    for (;;) {
        kernel.sleep(1000);
    }
}

}  // namespace

extern "C" void TIM16_IRQHandler() noexcept {
    if (!StepTimer::take_update()) {
        return;
    }
    gpio::toggle(step_pin);
    if (const std::uint32_t ticks = motion.next(); ticks != 0) {
        StepTimer::set_period(ticks);  // ARR without preload: the current period
    } else {
        StepTimer::stop();
        kernel.notify_from_isr(main_tcb, 0, 1, rtos::NotifyAction::SetBits);
    }
}

int main() {
    board::init();
    board::print("\nrtoos stepper - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);
    gpio::configure<pins>();
    StepTimer::init();
    StepTimer::enable_update_irq();
    cm33::nvic::ipr(StepTimer::irq) = 6u << (8 - RTOS_CM33_PRIO_BITS);  // kernel-aware
    cm33::nvic::enable(StepTimer::irq);
    kernel.create_thread(main_tcb, {.name = "main", .entry = &main_thread, .stack = main_stack, .priority = 10});
    kernel.start();
}
