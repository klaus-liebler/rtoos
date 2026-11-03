// SPDX-License-Identifier: MIT
//
// Stepper firmware for the Factory Control Unit: sigmoid moves on one axis, generated in
// hardware (rtos/motion/timer_stepper.hpp).
//
//   STEP  PB4 = TIM3_CH1 (the FCU's elevator axis): the timer toggles it, one edge per step
//         (TMC2209 DEDGE); GPDMA1 channel 4 writes each step's period into TIM3_ARR at the
//         update events, the CPU refills a buffer every 32 steps
//   DIR   PD7
//
// Moves: 2000 steps forward (sigmoid ramp up, cruise, ramp down), 101 back (too short for the
// full ramp: a symmetric triangle), then 1 and 2 steps. All the time an interrupt load runs
// above the stepper's interrupts: TIM4 every 700 us, 200 us busy. Edges that came from an
// interrupt would move with it; these must not. For each move the console reports the planned
// time from the first to the last edge:
//     stepper: move <steps> steps, edges span <ticks> ticks
// emulation/stepper_test.py records the STEP edges in Renode and checks the ramp, the counts
// and that every edge sits exactly on its timer tick.
#include <array>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/motion/timer_stepper.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/cache.hpp"
#include "rtos/port/stm32h5/gpdma.hpp"
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
constexpr std::array pins{gpio::alternate(step_pin, gpio::Signal::TIM3_CH1, gpio::Speed::Medium), gpio::output(dir_pin)};

// 1 MHz ticks: 10000 ticks per step at 100 steps/s fit the 16-bit timer
constexpr rtos::motion::SigmoidRamp ramp{.tick_hz = 1'000'000, .start_hz = 100, .cruise_hz = 1000, .ramp_us = 500'000};
using StepTimer = h5::Timer<h5::TimerId::Tim3, {.kernel_hz = board::clocks.timer1_hz(), .tick_hz = ramp.tick_hz}>;
using StepDma = h5::DmaChannel<{.channel = 4, .request = h5::DmaRequest::Tim3Up,
                                .direction = h5::DmaDirection::MemoryToPeripheral,
                                .memory_bytes = 2, .peripheral_bytes = 4, .priority = 3}>;
RTOS_DMA_BUFFER rtos::motion::TimerStepper<StepTimer, 1, StepDma, ramp> stepper;

// interrupt load
using Load = h5::Timer<h5::TimerId::Tim4, {.kernel_hz = board::clocks.timer1_hz(), .tick_hz = 1'000'000, .preload = false}>;
constexpr std::uint32_t load_period_us = 700, load_busy_cycles = board::sysclk_hz / 1'000'000 * 200;

Kernel::Thread main_tcb;
alignas(8) std::array<std::byte, 2048> main_stack;

void move(std::int32_t steps) {
    gpio::write(dir_pin, steps > 0);
    if (!stepper.start(steps)) {
        board::print("stepper: start failed\n");
        return;
    }
    std::uint32_t span = 0;
    for (std::uint32_t k = 1; k < stepper.plan().total(); ++k) {
        span += stepper.plan().period(k);
    }
    board::print("stepper: move ");
    board::print(steps < 0 ? "-" : "");
    board::print_dec(static_cast<std::uint32_t>(steps < 0 ? -steps : steps));
    board::print(" steps, edges span ");
    board::print_dec(span);
    board::print(" ticks\n");
    (void)kernel.notify_wait(0, rtos::wait_forever);
    board::print("stepper: done at ");
    board::print_dec(static_cast<std::uint32_t>(stepper.position() + 1'000'000));
    board::print(" (position + 1000000)\n");
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
    board::print_dec(static_cast<std::uint32_t>(decltype(stepper)::Motion::ramp.size()));
    board::print("\n");

    Load::set_period(load_period_us);
    Load::start();
    for (const std::int32_t steps : {2000, -101, 1, 2}) {
        kernel.sleep(200);  // a gap between the moves (the test splits the edges here)
        move(steps);
    }
    Load::stop();
    board::print("STEPPER RESULT: DONE\n");
    for (;;) {
        kernel.sleep(1000);
    }
}

void enable_irq(unsigned irq, unsigned priority) {
    cm33::nvic::ipr(irq) = static_cast<std::uint8_t>(priority << (8 - RTOS_CM33_PRIO_BITS));
    cm33::nvic::enable(irq);
}

}  // namespace

extern "C" void GPDMA1_CH4_IRQHandler() noexcept { stepper.on_dma_irq(); }

extern "C" void TIM3_IRQHandler() noexcept {
    if (stepper.on_timer_irq()) {
        kernel.notify_from_isr(main_tcb, 0, 1, rtos::NotifyAction::SetBits);
    }
}

extern "C" void TIM4_IRQHandler() noexcept {
    if (!Load::take_update()) {
        return;
    }
    const std::uint32_t start = cm33::CycleCounter::now();
    while (cm33::CycleCounter::now() - start < load_busy_cycles) {
    }
}

int main() {
    board::init();
    board::print("\nrtoos stepper - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);
    cm33::CycleCounter::enable();
    gpio::configure<pins>();
    stepper.init();
    Load::init();
    Load::enable_update_irq();
    enable_irq(Load::irq, 1);         // the load: above everything else, holds the stepper's ISRs off
    enable_irq(StepDma::irq, 6);      // kernel-aware (the timer ISR notifies)
    enable_irq(StepTimer::irq, 6);
    kernel.create_thread(main_tcb, {.name = "main", .entry = &main_thread, .stack = main_stack, .priority = 10});
    kernel.start();
}
