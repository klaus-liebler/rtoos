// SPDX-License-Identifier: MIT
//
// GPIO edge interrupts (EXTI) on the Factory Control Unit, in Renode (STM32H5_EXTI.cs):
//
//   1. light barrier 1 (PC7, NPN: low while interrupted): the platform passes an object every
//      7 ms for 1 ms (PulseGenerator.cs). Both edges by EXTI over one second: every pulse must
//      be counted (142 or 143), while sampling the pin every 50 ms - the I/O cycle - sees only
//      a few of them.
//   2. INA226 conversion ready (I2C4 interrupt-driven, ALERT on PC9, falling edge): the thread
//      sleeps until the ALERT interrupt, reads the measurement and acknowledges (MASK/ENABLE).
//      One conversion every 4 x (1.1 + 1.1) ms = 8.8 ms: 112 to 115 in one second, every one
//      24000 mV. Without acknowledging, ALERT stays low: one interrupt only.
//
// Ends with one line:
//     EXTI RESULT: PASS (<edges> barrier edges, <sampled> seen by sampling, <conversions> conversions)
//     EXTI RESULT: FAIL <step>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/exti.hpp"
#include "rtos/port/stm32h5/gpio.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
#include "rtos/rtos.hpp"

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 1;
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
namespace dev = rtos::devices;

constexpr gpio::Pin barrier_pin{gpio::GpioPort::C, 7};
constexpr gpio::Pin alert_pin{gpio::GpioPort::C, 9};
constexpr std::array pins{
    gpio::input(barrier_pin), gpio::input(alert_pin, gpio::Pull::Up),
    gpio::alternate({gpio::GpioPort::D, 12}, gpio::Signal::I2C4_SCL, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate({gpio::GpioPort::D, 13}, gpio::Signal::I2C4_SDA, gpio::Speed::Low, gpio::Pull::None, true)};

using Barrier = h5::ExtiLine<barrier_pin, h5::ExtiEdge::Both>;
using Alert = h5::ExtiLine<alert_pin, h5::ExtiEdge::Falling>;
std::atomic<std::uint32_t> barrier_falling{0}, barrier_rising{0}, alerts{0};

rtos::Semaphore<Kernel> i2c_done{kernel, 0, 1};
rtos::Semaphore<Kernel> conversion{kernel, 0, 100};
struct I2cWait {
    static void reset() noexcept {
        while (i2c_done.take(rtos::no_wait)) {
        }
    }
    static bool wait(std::uint32_t timeout_us) noexcept { return i2c_done.take(timeout_us / 1000 + 2); }
    static void signal() noexcept { (void)i2c_done.give_from_isr(); }
};
using Bus = h5::I2cMaster<h5::I2cId::I2c4, {.kernel_hz = board::clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz}, I2cWait>;
using Supply = dev::Ina226<Bus, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;

Kernel::Thread main_tcb;
alignas(8) std::array<std::byte, 2048> main_stack;

[[noreturn]] void finish(const char* failure, std::uint32_t a = 0, std::uint32_t b = 0, std::uint32_t c = 0) {
    board::print(failure == nullptr ? "EXTI RESULT: PASS (" : "EXTI RESULT: FAIL ");
    if (failure != nullptr) {
        board::print(failure);
        board::print(" (");
    }
    board::print_dec(a);
    board::print(" barrier edges, ");
    board::print_dec(b);
    board::print(" seen by sampling, ");
    board::print_dec(c);
    board::print(" conversions)\n");
    board::flush();
    for (;;) {
        kernel.sleep(1000);
    }
}

void main_thread(void*) {
    gpio::configure<pins>();

    // 1. light barrier
    Barrier::init();
    Barrier::enable();
    std::uint32_t sampled = 0;
    const std::uint32_t f0 = barrier_falling.load(), r0 = barrier_rising.load();
    for (int i = 0; i < 20; ++i) {  // 20 I/O cycles of 50 ms
        sampled += gpio::read(barrier_pin) ? 0 : 1;
        kernel.sleep(50);
    }
    Barrier::disable();
    const std::uint32_t falling = barrier_falling.load() - f0, rising = barrier_rising.load() - r0;
    board::print("exti: barrier interrupted ");
    board::print_dec(falling);
    board::print(" times (edges ");
    board::print_dec(falling + rising);
    board::print("), sampling every 50 ms saw ");
    board::print_dec(sampled);
    board::print("\n");
    if (falling < 142 || falling > 143 || rising + 1 < falling || rising > falling + 1 || sampled > 10) {
        finish("light barrier pulses", falling + rising, sampled);
    }

    // 2. INA226 conversion ready
    Bus::init();
    if (!Supply::probe() || !Supply::init() || !Supply::enable_conversion_ready_alert()) {
        finish("INA226 setup", falling + rising, sampled);
    }
    Alert::init();
    Alert::enable();
    std::uint32_t conversions = 0;
    const rtos::Tick end = kernel.now() + 1000;
    while (static_cast<std::int32_t>(end - kernel.now()) > 0) {
        if (!conversion.take(100)) {
            finish("no ALERT within 100 ms", falling + rising, sampled, conversions);
        }
        const auto m = Supply::read();
        const auto ready = Supply::acknowledge();
        if (!m || m->bus_mv != 24'000 || ready != true) {
            finish("measurement after ALERT", falling + rising, sampled, conversions);
        }
        ++conversions;
    }
    // without acknowledge: ALERT stays low, no further edge
    const std::uint32_t before = alerts.load();
    kernel.sleep(150);
    const std::uint32_t unacknowledged = alerts.load() - before;
    Alert::disable();
    board::print("exti: ");
    board::print_dec(conversions);
    board::print(" conversions by ALERT in 1 s, ");
    board::print_dec(unacknowledged);
    board::print(" ALERT edge(s) in 150 ms without acknowledge\n");
    if (conversions < 112 || conversions > 115 || unacknowledged > 1) {
        finish("conversion-ready ALERT", falling + rising, sampled, conversions);
    }
    finish(nullptr, falling + rising, sampled, conversions);
}

}  // namespace

extern "C" void EXTI7_IRQHandler() noexcept {
    const auto e = Barrier::take();
    if (e.falling) {
        barrier_falling.fetch_add(1);
    }
    if (e.rising) {
        barrier_rising.fetch_add(1);
    }
}

extern "C" void EXTI9_IRQHandler() noexcept {
    if (Alert::take()) {
        alerts.fetch_add(1);
        (void)conversion.give_from_isr();
    }
}

extern "C" void I2C4_EV_IRQHandler() noexcept { Bus::on_event_irq(); }
extern "C" void I2C4_ER_IRQHandler() noexcept { Bus::on_error_irq(); }

int main() {
    board::init();
    board::print("\nrtoos exti - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);
    cm33::CycleCounter::enable();
    for (const unsigned irq : {unsigned{Barrier::irq}, unsigned{Alert::irq}, unsigned{Bus::irq_event}, unsigned{Bus::irq_error}}) {
        cm33::nvic::ipr(irq) = 6u << (8 - RTOS_CM33_PRIO_BITS);
        cm33::nvic::enable(irq);
    }
    kernel.create_thread(main_tcb, {.name = "main", .entry = &main_thread, .stack = main_stack, .priority = 10});
    kernel.start();
}
