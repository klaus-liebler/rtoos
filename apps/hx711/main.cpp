// SPDX-License-Identifier: MIT
//
// HX711 load cell on the Factory Control Unit: the driver (rtos/devices/hx711.hpp) on SPI2,
// PD_SCK on SPI2_MOSI (PC3), DOUT on SPI2_MISO (PC2). emulation/renode/fcu.repl puts the HX711
// model (HX711.cs, 10 Hz, channel A = -2775642) behind the SPI model (STM32H5_SPI.cs), which
// shifts every bit in virtual time.
//
// The driver selects channel A gain 64. The chip starts at gain 128, so the first reading is the
// model's gain-128 code, every later one half of it - and a chip that was powered down and reset
// in between shows up as a gain-128 reading again.
//
//   1. quiet: readings without interference must all be the model's value
//   2. preempted: before each reading a one-shot TIM2 interrupt is armed that fires a few
//      CPU cycles into the transfer (the delay sweeps 1 .. 1500 cycles, the interrupt busy-waits
//      100 us). The HX711 takes PD_SCK high for more than 60 us as power-down, so a transfer
//      that pauses with MOSI high loses the reading. Every reading must still be right.
//   3. quiet again: the chip must not have been reset (gain and data path unchanged)
//
// Regression test for a combination of causes: (a) PD_SCK hangs on SPI2_MOSI, (b) the SPI
// driver started the transfer (CSTART) before it had filled the TxFIFO, and wrote the frames
// one by one in a polling loop, (c) the master suspends SCK when its TxFIFO runs empty and MOSI
// keeps the last bit, and every 0x55 clock byte ends with a 1 - so an interrupt of more than
// about 60 us between two TXDR writes powered the HX711 down in the middle of a reading.
// The fix fills the TxFIFO before CSTART: the 8 frames then run in hardware alone.
//
// Ends with one line:
//     HX711 RESULT: PASS (<readings> readings, <hits> interrupts during a transfer)
//     HX711 RESULT: FAIL <reason>
#include <array>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/devices/hx711.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/gpio.hpp"
#include "rtos/port/stm32h5/spi.hpp"
#include "rtos/port/stm32h5/timer.hpp"
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

constexpr gpio::Pin dout_pin{gpio::GpioPort::C, 2};
constexpr gpio::Pin clock_pin{gpio::GpioPort::C, 3};
constexpr std::array pins{gpio::alternate(dout_pin, gpio::Signal::SPI2_MISO, gpio::Speed::Low),
                          gpio::alternate(clock_pin, gpio::Signal::SPI2_MOSI, gpio::Speed::Low)};

// As the FCU firmware: 100 MHz / 128 = 781 kHz
using Spi2 = h5::SpiMaster<h5::SpiId::Spi2, {.kernel_hz = board::pll1_q_hz, .max_bus_hz = 1'000'000,
                                             .cpu_hz = board::sysclk_hz}>;
bool data_ready() noexcept { return !gpio::read(dout_pin); }
using Scale = dev::Hx711<Spi2, &data_ready, dev::Hx711Gain::A64, 1>;

constexpr std::int32_t expected_a128 = -2'775'642;  // emulation/renode/fcu.repl: hx711 channelA
constexpr std::int32_t expected = expected_a128 / 2;  // gain 64

// One-shot interrupt: TIM2 counts CPU cycles (APB1 timer clock = SYSCLK = 250 MHz)
using Burst = h5::Timer<h5::TimerId::Tim2, {.kernel_hz = board::clocks.timer1_hz(),
                                            .tick_hz = board::clocks.timer1_hz(), .preload = false}>;
static_assert(Burst::tick_hz == board::sysclk_hz);
constexpr std::uint32_t burst_cycles = board::sysclk_hz / 10'000;  // 100 us: longer than 60 us

using Spi2Cr1 = h5::PReg<0x0000'3800>;  // SPI2_CR1: SPE (bit 0) shows a transfer in progress
volatile std::uint32_t bursts = 0, bursts_in_transfer = 0;

Kernel::Thread main_tcb;
alignas(8) std::array<std::byte, 2048> main_stack;

[[noreturn]] void finish(const char* failure, std::int32_t value = 0, std::uint32_t delay = 0) {
    if (failure == nullptr) {
        board::print("HX711 RESULT: PASS (");
        board::print_dec(bursts);
        board::print(" readings, ");
        board::print_dec(bursts_in_transfer);
        board::print(" interrupts during a transfer)\n");
    } else {
        board::print("HX711 RESULT: FAIL ");
        board::print(failure);
        board::print(" (value ");
        board::print_hex(static_cast<std::uint32_t>(value));
        board::print(", interrupt delay ");
        board::print_dec(delay);
        board::print(" cycles)\n");
    }
    board::flush();
    for (;;) {
        kernel.sleep(1000);
    }
}

/// Waits for DOUT low (at most 1 s: 10 Hz plus the 400 ms settling time) and reads.
/// A reading of `expected_a128` after the first one means the chip was reset in between.
/// `delay` != 0: arms the interrupt that many cycles before the reading starts.
std::int32_t read_one(std::uint32_t delay) {
    for (int ms = 0; !data_ready(); ++ms) {
        if (ms > 1000) {
            finish("no data from the HX711 within 1 s", 0, delay);
        }
        kernel.sleep(1);
    }
    if (delay != 0) {
        Burst::set_counter(0);
        Burst::set_period(delay);
        Burst::start();
    }
    const auto v = Scale::poll();
    if (!v) {
        finish("SPI transfer failed", 0, delay);
    }
    return *v;
}

void quiet(const char* failure) {
    for (int i = 0; i < 5; ++i) {
        if (const std::int32_t v = read_one(0); v != expected) {
            finish(failure, v);
        }
    }
}

void main_thread(void*) {
    gpio::configure<pins>();
    Spi2::init();
    if (!Scale::init()) {
        finish("SPI init transfer failed");
    }
    board::print("hx711: SPI2 at ");
    board::print_dec(Spi2::bus_hz);
    board::print(" Hz, expecting ");
    board::print_hex(static_cast<std::uint32_t>(expected));
    board::print("\n");

    if (const std::int32_t v = read_one(0); v != expected_a128) {
        finish("first reading (gain 128) wrong", v);
    }
    quiet("wrong reading without interrupts");
    board::print("hx711: quiet readings ok\n");

    for (std::uint32_t delay = 1; delay <= 1500; delay += 23) {
        if (const std::int32_t v = read_one(delay); v != expected) {
            finish("wrong reading with an interrupt during the transfer", v, delay);
        }
    }
    board::print("hx711: preempted readings ok\n");

    quiet("wrong reading after the preempted ones");
    finish(nullptr);
}

}  // namespace

extern "C" void TIM2_IRQHandler() noexcept {
    if (!Burst::take_update()) {
        return;
    }
    Burst::stop();
    bursts = bursts + 1;
    if ((Spi2Cr1::read() & 1u) != 0) {
        bursts_in_transfer = bursts_in_transfer + 1;
    }
    const std::uint32_t start = cm33::CycleCounter::now();
    while (cm33::CycleCounter::now() - start < burst_cycles) {
    }
}

int main() {
    board::init();
    board::print("\nrtoos hx711 - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);
    cm33::CycleCounter::enable();  // SPI timeouts and the interrupt's busy wait
    Burst::init();
    Burst::enable_update_irq();
    // above every kernel-aware priority: nothing in the thread can hold it off
    cm33::nvic::ipr(Burst::irq) = 1u << (8 - RTOS_CM33_PRIO_BITS);
    cm33::nvic::enable(Burst::irq);
    kernel.create_thread(main_tcb, {.name = "main", .entry = &main_thread, .stack = main_stack, .priority = 10});
    kernel.start();
}
