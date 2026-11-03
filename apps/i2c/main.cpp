// SPDX-License-Identifier: MIT
//
// I2C on the Factory Control Unit: the interrupt-driven I2cMaster against the INA226 on I2C4
// (Renode: STM32H5_I2C.cs with bus timing, INA226.cs at 0x40 with 24 V and 5 mV over the
// 10 mOhm shunt).
//
//   1. probe: the INA226 answers, an empty address NACKs (and the bus works after it)
//   2. the INA226 driver: ID, configuration, a measurement of 24000 mV, 5000 uV, about 500 mA
//   3. a register written and read back
//   4. the CPU while the bytes move: 20 register reads with the polled driver, then with the
//      interrupt-driven one, while a less urgent thread counts. Busy-waiting holds it off
//      completely; sleeping on the interrupt lets it run.
//
// Ends with one line:
//     I2C RESULT: PASS (<polled> / <irq> background counts during 20 reads)
//     I2C RESULT: FAIL <step>
#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/devices/ina226.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/gpio.hpp"
#include "rtos/port/stm32h5/i2c.hpp"
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
namespace dev = rtos::devices;

constexpr std::array pins{
    gpio::alternate({gpio::GpioPort::D, 12}, gpio::Signal::I2C4_SCL, gpio::Speed::Low, gpio::Pull::None, true),
    gpio::alternate({gpio::GpioPort::D, 13}, gpio::Signal::I2C4_SDA, gpio::Speed::Low, gpio::Pull::None, true)};

rtos::Semaphore<Kernel> i2c_done{kernel, 0, 1};
struct I2cWait {
    static void reset() noexcept {
        while (i2c_done.take(rtos::no_wait)) {
        }
    }
    static bool wait(std::uint32_t timeout_us) noexcept { return i2c_done.take(timeout_us / 1000 + 2); }
    static void signal() noexcept { (void)i2c_done.give_from_isr(); }
};

constexpr h5::I2cConfig bus_config{.kernel_hz = board::clocks.pclk3_hz(), .cpu_hz = board::sysclk_hz};
using Bus = h5::I2cMaster<h5::I2cId::I2c4, bus_config, I2cWait>;
using PolledBus = h5::I2cMaster<h5::I2cId::I2c4, bus_config>;
using Supply = dev::Ina226<Bus, {.shunt_micro_ohm = 10'000, .max_current_ma = 8'000}>;

std::atomic<std::uint32_t> background{0};

Kernel::Thread main_tcb, background_tcb;
alignas(8) std::array<std::byte, 2048> main_stack;
alignas(8) std::array<std::byte, 512> background_stack;

[[noreturn]] void finish(const char* failure, std::uint32_t polled = 0, std::uint32_t irq = 0) {
    if (failure == nullptr) {
        board::print("I2C RESULT: PASS (");
        board::print_dec(polled);
        board::print(" / ");
        board::print_dec(irq);
        board::print(" background counts during 20 reads)\n");
    } else {
        board::print("I2C RESULT: FAIL ");
        board::print(failure);
        board::print("\n");
    }
    board::flush();
    for (;;) {
        kernel.sleep(1000);
    }
}

template <class B>
std::uint32_t background_during_reads() {
    const std::uint32_t before = background.load();
    std::array<std::byte, 2> v{};
    for (int i = 0; i < 20; ++i) {
        if (B::read_register(0x40, 0xFE, v) != h5::I2cResult::Ok) {
            finish("register read during the background count");
        }
    }
    return background.load() - before;
}

void main_thread(void*) {
    gpio::configure<pins>();
    Bus::init();
    if (!Bus::probe(0x40)) {
        finish("no INA226 at 0x40");
    }
    if (Bus::probe(0x41)) {
        finish("a device answers at the empty address 0x41");
    }
    if (!Supply::probe() || Supply::ids() != 0x5449'2260u) {
        finish("INA226 IDs after the NACK");
    }
    if (!Supply::init()) {
        finish("INA226 init");
    }
    kernel.sleep(40);  // one conversion: 16 x (1.1 ms + 1.1 ms)
    const auto m = Supply::read();
    if (!m || m->bus_mv != 24'000 || m->shunt_uv != 5'000 || m->current_ma < 495 || m->current_ma > 505) {
        finish("INA226 measurement");
    }
    board::print("i2c: INA226 ");
    board::print_dec(static_cast<std::uint32_t>(m->bus_mv));
    board::print(" mV ");
    board::print_dec(static_cast<std::uint32_t>(m->current_ma));
    board::print(" mA\n");
    using Regs = dev::I2cRegisters<Bus, 0x40>;
    if (!Regs::write16(0x07, 0x1234) || Regs::read16(0x07) != 0x1234) {
        finish("alert limit register written and read back");
    }

    const std::uint32_t polled = background_during_reads<PolledBus>();
    const std::uint32_t irq = background_during_reads<Bus>();
    board::print("i2c: background counts: polled ");
    board::print_dec(polled);
    board::print(", interrupt-driven ");
    board::print_dec(irq);
    board::print("\n");
    if (polled != 0 || irq < 20) {
        finish("the thread must sleep while the bytes move (interrupt-driven), not busy-wait", polled, irq);
    }
    finish(nullptr, polled, irq);
}

void background_thread(void*) {
    for (;;) {
        background.fetch_add(1);
        for (int i = 0; i < 200; ++i) {
            __asm volatile("" ::: "memory");
        }
    }
}

}  // namespace

extern "C" void I2C4_EV_IRQHandler() noexcept { Bus::on_event_irq(); }
extern "C" void I2C4_ER_IRQHandler() noexcept { Bus::on_error_irq(); }

int main() {
    board::init();
    board::print("\nrtoos i2c - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);
    cm33::CycleCounter::enable();
    for (const unsigned irq : {unsigned{Bus::irq_event}, unsigned{Bus::irq_error}}) {
        cm33::nvic::ipr(irq) = 6u << (8 - RTOS_CM33_PRIO_BITS);  // kernel-aware: gives the semaphore
        cm33::nvic::enable(irq);
    }
    kernel.create_thread(main_tcb, {.name = "main", .entry = &main_thread, .stack = main_stack, .priority = 10});
    kernel.create_thread(background_tcb,
                         {.name = "background", .entry = &background_thread, .stack = background_stack, .priority = 20});
    kernel.start();
}
