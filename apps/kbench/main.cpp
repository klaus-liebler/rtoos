// SPDX-License-Identifier: MIT
//
// Kernel benchmark: latencies of the kernel services, each min/avg/max over `runs` runs,
// measured with DWT_CYCCNT (real core cycles on the hardware, instructions in the emulators:
// both run 250 MIPS at CPI 1, so emulator numbers are instruction counts, not cycles).
//
//   notify -> waiting thread       a thread notifies a more urgent thread blocked in
//                                  notify_wait(): from the call until that thread runs
//   notify ping-pong round trip    two threads of equal priority notify each other: two
//                                  notifies, two waits, two context switches
//   semaphore give -> waiter       give() wakes a more urgent thread blocked in take()
//   queue round trip               send to a more urgent thread, it sends the message back
//   queue send + receive           same thread, no switch (the bare queue operations)
//   ISR -> thread                  software-pended IRQ, the handler notify_from_isr()s a
//                                  waiting thread: pend -> handler entry, pend -> thread
//                                  (kernel-aware priority and zero-latency priority)
//   pool alloc / free              ObjectPool try_allocate() + deallocate(), no waiters
//   Modbus PDU                     process_pdu() against a RegisterBank guarded by the
//                                  kernel lock (the network thread's share, no TCP/IP)
//
// The values include the cost of reading the cycle counter (printed as "overhead", not
// subtracted). The SysTick (1 kHz) keeps running, so a maximum may contain one tick.
// The console ends with "KBENCH RESULT: PASS" (emulation/run.py --result "KBENCH RESULT").
#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "board/fcu.hpp"
#include "rtos/modbus/modbus.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/port/cortex_m/registers.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/rtos.hpp"

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 10;
    static constexpr std::size_t notify_slots = 1;
};
using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, Config>;

}  // namespace

Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

namespace cortex_m = rtos::port::cortex_m;
namespace h5 = rtos::port::stm32h5;
using cortex_m::CycleCounter;
using rtos::NotifyAction;
using rtos::wait_forever;

constexpr std::uint32_t runs = 1000;

// ---------------------------------------------------------------------------
// Statistics
// ---------------------------------------------------------------------------

struct Stats {
    std::uint32_t min = UINT32_MAX;
    std::uint32_t max = 0;
    std::uint64_t sum = 0;
    std::uint32_t n = 0;
    void add(std::uint32_t v) noexcept {
        min = std::min(min, v);
        max = std::max(max, v);
        sum += v;
        ++n;
    }
};

void print_row(const char* name, const Stats& s) {
    board::print("KBENCH ");
    board::print(name);
    board::print(": min ");
    board::print_dec(s.min);
    board::print(" avg ");
    const std::uint64_t avg10 = s.n ? (s.sum * 10 + s.n / 2) / s.n : 0;
    board::print_dec(static_cast<std::uint32_t>(avg10 / 10));
    board::print(".");
    board::print_dec(static_cast<std::uint32_t>(avg10 % 10));
    board::print(" max ");
    board::print_dec(s.max);
    board::print(" n ");
    board::print_dec(s.n);
    board::print("\n");
    board::flush();
}

template <class F>
Stats measure(F&& f) {
    Stats s;
    for (std::uint32_t i = 0; i < runs; ++i) {
        const std::uint32_t t0 = CycleCounter::now();
        f();
        s.add(CycleCounter::now() - t0);
    }
    return s;
}

// ---------------------------------------------------------------------------
// Threads
// ---------------------------------------------------------------------------

struct ThreadSlot {
    Kernel::Thread tcb;
    alignas(8) std::array<std::byte, 1024> stack;
};
std::array<ThreadSlot, 8> slots;
std::size_t slots_used = 0;

Kernel::Thread& spawn(const char* name, Kernel::Entry entry, std::uint8_t priority) {
    ThreadSlot& s = slots[slots_used++];
    kernel.create_thread(s.tcb, {.name = name, .entry = entry, .arg = nullptr, .stack = s.stack,
                                 .priority = priority});
    return s.tcb;
}

constexpr std::uint8_t prio_runner = 20;
constexpr std::uint8_t prio_waiter = 10;  // more urgent than the runner

Kernel::Thread* runner = nullptr;
volatile std::uint32_t t_start = 0;   // stamp taken right before the triggering call
volatile std::uint32_t t_isr = 0;     // stamp at handler entry
Stats wake;                           // filled by the woken threads
Stats isr_entry;

// (a) notify -> more urgent waiting thread
void notify_waiter(void*) {
    for (;;) {
        (void)kernel.notify_wait(0, wait_forever);
        wake.add(CycleCounter::now() - t_start);
    }
}

// (a') ping-pong partner, same priority as the runner
void pong(void*) {
    for (;;) {
        (void)kernel.notify_wait(0, wait_forever);
        kernel.notify(*runner, 0, 1, NotifyAction::Increment);
    }
}

// (b) semaphore
rtos::Semaphore<Kernel> semaphore{kernel, 0, 1};
rtos::Semaphore<Kernel> free_semaphore{kernel, 0, 1};  // nobody waits on it
void semaphore_waiter(void*) {
    for (;;) {
        (void)semaphore.take(wait_forever);
        wake.add(CycleCounter::now() - t_start);
    }
}

// (c) message queues
rtos::MessageQueue<Kernel, std::uint32_t, 4> to_echo{kernel};
rtos::MessageQueue<Kernel, std::uint32_t, 4> from_echo{kernel};
rtos::MessageQueue<Kernel, std::uint32_t, 4> local_queue{kernel};
void echo(void*) {
    for (;;) {
        if (auto m = to_echo.receive(wait_forever)) {
            (void)from_echo.send(*m, wait_forever);
        }
    }
}

// (d) interrupt -> thread
Kernel::Thread* isr_target = nullptr;
void isr_waiter(void*) {
    for (;;) {
        (void)kernel.notify_wait(0, wait_forever);
        wake.add(CycleCounter::now() - t_start);
    }
}
constexpr unsigned irq_kernel_aware = static_cast<unsigned>(h5::Irq::TIM2);
constexpr unsigned irq_zero_latency = static_cast<unsigned>(h5::Irq::TIM3);

// (e) pool
struct Block {
    std::array<std::uint32_t, 16> data;  // 64 bytes
};
rtos::ObjectPool<Kernel, Block, 8> pool{kernel};

// (f) Modbus
using Registers = rtos::modbus::RegisterBank<256, 256, Kernel::Lock>;
Registers registers;

void run_isr_case(const char* name_entry, const char* name_thread, unsigned irq, std::uint8_t priority) {
    cortex_m::nvic::ipr(irq) = Port::priority(priority);
    cortex_m::nvic::enable(irq);
    wake = {};
    isr_entry = {};
    for (std::uint32_t i = 0; i < runs; ++i) {
        t_start = CycleCounter::now();
        cortex_m::nvic::pend(irq);  // handler, then the waiter, then back here
    }
    print_row(name_entry, isr_entry);
    print_row(name_thread, wake);
}

void runner_main(void*) {
    board::print("KBENCH unit: DWT_CYCCNT (cycles on hardware, instructions in emulators)\n");
    board::print("KBENCH sysclk_hz: ");
    board::print_dec(board::sysclk_hz);
    board::print("\n");

    print_row("overhead (two CYCCNT reads)", measure([] {}));

    // (a) notify -> more urgent waiting thread
    Kernel::Thread& waiter = spawn("waiter", &notify_waiter, prio_waiter);  // runs, blocks
    wake = {};
    for (std::uint32_t i = 0; i < runs; ++i) {
        t_start = CycleCounter::now();
        kernel.notify(waiter, 0, 1, NotifyAction::Increment);
    }
    print_row("notify -> waiting thread running", wake);

    // (a') ping-pong between two threads of equal priority
    Kernel::Thread& partner = spawn("pong", &pong, prio_runner);
    print_row("notify ping-pong round trip (2 switches)", measure([&] {
                  kernel.notify(partner, 0, 1, NotifyAction::Increment);
                  (void)kernel.notify_wait(0, wait_forever);
              }));

    // (b) semaphore give -> more urgent waiter
    spawn("semwait", &semaphore_waiter, prio_waiter);
    wake = {};
    for (std::uint32_t i = 0; i < runs; ++i) {
        t_start = CycleCounter::now();
        (void)semaphore.give();
    }
    print_row("semaphore give -> waiting thread running", wake);
    print_row("semaphore give + take (no waiter)", measure([] {
                  (void)free_semaphore.give();
                  (void)free_semaphore.try_take();
              }));

    // (c) message queue
    spawn("echo", &echo, prio_waiter);
    std::uint32_t value = 0;
    print_row("queue round trip via thread (2 switches)", measure([&] {
                  (void)to_echo.send(++value, wait_forever);
                  (void)from_echo.receive(wait_forever);
              }));
    print_row("queue send + receive (same thread)", measure([&] {
                  (void)local_queue.try_send(++value);
                  (void)local_queue.try_receive();
              }));

    // (d) interrupt -> thread
    isr_target = &spawn("isrwait", &isr_waiter, prio_waiter);
    run_isr_case("ISR pend -> handler entry (kernel-aware prio)", "ISR pend -> notified thread running (kernel-aware prio)",
                 irq_kernel_aware, 6);
    run_isr_case("ISR pend -> handler entry (zero-latency prio)", "ISR pend -> notified thread running (zero-latency prio)",
                 irq_zero_latency, 1);

    // (e) pool
    void* held = nullptr;
    print_row("pool allocate (64 B block)", [&] {
        Stats s;
        for (std::uint32_t i = 0; i < runs; ++i) {
            const std::uint32_t t0 = CycleCounter::now();
            held = pool.try_allocate();
            s.add(CycleCounter::now() - t0);
            pool.deallocate(held);
        }
        return s;
    }());
    print_row("pool free (64 B block)", [&] {
        Stats s;
        for (std::uint32_t i = 0; i < runs; ++i) {
            held = pool.try_allocate();
            const std::uint32_t t0 = CycleCounter::now();
            pool.deallocate(held);
            s.add(CycleCounter::now() - t0);
        }
        return s;
    }());
    print_row("pool allocate + free", measure([&] {
                  held = pool.try_allocate();
                  pool.deallocate(held);
              }));

    // (f) Modbus PDU processing
    for (std::uint16_t i = 0; i < 256; ++i) {
        registers.set_holding(i, i);
    }
    std::array<std::byte, rtos::modbus::max_pdu> response{};
    const std::array<std::byte, 5> read10{std::byte{0x03}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{10}};
    const std::array<std::byte, 5> read125{std::byte{0x03}, std::byte{0}, std::byte{0}, std::byte{0}, std::byte{125}};
    std::array<std::byte, 6 + 20> write10{std::byte{0x10}, std::byte{0}, std::byte{16}, std::byte{0}, std::byte{10},
                                          std::byte{20}};
    std::size_t length = 0;
    print_row("Modbus PDU read 10 holding registers", measure([&] {
                  length += rtos::modbus::process_pdu(registers, read10, response);
              }));
    print_row("Modbus PDU read 125 holding registers", measure([&] {
                  length += rtos::modbus::process_pdu(registers, read125, response);
              }));
    print_row("Modbus PDU write 10 holding registers", measure([&] {
                  length += rtos::modbus::process_pdu(registers, write10, response);
              }));

    const bool ok = length == runs * (2 + 20) + runs * (2 + 250) + runs * 5;
    board::print(ok ? "KBENCH RESULT: PASS\n" : "KBENCH RESULT: FAIL (Modbus responses)\n");
    board::flush();
    for (;;) {
        kernel.sleep(1000);
    }
}

}  // namespace

extern "C" {
void TIM2_IRQHandler() noexcept {
    t_isr = CycleCounter::now();
    isr_entry.add(t_isr - t_start);
    kernel.notify_from_isr(*isr_target, 0, 1, NotifyAction::Increment);
}
void TIM3_IRQHandler() noexcept {
    t_isr = CycleCounter::now();
    isr_entry.add(t_isr - t_start);
    kernel.notify_from_isr(*isr_target, 0, 1, NotifyAction::Increment);
}
}

int main() {
    board::init();
    CycleCounter::enable();
    board::print("\nrtoos kbench - kernel service latencies (STM32H563 / Factory Control Unit)\n");
    Port::configure(board::sysclk_hz, 1000);
    runner = &spawn("runner", &runner_main, prio_runner);
    kernel.start();
}
