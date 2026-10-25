// SPDX-License-Identifier: MIT
//
// On-target self-test of the core scheduler. Runs on the Factory Control Unit and
// in the emulators (QEMU machine "fcu", Renode platform fcu.repl); reports over the
// console (USART3) and ends with exactly one line
//
//     SELFTEST RESULT: PASS (n/n)      or      SELFTEST RESULT: FAIL (k/n)
//
// The tests exercise what the host unit tests can only simulate: real PendSV
// context switches, FPU lazy stacking, interrupts above the kernel's BASEPRI mask,
// SysTick timing, the kernel objects driven from interrupts of both classes
// (kernel-aware and zero-latency), the PSPLIM hardware stack limit and the DWT
// cycle counter against SysTick (both count SYSCLK cycles).
//
// A test whose hardware mechanism the platform does not implement (e.g. an
// emulator without PSPLIM enforcement) is reported as UNSUPPORTED, not as FAIL:
//
//     SELFTEST RESULT: PASS (11/11, 1 unsupported)
#include <array>
#include <atomic>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "board/fcu.hpp"
#include "rtos/port/cm33/cycle_counter.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/chip.hpp"
#include "rtos/rtos.hpp"
#include "rtos/sync/message_queue.hpp"
#include "rtos/sync/object_pool.hpp"
#include "rtos/sync/semaphore.hpp"

namespace {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 16;
    static constexpr std::size_t notify_slots = 2;
};

using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, Config>;
using rtos::NotifyAction;

}  // namespace

Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

namespace cm33 = rtos::port::cm33;
namespace h5 = rtos::port::stm32h5;

constexpr std::size_t slot_notify = 0;  // test-specific signalling
constexpr std::size_t slot_done = 1;    // "test thread finished" -> runner

// ---------------------------------------------------------------------------
// Infrastructure
// ---------------------------------------------------------------------------

struct ThreadSlot {
    Kernel::Thread tcb;
    alignas(8) std::array<std::byte, 1024> stack;
};
std::array<ThreadSlot, 14> slots;  // one per spawn() below
std::size_t slots_used = 0;

Kernel::Thread* runner = nullptr;

Kernel::Thread& spawn(const char* name, Kernel::Entry entry, std::uint8_t priority,
                      std::uint8_t threshold = Kernel::no_threshold, std::uint32_t time_slice = 0,
                      void* arg = nullptr) {
    contract_assert(slots_used < slots.size());
    ThreadSlot& s = slots[slots_used++];
    kernel.create_thread(s.tcb, {.name = name,
                                 .entry = entry,
                                 .arg = arg,
                                 .stack = s.stack,
                                 .priority = priority,
                                 .preemption_threshold = threshold,
                                 .time_slice = time_slice});
    return s.tcb;
}

/// Called by test threads when they are done; the runner waits for it.
void signal_done() { kernel.notify(*runner, slot_done, 0, NotifyAction::Increment); }

void wait_done(std::uint32_t count = 1) {
    std::uint32_t seen = 0;
    while (seen < count) {
        auto n = kernel.notify_wait(slot_done, 1000);
        if (!n) {
            return;  // timeout: the test's check will fail
        }
        seen += *n;
    }
}

// Ordered event trace written by the threads of a test.
std::array<char, 16> trace_buf{};
std::atomic<std::size_t> trace_len{0};
void trace_reset() { trace_len = 0; }
void mark(char c) { trace_buf[trace_len.fetch_add(1) % trace_buf.size()] = c; }
bool trace_is(std::string_view expected) {
    return std::string_view(trace_buf.data(), trace_len.load()) == expected;
}

unsigned passed = 0;
unsigned failed = 0;
unsigned unsupported = 0;

void report(std::string_view name, bool ok) {
    ok ? ++passed : ++failed;
    board::print(ok ? "[selftest] PASS " : "[selftest] FAIL ");
    board::print_dec(passed + failed);
    board::print(" ");
    board::print(name);
    board::print("\n");
    board::led_info_toggle();
}

void report_unsupported(std::string_view name) {
    ++unsupported;
    board::print("[selftest] UNSUPPORTED ");
    board::print(name);
    board::print("\n");
}

[[noreturn]] void finish() {
    __asm volatile("cpsid i" ::: "memory");
    board::print(failed == 0 ? "SELFTEST RESULT: PASS (" : "SELFTEST RESULT: FAIL (");
    board::print_dec(passed);
    board::print("/");
    board::print_dec(passed + failed);
    if (unsupported != 0) {
        board::print(", ");
        board::print_dec(unsupported);
        board::print(" unsupported");
    }
    board::print(")\n");
    board::flush();
    for (;;) {
        __asm volatile("wfi");
    }
}

[[noreturn]] void fatal(std::string_view what) {
    board::print("\n[selftest] ");
    board::print(what);
    board::print(" CFSR=");
    board::print_hex(cm33::scb::CFSR::read());
    board::print(" HFSR=");
    board::print_hex(cm33::scb::HFSR::read());
    board::print("\n");
    ++failed;
    finish();
}

// ---------------------------------------------------------------------------
// 1. Priority preemption through PendSV
// ---------------------------------------------------------------------------

void t1_high(void*) {
    mark('1');
    auto v = kernel.notify_wait(slot_notify, 100);
    mark(v && *v == 7 ? '3' : 'x');
}

void test_priority_preemption() {
    trace_reset();
    Kernel::Thread& high = spawn("t1", &t1_high, 5);  // preempts the runner at once
    mark('2');
    kernel.notify(high, slot_notify, 7, NotifyAction::Overwrite);  // preempts again
    mark('4');
    report("priority preemption", trace_is("1234"));
}

// ---------------------------------------------------------------------------
// 2. Preemption threshold (ThreadX semantics)
// ---------------------------------------------------------------------------

void t2_mid(void*) {
    mark('M');
    signal_done();
}
void t2_high(void*) { mark('H'); }
void t2_low(void*) {
    spawn("t2m", &t2_mid, 10);  // 10 is not more urgent than threshold 6: no switch
    mark('a');
    spawn("t2h", &t2_high, 4);  // 4 beats the threshold: preempts
    mark('b');                  // resumes before t2m although t2m is more urgent
}

void test_preemption_threshold() {
    trace_reset();
    spawn("t2l", &t2_low, 15, /*threshold=*/6);
    wait_done();
    report("preemption threshold", trace_is("aHbM"));
}

// ---------------------------------------------------------------------------
// 3. FPU context (s16-s31 + lazy stacking) survives context switches
// ---------------------------------------------------------------------------

volatile float fpu_mul = 1.0001f;  // volatile: no constant folding

[[gnu::noinline]] float fpu_work(float seed, bool yield) {
    float a = seed, b = seed * 0.5f, c = 0.0f, d = 1.0f;
    const float mul = fpu_mul;
    for (int i = 0; i < 300; ++i) {
        a = a * mul + 0.25f;
        b = b * mul - 0.125f;
        c = c + a * b * 1e-3f;
        d = d * 0.999f + c;
        if (yield && i % 7 == 0) {
            kernel.yield();  // a..d live in callee-saved s16+ across the switch
        }
    }
    return a + b + c + d;
}

std::array<float, 2> fpu_result{};
void t3_fpu(void* arg) {
    const auto i = reinterpret_cast<std::uintptr_t>(arg);
    fpu_result[i] = fpu_work(i == 0 ? 1.5f : -2.25f, true);
    signal_done();
}

void test_fpu_context() {
    const float expect0 = fpu_work(1.5f, false);
    const float expect1 = fpu_work(-2.25f, false);
    spawn("t3a", &t3_fpu, 12, Kernel::no_threshold, 0, reinterpret_cast<void*>(0));
    spawn("t3b", &t3_fpu, 12, Kernel::no_threshold, 0, reinterpret_cast<void*>(1));
    wait_done(2);
    report("FPU context switch", fpu_result[0] == expect0 && fpu_result[1] == expect1);
}

// ---------------------------------------------------------------------------
// 4. Lock-free notification from an interrupt ABOVE the kernel mask
// ---------------------------------------------------------------------------

Kernel::Thread* t4_target = nullptr;
std::uint32_t t4_value = 0;

void t4_wait(void*) {
    auto v = kernel.notify_wait(slot_notify, 100);
    t4_value = v.value_or(0);
    signal_done();
}

void test_isr_notify() {
    constexpr unsigned irq = static_cast<unsigned>(h5::Irq::TIM2);  // software-triggered
    t4_target = &spawn("t4", &t4_wait, 8);
    cm33::nvic::ipr(irq) = 2u << (8 - RTOS_CM33_PRIO_BITS);  // never masked by the kernel
    cm33::nvic::enable(irq);
    cm33::nvic::pend(irq);
    wait_done();
    report("notify from zero-latency ISR", t4_value == 0xA5);
}

// ---------------------------------------------------------------------------
// 5./6. SysTick-driven sleep and timeout
// ---------------------------------------------------------------------------

void test_sleep() {
    const rtos::Tick t0 = kernel.now();
    kernel.sleep(25);
    report("sleep 25 ticks", kernel.now() - t0 == 25);
}

void test_timeout() {
    const rtos::Tick t0 = kernel.now();
    const auto v = kernel.notify_wait(slot_notify, 10);
    report("notify timeout 10 ticks", !v && kernel.now() - t0 == 10);
}

// ---------------------------------------------------------------------------
// 7. Round-robin time slicing
// ---------------------------------------------------------------------------

std::atomic<bool> rr_stop{false};
std::array<std::atomic<std::uint32_t>, 2> rr_count{};

void t7_spin(void* arg) {
    auto& count = rr_count[reinterpret_cast<std::uintptr_t>(arg)];
    while (!rr_stop.load(std::memory_order_relaxed)) {
        count.fetch_add(1, std::memory_order_relaxed);
    }
    signal_done();
}

void t7_control(void*) {
    // Most urgent thread: creates the spinners without being preempted by them.
    spawn("t7a", &t7_spin, 12, Kernel::no_threshold, /*time_slice=*/2, reinterpret_cast<void*>(0));
    spawn("t7b", &t7_spin, 12, Kernel::no_threshold, /*time_slice=*/2, reinterpret_cast<void*>(1));
    kernel.sleep(20);
    rr_stop = true;
}

void test_round_robin() {
    spawn("t7c", &t7_control, 3);
    wait_done(2);
    report("round-robin time slicing", rr_count[0] > 0 && rr_count[1] > 0);
}

// ---------------------------------------------------------------------------
// 8.-10. Kernel objects driven from interrupts (software-triggered IRQs)
//   kernel-aware ISR  : priority 6 (masked by the kernel lock, may take it)
//   zero-latency ISR  : priority 1 (never masked, lock-free services only)
// ---------------------------------------------------------------------------

constexpr std::uint8_t kernel_aware_priority = 6u << (8 - RTOS_CM33_PRIO_BITS);
constexpr std::uint8_t zero_latency_priority = 1u << (8 - RTOS_CM33_PRIO_BITS);

void trigger(h5::Irq irq, std::uint8_t priority) {
    const auto n = static_cast<unsigned>(irq);
    cm33::nvic::ipr(n) = priority;
    cm33::nvic::enable(n);
    cm33::nvic::pend(n);
}

rtos::Semaphore<Kernel> isr_semaphore{kernel, 0, 1};

void test_semaphore_from_isr() {
    trigger(h5::Irq::TIM3, kernel_aware_priority);  // runs at once, gives
    const bool first = isr_semaphore.take(10);
    const bool second = isr_semaphore.take(5);  // nothing more given: timeout
    report("semaphore give_from_isr", first && !second);
}

rtos::MessageQueue<Kernel, std::uint32_t, 4> isr_queue{kernel};

void test_queue_from_isr() {
    trigger(h5::Irq::TIM4, kernel_aware_priority);  // sends 11, 22, 33
    const auto m1 = isr_queue.receive(10);
    const auto m2 = isr_queue.receive(10);
    const auto m3 = isr_queue.receive(10);
    report("message queue try_send_from_isr", m1 == 11u && m2 == 22u && m3 == 33u);
}

struct Packet {
    std::uint32_t tag;
};
rtos::ObjectPool<Kernel, Packet, 1> pool{kernel};
Packet* pool_block_held_by_isr = nullptr;
std::uint32_t t10_tag = 0;

void t10_allocator(void*) {
    auto packet = pool.make(100, 0xC0FFEEu);  // pool empty: blocks until the ISR frees
    t10_tag = packet ? packet->tag : 0;
    signal_done();
}

void test_pool_free_from_zero_latency_isr() {
    pool_block_held_by_isr = pool.try_make(1u).release();
    spawn("t10", &t10_allocator, 9);  // more urgent than the runner: blocks at once
    trigger(h5::Irq::TIM5, zero_latency_priority);  // frees lock-free, defers the wake
    wait_done();
    report("pool free from zero-latency ISR wakes allocator", t10_tag == 0xC0FFEEu && pool.available() == 1);
}

// ---------------------------------------------------------------------------
// 11. Hardware stack-limit check (PSPLIM -> UsageFault STKOF), non-destructive:
//    the thread raises its own PSPLIM 256 bytes below the current SP (still inside
//    its real stack; room for the exception frame) and then allocates 512 bytes.
//    The UsageFault handler lowers PSPLIM back to the real limit and returns; the
//    allocation is re-executed and succeeds. Nothing outside the stack is touched,
//    on any platform.
// ---------------------------------------------------------------------------

std::atomic<bool> stkof_armed{false};
std::atomic<bool> stkof_seen{false};
std::uint32_t t8_result = 0;

[[gnu::noinline]] std::uint32_t use_512_bytes_of_stack() {
    std::array<volatile std::uint8_t, 512> buffer{};
    buffer[0] = 1;
    buffer[511] = 2;
    return buffer[0] + buffer[511];
}

void set_psplim(std::uintptr_t limit) { __asm volatile("msr psplim, %0 \n isb" ::"r"(limit) : "memory"); }

void t8_stack_limit(void*) {
    std::uintptr_t psp;
    __asm volatile("mrs %0, psp" : "=r"(psp));
    stkof_armed = true;
    set_psplim((psp - 256) & ~std::uintptr_t{7});
    t8_result = use_512_bytes_of_stack();
    set_psplim(kernel.current()->stack_limit);
    stkof_armed = false;
    signal_done();
}

void test_stack_limit() {
    spawn("t8", &t8_stack_limit, 7);
    wait_done();
    if (t8_result != 3) {
        report("stack limit (PSPLIM)", false);
    } else if (stkof_seen) {
        report("stack limit (PSPLIM)", true);
    } else {
        report_unsupported("stack limit (PSPLIM not enforced by this platform)");
    }
}

// ---------------------------------------------------------------------------
// 12. DWT cycle counter: 20 SysTick periods = 20 ms = sysclk_hz / 50 cycles
// ---------------------------------------------------------------------------

void test_cycle_counter() {
    if (!cm33::CycleCounter::enable()) {
        report("cycle counter (DWT_CYCCNT)", false);
        return;
    }
    kernel.sleep(1);  // align to a tick boundary
    const std::uint32_t c0 = cm33::CycleCounter::now();
    kernel.sleep(20);
    const std::uint32_t cycles = cm33::CycleCounter::now() - c0;
    constexpr std::uint32_t expected = board::sysclk_hz / 50;
    board::print("[selftest]   20 ms = ");
    board::print_dec(cycles);
    board::print(" cycles\n");
    report("cycle counter vs. SysTick (+-10 %)", cycles > expected / 10 * 9 && cycles < expected / 10 * 11);
}

// ---------------------------------------------------------------------------

void runner_main(void*) {
    test_priority_preemption();
    test_preemption_threshold();
    test_fpu_context();
    test_isr_notify();
    test_sleep();
    test_timeout();
    test_round_robin();
    test_semaphore_from_isr();
    test_queue_from_isr();
    test_pool_free_from_zero_latency_isr();
    test_stack_limit();
    test_cycle_counter();
    finish();
}

}  // namespace

extern "C" {
void TIM2_IRQHandler() noexcept { kernel.notify_from_isr(*t4_target, slot_notify, 0xA5, NotifyAction::SetBits); }
void TIM3_IRQHandler() noexcept { (void)isr_semaphore.give_from_isr(); }
void TIM4_IRQHandler() noexcept {
    for (std::uint32_t i = 1; i <= 3; ++i) {
        (void)isr_queue.try_send_from_isr(i * 11);
    }
}
void TIM5_IRQHandler() noexcept { pool.destroy_from_isr(pool_block_held_by_isr); }

void UsageFault_Handler() noexcept {
    if (stkof_armed && (cm33::scb::CFSR::read() & cm33::scb::cfsr_stkof) != 0) {
        cm33::scb::CFSR::write(cm33::scb::cfsr_stkof);  // write-1-to-clear
        set_psplim(kernel.current()->stack_limit);
        stkof_seen = true;
        return;  // the faulting stack allocation is re-executed and now succeeds
    }
    fatal("UsageFault");
}
void HardFault_Handler() noexcept { fatal("HardFault"); }
void MemManage_Handler() noexcept { fatal("MemManage"); }
void BusFault_Handler() noexcept { fatal("BusFault"); }
}

int main() {
    board::init();
    board::print("\nrtoos selftest - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);
    runner = &spawn("runner", &runner_main, 20);
    kernel.start();
}
