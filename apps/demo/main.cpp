// SPDX-License-Identifier: MIT
//
// Demo for the Factory Control Unit: three threads exercising the core scheduler.
//
//   sampler  (prio  5)            : wakes every 10 ms, signals the worker via a
//                                   lock-free task notification (same call works
//                                   from any ISR, e.g. a DMA-complete interrupt)
//   worker   (prio 10)            : blocks on notification slot 0 with timeout
//   logger   (prio 20, thresh 10) : shares state with worker -> preemption threshold
//                                   = worker's priority; the two never preempt each
//                                   other, so no mutex is needed. Blinks LED_INFO and
//                                   prints the statistics once per second.
#include <array>
#include <cstddef>
#include <cstdint>

#include "board/fcu.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/cache.hpp"
#include "rtos/rtos.hpp"

namespace {

struct AppConfig : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 4;
    static constexpr std::size_t notify_slots = 2;
};

using Port = rtos::port::cm33::Cm33Port;
using Kernel = rtos::Kernel<Port, AppConfig>;

}  // namespace

Kernel kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

constexpr std::size_t stack_bytes = 1024;
Kernel::Thread sampler_tcb, worker_tcb, logger_tcb;
alignas(8) std::array<std::byte, stack_bytes> sampler_stack, worker_stack, logger_stack;

// Example DMA target: lives in the non-cacheable region, line-aligned and padded.
RTOS_DMA_BUFFER rtos::port::stm32h5::DmaBuffer<std::uint16_t, 64> adc_samples;

// Message blocks from a deterministic pool instead of a heap.
rtos::BlockPool<32, 8> message_pool;

std::uint32_t processed = 0;  // worker + logger, protected by preemption threshold
std::uint32_t timeouts = 0;

constexpr std::uint32_t evt_sample_ready = 1u << 0;

void sampler(void*) {
    for (;;) {
        kernel.sleep(10);
        adc_samples.data[0] = static_cast<std::uint16_t>(kernel.now());
        kernel.notify(worker_tcb, 0, evt_sample_ready, rtos::NotifyAction::SetBits);
    }
}

void worker(void*) {
    for (;;) {
        if (auto events = kernel.notify_wait(0, 100)) {
            if ((*events & evt_sample_ready) != 0) {
                if (void* msg = message_pool.allocate()) {
                    *static_cast<std::uint32_t*>(msg) = adc_samples.data[0];
                    message_pool.deallocate(msg);
                }
                ++processed;
            }
        } else {
            ++timeouts;
        }
    }
}

void logger(void*) {
    for (;;) {
        kernel.sleep(1000);
        // Running with threshold 10: worker (prio 10) cannot preempt this section.
        board::led_info_toggle();
        board::print("t=");
        board::print_dec(kernel.now());
        board::print(" ms  samples=");
        board::print_dec(processed);
        board::print("  timeouts=");
        board::print_dec(timeouts);
        board::print("\n");
        processed = 0;
    }
}

}  // namespace

int main() {
    board::init();
    board::print("\nrtoos demo - STM32H563 / Factory Control Unit\n");
    Port::configure(board::sysclk_hz, 1000);

    kernel.create_thread(sampler_tcb, {.name = "sampler", .entry = &sampler, .stack = sampler_stack, .priority = 5});
    kernel.create_thread(worker_tcb, {.name = "worker", .entry = &worker, .stack = worker_stack, .priority = 10});
    kernel.create_thread(logger_tcb, {.name = "logger",
                                      .entry = &logger,
                                      .stack = logger_stack,
                                      .priority = 20,
                                      .preemption_threshold = 10});
    kernel.start();  // never returns
}
