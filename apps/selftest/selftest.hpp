// SPDX-License-Identifier: MIT
//
// The on-target self-test, split into the kernel tests every chip runs (main.cpp) and the
// target's part (<target>/target.cpp: console, spare interrupts, fault handlers, the tests of
// what only that chip has). The target's header (<target>/selftest_target.hpp, found through
// the include path of the firmware) provides:
//
//   namespace target {
//     using Port = ...;                              // the CpuPort
//     inline constexpr std::uint32_t sysclk_hz;      // SysTick / DWT clock
//     inline constexpr std::string_view name;        // banner
//     inline constexpr std::array<unsigned, 4> spare_irqs;  // software-triggered interrupts
//     void init(); print(); print_dec(); print_hex(); flush(); led_toggle();
//   }
#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>

#include "rtos/rtos.hpp"
#include "selftest_target.hpp"

namespace selftest {

struct Config : rtos::DefaultConfig {
    static constexpr std::size_t max_threads = 16;
    static constexpr std::size_t notify_slots = 2;
};

using Kernel = rtos::Kernel<target::Port, Config>;

inline constexpr std::size_t slot_notify = 0;  // test-specific signalling
inline constexpr std::size_t slot_done = 1;    // "test thread finished" -> runner

Kernel::Thread& spawn(const char* name, Kernel::Entry entry, std::uint8_t priority,
                      std::uint8_t threshold = Kernel::no_threshold, std::uint32_t time_slice = 0,
                      void* arg = nullptr);
/// Called by test threads when they are done; the runner waits for it.
void signal_done();
void wait_done(std::uint32_t count = 1);

void report(std::string_view name, bool ok);
/// A test whose hardware mechanism the platform does not implement (emulator gap).
void report_unsupported(std::string_view name);
[[noreturn]] void fatal(std::string_view what);

/// The handler of spare interrupt `index` (target::spare_irqs); the target's vectors call it.
void spare_irq(unsigned index) noexcept;

/// The target's own tests, run after the kernel tests (target.cpp).
void target_tests();

}  // namespace selftest

extern selftest::Kernel kernel;
