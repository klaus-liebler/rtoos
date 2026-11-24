// SPDX-License-Identifier: MIT
//
// Cost of a login check: PBKDF2-HMAC-SHA256 (rtos/auth/pbkdf2.hpp) on the target, to choose the
// iteration count of the stored password records (docs/security.md "Login cost"): cycles per
// iteration, the iterations that fit 100/150/200/300 ms at SYSCLK, and the longest background step
// of `step_iterations` iterations. DWT_CYCCNT: real cycles on the hardware, instructions in the
// emulators. Ends with "AUTHBENCH RESULT: PASS" (emulation/run.py --result "AUTHBENCH RESULT").
#include <array>
#include <cstddef>
#include <cstdint>
#include <string_view>

#include "board/fcu.hpp"
#include "rtos/auth/pbkdf2.hpp"
#include "rtos/port/cm33/port.hpp"
#include "rtos/port/cortex_m/cycle_counter.hpp"
#include "rtos/rtos.hpp"

// The port's PendSV/SysTick handlers need a kernel; the benchmark never starts it.
rtos::Kernel<rtos::port::cm33::Cm33Port> kernel;
RTOS_CM33_BIND_KERNEL(kernel)

namespace {

using rtos::port::cortex_m::CycleCounter;
constexpr std::uint32_t iterations = 2000;
constexpr std::uint32_t step_iterations = 64;  // fcu auth.cpp: iterations_per_step

}  // namespace

int main() {
    board::init();
    CycleCounter::enable();
    board::print("\nrtoos authbench - PBKDF2-HMAC-SHA256 (DWT_CYCCNT: cycles on hardware, instructions in emulators)\n");
    constexpr std::string_view password = "Label-Pass-1234567";
    std::array<std::byte, 16> salt{};
    for (std::size_t i = 0; i < salt.size(); ++i) salt[i] = static_cast<std::byte>(i * 17);
    rtos::auth::Pbkdf2Sha256 kdf;
    const std::uint32_t t0 = CycleCounter::now();
    const bool ok = kdf.start(std::as_bytes(std::span(password)), salt, iterations);
    const std::uint32_t t_start = CycleCounter::now() - t0;
    std::uint32_t max_step = 0, total = t_start;
    bool more = ok;
    while (more) {
        const std::uint32_t s0 = CycleCounter::now();
        more = kdf.step(step_iterations);
        const std::uint32_t d = CycleCounter::now() - s0;
        total += d;
        max_step = d > max_step ? d : max_step;
    }
    const std::uint32_t per_iteration = total / iterations;
    board::print("AUTHBENCH iterations ");
    board::print_dec(iterations);
    board::print(" total ");
    board::print_dec(total);
    board::print(" per_iteration ");
    board::print_dec(per_iteration);
    board::print(" start ");
    board::print_dec(t_start);
    board::print(" max_step(");
    board::print_dec(step_iterations);
    board::print(") ");
    board::print_dec(max_step);
    board::print("\n");
    for (const std::uint32_t ms : {100u, 150u, 200u, 300u}) {
        board::print("AUTHBENCH iterations for ");
        board::print_dec(ms);
        board::print(" ms at sysclk: ");
        board::print_dec(static_cast<std::uint32_t>(static_cast<std::uint64_t>(board::sysclk_hz) / 1000 * ms / per_iteration));
        board::print("\n");
    }
    // first byte of the result, so the work cannot be optimised away
    board::print("AUTHBENCH result[0] ");
    board::print_dec(std::to_integer<std::uint32_t>(kdf.result()[0]));
    board::print(ok && kdf.done() ? "\nAUTHBENCH RESULT: PASS\n" : "\nAUTHBENCH RESULT: FAIL\n");
    board::flush();
    for (;;) {
        __asm volatile("wfi");
    }
}
