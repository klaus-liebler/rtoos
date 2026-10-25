// SPDX-License-Identifier: MIT
//
// Freestanding start-up for STM32H563: Reset_Handler and abort(). The vector table
// is generated (vectors.cpp). No CMSIS, no newlib start files.
//
// Contract violations: the firmware is built with
// -fcontract-evaluation-semantic=quick_enforce, i.e. a violated contract calls
// std::terminate() directly (no violation handler) -> abort() below. Defining
// abort() here keeps newlib's version (raise/signal/malloc) out of the image.
#include <cstdint>

extern "C" {
extern std::uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();
int main();

[[noreturn]] void Reset_Handler() noexcept {
    // Grant full access to the FPU (CP10/CP11) before any FP instruction can run.
    *reinterpret_cast<volatile std::uint32_t*>(0xE000'ED88) |= 0xFu << 20;
    __asm volatile("dsb \n isb" ::: "memory");

    for (std::uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++;
    }
    for (std::uint32_t* dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }
    for (auto* ctor = __init_array_start; ctor != __init_array_end; ++ctor) {
        (*ctor)();
    }
    main();
    for (;;) {
    }
}

[[noreturn]] void abort() noexcept {
    __asm volatile("cpsid i \n bkpt #1");
    for (;;) {
    }
}
}
