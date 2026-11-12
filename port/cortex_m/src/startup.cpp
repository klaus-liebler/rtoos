// SPDX-License-Identifier: MIT
//
// Freestanding start-up for the Cortex-M chips (STM32H5, STM32G4): Reset_Handler and abort().
// The vector table is the chip's (generated vectors.cpp). No CMSIS, no newlib start files.
//
// Contract violations: the firmware is built with
// -fcontract-evaluation-semantic=quick_enforce, i.e. a violated contract calls
// std::terminate() directly (no violation handler) -> abort() below. Defining
// abort() here keeps newlib's version (raise/signal/malloc) out of the image.
//
// No heap at run time: newlib-nano's printf family references malloc (the log journal's
// vsnprintf never needs it for integers and strings), so _sbrk exists - and always refuses.
// A malloc() anywhere therefore returns nullptr instead of growing into the stacks.
#include <cerrno>
#include <cstddef>
#include <cstdint>

extern "C" {
extern std::uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
/// The DMA regions outside .bss (linker scripts; empty where the chip has none).
extern std::uint32_t __dma_start, __dma_end, __sdmmc_start, __sdmmc_end;
extern void (*__init_array_start[])();
extern void (*__init_array_end[])();
int main();

/// Board hook for pin-level fixes that cannot wait for main(): runs first after reset, before
/// .data is copied, .bss is zeroed and constructors run. Peripheral register writes only - no
/// globals, no FPU. Default: nothing; a board overrides it.
[[gnu::weak]] void board_early_init() noexcept {}

[[noreturn]] void Reset_Handler() noexcept {
    board_early_init();

    // Grant full access to the FPU (CP10/CP11) before any FP instruction can run.
    *reinterpret_cast<volatile std::uint32_t*>(0xE000'ED88) |= 0xFu << 20;
    __asm volatile("dsb \n isb" ::: "memory");

    for (std::uint32_t *src = &_sidata, *dst = &_sdata; dst < &_edata;) {
        *dst++ = *src++;
    }
    for (std::uint32_t* dst = &_sbss; dst < &_ebss;) {
        *dst++ = 0;
    }
    // The DMA sections are zeroed like .bss - the start value C++ gives every object before its
    // constructor runs. Not zeroed, an object that the compiler constant-initializes (no
    // constructor in __init_array) kept what the SRAM held after power-on: FCU revision 1 locked
    // up at boot (a stepper's "moving" flag set, its motion plan null). Emulators start with zeroed
    // RAM, so only the board showed it. Non-zero start values cannot live there at all (NOLOAD:
    // nothing is copied) - tools/check_noload.py refuses such an image.
    for (std::uint32_t* dst = &__dma_start; dst < &__dma_end;) {
        *dst++ = 0;
    }
    for (std::uint32_t* dst = &__sdmmc_start; dst < &__sdmmc_end;) {
        *dst++ = 0;
    }
    for (auto* ctor = __init_array_start; ctor != __init_array_end; ++ctor) {
        (*ctor)();
    }
    main();
    for (;;) {
    }
}

void* _sbrk(std::ptrdiff_t) noexcept {
    errno = ENOMEM;
    return reinterpret_cast<void*>(-1);
}

[[noreturn]] void abort() noexcept {
    __asm volatile("cpsid i \n bkpt #1");
    for (;;) {
    }
}
}
