// SPDX-License-Identifier: MIT
//
// Self-test, the Factory Control Unit's part: kernel binding, interrupt vectors, fault handlers
// and the tests of what only the STM32H563 has - the PSPLIM hardware stack limit (ARMv8-M) and
// the DTS temperature sensor (driver against the behaviour measured on the FCU).
#include <array>
#include <atomic>
#include <cstdint>

#include "rtos/port/cm33/port.hpp"
#include "rtos/port/stm32h5/dts.hpp"
#include "selftest.hpp"

RTOS_CM33_BIND_KERNEL(kernel)

namespace {

namespace cm33 = rtos::port::cm33;
namespace h5 = rtos::port::stm32h5;
using namespace selftest;

// ---------------------------------------------------------------------------
// Hardware stack-limit check (PSPLIM -> UsageFault STKOF), non-destructive:
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
// DTS temperature sensor. Regression: TS1_RDY reads 0 while the sensor
//     measures continuously (TS1_START = 1); the driver once read DR only with
//     TS1_RDY set and never delivered a temperature after start(). The emulators
//     model the FCU's behaviour (DR of about 28.6 degC with its factory values).
// ---------------------------------------------------------------------------

void test_cpu_temperature() {
    using Temperature = h5::Dts<{.pclk_hz = board::clocks.pclk1_hz()}>;
    Temperature::init();
    for (int i = 0; i < 10 && !Temperature::ready(); ++i) {
        kernel.sleep(1);
    }
    const bool was_ready = Temperature::ready();
    Temperature::start();
    kernel.sleep(5);
    const bool busy = !Temperature::ready();
    const auto t = Temperature::centi_celsius();
    board::print("[selftest]   CPU temperature ");
    board::print_dec(t ? static_cast<std::uint32_t>(*t) : 0u);
    board::print(" 1/100 degC\n");
    report("DTS temperature while measuring continuously (20..40 degC)",
           was_ready && busy && t && *t > 2000 && *t < 4000);
}

}  // namespace

void selftest::target_tests() {
    test_stack_limit();
    test_cpu_temperature();
}

extern "C" {
void TIM2_IRQHandler() noexcept { spare_irq(0); }
void TIM3_IRQHandler() noexcept { spare_irq(1); }
void TIM4_IRQHandler() noexcept { spare_irq(2); }
void TIM5_IRQHandler() noexcept { spare_irq(3); }

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
