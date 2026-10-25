// SPDX-License-Identifier: MIT
//
// Cortex-M33 context switch.
//
// Saved context layout on the thread stack (growing down, lowest address = saved SP):
//
//   saved SP -> r4 r5 r6 r7 r8 r9 r10 r11 EXC_RETURN   (software frame, 9 words)
//               [s16 ... s31]                           (only if EXC_RETURN.FType == 0)
//               r0 r1 r2 r3 r12 lr pc xPSR              (hardware frame)
//               [s0 ... s15 FPSCR reserved]             (hardware, lazy, if FP active)
//
// A fresh thread stores EXC_RETURN = 0 (sentinel) and is resumed with the
// EXC_RETURN template captured by the start-up SVC (Thread mode, PSP, no FP frame,
// same security state as the kernel).
#include "rtos/port/cm33/port.hpp"

extern "C" {

std::uint32_t rtos_port_start_ctx[2];
/// EXC_RETURN used for threads that have never run. Written once by SVC_Handler.
std::uint32_t rtos_port_exc_return_template;

std::uint64_t rtos_port_switch(void* sp) noexcept;  // RTOS_CM33_BIND_KERNEL

/// Trap for a thread that returns through LR (never happens: the kernel trampoline
/// terminates threads). Lands here only on a corrupted stack.
[[noreturn]] void rtos_port_thread_return_trap() noexcept {
    __asm volatile("bkpt #0");
    for (;;) {
    }
}

// --------------------------------------------------------------------------------
// PendSV: save outgoing context, ask the kernel for the next thread, restore it.
// Runs at the lowest priority; the kernel locks itself inside switch_context().
// --------------------------------------------------------------------------------
[[gnu::naked]] void PendSV_Handler() noexcept {
    __asm volatile(
        "   mrs     r0, psp                         \n"
        "   isb                                     \n"
#if defined(__ARM_FP)
        // Thread used the FPU -> callee-saved FP registers belong to its context.
        // (The VSTM also triggers any pending lazy stacking of s0-s15.)
        "   tst     lr, #0x10                       \n"
        "   it      eq                              \n"
        "   vstmdbeq r0!, {s16-s31}                 \n"
#endif
        "   stmdb   r0!, {r4-r11, lr}               \n"
        "   bl      rtos_port_switch                \n"  // r0 = next PSP, r1 = next PSPLIM
        "   ldmia   r0!, {r4-r11, lr}               \n"
        // Fresh thread: replace sentinel 0 by the EXC_RETURN template.
        "   cmp     lr, #0                          \n"
        "   ittt    eq                              \n"
        "   movweq  lr, #:lower16:rtos_port_exc_return_template \n"
        "   movteq  lr, #:upper16:rtos_port_exc_return_template \n"
        "   ldreq   lr, [lr]                        \n"
#if defined(__ARM_FP)
        "   tst     lr, #0x10                       \n"
        "   it      eq                              \n"
        "   vldmiaeq r0!, {s16-s31}                 \n"
#endif
        // Limit first: the new PSP is always above the new limit.
        "   msr     psplim, r1                      \n"
        "   msr     psp, r0                         \n"
        "   isb                                     \n"
        "   bx      lr                              \n");
}

// --------------------------------------------------------------------------------
// SVC #0: launch the first thread. Captures the EXC_RETURN template, reclaims the
// boot (MSP) stack and returns into the thread on PSP.
// --------------------------------------------------------------------------------
[[gnu::naked]] void SVC_Handler() noexcept {
    __asm volatile(
        "   movw    r2, #:lower16:rtos_port_start_ctx \n"
        "   movt    r2, #:upper16:rtos_port_start_ctx \n"
        "   ldmia   r2, {r0, r1}                    \n"  // r0 = PSP, r1 = PSPLIM
        "   ldmia   r0!, {r4-r11}                   \n"
        "   adds    r0, r0, #4                      \n"  // skip EXC_RETURN sentinel
        // Caller was Thread mode on MSP. Keep the security bits (S, ES, DCRS),
        // select PSP (bit 2) and a standard, FP-less frame (bit 4).
        "   orr     lr, lr, #0x14                   \n"
        "   movw    r2, #:lower16:rtos_port_exc_return_template \n"
        "   movt    r2, #:upper16:rtos_port_exc_return_template \n"
        "   str     lr, [r2]                        \n"
        "   msr     psplim, r1                      \n"
        "   msr     psp, r0                         \n"
        // Reset MSP to its initial value (vector table entry 0): the main() stack
        // is never returned to and becomes the dedicated interrupt stack.
        "   movw    r2, #0xED08                     \n"
        "   movt    r2, #0xE000                     \n"
        "   ldr     r2, [r2]                        \n"
        "   ldr     r2, [r2]                        \n"
        "   msr     msp, r2                         \n"
        "   isb                                     \n"
        "   bx      lr                              \n");
}

}  // extern "C"

namespace rtos::port::cm33 {

void* Cm33Port::init_stack(std::span<std::byte> stack, PortEntry entry, void* a0, void* a1) noexcept {
    // AAPCS: 8-byte aligned stack at function entry.
    auto top = (reinterpret_cast<std::uintptr_t>(stack.data()) + stack.size()) & ~std::uintptr_t{7};
    auto* sp = reinterpret_cast<std::uint32_t*>(top);

    // Hardware exception frame, unstacked by the exception return.
    *--sp = 0x0100'0000u;  // xPSR: Thumb bit
    *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(entry)) & ~1u;  // PC
    *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(&rtos_port_thread_return_trap));  // LR
    *--sp = 0;  // r12
    *--sp = 0;  // r3
    *--sp = 0;  // r2
    *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(a1));  // r1
    *--sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(a0));  // r0

    // Software frame: EXC_RETURN sentinel (highest address), then r11..r4.
    *--sp = 0;
    for (int r = 0; r < 8; ++r) {
        *--sp = 0;
    }
    return sp;
}

void Cm33Port::start_first_thread(void* sp, std::uintptr_t limit) noexcept {
    rtos_port_start_ctx[0] = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(sp));
    rtos_port_start_ctx[1] = static_cast<std::uint32_t>(limit);
    __asm volatile(
        // CONTROL.FPCA = 0: main()'s FP state is abandoned, the SVC frame is standard.
        "   mrs     r0, control     \n"
        "   bic     r0, r0, #4      \n"
        "   msr     control, r0     \n"
        "   isb                     \n"
        "   mov     r0, #0          \n"
        "   msr     basepri, r0     \n"
        "   cpsie   i               \n"
        "   dsb                     \n"
        "   isb                     \n"
        "   svc     #0              \n" ::
            : "r0", "memory");
    for (;;) {
    }  // not reached
}

void Cm33Port::configure(std::uint32_t core_clock_hz, std::uint32_t tick_hz) noexcept {
    // Dedicated fault handlers; in particular UsageFault reports PSPLIM stack
    // overflows (UFSR.STKOF) instead of escalating to HardFault.
    scb::SHCSR::set(scb::shcsr_memfaultena | scb::shcsr_busfaultena | scb::shcsr_usgfaultena);

    // PendSV and SysTick at the lowest priority, SVC most urgent (only used at start).
    scb::SHPR3::write((scb::SHPR3::read() & 0x0000'FFFFu) | (lowest_priority << 16) | (lowest_priority << 24));
    scb::SHPR2::write(scb::SHPR2::read() & 0x00FF'FFFFu);

#if defined(__ARM_FP)
    fpu::FPCCR::set(fpu::fpccr_aspen | fpu::fpccr_lspen);
#endif

    systick::CTRL::write(0);
    systick::LOAD::write(core_clock_hz / tick_hz - 1u);
    systick::VAL::write(0);
    systick::CTRL::write(systick::ctrl_clksource_cpu | systick::ctrl_tickint | systick::ctrl_enable);
}

}  // namespace rtos::port::cm33
