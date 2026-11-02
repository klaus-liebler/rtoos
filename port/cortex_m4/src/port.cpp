// SPDX-License-Identifier: MIT
//
// Cortex-M4 context switch.
//
// Saved context layout on the thread stack (growing down, lowest address = saved SP):
//
//   saved SP -> r4 r5 r6 r7 r8 r9 r10 r11 EXC_RETURN   (software frame, 9 words)
//               [s16 ... s31]                           (only if EXC_RETURN.FType == 0)
//               r0 r1 r2 r3 r12 lr pc xPSR              (hardware frame)
//               [s0 ... s15 FPSCR reserved]             (hardware, lazy, if FP active)
//
// A fresh thread stores EXC_RETURN = 0 (sentinel, cortex_m::CoreServices::init_stack) and is
// resumed with the EXC_RETURN template captured by the start-up SVC (Thread mode, PSP, no FP
// frame).
#include "rtos/port/cm4/port.hpp"

extern "C" {

std::uint32_t rtos_port_start_sp;
/// EXC_RETURN used for threads that have never run. Written once by SVC_Handler.
std::uint32_t rtos_port_exc_return_template;

void* rtos_port_switch(void* sp) noexcept;  // RTOS_CM4_BIND_KERNEL

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
        "   bl      rtos_port_switch                \n"  // r0 = next PSP
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
        "   movw    r2, #:lower16:rtos_port_start_sp \n"
        "   movt    r2, #:upper16:rtos_port_start_sp \n"
        "   ldr     r0, [r2]                        \n"  // r0 = PSP
        "   ldmia   r0!, {r4-r11}                   \n"
        "   adds    r0, r0, #4                      \n"  // skip EXC_RETURN sentinel
        // Caller was Thread mode on MSP: select PSP (bit 2) and a standard, FP-less
        // frame (bit 4) -> 0xFFFFFFFD.
        "   orr     lr, lr, #0x14                   \n"
        "   movw    r2, #:lower16:rtos_port_exc_return_template \n"
        "   movt    r2, #:upper16:rtos_port_exc_return_template \n"
        "   str     lr, [r2]                        \n"
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

namespace rtos::port::cm4 {

void Cm4Port::start_first_thread(void* sp, std::uintptr_t) noexcept {
    rtos_port_start_sp = static_cast<std::uint32_t>(reinterpret_cast<std::uintptr_t>(sp));
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

}  // namespace rtos::port::cm4
