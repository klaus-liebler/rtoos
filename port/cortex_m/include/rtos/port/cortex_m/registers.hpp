// SPDX-License-Identifier: MIT
//
// Minimal, type-safe access to the System Control Space registers that ARMv7-M (Cortex-M4) and
// ARMv8-M Mainline (Cortex-M33) share: same addresses, same bits (ARMv7-M and ARMv8-M
// Architecture Reference Manuals). Only what the kernel ports and the drivers need; what one
// architecture adds (ARMv8-M: stack limit fault, the PMSAv8 MPU) lives in its port.
#pragma once

#include <cstdint>

namespace rtos::port::cortex_m {

/// Memory-mapped 32-bit register at a fixed address.
template <std::uintptr_t Address>
struct Reg {
    static volatile std::uint32_t& ref() noexcept { return *reinterpret_cast<volatile std::uint32_t*>(Address); }
    static std::uint32_t read() noexcept { return ref(); }
    static void write(std::uint32_t v) noexcept { ref() = v; }
    static void set(std::uint32_t mask) noexcept { ref() = ref() | mask; }
    static void clear(std::uint32_t mask) noexcept { ref() = ref() & ~mask; }
};

namespace scb {
using ICSR = Reg<0xE000'ED04>;   // Interrupt Control and State
using VTOR = Reg<0xE000'ED08>;   // Vector Table Offset
using AIRCR = Reg<0xE000'ED0C>;  // Application Interrupt and Reset Control
using SHPR2 = Reg<0xE000'ED1C>;  // SVCall priority [31:24]
using SHPR3 = Reg<0xE000'ED20>;  // PendSV [23:16], SysTick [31:24]
using SHCSR = Reg<0xE000'ED24>;  // System handler control and state
using CFSR = Reg<0xE000'ED28>;   // Configurable fault status (MMFSR|BFSR|UFSR)
using HFSR = Reg<0xE000'ED2C>;   // HardFault status
using CPACR = Reg<0xE000'ED88>;  // Coprocessor access (CP10/CP11 = FPU)
inline constexpr std::uint32_t icsr_pendsvset = 1u << 28;
inline constexpr std::uint32_t aircr_vectkey = 0x05FAu << 16;    // write key
inline constexpr std::uint32_t aircr_prigroup_mask = 0x7u << 8;  // keep on write
inline constexpr std::uint32_t aircr_sysresetreq = 1u << 2;
inline constexpr std::uint32_t shcsr_memfaultena = 1u << 16;
inline constexpr std::uint32_t shcsr_busfaultena = 1u << 17;
inline constexpr std::uint32_t shcsr_usgfaultena = 1u << 18;
}  // namespace scb

namespace nvic {
inline volatile std::uint8_t& ipr(unsigned irq) noexcept {
    return *reinterpret_cast<volatile std::uint8_t*>(0xE000'E400 + irq);
}
inline void enable(unsigned irq) noexcept {
    *reinterpret_cast<volatile std::uint32_t*>(0xE000'E100 + 4 * (irq / 32)) = 1u << (irq % 32);
}
/// Software-trigger an interrupt (set its pending bit).
inline void pend(unsigned irq) noexcept {
    *reinterpret_cast<volatile std::uint32_t*>(0xE000'E200 + 4 * (irq / 32)) = 1u << (irq % 32);
}
}  // namespace nvic

namespace systick {
using CTRL = Reg<0xE000'E010>;
using LOAD = Reg<0xE000'E014>;
using VAL = Reg<0xE000'E018>;
inline constexpr std::uint32_t ctrl_enable = 1u << 0;
inline constexpr std::uint32_t ctrl_tickint = 1u << 1;
inline constexpr std::uint32_t ctrl_clksource_cpu = 1u << 2;
}  // namespace systick

namespace dcb {
using DEMCR = Reg<0xE000'EDFC>;  // Debug Exception and Monitor Control
inline constexpr std::uint32_t demcr_trcena = 1u << 24;  // enables DWT and ITM
}  // namespace dcb

namespace dwt {
using CTRL = Reg<0xE000'1000>;
using CYCCNT = Reg<0xE000'1004>;
inline constexpr std::uint32_t ctrl_cyccntena = 1u << 0;
inline constexpr std::uint32_t ctrl_nocyccnt = 1u << 25;  // RO: no cycle counter implemented
}  // namespace dwt

namespace fpu {
using FPCCR = Reg<0xE000'EF34>;  // Floating-point context control
inline constexpr std::uint32_t fpccr_aspen = 1u << 31;  // automatic FP state preservation
inline constexpr std::uint32_t fpccr_lspen = 1u << 30;  // lazy FP state preservation
}  // namespace fpu

}  // namespace rtos::port::cortex_m
