// SPDX-License-Identifier: MIT
//
// ARM Cortex-M7 (ARMv7E-M, FPv5-D16) implementation of rtos::CpuPort.
//
// The M7 runs the ARMv7-M port of the Cortex-M4 unchanged (cm4/port.hpp, cortex_m4/src/port.cpp):
// the same exception model, BASEPRI kernel lock, PendSV context switch and lazy FPU stacking.
// The double-precision FPU does not change the frame: d0-d15 are s0-s31, the context switch
// saves s16-s31 (= d8-d15) as on the M4. No stack limit register (ARMv7-M): the kernel's
// stack check at every context switch, as on the M4.
//
// ARM erratum 837070 (a BASEPRI raise takes effect one instruction late) affects Cortex-M7
// r0p0/r0p1 only; the STM32H743 has r1p1 (ST's stm32h743xx.h, __CM7_REV 0x0101), so the
// kernel lock needs no CPSID around MSR BASEPRI.
//
// What the M7 adds: the L1 caches (cm7/cache.hpp).
#pragma once

#include "rtos/port/cm4/port.hpp"

namespace rtos::port::cm7 {

// The core registers and the cycle counter of the Cortex-M family (cm7::nvic, cm7::CycleCounter, ...).
using namespace cortex_m;

using Core = cm4::Core;
inline constexpr std::uint32_t kernel_basepri = cm4::kernel_basepri;
inline constexpr std::uint32_t lowest_priority = cm4::lowest_priority;
using Cm7Port = cm4::Cm4Port;

static_assert(rtos::CpuPort<Cm7Port>);

}  // namespace rtos::port::cm7

/// Connects one kernel instance to the port's exception handlers (RTOS_CM4_BIND_KERNEL).
#define RTOS_CM7_BIND_KERNEL(kernel_object) RTOS_CM4_BIND_KERNEL(kernel_object)
