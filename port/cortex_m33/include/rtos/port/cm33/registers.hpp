// SPDX-License-Identifier: MIT
//
// The ARMv8-M System Control Space: the registers all Cortex-M cores share (cortex_m/registers.hpp,
// reachable here as cm33::scb, cm33::nvic, ...) and what ARMv8-M adds - the stack limit fault
// and the PMSAv8 MPU. Addresses from the ARMv8-M Architecture Reference Manual.
#pragma once

#include <cstdint>

#include "rtos/port/cortex_m/registers.hpp"

namespace rtos::port::cm33 {

using namespace cortex_m;

namespace scb {
using namespace cortex_m::scb;
inline constexpr std::uint32_t cfsr_stkof = 1u << 20;  // UFSR.STKOF: PSPLIM/MSPLIM violation
}  // namespace scb

namespace mpu {
using TYPE = Reg<0xE000'ED90>;
using CTRL = Reg<0xE000'ED94>;
using RNR = Reg<0xE000'ED98>;
using RBAR = Reg<0xE000'ED9C>;
using RLAR = Reg<0xE000'EDA0>;
using MAIR0 = Reg<0xE000'EDC0>;
using MAIR1 = Reg<0xE000'EDC4>;
inline constexpr std::uint32_t ctrl_enable = 1u << 0;
inline constexpr std::uint32_t ctrl_hfnmiena = 1u << 1;
inline constexpr std::uint32_t ctrl_privdefena = 1u << 2;
}  // namespace mpu

}  // namespace rtos::port::cm33
