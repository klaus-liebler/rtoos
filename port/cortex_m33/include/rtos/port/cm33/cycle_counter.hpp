// SPDX-License-Identifier: MIT
//
// CPU cycle counter (DWT_CYCCNT): 32 bit, counts core clock cycles, wraps after
// 2^32 cycles (17 s at 250 MHz). Differences of two now() values are exact as long
// as the interval is shorter than that (unsigned wrap-around arithmetic).
#pragma once

#include <cstdint>

#include "rtos/port/cm33/registers.hpp"

namespace rtos::port::cm33 {

struct CycleCounter {
    /// Start counting from 0. Returns false if the core has no cycle counter.
    static bool enable() noexcept {
        dcb::DEMCR::set(dcb::demcr_trcena);
        if ((dwt::CTRL::read() & dwt::ctrl_nocyccnt) != 0) {
            return false;
        }
        dwt::CYCCNT::write(0);
        dwt::CTRL::set(dwt::ctrl_cyccntena);
        return true;
    }

    [[nodiscard]] static std::uint32_t now() noexcept { return dwt::CYCCNT::read(); }
};

}  // namespace rtos::port::cm33
