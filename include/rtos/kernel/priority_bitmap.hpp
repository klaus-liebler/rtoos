// SPDX-License-Identifier: MIT
//
// 32-level priority bitmap. Bit n set <=> priority level n is non-empty.
// `highest()` is a single count-trailing-zeros: on Cortex-M33 `RBIT` + `CLZ`,
// i.e. two cycles, independent of the number of threads -> O(1).
#pragma once

#include <bit>
#include <contracts>
#include <cstdint>

namespace rtos {

class PriorityBitmap {
public:
    static constexpr unsigned levels = 32;

    void set(unsigned prio) noexcept {
        contract_assert(prio < levels);
        bits_ |= 1u << prio;
    }
    void clear(unsigned prio) noexcept {
        contract_assert(prio < levels);
        bits_ &= ~(1u << prio);
    }
    [[nodiscard]] bool test(unsigned prio) const noexcept {
        contract_assert(prio < levels);
        return (bits_ >> prio) & 1u;
    }
    [[nodiscard]] bool empty() const noexcept { return bits_ == 0; }

    /// Most urgent (numerically lowest) set level; `levels` when empty.
    [[nodiscard]] unsigned highest() const noexcept {
        return static_cast<unsigned>(std::countr_zero(bits_));
    }

private:
    std::uint32_t bits_ = 0;
};

}  // namespace rtos
