// SPDX-License-Identifier: MIT
//
// Raw NOR flash as the storage layers need it (littlefs, the PLC's program slots): erase blocks,
// program units, read anywhere. Satisfied by the ports' flash::Region (STM32H5, STM32H7) and the
// RAM fakes of the host tests.
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos::storage {

template <class F>
concept FlashMemory = requires(F& f, std::uint32_t block, std::uint32_t offset, std::span<std::byte> in,
                               std::span<const std::byte> out) {
    { F::block_bytes } -> std::convertible_to<std::uint32_t>;    // erase unit
    { F::block_count } -> std::convertible_to<std::uint32_t>;
    { F::program_bytes } -> std::convertible_to<std::uint32_t>;  // program unit
    { f.read(block, offset, in) } noexcept -> std::same_as<bool>;
    { f.program(block, offset, out) } noexcept -> std::same_as<bool>;
    { f.erase(block) } noexcept -> std::same_as<bool>;
};

}  // namespace rtos::storage
