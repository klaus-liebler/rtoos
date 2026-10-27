// SPDX-License-Identifier: MIT
//
// Deterministic mutation fuzzing support (fuzz_test.cpp, usb_fuzz_test.cpp): a
// seeded mutator and the iteration/seed overrides from the environment
// (RTOS_FUZZ_ITERATIONS, RTOS_FUZZ_SEED).
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <random>
#include <vector>

#include "rtos/net/bytes.hpp"

namespace fuzz {

using Bytes = std::vector<std::byte>;
using rtos::net::store_be16;

inline std::size_t iterations(std::size_t base) {
    if (const char* s = std::getenv("RTOS_FUZZ_ITERATIONS")) {
        return std::strtoul(s, nullptr, 10);
    }
    return base;
}

/// RTOS_FUZZ_SEED (environment) varies the mutation sequences.
inline std::uint32_t seed(std::uint32_t base) {
    if (const char* s = std::getenv("RTOS_FUZZ_SEED")) {
        return base ^ static_cast<std::uint32_t>(std::strtoul(s, nullptr, 0));
    }
    return base;
}

class Mutator {
public:
    explicit Mutator(std::uint32_t seed) : rng_(seed) {}

    std::uint32_t below(std::uint32_t n) { return n == 0 ? 0 : static_cast<std::uint32_t>(rng_() % n); }
    bool chance(std::uint32_t one_in) { return below(one_in) == 0; }

    /// 1-3 mutations; `keep` leading bytes stay untouched (e.g. the Ethernet header).
    void mutate(Bytes& b, std::size_t keep = 0, std::size_t max_size = 1514) {
        static constexpr std::uint8_t interesting8[] = {0x00, 0x01, 0x3F, 0x40, 0x7F, 0x80, 0xC0, 0xFF};
        static constexpr std::uint16_t interesting16[] = {0x0000, 0x0001, 0x00FF, 0x0100, 0x7FFF, 0x8000, 0xFFFF};
        for (std::uint32_t n = 1 + below(3); n != 0; --n) {
            if (b.size() <= keep) {
                b.resize(keep + 1 + below(16));
            }
            const std::size_t at = keep + below(static_cast<std::uint32_t>(b.size() - keep));
            switch (below(7)) {
                case 0: b[at] ^= static_cast<std::byte>(1u << below(8)); break;
                case 1: b[at] = std::byte{interesting8[below(std::size(interesting8))]}; break;
                case 2:
                    if (at + 1 < b.size()) {
                        store_be16(b.data() + at, interesting16[below(std::size(interesting16))]);
                    }
                    break;
                case 3: b.resize(keep + below(static_cast<std::uint32_t>(b.size() - keep))); break;
                case 4:
                    for (std::uint32_t k = 1 + below(64); k != 0 && b.size() < max_size; --k) {
                        b.push_back(static_cast<std::byte>(rng_()));
                    }
                    break;
                case 5: {  // copy a chunk within the message
                    const std::size_t from = keep + below(static_cast<std::uint32_t>(b.size() - keep));
                    const std::size_t len = std::min<std::size_t>({1 + below(16), b.size() - from, b.size() - at});
                    std::memmove(b.data() + at, b.data() + from, len);
                    break;
                }
                default: b[at] = static_cast<std::byte>(rng_()); break;
            }
        }
    }

private:
    std::mt19937 rng_;
};

}  // namespace fuzz
