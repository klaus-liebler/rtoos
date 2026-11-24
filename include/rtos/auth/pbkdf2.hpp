// SPDX-License-Identifier: MIT
//
// PBKDF2-HMAC-SHA256 (RFC 8018 5.2) with one output block (32 bytes), in steps: a login check
// costs 100-300 ms of CPU by design, so it must not run as one call in the network thread
// (service.hpp: background work in bounded units). step(n) runs n iterations and returns.
//
// Each iteration is two SHA-256 compressions: the HMAC's inner and outer key blocks are hashed
// once in start() and their midstates copied for every iteration (wolfSSL's HMAC would hash the
// key pads again each time: four compressions). The output is plain PBKDF2: the builder's
// System.Security.Cryptography.Rfc2898DeriveBytes and Python's hashlib.pbkdf2_hmac compute the
// same value (host test pbkdf2_test.cpp: the RFC 7914 vector and wolfCrypt's SHA-256).
//
// Password, salt and intermediate values are wiped by clear() and the destructor.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos::auth {

inline constexpr std::size_t hash_bytes = 32;  ///< derived key: one SHA-256 block
inline constexpr std::size_t salt_bytes = 16;
inline constexpr std::size_t max_password_bytes = 64;

using Hash = std::array<std::byte, hash_bytes>;

class Pbkdf2Sha256 {
public:
    Pbkdf2Sha256() noexcept = default;
    Pbkdf2Sha256(const Pbkdf2Sha256&) = delete;
    Pbkdf2Sha256& operator=(const Pbkdf2Sha256&) = delete;
    ~Pbkdf2Sha256() { clear(); }

    /// Begins a derivation; false if a parameter is out of range (empty or too long password,
    /// salt longer than 64 bytes, 0 iterations) or SHA-256 failed.
    bool start(std::span<const std::byte> password, std::span<const std::byte> salt,
               std::uint32_t iterations) noexcept;

    /// Runs up to `max_iterations` iterations. True while iterations are left.
    bool step(std::uint32_t max_iterations) noexcept;

    [[nodiscard]] bool done() const noexcept { return started_ && left_ == 0; }
    [[nodiscard]] bool failed() const noexcept { return failed_; }
    /// The derived key (valid when done()).
    [[nodiscard]] const Hash& result() const noexcept { return t_; }

    /// Wipes all secret state.
    void clear() noexcept;

    /// The whole derivation at once (provisioning tools, tests). nullopt-like: all zero on failure.
    static Hash derive(std::span<const std::byte> password, std::span<const std::byte> salt,
                       std::uint32_t iterations) noexcept;

private:
    // wolfCrypt's wc_Sha256 midstates (opaque here: the header stays free of wolfSSL's macros)
    static constexpr std::size_t sha_state_bytes = 192;
    alignas(16) std::array<std::byte, sha_state_bytes> inner_{};
    alignas(16) std::array<std::byte, sha_state_bytes> outer_{};
    Hash u_{};  ///< U_i
    Hash t_{};  ///< U_1 xor ... xor U_i
    std::uint32_t left_ = 0;
    bool started_ = false;
    bool failed_ = false;
};

/// Compare two byte strings in time independent of their content (not of their length).
[[nodiscard]] constexpr bool equal_constant_time(std::span<const std::byte> a, std::span<const std::byte> b) noexcept {
    if (a.size() != b.size()) {
        return false;
    }
    std::byte d{0};
    for (std::size_t i = 0; i < a.size(); ++i) {
        d |= a[i] ^ b[i];
    }
    return d == std::byte{0};
}

}  // namespace rtos::auth
