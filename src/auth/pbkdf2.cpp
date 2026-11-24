// SPDX-License-Identifier: MIT
#include "rtos/auth/pbkdf2.hpp"

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/sha256.h>

#include <algorithm>
#include <cstring>

namespace rtos::auth {
namespace {

constexpr std::size_t block = 64;  // SHA-256 block

wc_Sha256* sha(std::array<std::byte, 192>& storage) noexcept { return reinterpret_cast<wc_Sha256*>(storage.data()); }

/// Copy the midstate `from` into `to`, hash `data`, write the digest to `out`.
bool finish_from(const wc_Sha256* from, wc_Sha256* to, std::span<const std::byte> data, std::byte* out) noexcept {
    return wc_Sha256Copy(const_cast<wc_Sha256*>(from), to) == 0 &&
           wc_Sha256Update(to, reinterpret_cast<const byte*>(data.data()), static_cast<word32>(data.size())) == 0 &&
           wc_Sha256Final(to, reinterpret_cast<byte*>(out)) == 0;
}

void wipe(void* p, std::size_t n) noexcept {
    auto* v = static_cast<volatile unsigned char*>(p);
    for (std::size_t i = 0; i < n; ++i) {
        v[i] = 0;
    }
}

}  // namespace

static_assert(sizeof(wc_Sha256) <= 192, "Pbkdf2Sha256::sha_state_bytes too small for wc_Sha256");

bool Pbkdf2Sha256::start(std::span<const std::byte> password, std::span<const std::byte> salt,
                         std::uint32_t iterations) noexcept {
    clear();
    if (password.empty() || password.size() > max_password_bytes || salt.size() > 64 || iterations == 0) {
        failed_ = true;
        return false;
    }
    // HMAC key pads (the password is at most 64 bytes: never hashed down first)
    std::array<std::byte, block> ipad{}, opad{};
    for (std::size_t i = 0; i < block; ++i) {
        const std::byte k = i < password.size() ? password[i] : std::byte{0};
        ipad[i] = k ^ std::byte{0x36};
        opad[i] = k ^ std::byte{0x5c};
    }
    bool ok = wc_InitSha256(sha(inner_)) == 0 && wc_InitSha256(sha(outer_)) == 0 &&
              wc_Sha256Update(sha(inner_), reinterpret_cast<const byte*>(ipad.data()), block) == 0 &&
              wc_Sha256Update(sha(outer_), reinterpret_cast<const byte*>(opad.data()), block) == 0;
    wipe(ipad.data(), ipad.size());
    wipe(opad.data(), opad.size());
    // U_1 = HMAC(P, S || INT(1))
    if (ok) {
        alignas(16) std::array<std::byte, 192> work{};
        std::array<std::byte, 64 + 4> s1{};
        std::copy(salt.begin(), salt.end(), s1.begin());
        s1[salt.size() + 3] = std::byte{1};
        std::array<std::byte, hash_bytes> inner{};
        ok = finish_from(sha(inner_), sha(work), std::span(s1).first(salt.size() + 4), inner.data()) &&
             finish_from(sha(outer_), sha(work), inner, u_.data());
        wipe(work.data(), work.size());
        wipe(inner.data(), inner.size());
    }
    if (!ok) {
        clear();
        failed_ = true;
        return false;
    }
    t_ = u_;
    left_ = iterations - 1;
    started_ = true;
    return true;
}

bool Pbkdf2Sha256::step(std::uint32_t max_iterations) noexcept {
    if (!started_ || failed_) {
        return false;
    }
    alignas(16) std::array<std::byte, 192> work{};
    std::array<std::byte, hash_bytes> inner{};
    const std::uint32_t n = std::min(left_, max_iterations);
    for (std::uint32_t i = 0; i < n; ++i) {
        // U_i = HMAC(P, U_{i-1})
        if (!finish_from(sha(inner_), sha(work), u_, inner.data()) || !finish_from(sha(outer_), sha(work), inner, u_.data())) {
            failed_ = true;
            left_ = 0;
            break;
        }
        for (std::size_t b = 0; b < hash_bytes; ++b) {
            t_[b] ^= u_[b];
        }
    }
    if (!failed_) {
        left_ -= n;
    }
    wipe(work.data(), work.size());
    wipe(inner.data(), inner.size());
    return left_ != 0;
}

void Pbkdf2Sha256::clear() noexcept {
    wipe(inner_.data(), inner_.size());
    wipe(outer_.data(), outer_.size());
    wipe(u_.data(), u_.size());
    left_ = 0;
    started_ = false;
    failed_ = false;
}

Hash Pbkdf2Sha256::derive(std::span<const std::byte> password, std::span<const std::byte> salt,
                          std::uint32_t iterations) noexcept {
    Pbkdf2Sha256 p;
    Hash out{};
    if (p.start(password, salt, iterations)) {
        while (p.step(4096)) {
        }
        if (p.done() && !p.failed()) {
            out = p.result();
        }
    }
    return out;
}

}  // namespace rtos::auth
