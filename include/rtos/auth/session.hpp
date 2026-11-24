// SPDX-License-Identifier: MIT
//
// Login sessions of a web server: after a successful login the device issues a random token
// (256 bits from the RNG, base64url, 43 characters) that the browser keeps in an HttpOnly cookie.
// Every later request presents it; the table finds the session in constant time per entry, checks
// its inactivity and lifetime limits and refreshes the inactivity timer.
//
//   * A login always creates a NEW token (session fixation: a token the client brings is never
//     promoted), and the caller ends the session the client presented, if any (logout semantics).
//   * Full table: the least recently used session is dropped.
//   * Handle {slot, generation}: a long-lived binding (an open WebSocket) refers to its session
//     without keeping the token; a slot reused for another login has another generation, so a
//     stale binding never inherits someone else's session.
//   * Tokens are compared in constant time; removing a session wipes it.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "rtos/auth/access.hpp"
#include "rtos/time.hpp"

namespace rtos::auth {

/// Fill `out` with cryptographically strong random bytes; false on failure.
using RandomSource = bool (*)(std::span<std::byte> out) noexcept;

inline constexpr std::size_t token_bytes = 32;
inline constexpr std::size_t token_chars = 43;  ///< base64url without padding
using Token = std::array<char, token_chars>;

struct SessionPolicy {
    std::uint32_t idle_ms = 30 * 60'000;            ///< inactivity timeout
    std::uint32_t lifetime_ms = 12 * 3600'000;      ///< absolute limit (log in again)
};

struct SessionHandle {
    std::uint8_t slot = 0xFF;
    std::uint8_t generation = 0;
    [[nodiscard]] constexpr bool valid() const noexcept { return slot != 0xFF; }
    friend constexpr bool operator==(SessionHandle, SessionHandle) = default;
};

struct SessionInfo {
    std::uint8_t user = 0;  ///< index in the user table
    Role role = Role::None;
    bool must_change_password = false;
    bool local = false;  ///< logged in over the trusted (USB) interface
    std::uint32_t created_ms = 0;
    std::uint32_t last_used_ms = 0;
};

/// RFC 4648 base64url without padding.
constexpr void base64url(std::span<const std::byte> in, std::span<char> out) noexcept {
    constexpr std::string_view alphabet = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
    std::size_t o = 0;
    std::uint32_t acc = 0;
    int bits = 0;
    for (std::byte b : in) {
        acc = (acc << 8) | std::to_integer<std::uint32_t>(b);
        bits += 8;
        while (bits >= 6 && o < out.size()) {
            bits -= 6;
            out[o++] = alphabet[(acc >> bits) & 63];
        }
    }
    if (bits > 0 && o < out.size()) {
        out[o++] = alphabet[(acc << (6 - bits)) & 63];
    }
}

/// The value of cookie `name` in a Cookie header ("a=1; rtoos_session=xyz").
[[nodiscard]] constexpr std::optional<std::string_view> cookie_value(std::string_view header, std::string_view name) noexcept {
    while (!header.empty()) {
        const std::size_t semi = header.find(';');
        std::string_view item = header.substr(0, semi);
        while (!item.empty() && item.front() == ' ') {
            item.remove_prefix(1);
        }
        if (item.size() > name.size() && item.starts_with(name) && item[name.size()] == '=') {
            return item.substr(name.size() + 1);
        }
        if (semi == std::string_view::npos) {
            break;
        }
        header.remove_prefix(semi + 1);
    }
    return std::nullopt;
}

template <std::size_t N>
class SessionTable {
    static_assert(N >= 1 && N < 0xFF);

public:
    explicit SessionTable(RandomSource random, const SessionPolicy& policy = {}) noexcept
        : random_(random), policy_(policy) {}

    /// A new session; writes its token into `token`. nullopt if the RNG failed.
    std::optional<SessionHandle> create(const SessionInfo& info, std::uint32_t now_ms, Token& token) noexcept {
        std::array<std::byte, token_bytes> raw{};
        if (random_ == nullptr || !random_(raw)) {
            return std::nullopt;
        }
        std::size_t slot = N;
        for (std::size_t i = 0; i < N; ++i) {
            if (!slots_[i].used) {
                slot = i;
                break;
            }
            if (slot == N || rtos::earlier(slots_[i].info.last_used_ms, slots_[slot].info.last_used_ms)) {
                slot = i;  // least recently used, unless a free slot turns up
            }
        }
        Slot& s = slots_[slot];
        wipe(s);
        s.used = true;
        ++s.generation;
        base64url(raw, s.token);
        for (auto& b : raw) {
            b = std::byte{0};
        }
        s.info = info;
        s.info.created_ms = s.info.last_used_ms = now_ms;
        token = s.token;
        return SessionHandle{static_cast<std::uint8_t>(slot), s.generation};
    }

    /// The session of `token` if it is valid at `now_ms` (and refreshes its inactivity timer).
    std::optional<SessionHandle> find(std::string_view token, std::uint32_t now_ms) noexcept {
        if (token.size() != token_chars) {
            return std::nullopt;
        }
        std::optional<SessionHandle> found;
        for (std::size_t i = 0; i < N; ++i) {
            // every slot is compared, so the time does not tell which one matched
            const bool match = slots_[i].used && equal(slots_[i].token, token);
            if (match) {
                found = SessionHandle{static_cast<std::uint8_t>(i), slots_[i].generation};
            }
        }
        if (found && !touch(*found, now_ms)) {
            return std::nullopt;
        }
        return found;
    }

    /// The session behind `h` is still the same and valid; refreshes its inactivity timer.
    bool touch(SessionHandle h, std::uint32_t now_ms) noexcept {
        Slot* s = slot(h);
        if (s == nullptr) {
            return false;
        }
        if (expired(*s, now_ms)) {
            wipe(*s);
            return false;
        }
        s->info.last_used_ms = now_ms;
        return true;
    }

    /// Valid at `now_ms`, without refreshing.
    [[nodiscard]] bool alive(SessionHandle h, std::uint32_t now_ms) const noexcept {
        const Slot* s = slot(h);
        return s != nullptr && !expired(*s, now_ms);
    }

    [[nodiscard]] const SessionInfo* info(SessionHandle h) const noexcept {
        const Slot* s = slot(h);
        return s != nullptr ? &s->info : nullptr;
    }
    SessionInfo* info(SessionHandle h) noexcept {
        Slot* s = slot(h);
        return s != nullptr ? &s->info : nullptr;
    }

    void remove(SessionHandle h) noexcept {
        if (Slot* s = slot(h)) {
            wipe(*s);
        }
    }

    /// Ends every session of `user` (password changed, user deleted, role changed).
    std::size_t remove_user(std::uint8_t user, SessionHandle except = {}) noexcept {
        std::size_t n = 0;
        for (std::size_t i = 0; i < N; ++i) {
            if (slots_[i].used && slots_[i].info.user == user &&
                !(except.slot == i && except.generation == slots_[i].generation)) {
                wipe(slots_[i]);
                ++n;
            }
        }
        return n;
    }

    /// Drops expired sessions (call now and then; find()/touch() also do it on the way).
    void expire(std::uint32_t now_ms) noexcept {
        for (auto& s : slots_) {
            if (s.used && expired(s, now_ms)) {
                wipe(s);
            }
        }
    }

    [[nodiscard]] std::size_t count() const noexcept {
        std::size_t n = 0;
        for (const auto& s : slots_) {
            n += s.used;
        }
        return n;
    }
    [[nodiscard]] const SessionPolicy& policy() const noexcept { return policy_; }

private:
    struct Slot {
        Token token{};
        SessionInfo info{};
        std::uint8_t generation = 0;
        bool used = false;
    };

    static bool equal(const Token& a, std::string_view b) noexcept {
        unsigned char d = 0;
        for (std::size_t i = 0; i < token_chars; ++i) {
            d |= static_cast<unsigned char>(a[i] ^ b[i]);
        }
        return d == 0;
    }
    [[nodiscard]] bool expired(const Slot& s, std::uint32_t now_ms) const noexcept {
        return rtos::reached(now_ms, s.info.last_used_ms + policy_.idle_ms) ||
               rtos::reached(now_ms, s.info.created_ms + policy_.lifetime_ms);
    }
    static void wipe(Slot& s) noexcept {
        volatile char* t = s.token.data();
        for (std::size_t i = 0; i < token_chars; ++i) {
            t[i] = 0;
        }
        s.info = {};
        s.used = false;
    }
    Slot* slot(SessionHandle h) noexcept {
        return h.slot < N && slots_[h.slot].used && slots_[h.slot].generation == h.generation ? &slots_[h.slot] : nullptr;
    }
    const Slot* slot(SessionHandle h) const noexcept {
        return h.slot < N && slots_[h.slot].used && slots_[h.slot].generation == h.generation ? &slots_[h.slot] : nullptr;
    }

    RandomSource random_;
    SessionPolicy policy_;
    std::array<Slot, N> slots_{};
};

}  // namespace rtos::auth
