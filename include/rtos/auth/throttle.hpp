// SPDX-License-Identifier: MIT
//
// Login throttling against password guessing, without ever blocking the server: a refused
// attempt is answered at once ("429, try again in n s"), nothing waits.
//
//   per client address  the first `free_failures` failures cost nothing; then every failure
//                       doubles the wait (base_delay_ms, 2x, 4x ... up to max_delay_ms); after
//                       `lockout_failures` the address is locked out for lockout_ms. A success
//                       clears the address.
//   global              more than `global_failures` failures within `global_window_ms` (many
//                       addresses: a distributed attack) lock every remote login for
//                       global_lockout_ms. Logins over the trusted local interface (USB) are
//                       exempt from the global lock, so an attacker on the LAN cannot keep the
//                       owner out - they are still throttled per address.
//
// The table holds `N` addresses; a new one replaces the entry that changed least recently (an
// attacker with many addresses is bounded by the global limit, not by the table).
// Checking is wait_ms() before the (expensive) password check, recording is failure()/success()
// after it: the throttle also caps the CPU an attacker can burn on PBKDF2.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/time.hpp"

namespace rtos::auth {

struct ThrottlePolicy {
    std::uint8_t free_failures = 3;
    std::uint32_t base_delay_ms = 1'000;
    std::uint32_t max_delay_ms = 60'000;
    std::uint8_t lockout_failures = 10;
    std::uint32_t lockout_ms = 5 * 60'000;
    std::uint16_t global_failures = 30;
    std::uint32_t global_window_ms = 60'000;
    std::uint32_t global_lockout_ms = 60'000;
};

template <std::size_t N>
class LoginThrottle {
public:
    explicit constexpr LoginThrottle(const ThrottlePolicy& policy = {}) noexcept : policy_(policy) {}

    /// Milliseconds until `address` may try again (0: now). `local`: the trusted interface.
    [[nodiscard]] std::uint32_t wait_ms(std::uint32_t address, bool local, std::uint32_t now_ms) const noexcept {
        std::uint32_t wait = 0;
        if (!local && global_locked_ && !rtos::reached(now_ms, global_until_)) {
            wait = global_until_ - now_ms;
        }
        if (const Entry* e = find(address); e != nullptr && !rtos::reached(now_ms, e->until)) {
            wait = std::max(wait, e->until - now_ms);
        }
        return wait;
    }

    void failure(std::uint32_t address, bool local, std::uint32_t now_ms) noexcept {
        Entry& e = entry(address, now_ms);
        e.failures = static_cast<std::uint8_t>(std::min<unsigned>(e.failures + 1u, 255u));
        e.changed = now_ms;
        if (e.failures >= policy_.lockout_failures) {
            e.until = now_ms + policy_.lockout_ms;
        } else if (e.failures > policy_.free_failures) {
            const unsigned shift = std::min<unsigned>(e.failures - policy_.free_failures - 1u, 16u);
            e.until = now_ms + std::min<std::uint32_t>(policy_.max_delay_ms, policy_.base_delay_ms << shift);
        } else {
            e.until = now_ms;
        }
        ++total_failures_;
        if (local) {
            return;  // the owner at the device does not count against everyone else
        }
        if (rtos::reached(now_ms, window_start_ + policy_.global_window_ms)) {
            window_start_ = now_ms;
            window_failures_ = 0;
        }
        if (++window_failures_ > policy_.global_failures) {
            global_locked_ = true;
            global_until_ = now_ms + policy_.global_lockout_ms;
            ++global_lockouts_;
            window_failures_ = 0;
            window_start_ = now_ms;
        }
    }

    void success(std::uint32_t address) noexcept {
        for (auto& e : entries_) {
            if (e.used && e.address == address) {
                e = {};
            }
        }
    }

    [[nodiscard]] std::uint32_t failures(std::uint32_t address) const noexcept {
        const Entry* e = find(address);
        return e != nullptr ? e->failures : 0;
    }
    [[nodiscard]] std::uint32_t total_failures() const noexcept { return total_failures_; }
    [[nodiscard]] std::uint32_t global_lockouts() const noexcept { return global_lockouts_; }

private:
    struct Entry {
        std::uint32_t address = 0;
        std::uint32_t until = 0;
        std::uint32_t changed = 0;
        std::uint8_t failures = 0;
        bool used = false;
    };

    const Entry* find(std::uint32_t address) const noexcept {
        for (const auto& e : entries_) {
            if (e.used && e.address == address) {
                return &e;
            }
        }
        return nullptr;
    }

    Entry& entry(std::uint32_t address, std::uint32_t now_ms) noexcept {
        Entry* victim = nullptr;
        for (auto& e : entries_) {
            if (e.used && e.address == address) {
                return e;
            }
            if (!e.used) {
                if (victim == nullptr || victim->used) victim = &e;
            } else if (victim == nullptr || (victim->used && rtos::earlier(e.changed, victim->changed))) {
                victim = &e;
            }
        }
        *victim = {.address = address, .until = now_ms, .changed = now_ms, .failures = 0, .used = true};
        return *victim;
    }

    ThrottlePolicy policy_;
    std::array<Entry, N> entries_{};
    std::uint32_t window_start_ = 0;
    std::uint32_t window_failures_ = 0;
    std::uint32_t global_until_ = 0;
    bool global_locked_ = false;
    std::uint32_t total_failures_ = 0;
    std::uint32_t global_lockouts_ = 0;
};

}  // namespace rtos::auth
