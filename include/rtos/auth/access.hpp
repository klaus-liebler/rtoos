// SPDX-License-Identifier: MIT
//
// Roles, permissions and the physical-presence rule, independent of the protocol: the web UI's
// WebSocket messages use it today; OPC UA user tokens and SSH logins map their users to the same
// roles and ask the same question (decide()).
//
//   Observer    read: values, status, logs
//   Operator    + write registers, move actuators
//   Programmer  + PLC download, debugging, forcing, HMI pages
//   Admin       + users and passwords, certificates, firmware, reset
//
// A Permission names the least role and whether the action also needs physical presence: proof
// that the person is at the machine - a recent press of a button on the board, or a session that
// came in over the USB interface (only someone with a cable at the device can open one). Remote
// access alone must not change what the machine does in a way that can hurt someone (PLC
// download, forcing) or take the device over (firmware, certificates, password reset).
#pragma once

#include <cstdint>
#include <optional>
#include <string_view>

#include "rtos/time.hpp"

namespace rtos::auth {

enum class Role : std::uint8_t { None = 0, Observer = 1, Operator = 2, Programmer = 3, Admin = 4 };

[[nodiscard]] constexpr std::string_view role_name(Role r) noexcept {
    switch (r) {
        case Role::Observer: return "observer";
        case Role::Operator: return "operator";
        case Role::Programmer: return "programmer";
        case Role::Admin: return "admin";
        case Role::None: break;
    }
    return "none";
}

[[nodiscard]] constexpr std::optional<Role> parse_role(std::string_view s) noexcept {
    for (Role r : {Role::Observer, Role::Operator, Role::Programmer, Role::Admin}) {
        if (s == role_name(r)) {
            return r;
        }
    }
    return std::nullopt;
}

struct Permission {
    Role least = Role::Admin;
    bool presence = false;  ///< also needs physical presence
};

namespace permission {
inline constexpr Permission read{Role::Observer, false};
inline constexpr Permission operate{Role::Operator, false};
inline constexpr Permission program{Role::Programmer, false};
inline constexpr Permission program_critical{Role::Programmer, true};  ///< PLC download, forcing
inline constexpr Permission admin{Role::Admin, false};
inline constexpr Permission admin_critical{Role::Admin, true};  ///< firmware, certificates, password reset
}  // namespace permission

enum class Decision : std::uint8_t {
    Allowed,
    NotAuthenticated,  ///< no (valid) session
    RoleTooLow,
    PresenceRequired,
    PasswordChangeRequired,  ///< first login: nothing but the password change until it is done
};

[[nodiscard]] constexpr std::string_view decision_name(Decision d) noexcept {
    switch (d) {
        case Decision::Allowed: return "allowed";
        case Decision::NotAuthenticated: return "not authenticated";
        case Decision::RoleTooLow: return "role too low";
        case Decision::PresenceRequired: return "physical presence required";
        case Decision::PasswordChangeRequired: return "password change required";
    }
    return "?";
}

/// Who asks: the role of the session, whether it must change its password first, and whether it
/// is physically present (Presence::present()).
struct Subject {
    Role role = Role::None;
    bool must_change_password = false;
    bool present = false;
};

[[nodiscard]] constexpr Decision decide(const Subject& who, Permission need) noexcept {
    if (who.role == Role::None) {
        return Decision::NotAuthenticated;
    }
    if (who.must_change_password) {
        return Decision::PasswordChangeRequired;
    }
    if (static_cast<std::uint8_t>(who.role) < static_cast<std::uint8_t>(need.least)) {
        return Decision::RoleTooLow;
    }
    if (need.presence && !who.present) {
        return Decision::PresenceRequired;
    }
    return Decision::Allowed;
}

/// The button half of the presence rule: a press opens a window of `window_ms`.
class PresenceButton {
public:
    explicit constexpr PresenceButton(std::uint32_t window_ms = 120'000) noexcept : window_ms_(window_ms) {}

    void pressed(std::uint32_t now_ms) noexcept {
        last_ = now_ms;
        valid_ = true;
    }
    /// The window is open (a press less than window_ms ago).
    [[nodiscard]] bool recent(std::uint32_t now_ms) const noexcept {
        return valid_ && !rtos::reached(now_ms, last_ + window_ms_);
    }
    /// Milliseconds left in the window (0: closed).
    [[nodiscard]] std::uint32_t remaining_ms(std::uint32_t now_ms) const noexcept {
        return recent(now_ms) ? last_ + window_ms_ - now_ms : 0;
    }
    /// Closes the window (after a critical action, so one press covers one action).
    void consume() noexcept { valid_ = false; }

    /// Present: over the trusted (local) interface, or the button was pressed recently.
    [[nodiscard]] bool present(bool local_interface, std::uint32_t now_ms) const noexcept {
        return local_interface || recent(now_ms);
    }

private:
    std::uint32_t window_ms_;
    std::uint32_t last_ = 0;
    bool valid_ = false;
};

}  // namespace rtos::auth
