// SPDX-License-Identifier: MIT
//
// User accounts: name, role and PBKDF2-HMAC-SHA256 password record (salt, iteration count, hash -
// never the password), and their store in the MCU's flash.
//
//   * The first account comes from provisioning: the builder generates a per-device one-time
//     password, prints it on the label and compiles only its record into the firmware
//     (initial_users()). It is marked must_change: the first login can do nothing but set a new
//     password. No two devices share a password (ETSI EN 303 645 5.1, CRA Annex I).
//   * CredentialStore keeps the table in two flash blocks, A/B: a save writes the block that is
//     not current (erase, record, CRC last) with a sequence number one higher, so a power loss
//     leaves the old or the new table, never half of one. load() takes the valid block with the
//     higher sequence number; with none (a new device) the caller starts from the provisioning
//     table. save() erases flash: call it from a thread that may block, never the network thread.
//   * PasswordCheck verifies a password in steps (pbkdf2.hpp: background work), PasswordSetter
//     hashes a new one with a fresh salt the same way.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <string_view>

#include "rtos/auth/access.hpp"
#include "rtos/auth/pbkdf2.hpp"
#include "rtos/auth/session.hpp"
#include "rtos/storage/flash_memory.hpp"

namespace rtos::auth {

inline constexpr std::size_t max_user_name = 15;
inline constexpr std::size_t min_password_chars = 10;  ///< new passwords (the label's are longer)

struct PasswordRecord {
    std::uint32_t iterations = 0;
    std::array<std::byte, salt_bytes> salt{};
    Hash hash{};
};

struct UserRecord {
    std::array<char, max_user_name + 1> name{};  ///< zero-terminated; empty: unused entry
    Role role = Role::None;
    std::uint8_t flags = 0;
    std::uint16_t reserved = 0;
    PasswordRecord password{};

    static constexpr std::uint8_t must_change = 1;  ///< one-time password: change at first login

    [[nodiscard]] bool used() const noexcept { return name[0] != '\0'; }
    [[nodiscard]] std::string_view user_name() const noexcept {
        return {name.data(), std::min(std::strlen(name.data()), max_user_name)};
    }
    [[nodiscard]] bool must_change_password() const noexcept { return (flags & must_change) != 0; }
};
static_assert(sizeof(UserRecord) == 16 + 4 + 4 + 16 + 32);

/// A valid user name: 1..15 of [a-z0-9._-].
[[nodiscard]] constexpr bool valid_user_name(std::string_view n) noexcept {
    if (n.empty() || n.size() > max_user_name) return false;
    for (char c : n) {
        if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '.' || c == '_' || c == '-')) return false;
    }
    return true;
}

/// Why a new password is refused (nullopt: acceptable).
[[nodiscard]] constexpr std::optional<std::string_view> password_problem(std::string_view pw, std::string_view user) noexcept {
    if (pw.size() < min_password_chars) return "too short (at least 10 characters)";
    if (pw.size() > max_password_bytes) return "too long (at most 64 bytes)";
    if (pw == user) return "equals the user name";
    for (char c : pw) {
        if (static_cast<unsigned char>(c) < 0x20 || c == 0x7F) return "contains control characters";
    }
    return std::nullopt;
}

template <std::size_t MaxUsers>
struct UserTable {
    std::array<UserRecord, MaxUsers> users{};

    [[nodiscard]] std::optional<std::uint8_t> find(std::string_view name) const noexcept {
        for (std::size_t i = 0; i < MaxUsers; ++i) {
            if (users[i].used() && users[i].user_name() == name) return static_cast<std::uint8_t>(i);
        }
        return std::nullopt;
    }
    [[nodiscard]] std::optional<std::uint8_t> free_slot() const noexcept {
        for (std::size_t i = 0; i < MaxUsers; ++i) {
            if (!users[i].used()) return static_cast<std::uint8_t>(i);
        }
        return std::nullopt;
    }
    [[nodiscard]] std::size_t admins() const noexcept {
        return static_cast<std::size_t>(std::ranges::count_if(users, [](const UserRecord& u) { return u.used() && u.role == Role::Admin; }));
    }
};

constexpr std::uint32_t crc32(std::span<const std::byte> data, std::uint32_t crc = 0) noexcept {
    crc = ~crc;
    for (std::byte b : data) {
        crc ^= std::to_integer<std::uint32_t>(b);
        for (int k = 0; k < 8; ++k) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}

template <storage::FlashMemory Flash, std::size_t MaxUsers>
class CredentialStore {
    static_assert(Flash::block_count >= 2, "two blocks: A and B");

public:
    using Table = UserTable<MaxUsers>;
    static constexpr std::uint32_t magic = 0x48545541;  // "AUTH"

    explicit CredentialStore(Flash& flash) noexcept : flash_(flash) {}

    /// The newest valid table into `out`; false if neither block holds one (`out` untouched).
    bool load(Table& out) noexcept {
        current_ = -1;
        for (int b = 0; b < 2; ++b) {
            Record r{};
            if (!read(b, r)) continue;
            if (current_ < 0 || static_cast<std::int32_t>(r.sequence - sequence_) > 0) {
                current_ = b;
                sequence_ = r.sequence;
                out = r.table;
            }
        }
        return current_ >= 0;
    }

    /// Writes `table` as the new current one (blocks: erases and programs the other block).
    bool save(const Table& table) noexcept {
        const int target = current_ == 0 ? 1 : 0;
        Record r{};
        r.magic = magic;
        r.sequence = current_ < 0 ? 1 : sequence_ + 1;
        r.table = table;
        r.crc = crc32(std::as_bytes(std::span(&r, 1)).first(offsetof(Record, crc)));
        if (!flash_.erase(static_cast<std::uint32_t>(target))) return false;
        // body first, the word with the CRC last: a record is valid only when it is complete
        const auto bytes = std::as_bytes(std::span(&r, 1));
        constexpr std::size_t tail = round_up(sizeof(std::uint32_t), Flash::program_bytes);
        constexpr std::size_t body = sizeof(Record) - tail;
        static_assert(sizeof(Record) % Flash::program_bytes == 0 && offsetof(Record, crc) >= body);
        if (!flash_.program(static_cast<std::uint32_t>(target), 0, bytes.first(body)) ||
            !flash_.program(static_cast<std::uint32_t>(target), body, bytes.subspan(body))) {
            return false;
        }
        Record check{};
        if (!read(target, check) || check.sequence != r.sequence) return false;
        current_ = target;
        sequence_ = r.sequence;
        return true;
    }

    [[nodiscard]] std::uint32_t sequence() const noexcept { return sequence_; }

private:
    static constexpr std::size_t round_up(std::size_t n, std::size_t to) noexcept { return (n + to - 1) / to * to; }

    struct Record {
        std::uint32_t magic;
        std::uint32_t sequence;
        Table table;
        std::array<std::byte, round_up(8 + sizeof(Table) + 4, Flash::program_bytes) - (8 + sizeof(Table) + 4)> pad;
        std::uint32_t crc;
    };
    static_assert(sizeof(Record) <= Flash::block_bytes);

    bool read(int block, Record& r) noexcept {
        if (!flash_.read(static_cast<std::uint32_t>(block), 0, std::as_writable_bytes(std::span(&r, 1)))) return false;
        return r.magic == magic && r.crc == crc32(std::as_bytes(std::span(&r, 1)).first(offsetof(Record, crc)));
    }

    Flash& flash_;
    int current_ = -1;
    std::uint32_t sequence_ = 0;
};

/// Verifies a password against a record in steps: start(), then step() until it returns false,
/// then matches().
class PasswordCheck {
public:
    bool start(std::string_view password, const PasswordRecord& record) noexcept {
        expected_ = record.hash;
        return kdf_.start(std::as_bytes(std::span(password)), record.salt, record.iterations);
    }
    bool step(std::uint32_t iterations) noexcept { return kdf_.step(iterations); }
    [[nodiscard]] bool done() const noexcept { return kdf_.done() || kdf_.failed(); }
    [[nodiscard]] bool matches() const noexcept {
        return kdf_.done() && !kdf_.failed() && equal_constant_time(kdf_.result(), expected_);
    }
    void clear() noexcept { kdf_.clear(); }

private:
    Pbkdf2Sha256 kdf_;
    Hash expected_{};
};

/// Hashes a new password in steps with a fresh random salt.
class PasswordSetter {
public:
    bool start(std::string_view password, std::uint32_t iterations, RandomSource random) noexcept {
        record_ = {};
        record_.iterations = iterations;
        if (random == nullptr || !random(record_.salt)) return false;
        return kdf_.start(std::as_bytes(std::span(password)), record_.salt, iterations);
    }
    bool step(std::uint32_t iterations) noexcept { return kdf_.step(iterations); }
    [[nodiscard]] bool done() const noexcept { return kdf_.done() || kdf_.failed(); }
    [[nodiscard]] std::optional<PasswordRecord> record() noexcept {
        if (!kdf_.done() || kdf_.failed()) return std::nullopt;
        record_.hash = kdf_.result();
        return record_;
    }
    void clear() noexcept {
        kdf_.clear();
        record_ = {};
    }

private:
    Pbkdf2Sha256 kdf_;
    PasswordRecord record_{};
};

}  // namespace rtos::auth
