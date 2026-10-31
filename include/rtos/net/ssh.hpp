// SPDX-License-Identifier: MIT
//
// SSH server sessions on wolfSSH (the log terminal, log_terminal.hpp), the counterpart of
// tls.hpp: SshServerContext holds the host key, the authorized client keys and a static memory
// pool of its own; SshSession runs one connection over a ByteTransport (a TCP connection),
// non-blocking.
//
//   SshServerContext ssh;
//   ssh.init(ssh_memory, {.host_key_der = host_key, .authorized_keys = keys}, 2, layout);
//   session.open(ssh, connection);
//   session.advance();    // key exchange, authentication, channel: until Open
//   session.write(text);  // terminal output; session.read() terminal input
//
// Authentication: public keys only, the OpenSSH authorized_keys lines given at init
// ("ssh-ed25519 AAAA... comment", "ecdsa-sha2-nistp256 AAAA..."). wolfSSH checks the
// signature, SshServerContext the key. The host key is ECDSA P-256 (DER, SEC1 or PKCS#8).
//
// Like TLS, wolfSSL_Init and the entropy source must be set up first
// (TlsServerContext::initialize_library).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "rtos/net/tls.hpp"

namespace rtos::net {

// ---- authorized keys ------------------------------------------------------------------------

/// A client key as SSH puts it on the wire: the base64 part of an authorized_keys line.
struct SshPublicKey {
    static constexpr std::size_t max_blob = 128;  ///< ecdsa-sha2-nistp256: 104, ssh-ed25519: 51
    std::array<std::byte, max_blob> blob{};
    std::size_t size = 0;
    [[nodiscard]] std::span<const std::byte> bytes() const noexcept { return {blob.data(), size}; }
};

namespace ssh_detail {
constexpr int base64_value(char c) noexcept {
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'a' + 26;
    if (c >= '0' && c <= '9') return c - '0' + 52;
    if (c == '+') return 62;
    if (c == '/') return 63;
    return -1;
}
}  // namespace ssh_detail

/// Parse one authorized_keys line ("<type> <base64> [comment]"); nullopt if malformed, of an
/// unsupported type, or if the type inside the blob differs from the one in front.
constexpr std::optional<SshPublicKey> parse_authorized_key(std::string_view line) noexcept {
    const auto space = line.find(' ');
    if (space == std::string_view::npos) {
        return std::nullopt;
    }
    const std::string_view type = line.substr(0, space);
    if (type != "ssh-ed25519" && type != "ecdsa-sha2-nistp256") {
        return std::nullopt;
    }
    std::string_view b64 = line.substr(space + 1);
    b64 = b64.substr(0, b64.find(' '));
    SshPublicKey key;
    std::uint32_t acc = 0;
    int bits = 0;
    for (const char c : b64) {
        if (c == '=') {
            break;
        }
        const int v = ssh_detail::base64_value(c);
        if (v < 0) {
            return std::nullopt;
        }
        acc = (acc << 6) | static_cast<std::uint32_t>(v);
        bits += 6;
        if (bits >= 8) {
            bits -= 8;
            if (key.size == key.blob.size()) {
                return std::nullopt;
            }
            key.blob[key.size++] = static_cast<std::byte>(acc >> bits);
        }
    }
    // the blob starts with the type again: uint32 length, name
    if (key.size < 4 + type.size()) {
        return std::nullopt;
    }
    const std::uint32_t n = (std::to_integer<std::uint32_t>(key.blob[0]) << 24) |
                            (std::to_integer<std::uint32_t>(key.blob[1]) << 16) |
                            (std::to_integer<std::uint32_t>(key.blob[2]) << 8) | std::to_integer<std::uint32_t>(key.blob[3]);
    if (n != type.size()) {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (static_cast<char>(key.blob[4 + i]) != type[i]) {
            return std::nullopt;
        }
    }
    return key;
}

// ---- context ------------------------------------------------------------------------------

struct SshCredentials {
    std::span<const std::byte> host_key_der;           ///< ECDSA P-256 private key (DER)
    std::span<const std::string_view> authorized_keys;  ///< OpenSSH authorized_keys lines
};

enum class SshInitResult : std::uint8_t { Ok, Library, StaticMemory, Context, HostKey, AuthorizedKeys };

enum class SshStatus : std::uint8_t {
    Idle,
    Handshake,  ///< key exchange, authentication, channel setup
    Open,       ///< the terminal channel is open
    Closed,     ///< the client closed the channel or disconnected
    Failed,
};

struct SshStats {
    std::uint32_t sessions = 0;        ///< reached Open
    std::uint32_t rejected_keys = 0;   ///< authentication with a key not in the list
};

class SshServerContext {
public:
    static constexpr std::size_t max_authorized_keys = 8;

    SshServerContext() noexcept = default;
    SshServerContext(const SshServerContext&) = delete;
    SshServerContext& operator=(const SshServerContext&) = delete;

    /// `memory` (16-byte aligned) becomes the static pool of every SSH session; `layout` gives
    /// its buckets (tls_memory.hpp sizing), empty: wolfSSL's default buckets.
    [[nodiscard]] SshInitResult init(std::span<std::byte> memory, const SshCredentials& credentials,
                                     int max_sessions, const TlsPoolLayout& layout = {}) noexcept;

    /// True if `key` (wire blob) is one of the authorized keys.
    [[nodiscard]] bool authorized(std::span<const std::byte> key) const noexcept;

    [[nodiscard]] static const SshStats& stats() noexcept;
    [[nodiscard]] int last_error() const noexcept { return last_error_; }

private:
    friend class SshSession;
    friend struct SshIo;
    void* ctx_ = nullptr;  // WOLFSSH_CTX*
    std::array<SshPublicKey, max_authorized_keys> keys_{};
    std::size_t key_count_ = 0;
    int last_error_ = 0;
};

// ---- session ------------------------------------------------------------------------------

class SshSession {
public:
    SshSession() noexcept = default;
    SshSession(const SshSession&) = delete;
    SshSession& operator=(const SshSession&) = delete;
    ~SshSession() { reset(); }

    template <ByteTransport T>
    bool open(SshServerContext& context, T& transport) noexcept {
        return open(context, Transport{&transport,
                                       [](void* t, std::span<std::byte> in) noexcept { return static_cast<T*>(t)->read(in); },
                                       [](void* t, std::span<const std::byte> out) noexcept {
                                           return static_cast<T*>(t)->write(out);
                                       },
                                       [](void* t) noexcept { return static_cast<T*>(t)->eof(); }});
    }

    /// Drive the protocol: the handshake until Open, then channel traffic (window
    /// adjustments, the peer's close). Call whenever the transport has data or room.
    SshStatus advance() noexcept;

    /// Terminal input (keystrokes), at most out.size() bytes; 0 if none now.
    std::size_t read(std::span<std::byte> out) noexcept;
    /// Terminal output. Returns the bytes consumed (0: window or transport full: retry).
    std::size_t write(std::span<const std::byte> data) noexcept;
    /// Nothing queued inside wolfSSH: write() can take new data.
    [[nodiscard]] bool can_write() const noexcept;

    /// Disconnect (best effort) and keep the session until reset().
    void shutdown() noexcept;
    void reset() noexcept;

    [[nodiscard]] SshStatus status() const noexcept { return status_; }
    [[nodiscard]] int last_error() const noexcept { return last_error_; }
    [[nodiscard]] std::string_view user() const noexcept { return {user_.data(), user_size_}; }

private:
    struct Transport {
        void* self = nullptr;
        std::size_t (*read)(void*, std::span<std::byte>) noexcept = nullptr;
        std::size_t (*write)(void*, std::span<const std::byte>) noexcept = nullptr;
        bool (*eof)(void*) noexcept = nullptr;
    };
    bool open(SshServerContext& context, const Transport& transport) noexcept;
    void fail(int error) noexcept;
    friend struct SshIo;

    void* ssh_ = nullptr;  // WOLFSSH*
    SshServerContext* context_ = nullptr;
    Transport transport_{};
    SshStatus status_ = SshStatus::Idle;
    int last_error_ = 0;
    std::array<char, 32> user_{};
    std::size_t user_size_ = 0;
};

}  // namespace rtos::net
