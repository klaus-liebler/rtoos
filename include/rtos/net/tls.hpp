// SPDX-License-Identifier: MIT
//
// TLS 1.3 server sessions (wolfSSL) over any byte transport.
//
// TlsServerContext   certificate + private key + the static memory pool from which
//                    wolfSSL allocates everything (no heap; WOLFSSL_STATIC_MEMORY)
// TlsSession         one connection; non-blocking: every call does as much as the
//                    transport allows and reports "come back later" otherwise
//
// The transport is any type satisfying ByteTransport (rtos::net TCP connections,
// an in-memory pipe in the host tests). It is reached through two plain function
// pointers because wolfSSL's I/O callbacks are C callbacks.
//
// Records are wolfSSL's: write() turns each call into one record (up to
// max_write_chunk bytes) - a WebSocket frame passed in one call is one record, sent
// at once (the stack has no Nagle). Copies: the plaintext is copied into wolfSSL's
// record buffer and encrypted there, the ciphertext copied into TCP segments; on
// receive the reverse. A zero-copy record layer of our own was measured
// (apps/bench): the copies cost 3-6 % of the record CPU time - not worth owning
// security-critical record code. wolfSSL itself reports the same order of magnitude
// for its TLS 1.2 zero-copy send path (PR 10899: +1 % ChaCha20, +2.9 % AES-GCM).
#pragma once

#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos::net {

template <class T>
concept ByteTransport = requires(T& t, std::span<std::byte> in, std::span<const std::byte> out) {
    { t.read(in) } noexcept -> std::same_as<std::size_t>;
    { t.write(out) } noexcept -> std::same_as<std::size_t>;
    { t.eof() } noexcept -> std::same_as<bool>;
};

struct TlsCredentials {
    std::span<const std::byte> certificate_der;  ///< server certificate (X.509, DER)
    std::span<const std::byte> private_key_der;  ///< ECDSA P-256 key (DER)
};

/// Entropy for wolfSSL's DRBG seed: fill `out` completely, return true on success.
using EntropySource = bool (*)(std::span<std::byte> out) noexcept;

/// Seconds since an arbitrary epoch (wolfSSL's LowResTimer, handshake timeouts).
using SecondsSource = std::uint32_t (*)() noexcept;

/// Free-running CPU cycle counter (performance statistics; may be null).
using CycleSource = std::uint32_t (*)() noexcept;

/// Outcome of TlsServerContext::init(): the first step that failed.
enum class TlsInitResult : std::uint8_t {
    Ok,
    StaticMemory,  ///< pool too small / misaligned
    Groups,        ///< key exchange groups
    CipherSuites,
    Certificate,   ///< certificate DER not accepted
    PrivateKey,    ///< key DER not accepted / does not match the certificate
};

enum class TlsStatus : std::uint8_t {
    Idle,         ///< no session
    Handshaking,
    Open,
    Closed,       ///< close_notify received or sent
    Failed,       ///< fatal alert / protocol error
};

/// Cost of TLS calls as a straight line, cycles = fixed + per_byte * bytes, fitted
/// by least squares over every call. A plain cycles/bytes average would drift with
/// the mix of record sizes (10 Hz 39-byte status records are almost all fixed cost).
struct TlsCostModel {
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;

    void add(std::size_t bytes, std::uint32_t cycles) noexcept {
        const auto x = static_cast<double>(bytes);
        const auto y = static_cast<double>(cycles);
        n += 1;
        sx += x;
        sy += y;
        sxx += x * x;
        sxy += x * y;
    }
    /// Cycles per additional byte (0 until two different sizes were seen).
    [[nodiscard]] double per_byte() const noexcept {
        const double d = n * sxx - sx * sx;
        return d > 0 ? (n * sxy - sx * sy) / d : 0;
    }
    /// Cycles per call independent of its size (record header, tag, I/O).
    [[nodiscard]] double fixed() const noexcept { return n > 0 ? (sy - per_byte() * sx) / n : 0; }
};

/// Cycle counts of all sessions (CycleSource of initialize_library()).
struct TlsStats {
    std::uint32_t handshake_cycles = 0;  ///< CPU cycles of the last completed handshake
    std::uint32_t handshakes = 0;
    TlsCostModel write;  ///< wolfSSL_write(): copy + encrypt + copy to TCP, per record
    TlsCostModel read;   ///< wolfSSL_read() calls that returned data
};

/// Bucket layout of the static pool: block sizes (ascending) and how many blocks of each per
/// round; wolfSSL repeats the rounds until the memory is used up. Empty: wolfSSL's defaults.
struct TlsPoolLayout {
    std::span<const std::uint32_t> sizes;
    std::span<const std::uint32_t> counts;
};

class TlsServerContext {
public:
    TlsServerContext() noexcept = default;
    TlsServerContext(const TlsServerContext&) = delete;
    TlsServerContext& operator=(const TlsServerContext&) = delete;
    ~TlsServerContext();

    /// One-time library set-up. `entropy` seeds the DRBG; `seconds` drives timeouts;
    /// `cycles` (optional) feeds stats().
    static bool initialize_library(EntropySource entropy, SecondsSource seconds,
                                   CycleSource cycles = nullptr) noexcept;
    [[nodiscard]] static const TlsStats& stats() noexcept;

    /// `memory` (16-byte aligned) becomes wolfSSL's static pool for this context and
    /// all its sessions, divided into buckets per `layout`; at most `max_sessions` sessions
    /// are open at the same time.
    [[nodiscard]] TlsInitResult init(std::span<std::byte> memory, const TlsCredentials& credentials,
                                     int max_sessions, const TlsPoolLayout& layout = {}) noexcept;
    /// wolfSSL error code of the failed step (0 if none).
    [[nodiscard]] int last_error() const noexcept { return last_error_; }

    [[nodiscard]] void* native() const noexcept { return ctx_; }

private:
    void* ctx_ = nullptr;  // WOLFSSL_CTX*
    int last_error_ = 0;
};

class TlsSession {
public:
    /// Largest plaintext chunk per TLS record written by write(): bounds wolfSSL's
    /// record buffer (static pool bucket) and the latency of one record.
    static constexpr std::size_t max_write_chunk = 4096;

    TlsSession() noexcept = default;
    TlsSession(const TlsSession&) = delete;
    TlsSession& operator=(const TlsSession&) = delete;
    ~TlsSession() { reset(); }

    template <ByteTransport T>
    bool open(TlsServerContext& context, T& transport) noexcept {
        return open(context, Transport{&transport,
                                       [](void* t, std::span<std::byte> in) noexcept { return static_cast<T*>(t)->read(in); },
                                       [](void* t, std::span<const std::byte> out) noexcept {
                                           return static_cast<T*>(t)->write(out);
                                       },
                                       [](void* t) noexcept { return static_cast<T*>(t)->eof(); }});
    }

    /// Drive the handshake (call whenever the transport has data or room).
    TlsStatus advance() noexcept;

    /// Decrypted application data, at most out.size() bytes; 0 if none available now.
    std::size_t read(std::span<std::byte> out) noexcept;
    /// Cycle counter value when the last read() that returned data started.
    [[nodiscard]] std::uint32_t read_cycle_stamp() const noexcept { return read_stamp_; }

    /// Encrypt and send up to max_write_chunk bytes of `data` as one record. Returns
    /// the number of bytes consumed. If the transport is full, the record stays
    /// queued inside wolfSSL: the consumed bytes must stay valid and unchanged until
    /// output_pending() is false (flush() retries).
    std::size_t write(std::span<const std::byte> data) noexcept;

    /// Retry a queued record. Returns true if nothing is pending any more.
    bool flush() noexcept;
    [[nodiscard]] bool output_pending() const noexcept { return pending_size_ != 0; }

    /// Send close_notify (best effort) and keep the session until reset().
    void shutdown() noexcept;
    /// Free the wolfSSL session.
    void reset() noexcept;

    [[nodiscard]] TlsStatus status() const noexcept { return status_; }
    /// wolfSSL error that made the session fail (0 if none).
    [[nodiscard]] int last_error() const noexcept { return last_error_; }
    [[nodiscard]] const char* cipher_suite() const noexcept;

private:
    struct Transport {
        void* self = nullptr;
        std::size_t (*read)(void*, std::span<std::byte>) noexcept = nullptr;
        std::size_t (*write)(void*, std::span<const std::byte>) noexcept = nullptr;
        bool (*eof)(void*) noexcept = nullptr;
    };

    bool open(TlsServerContext& context, const Transport& transport) noexcept;
    void fail_on(int error) noexcept;
    friend struct TlsIo;  // wolfSSL I/O callbacks (tls.cpp)

    void* ssl_ = nullptr;  // WOLFSSL*
    Transport transport_{};
    TlsStatus status_ = TlsStatus::Idle;
    const std::byte* pending_data_ = nullptr;
    std::size_t pending_size_ = 0;
    std::uint32_t handshake_cycles_ = 0;
    std::uint32_t read_stamp_ = 0;
    int last_error_ = 0;
};

}  // namespace rtos::net
