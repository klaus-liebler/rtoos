// SPDX-License-Identifier: MIT
#include "rtos/net/tls.hpp"

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssl/wolfio.h>

#include <algorithm>

namespace {

rtos::net::EntropySource g_entropy = nullptr;
rtos::net::SecondsSource g_seconds = nullptr;
rtos::net::CycleSource g_cycles = nullptr;
rtos::net::TlsStats g_stats{};

std::uint32_t cycles() noexcept { return g_cycles != nullptr ? g_cycles() : 0; }

}  // namespace

// Hooks required by user_settings.h (CUSTOM_RAND_GENERATE_SEED, USER_TICKS).
extern "C" int rtos_tls_seed(unsigned char* output, unsigned int size) {
    if (g_entropy == nullptr) {
        return -1;
    }
    return g_entropy({reinterpret_cast<std::byte*>(output), size}) ? 0 : -1;
}

extern "C" word32 LowResTimer(void) { return g_seconds != nullptr ? g_seconds() : 0; }

namespace rtos::net {

/// wolfSSL I/O callbacks: map the transport onto wolfSSL's non-blocking protocol.
struct TlsIo {
    static int receive(WOLFSSL*, char* buffer, int size, void* context) {
        auto& t = static_cast<TlsSession*>(context)->transport_;
        const std::size_t n = t.read(t.self, {reinterpret_cast<std::byte*>(buffer), static_cast<std::size_t>(size)});
        if (n != 0) {
            return static_cast<int>(n);
        }
        return t.eof(t.self) ? WOLFSSL_CBIO_ERR_CONN_CLOSE : WOLFSSL_CBIO_ERR_WANT_READ;
    }

    static int send(WOLFSSL*, char* buffer, int size, void* context) {
        auto& t = static_cast<TlsSession*>(context)->transport_;
        const std::size_t n =
            t.write(t.self, {reinterpret_cast<const std::byte*>(buffer), static_cast<std::size_t>(size)});
        return n != 0 ? static_cast<int>(n) : WOLFSSL_CBIO_ERR_WANT_WRITE;
    }
};

// ---- context --------------------------------------------------------------------------

bool TlsServerContext::initialize_library(EntropySource entropy, SecondsSource seconds, CycleSource cycle_source) noexcept {
    g_entropy = entropy;
    g_seconds = seconds;
    g_cycles = cycle_source;
    return wolfSSL_Init() == WOLFSSL_SUCCESS;
}

const TlsStats& TlsServerContext::stats() noexcept { return g_stats; }

TlsInitResult TlsServerContext::init(std::span<std::byte> memory, const TlsCredentials& credentials,
                                     int max_sessions) noexcept {
    WOLFSSL_CTX* ctx = nullptr;
    last_error_ = wolfSSL_CTX_load_static_memory(&ctx, wolfTLSv1_3_server_method_ex,
                                                 reinterpret_cast<unsigned char*>(memory.data()),
                                                 static_cast<unsigned int>(memory.size()), WOLFMEM_GENERAL, max_sessions);
    if (last_error_ != WOLFSSL_SUCCESS) {
        return TlsInitResult::StaticMemory;
    }
    ctx_ = ctx;
    // Some wolfSSL paths allocate without a heap hint (e.g. while parsing the
    // certificate); with WOLFSSL_NO_MALLOC those go to the global hint. Use the first
    // context's static pool for them.
    if (wolfSSL_GetGlobalHeapHint() == nullptr) {
        wolfSSL_SetGlobalHeapHint(wolfSSL_CTX_GetHeap(ctx, nullptr));
    }
    // X25519 first (browsers send a key share for it), P-256 as fallback.
    int groups[] = {WOLFSSL_ECC_X25519, WOLFSSL_ECC_SECP256R1};
    if ((last_error_ = wolfSSL_CTX_set_groups(ctx, groups, 2)) != WOLFSSL_SUCCESS) {
        return TlsInitResult::Groups;
    }
    if ((last_error_ = wolfSSL_CTX_set_cipher_list(ctx, "TLS13-CHACHA20-POLY1305-SHA256:TLS13-AES128-GCM-SHA256:"
                                                        "TLS13-AES256-GCM-SHA384")) != WOLFSSL_SUCCESS) {
        return TlsInitResult::CipherSuites;
    }
    if ((last_error_ = wolfSSL_CTX_use_certificate_buffer(
             ctx, reinterpret_cast<const unsigned char*>(credentials.certificate_der.data()),
             static_cast<long>(credentials.certificate_der.size()), WOLFSSL_FILETYPE_ASN1)) != WOLFSSL_SUCCESS) {
        return TlsInitResult::Certificate;
    }
    if ((last_error_ = wolfSSL_CTX_use_PrivateKey_buffer(
             ctx, reinterpret_cast<const unsigned char*>(credentials.private_key_der.data()),
             static_cast<long>(credentials.private_key_der.size()), WOLFSSL_FILETYPE_ASN1)) != WOLFSSL_SUCCESS) {
        return TlsInitResult::PrivateKey;
    }
    last_error_ = 0;
    return TlsInitResult::Ok;
}

TlsServerContext::~TlsServerContext() {
    if (ctx_ != nullptr) {
        wolfSSL_CTX_free(static_cast<WOLFSSL_CTX*>(ctx_));
    }
}

// ---- session --------------------------------------------------------------------------

bool TlsSession::open(TlsServerContext& context, const Transport& transport) noexcept {
    reset();
    WOLFSSL* ssl = wolfSSL_new(static_cast<WOLFSSL_CTX*>(context.native()));
    if (ssl == nullptr) {
        return false;  // static pool exhausted (too many sessions)
    }
    ssl_ = ssl;
    transport_ = transport;
    wolfSSL_SSLSetIORecv(ssl, &TlsIo::receive);
    wolfSSL_SSLSetIOSend(ssl, &TlsIo::send);
    wolfSSL_SetIOReadCtx(ssl, this);
    wolfSSL_SetIOWriteCtx(ssl, this);
    status_ = TlsStatus::Handshaking;
    advance();
    return status_ != TlsStatus::Failed;
}

void TlsSession::fail_on(int error) noexcept {
    switch (error) {
        case WOLFSSL_ERROR_WANT_READ:
        case WOLFSSL_ERROR_WANT_WRITE: return;
        case WOLFSSL_ERROR_ZERO_RETURN: status_ = TlsStatus::Closed; return;
        default: status_ = TlsStatus::Failed; return;
    }
}

TlsStatus TlsSession::advance() noexcept {
    if (status_ != TlsStatus::Handshaking) {
        return status_;
    }
    auto* ssl = static_cast<WOLFSSL*>(ssl_);
    const std::uint32_t t0 = cycles();
    const int result = wolfSSL_accept(ssl);
    handshake_cycles_ += cycles() - t0;
    if (result == WOLFSSL_SUCCESS) {
        status_ = TlsStatus::Open;
        g_stats.handshake_cycles = handshake_cycles_;
        ++g_stats.handshakes;
    } else {
        fail_on(wolfSSL_get_error(ssl, result));
    }
    return status_;
}

std::size_t TlsSession::read(std::span<std::byte> out) noexcept {
    if (advance() != TlsStatus::Open || out.empty()) {
        return 0;
    }
    auto* ssl = static_cast<WOLFSSL*>(ssl_);
    const std::uint32_t t0 = cycles();
    const int n = wolfSSL_read(ssl, out.data(), static_cast<int>(std::min<std::size_t>(out.size(), 0x7FFF'FFFF)));
    if (n > 0) {
        read_stamp_ = t0;
        g_stats.read.add(static_cast<std::size_t>(n), cycles() - t0);
        return static_cast<std::size_t>(n);
    }
    fail_on(wolfSSL_get_error(ssl, n));
    return 0;
}

bool TlsSession::flush() noexcept {
    if (pending_size_ == 0) {
        return true;
    }
    auto* ssl = static_cast<WOLFSSL*>(ssl_);
    const int n = wolfSSL_write(ssl, pending_data_, static_cast<int>(pending_size_));  // same data: wolfSSL resumes
    if (n > 0) {
        pending_data_ = nullptr;
        pending_size_ = 0;
        return true;
    }
    fail_on(wolfSSL_get_error(ssl, n));
    return false;
}

std::size_t TlsSession::write(std::span<const std::byte> data) noexcept {
    if (advance() != TlsStatus::Open || data.empty() || !flush()) {
        return 0;
    }
    auto* ssl = static_cast<WOLFSSL*>(ssl_);
    const std::size_t chunk = std::min(data.size(), max_write_chunk);
    const std::uint32_t t0 = cycles();
    const int n = wolfSSL_write(ssl, data.data(), static_cast<int>(chunk));
    const std::uint32_t spent = cycles() - t0;
    if (n > 0) {
        g_stats.write.add(static_cast<std::size_t>(n), spent);
        return static_cast<std::size_t>(n);
    }
    const int error = wolfSSL_get_error(ssl, n);
    if (error == WOLFSSL_ERROR_WANT_WRITE) {
        g_stats.write.add(chunk, spent);  // record built, the transport was full
        pending_data_ = data.data();  // record queued in wolfSSL: retry with the same bytes
        pending_size_ = chunk;
        return chunk;
    }
    fail_on(error);
    return 0;
}

void TlsSession::shutdown() noexcept {
    if (ssl_ != nullptr && (status_ == TlsStatus::Open || status_ == TlsStatus::Closed)) {
        wolfSSL_shutdown(static_cast<WOLFSSL*>(ssl_));
        status_ = TlsStatus::Closed;
    }
}

void TlsSession::reset() noexcept {
    if (ssl_ != nullptr) {
        wolfSSL_free(static_cast<WOLFSSL*>(ssl_));
        ssl_ = nullptr;
    }
    status_ = TlsStatus::Idle;
    pending_data_ = nullptr;
    pending_size_ = 0;
    handshake_cycles_ = 0;
}

const char* TlsSession::cipher_suite() const noexcept {
    return ssl_ != nullptr ? wolfSSL_get_cipher(static_cast<WOLFSSL*>(ssl_)) : "";
}

}  // namespace rtos::net
