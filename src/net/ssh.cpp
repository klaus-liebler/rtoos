// SPDX-License-Identifier: MIT
#include "rtos/net/ssh.hpp"

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssh/error.h>
#include <wolfssh/ssh.h>

#include <algorithm>
#include <cstring>

namespace {
rtos::net::SshStats g_stats{};
}  // namespace

#if defined(__arm__)
// wolfSSH's diagnostics print nothing on the target (cmake/wolfssh.cmake).
extern "C" int rtos_wolfssh_printf(const char*, ...) { return 0; }
extern "C" int rtos_wolfssh_fprintf(void*, const char*, ...) { return 0; }
#endif

namespace rtos::net {

/// wolfSSH callbacks: transport I/O and the public key check.
struct SshIo {
    static int receive(WOLFSSH*, void* buffer, word32 size, void* context) {
        auto& t = static_cast<SshSession*>(context)->transport_;
        const std::size_t n = t.read(t.self, {static_cast<std::byte*>(buffer), size});
        if (n != 0) {
            return static_cast<int>(n);
        }
        return t.eof(t.self) ? WS_CBIO_ERR_CONN_CLOSE : WS_CBIO_ERR_WANT_READ;
    }
    static int send(WOLFSSH*, void* buffer, word32 size, void* context) {
        auto& t = static_cast<SshSession*>(context)->transport_;
        const std::size_t n = t.write(t.self, {static_cast<const std::byte*>(buffer), size});
        return n != 0 ? static_cast<int>(n) : WS_CBIO_ERR_WANT_WRITE;
    }

    /// Public keys only, and only the authorized ones (wolfSSH verifies the signature).
    /// WOLFSSH_USERAUTH_SUCCESS is 0: every other path must return a failure.
    static int user_auth(byte type, WS_UserAuthData* data, void* context) {
        auto* session = static_cast<SshSession*>(context);
        if (session == nullptr || data == nullptr || type != WOLFSSH_USERAUTH_PUBLICKEY) {
            return WOLFSSH_USERAUTH_FAILURE;
        }
        const std::size_t n = std::min<std::size_t>(data->usernameSz, session->user_.size());
        std::memcpy(session->user_.data(), data->username, n);
        session->user_size_ = n;
        const auto& pk = data->sf.publicKey;
        if (pk.isCert || pk.publicKey == nullptr ||
            !session->context_->authorized({reinterpret_cast<const std::byte*>(pk.publicKey), pk.publicKeySz})) {
            ++g_stats.rejected_keys;
            return WOLFSSH_USERAUTH_INVALID_PUBLICKEY;
        }
        return WOLFSSH_USERAUTH_SUCCESS;
    }
    static int auth_types(WOLFSSH*, void*) { return WOLFSSH_USERAUTH_PUBLICKEY; }
};

// ---- context ----------------------------------------------------------------------------

SshInitResult SshServerContext::init(std::span<std::byte> memory, const SshCredentials& credentials,
                                     int max_sessions, const TlsPoolLayout& layout) noexcept {
    key_count_ = 0;
    for (const std::string_view line : credentials.authorized_keys) {
        const auto key = parse_authorized_key(line);
        if (!key || key_count_ == keys_.size()) {
            return SshInitResult::AuthorizedKeys;
        }
        keys_[key_count_++] = *key;
    }
    if ((last_error_ = wolfSSH_Init()) != WS_SUCCESS) {
        return SshInitResult::Library;
    }
    WOLFSSL_HEAP_HINT* hint = nullptr;
    if (layout.sizes.empty()) {
        last_error_ = wc_LoadStaticMemory(&hint, reinterpret_cast<unsigned char*>(memory.data()),
                                          static_cast<unsigned int>(memory.size()), WOLFMEM_GENERAL, max_sessions);
    } else {
        if (layout.sizes.size() != layout.counts.size() || layout.sizes.size() > WOLFMEM_MAX_BUCKETS) {
            return SshInitResult::StaticMemory;
        }
        std::array<word32, WOLFMEM_MAX_BUCKETS> sizes{}, counts{};
        std::copy(layout.sizes.begin(), layout.sizes.end(), sizes.begin());
        std::copy(layout.counts.begin(), layout.counts.end(), counts.begin());
        last_error_ = wc_LoadStaticMemory_ex(&hint, static_cast<unsigned int>(layout.sizes.size()), sizes.data(),
                                             counts.data(), reinterpret_cast<unsigned char*>(memory.data()),
                                             static_cast<unsigned int>(memory.size()), WOLFMEM_GENERAL, max_sessions);
    }
    if (last_error_ != 0) {
        return SshInitResult::StaticMemory;
    }
    // wolfCrypt paths without a heap hint use the global one (WOLFSSL_NO_MALLOC); TLS sets
    // it to its pool first, else this one serves.
    if (wolfSSL_GetGlobalHeapHint() == nullptr) {
        wolfSSL_SetGlobalHeapHint(hint);
    }
    WOLFSSH_CTX* ctx = wolfSSH_CTX_new(WOLFSSH_ENDPOINT_SERVER, hint);
    if (ctx == nullptr) {
        return SshInitResult::Context;
    }
    ctx_ = ctx;
    wolfSSH_SetIORecv(ctx, &SshIo::receive);
    wolfSSH_SetIOSend(ctx, &SshIo::send);
    wolfSSH_SetUserAuth(ctx, &SshIo::user_auth);
    wolfSSH_SetUserAuthTypes(ctx, &SshIo::auth_types);
    if ((last_error_ = wolfSSH_CTX_UsePrivateKey_buffer(
             ctx, reinterpret_cast<const byte*>(credentials.host_key_der.data()),
             static_cast<word32>(credentials.host_key_der.size()), WOLFSSH_FORMAT_ASN1)) != WS_SUCCESS) {
        return SshInitResult::HostKey;
    }
    last_error_ = 0;
    return SshInitResult::Ok;
}

bool SshServerContext::authorized(std::span<const std::byte> key) const noexcept {
    return std::any_of(keys_.begin(), keys_.begin() + static_cast<std::ptrdiff_t>(key_count_),
                       [&](const SshPublicKey& k) { return std::ranges::equal(k.bytes(), key); });
}

const SshStats& SshServerContext::stats() noexcept { return g_stats; }

// ---- session ------------------------------------------------------------------------------

bool SshSession::open(SshServerContext& context, const Transport& transport) noexcept {
    reset();
    if (context.ctx_ == nullptr) {
        return false;
    }
    WOLFSSH* ssh = wolfSSH_new(static_cast<WOLFSSH_CTX*>(context.ctx_));
    if (ssh == nullptr) {
        last_error_ = WS_MEMORY_E;
        status_ = SshStatus::Failed;
        return false;
    }
    ssh_ = ssh;
    context_ = &context;
    transport_ = transport;
    user_size_ = 0;
    wolfSSH_SetIOReadCtx(ssh, this);
    wolfSSH_SetIOWriteCtx(ssh, this);
    wolfSSH_SetUserAuthCtx(ssh, this);
    status_ = SshStatus::Handshake;
    return true;
}

void SshSession::fail(int error) noexcept {
    last_error_ = error;
    status_ = error == WS_DISCONNECT || error == WS_SOCKET_ERROR_E || error == WS_CHANNEL_CLOSED
                  ? SshStatus::Closed
                  : SshStatus::Failed;
}

SshStatus SshSession::advance() noexcept {
    auto* ssh = static_cast<WOLFSSH*>(ssh_);
    if (status_ == SshStatus::Handshake) {
        if (wolfSSH_accept(ssh) == WS_SUCCESS) {
            status_ = SshStatus::Open;
            ++g_stats.sessions;
        } else if (const int e = wolfSSH_get_error(ssh); e != WS_WANT_READ && e != WS_WANT_WRITE) {
            fail(e);
        }
        return status_;
    }
    if (status_ == SshStatus::Open) {
        const int r = wolfSSH_worker(ssh, nullptr);
        if (r == WS_CHANNEL_CLOSED || r == WS_EOF) {
            status_ = SshStatus::Closed;  // a terminal: the client is gone
        } else if (r == WS_FATAL_ERROR) {
            // "fatal" with a want only means: nothing more to do now
            if (const int e = wolfSSH_get_error(ssh); e != WS_WANT_READ && e != WS_WANT_WRITE) {
                fail(e);
            }
        } else if (r < 0 && r != WS_WANT_READ && r != WS_WANT_WRITE && r != WS_REKEYING && r != WS_CHAN_RXD &&
                   r != WS_EXTDATA) {
            fail(r);
        }
    }
    return status_;
}

std::size_t SshSession::read(std::span<std::byte> out) noexcept {
    if (status_ != SshStatus::Open || out.empty()) {
        return 0;
    }
    const int r = wolfSSH_stream_read(static_cast<WOLFSSH*>(ssh_), reinterpret_cast<byte*>(out.data()),
                                      static_cast<word32>(out.size()));
    return r > 0 ? static_cast<std::size_t>(r) : 0;
}

std::size_t SshSession::write(std::span<const std::byte> data) noexcept {
    if (status_ != SshStatus::Open || data.empty()) {
        return 0;
    }
    // Framed data counts as sent even if the transport takes it later (wolfSSH keeps it).
    const int r = wolfSSH_stream_send(static_cast<WOLFSSH*>(ssh_), const_cast<byte*>(reinterpret_cast<const byte*>(data.data())),
                                      static_cast<word32>(data.size()));
    return r > 0 ? static_cast<std::size_t>(r) : 0;
}

bool SshSession::can_write() const noexcept {
    return status_ == SshStatus::Open && wolfSSH_OutputPending(static_cast<const WOLFSSH*>(ssh_)) == 0;
}

void SshSession::shutdown() noexcept {
    if (ssh_ != nullptr && (status_ == SshStatus::Open || status_ == SshStatus::Handshake)) {
        (void)wolfSSH_shutdown(static_cast<WOLFSSH*>(ssh_));
    }
    status_ = SshStatus::Closed;
}

void SshSession::reset() noexcept {
    if (ssh_ != nullptr) {
        wolfSSH_free(static_cast<WOLFSSH*>(ssh_));
        ssh_ = nullptr;
    }
    context_ = nullptr;
    status_ = SshStatus::Idle;
    last_error_ = 0;
}

}  // namespace rtos::net
