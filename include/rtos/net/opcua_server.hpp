// SPDX-License-Identifier: MIT
//
// OPC UA server (Binary, opc.tcp) on TcpServer: the protocol is the FCU's own
// opcua_native library (rtos/opcua/: Hello, SecureChannel, Session, Read/Write/Browse/
// GetEndpoints), this file is the transport - one Session per client connection.
//
// Security (rtos/opcua/secure_channel.hpp):
//   * Basic256Sha256, mode Sign: the client certificate must chain to the trusted CA
//     (rtoosCA) and be valid now; full access
//   * None: read-only (GetEndpoints, sessions, Read, Browse)
//   * this server's identity: an RSA-2048 application instance certificate (OpcUaIdentity)
//
// Fairness (service.hpp): OpenSecureChannel costs three RSA-2048 private-key operations
// (two OAEP decryptions, one signature) - the most expensive single step of the whole
// device. An OPN is background work (one step), every other message is answered inline.
//
// Framing: one UA TCP message at a time (header MessageSize, single chunk, at most
// Connection's buffer); a response the TCP window cannot take at once is finished before
// the next message is read.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/net/interface.hpp"
#include "rtos/net/tcp_server.hpp"
#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/connection.hpp"
#include "rtos/opcua/secure_channel.hpp"
#include "rtos/opcua/security_crypto.hpp"
#include "rtos/opcua/tcp_message.hpp"
#include "rtos/opcua/transport.hpp"

namespace rtos::net {

/// The server's application instance certificate (RSA-2048, SAN URI = the application URI),
/// its key (PKCS#1 or PKCS#8 DER), and the CA that signed it and every client certificate.
struct OpcUaIdentity {
    std::span<const std::byte> certificate_der;
    std::span<const std::byte> private_key_der;
    std::span<const std::byte> ca_der;
};

enum class OpcUaInitResult : std::uint8_t { Ok, Crypto, Key, TrustedCa };

struct OpcUaServerConfig {
    std::uint32_t idle_timeout_ms = 600'000;  ///< close a silent client (0: never)
};

struct OpcUaServerStats {
    std::uint32_t messages = 0;
    std::uint32_t opens = 0;              ///< OpenSecureChannel requests (background steps)
    std::uint32_t untrusted_clients = 0;  ///< client certificates rejected
    std::uint32_t oversized = 0;          ///< messages larger than the buffer (connection closed)
};

template <class Stack, std::size_t MaxClients = 2>
class OpcUaServer {
public:
    using Connection = typename Stack::TcpConnection;
    static constexpr std::uint16_t default_port = 4840;
    static constexpr std::size_t buffer_size = 8192;  ///< receive buffer = Connection's limit

    /// `endpoint_url` (e.g. "opc.tcp://fcu.local:4840") and `address_space` outlive the server.
    OpcUaServer(Stack& stack, const opcua::AddressSpace& address_space, std::string_view endpoint_url,
                const OpcUaServerConfig& config = {}) noexcept
        : tcp_(stack), address_space_(address_space), endpoint_url_(endpoint_url), config_(config) {
        for (std::size_t i = 0; i < MaxClients; ++i) {
            auto& s = tcp_.session(static_cast<std::uint8_t>(i));
            s.owner = this;
            s.slot = static_cast<std::uint32_t>(i);
        }
    }

    OpcUaServer(const OpcUaServer&) = delete;
    OpcUaServer& operator=(const OpcUaServer&) = delete;

    /// Crypto pool (`crypto_memory`, 16-byte aligned, ~24 KB), server key, trusted CA. Once.
    [[nodiscard]] OpcUaInitResult init(const OpcUaIdentity& id, std::span<std::byte> crypto_memory) noexcept {
        namespace sec = opcua::security;
        if (!sec::InitCrypto(crypto_memory)) {
            return OpcUaInitResult::Crypto;
        }
        identity_.certificateDer = bytes(id.certificate_der);
        identity_.caCertificateDer = bytes(id.ca_der);
        if (!sec::LoadServerKey(bytes(id.private_key_der), identity_.privateKey)) {
            return OpcUaInitResult::Key;
        }
        if (!sec::SetTrustedCa(bytes(id.ca_der))) {
            return OpcUaInitResult::TrustedCa;
        }
        opcua::SetServerIdentity(identity_);
        return OpcUaInitResult::Ok;
    }

    /// Serve OPC UA on `port` on the interfaces in `on` (default: all).
    bool start(std::uint16_t port = default_port, InterfaceSet on = InterfaceSet::all()) noexcept {
        return tcp_.listen(port, on);
    }

    // ---- NetService --------------------------------------------------------------
    void service() noexcept { tcp_.service(); }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return tcp_.next_deadline(); }
    [[nodiscard]] bool background_pending() const noexcept { return tcp_.background_pending(); }
    void background_step() noexcept { tcp_.background_step(); }

    [[nodiscard]] const OpcUaServerStats& stats() const noexcept { return stats_; }
    [[nodiscard]] std::size_t clients() const noexcept { return tcp_.active_count(); }

private:
    static std::span<const opcua::Byte> bytes(std::span<const std::byte> s) noexcept {
        return {reinterpret_cast<const opcua::Byte*>(s.data()), s.size()};
    }

    /// One client connection: the transport of an opcua::Connection.
    struct Session final : opcua::ITransport {
        OpcUaServer* owner = nullptr;
        std::uint32_t slot = 0;
        Connection* conn = nullptr;
        opcua::Connection protocol;
        std::array<opcua::Byte, buffer_size> rx{};
        std::size_t rx_size = 0;
        std::span<const opcua::Byte> pending{};  ///< response not yet taken by TCP
        bool open_waiting = false;               ///< a complete OPN: the next background step
        bool close_requested = false;
        bool done = false;
        std::uint32_t last_ms = 0;

        // ---- opcua::ITransport -------------------------------------------------------
        bool Send(std::span<const opcua::Byte> data) override {
            pending = data;  // Connection's send buffer: valid until its next HandleMessage()
            flush();
            return true;
        }
        void RequestClose() override { close_requested = true; }

        // ---- TcpSession ---------------------------------------------------------------
        bool open(Connection& c, std::uint32_t now) noexcept {
            conn = &c;
            protocol.Reset(slot, this, &owner->address_space_, owner->endpoint_url_);
            rx_size = 0;
            pending = {};
            open_waiting = close_requested = done = false;
            last_ms = now;
            return true;
        }

        void pump(Connection&, std::uint32_t now, bool event) noexcept {
            if (event) {
                last_ms = now;
            }
            if (done || !flush()) {
                return;
            }
            while (!close_requested && !open_waiting) {
                rx_size += conn->read(std::as_writable_bytes(std::span(rx)).subspan(rx_size));
                opcua::TcpHeader header;
                if (rx_size < opcua::TCP_HEADER_SIZE || !opcua::PeekTcpHeader(std::span(rx).first(rx_size), header)) {
                    break;
                }
                if (header.messageSize < opcua::TCP_HEADER_SIZE || header.messageSize > rx.size()) {
                    ++owner->stats_.oversized;
                    close_requested = true;  // cannot be framed: the stream is lost
                    break;
                }
                if (rx_size < header.messageSize) {
                    break;  // the rest is still on its way
                }
                if (header.Is("OPN")) {
                    open_waiting = true;  // RSA: a background step (OpcUaServer comment)
                    break;
                }
                handle(header.messageSize);
                if (!flush()) {
                    return;
                }
            }
            const std::uint32_t idle = owner->config_.idle_timeout_ms;
            if ((close_requested && pending.empty()) || conn->eof() ||
                (idle != 0 && static_cast<std::int32_t>(now - (last_ms + idle)) >= 0)) {
                conn->close();
                done = true;
            }
        }

        void closed() noexcept { conn = nullptr; }
        [[nodiscard]] bool finished() const noexcept { return done; }
        [[nodiscard]] bool evictable() const noexcept { return rx_size == 0 && pending.empty() && !open_waiting; }
        [[nodiscard]] std::uint32_t deadline() const noexcept {
            const std::uint32_t idle = owner->config_.idle_timeout_ms;
            return done || idle == 0 ? 0 : last_ms + idle;
        }

        // ---- BackgroundSession ------------------------------------------------------------
        [[nodiscard]] bool background() const noexcept { return open_waiting && !done; }
        bool step(Connection&, std::uint32_t) noexcept {
            opcua::TcpHeader header;
            if (open_waiting && opcua::PeekTcpHeader(std::span(rx).first(rx_size), header)) {
                ++owner->stats_.opens;
                open_waiting = false;
                handle(header.messageSize);
                if (protocol.LastCertificateCheck() != opcua::security::CertificateCheck::Ok) {
                    ++owner->stats_.untrusted_clients;
                }
            }
            return false;  // continue inline (TcpServer pumps once the step is over)
        }

    private:
        /// Process the complete message at the front of rx, then drop it.
        void handle(std::size_t size) noexcept {
            ++owner->stats_.messages;
            protocol.HandleMessage(std::span<const opcua::Byte>(rx).first(size));
            std::copy(rx.begin() + static_cast<std::ptrdiff_t>(size), rx.begin() + static_cast<std::ptrdiff_t>(rx_size),
                      rx.begin());
            rx_size -= size;
        }

        /// Hand the pending response to TCP. True when nothing is pending.
        bool flush() noexcept {
            if (!pending.empty() && conn != nullptr) {
                pending = pending.subspan(conn->write(std::as_bytes(pending)));
            }
            return pending.empty();
        }
    };

    TcpServer<Stack, Session, MaxClients> tcp_;
    const opcua::AddressSpace& address_space_;
    std::string_view endpoint_url_;
    OpcUaServerConfig config_;
    opcua::ServerIdentity identity_{};
    OpcUaServerStats stats_{};
};

}  // namespace rtos::net
