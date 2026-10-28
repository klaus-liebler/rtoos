// SPDX-License-Identifier: MIT
//
// WebSocket messages from other threads. The HTTPS server belongs to the network thread
// (service.hpp: no locks); a control loop, a sensor thread or a log mirror that wants to
// tell the browsers something posts the message here instead:
//
//   rtos::net::WebSocketOutbox<Server, 2048, Kernel::Lock> outbox{server};
//   outbox.set_wake([](void*) noexcept { kernel.notify(net_tcb, slot_wake, 1, SetBits); }, nullptr);
//   runtime.add(outbox, "ws-outbox");                 // network thread
//
//   outbox.broadcast(status);                          // any thread or kernel-aware ISR
//   outbox.send(session, reply);
//
// Posting copies the message into a byte ring under one short Guard section and never
// waits: if the ring is full, the message is refused and counted (dropped). The network
// thread delivers everything queued at its next pass. A session whose own transmit queue
// is full at that moment misses the message (counted as undelivered): status and log
// messages are superseded by the next ones anyway.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace rtos::net {

struct WebSocketOutboxStats {
    std::uint32_t posted = 0;
    std::uint32_t dropped = 0;      ///< refused: ring full or message too large
    std::uint32_t undelivered = 0;  ///< a session's transmit queue was full at delivery
};

template <class Server, std::size_t Bytes = 2048, class Guard = struct WebSocketOutboxNoGuard>
class WebSocketOutbox {
public:
    using SessionId = typename Server::SessionId;
    static constexpr SessionId all_sessions = static_cast<SessionId>(0xFF);
    static constexpr std::size_t max_message = 512;

    explicit WebSocketOutbox(Server& server) noexcept : server_(server) {}
    WebSocketOutbox(const WebSocketOutbox&) = delete;
    WebSocketOutbox& operator=(const WebSocketOutbox&) = delete;

    /// Called after every successful post, outside the Guard: wake the network thread.
    void set_wake(void (*wake)(void*) noexcept, void* context) noexcept {
        wake_ = wake;
        wake_context_ = context;
    }

    bool broadcast(std::span<const std::byte> message, bool binary = true) noexcept {
        return post(all_sessions, message, binary);
    }
    bool send(SessionId session, std::span<const std::byte> message, bool binary = true) noexcept {
        return post(session, message, binary);
    }

    // ---- NetService (network thread) ---------------------------------------------------
    void service() noexcept {
        std::array<std::byte, max_message> message;
        for (;;) {
            Header h{};
            {
                [[maybe_unused]] const Guard guard{};
                if (used_ == 0) {
                    return;
                }
                read_bytes(std::as_writable_bytes(std::span(&h, 1)));
                read_bytes(std::span(message).first(h.length));
            }
            const auto data = std::span<const std::byte>(message).first(h.length);
            if (h.session == all_sessions) {
                const std::size_t clients = server_.websocket_clients();
                const std::size_t sent = server_.broadcast(data, h.binary != 0);
                stats_.undelivered += static_cast<std::uint32_t>(clients > sent ? clients - sent : 0);
            } else if (!server_.send(h.session, data, h.binary != 0)) {
                ++stats_.undelivered;
            }
        }
    }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return 0; }

    [[nodiscard]] WebSocketOutboxStats stats() const noexcept {
        [[maybe_unused]] const Guard guard{};
        return stats_;
    }

private:
    struct Header {
        std::uint16_t length;
        SessionId session;
        std::uint8_t binary;
    };

    bool post(SessionId session, std::span<const std::byte> message, bool binary) noexcept {
        bool ok = false;
        {
            [[maybe_unused]] const Guard guard{};
            const std::size_t need = sizeof(Header) + message.size();
            if (message.size() <= max_message && Bytes - used_ >= need) {
                const Header h{static_cast<std::uint16_t>(message.size()), session, binary ? std::uint8_t{1} : std::uint8_t{0}};
                write_bytes(std::as_bytes(std::span(&h, 1)));
                write_bytes(message);
                ++stats_.posted;
                ok = true;
            } else {
                ++stats_.dropped;
            }
        }
        if (ok && wake_ != nullptr) {
            wake_(wake_context_);
        }
        return ok;
    }

    void write_bytes(std::span<const std::byte> in) noexcept {
        for (const std::byte b : in) {
            ring_[(head_ + used_++) % Bytes] = b;
        }
    }
    void read_bytes(std::span<std::byte> out) noexcept {
        for (std::byte& b : out) {
            b = ring_[head_];
            head_ = (head_ + 1) % Bytes;
            --used_;
        }
    }

    Server& server_;
    std::array<std::byte, Bytes> ring_{};
    std::size_t head_ = 0;
    std::size_t used_ = 0;
    WebSocketOutboxStats stats_{};
    void (*wake_)(void*) noexcept = nullptr;
    void* wake_context_ = nullptr;
};

struct WebSocketOutboxNoGuard {};

}  // namespace rtos::net
