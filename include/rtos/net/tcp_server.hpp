// SPDX-License-Identifier: MIT
//
// TcpServer: the connection handling every TCP-based server needs (HTTPS today,
// Modbus TCP and OPC UA later), so a protocol only implements its Session type.
//
//   * listens on one port, owns a fixed table of MaxSessions sessions (no heap)
//   * a new connection takes a free session; with none free it evicts the session
//     that has been idle longest and says so (evictable(): e.g. a keep-alive
//     connection between two requests) - otherwise the connection is refused (RST)
//   * pumps a session on every TCP event of its connection and when its deadline()
//     has passed (timeouts, retries, keep-alive pings)
//   * frees a session as soon as it reports finished() (it has sent its FIN; TCP
//     completes the close on its own), or when the connection goes away
//   * runs in the network thread as a NetService (service(), next_deadline())
//
// The Session type (default-constructible) provides:
//
//   bool open(Connection& c, std::uint32_t now)   accepted; false refuses (abort)
//   void pump(Connection& c, std::uint32_t now, bool event)
//        do all work possible now; `event`: data, window or close from the peer
//        (false: called for the deadline or by service())
//   void closed()                                  connection gone: release resources
//   bool finished() const                          protocol over, FIN sent
//   bool evictable() const                         idle: may be dropped for a newcomer
//   std::uint32_t deadline() const                 absolute ms for pump() without
//                                                  events, 0 = none
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/net/service.hpp"
#include "rtos/net/tcp.hpp"

namespace rtos::net {

template <class S, class Connection>
concept TcpSession = std::default_initializable<S> &&
                     requires(S& s, const S& cs, Connection& c, std::uint32_t now, bool event) {
                         { s.open(c, now) } noexcept -> std::same_as<bool>;
                         { s.pump(c, now, event) } noexcept;
                         { s.closed() } noexcept;
                         { cs.finished() } noexcept -> std::same_as<bool>;
                         { cs.evictable() } noexcept -> std::same_as<bool>;
                         { cs.deadline() } noexcept -> std::convertible_to<std::uint32_t>;
                     };

struct TcpServerStats {
    std::uint32_t accepted = 0;
    std::uint32_t refused = 0;    ///< no session free or evictable, or open() said no
    std::uint32_t evictions = 0;  ///< idle sessions dropped for a new connection
};

template <class Stack, class Session, std::size_t MaxSessions>
    requires TcpSession<Session, typename Stack::TcpConnection>
class TcpServer {
public:
    using Connection = typename Stack::TcpConnection;
    using SessionId = std::uint8_t;
    static_assert(MaxSessions <= 255);

    explicit TcpServer(Stack& stack) noexcept : stack_(stack) {}

    TcpServer(const TcpServer&) = delete;
    TcpServer& operator=(const TcpServer&) = delete;

    /// Serve `port` on the interfaces in `on` (default: all).
    bool listen(std::uint16_t port, InterfaceSet on = InterfaceSet::all()) noexcept {
        return stack_.tcp().listen(port, handler_, on);
    }

    // ---- NetService ------------------------------------------------------------

    /// Pump every active session (retries, deadlines) and free finished ones.
    void service() noexcept {
        const std::uint32_t now = stack_.now_ms();
        for (auto& slot : slots_) {
            if (slot.connection != nullptr) {
                slot.session.pump(*slot.connection, now, false);
                reap(slot);
            }
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = 0;
        for (const auto& slot : slots_) {
            if (slot.connection != nullptr) {
                best = earliest(best, slot.session.deadline());
            }
        }
        return best;
    }

    // ---- sessions -----------------------------------------------------------------

    [[nodiscard]] static constexpr std::size_t capacity() noexcept { return MaxSessions; }
    [[nodiscard]] bool active(SessionId id) const noexcept { return id < MaxSessions && slots_[id].connection != nullptr; }
    [[nodiscard]] Session& session(SessionId id) noexcept { return slots_[id].session; }
    [[nodiscard]] const Session& session(SessionId id) const noexcept { return slots_[id].session; }
    /// The connection of an active session (nullptr otherwise).
    [[nodiscard]] Connection* connection(SessionId id) noexcept { return slots_[id].connection; }
    [[nodiscard]] std::size_t active_count() const noexcept {
        std::size_t n = 0;
        for (const auto& slot : slots_) {
            n += slot.connection != nullptr;
        }
        return n;
    }
    [[nodiscard]] const TcpServerStats& stats() const noexcept { return stats_; }

    /// Pump session `id` outside a TCP event (e.g. after queueing output for it).
    void pump(SessionId id) noexcept {
        Slot& slot = slots_[id];
        if (slot.connection != nullptr) {
            slot.session.pump(*slot.connection, stack_.now_ms(), false);
            reap(slot);
        }
    }

private:
    struct Slot {
        Session session{};
        Connection* connection = nullptr;
        std::uint32_t last_event_ms = 0;
    };

    /// Free the slot of a finished session; the connection completes its close alone.
    void reap(Slot& slot) noexcept {
        if (slot.connection != nullptr && slot.session.finished()) {
            slot.connection->user = nullptr;
            slot.connection = nullptr;
            slot.session.closed();
        }
    }

    void release(Slot& slot) noexcept {
        if (slot.connection != nullptr) {
            slot.connection->user = nullptr;
            slot.connection = nullptr;
            slot.session.closed();
        }
    }

    static void on_accepted(void* self, TcpConnectionBase& base) noexcept {
        auto& server = *static_cast<TcpServer*>(self);
        auto& conn = static_cast<Connection&>(base);
        const std::uint32_t now = server.stack_.now_ms();
        Slot* slot = nullptr;
        for (auto& s : server.slots_) {
            if (s.connection == nullptr) {
                slot = &s;
                break;
            }
        }
        if (slot == nullptr) {
            // All busy: evict the session that has been idle longest.
            for (auto& s : server.slots_) {
                if (s.session.evictable() &&
                    (slot == nullptr || static_cast<std::int32_t>(s.last_event_ms - slot->last_event_ms) < 0)) {
                    slot = &s;
                }
            }
            if (slot == nullptr) {
                ++server.stats_.refused;
                conn.abort();
                return;
            }
            ++server.stats_.evictions;
            Connection* victim = slot->connection;
            server.release(*slot);
            victim->abort();
        }
        slot->connection = &conn;
        slot->last_event_ms = now;
        conn.user = slot;
        if (!slot->session.open(conn, now)) {
            ++server.stats_.refused;
            server.release(*slot);
            conn.abort();
            return;
        }
        ++server.stats_.accepted;
        server.reap(*slot);
    }

    static void on_activity(void* self, TcpConnectionBase& base) noexcept {
        auto* slot = static_cast<Slot*>(base.user);
        if (slot != nullptr && slot->connection != nullptr) {
            auto& server = *static_cast<TcpServer*>(self);
            slot->last_event_ms = server.stack_.now_ms();
            slot->session.pump(*slot->connection, slot->last_event_ms, true);
            server.reap(*slot);
        }
    }

    static void on_closed(void* self, TcpConnectionBase& base, TcpCloseReason) noexcept {
        auto* slot = static_cast<Slot*>(base.user);
        if (slot != nullptr) {
            static_cast<TcpServer*>(self)->release(*slot);
        }
    }

    Stack& stack_;
    std::array<Slot, MaxSessions> slots_{};
    TcpServerStats stats_{};
    TcpHandler handler_{this, &TcpServer::on_accepted, &TcpServer::on_activity, &TcpServer::on_activity,
                        &TcpServer::on_closed};
};

}  // namespace rtos::net
