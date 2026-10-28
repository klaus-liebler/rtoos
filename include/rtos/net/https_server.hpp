// SPDX-License-Identifier: MIT
//
// HTTPS server with WebSocket endpoint: TCP (stack) -> TLS 1.3 (wolfSSL) -> HTTP/1.1
// -> static resources or a WebSocket session. Built on TcpServer (tcp_server.hpp),
// which handles the session table, eviction of idle keep-alive connections and
// deadlines; this file is the protocol: one Session per connection.
//
//   * Resources are spans (e.g. a brotli-compressed web app compiled into flash with
//     #embed); the body is handed to TLS straight from there.
//   * One WebSocket path; binary/text messages up to MaxMessage bytes in both
//     directions. Each outgoing frame (header + payload) is one TLS record, sent at
//     once (the stack has no Nagle).
//   * Timeouts (HttpsServerConfig): a handshake must finish, a keep-alive connection
//     must send its next request in time; a WebSocket peer that has been silent for
//     ws_ping_interval_ms is pinged and closed if nothing arrives within
//     ws_pong_timeout_ms (a vanished browser does not hold its session forever).
//
// Fairness (service.hpp): the TLS handshake and responses with a body larger than
// inline_response_max are background work - one handshake flight or one record of
// at most bulk_quantum_bytes per step - and such a response marks its connection
// bulk (TCP bulk budget). Small responses and WebSocket traffic stay inline. At most
// max_handshakes handshakes run at a time (each holds several KB of the TLS pool
// between its flights); further clients wait with their ClientHello in TCP.
//
// Security: TLS only; the WebSocket upgrade requires Origin == https://<Host>
// (cross-site WebSocket hijacking); a resource stored with Content-Encoding br is
// only served to clients that accept br (406 otherwise).
//
// No heap: sessions are a fixed table; wolfSSL allocates from the TLS context's
// static pool.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/net/http.hpp"
#include "rtos/net/tcp.hpp"
#include "rtos/net/tcp_server.hpp"
#include "rtos/net/tls.hpp"
#include "rtos/net/websocket.hpp"

namespace rtos::net {

struct HttpResource {
    std::string_view path;              ///< exact path, e.g. "/"; "*": every GET path no other
                                        ///< resource matches (single-page apps with client routes)
    std::string_view content_type;      ///< e.g. "text/html; charset=utf-8"
    std::string_view content_encoding;  ///< "br" for pre-compressed content, or empty
    std::span<const std::byte> body;
};

struct HttpsServerConfig {
    std::uint32_t handshake_timeout_ms = 10'000;
    std::uint32_t keep_alive_timeout_ms = 10'000;  ///< for the next request
    std::uint32_t ws_ping_interval_ms = 20'000;    ///< peer silent this long: ping
    std::uint32_t ws_pong_timeout_ms = 10'000;     ///< no data after the ping: close
    std::size_t inline_response_max = 1024;        ///< larger bodies are background work
    std::size_t bulk_quantum_bytes = 2048;         ///< body bytes (one record) per step
    std::uint8_t max_handshakes = 2;               ///< concurrent TLS handshakes
};

template <class Stack, std::size_t MaxSessions = 4, std::size_t MaxMessage = 512, std::size_t TxQueue = 4>
class HttpsServer {
public:
    using SessionId = std::uint8_t;
    using Connection = typename Stack::TcpConnection;

    struct WebSocketHandler {
        void* context = nullptr;
        void (*opened)(void* context, SessionId id) noexcept = nullptr;
        void (*message)(void* context, SessionId id, std::span<const std::byte> data, bool binary) noexcept = nullptr;
        void (*closed)(void* context, SessionId id) noexcept = nullptr;
    };

    HttpsServer(Stack& stack, TlsServerContext& tls, std::span<const HttpResource> resources,
                std::string_view websocket_path, const WebSocketHandler& websocket,
                const HttpsServerConfig& config = {}) noexcept
        : tcp_(stack), stack_(stack), tls_(tls), resources_(resources), ws_path_(websocket_path), ws_(websocket),
          config_(config) {
        for (std::size_t i = 0; i < MaxSessions; ++i) {
            tcp_.session(static_cast<SessionId>(i)).bind(this, static_cast<SessionId>(i));
        }
    }

    HttpsServer(const HttpsServer&) = delete;
    HttpsServer& operator=(const HttpsServer&) = delete;

    /// Serve HTTPS on `port` on the interfaces in `on` (default: all).
    bool start(std::uint16_t port = 443, InterfaceSet on = InterfaceSet::all()) noexcept {
        return tcp_.listen(port, on);
    }

    // ---- NetService ------------------------------------------------------------
    void service() noexcept { tcp_.service(); }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return tcp_.next_deadline(); }
    [[nodiscard]] bool background_pending() const noexcept { return tcp_.background_pending(); }
    void background_step() noexcept { tcp_.background_step(); }

    // ---- WebSocket ---------------------------------------------------------------

    /// Queue a message for WebSocket session `id` (sent as one TLS record). False if
    /// not connected or the queue is full (the caller may retry later).
    bool send(SessionId id, std::span<const std::byte> message, bool binary = true) noexcept {
        if (!tcp_.active(id) || message.size() > MaxMessage) {
            return false;
        }
        return tcp_.session(id).send(message, binary);
    }

    /// send() to every open WebSocket session; returns how many accepted it.
    std::size_t broadcast(std::span<const std::byte> message, bool binary = true) noexcept {
        std::size_t n = 0;
        for (std::size_t i = 0; i < MaxSessions; ++i) {
            n += send(static_cast<SessionId>(i), message, binary);
        }
        return n;
    }

    [[nodiscard]] std::size_t websocket_clients() const noexcept {
        std::size_t n = 0;
        for (std::size_t i = 0; i < MaxSessions; ++i) {
            n += tcp_.active(static_cast<SessionId>(i)) && tcp_.session(static_cast<SessionId>(i)).websocket();
        }
        return n;
    }

    // ---- statistics --------------------------------------------------------------
    [[nodiscard]] std::uint32_t requests() const noexcept { return requests_; }
    [[nodiscard]] std::uint32_t handshake_failures() const noexcept { return handshake_failures_; }
    [[nodiscard]] std::uint32_t evictions() const noexcept { return tcp_.stats().evictions; }
    [[nodiscard]] std::uint32_t timeouts() const noexcept { return timeouts_; }
    [[nodiscard]] std::uint32_t dead_websockets() const noexcept { return dead_websockets_; }
    /// Open sessions reset because TLS failed (and the wolfSSL error of the last one).
    [[nodiscard]] std::uint32_t tls_failures() const noexcept { return tls_failures_; }
    [[nodiscard]] int last_tls_error() const noexcept { return last_tls_error_; }
    /// Cycle counter value when the TLS read that delivered the current message
    /// started (valid inside the message handler).
    [[nodiscard]] std::uint32_t read_cycle_stamp(SessionId id) const noexcept {
        return tcp_.session(id).read_cycle_stamp();
    }

private:
    enum class Phase : std::uint8_t { Free, Handshake, Request, Response, WebSocket };

    /// A handshake turn became free: let the sessions waiting for one try again.
    void wake_handshake_waiters(SessionId except) noexcept {
        for (std::size_t i = 0; i < MaxSessions; ++i) {
            const auto id = static_cast<SessionId>(i);
            if (id != except && tcp_.active(id) && tcp_.session(id).waiting_for_handshake_turn()) {
                tcp_.pump(id);  // background session: marked ready
            }
        }
    }

    /// One connection: TLS -> HTTP request/response or WebSocket (TcpSession).
    class Session {
    public:
        void bind(HttpsServer* owner, SessionId id) noexcept {
            owner_ = owner;
            id_ = id;
        }

        // ---- TcpSession -------------------------------------------------------------
        bool open(Connection& c, std::uint32_t now) noexcept {
            conn_ = &c;
            phase_ = Phase::Handshake;
            last_event_ms_ = now;
            request_ = {};
            closing_ = upgrade_ = finishing_ = finished_ = false;
            tx_head_ = tx_count_ = tx_sent_ = 0;
            if (!tls_.open(owner_->tls_, c)) {
                ++owner_->handshake_failures_;
                return false;
            }
            return true;
        }

        void pump(Connection&, std::uint32_t now, bool event) noexcept {
            if (event) {
                last_event_ms_ = now;
            }
            if (finishing_) {
                close_when_flushed();
                return;
            }
            if (expired(now)) {
                switch (phase_) {
                    case Phase::Handshake:
                        ++owner_->timeouts_;
                        conn_->abort();  // -> closed()
                        return;
                    case Phase::Request:
                        ++owner_->timeouts_;
                        finish();
                        return;
                    case Phase::WebSocket:
                        if (ping_outstanding_) {
                            ++owner_->dead_websockets_;
                            conn_->abort();  // the peer is gone
                            return;
                        }
                        ping_outstanding_ = true;
                        ping_sent_ms_ = now;
                        queue_control(ws_opcode::ping, {});
                        break;
                    case Phase::Response:
                    case Phase::Free: break;
                }
            }
            for (int guard = 0; guard < 16 && conn_ != nullptr && phase_ != Phase::Free && !finishing_; ++guard) {
                if (background()) {
                    break;  // expensive from here on: continues as background steps
                }
                const Phase before = phase_;
                switch (phase_) {
                    case Phase::Handshake: handshake(); break;
                    case Phase::Request: read_request(); break;
                    case Phase::Response: send_response(); break;
                    case Phase::WebSocket: websocket(now); break;
                    case Phase::Free: return;
                }
                if (phase_ == before) {
                    break;  // no state change: wait for the next event
                }
            }
            if (conn_ != nullptr && phase_ != Phase::Free && !finishing_ && phase_ != Phase::Response && conn_->eof()) {
                finish();
            }
            abort_if_failed();
        }

        void closed() noexcept {
            if (phase_ == Phase::WebSocket && owner_->ws_.closed != nullptr) {
                owner_->ws_.closed(owner_->ws_.context, id_);
            }
            end_handshake_turn();
            tls_.reset();
            conn_ = nullptr;
            phase_ = Phase::Free;
            bulk_ = false;
        }

        // ---- BackgroundSession -------------------------------------------------------
        [[nodiscard]] bool background() const noexcept {
            return !finishing_ && (phase_ == Phase::Handshake || (phase_ == Phase::Response && bulk_));
        }

        bool step(Connection&, std::uint32_t) noexcept {
            bool more = false;
            switch (phase_) {
                case Phase::Handshake:
                    if (take_handshake_turn()) {
                        handshake();  // the next flight needs the peer
                    }
                    break;
                case Phase::Response: more = send_response(owner_->config_.bulk_quantum_bytes); break;
                default: break;
            }
            return !abort_if_failed() && more;
        }

        [[nodiscard]] bool finished() const noexcept { return finished_; }

        /// Keep-alive connection waiting for its next request (nothing received yet).
        [[nodiscard]] bool evictable() const noexcept {
            return phase_ == Phase::Request && !finishing_ && request_.empty() && conn_ != nullptr &&
                   conn_->readable() == 0;
        }

        [[nodiscard]] std::uint32_t deadline() const noexcept {
            const HttpsServerConfig& cfg = owner_->config_;
            switch (phase_) {
                case Phase::Handshake: return last_event_ms_ + cfg.handshake_timeout_ms;
                case Phase::Request: return finishing_ ? 0 : last_event_ms_ + cfg.keep_alive_timeout_ms;
                case Phase::WebSocket:
                    return ping_outstanding_ ? ping_sent_ms_ + cfg.ws_pong_timeout_ms
                                             : last_rx_ms_ + cfg.ws_ping_interval_ms;
                case Phase::Response:
                case Phase::Free: return 0;
            }
            return 0;
        }

        // ---- used by the server ---------------------------------------------------
        [[nodiscard]] bool websocket() const noexcept { return phase_ == Phase::WebSocket; }
        [[nodiscard]] bool waiting_for_handshake_turn() const noexcept {
            return conn_ != nullptr && phase_ == Phase::Handshake && !handshake_turn_;
        }
        [[nodiscard]] std::uint32_t read_cycle_stamp() const noexcept { return tls_.read_cycle_stamp(); }

        bool send(std::span<const std::byte> message, bool binary) noexcept {
            if (phase_ != Phase::WebSocket || tx_count_ == TxQueue || closing_ || finishing_) {
                return false;
            }
            queue_frame(binary ? ws_opcode::binary : ws_opcode::text, message);
            flush_queue();  // output only: never re-enters the message handler
            return true;
        }

    private:
        struct Frame {
            std::array<std::byte, 4 + MaxMessage> bytes{};
            std::size_t size = 0;
        };

        /// A failed TLS session (e.g. MEMORY_E from the static pool) can never send
        /// again: reset the connection instead of keeping a session that waits forever.
        bool abort_if_failed() noexcept {
            if (conn_ == nullptr || tls_.status() != TlsStatus::Failed || phase_ == Phase::Handshake) {
                return false;  // (a failed handshake aborts in handshake())
            }
            ++owner_->tls_failures_;
            owner_->last_tls_error_ = tls_.last_error();
            conn_->abort();  // -> closed()
            return true;
        }

        [[nodiscard]] bool expired(std::uint32_t now) const noexcept {
            const std::uint32_t d = deadline();
            return d != 0 && static_cast<std::int32_t>(now - d) >= 0;
        }

        /// True if this session may run its handshake now (max_handshakes).
        bool take_handshake_turn() noexcept {
            if (!handshake_turn_) {
                if (owner_->handshakes_ >= owner_->config_.max_handshakes) {
                    return false;  // end_handshake_turn() of another session wakes us
                }
                handshake_turn_ = true;
                ++owner_->handshakes_;
            }
            return true;
        }

        void end_handshake_turn() noexcept {
            if (handshake_turn_) {
                handshake_turn_ = false;
                --owner_->handshakes_;
                owner_->wake_handshake_waiters(id_);
            }
        }

        void handshake() noexcept {
            switch (tls_.advance()) {
                case TlsStatus::Open:
                    phase_ = Phase::Request;
                    end_handshake_turn();
                    break;
                case TlsStatus::Failed:
                case TlsStatus::Closed:
                    ++owner_->handshake_failures_;
                    conn_->abort();  // -> closed()
                    break;
                default: break;
            }
        }

        void read_request() noexcept {
            using State = typename HttpRequestParser<1536>::State;
            State state = request_.state();
            while (state == State::Incomplete) {
                auto space = request_.free_space();
                const std::size_t n = tls_.read(std::as_writable_bytes(space));
                if (n == 0) {
                    if (tls_.status() != TlsStatus::Open) {
                        finish();
                    }
                    return;
                }
                state = request_.commit(n);
            }
            if (state != State::Complete) {
                respond_error("400 Bad Request");
                return;
            }
            ++owner_->requests_;
            route();
        }

        void route() noexcept {
            const auto& r = request_;
            keep_alive_ = r.keep_alive();
            std::string_view path = r.target();
            if (const std::size_t q = path.find('?'); q != std::string_view::npos) {
                path = path.substr(0, q);
            }
            if (r.method() != "GET") {
                respond_error("405 Method Not Allowed");
                return;
            }
            if (path == owner_->ws_path_) {
                upgrade();
                return;
            }
            const HttpResource* match = nullptr;
            for (const auto& res : owner_->resources_) {
                if (res.path == path) {
                    match = &res;
                    break;
                }
                if (res.path == "*" && match == nullptr) {
                    match = &res;  // fallback, unless an exact path follows
                }
            }
            if (match != nullptr) {
                const HttpResource& res = *match;
                if (!res.content_encoding.empty()) {
                    const auto accept = r.header("Accept-Encoding");
                    if (!accept || !contains_token(*accept, res.content_encoding)) {
                        respond_error("406 Not Acceptable");
                        return;
                    }
                }
                begin_header("200 OK");
                add_header("Content-Type", res.content_type);
                if (!res.content_encoding.empty()) {
                    add_header("Content-Encoding", res.content_encoding);
                }
                add_length(res.body.size());
                add_header("Cache-Control", "no-cache");
                end_header();
                body_ = res.body;
                bulk_ = body_.size() > owner_->config_.inline_response_max;
                conn_->set_bulk(bulk_);
                return;
            }
            respond_error("404 Not Found");
        }

        void upgrade() noexcept {
            const auto& r = request_;
            const auto upgrade = r.header("Upgrade");
            const auto connection = r.header("Connection");
            const auto version = r.header("Sec-WebSocket-Version");
            const auto key = r.header("Sec-WebSocket-Key");
            const auto host = r.header("Host");
            const auto origin = r.header("Origin");
            std::array<char, 29> accept{};
            if (!upgrade || !iequals(*upgrade, "websocket") || !connection || !contains_token(*connection, "upgrade") ||
                !version || *version != "13" || !key || !websocket_accept_key(*key, accept)) {
                respond_error("400 Bad Request");
                return;
            }
            // Same origin only: the page must have been loaded from this device.
            constexpr std::string_view scheme = "https://";
            if (origin && (!host || origin->size() != scheme.size() + host->size() || !origin->starts_with(scheme) ||
                           !iequals(origin->substr(scheme.size()), *host))) {
                respond_error("403 Forbidden");
                return;
            }
            header_size_ = header_sent_ = 0;
            append("HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n");
            add_header("Sec-WebSocket-Accept", std::string_view(accept.data(), 28));
            append("\r\n");
            body_ = {};
            body_sent_ = 0;
            phase_ = Phase::Response;
            keep_alive_ = true;
            decoder_ = {};
            upgrade_ = true;
        }

        void respond_error(std::string_view status) noexcept {
            begin_header(status);
            add_header("Content-Type", "text/plain");
            add_length(status.size());
            close_after_ = true;
            add_header("Connection", "close");
            end_header();
            // Body: the status text. `status` is a string literal (static storage), so
            // the span stays valid while a TLS record referencing it is queued.
            body_ = std::as_bytes(std::span<const char>(status.data(), status.size()));
        }

        /// Send header and up to `max_body` body bytes. True if body bytes are left
        /// and the transport took everything so far (another step can go on).
        bool send_response(std::size_t max_body = static_cast<std::size_t>(-1)) noexcept {
            if (!tls_.flush()) {
                return false;
            }
            while (header_sent_ < header_size_) {
                if (conn_->writable() == 0) {
                    return false;  // no record that TCP cannot take: it would hold TLS memory
                }
                const std::size_t n =
                    tls_.write(std::as_bytes(std::span(header_.data() + header_sent_, header_size_ - header_sent_)));
                if (n == 0) {
                    return false;
                }
                header_sent_ += n;
            }
            std::size_t budget = max_body;
            while (body_sent_ < body_.size() && budget != 0) {
                if (conn_->writable() == 0) {
                    return false;  // window full or no bulk turn: wait for an ACK / the turn
                }
                const std::size_t n = tls_.write(body_.subspan(body_sent_, std::min(body_.size() - body_sent_, budget)));
                if (n == 0) {
                    return false;
                }
                body_sent_ += n;
                budget -= std::min(n, budget);
            }
            if (body_sent_ < body_.size()) {
                return !tls_.output_pending();
            }
            if (!tls_.flush()) {
                return false;  // last record still queued: header/body must stay until it is out
            }
            bulk_ = false;
            conn_->set_bulk(false);
            if (upgrade_) {
                upgrade_ = false;
                request_.next();
                phase_ = Phase::WebSocket;
                last_rx_ms_ = owner_->stack_.now_ms();
                ping_outstanding_ = false;
                if (owner_->ws_.opened != nullptr) {
                    owner_->ws_.opened(owner_->ws_.context, id_);
                }
                return false;
            }
            if (close_after_ || !keep_alive_) {
                finish();
                return false;
            }
            request_.next();
            phase_ = Phase::Request;
            return false;
        }

        /// Write queued frames in order, one record each. True if the queue is empty.
        bool flush_queue() noexcept {
            while (tls_.flush() && tx_count_ != 0) {
                Frame& f = tx_[tx_head_];
                while (tx_sent_ < f.size) {
                    const std::size_t n = tls_.write({f.bytes.data() + tx_sent_, f.size - tx_sent_});
                    if (n == 0) {
                        return false;
                    }
                    tx_sent_ += n;
                }
                if (!tls_.flush()) {
                    return false;  // frame bytes must stay until the record is out
                }
                tx_head_ = (tx_head_ + 1) % TxQueue;
                --tx_count_;
                tx_sent_ = 0;
            }
            return tx_count_ == 0 && !tls_.output_pending();
        }

        void websocket(std::uint32_t now) noexcept {
            if (!flush_queue()) {
                return;  // output first: back-pressure on the input
            }
            if (closing_) {
                finish();
                return;
            }
            for (;;) {
                const std::size_t n = tls_.read(rx_);
                if (n == 0) {
                    if (tls_.status() != TlsStatus::Open) {
                        finish();
                    }
                    return;
                }
                last_rx_ms_ = now;  // any frame, pongs included, proves the peer is there
                ping_outstanding_ = false;
                std::size_t pos = 0;
                while (pos < n) {
                    typename WebSocketDecoder<MaxMessage>::Event event;
                    pos += decoder_.feed({rx_.data() + pos, n - pos}, event);
                    using E = typename WebSocketDecoder<MaxMessage>::Event;
                    switch (event) {
                        case E::Message:
                            if (owner_->ws_.message != nullptr) {
                                owner_->ws_.message(owner_->ws_.context, id_, decoder_.message(),
                                                    decoder_.message_opcode() == ws_opcode::binary);
                            }
                            break;
                        case E::Ping: queue_control(ws_opcode::pong, decoder_.control_payload()); break;
                        case E::Close: close_websocket(decoder_.control_payload()); return;
                        case E::Error: {
                            const std::uint16_t code = decoder_.close_code();
                            const std::array<std::byte, 2> payload{static_cast<std::byte>(code >> 8),
                                                                   static_cast<std::byte>(code)};
                            close_websocket(payload);
                            return;
                        }
                        case E::None: break;
                    }
                }
            }
        }

        void queue_frame(std::uint8_t opcode, std::span<const std::byte> payload) noexcept {
            Frame& f = tx_[(tx_head_ + tx_count_) % TxQueue];
            const std::size_t h = websocket_frame_header(f.bytes.data(), opcode, payload.size());
            std::copy(payload.begin(), payload.end(), f.bytes.begin() + static_cast<std::ptrdiff_t>(h));
            f.size = h + payload.size();
            ++tx_count_;
        }

        void queue_control(std::uint8_t opcode, std::span<const std::byte> payload) noexcept {
            if (tx_count_ < TxQueue) {
                queue_frame(opcode, payload.first(std::min<std::size_t>(payload.size(), 125)));
                flush_queue();
            }
        }

        void close_websocket(std::span<const std::byte> payload) noexcept {
            queue_control(ws_opcode::close, payload.first(std::min<std::size_t>(payload.size(), 2)));
            closing_ = true;
            if (flush_queue()) {
                finish();
            }
        }

        /// Graceful end: once all output is out, close_notify and TCP FIN.
        void finish() noexcept {
            if (conn_ == nullptr || finishing_) {
                return;
            }
            finishing_ = true;
            close_when_flushed();
        }

        void close_when_flushed() noexcept {
            if (conn_ != nullptr && tls_.flush()) {
                tls_.shutdown();  // close_notify (best effort)
                conn_->close();
                finished_ = true;  // TcpServer frees the session; TCP completes the close
            }
        }

        // ---- header formatting -----------------------------------------------------
        void append(std::string_view text) noexcept {
            const std::size_t n = std::min(text.size(), header_.size() - header_size_);
            std::copy_n(text.begin(), n, header_.begin() + static_cast<std::ptrdiff_t>(header_size_));
            header_size_ += n;
        }
        void begin_header(std::string_view status) noexcept {
            header_size_ = header_sent_ = 0;
            body_ = {};
            body_sent_ = 0;
            bulk_ = false;
            close_after_ = false;
            phase_ = Phase::Response;
            append("HTTP/1.1 ");
            append(status);
            append("\r\nX-Content-Type-Options: nosniff\r\n");
        }
        void add_header(std::string_view name, std::string_view value) noexcept {
            append(name);
            append(": ");
            append(value);
            append("\r\n");
        }
        void add_length(std::size_t length) noexcept {
            std::array<char, 12> digits{};
            std::size_t n = 0;
            do {
                digits[n++] = static_cast<char>('0' + length % 10);
                length /= 10;
            } while (length != 0);
            append("Content-Length: ");
            while (n != 0) {
                append(std::string_view(&digits[--n], 1));
            }
            append("\r\n");
        }
        void end_header() noexcept {
            if (!close_after_) {
                append(keep_alive_ ? "Connection: keep-alive\r\n" : "Connection: close\r\n");
            }
            append("\r\n");
        }

        HttpsServer* owner_ = nullptr;
        SessionId id_ = 0;
        Connection* conn_ = nullptr;
        Phase phase_ = Phase::Free;
        TlsSession tls_;
        HttpRequestParser<1536> request_;
        std::uint32_t last_event_ms_ = 0;
        // response
        std::array<char, 384> header_{};
        std::size_t header_size_ = 0;
        std::size_t header_sent_ = 0;
        std::span<const std::byte> body_{};
        std::size_t body_sent_ = 0;
        bool keep_alive_ = false;
        bool close_after_ = false;
        bool bulk_ = false;       ///< large body: background steps on a bulk connection
        bool handshake_turn_ = false;  ///< counted in max_handshakes
        bool upgrade_ = false;    ///< 101 response in flight: switch to WebSocket afterwards
        bool finishing_ = false;  ///< closing: close_notify + FIN once the output is out
        bool finished_ = false;   ///< FIN sent: the slot may be reused
        // WebSocket: frames (header + payload contiguous: one write = one record)
        WebSocketDecoder<MaxMessage> decoder_;
        std::array<std::byte, 16 + MaxMessage> rx_{};  ///< a whole frame per TLS read
        std::array<Frame, TxQueue> tx_{};
        std::size_t tx_head_ = 0;
        std::size_t tx_count_ = 0;
        std::size_t tx_sent_ = 0;
        bool closing_ = false;
        std::uint32_t last_rx_ms_ = 0;
        std::uint32_t ping_sent_ms_ = 0;
        bool ping_outstanding_ = false;
    };

    TcpServer<Stack, Session, MaxSessions> tcp_;
    Stack& stack_;
    TlsServerContext& tls_;
    std::span<const HttpResource> resources_;
    std::string_view ws_path_;
    WebSocketHandler ws_;
    HttpsServerConfig config_;
    std::uint32_t requests_ = 0;
    std::uint32_t handshake_failures_ = 0;
    std::uint32_t timeouts_ = 0;
    std::uint32_t dead_websockets_ = 0;
    std::uint32_t tls_failures_ = 0;
    int last_tls_error_ = 0;
    std::uint8_t handshakes_ = 0;  ///< handshakes running (max_handshakes)
};

}  // namespace rtos::net
