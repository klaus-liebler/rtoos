// SPDX-License-Identifier: MIT
//
// HTTPS server with WebSocket endpoint: TCP (stack) -> TLS 1.3 (wolfSSL) -> HTTP/1.1
// -> static resources or a WebSocket session.
//
//   * Resources are spans (e.g. a brotli-compressed web app compiled into flash with
//     #embed); the body is handed to TLS straight from there.
//   * One WebSocket path; binary/text messages up to MaxMessage bytes in both
//     directions. Each outgoing frame (header + payload) is one TLS record, sent at
//     once (the stack has no Nagle).
//   * Every session is a non-blocking state machine driven by TCP events and by
//     service() (retry of output that did not fit into the TCP window, timeouts).
//
// Connection budget: browsers keep idle keep-alive connections open for minutes
// and open spare connections speculatively. A new connection that finds no free
// session therefore evicts the longest-idle keep-alive session (between two
// requests; never a WebSocket or a response in progress) instead of being refused.
// A keep-alive session without a complete request for keep_alive_timeout_ms is
// closed, a handshake not finished within handshake_timeout_ms is aborted
// (slowloris); both count from the last TCP event.
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
#include "rtos/net/tls.hpp"
#include "rtos/net/websocket.hpp"

namespace rtos::net {

struct HttpResource {
    std::string_view path;              ///< exact path, e.g. "/"
    std::string_view content_type;      ///< e.g. "text/html; charset=utf-8"
    std::string_view content_encoding;  ///< "br" for pre-compressed content, or empty
    std::span<const std::byte> body;
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

    static constexpr std::uint32_t keep_alive_timeout_ms = 10'000;
    static constexpr std::uint32_t handshake_timeout_ms = 10'000;

    HttpsServer(Stack& stack, TlsServerContext& tls, std::span<const HttpResource> resources,
                std::string_view websocket_path, const WebSocketHandler& websocket) noexcept
        : stack_(stack), tls_(tls), resources_(resources), ws_path_(websocket_path), ws_(websocket) {
        for (std::size_t i = 0; i < MaxSessions; ++i) {
            sessions_[i].id = static_cast<SessionId>(i);
        }
    }

    HttpsServer(const HttpsServer&) = delete;
    HttpsServer& operator=(const HttpsServer&) = delete;

    bool start(std::uint16_t port = 443) noexcept { return stack_.tcp().listen(port, tcp_handler_); }

    /// Queue a message for WebSocket session `id` (sent as one TLS record). False if
    /// not connected or the queue is full (the caller may retry later).
    bool send(SessionId id, std::span<const std::byte> message, bool binary = true) noexcept {
        if (id >= MaxSessions || message.size() > MaxMessage) {
            return false;
        }
        Session& s = sessions_[id];
        if (s.phase != Phase::WebSocket || s.tx_count == TxQueue || s.closing || s.finishing) {
            return false;
        }
        queue_frame(s, binary ? ws_opcode::binary : ws_opcode::text, message);
        flush_queue(s);  // output only: never re-enters the message handler
        return true;
    }

    /// send() to every open WebSocket session; returns how many accepted it.
    std::size_t broadcast(std::span<const std::byte> message, bool binary = true) noexcept {
        std::size_t n = 0;
        for (auto& s : sessions_) {
            n += s.phase == Phase::WebSocket && send(s.id, message, binary);
        }
        return n;
    }

    /// Retry pending output of all sessions and enforce the timeouts (call after
    /// Stack::poll()).
    void service() noexcept {
        for (auto& s : sessions_) {
            if (s.phase == Phase::Free) {
                continue;
            }
            pump(s);
            if (s.phase == Phase::Handshake && expired(s, handshake_timeout_ms)) {
                ++timeouts_;
                abort(s);
            } else if (s.phase == Phase::Request && !s.finishing && expired(s, keep_alive_timeout_ms)) {
                ++timeouts_;
                finish(s);
            }
        }
    }

    /// Earliest timeout (0: none).
    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = 0;
        for (const auto& s : sessions_) {
            std::uint32_t t = 0;
            if (s.phase == Phase::Handshake) {
                t = s.last_activity_ms + handshake_timeout_ms;
            } else if (s.phase == Phase::Request && !s.finishing) {
                t = s.last_activity_ms + keep_alive_timeout_ms;
            }
            if (t != 0 && (best == 0 || static_cast<std::int32_t>(t - best) < 0)) {
                best = t;
            }
        }
        return best;
    }

    [[nodiscard]] std::size_t websocket_clients() const noexcept {
        std::size_t n = 0;
        for (const auto& s : sessions_) {
            n += s.phase == Phase::WebSocket;
        }
        return n;
    }
    [[nodiscard]] std::uint32_t requests() const noexcept { return requests_; }
    [[nodiscard]] std::uint32_t handshake_failures() const noexcept { return handshake_failures_; }
    [[nodiscard]] std::uint32_t evictions() const noexcept { return evictions_; }
    [[nodiscard]] std::uint32_t timeouts() const noexcept { return timeouts_; }
    /// Cycle counter value when the TLS read that delivered the current message
    /// started (valid inside the message handler).
    [[nodiscard]] std::uint32_t read_cycle_stamp(SessionId id) const noexcept {
        return sessions_[id].tls.read_cycle_stamp();
    }

private:
    enum class Phase : std::uint8_t { Free, Handshake, Request, Response, WebSocket };

    struct Frame {
        std::array<std::byte, 4 + MaxMessage> bytes{};
        std::size_t size = 0;
    };

    struct Session {
        SessionId id = 0;
        Phase phase = Phase::Free;
        Connection* connection = nullptr;
        TlsSession tls;
        HttpRequestParser<1536> request;
        // response
        std::array<char, 384> header{};
        std::size_t header_size = 0;
        std::size_t header_sent = 0;
        std::span<const std::byte> body{};
        std::size_t body_sent = 0;
        bool keep_alive = false;
        bool close_after = false;
        bool upgrade = false;    ///< 101 response in flight: switch to WebSocket afterwards
        bool finishing = false;  ///< closing: close_notify + FIN once the output is out
        std::uint32_t last_activity_ms = 0;  ///< accepted / last TCP event
        // WebSocket: frames (header + payload contiguous: one write = one record)
        WebSocketDecoder<MaxMessage> decoder;
        std::array<std::byte, 16 + MaxMessage> rx{};  ///< a whole frame per TLS read (masked header + payload)
        std::array<Frame, TxQueue> tx{};
        std::size_t tx_head = 0;
        std::size_t tx_count = 0;
        std::size_t tx_sent = 0;
        bool closing = false;
    };

    /// Keep-alive connection waiting for its next request (nothing received yet).
    [[nodiscard]] static bool idle(const Session& s) noexcept {
        return s.phase == Phase::Request && !s.finishing && s.request.empty() && s.connection != nullptr &&
               s.connection->readable() == 0;
    }

    [[nodiscard]] bool expired(const Session& s, std::uint32_t timeout_ms) const noexcept {
        return static_cast<std::int32_t>(stack_.now_ms() - (s.last_activity_ms + timeout_ms)) >= 0;
    }

    // ---- TCP events ------------------------------------------------------------------

    static void on_accepted(void* self, TcpConnectionBase& base) noexcept {
        auto& server = *static_cast<HttpsServer*>(self);
        auto& conn = static_cast<Connection&>(base);
        for (auto& s : server.sessions_) {
            if (s.phase == Phase::Free) {
                server.open(s, conn);
                return;
            }
        }
        // All busy: evict the longest-idle keep-alive session (the client simply opens
        // a new connection when it needs one again).
        Session* victim = nullptr;
        for (auto& s : server.sessions_) {
            if (idle(s) && (victim == nullptr ||
                            static_cast<std::int32_t>(s.last_activity_ms - victim->last_activity_ms) < 0)) {
                victim = &s;
            }
        }
        if (victim == nullptr) {
            conn.abort();  // nothing idle: refuse
            return;
        }
        ++server.evictions_;
        server.abort(*victim);  // -> on_closed -> release: the session is free again
        if (victim->phase == Phase::Free) {
            server.open(*victim, conn);
        } else {
            conn.abort();
        }
    }

    static void on_activity(void* self, TcpConnectionBase& base) noexcept {
        auto* s = static_cast<Session*>(base.user);
        if (s != nullptr) {
            auto& server = *static_cast<HttpsServer*>(self);
            s->last_activity_ms = server.stack_.now_ms();
            server.pump(*s);
        }
    }

    static void on_closed(void* self, TcpConnectionBase& base, TcpCloseReason) noexcept {
        auto* s = static_cast<Session*>(base.user);
        if (s != nullptr) {
            static_cast<HttpsServer*>(self)->release(*s);
        }
    }

    void open(Session& s, Connection& conn) noexcept {
        s.connection = &conn;
        conn.user = &s;
        s.phase = Phase::Handshake;
        s.last_activity_ms = stack_.now_ms();
        s.request = {};
        s.closing = false;
        s.upgrade = false;
        s.finishing = false;
        s.tx_head = s.tx_count = s.tx_sent = 0;
        if (!s.tls.open(tls_, conn)) {
            ++handshake_failures_;
            conn.abort();  // -> on_closed -> release
        }
    }

    void release(Session& s) noexcept {
        if (s.phase == Phase::WebSocket && ws_.closed != nullptr) {
            ws_.closed(ws_.context, s.id);
        }
        s.tls.reset();
        if (s.connection != nullptr) {
            s.connection->user = nullptr;
        }
        s.connection = nullptr;
        s.phase = Phase::Free;
    }

    // ---- state machine ---------------------------------------------------------------

    void pump(Session& s) noexcept {
        if (s.finishing) {
            close_when_flushed(s);
            return;
        }
        for (int guard = 0; guard < 16 && s.phase != Phase::Free && !s.finishing; ++guard) {
            const Phase before = s.phase;
            switch (s.phase) {
                case Phase::Handshake: handshake(s); break;
                case Phase::Request: read_request(s); break;
                case Phase::Response: send_response(s); break;
                case Phase::WebSocket: websocket(s); break;
                case Phase::Free: return;
            }
            if (s.phase == before) {
                break;  // no state change: wait for the next event
            }
        }
        if (s.phase != Phase::Free && !s.finishing && s.connection != nullptr && s.connection->eof() &&
            s.phase != Phase::Response) {
            finish(s);
        }
    }

    void handshake(Session& s) noexcept {
        switch (s.tls.advance()) {
            case TlsStatus::Open: s.phase = Phase::Request; break;
            case TlsStatus::Failed:
            case TlsStatus::Closed:
                ++handshake_failures_;
                abort(s);
                break;
            default: break;
        }
    }

    void read_request(Session& s) noexcept {
        using State = typename HttpRequestParser<1536>::State;
        State state = s.request.state();
        while (state == State::Incomplete) {
            auto space = s.request.free_space();
            const std::size_t n = s.tls.read(std::as_writable_bytes(space));
            if (n == 0) {
                if (s.tls.status() != TlsStatus::Open) {
                    finish(s);
                }
                return;
            }
            state = s.request.commit(n);
        }
        if (state != State::Complete) {
            respond_error(s, "400 Bad Request");
            return;
        }
        ++requests_;
        route(s);
    }

    void route(Session& s) noexcept {
        const auto& r = s.request;
        s.keep_alive = r.keep_alive();
        std::string_view path = r.target();
        if (const std::size_t q = path.find('?'); q != std::string_view::npos) {
            path = path.substr(0, q);
        }
        if (r.method() != "GET") {
            respond_error(s, "405 Method Not Allowed");
            return;
        }
        if (path == ws_path_) {
            upgrade(s);
            return;
        }
        for (const auto& res : resources_) {
            if (res.path == path) {
                if (!res.content_encoding.empty()) {
                    const auto accept = r.header("Accept-Encoding");
                    if (!accept || !contains_token(*accept, res.content_encoding)) {
                        respond_error(s, "406 Not Acceptable");
                        return;
                    }
                }
                begin_header(s, "200 OK");
                add_header(s, "Content-Type", res.content_type);
                if (!res.content_encoding.empty()) {
                    add_header(s, "Content-Encoding", res.content_encoding);
                }
                add_length(s, res.body.size());
                add_header(s, "Cache-Control", "no-cache");
                end_header(s);
                s.body = res.body;
                return;
            }
        }
        respond_error(s, "404 Not Found");
    }

    void upgrade(Session& s) noexcept {
        const auto& r = s.request;
        const auto upgrade = r.header("Upgrade");
        const auto connection = r.header("Connection");
        const auto version = r.header("Sec-WebSocket-Version");
        const auto key = r.header("Sec-WebSocket-Key");
        const auto host = r.header("Host");
        const auto origin = r.header("Origin");
        std::array<char, 29> accept{};
        if (!upgrade || !iequals(*upgrade, "websocket") || !connection || !contains_token(*connection, "upgrade") ||
            !version || *version != "13" || !key || !websocket_accept_key(*key, accept)) {
            respond_error(s, "400 Bad Request");
            return;
        }
        // Same origin only: the page must have been loaded from this device.
        constexpr std::string_view scheme = "https://";
        if (origin && (!host || origin->size() != scheme.size() + host->size() || !origin->starts_with(scheme) ||
                       !iequals(origin->substr(scheme.size()), *host))) {
            respond_error(s, "403 Forbidden");
            return;
        }
        s.header_size = s.header_sent = 0;
        append(s, "HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n");
        add_header(s, "Sec-WebSocket-Accept", std::string_view(accept.data(), 28));
        append(s, "\r\n");
        s.body = {};
        s.body_sent = 0;
        s.phase = Phase::Response;
        s.keep_alive = true;
        s.decoder = {};
        s.upgrade = true;
    }

    void respond_error(Session& s, std::string_view status) noexcept {
        begin_header(s, status);
        add_header(s, "Content-Type", "text/plain");
        add_length(s, status.size());
        s.close_after = true;
        add_header(s, "Connection", "close");
        end_header(s);
        // Body: the status text. `status` is a string literal (static storage), so the
        // span stays valid while a TLS record referencing it is queued.
        s.body = std::as_bytes(std::span<const char>(status.data(), status.size()));
    }

    void send_response(Session& s) noexcept {
        if (!s.tls.flush()) {
            return;
        }
        while (s.header_sent < s.header_size) {
            const std::size_t n = s.tls.write(std::as_bytes(std::span(s.header.data() + s.header_sent, s.header_size - s.header_sent)));
            if (n == 0) {
                return;
            }
            s.header_sent += n;
        }
        while (s.body_sent < s.body.size()) {
            const std::size_t n = s.tls.write(s.body.subspan(s.body_sent));
            if (n == 0) {
                return;
            }
            s.body_sent += n;
        }
        if (!s.tls.flush()) {
            return;  // last record still queued: header/body must stay until it is out
        }
        if (s.upgrade) {
            s.upgrade = false;
            s.request.next();
            s.phase = Phase::WebSocket;
            if (ws_.opened != nullptr) {
                ws_.opened(ws_.context, s.id);
            }
            return;
        }
        if (s.close_after || !s.keep_alive) {
            finish(s);
            return;
        }
        s.request.next();
        s.phase = Phase::Request;
    }

    /// Write queued frames in order, one record each. True if the queue is empty.
    static bool flush_queue(Session& s) noexcept {
        while (s.tls.flush() && s.tx_count != 0) {
            Frame& f = s.tx[s.tx_head];
            while (s.tx_sent < f.size) {
                const std::size_t n = s.tls.write({f.bytes.data() + s.tx_sent, f.size - s.tx_sent});
                if (n == 0) {
                    return false;
                }
                s.tx_sent += n;
            }
            if (!s.tls.flush()) {
                return false;  // frame bytes must stay until the record is out
            }
            s.tx_head = (s.tx_head + 1) % TxQueue;
            --s.tx_count;
            s.tx_sent = 0;
        }
        return s.tx_count == 0 && !s.tls.output_pending();
    }

    void websocket(Session& s) noexcept {
        if (!flush_queue(s)) {
            return;  // output first: back-pressure on the input
        }
        if (s.closing) {
            finish(s);
            return;
        }
        for (;;) {
            const std::size_t n = s.tls.read(s.rx);
            if (n == 0) {
                if (s.tls.status() != TlsStatus::Open) {
                    finish(s);
                }
                return;
            }
            std::size_t pos = 0;
            while (pos < n) {
                typename WebSocketDecoder<MaxMessage>::Event event;
                pos += s.decoder.feed({s.rx.data() + pos, n - pos}, event);
                using E = typename WebSocketDecoder<MaxMessage>::Event;
                switch (event) {
                    case E::Message:
                        if (ws_.message != nullptr) {
                            ws_.message(ws_.context, s.id, s.decoder.message(),
                                        s.decoder.message_opcode() == ws_opcode::binary);
                        }
                        break;
                    case E::Ping: queue_control(s, ws_opcode::pong, s.decoder.control_payload()); break;
                    case E::Close: close_websocket(s, s.decoder.control_payload()); return;
                    case E::Error: {
                        const std::uint16_t code = s.decoder.close_code();
                        const std::array<std::byte, 2> payload{static_cast<std::byte>(code >> 8), static_cast<std::byte>(code)};
                        close_websocket(s, payload);
                        return;
                    }
                    case E::None: break;
                }
            }
        }
    }

    void queue_frame(Session& s, std::uint8_t opcode, std::span<const std::byte> payload) noexcept {
        Frame& f = s.tx[(s.tx_head + s.tx_count) % TxQueue];
        const std::size_t h = websocket_frame_header(f.bytes.data(), opcode, payload.size());
        std::copy(payload.begin(), payload.end(), f.bytes.begin() + static_cast<std::ptrdiff_t>(h));
        f.size = h + payload.size();
        ++s.tx_count;
    }

    void queue_control(Session& s, std::uint8_t opcode, std::span<const std::byte> payload) noexcept {
        if (s.tx_count < TxQueue) {
            queue_frame(s, opcode, payload.first(std::min<std::size_t>(payload.size(), 125)));
            flush_queue(s);
        }
    }

    void close_websocket(Session& s, std::span<const std::byte> payload) noexcept {
        queue_control(s, ws_opcode::close, payload.first(std::min<std::size_t>(payload.size(), 2)));
        s.closing = true;
        if (flush_queue(s)) {
            finish(s);
        }
    }

    /// Graceful end: once all output is out, close_notify and TCP FIN.
    void finish(Session& s) noexcept {
        if (s.connection == nullptr || s.finishing) {
            return;
        }
        s.finishing = true;
        close_when_flushed(s);
    }

    /// The session is free at once: TCP finishes the close (and any retransmissions)
    /// on its own.
    void close_when_flushed(Session& s) noexcept {
        if (s.connection != nullptr && s.tls.flush()) {
            s.tls.shutdown();  // close_notify (best effort)
            s.connection->close();
            release(s);
        }
    }

    void abort(Session& s) noexcept {
        if (s.connection != nullptr) {
            s.connection->abort();  // -> on_closed -> release
        }
    }

    // ---- header formatting -------------------------------------------------------

    void append(Session& s, std::string_view text) noexcept {
        const std::size_t n = std::min(text.size(), s.header.size() - s.header_size);
        std::copy_n(text.begin(), n, s.header.begin() + static_cast<std::ptrdiff_t>(s.header_size));
        s.header_size += n;
    }
    void begin_header(Session& s, std::string_view status) noexcept {
        s.header_size = s.header_sent = 0;
        s.body = {};
        s.body_sent = 0;
        s.close_after = false;
        s.phase = Phase::Response;
        append(s, "HTTP/1.1 ");
        append(s, status);
        append(s, "\r\nX-Content-Type-Options: nosniff\r\n");
    }
    void add_header(Session& s, std::string_view name, std::string_view value) noexcept {
        append(s, name);
        append(s, ": ");
        append(s, value);
        append(s, "\r\n");
    }
    void add_length(Session& s, std::size_t length) noexcept {
        std::array<char, 12> digits{};
        std::size_t n = 0;
        do {
            digits[n++] = static_cast<char>('0' + length % 10);
            length /= 10;
        } while (length != 0);
        append(s, "Content-Length: ");
        while (n != 0) {
            append(s, std::string_view(&digits[--n], 1));
        }
        append(s, "\r\n");
    }
    void end_header(Session& s) noexcept {
        if (!s.close_after) {
            append(s, s.keep_alive ? "Connection: keep-alive\r\n" : "Connection: close\r\n");
        }
        append(s, "\r\n");
    }

    Stack& stack_;
    TlsServerContext& tls_;
    std::span<const HttpResource> resources_;
    std::string_view ws_path_;
    WebSocketHandler ws_;
    std::array<Session, MaxSessions> sessions_{};
    TcpHandler tcp_handler_{this, &HttpsServer::on_accepted, &HttpsServer::on_activity, &HttpsServer::on_activity,
                            &HttpsServer::on_closed};
    std::uint32_t requests_ = 0;
    std::uint32_t handshake_failures_ = 0;
    std::uint32_t evictions_ = 0;
    std::uint32_t timeouts_ = 0;
};

}  // namespace rtos::net
