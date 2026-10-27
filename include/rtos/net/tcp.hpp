// SPDX-License-Identifier: MIT
//
// TCP (RFC 793/1122/6298), server side: passive open on listening ports.
//
// Zero copy
//   * Received in-order payload stays in its PacketBuffer and is queued on the
//     connection; read() copies out of it.
//   * Sent segments are built in place (payload written after the stack's
//     headroom, headers prepended) and stay referenced in the connection's
//     retransmission ring until acknowledged; retransmission re-sends the same
//     buffer (header fields refreshed once the driver has released it).
//
// Simplifications (documented, deliberate)
//   * Out-of-order segments are dropped (the peer retransmits); a duplicate ACK
//     signals the gap. Typical for small embedded stacks on a LAN.
//   * No congestion window: the amount in flight is bounded by
//     min(peer window, Cfg::tx_segments * MSS) per connection, which is also the
//     bound on buffer usage.
//   * No TCP options except MSS.
//
// Zero window (RFC 1122 4.2.2.17): while the peer's window is closed and nothing is
// in flight, a persist timer sends window probes (an ACK with seq = SND.NXT - 1,
// which the peer answers with its current window; like Linux) with exponential
// backoff, so a lost window update cannot stall the connection.
//
// Events (TcpHandler, all invoked from Stack::poll in the network thread):
//   accepted   connection established; set conn.user
//   received   new in-order data or end of stream (conn.eof())
//   writable   send space became available (data acknowledged)
//   closed     the connection is gone (after this call the slot is reused)
#pragma once

#include <algorithm>
#include <array>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>

#include "rtos/net/address.hpp"
#include "rtos/net/bytes.hpp"
#include "rtos/net/checksum.hpp"
#include "rtos/net/interface.hpp"
#include "rtos/net/packet.hpp"

namespace rtos::net {

enum class TcpState : std::uint8_t {
    Closed,
    SynReceived,
    Established,
    CloseWait,
    LastAck,
    FinWait1,
    FinWait2,
    Closing,
    TimeWait,
};

enum class TcpCloseReason : std::uint8_t {
    Normal,   ///< FIN handshake completed
    Reset,    ///< RST received
    Timeout,  ///< retransmission limit reached
    Aborted,  ///< local abort()
};

namespace tcp_flag {
inline constexpr std::uint8_t fin = 0x01;
inline constexpr std::uint8_t syn = 0x02;
inline constexpr std::uint8_t rst = 0x04;
inline constexpr std::uint8_t psh = 0x08;
inline constexpr std::uint8_t ack = 0x10;
}  // namespace tcp_flag

inline constexpr std::size_t tcp_header_size = 20;
inline constexpr std::uint8_t ip_proto_tcp = 6;

class TcpConnectionBase;

struct TcpHandler {
    void* context = nullptr;
    void (*accepted)(void* context, TcpConnectionBase& conn) noexcept = nullptr;
    void (*received)(void* context, TcpConnectionBase& conn) noexcept = nullptr;
    void (*writable)(void* context, TcpConnectionBase& conn) noexcept = nullptr;
    void (*closed)(void* context, TcpConnectionBase& conn, TcpCloseReason reason) noexcept = nullptr;
};

/// Bulk connections that may have data in flight at the same time
/// (Cfg::tcp_bulk_connections). Admission is per connection, not per buffer: an
/// admitted connection uses its full window - a share of one or two segments would
/// meet the peers' delayed ACK (200 ms per odd segment on Windows) and crawl.
struct TcpBulkBudget {
    std::uint32_t limit = 0;   ///< 0 = no limit
    std::uint32_t active = 0;  ///< bulk connections with data in flight
    [[nodiscard]] bool admits() const noexcept { return limit == 0 || active < limit; }
};

/// Application-facing part of a connection (independent of the stack type).
class TcpConnectionBase {
public:
    void* user = nullptr;  ///< free for the application (e.g. its session object)

    [[nodiscard]] TcpState state() const noexcept { return state_; }
    [[nodiscard]] Ipv4Address remote_address() const noexcept { return remote_ip_; }
    /// Our address of this connection (the destination of the SYN).
    [[nodiscard]] Ipv4Address local_address() const noexcept { return local_ip_; }
    [[nodiscard]] std::uint16_t remote_port() const noexcept { return remote_port_; }
    [[nodiscard]] std::uint16_t local_port() const noexcept { return local_port_; }
    [[nodiscard]] std::uint16_t mss() const noexcept { return mss_; }

    /// Bytes received and not yet read.
    [[nodiscard]] std::size_t readable() const noexcept { return rx_bytes_; }
    /// Peer has closed its direction and all its data has been read.
    [[nodiscard]] bool eof() const noexcept { return fin_received_ && rx_bytes_ == 0; }

    /// Copy up to out.size() received bytes. Returns the number copied.
    std::size_t read(std::span<std::byte> out) noexcept {
        std::size_t copied = 0;
        while (copied < out.size() && rx_head_ != nullptr) {
            PacketBuffer& b = *rx_head_;
            const std::size_t n = std::min(out.size() - copied, b.size());
            std::memcpy(out.data() + copied, b.data(), n);
            b.pull_front(n);
            copied += n;
            if (b.size() == 0) {
                drop_rx_head();
            }
        }
        rx_bytes_ -= copied;
        if (copied != 0) {
            window_update_pending_ = true;
        }
        return copied;
    }

    /// Bytes that may be sent right now (window, segment and buffer limits).
    [[nodiscard]] std::size_t writable() const noexcept {
        if (!can_send()) {
            return 0;
        }
        const std::uint32_t in_flight = snd_nxt_ - snd_una_;
        const std::uint32_t limit = std::min<std::uint32_t>(snd_wnd_, tx_window_limit_);
        if (bulk_ && bulk_in_flight_ == 0 && bulk_budget_ != nullptr && !bulk_budget_->admits()) {
            return 0;  // other bulk connections are sending: wait for a turn
        }
        return limit > in_flight ? limit - in_flight : 0;
    }

    /// Traffic class. At most Cfg::tcp_bulk_connections bulk connections (large
    /// downloads) send at the same time, so however many downloads run, they hold at
    /// most that many windows of transmit buffers and cannot take the packet pool
    /// from the replies of interactive connections (Modbus, OPC UA, WebSocket).
    void set_bulk(bool bulk) noexcept { bulk_ = bulk; }
    [[nodiscard]] bool bulk() const noexcept { return bulk_; }

    /// Graceful close: FIN after all queued data.
    void close() noexcept { close_requested_ = true; }

    [[nodiscard]] bool can_send() const noexcept {
        return (state_ == TcpState::Established || state_ == TcpState::CloseWait) && !close_requested_;
    }

protected:
    void drop_rx_head() noexcept {
        PacketBuffer* b = rx_head_;
        rx_head_ = b->next;
        if (rx_head_ == nullptr) {
            rx_tail_ = nullptr;
        }
        b->next = nullptr;
        PacketRef adopt(b);  // drops the queue's reference
        --rx_segments_;
    }

    void clear_rx() noexcept {
        while (rx_head_ != nullptr) {
            drop_rx_head();
        }
        rx_bytes_ = 0;
    }

    TcpState state_ = TcpState::Closed;
    Ipv4Address remote_ip_{};
    Ipv4Address local_ip_{};
    std::uint16_t remote_port_ = 0;
    std::uint16_t local_port_ = 0;
    std::uint16_t mss_ = 536;

    // send sequence space
    std::uint32_t iss_ = 0;
    std::uint32_t snd_una_ = 0;
    std::uint32_t snd_nxt_ = 0;
    std::uint32_t snd_wnd_ = 0;
    std::uint32_t tx_window_limit_ = 0;

    // receive sequence space
    std::uint32_t rcv_nxt_ = 0;

    // received data queue (buffers linked through PacketBuffer::next, each owning a ref)
    PacketBuffer* rx_head_ = nullptr;
    PacketBuffer* rx_tail_ = nullptr;
    std::size_t rx_bytes_ = 0;
    std::uint8_t rx_segments_ = 0;

    bool fin_received_ = false;
    bool fin_sent_ = false;
    bool close_requested_ = false;
    bool ack_pending_ = false;
    bool window_update_pending_ = false;
    bool bulk_ = false;
    std::uint8_t bulk_in_flight_ = 0;  ///< segments queued while bulk
    TcpBulkBudget* bulk_budget_ = nullptr;
};

/// TCP for stack `Ip`. `Ip` provides:
///   PacketRef allocate()                       buffer with full TX headroom
///   bool send_ipv4(PacketRef, Ipv4Address dst, std::uint8_t protocol, Ipv4Address src)
///                                              (through the interface owning src)
///   std::uint32_t now_ms()
///   std::uint32_t random32()
template <class Ip, class Cfg>
class Tcp {
public:
    static constexpr std::size_t max_connections = Cfg::tcp_connections;
    static constexpr std::size_t max_listeners = Cfg::tcp_listeners;
    static constexpr std::size_t tx_segments = Cfg::tcp_tx_segments;    ///< unacked segments per connection
    static constexpr std::size_t rx_segments = Cfg::tcp_rx_segments;    ///< queued RX buffers per connection
    static constexpr std::uint16_t local_mss = Cfg::tcp_mss;
    /// Smallest peer MSS taken (like Linux tcp_min_snd_mss): an MSS option of 0 would
    /// stall the connection, a tiny one makes it a cheap way to exhaust the buffers.
    static constexpr std::uint16_t min_peer_mss = 48;
    static constexpr std::uint32_t rto_initial_ms = 1000;
    static constexpr std::uint32_t rto_min_ms = 200;
    static constexpr std::uint32_t rto_max_ms = 60000;
    static constexpr std::uint8_t max_retries = 8;
    static constexpr std::uint32_t time_wait_ms = Cfg::tcp_time_wait_ms;
    static constexpr std::uint32_t fin_wait2_ms = Cfg::tcp_fin_wait2_ms;
    static constexpr std::size_t bulk_connections = Cfg::tcp_bulk_connections;

    class Connection : public TcpConnectionBase {
    public:
        /// Buffer for one segment payload: write up to mss() bytes after data().
        [[nodiscard]] PacketRef allocate() noexcept { return tcp_->ip_.allocate(); }

        /// Queue and transmit one segment. `payload->size()` <= min(mss(), writable()).
        bool send(PacketRef payload) noexcept { return tcp_->send_data(*this, std::move(payload)); }

        /// Copy `data` into as many segments as the window allows. Returns bytes taken.
        std::size_t write(std::span<const std::byte> data) noexcept {
            std::size_t done = 0;
            while (done < data.size()) {
                const std::size_t n = std::min({data.size() - done, writable(), std::size_t{mss_}});
                if (n == 0) {
                    break;
                }
                PacketRef seg = allocate();
                if (!seg || seg->tailroom() < n) {
                    break;
                }
                seg->append(data.subspan(done, n));
                if (!send(std::move(seg))) {
                    break;
                }
                done += n;
            }
            return done;
        }

        /// Immediate reset.
        void abort() noexcept { tcp_->abort(*this, TcpCloseReason::Aborted); }

        Connection() noexcept = default;
        Connection(const Connection&) = delete;
        Connection& operator=(const Connection&) = delete;
        ~Connection() { reset_state(); }  // the receive queue holds raw references

    private:
        friend class Tcp;

        struct Segment {
            PacketRef packet;
            std::uint32_t seq = 0;
            std::uint32_t sent_ms = 0;
            std::uint16_t tcp_offset = 0;  ///< TCP header position in the buffer
            std::uint16_t tcp_length = 0;  ///< header + payload
            std::uint16_t seq_length = 0;  ///< payload + SYN + FIN
            std::uint8_t flags = 0;
            bool retransmitted = false;
            bool bulk = false;  ///< counted in the bulk budget
        };

        /// Release a transmit segment (and its share of the bulk budget).
        void drop(Segment& s) noexcept {
            if (s.bulk && --bulk_in_flight_ == 0 && tcp_ != nullptr) {
                --tcp_->bulk_.active;
            }
            s.bulk = false;
            s.packet.reset();
        }

        void reset_state() noexcept {
            clear_rx();
            for (auto& s : tx_) {
                drop(s);
            }
            tx_head_ = tx_count_ = 0;
            static_cast<TcpConnectionBase&>(*this) = TcpConnectionBase{};
            bulk_budget_ = tcp_ != nullptr ? &tcp_->bulk_ : nullptr;
            rto_ms_ = rto_initial_ms;
            srtt_ms_ = rttvar_ms_ = 0;
            rtt_valid_ = false;
            retries_ = 0;
            deadline_ms_ = 0;
            persist_ms_ = persist_interval_ms_ = 0;
            listener_ = nullptr;
        }

        Tcp* tcp_ = nullptr;
        const TcpHandler* listener_ = nullptr;
        std::array<Segment, tx_segments + 1> tx_{};  // +1: SYN or FIN
        std::size_t tx_head_ = 0;
        std::size_t tx_count_ = 0;
        std::uint32_t rto_ms_ = rto_initial_ms;
        std::uint32_t srtt_ms_ = 0;
        std::uint32_t rttvar_ms_ = 0;
        std::uint32_t deadline_ms_ = 0;  ///< retransmission or TIME-WAIT expiry, 0 = none
        std::uint32_t persist_ms_ = 0;   ///< next window probe, 0 = window open
        std::uint32_t persist_interval_ms_ = 0;
        std::uint32_t opened_ms_ = 0;    ///< SYN received
        std::uint8_t retries_ = 0;
        bool rtt_valid_ = false;
    };

    explicit Tcp(Ip& ip) noexcept : ip_(ip) {
        for (auto& c : connections_) {
            c.tcp_ = this;
            c.bulk_budget_ = &bulk_;
        }
    }

    Tcp(const Tcp&) = delete;
    Tcp& operator=(const Tcp&) = delete;

    /// Accept connections on `port`. The handler must outlive the stack.
    /// Accept connections to `port` arriving on the interfaces in `on` (default: all).
    /// Listeners may share a port when their interface sets do not overlap; a SYN on
    /// an interface without a listener is answered with RST.
    bool listen(std::uint16_t port, const TcpHandler& handler, InterfaceSet on = InterfaceSet::all()) noexcept {
        if (on.empty()) {
            return false;
        }
        for (const auto& l : listeners_) {
            if (l.handler != nullptr && l.port == port && l.on.intersects(on)) {
                return false;
            }
        }
        for (auto& l : listeners_) {
            if (l.handler == nullptr) {
                l = {port, on, &handler};
                return true;
            }
        }
        return false;
    }

    /// Stop serving `handler`: remove its listeners and reset its connections (the
    /// handler is not called any more; e.g. a server object being destroyed).
    void unlisten(const TcpHandler& handler) noexcept {
        for (auto& l : listeners_) {
            if (l.handler == &handler) {
                l = Listener{};
            }
        }
        for (auto& c : connections_) {
            if (c.state_ != TcpState::Closed && c.listener_ == &handler) {
                c.listener_ = nullptr;
                abort(c, TcpCloseReason::Aborted);
            }
        }
    }

    [[nodiscard]] std::size_t active_connections() const noexcept {
        std::size_t n = 0;
        for (const auto& c : connections_) {
            n += c.state_ != TcpState::Closed;
        }
        return n;
    }

    // ---- called by the stack ----------------------------------------------------

    /// A TCP segment for us. `segment` starts at the TCP header.
    void input(PacketRef segment, Ipv4Address src, Ipv4Address dst, InterfaceId arrived) noexcept {
        PacketBuffer& b = *segment;
        if (b.size() < tcp_header_size) {
            return;
        }
        const std::byte* h = b.data();
        const std::size_t header_len = (std::to_integer<std::size_t>(h[12]) >> 4) * 4;
        if (header_len < tcp_header_size || header_len > b.size()) {
            return;
        }
        if (checksum_finish(checksum_add(b.bytes(), pseudo_header_sum(src, dst, b.size()))) != 0) {
            ++checksum_errors_;
            return;
        }
        Incoming seg{};
        seg.src = src;
        seg.dst = dst;
        seg.interface = arrived;
        seg.src_port = load_be16(h);
        seg.dst_port = load_be16(h + 2);
        seg.seq = load_be32(h + 4);
        seg.ack = load_be32(h + 8);
        seg.flags = std::to_integer<std::uint8_t>(h[13]);
        seg.window = load_be16(h + 14);
        seg.mss = parse_mss(h, header_len);
        b.pull_front(header_len);
        seg.payload_len = b.size();

        Connection* conn = find(src, seg.src_port, seg.dst_port);
        if (conn == nullptr) {
            no_connection(seg);
            return;
        }
        process(*conn, seg, std::move(segment));
    }

    /// Timers and deferred output; called once per poll pass.
    void service() noexcept {
        const std::uint32_t now = ip_.now_ms();
        wake_bulk_waiters();
        for (auto& c : connections_) {
            if (c.state_ == TcpState::Closed) {
                continue;
            }
            if (c.close_requested_ && !c.fin_sent_ && c.tx_count_ < c.tx_.size() &&
                (c.state_ == TcpState::Established || c.state_ == TcpState::CloseWait)) {
                send_fin(c);
            }
            if (c.deadline_ms_ != 0 && static_cast<std::int32_t>(now - c.deadline_ms_) >= 0) {
                expire(c);
                if (c.state_ == TcpState::Closed) {
                    continue;
                }
            }
            if (c.persist_ms_ != 0 && static_cast<std::int32_t>(now - c.persist_ms_) >= 0) {
                probe_window(c, now);
            }
            if (c.ack_pending_ || c.window_update_pending_) {
                send_control(c, tcp_flag::ack);
            }
        }
    }

    /// Earliest timer deadline in ms (0 = none).
    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = 0;
        for (const auto& c : connections_) {
            for (const std::uint32_t t : {c.deadline_ms_, c.persist_ms_}) {
                if (t != 0 && (best == 0 || static_cast<std::int32_t>(t - best) < 0)) {
                    best = t;
                }
            }
        }
        return best;
    }

    [[nodiscard]] std::uint32_t checksum_errors() const noexcept { return checksum_errors_; }
    /// Half-open connections dropped to make room for a new SYN.
    [[nodiscard]] std::uint32_t syn_recycled() const noexcept { return syn_recycled_; }
    /// Bulk connections with data in flight right now (see set_bulk()).
    [[nodiscard]] std::uint32_t bulk_active() const noexcept { return bulk_.active; }

    static std::uint32_t pseudo_header_sum(Ipv4Address src, Ipv4Address dst, std::size_t length) noexcept {
        return (src.value >> 16) + (src.value & 0xFFFF) + (dst.value >> 16) + (dst.value & 0xFFFF) +
               ip_proto_tcp + static_cast<std::uint32_t>(length);
    }

private:
    struct Listener {
        std::uint16_t port = 0;
        InterfaceSet on = InterfaceSet::none();
        const TcpHandler* handler = nullptr;
    };

    struct Incoming {
        Ipv4Address src;
        Ipv4Address dst;
        InterfaceId interface;
        std::uint16_t src_port, dst_port;
        std::uint32_t seq, ack;
        std::uint8_t flags;
        std::uint16_t window;
        std::uint16_t mss;
        std::size_t payload_len;
        [[nodiscard]] std::uint32_t seq_length() const noexcept {
            return static_cast<std::uint32_t>(payload_len) + ((flags & tcp_flag::syn) ? 1 : 0) +
                   ((flags & tcp_flag::fin) ? 1 : 0);
        }
    };

    static std::uint16_t parse_mss(const std::byte* h, std::size_t header_len) noexcept {
        std::size_t i = tcp_header_size;
        while (i < header_len) {
            const auto kind = std::to_integer<std::uint8_t>(h[i]);
            if (kind == 0) {
                break;
            }
            if (kind == 1) {
                ++i;
                continue;
            }
            if (i + 1 >= header_len) {
                break;
            }
            const auto len = std::to_integer<std::size_t>(h[i + 1]);
            if (len < 2 || i + len > header_len) {
                break;
            }
            if (kind == 2 && len == 4) {
                return load_be16(h + i + 2);
            }
            i += len;
        }
        return 536;  // RFC 1122 default
    }

    Connection* find(Ipv4Address ip, std::uint16_t remote_port, std::uint16_t local_port) noexcept {
        for (auto& c : connections_) {
            if (c.state_ != TcpState::Closed && c.remote_ip_ == ip && c.remote_port_ == remote_port &&
                c.local_port_ == local_port) {
                return &c;
            }
        }
        return nullptr;
    }

    const TcpHandler* listener_for(std::uint16_t port, InterfaceId interface) const noexcept {
        for (const auto& l : listeners_) {
            if (l.handler != nullptr && l.port == port && l.on.contains(interface)) {
                return l.handler;
            }
        }
        return nullptr;
    }

    void no_connection(const Incoming& seg) noexcept {
        if (seg.flags & tcp_flag::rst) {
            return;
        }
        const TcpHandler* handler = listener_for(seg.dst_port, seg.interface);
        if (handler != nullptr && (seg.flags & tcp_flag::syn) && !(seg.flags & tcp_flag::ack)) {
            if (Connection* c = free_slot(); c != nullptr) {
                open(*c, seg, handler);
                return;
            }
        }
        send_reset(seg);
    }

    /// A closed connection, else the oldest half-open one (like lwIP): otherwise a
    /// few SYNs that are never completed (a SYN flood, a scan) would hold every slot
    /// for the minutes of SYN-ACK retransmissions.
    Connection* free_slot() noexcept {
        Connection* oldest = nullptr;
        for (auto& c : connections_) {
            if (c.state_ == TcpState::Closed) {
                return &c;
            }
            if (c.state_ == TcpState::SynReceived &&
                (oldest == nullptr || static_cast<std::int32_t>(c.opened_ms_ - oldest->opened_ms_) < 0)) {
                oldest = &c;
            }
        }
        if (oldest != nullptr) {
            ++syn_recycled_;
            oldest->reset_state();  // never reported to the application
        }
        return oldest;
    }

    void open(Connection& c, const Incoming& seg, const TcpHandler* handler) noexcept {
        c.reset_state();
        c.listener_ = handler;
        c.state_ = TcpState::SynReceived;
        c.opened_ms_ = ip_.now_ms();
        c.remote_ip_ = seg.src;
        c.local_ip_ = seg.dst;
        c.remote_port_ = seg.src_port;
        c.local_port_ = seg.dst_port;
        c.mss_ = std::clamp(seg.mss, min_peer_mss, local_mss);
        c.tx_window_limit_ = static_cast<std::uint32_t>(tx_segments) * c.mss_;
        c.rcv_nxt_ = seg.seq + 1;
        c.iss_ = ip_.random32();
        c.snd_una_ = c.iss_;
        c.snd_nxt_ = c.iss_;
        c.snd_wnd_ = seg.window;
        // SYN-ACK with MSS option
        PacketRef p = ip_.allocate();
        if (!p) {
            c.reset_state();
            return;
        }
        std::byte* opt = p->append(4);
        opt[0] = std::byte{2};
        opt[1] = std::byte{4};
        store_be16(opt + 2, local_mss);
        queue_segment(c, std::move(p), tcp_flag::syn | tcp_flag::ack, 1, 4);
    }

    void process(Connection& c, Incoming seg, PacketRef segment) noexcept {
        if (c.state_ == TcpState::SynReceived) {
            if (seg.flags & tcp_flag::rst) {
                c.reset_state();
                return;
            }
            if ((seg.flags & tcp_flag::syn) && seg.seq + 1 == c.rcv_nxt_) {
                retransmit_oldest(c);  // our SYN-ACK got lost
                return;
            }
            if (!(seg.flags & tcp_flag::ack) || seg.ack != c.iss_ + 1) {
                send_reset(seg);
                return;
            }
            c.state_ = TcpState::Established;
            if (c.listener_->accepted != nullptr) {
                c.listener_->accepted(c.listener_->context, c);
            }
            if (c.state_ == TcpState::Closed) {
                return;  // the application aborted
            }
            acknowledge(c, seg);  // releases the SYN-ACK; may report "writable"
            if (seg.payload_len == 0 && !(seg.flags & tcp_flag::fin)) {
                return;
            }
        }

        // A retransmission that starts in data we already have but carries new data
        // too (peers repacketize, lwIP test_tcp_recv_inseq_trim): drop the old part.
        if (seq_lt(seg.seq, c.rcv_nxt_) && !(seg.flags & (tcp_flag::syn | tcp_flag::rst))) {
            const std::uint32_t old = c.rcv_nxt_ - seg.seq;
            if (old < seg.payload_len || (old == seg.payload_len && (seg.flags & tcp_flag::fin))) {
                segment->pull_front(old);
                seg.payload_len -= old;
                seg.seq = c.rcv_nxt_;
            }
        }

        // Sequence check (RFC 793 3.9, simplified to in-order acceptance).
        const std::uint32_t seq_len = seg.seq_length();
        if (seg.seq != c.rcv_nxt_) {
            if (seg.flags & tcp_flag::rst) {
                // RFC 5961 3.2: inside the window but not exact -> challenge ACK, else ignore.
                if (seq_lt(c.rcv_nxt_, seg.seq) && seq_lt(seg.seq, c.rcv_nxt_ + receive_window(c))) {
                    c.ack_pending_ = true;
                }
                return;
            }
            if (seq_len != 0 || seq_lt(seg.seq, c.rcv_nxt_)) {
                c.ack_pending_ = true;  // duplicate or out of order: tell the peer where we are
            }
            if (seg.flags & tcp_flag::ack) {
                acknowledge(c, seg);
            }
            return;
        }
        if (seg.flags & tcp_flag::rst) {
            terminate(c, TcpCloseReason::Reset);
            return;
        }
        if (seg.flags & tcp_flag::syn) {
            c.ack_pending_ = true;  // challenge ACK
            return;
        }
        if (seg.flags & tcp_flag::ack) {
            if (seq_lt(c.snd_nxt_, seg.ack)) {
                c.ack_pending_ = true;  // acknowledges data never sent: ACK and drop (RFC 793, 5961 5.2)
                return;
            }
            acknowledge(c, seg);
            if (c.state_ == TcpState::Closed) {
                return;
            }
        }

        bool notify = false;
        if (seg.payload_len != 0 && accepts_data(c.state_)) {
            if (c.rx_segments_ >= rx_segments) {
                ++rx_overruns_;
                c.ack_pending_ = true;  // drop; the peer retransmits once we have read
                return;
            }
            PacketBuffer* b = segment.release();
            b->next = nullptr;
            if (c.rx_tail_ != nullptr) {
                c.rx_tail_->next = b;
            } else {
                c.rx_head_ = b;
            }
            c.rx_tail_ = b;
            ++c.rx_segments_;
            c.rx_bytes_ += seg.payload_len;
            c.rcv_nxt_ += static_cast<std::uint32_t>(seg.payload_len);
            c.ack_pending_ = true;
            notify = true;
        }

        if ((seg.flags & tcp_flag::fin) && !c.fin_received_) {
            c.fin_received_ = true;
            c.rcv_nxt_ += 1;
            c.ack_pending_ = true;
            notify = true;
            switch (c.state_) {
                case TcpState::Established: c.state_ = TcpState::CloseWait; break;
                case TcpState::FinWait1: c.state_ = TcpState::Closing; break;
                case TcpState::FinWait2: enter_time_wait(c); break;
                default: break;
            }
        }
        if (notify && c.listener_->received != nullptr) {
            c.listener_->received(c.listener_->context, c);
        }
    }

    static bool accepts_data(TcpState s) noexcept {
        return s == TcpState::Established || s == TcpState::FinWait1 || s == TcpState::FinWait2;
    }

    /// ACK and window processing.
    void acknowledge(Connection& c, const Incoming& seg) noexcept {
        if (seq_lt(c.snd_una_, seg.ack) && seq_le(seg.ack, c.snd_nxt_)) {
            const std::uint32_t now = ip_.now_ms();
            // Release fully acknowledged segments; take one RTT sample (Karn).
            while (c.tx_count_ != 0) {
                auto& s = c.tx_[c.tx_head_];
                if (!seq_le(s.seq + s.seq_length, seg.ack)) {
                    break;
                }
                if (!s.retransmitted) {
                    update_rtt(c, now - s.sent_ms);
                }
                c.drop(s);
                c.tx_head_ = (c.tx_head_ + 1) % c.tx_.size();
                --c.tx_count_;
            }
            c.snd_una_ = seg.ack;
            c.retries_ = 0;
            c.deadline_ms_ = c.tx_count_ != 0 ? now + c.rto_ms_ : 0;
            c.snd_wnd_ = seg.window;

            const bool fin_acked = c.fin_sent_ && seg.ack == c.snd_nxt_;
            if (fin_acked) {
                switch (c.state_) {
                    case TcpState::FinWait1:
                        // Our side is done; a peer that never sends its FIN must not hold
                        // the slot forever (like Linux tcp_fin_timeout).
                        c.state_ = TcpState::FinWait2;
                        c.deadline_ms_ = now + fin_wait2_ms;
                        break;
                    case TcpState::Closing: enter_time_wait(c); return;
                    case TcpState::LastAck: terminate(c, TcpCloseReason::Normal); return;
                    default: break;
                }
            }
            if (c.can_send() && c.writable() != 0 && c.listener_->writable != nullptr) {
                c.listener_->writable(c.listener_->context, c);
            }
        } else if (seg.ack == c.snd_una_) {
            // Pure window update (or the answer to a window probe).
            const bool opened = seg.window > c.snd_wnd_;
            c.snd_wnd_ = seg.window;
            if (opened && c.can_send() && c.writable() != 0 && c.listener_->writable != nullptr) {
                c.listener_->writable(c.listener_->context, c);
            }
        }
        update_persist(c);
    }

    /// A bulk connection finished its turn since the last pass: tell the waiting ones
    /// that can send now. They have nothing in flight - no ACK of their own would
    /// ever wake them.
    void wake_bulk_waiters() noexcept {
        if (bulk_.active < bulk_reported_) {
            for (auto& c : connections_) {
                if (c.bulk_ && c.can_send() && c.writable() != 0 && c.listener_ != nullptr &&
                    c.listener_->writable != nullptr) {
                    c.listener_->writable(c.listener_->context, c);
                }
            }
        }
        bulk_reported_ = bulk_.active;
    }

    /// Start the persist timer when the peer's window is closed and nothing is in
    /// flight (no retransmission would ever reveal a reopened window); stop it otherwise.
    void update_persist(Connection& c) noexcept {
        if (c.snd_wnd_ != 0 || c.tx_count_ != 0 || !c.can_send()) {
            c.persist_ms_ = 0;
        } else if (c.persist_ms_ == 0) {
            c.persist_interval_ms_ = c.rto_ms_;
            c.persist_ms_ = ip_.now_ms() + c.persist_interval_ms_;
        }
    }

    void probe_window(Connection& c, std::uint32_t now) noexcept {
        update_persist(c);
        if (c.persist_ms_ == 0) {
            return;
        }
        PacketRef p = ip_.allocate();
        if (p) {
            p->push_front(tcp_header_size);
            write_header(c, *p, c.snd_nxt_ - 1, tcp_flag::ack, tcp_header_size);  // already acked: elicits an ACK
            c.ack_pending_ = false;
            c.window_update_pending_ = false;
            ip_.send_ipv4(std::move(p), c.remote_ip_, ip_proto_tcp, c.local_ip_);
        }
        c.persist_interval_ms_ = std::min(c.persist_interval_ms_ * 2, rto_max_ms);
        c.persist_ms_ = now + c.persist_interval_ms_;
    }

    void update_rtt(Connection& c, std::uint32_t sample) noexcept {
        // RFC 6298 (alpha = 1/8, beta = 1/4), integer arithmetic in ms.
        if (!c.rtt_valid_) {
            c.rtt_valid_ = true;
            c.srtt_ms_ = sample;
            c.rttvar_ms_ = sample / 2;
        } else {
            const std::uint32_t err = sample > c.srtt_ms_ ? sample - c.srtt_ms_ : c.srtt_ms_ - sample;
            c.rttvar_ms_ = (3 * c.rttvar_ms_ + err) / 4;
            c.srtt_ms_ = (7 * c.srtt_ms_ + sample) / 8;
        }
        c.rto_ms_ = std::clamp<std::uint32_t>(c.srtt_ms_ + std::max<std::uint32_t>(1, 4 * c.rttvar_ms_),
                                              rto_min_ms, rto_max_ms);
    }

    bool send_data(Connection& c, PacketRef payload) noexcept {
        contract_assert(payload && payload->size() <= c.mss_);
        if (payload->size() > c.writable() || c.tx_count_ >= tx_segments) {
            return false;
        }
        const auto len = static_cast<std::uint16_t>(payload->size());
        queue_segment(c, std::move(payload), tcp_flag::ack | tcp_flag::psh, len, 0);
        return true;
    }

    void send_fin(Connection& c) noexcept {
        PacketRef p = ip_.allocate();
        if (!p) {
            return;  // retried in the next service() pass
        }
        c.fin_sent_ = true;
        c.state_ = c.state_ == TcpState::Established ? TcpState::FinWait1 : TcpState::LastAck;
        queue_segment(c, std::move(p), tcp_flag::fin | tcp_flag::ack, 1, 0);
    }

    /// Prepend the TCP header to `p` (payload + `option_len` option bytes at data()),
    /// put it into the retransmission ring and send it.
    void queue_segment(Connection& c, PacketRef p, std::uint8_t flags, std::uint16_t seq_len,
                       std::uint16_t option_len) noexcept {
        p->push_front(tcp_header_size);
        auto& s = c.tx_[(c.tx_head_ + c.tx_count_) % c.tx_.size()];
        ++c.tx_count_;
        s.seq = c.snd_nxt_;
        s.flags = flags;
        s.seq_length = seq_len;
        s.tcp_offset = static_cast<std::uint16_t>(p->offset());
        s.tcp_length = static_cast<std::uint16_t>(p->size());
        s.retransmitted = false;
        write_header(c, *p, s.seq, flags, static_cast<std::uint16_t>(tcp_header_size + option_len));
        c.snd_nxt_ += seq_len;
        s.sent_ms = ip_.now_ms();
        s.packet = p;
        s.bulk = c.bulk_;
        if (s.bulk && c.bulk_in_flight_++ == 0) {
            ++bulk_.active;
        }
        if (c.deadline_ms_ == 0) {
            c.deadline_ms_ = s.sent_ms + c.rto_ms_;
        }
        c.ack_pending_ = false;  // piggybacked
        c.window_update_pending_ = false;
        ip_.send_ipv4(std::move(p), c.remote_ip_, ip_proto_tcp, c.local_ip_);
    }

    std::uint16_t receive_window(const Connection& c) const noexcept {
        const std::size_t budget = rx_segments * c.mss_;
        return static_cast<std::uint16_t>(std::min<std::size_t>(budget > c.rx_bytes_ ? budget - c.rx_bytes_ : 0, 0xFFFF));
    }

    void write_header(const Connection& c, PacketBuffer& b, std::uint32_t seq, std::uint8_t flags,
                      std::uint16_t header_len) noexcept {
        std::byte* h = b.data();
        store_be16(h, c.local_port_);
        store_be16(h + 2, c.remote_port_);
        store_be32(h + 4, seq);
        store_be32(h + 8, (flags & tcp_flag::ack) ? c.rcv_nxt_ : 0);
        h[12] = static_cast<std::byte>((header_len / 4) << 4);
        h[13] = static_cast<std::byte>(flags);
        store_be16(h + 14, receive_window(c));
        store_be16(h + 16, 0);
        store_be16(h + 18, 0);
        const std::uint16_t sum =
            checksum_finish(checksum_add(b.bytes(), pseudo_header_sum(c.local_ip_, c.remote_ip_, b.size())));
        store_be16(h + 16, sum);
    }

    void retransmit_oldest(Connection& c) noexcept {
        if (c.tx_count_ == 0) {
            return;
        }
        auto& s = c.tx_[c.tx_head_];
        if (s.packet->use_count() > 1) {
            c.deadline_ms_ = ip_.now_ms() + 10;  // still in the TX DMA: retry shortly
            return;
        }
        s.packet->set_view(s.tcp_offset, s.tcp_length);
        const std::uint16_t header_len =
            static_cast<std::uint16_t>((std::to_integer<std::uint16_t>(s.packet->data()[12]) >> 4) * 4);
        write_header(c, *s.packet, s.seq, s.flags, header_len);
        s.retransmitted = true;
        s.sent_ms = ip_.now_ms();
        c.ack_pending_ = false;
        ip_.send_ipv4(s.packet, c.remote_ip_, ip_proto_tcp, c.local_ip_);
    }

    void expire(Connection& c) noexcept {
        if (c.state_ == TcpState::TimeWait) {
            terminate(c, TcpCloseReason::Normal);
            return;
        }
        if (c.state_ == TcpState::FinWait2) {
            abort(c, TcpCloseReason::Timeout);
            return;
        }
        if (c.tx_count_ == 0) {
            c.deadline_ms_ = 0;
            return;
        }
        if (++c.retries_ > max_retries) {
            abort(c, TcpCloseReason::Timeout);
            return;
        }
        c.rto_ms_ = std::min(c.rto_ms_ * 2, rto_max_ms);
        c.deadline_ms_ = ip_.now_ms() + c.rto_ms_;
        retransmit_oldest(c);
    }

    void enter_time_wait(Connection& c) noexcept {
        c.state_ = TcpState::TimeWait;
        for (auto& s : c.tx_) {
            c.drop(s);
        }
        c.tx_count_ = 0;
        c.deadline_ms_ = ip_.now_ms() + time_wait_ms;
    }

    void abort(Connection& c, TcpCloseReason reason) noexcept {
        if (c.state_ == TcpState::Closed) {
            return;
        }
        send_control(c, tcp_flag::rst | tcp_flag::ack);
        terminate(c, reason);
    }

    void terminate(Connection& c, TcpCloseReason reason) noexcept {
        const TcpHandler* handler = c.listener_;
        const bool was_open = c.state_ != TcpState::SynReceived;
        c.state_ = TcpState::Closed;
        if (was_open && handler != nullptr && handler->closed != nullptr) {
            handler->closed(handler->context, c, reason);
        }
        c.reset_state();
    }

    /// Pure ACK / RST (not retransmitted).
    void send_control(Connection& c, std::uint8_t flags) noexcept {
        PacketRef p = ip_.allocate();
        if (!p) {
            return;  // ack_pending_ stays set: next pass
        }
        p->push_front(tcp_header_size);
        write_header(c, *p, c.snd_nxt_, flags, tcp_header_size);
        c.ack_pending_ = false;
        c.window_update_pending_ = false;
        ip_.send_ipv4(std::move(p), c.remote_ip_, ip_proto_tcp, c.local_ip_);
    }

    /// RST for a segment without connection (RFC 793 "reset generation").
    void send_reset(const Incoming& seg) noexcept {
        PacketRef p = ip_.allocate();
        if (!p) {
            return;
        }
        std::byte* h = p->push_front(tcp_header_size);
        const bool has_ack = seg.flags & tcp_flag::ack;
        store_be16(h, seg.dst_port);
        store_be16(h + 2, seg.src_port);
        store_be32(h + 4, has_ack ? seg.ack : 0);
        store_be32(h + 8, has_ack ? 0 : seg.seq + seg.seq_length());
        h[12] = static_cast<std::byte>((tcp_header_size / 4) << 4);
        h[13] = static_cast<std::byte>(has_ack ? tcp_flag::rst : (tcp_flag::rst | tcp_flag::ack));
        store_be16(h + 14, 0);
        store_be16(h + 16, 0);
        store_be16(h + 18, 0);
        store_be16(h + 16, checksum_finish(checksum_add(p->bytes(), pseudo_header_sum(seg.dst, seg.src, p->size()))));
        ip_.send_ipv4(std::move(p), seg.src, ip_proto_tcp, seg.dst);
    }

    Ip& ip_;
    std::array<Connection, max_connections> connections_{};
    std::array<Listener, max_listeners> listeners_{};
    std::uint32_t checksum_errors_ = 0;
    std::uint32_t rx_overruns_ = 0;
    std::uint32_t syn_recycled_ = 0;
    TcpBulkBudget bulk_{static_cast<std::uint32_t>(bulk_connections), 0};
    std::uint32_t bulk_reported_ = 0;  ///< bulk_.active at the last wake_bulk_waiters()
};

}  // namespace rtos::net
