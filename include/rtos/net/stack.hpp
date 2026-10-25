// SPDX-License-Identifier: MIT
//
// rtos::net::Stack - single-threaded IPv4 stack: Ethernet, ARP, IPv4, ICMP echo,
// UDP and TCP (tcp.hpp).
//
// Execution model: everything runs in one thread that calls poll(now_ms) whenever
// the driver signals a frame or next_deadline() is reached. No locks, no
// allocation; the only shared resource is the lock-free PacketPool.
//
// Zero copy: received frames are processed in their DMA buffer (headers stripped
// in place); ICMP echo replies reuse the request buffer. Outgoing data is written
// once into a buffer with `headroom` bytes in front, every layer prepends its
// header in place, and the driver DMAs from that buffer.
//
// Addressing: static (constructor) or at run time through set_config() (DHCP,
// dhcp.hpp). Unconfigured (address 0.0.0.0) the stack sends with source 0.0.0.0,
// receives broadcasts, and answers neither ARP nor ICMP.
//
// Buffer budget: the packet pool is shared by the driver's receive ring and every
// sender. Senders (allocate(): UDP, TCP data and control, ARP) never take the last
// Cfg::rx_reserve buffers, so received frames - among them the ACKs that release
// transmit buffers - always find a fresh DMA buffer. Without the reserve, transmit
// segments in flight on all connections could empty the pool, every received ACK
// would be dropped, and the connections would stall until they time out.
//
// Not implemented: IP fragmentation/reassembly (fragments are dropped; TCP MSS and
// the 512-byte WebSocket messages never need it), IP options, IPv6.
#pragma once

#include <array>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/net/address.hpp"
#include "rtos/net/bytes.hpp"
#include "rtos/net/checksum.hpp"
#include "rtos/net/driver.hpp"
#include "rtos/net/packet.hpp"
#include "rtos/net/tcp.hpp"

namespace rtos::net {

inline constexpr std::size_t ethernet_header_size = 14;
inline constexpr std::size_t ipv4_header_size = 20;
inline constexpr std::size_t udp_header_size = 8;
inline constexpr std::uint16_t ethertype_ipv4 = 0x0800;
inline constexpr std::uint16_t ethertype_arp = 0x0806;
inline constexpr std::uint8_t ip_proto_icmp = 1;
inline constexpr std::uint8_t ip_proto_udp = 17;

struct DefaultStackConfig {
    static constexpr std::size_t arp_entries = 8;
    static constexpr std::size_t udp_sockets = 4;
    static constexpr std::size_t multicast_groups = 2;
    static constexpr std::size_t tcp_connections = 4;
    static constexpr std::size_t tcp_listeners = 2;
    static constexpr std::size_t tcp_tx_segments = 4;
    static constexpr std::size_t tcp_rx_segments = 4;
    static constexpr std::uint16_t tcp_mss = 1460;
    static constexpr std::uint32_t tcp_time_wait_ms = 2000;
    /// Pool buffers only the receive path may take (see "Buffer budget").
    static constexpr std::size_t rx_reserve = 8;
    /// FIN_WAIT_2 without the peer's FIN: give up (RST) after this time.
    static constexpr std::uint32_t tcp_fin_wait2_ms = 15'000;
};

/// A received datagram; `payload` is a view into the receive buffer.
struct UdpDatagram {
    Ipv4Address source;
    std::uint16_t source_port;
    Ipv4Address destination;
    std::uint16_t destination_port;
    std::span<const std::byte> payload;
};

struct UdpHandler {
    void* context = nullptr;
    void (*received)(void* context, const UdpDatagram& datagram) noexcept = nullptr;
};

struct StackStats {
    std::uint32_t rx_frames = 0;
    std::uint32_t tx_frames = 0;
    std::uint32_t rx_dropped = 0;
    std::uint32_t tx_dropped = 0;
    std::uint32_t arp_requests = 0;
    std::uint32_t tx_no_buffer = 0;  ///< sends deferred: pool down to the receive reserve
};

template <NetDriver Driver, PacketAllocator Pool, class Cfg = DefaultStackConfig>
class Stack {
public:
    /// Bytes reserved in front of every outgoing payload: Ethernet + IPv4 + TCP
    /// (+4 for the MSS option), rounded up so the payload starts 4-byte aligned.
    static constexpr std::size_t headroom = 64;
    static constexpr std::uint32_t arp_timeout_ms = 300'000;
    static constexpr std::uint32_t arp_retry_ms = 1000;

    using TcpLayer = Tcp<Stack, Cfg>;
    using TcpConnection = typename TcpLayer::Connection;

    Stack(Driver& driver, Pool& pool, const Ipv4Config& config) noexcept
        : driver_(driver), pool_(pool), config_(config), mac_(driver.mac()), tcp_(*this) {}

    Stack(const Stack&) = delete;
    Stack& operator=(const Stack&) = delete;

    /// Process all received frames and expired timers.
    void poll(std::uint32_t now_ms) noexcept {
        now_ms_ = now_ms;
        while (PacketRef frame = driver_.receive()) {
            ++stats_.rx_frames;
            input(std::move(frame));
        }
        service_arp();
        tcp_.service();
    }

    /// Earliest point in time poll() must run again without new frames (0 = none).
    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = tcp_.next_deadline();
        for (const auto& e : arp_) {
            if (e.pending) {
                const std::uint32_t t = e.requested_ms + arp_retry_ms;
                if (best == 0 || static_cast<std::int32_t>(t - best) < 0) {
                    best = t;
                }
            }
        }
        return best;
    }

    // ---- configuration ----------------------------------------------------------

    [[nodiscard]] Ipv4Address local_address() const noexcept { return config_.address; }
    [[nodiscard]] const Ipv4Config& config() const noexcept { return config_; }
    [[nodiscard]] bool configured() const noexcept { return config_.address != Ipv4Address::any(); }

    /// New address/netmask/gateway (DHCP lease, fallback, 0.0.0.0 = unconfigured).
    /// A changed address or subnet invalidates the ARP cache.
    void set_config(const Ipv4Config& config) noexcept {
        if (config.address != config_.address || config.netmask != config_.netmask) {
            for (auto& e : arp_) {
                e.parked.reset();
                e = ArpEntry{};
            }
        }
        config_ = config;
    }
    [[nodiscard]] MacAddress mac() const noexcept { return mac_; }
    [[nodiscard]] std::uint32_t now_ms() const noexcept { return now_ms_; }
    [[nodiscard]] const StackStats& stats() const noexcept { return stats_; }

    /// Source of initial sequence numbers (the board supplies a hardware RNG).
    void set_random(std::uint32_t (*random)(void* context) noexcept, void* context) noexcept {
        random_ = random;
        random_context_ = context;
    }
    [[nodiscard]] std::uint32_t random32() noexcept {
        return random_ != nullptr ? random_(random_context_) : (lcg_ = lcg_ * 1664525u + 1013904223u);
    }

    /// Receive IPv4 traffic for multicast `group` (the driver passes all multicast).
    bool join(Ipv4Address group) noexcept {
        contract_assert(group.is_multicast());
        for (auto& g : groups_) {
            if (g == group) {
                return true;
            }
        }
        for (auto& g : groups_) {
            if (g == Ipv4Address::any()) {
                g = group;
                return true;
            }
        }
        return false;
    }

    // ---- buffers ------------------------------------------------------------------

    static constexpr std::size_t rx_reserve = Cfg::rx_reserve;

    /// Buffer for an outgoing payload (headroom for all headers reserved). Empty if
    /// the pool is down to the receive reserve: callers retry later.
    [[nodiscard]] PacketRef allocate() noexcept { return allocate_tx(headroom); }

    // ---- UDP ----------------------------------------------------------------------

    bool udp_bind(std::uint16_t port, const UdpHandler& handler) noexcept {
        for (auto& s : udp_) {
            if (s.handler != nullptr && s.port == port) {
                return false;  // in use
            }
        }
        for (auto& s : udp_) {
            if (s.handler == nullptr) {
                s = {port, &handler};
                return true;
            }
        }
        return false;
    }

    void udp_unbind(std::uint16_t port) noexcept {
        for (auto& s : udp_) {
            if (s.handler != nullptr && s.port == port) {
                s = UdpSocket{};
            }
        }
    }

    /// Send `payload` (at data(), headroom reserved) as one datagram.
    bool udp_send(PacketRef payload, Ipv4Address destination, std::uint16_t destination_port,
                  std::uint16_t source_port) noexcept {
                      contract_assert(payload && payload->headroom() >= udp_header_size + ipv4_header_size + ethernet_header_size);
        PacketBuffer& b = *payload;
        std::byte* h = b.push_front(udp_header_size);
        store_be16(h, source_port);
        store_be16(h + 2, destination_port);
        store_be16(h + 4, static_cast<std::uint16_t>(b.size()));
        store_be16(h + 6, 0);
        const std::uint32_t pseudo = pseudo_sum(config_.address, destination, ip_proto_udp, b.size());
        std::uint16_t sum = checksum_finish(checksum_add(b.bytes(), pseudo));
        store_be16(h + 6, sum == 0 ? 0xFFFF : sum);  // 0 means "no checksum"
        return send_ipv4(std::move(payload), destination, ip_proto_udp);
    }

    // ---- TCP ----------------------------------------------------------------------

    [[nodiscard]] TcpLayer& tcp() noexcept { return tcp_; }

    // ---- IPv4 output (also used by TCP) -------------------------------------------

    /// Prepend the IPv4 header to `packet` (transport header at data()) and send it.
    bool send_ipv4(PacketRef packet, Ipv4Address destination, std::uint8_t protocol) noexcept {
        PacketBuffer& b = *packet;
        if (b.headroom() < ipv4_header_size + ethernet_header_size) {
            ++stats_.tx_dropped;
            return false;
        }
        std::byte* h = b.push_front(ipv4_header_size);
        h[0] = std::byte{0x45};  // version 4, IHL 5
        h[1] = std::byte{0};
        store_be16(h + 2, static_cast<std::uint16_t>(b.size()));
        store_be16(h + 4, ip_id_++);
        store_be16(h + 6, 0x4000);  // don't fragment
        h[8] = std::byte{destination.is_multicast() ? std::uint8_t{255} : std::uint8_t{64}};
        h[9] = std::byte{protocol};
        store_be16(h + 10, 0);
        config_.address.store(h + 12);
        destination.store(h + 16);
        store_be16(h + 10, checksum({h, ipv4_header_size}));

        if (destination.is_multicast()) {
            return send_frame(std::move(packet), destination.multicast_mac(), ethertype_ipv4);
        }
        if (destination.is_broadcast() || destination == config_.subnet_broadcast()) {
            return send_frame(std::move(packet), MacAddress::broadcast(), ethertype_ipv4);
        }
        const Ipv4Address next_hop = config_.on_link(destination) ? destination : config_.gateway;
        return resolve_and_send(std::move(packet), next_hop);
    }

    static std::uint32_t pseudo_sum(Ipv4Address src, Ipv4Address dst, std::uint8_t protocol, std::size_t length) noexcept {
        return (src.value >> 16) + (src.value & 0xFFFF) + (dst.value >> 16) + (dst.value & 0xFFFF) + protocol +
               static_cast<std::uint32_t>(length);
    }

private:
    struct ArpEntry {
        Ipv4Address ip{};
        MacAddress mac{};
        std::uint32_t updated_ms = 0;
        std::uint32_t requested_ms = 0;
        std::uint8_t attempts = 0;
        bool valid = false;
        bool pending = false;
        PacketRef parked;  ///< one packet waiting for the resolution
    };

    struct UdpSocket {
        std::uint16_t port = 0;
        const UdpHandler* handler = nullptr;
    };

    PacketRef allocate_tx(std::size_t front) noexcept {
        if (pool_.available() <= rx_reserve) {
            ++stats_.tx_no_buffer;
            return {};
        }
        return pool_.allocate(front);
    }

    // ---- input --------------------------------------------------------------------

    void input(PacketRef frame) noexcept {
        PacketBuffer& b = *frame;
        if (b.size() < ethernet_header_size) {
            ++stats_.rx_dropped;
            return;
        }
        const std::uint16_t type = load_be16(b.data() + 12);
        b.pull_front(ethernet_header_size);
        if (type == ethertype_arp) {
            arp_input(std::move(frame));
        } else if (type == ethertype_ipv4) {
            ipv4_input(std::move(frame));
        } else {
            ++stats_.rx_dropped;
        }
    }

    void arp_input(PacketRef packet) noexcept {
        PacketBuffer& b = *packet;
        if (b.size() < 28) {
            return;
        }
        std::byte* a = b.data();
        if (load_be16(a) != 1 || load_be16(a + 2) != ethertype_ipv4 || a[4] != std::byte{6} || a[5] != std::byte{4}) {
            return;
        }
        const std::uint16_t op = load_be16(a + 6);
        const MacAddress sender_mac = MacAddress::load(a + 8);
        const Ipv4Address sender_ip = Ipv4Address::load(a + 14);
        const Ipv4Address target_ip = Ipv4Address::load(a + 24);
        const bool for_us = configured() && target_ip == config_.address;
        if (for_us || arp_find(sender_ip) != nullptr) {
            arp_update(sender_ip, sender_mac);  // RFC 826: merge
        }
        if (op == 1 && for_us) {
            // Reply in place: turn the request around.
            store_be16(a + 6, 2);
            sender_mac.store(a + 18);
            sender_ip.store(a + 24);
            mac_.store(a + 8);
            config_.address.store(a + 14);
            b.truncate(28);
            send_frame(std::move(packet), sender_mac, ethertype_arp);
        }
    }

    void ipv4_input(PacketRef packet) noexcept {
        PacketBuffer& b = *packet;
        if (b.size() < ipv4_header_size) {
            ++stats_.rx_dropped;
            return;
        }
        const std::byte* h = b.data();
        const std::size_t header_len = (std::to_integer<std::size_t>(h[0]) & 0x0F) * 4;
        const std::size_t total = load_be16(h + 2);
        if ((std::to_integer<std::uint8_t>(h[0]) >> 4) != 4 || header_len < ipv4_header_size || total < header_len ||
            total > b.size() || checksum({h, header_len}) != 0) {
            ++stats_.rx_dropped;
            return;
        }
        if ((load_be16(h + 6) & 0x3FFF) != 0) {  // MF set or fragment offset != 0
            ++stats_.rx_dropped;
            return;
        }
        const Ipv4Address src = Ipv4Address::load(h + 12);
        const Ipv4Address dst = Ipv4Address::load(h + 16);
        if (!accepts(dst)) {
            return;
        }
        const std::uint8_t protocol = std::to_integer<std::uint8_t>(h[9]);
        b.truncate(total);  // strip Ethernet padding
        b.pull_front(header_len);
        switch (protocol) {
            case ip_proto_icmp: icmp_input(std::move(packet), src, dst); break;
            case ip_proto_udp: udp_input(std::move(packet), src, dst); break;
            case ip_proto_tcp: tcp_.input(std::move(packet), src, dst); break;
            default: ++stats_.rx_dropped; break;
        }
    }

    [[nodiscard]] bool accepts(Ipv4Address dst) const noexcept {
        if (dst == config_.address || dst.is_broadcast() || dst == config_.subnet_broadcast()) {
            return true;
        }
        if (dst.is_multicast()) {
            for (const auto& g : groups_) {
                if (g == dst) {
                    return true;
                }
            }
        }
        return false;
    }

    void icmp_input(PacketRef packet, Ipv4Address src, Ipv4Address dst) noexcept {
        PacketBuffer& b = *packet;
        if (b.size() < 8 || checksum(b.bytes()) != 0 || !configured() || dst != config_.address) {
            return;
        }
        std::byte* m = b.data();
        if (m[0] != std::byte{8}) {
            return;  // echo request only
        }
        // Echo reply in place: type 8 -> 0, checksum recomputed over the same bytes.
        m[0] = std::byte{0};
        store_be16(m + 2, 0);
        store_be16(m + 2, checksum(b.bytes()));
        send_ipv4(std::move(packet), src, ip_proto_icmp);
    }

    void udp_input(PacketRef packet, Ipv4Address src, Ipv4Address dst) noexcept {
        PacketBuffer& b = *packet;
        if (b.size() < udp_header_size) {
            return;
        }
        const std::byte* h = b.data();
        const std::size_t length = load_be16(h + 4);
        if (length < udp_header_size || length > b.size()) {
            ++stats_.rx_dropped;
            return;
        }
        b.truncate(length);
        if (load_be16(h + 6) != 0 &&
            checksum_finish(checksum_add(b.bytes(), pseudo_sum(src, dst, ip_proto_udp, length))) != 0) {
            ++stats_.rx_dropped;
            return;
        }
        const std::uint16_t dst_port = load_be16(h + 2);
        for (const auto& s : udp_) {
            if (s.handler != nullptr && s.port == dst_port) {
                const UdpDatagram d{src, load_be16(h), dst, dst_port,
                                    std::span<const std::byte>(h + udp_header_size, length - udp_header_size)};
                s.handler->received(s.handler->context, d);
                return;
            }
        }
    }

    // ---- output -------------------------------------------------------------------

    bool send_frame(PacketRef packet, const MacAddress& destination, std::uint16_t type) noexcept {
        PacketBuffer& b = *packet;
        std::byte* h = b.push_front(ethernet_header_size);
        destination.store(h);
        mac_.store(h + 6);
        store_be16(h + 12, type);
        if (!driver_.transmit(packet)) {
            ++stats_.tx_dropped;
            return false;
        }
        ++stats_.tx_frames;
        return true;
    }

    bool resolve_and_send(PacketRef packet, Ipv4Address next_hop) noexcept {
        if (ArpEntry* e = arp_find(next_hop); e != nullptr && e->valid) {
            return send_frame(std::move(packet), e->mac, ethertype_ipv4);
        }
        ArpEntry& e = arp_slot(next_hop);
        e.parked = std::move(packet);  // a newer packet replaces an older one
        if (!e.pending) {
            e.pending = true;
            e.attempts = 0;
            arp_request(e);
        }
        return true;
    }

    // ---- ARP cache ------------------------------------------------------------------

    ArpEntry* arp_find(Ipv4Address ip) noexcept {
        for (auto& e : arp_) {
            if ((e.valid || e.pending) && e.ip == ip) {
                return &e;
            }
        }
        return nullptr;
    }

    /// Existing entry for `ip`, else a free one, else the least recently updated.
    ArpEntry& arp_slot(Ipv4Address ip) noexcept {
        if (ArpEntry* e = arp_find(ip)) {
            return *e;
        }
        ArpEntry* victim = &arp_[0];
        for (auto& e : arp_) {
            if (!e.valid && !e.pending) {
                victim = &e;
                break;
            }
            if (static_cast<std::int32_t>(e.updated_ms - victim->updated_ms) < 0) {
                victim = &e;
            }
        }
        victim->parked.reset();
        *victim = ArpEntry{};
        victim->ip = ip;
        return *victim;
    }

    void arp_update(Ipv4Address ip, const MacAddress& mac) noexcept {
        ArpEntry& e = arp_slot(ip);
        e.mac = mac;
        e.valid = true;
        e.pending = false;
        e.updated_ms = now_ms_;
        if (e.parked) {
            send_frame(std::move(e.parked), mac, ethertype_ipv4);
        }
    }

    void arp_request(ArpEntry& e) noexcept {
        e.requested_ms = now_ms_;
        ++e.attempts;
        ++stats_.arp_requests;
        PacketRef p = allocate_tx(ethernet_header_size);
        if (!p) {
            return;
        }
        std::byte* a = p->append(28);
        store_be16(a, 1);
        store_be16(a + 2, ethertype_ipv4);
        a[4] = std::byte{6};
        a[5] = std::byte{4};
        store_be16(a + 6, 1);
        mac_.store(a + 8);
        config_.address.store(a + 14);
        MacAddress{}.store(a + 18);
        e.ip.store(a + 24);
        send_frame(std::move(p), MacAddress::broadcast(), ethertype_arp);
    }

    void service_arp() noexcept {
        for (auto& e : arp_) {
            if (e.pending && static_cast<std::int32_t>(now_ms_ - (e.requested_ms + arp_retry_ms)) >= 0) {
                if (e.attempts >= 3) {
                    e.parked.reset();
                    e = ArpEntry{};
                } else {
                    arp_request(e);
                }
            } else if (e.valid && now_ms_ - e.updated_ms > arp_timeout_ms) {
                e = ArpEntry{};
            }
        }
    }

    Driver& driver_;
    Pool& pool_;
    Ipv4Config config_;
    MacAddress mac_;
    TcpLayer tcp_;
    std::array<ArpEntry, Cfg::arp_entries> arp_{};
    std::array<UdpSocket, Cfg::udp_sockets> udp_{};
    std::array<Ipv4Address, Cfg::multicast_groups> groups_{};
    StackStats stats_{};
    std::uint32_t now_ms_ = 0;
    std::uint16_t ip_id_ = 0;
    std::uint32_t (*random_)(void*) noexcept = nullptr;
    void* random_context_ = nullptr;
    std::uint32_t lcg_ = 0x12345678u;
};

}  // namespace rtos::net
