// SPDX-License-Identifier: MIT
//
// rtos::net::Stack - single-threaded IPv4 stack for one or more interfaces:
// Ethernet, ARP, IPv4, ICMP echo, UDP and TCP (tcp.hpp).
//
// Execution model: everything runs in one thread that calls poll(now_ms) whenever
// a driver signals a frame or next_deadline() is reached. No locks, no allocation;
// the only shared resource is the lock-free PacketPool.
//
// Interfaces: attach() adds a driver (any NetDriver: the STM32H5 Ethernet MAC, later
// e.g. USB CDC-NCM) with its own address, ARP cache, multicast groups and link
// state. TCP, UDP, the packet pool and the services on top are shared.
//   * Routing: a destination on the link of a configured interface goes there,
//     anything else to the first configured interface with a gateway (attach order
//     = priority). Broadcasts and multicasts name their interface (send_ipv4_on,
//     udp_send_on).
//   * Strong host model: a packet with a given source address (TCP connections,
//     replies) leaves through the interface that owns that address; an interface
//     accepts only its own unicast address, broadcasts and joined groups.
//   * Events (subscribe()): link up/down, address changed, address conflict - DHCP,
//     mDNS and the application react without polling.
//
// Address claiming (RFC 5227): claim_address() probes a new address with ARP
// (sender 0.0.0.0) before using it, then announces it; afterwards a conflicting ARP
// from another station is answered once (defence) and gives the address up if it
// repeats within DEFEND_INTERVAL. set_config() takes an address without probing.
//
// Zero copy: received frames are processed in their DMA buffer (headers stripped
// in place); ICMP echo replies reuse the request buffer. Outgoing data is written
// once into a buffer with `headroom` bytes in front, every layer prepends its
// header in place, and the driver DMAs from that buffer.
//
// Buffer budget: the packet pool is shared by the drivers' receive rings and every
// sender. Senders (allocate(): UDP, TCP data and control, ARP) never take the last
// Cfg::rx_reserve buffers, so received frames - among them the ACKs that release
// transmit buffers - always find a fresh DMA buffer. Without the reserve, transmit
// segments in flight on all connections could empty the pool, every received ACK
// would be dropped, and the connections would stall until they time out.
//
// Not implemented: IP fragmentation/reassembly (fragments are dropped; TCP MSS and
// the 512-byte WebSocket messages never need it), IP options, IP forwarding between
// interfaces, IPv6.
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

using InterfaceId = std::uint8_t;
inline constexpr InterfaceId any_interface = 0xFF;

struct DefaultStackConfig {
    static constexpr std::size_t interfaces = 2;
    static constexpr std::size_t arp_entries = 8;       ///< per interface
    static constexpr std::size_t multicast_groups = 2;  ///< per interface
    static constexpr std::size_t udp_sockets = 4;
    static constexpr std::size_t event_handlers = 8;
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
    InterfaceId interface;
    std::span<const std::byte> payload;
};

struct UdpHandler {
    void* context = nullptr;
    void (*received)(void* context, const UdpDatagram& datagram) noexcept = nullptr;
};

/// Interface events (Stack::subscribe). Every member may be null.
struct NetEventHandler {
    void* context = nullptr;
    void (*link_changed)(void* context, InterfaceId interface, bool up) noexcept = nullptr;
    /// New configuration in effect (set_config, a finished claim, an address given up).
    void (*address_changed)(void* context, InterfaceId interface, const Ipv4Config& config) noexcept = nullptr;
    /// `address` is used by another station: a claim failed or a defence was lost.
    void (*address_conflict)(void* context, InterfaceId interface, Ipv4Address address) noexcept = nullptr;
};

struct StackStats {
    std::uint32_t rx_frames = 0;
    std::uint32_t tx_frames = 0;
    std::uint32_t rx_dropped = 0;
    std::uint32_t tx_dropped = 0;
    std::uint32_t arp_requests = 0;
    std::uint32_t tx_no_buffer = 0;  ///< sends deferred: pool down to the receive reserve
    std::uint32_t address_conflicts = 0;
};

template <PacketAllocator Pool, class Cfg = DefaultStackConfig>
class Stack {
public:
    /// Bytes reserved in front of every outgoing payload: Ethernet + IPv4 + TCP
    /// (+4 for the MSS option), rounded up so the payload starts 4-byte aligned.
    static constexpr std::size_t headroom = 64;
    static constexpr std::size_t rx_reserve = Cfg::rx_reserve;
    static constexpr std::size_t max_interfaces = Cfg::interfaces;
    static constexpr std::uint32_t arp_timeout_ms = 300'000;
    static constexpr std::uint32_t arp_retry_ms = 1000;
    // RFC 5227 timing (ms)
    static constexpr std::uint32_t probe_wait_ms = 1000, probe_min_ms = 1000, probe_max_ms = 2000;
    static constexpr std::uint32_t announce_wait_ms = 2000, announce_interval_ms = 2000, defend_interval_ms = 10'000;
    static constexpr std::uint8_t probe_num = 3, announce_num = 2;

    using TcpLayer = Tcp<Stack, Cfg>;
    using TcpConnection = typename TcpLayer::Connection;

    explicit Stack(Pool& pool) noexcept : pool_(pool), tcp_(*this) {}

    Stack(const Stack&) = delete;
    Stack& operator=(const Stack&) = delete;

    /// Add an interface. `driver` must outlive the stack. Returns its id (0, 1, ...).
    template <NetDriver Driver>
    InterfaceId attach(Driver& driver, const Ipv4Config& config = {}) noexcept {
        contract_assert(interface_count_ < max_interfaces);
        Interface& i = interfaces_[interface_count_];
        i.driver = {&driver, [](void* d) noexcept { return static_cast<Driver*>(d)->link_up(); },
                    [](void* d, PacketRef f) noexcept { return static_cast<Driver*>(d)->transmit(std::move(f)); },
                    [](void* d) noexcept { return static_cast<Driver*>(d)->receive(); }};
        i.mac = driver.mac();
        i.config = config;
        return static_cast<InterfaceId>(interface_count_++);
    }

    /// Process all received frames, link changes and expired timers.
    void poll(std::uint32_t now_ms) noexcept {
        now_ms_ = now_ms;
        for (std::size_t n = 0; n < interface_count_; ++n) {
            const auto id = static_cast<InterfaceId>(n);
            Interface& i = interfaces_[n];
            if (const bool up = i.driver.link_up(i.driver.self); up != i.link) {
                i.link = up;
                for (const auto* h : handlers_) {
                    if (h != nullptr && h->link_changed != nullptr) {
                        h->link_changed(h->context, id, up);
                    }
                }
            }
            while (PacketRef frame = i.driver.receive(i.driver.self)) {
                ++stats_.rx_frames;
                input(id, std::move(frame));
            }
            service_arp(id);
            service_claim(id);
        }
        tcp_.service();
    }

    /// Earliest point in time poll() must run again without new frames (0 = none).
    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = tcp_.next_deadline();
        const auto take = [&best](std::uint32_t t) noexcept {
            if (best == 0 || static_cast<std::int32_t>(t - best) < 0) {
                best = t;
            }
        };
        for (std::size_t n = 0; n < interface_count_; ++n) {
            const Interface& i = interfaces_[n];
            for (const auto& e : i.arp) {
                if (e.pending) {
                    take(e.requested_ms + arp_retry_ms);
                }
            }
            if (i.claim.phase == ClaimPhase::Probing || i.claim.phase == ClaimPhase::Announcing) {
                take(i.claim.next_ms);
            }
        }
        return best;
    }

    // ---- interfaces -------------------------------------------------------------

    [[nodiscard]] std::size_t interface_count() const noexcept { return interface_count_; }
    [[nodiscard]] const Ipv4Config& config(InterfaceId i) const noexcept { return at(i).config; }
    [[nodiscard]] Ipv4Address local_address(InterfaceId i) const noexcept { return at(i).config.address; }
    [[nodiscard]] bool configured(InterfaceId i) const noexcept { return at(i).config.address != Ipv4Address::any(); }
    [[nodiscard]] MacAddress mac(InterfaceId i) const noexcept { return at(i).mac; }
    [[nodiscard]] bool link_up(InterfaceId i) const noexcept { return at(i).link; }
    /// Address claim in progress (RFC 5227 probing/announcing).
    [[nodiscard]] bool claiming(InterfaceId i) const noexcept { return at(i).claim.phase == ClaimPhase::Probing; }

    /// Use `config` at once (no probing). 0.0.0.0 = unconfigured. A changed address
    /// or subnet invalidates the interface's ARP cache.
    void set_config(InterfaceId id, const Ipv4Config& config) noexcept {
        Interface& i = at(id);
        i.claim = {};
        apply(id, config);
    }

    /// Take `config` after an RFC 5227 probe of its address: address_changed() when it
    /// is in use (after the probes), address_conflict() if another station has it.
    /// The interface stays unconfigured while probing.
    void claim_address(InterfaceId id, const Ipv4Config& config) noexcept {
        Interface& i = at(id);
        apply(id, {});
        i.claim = {};
        i.claim.phase = ClaimPhase::Probing;
        i.claim.config = config;
        i.claim.next_ms = now_ms_ + random32() % (probe_wait_ms + 1);
    }

    /// Interface a packet to `destination` leaves through (any_interface: no route).
    [[nodiscard]] InterfaceId route(Ipv4Address destination) const noexcept {
        for (std::size_t n = 0; n < interface_count_; ++n) {
            const Interface& i = interfaces_[n];
            if (i.config.address != Ipv4Address::any() && i.config.on_link(destination)) {
                return static_cast<InterfaceId>(n);
            }
        }
        for (std::size_t n = 0; n < interface_count_; ++n) {
            const Interface& i = interfaces_[n];
            if (i.config.address != Ipv4Address::any() && i.config.gateway != Ipv4Address::any()) {
                return static_cast<InterfaceId>(n);
            }
        }
        return any_interface;
    }
    [[nodiscard]] bool routable(Ipv4Address destination) const noexcept { return route(destination) != any_interface; }

    /// Receive IPv4 traffic for multicast `group` on interface `id`.
    bool join(InterfaceId id, Ipv4Address group) noexcept {
        contract_assert(group.is_multicast());
        auto& groups = at(id).groups;
        for (auto& g : groups) {
            if (g == group) {
                return true;
            }
        }
        for (auto& g : groups) {
            if (g == Ipv4Address::any()) {
                g = group;
                return true;
            }
        }
        return false;
    }

    /// Subscribe to interface events (the handler must stay valid).
    bool subscribe(const NetEventHandler& handler) noexcept {
        for (auto& h : handlers_) {
            if (h == nullptr) {
                h = &handler;
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] std::uint32_t now_ms() const noexcept { return now_ms_; }
    [[nodiscard]] const StackStats& stats() const noexcept { return stats_; }

    /// Source of initial sequence numbers and protocol randomness (the board supplies a
    /// hardware RNG).
    void set_random(std::uint32_t (*random)(void* context) noexcept, void* context) noexcept {
        random_ = random;
        random_context_ = context;
    }
    [[nodiscard]] std::uint32_t random32() noexcept {
        return random_ != nullptr ? random_(random_context_) : (lcg_ = lcg_ * 1664525u + 1013904223u);
    }

    // ---- buffers ------------------------------------------------------------------

    /// Buffer for an outgoing payload (headroom for all headers reserved). Empty if
    /// the pool is down to the receive reserve: callers retry later.
    [[nodiscard]] PacketRef allocate() noexcept { return allocate_tx(headroom); }

    // ---- UDP ----------------------------------------------------------------------

    /// Deliver datagrams for `port` (from `only` or, by default, any interface).
    /// Several sockets may share a port when they are bound to different interfaces.
    bool udp_bind(std::uint16_t port, const UdpHandler& handler, InterfaceId only = any_interface) noexcept {
        for (auto& s : udp_) {
            if (s.handler != nullptr && s.port == port &&
                (s.interface == only || s.interface == any_interface || only == any_interface)) {
                return false;  // in use
            }
        }
        for (auto& s : udp_) {
            if (s.handler == nullptr) {
                s = {port, only, &handler};
                return true;
            }
        }
        return false;
    }

    void udp_unbind(std::uint16_t port, InterfaceId only = any_interface) noexcept {
        for (auto& s : udp_) {
            if (s.handler != nullptr && s.port == port && s.interface == only) {
                s = UdpSocket{};
            }
        }
    }

    /// Send `payload` (at data(), headroom reserved) as one datagram, routed.
    bool udp_send(PacketRef payload, Ipv4Address destination, std::uint16_t destination_port,
                  std::uint16_t source_port) noexcept {
        const InterfaceId id = route(destination);
        if (id == any_interface) {
            ++stats_.tx_dropped;
            return false;
        }
        return udp_send_on(id, std::move(payload), destination, destination_port, source_port);
    }

    /// Send through interface `id` (broadcast, multicast, DHCP before an address).
    bool udp_send_on(InterfaceId id, PacketRef payload, Ipv4Address destination, std::uint16_t destination_port,
                     std::uint16_t source_port) noexcept {
        contract_assert(payload && payload->headroom() >= udp_header_size + ipv4_header_size + ethernet_header_size);
        PacketBuffer& b = *payload;
        std::byte* h = b.push_front(udp_header_size);
        store_be16(h, source_port);
        store_be16(h + 2, destination_port);
        store_be16(h + 4, static_cast<std::uint16_t>(b.size()));
        store_be16(h + 6, 0);
        const std::uint32_t pseudo = pseudo_sum(local_address(id), destination, ip_proto_udp, b.size());
        std::uint16_t sum = checksum_finish(checksum_add(b.bytes(), pseudo));
        store_be16(h + 6, sum == 0 ? 0xFFFF : sum);  // 0 means "no checksum"
        return send_ipv4_on(id, std::move(payload), destination, ip_proto_udp);
    }

    // ---- TCP ----------------------------------------------------------------------

    [[nodiscard]] TcpLayer& tcp() noexcept { return tcp_; }

    // ---- IPv4 output (also used by TCP) -------------------------------------------

    /// Prepend the IPv4 header to `packet` (transport header at data()) and send it:
    /// from `source` through the interface that owns it, or routed if `source` is any.
    bool send_ipv4(PacketRef packet, Ipv4Address destination, std::uint8_t protocol,
                   Ipv4Address source = Ipv4Address::any()) noexcept {
        InterfaceId id = any_interface;
        if (source != Ipv4Address::any()) {
            for (std::size_t n = 0; n < interface_count_; ++n) {
                if (interfaces_[n].config.address == source) {
                    id = static_cast<InterfaceId>(n);
                }
            }
        } else {
            id = route(destination);
        }
        if (id == any_interface) {
            ++stats_.tx_dropped;  // address gone or no route
            return false;
        }
        return send_ipv4_on(id, std::move(packet), destination, protocol);
    }

    /// Send through interface `id` with its address as source.
    bool send_ipv4_on(InterfaceId id, PacketRef packet, Ipv4Address destination, std::uint8_t protocol) noexcept {
        Interface& i = at(id);
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
        i.config.address.store(h + 12);
        destination.store(h + 16);
        store_be16(h + 10, checksum({h, ipv4_header_size}));

        if (destination.is_multicast()) {
            return send_frame(id, std::move(packet), destination.multicast_mac(), ethertype_ipv4);
        }
        if (destination.is_broadcast() || (i.config.address != Ipv4Address::any() &&
                                           destination == i.config.subnet_broadcast())) {
            return send_frame(id, std::move(packet), MacAddress::broadcast(), ethertype_ipv4);
        }
        const Ipv4Address next_hop = i.config.on_link(destination) ? destination : i.config.gateway;
        return resolve_and_send(id, std::move(packet), next_hop);
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

    enum class ClaimPhase : std::uint8_t { Idle, Probing, Announcing, Defending };

    struct Claim {
        ClaimPhase phase = ClaimPhase::Idle;
        Ipv4Config config{};
        std::uint32_t next_ms = 0;
        std::uint8_t sent = 0;
        std::uint32_t last_defence_ms = 0;
        bool defended = false;
    };

    struct ErasedDriver {
        void* self = nullptr;
        bool (*link_up)(void*) noexcept = nullptr;
        bool (*transmit)(void*, PacketRef) noexcept = nullptr;
        PacketRef (*receive)(void*) noexcept = nullptr;
    };

    struct Interface {
        ErasedDriver driver{};
        MacAddress mac{};
        Ipv4Config config{};
        bool link = false;
        std::array<Ipv4Address, Cfg::multicast_groups> groups{};
        std::array<ArpEntry, Cfg::arp_entries> arp{};
        Claim claim{};
    };

    struct UdpSocket {
        std::uint16_t port = 0;
        InterfaceId interface = any_interface;
        const UdpHandler* handler = nullptr;
    };

    Interface& at(InterfaceId id) noexcept {
        contract_assert(id < interface_count_);
        return interfaces_[id];
    }
    const Interface& at(InterfaceId id) const noexcept {
        contract_assert(id < interface_count_);
        return interfaces_[id];
    }

    void apply(InterfaceId id, const Ipv4Config& config) noexcept {
        Interface& i = at(id);
        if (config.address != i.config.address || config.netmask != i.config.netmask) {
            for (auto& e : i.arp) {
                e.parked.reset();
                e = ArpEntry{};
            }
        }
        const bool changed = config.address != i.config.address || config.netmask != i.config.netmask ||
                             config.gateway != i.config.gateway;
        i.config = config;
        if (changed) {
            for (const auto* h : handlers_) {
                if (h != nullptr && h->address_changed != nullptr) {
                    h->address_changed(h->context, id, config);
                }
            }
        }
    }

    void conflict(InterfaceId id, Ipv4Address address) noexcept {
        ++stats_.address_conflicts;
        for (const auto* h : handlers_) {
            if (h != nullptr && h->address_conflict != nullptr) {
                h->address_conflict(h->context, id, address);
            }
        }
    }

    PacketRef allocate_tx(std::size_t front) noexcept {
        if (pool_.available() <= rx_reserve) {
            ++stats_.tx_no_buffer;
            return {};
        }
        return pool_.allocate(front);
    }

    // ---- input --------------------------------------------------------------------

    void input(InterfaceId id, PacketRef frame) noexcept {
        PacketBuffer& b = *frame;
        if (b.size() < ethernet_header_size) {
            ++stats_.rx_dropped;
            return;
        }
        const std::uint16_t type = load_be16(b.data() + 12);
        b.pull_front(ethernet_header_size);
        if (type == ethertype_arp) {
            arp_input(id, std::move(frame));
        } else if (type == ethertype_ipv4) {
            ipv4_input(id, std::move(frame));
        } else {
            ++stats_.rx_dropped;
        }
    }

    void arp_input(InterfaceId id, PacketRef packet) noexcept {
        Interface& i = at(id);
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
        if (sender_mac == i.mac) {
            return;  // our own frame looped back
        }
        if (watch_claim(id, sender_ip, target_ip)) {
            return;
        }
        const bool configured = i.config.address != Ipv4Address::any();
        const bool for_us = configured && target_ip == i.config.address;
        if (sender_ip != Ipv4Address::any() && (for_us || arp_find(i, sender_ip) != nullptr)) {
            arp_update(id, sender_ip, sender_mac);  // RFC 826: merge
        }
        if (op == 1 && for_us) {
            // Reply in place: turn the request around.
            store_be16(a + 6, 2);
            sender_mac.store(a + 18);
            sender_ip.store(a + 24);
            i.mac.store(a + 8);
            i.config.address.store(a + 14);
            b.truncate(28);
            send_frame(id, std::move(packet), sender_mac, ethertype_arp);
        }
    }

    void ipv4_input(InterfaceId id, PacketRef packet) noexcept {
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
        if (!accepts(id, dst)) {
            return;
        }
        const std::uint8_t protocol = std::to_integer<std::uint8_t>(h[9]);
        b.truncate(total);  // strip Ethernet padding
        b.pull_front(header_len);
        switch (protocol) {
            case ip_proto_icmp: icmp_input(id, std::move(packet), src, dst); break;
            case ip_proto_udp: udp_input(id, std::move(packet), src, dst); break;
            case ip_proto_tcp:
                if (dst == local_address(id)) {
                    tcp_.input(std::move(packet), src, dst);
                }
                break;
            default: ++stats_.rx_dropped; break;
        }
    }

    [[nodiscard]] bool accepts(InterfaceId id, Ipv4Address dst) const noexcept {
        const Interface& i = at(id);
        const bool configured = i.config.address != Ipv4Address::any();
        if ((configured && (dst == i.config.address || dst == i.config.subnet_broadcast())) || dst.is_broadcast()) {
            return true;
        }
        if (dst.is_multicast()) {
            for (const auto& g : i.groups) {
                if (g == dst) {
                    return true;
                }
            }
        }
        return false;
    }

    void icmp_input(InterfaceId id, PacketRef packet, Ipv4Address src, Ipv4Address dst) noexcept {
        PacketBuffer& b = *packet;
        if (b.size() < 8 || checksum(b.bytes()) != 0 || !configured(id) || dst != local_address(id)) {
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
        send_ipv4_on(id, std::move(packet), src, ip_proto_icmp);
    }

    void udp_input(InterfaceId id, PacketRef packet, Ipv4Address src, Ipv4Address dst) noexcept {
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
            if (s.handler != nullptr && s.port == dst_port && (s.interface == any_interface || s.interface == id)) {
                const UdpDatagram d{src, load_be16(h), dst, dst_port, id,
                                    std::span<const std::byte>(h + udp_header_size, length - udp_header_size)};
                s.handler->received(s.handler->context, d);
                return;
            }
        }
    }

    // ---- output -------------------------------------------------------------------

    bool send_frame(InterfaceId id, PacketRef packet, const MacAddress& destination, std::uint16_t type) noexcept {
        Interface& i = at(id);
        PacketBuffer& b = *packet;
        std::byte* h = b.push_front(ethernet_header_size);
        destination.store(h);
        i.mac.store(h + 6);
        store_be16(h + 12, type);
        if (!i.driver.transmit(i.driver.self, packet)) {
            ++stats_.tx_dropped;
            return false;
        }
        ++stats_.tx_frames;
        return true;
    }

    bool resolve_and_send(InterfaceId id, PacketRef packet, Ipv4Address next_hop) noexcept {
        Interface& i = at(id);
        if (ArpEntry* e = arp_find(i, next_hop); e != nullptr && e->valid) {
            return send_frame(id, std::move(packet), e->mac, ethertype_ipv4);
        }
        ArpEntry& e = arp_slot(i, next_hop);
        e.parked = std::move(packet);  // a newer packet replaces an older one
        if (!e.pending) {
            e.pending = true;
            e.attempts = 0;
            arp_request(id, e);
        }
        return true;
    }

    // ---- ARP cache ------------------------------------------------------------------

    static ArpEntry* arp_find(Interface& i, Ipv4Address ip) noexcept {
        for (auto& e : i.arp) {
            if ((e.valid || e.pending) && e.ip == ip) {
                return &e;
            }
        }
        return nullptr;
    }

    /// Existing entry for `ip`, else a free one, else the least recently updated.
    static ArpEntry& arp_slot(Interface& i, Ipv4Address ip) noexcept {
        if (ArpEntry* e = arp_find(i, ip)) {
            return *e;
        }
        ArpEntry* victim = &i.arp[0];
        for (auto& e : i.arp) {
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

    void arp_update(InterfaceId id, Ipv4Address ip, const MacAddress& mac) noexcept {
        ArpEntry& e = arp_slot(at(id), ip);
        e.mac = mac;
        e.valid = true;
        e.pending = false;
        e.updated_ms = now_ms_;
        if (e.parked) {
            send_frame(id, std::move(e.parked), mac, ethertype_ipv4);
        }
    }

    /// ARP request (op 1) for `target` from `sender` (0.0.0.0: RFC 5227 probe).
    void send_arp(InterfaceId id, Ipv4Address sender, Ipv4Address target) noexcept {
        Interface& i = at(id);
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
        i.mac.store(a + 8);
        sender.store(a + 14);
        MacAddress{}.store(a + 18);
        target.store(a + 24);
        send_frame(id, std::move(p), MacAddress::broadcast(), ethertype_arp);
    }

    void arp_request(InterfaceId id, ArpEntry& e) noexcept {
        e.requested_ms = now_ms_;
        ++e.attempts;
        send_arp(id, local_address(id), e.ip);
    }

    void service_arp(InterfaceId id) noexcept {
        for (auto& e : at(id).arp) {
            if (e.pending && static_cast<std::int32_t>(now_ms_ - (e.requested_ms + arp_retry_ms)) >= 0) {
                if (e.attempts >= 3) {
                    e.parked.reset();
                    e = ArpEntry{};
                } else {
                    arp_request(id, e);
                }
            } else if (e.valid && now_ms_ - e.updated_ms > arp_timeout_ms) {
                e = ArpEntry{};
            }
        }
    }

    // ---- address claim (RFC 5227) ----------------------------------------------------

    void service_claim(InterfaceId id) noexcept {
        Claim& c = at(id).claim;
        if ((c.phase != ClaimPhase::Probing && c.phase != ClaimPhase::Announcing) ||
            static_cast<std::int32_t>(now_ms_ - c.next_ms) < 0) {
            return;
        }
        if (c.phase == ClaimPhase::Probing) {
            if (c.sent < probe_num) {
                send_arp(id, Ipv4Address::any(), c.config.address);  // probe
                ++c.sent;
                c.next_ms = now_ms_ + (c.sent < probe_num ? probe_min_ms + random32() % (probe_max_ms - probe_min_ms + 1)
                                                          : announce_wait_ms);
                return;
            }
            // No answer: the address is ours.
            c.phase = ClaimPhase::Announcing;
            c.sent = 0;
            apply(id, c.config);
        }
        send_arp(id, c.config.address, c.config.address);  // announcement (gratuitous ARP)
        if (++c.sent < announce_num) {
            c.next_ms = now_ms_ + announce_interval_ms;
        } else {
            c.phase = ClaimPhase::Defending;
        }
    }

    /// RFC 5227 2.1.1 / 2.4: is this ARP a conflict with our claimed or used address?
    /// Returns true if it was handled (and must not be merged into the cache).
    bool watch_claim(InterfaceId id, Ipv4Address sender_ip, Ipv4Address target_ip) noexcept {
        Interface& i = at(id);
        Claim& c = i.claim;
        if (c.phase == ClaimPhase::Probing) {
            // Someone uses the address, or probes for it at the same time.
            const Ipv4Address candidate = c.config.address;
            if (sender_ip == candidate || (sender_ip == Ipv4Address::any() && target_ip == candidate)) {
                c = {};
                conflict(id, candidate);
                return true;
            }
            return false;
        }
        if (i.config.address == Ipv4Address::any() || sender_ip != i.config.address) {
            return false;
        }
        // Another station claims our address: defend once per DEFEND_INTERVAL.
        if (c.defended && now_ms_ - c.last_defence_ms < defend_interval_ms) {
            const Ipv4Address lost = i.config.address;
            c = {};
            apply(id, {});
            conflict(id, lost);
            return true;
        }
        c.defended = true;
        c.last_defence_ms = now_ms_;
        send_arp(id, i.config.address, i.config.address);
        return true;
    }

    Pool& pool_;
    std::array<Interface, Cfg::interfaces> interfaces_{};
    std::size_t interface_count_ = 0;
    TcpLayer tcp_;
    std::array<UdpSocket, Cfg::udp_sockets> udp_{};
    std::array<const NetEventHandler*, Cfg::event_handlers> handlers_{};
    StackStats stats_{};
    std::uint32_t now_ms_ = 0;
    std::uint16_t ip_id_ = 0;
    std::uint32_t (*random_)(void*) noexcept = nullptr;
    void* random_context_ = nullptr;
    std::uint32_t lcg_ = 0x12345678u;
};

}  // namespace rtos::net
