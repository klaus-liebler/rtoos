// SPDX-License-Identifier: MIT
//
// DHCP server (RFC 2131) for one interface - typically a point-to-point link such as
// USB CDC-NCM, where the device hands the host PC its address.
//
//   * pool: `count` consecutive addresses from `first` (in the interface's subnet);
//     a client keeps its address (binding by hardware address)
//   * DISCOVER -> OFFER (held for offer_hold_ms), REQUEST -> ACK/NAK for the
//     selecting, INIT-REBOOT, renewing and rebinding cases, DECLINE (address
//     quarantined for declined_hold_ms), RELEASE
//   * options: subnet mask, lease time with T1/T2, server id; router and DNS only if
//     configured - on a USB link the device must not become the PC's default route
//   * replies to clients without an address are broadcast on the interface (a
//     unicast to yiaddr would need an ARP answer the client cannot give yet)
// Runs in the network thread like every service; the interface must be configured
// before start() (its address is the server identifier).
// Not implemented: relay agents (giaddr), DHCPINFORM, persistence across resets.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/net/dhcp.hpp"
#include "rtos/net/stack.hpp"

namespace rtos::net {

struct DhcpServerConfig {
    Ipv4Address first{};           ///< first address of the pool
    std::uint8_t count = 1;        ///< pool size (<= MaxLeases)
    std::uint32_t lease_s = 3600;
    Ipv4Address router{};          ///< option 3 if set (default: none)
    Ipv4Address dns{};             ///< option 6 if set
};

template <class Stack, std::size_t MaxLeases = 4>
class DhcpServer {
public:
    static constexpr std::uint32_t offer_hold_ms = 10'000;
    static constexpr std::uint32_t declined_hold_ms = 600'000;

    DhcpServer(Stack& stack, InterfaceId interface, const DhcpServerConfig& config) noexcept
        : stack_(stack), interface_(interface), config_(config) {
        contract_assert(config.count >= 1 && config.count <= MaxLeases);
    }

    DhcpServer(const DhcpServer&) = delete;
    DhcpServer& operator=(const DhcpServer&) = delete;

    bool start() noexcept {
        return stack_.configured(interface_) &&
               stack_.udp_bind(dhcp::server_port, udp_handler_, InterfaceSet::of({interface_}));
    }

    // NetService: expiry is checked lazily when an address is needed.
    void service() noexcept {}
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return 0; }

    /// Address bound to `mac`, or 0.0.0.0.
    [[nodiscard]] Ipv4Address address_of(const MacAddress& mac) const noexcept {
        for (std::size_t i = 0; i < config_.count; ++i) {
            if (leases_[i].state == LeaseState::Bound && leases_[i].mac == mac && !expired(leases_[i])) {
                return address(i);
            }
        }
        return {};
    }
    [[nodiscard]] std::size_t bound_count() const noexcept {
        std::size_t n = 0;
        for (std::size_t i = 0; i < config_.count; ++i) {
            n += leases_[i].state == LeaseState::Bound && !expired(leases_[i]);
        }
        return n;
    }

private:
    enum class LeaseState : std::uint8_t { Free, Offered, Bound, Declined };
    struct Lease {
        LeaseState state = LeaseState::Free;
        MacAddress mac{};
        std::uint32_t until_ms = 0;  ///< end of offer hold, lease or quarantine
    };

    [[nodiscard]] Ipv4Address address(std::size_t i) const noexcept {
        return Ipv4Address{config_.first.value + static_cast<std::uint32_t>(i)};
    }
    /// Pool index of `a`, or -1.
    [[nodiscard]] int index_of(Ipv4Address a) const noexcept {
        const std::uint32_t d = a.value - config_.first.value;
        return d < config_.count ? static_cast<int>(d) : -1;
    }
    [[nodiscard]] bool expired(const Lease& l) const noexcept {
        return l.state != LeaseState::Free && static_cast<std::int32_t>(stack_.now_ms() - l.until_ms) >= 0;
    }
    /// Lease of `mac` (any non-free state), else -1.
    [[nodiscard]] int find(const MacAddress& mac) const noexcept {
        for (std::size_t i = 0; i < config_.count; ++i) {
            const Lease& l = leases_[i];
            if ((l.state == LeaseState::Offered || l.state == LeaseState::Bound) && l.mac == mac && !expired(l)) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }
    /// A free (or expired) address, preferring the one `mac` had before.
    [[nodiscard]] int allocate(const MacAddress& mac) const noexcept {
        int any = -1;
        for (std::size_t i = 0; i < config_.count; ++i) {
            const Lease& l = leases_[i];
            const bool usable = l.state == LeaseState::Free || expired(l);
            if (usable && l.mac == mac) {
                return static_cast<int>(i);
            }
            if (usable && any < 0) {
                any = static_cast<int>(i);
            }
        }
        return any;
    }

    static void received(void* self, const UdpDatagram& d) noexcept { static_cast<DhcpServer*>(self)->request(d); }

    void request(const UdpDatagram& d) noexcept {
        const auto m = d.payload;
        if (m.size() < dhcp::options_offset || m[0] != std::byte{1} || m[1] != std::byte{1} || m[2] != std::byte{6} ||
            load_be32(m.data() + dhcp::bootp_size) != dhcp::magic_cookie) {
            return;
        }
        dhcp::Options o;
        if (!dhcp::parse_options(m.subspan(dhcp::options_offset), o)) {
            return;
        }
        const MacAddress mac = MacAddress::load(m.data() + 28);
        const Ipv4Address ciaddr = Ipv4Address::load(m.data() + 12);
        const Ipv4Address self = stack_.local_address(interface_);
        const std::uint32_t now = stack_.now_ms();
        switch (o.type) {
            case dhcp::discover: {
                int i = find(mac);
                if (i < 0) {
                    i = allocate(mac);
                }
                if (i < 0) {
                    return;  // pool exhausted: stay silent
                }
                Lease& l = leases_[static_cast<std::size_t>(i)];
                if (l.state != LeaseState::Bound) {
                    l = {LeaseState::Offered, mac, now + offer_hold_ms};
                }
                reply(m, dhcp::offer, address(static_cast<std::size_t>(i)));
                return;
            }
            case dhcp::request: {
                // RFC 2131 4.3.2: SELECTING names a server; INIT-REBOOT has neither a
                // server id nor ciaddr; RENEWING/REBINDING have ciaddr.
                if (o.server != Ipv4Address::any() && o.server != self) {
                    // The client chose another server: drop our offer.
                    if (const int i = find(mac); i >= 0 && leases_[static_cast<std::size_t>(i)].state == LeaseState::Offered) {
                        leases_[static_cast<std::size_t>(i)] = {};
                    }
                    return;
                }
                const bool selecting = o.server == self;
                const Ipv4Address wanted = ciaddr != Ipv4Address::any() ? ciaddr : o.requested;
                const int i = index_of(wanted);
                const int mine = find(mac);
                const bool free = i >= 0 && (leases_[static_cast<std::size_t>(i)].state == LeaseState::Free ||
                                             expired(leases_[static_cast<std::size_t>(i)]));
                if (i >= 0 && (i == mine || (mine < 0 && free && selecting))) {
                    leases_[static_cast<std::size_t>(i)] = {LeaseState::Bound, mac, now + config_.lease_s * 1000};
                    reply(m, dhcp::ack, wanted);
                } else if (selecting || i >= 0 || !stack_.config(interface_).on_link(wanted)) {
                    // Our pool, but not this client's address; or the wrong network.
                    reply(m, dhcp::nak, {});
                }
                return;  // otherwise: not ours to answer
            }
            case dhcp::decline:
                if (const int i = index_of(o.requested); i >= 0 && leases_[static_cast<std::size_t>(i)].mac == mac) {
                    leases_[static_cast<std::size_t>(i)] = {LeaseState::Declined, {}, now + declined_hold_ms};
                }
                return;
            case dhcp::release:
                if (const int i = index_of(ciaddr); i >= 0 && leases_[static_cast<std::size_t>(i)].mac == mac) {
                    leases_[static_cast<std::size_t>(i)] = {LeaseState::Free, mac, 0};
                }
                return;
            default: return;
        }
    }

    void reply(std::span<const std::byte> req, std::uint8_t type, Ipv4Address yiaddr) noexcept {
        PacketRef p = stack_.allocate();
        if (!p || p->tailroom() < dhcp::min_message + 64) {
            return;
        }
        std::byte* m = p->append(dhcp::options_offset);
        std::fill(m, m + dhcp::options_offset, std::byte{0});
        m[0] = std::byte{2};  // BOOTREPLY
        m[1] = std::byte{1};
        m[2] = std::byte{6};
        std::copy_n(req.data() + 4, 4, m + 4);    // xid
        std::copy_n(req.data() + 10, 2, m + 10);  // flags
        if (type != dhcp::nak) {
            std::copy_n(req.data() + 12, 4, m + 12);  // ciaddr
            yiaddr.store(m + 16);
        }
        std::copy_n(req.data() + 28, 16, m + 28);  // chaddr
        store_be32(m + dhcp::bootp_size, dhcp::magic_cookie);
        dhcp::OptionWriter w(*p);
        const std::array<std::byte, 1> message_type{std::byte{type}};
        w.add(53, message_type);
        w.add(54, stack_.local_address(interface_));
        if (type != dhcp::nak) {
            w.add32(51, config_.lease_s);
            w.add32(58, config_.lease_s / 2);
            w.add32(59, config_.lease_s / 8 * 7);
            w.add(1, stack_.config(interface_).netmask);
            if (config_.router != Ipv4Address::any()) {
                w.add(3, config_.router);
            }
            if (config_.dns != Ipv4Address::any()) {
                w.add(6, config_.dns);
            }
        }
        w.finish();
        const Ipv4Address ciaddr = Ipv4Address::load(req.data() + 12);
        const Ipv4Address to = type != dhcp::nak && ciaddr != Ipv4Address::any() ? ciaddr : Ipv4Address::broadcast();
        stack_.udp_send_on(interface_, std::move(p), to, dhcp::client_port, dhcp::server_port);
    }

    Stack& stack_;
    InterfaceId interface_;
    DhcpServerConfig config_;
    std::array<Lease, MaxLeases> leases_{};
    UdpHandler udp_handler_{this, &DhcpServer::received};
};

}  // namespace rtos::net
