// SPDX-License-Identifier: MIT
//
// DHCP client (RFC 2131, options RFC 2132) for one interface of the stack.
//
//   Selecting   DHCPDISCOVER (broadcast, source 0.0.0.0, BROADCAST flag: the stack
//               cannot receive unicast without an address)
//   Requesting  DHCPREQUEST for the first offer (requested address + server id)
//   Checking    the ACKed address is probed with ARP before use (RFC 5227 via
//               Stack::claim_address); in use by another station: DHCPDECLINE and a
//               new DISCOVER after 10 s (RFC 2131 3.1.5)
//   Bound       handler.bound() gets router, DNS and NTP servers; renewal at T1
//               (option 58, default 0.5 x lease)
//   Renewing    DHCPREQUEST unicast to the leasing server (ciaddr = our address)
//   Rebinding   from T2 (option 59, default 0.875 x lease) broadcast to any server
//   Rebooting   link back after a loss (stack event): INIT-REBOOT DHCPREQUEST for the
//               leased address (RFC 2131 3.7); NAK -> Selecting, no answer -> keep
//               the lease until it expires
//   expiry/NAK  address removed (handler.lost()), back to Selecting
//   conflict    the stack lost the address to another station (RFC 5227 defence):
//               DHCPDECLINE, back to Selecting
//
// Retransmission: 4, 8, 16, 32, 64 s (+-1 s random) while acquiring; in Renewing and
// Rebinding half the remaining time, at least 60 s (RFC 2131 4.4.5). Every exchange
// uses a fresh random xid; replies must match xid and our hardware address.
// Leases longer than 24 days are renewed after 24 days (32-bit millisecond clock).
//
// Not implemented: option overload, relay agents (giaddr), FQDN option.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/net/stack.hpp"

namespace rtos::net {

struct DhcpLease {
    Ipv4Config config{};                ///< address, netmask, router
    Ipv4Address server{};               ///< DHCP server that granted the lease
    std::array<Ipv4Address, 2> dns{};   ///< option 6 (unused entries: 0.0.0.0)
    std::array<Ipv4Address, 2> ntp{};   ///< option 42
    std::uint32_t lease_s = 0;
};

namespace dhcp {
inline constexpr std::uint16_t client_port = 68;
inline constexpr std::uint16_t server_port = 67;
inline constexpr std::uint8_t discover = 1, offer = 2, request = 3, decline = 4, ack = 5, nak = 6, release = 7,
                              inform = 8;
inline constexpr std::size_t bootp_size = 236;
inline constexpr std::size_t options_offset = bootp_size + 4;  // after the magic cookie
inline constexpr std::size_t min_message = 300;                // RFC 1542: BOOTP relays
inline constexpr std::uint32_t magic_cookie = 0x6382'5363;

/// Options of a received message (the subset client and server use).
struct Options {
    std::uint8_t type = 0;
    Ipv4Address server{};     ///< 54
    Ipv4Address requested{};  ///< 50
    Ipv4Address netmask{};    ///< 1
    Ipv4Address router{};     ///< 3
    std::array<Ipv4Address, 2> dns{};  ///< 6
    std::array<Ipv4Address, 2> ntp{};  ///< 42
    std::uint32_t lease_s = 0, t1_s = 0, t2_s = 0;  ///< 51, 58, 59
};

/// Parse the options behind the magic cookie. False if malformed or unterminated.
inline bool parse_options(std::span<const std::byte> o, Options& out) noexcept {
    auto address_list = [](std::span<const std::byte> v, std::array<Ipv4Address, 2>& list) noexcept {
        for (std::size_t i = 0; i < list.size() && 4 * i + 4 <= v.size(); ++i) {
            list[i] = Ipv4Address::load(v.data() + 4 * i);
        }
    };
    std::size_t pos = 0;
    while (pos < o.size()) {
        const auto code = std::to_integer<std::uint8_t>(o[pos]);
        if (code == 0) {
            ++pos;
            continue;
        }
        if (code == 255) {
            return true;
        }
        if (pos + 2 > o.size()) {
            return false;
        }
        const auto len = std::to_integer<std::size_t>(o[pos + 1]);
        if (pos + 2 + len > o.size()) {
            return false;
        }
        const auto v = o.subspan(pos + 2, len);
        const bool four = len >= 4;
        const Ipv4Address address = four ? Ipv4Address::load(v.data()) : Ipv4Address{};
        switch (code) {
            case 53: out.type = len == 1 ? std::to_integer<std::uint8_t>(v[0]) : 0; break;
            case 54: out.server = address; break;
            case 50: out.requested = address; break;
            case 1: out.netmask = address; break;
            case 3: out.router = address; break;
            case 6: address_list(v, out.dns); break;
            case 42: address_list(v, out.ntp); break;
            case 51: out.lease_s = four ? load_be32(v.data()) : 0; break;
            case 58: out.t1_s = four ? load_be32(v.data()) : 0; break;
            case 59: out.t2_s = four ? load_be32(v.data()) : 0; break;
            default: break;
        }
        pos += 2 + len;
    }
    return false;
}

/// Writes options into a packet buffer.
class OptionWriter {
public:
    explicit OptionWriter(PacketBuffer& p) noexcept : p_(p) {}
    void add(std::uint8_t code, std::span<const std::byte> value) noexcept {
        std::byte* o = p_.append(2 + value.size());
        o[0] = std::byte{code};
        o[1] = static_cast<std::byte>(value.size());
        std::copy(value.begin(), value.end(), o + 2);
    }
    void add(std::uint8_t code, Ipv4Address a) noexcept {
        std::array<std::byte, 4> v{};
        a.store(v.data());
        add(code, v);
    }
    void add32(std::uint8_t code, std::uint32_t value) noexcept {
        std::array<std::byte, 4> v{};
        store_be32(v.data(), value);
        add(code, v);
    }
    /// End option and padding to the BOOTP minimum size.
    void finish() noexcept {
        *p_.append(1) = std::byte{255};
        if (p_.size() < min_message) {
            const std::size_t pad = min_message - p_.size();
            std::fill_n(p_.append(pad), pad, std::byte{0});
        }
    }

private:
    PacketBuffer& p_;
};
}  // namespace dhcp

template <class Stack>
class DhcpClient {
public:
    enum class State : std::uint8_t { Stopped, Selecting, Requesting, Checking, Bound, Renewing, Rebinding, Rebooting };

    struct Handler {
        void* context = nullptr;
        void (*bound)(void* context, const DhcpLease& lease, bool renewed) noexcept = nullptr;
        void (*lost)(void* context) noexcept = nullptr;  ///< lease expired, NAKed or address lost
        /// The offered address is used by another station (declined).
        void (*declined)(void* context, Ipv4Address address) noexcept = nullptr;
    };

    static constexpr std::uint32_t decline_wait_ms = 10'000;
    static constexpr std::uint8_t reboot_attempts = 3;

    DhcpClient(Stack& stack, InterfaceId interface, std::string_view host_name, const Handler& handler) noexcept
        : stack_(stack), interface_(interface), host_name_(host_name.substr(0, 63)), handler_(handler) {}

    DhcpClient(const DhcpClient&) = delete;
    DhcpClient& operator=(const DhcpClient&) = delete;

    bool start() noexcept {
        if (!stack_.udp_bind(dhcp::client_port, udp_handler_, InterfaceSet::of({interface_})) ||
            !stack_.subscribe(events_)) {
            return false;
        }
        restart(0);
        return true;
    }

    /// Stop the protocol (e.g. to fall back to a static address); the interface's
    /// current configuration is left as it is.
    void stop() noexcept {
        stack_.udp_unbind(dhcp::client_port, udp_handler_);
        state_ = State::Stopped;
    }

    void service() noexcept {
        if (state_ == State::Stopped || state_ == State::Checking || !reached(next_ms_) ||
            !stack_.link_up(interface_)) {
            return;  // nothing to do yet; without a link messages would go nowhere
        }
        switch (state_) {
            case State::Selecting: send(dhcp::discover); break;
            case State::Requesting:
                if (attempts_ >= 4) {
                    restart(0);  // the server stopped answering: start over
                    return;
                }
                send(dhcp::request);
                break;
            case State::Bound:
                state_ = State::Renewing;
                new_transaction();
                send(dhcp::request);
                break;
            case State::Renewing:
                if (reached(t2_ms_)) {
                    state_ = State::Rebinding;
                    new_transaction();
                }
                send(dhcp::request);
                break;
            case State::Rebinding:
                if (reached(expiry_ms_)) {
                    lose();
                    return;
                }
                send(dhcp::request);
                break;
            case State::Rebooting:
                if (attempts_ >= reboot_attempts) {
                    resume_lease();  // no server answered: keep the lease (RFC 2131 3.7)
                    return;
                }
                send(dhcp::request);
                break;
            case State::Checking:
            case State::Stopped: return;
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        // Without a link the link-up event wakes the client.
        return state_ == State::Stopped || state_ == State::Checking || !stack_.link_up(interface_) ? 0 : next_ms_;
    }
    [[nodiscard]] State state() const noexcept { return state_; }
    [[nodiscard]] bool bound() const noexcept {
        return state_ == State::Bound || state_ == State::Renewing || state_ == State::Rebinding ||
               state_ == State::Rebooting;
    }
    [[nodiscard]] const DhcpLease& lease() const noexcept { return lease_; }
    [[nodiscard]] InterfaceId interface() const noexcept { return interface_; }

private:
    static constexpr std::uint32_t max_lease_ms = 24u * 24 * 3600 * 1000;

    [[nodiscard]] bool reached(std::uint32_t t) const noexcept {
        return static_cast<std::int32_t>(stack_.now_ms() - t) >= 0;
    }

    void restart(std::uint32_t delay_ms) noexcept {
        state_ = State::Selecting;
        new_transaction();
        acquisition_start_ms_ = stack_.now_ms();
        next_ms_ = stack_.now_ms() + delay_ms;
    }

    void new_transaction() noexcept {
        xid_ = stack_.random32();
        attempts_ = 0;
    }

    void lose() noexcept {
        stack_.set_config(interface_, {});
        lease_ = {};
        if (handler_.lost != nullptr) {
            handler_.lost(handler_.context);
        }
        restart(0);
    }

    /// Back to Bound with the timers of the current lease (Rebooting without answer).
    void resume_lease() noexcept {
        if (reached(expiry_ms_)) {
            lose();
            return;
        }
        state_ = reached(t2_ms_) ? State::Rebinding : reached(t1_ms_) ? State::Renewing : State::Bound;
        new_transaction();
        next_ms_ = state_ == State::Bound ? t1_ms_ : stack_.now_ms();
    }

    /// Next retransmission after `attempts_` sends.
    void schedule_retry() noexcept {
        const std::uint32_t now = stack_.now_ms();
        if (state_ == State::Selecting || state_ == State::Requesting || state_ == State::Rebooting) {
            const std::uint32_t base = std::min<std::uint32_t>(4000u << std::min<std::uint8_t>(attempts_ - 1, 4), 64000);
            next_ms_ = now + base - 1000 + stack_.random32() % 2001;  // +-1 s
            return;
        }
        const std::uint32_t until = state_ == State::Renewing ? t2_ms_ : expiry_ms_;
        const std::uint32_t remaining = static_cast<std::int32_t>(until - now) > 0 ? until - now : 0;
        next_ms_ = now + std::min(remaining, std::max<std::uint32_t>(remaining / 2, 60'000));
    }

    /// BOOTREQUEST header + client options; the caller adds message specifics.
    PacketRef message(std::uint8_t type, bool broadcast_flag, Ipv4Address ciaddr) noexcept {
        PacketRef p = stack_.allocate();
        if (!p || p->tailroom() < dhcp::min_message + 64) {
            return {};
        }
        std::byte* m = p->append(dhcp::options_offset);
        std::fill(m, m + dhcp::options_offset, std::byte{0});
        m[0] = std::byte{1};  // BOOTREQUEST
        m[1] = std::byte{1};  // Ethernet
        m[2] = std::byte{6};
        store_be32(m + 4, xid_);
        const std::uint32_t secs = (stack_.now_ms() - acquisition_start_ms_) / 1000;
        store_be16(m + 8, static_cast<std::uint16_t>(std::min<std::uint32_t>(secs, 0xFFFF)));
        if (broadcast_flag) {
            store_be16(m + 10, 0x8000);  // BROADCAST: we cannot receive unicast yet
        }
        ciaddr.store(m + 12);
        stack_.mac(interface_).store(m + 28);
        store_be32(m + dhcp::bootp_size, dhcp::magic_cookie);
        dhcp::OptionWriter w(*p);
        const std::array<std::byte, 1> message_type{std::byte{type}};
        w.add(53, message_type);
        std::array<std::byte, 7> client_id{std::byte{1}};  // hardware type Ethernet + MAC
        stack_.mac(interface_).store(client_id.data() + 1);
        w.add(61, client_id);
        return p;
    }

    void send(std::uint8_t type) noexcept {
        ++attempts_;
        schedule_retry();
        const bool renewing = state_ == State::Renewing || state_ == State::Rebinding;
        PacketRef p = message(type, !renewing, renewing ? lease_.config.address : Ipv4Address::any());
        if (!p) {
            return;
        }
        dhcp::OptionWriter w(*p);
        w.add(12, std::as_bytes(std::span(host_name_.data(), host_name_.size())));
        if (state_ == State::Requesting) {
            w.add(50, offered_);       // requested address
            w.add(54, offer_server_);  // server identifier
        } else if (state_ == State::Rebooting) {
            w.add(50, lease_.config.address);  // INIT-REBOOT: requested address, no server id
        }
        constexpr std::array<std::byte, 7> parameters{std::byte{1}, std::byte{3}, std::byte{6}, std::byte{42},
                                                      std::byte{51}, std::byte{58}, std::byte{59}};
        w.add(55, parameters);
        w.finish();
        if (type == dhcp::request && !((state_ == State::Requesting || state_ == State::Rebooting) && attempts_ > 1)) {
            request_sent_ms_ = stack_.now_ms();  // the lease counts from the (first) request
        }
        // Renewing: unicast to the leasing server; otherwise broadcast on our interface.
        if (state_ == State::Renewing) {
            stack_.udp_send_on(interface_, std::move(p), lease_.server, dhcp::server_port, dhcp::client_port);
        } else {
            stack_.udp_send_on(interface_, std::move(p), Ipv4Address::broadcast(), dhcp::server_port,
                               dhcp::client_port);
        }
    }

    void send_decline(Ipv4Address address, Ipv4Address server) noexcept {
        new_transaction();
        PacketRef p = message(dhcp::decline, false, Ipv4Address::any());
        if (!p) {
            return;
        }
        dhcp::OptionWriter w(*p);
        w.add(50, address);
        w.add(54, server);
        w.finish();
        stack_.udp_send_on(interface_, std::move(p), Ipv4Address::broadcast(), dhcp::server_port, dhcp::client_port);
    }

    static void received(void* self, const UdpDatagram& d) noexcept { static_cast<DhcpClient*>(self)->reply(d); }

    void reply(const UdpDatagram& d) noexcept {
        const auto m = d.payload;
        if (state_ == State::Stopped || state_ == State::Bound || state_ == State::Checking ||
            d.source_port != dhcp::server_port || m.size() < dhcp::options_offset || m[0] != std::byte{2} ||
            m[1] != std::byte{1} || m[2] != std::byte{6} || load_be32(m.data() + 4) != xid_ ||
            MacAddress::load(m.data() + 28) != stack_.mac(interface_) ||
            load_be32(m.data() + dhcp::bootp_size) != dhcp::magic_cookie) {
            return;
        }
        dhcp::Options o;
        if (!dhcp::parse_options(m.subspan(dhcp::options_offset), o) || o.server == Ipv4Address::any()) {
            return;
        }
        const Ipv4Address yiaddr = Ipv4Address::load(m.data() + 16);
        const bool renewing = state_ == State::Renewing || state_ == State::Rebinding;
        if (o.type == dhcp::offer && state_ == State::Selecting) {
            if (yiaddr == Ipv4Address::any() || yiaddr.is_broadcast() || yiaddr.is_multicast()) {
                return;
            }
            offered_ = yiaddr;
            offer_server_ = o.server;
            state_ = State::Requesting;
            attempts_ = 0;
            send(dhcp::request);
            return;
        }
        if (state_ != State::Requesting && state_ != State::Rebooting && !renewing) {
            return;
        }
        const Ipv4Address expected_server = renewing ? lease_.server : offer_server_;
        if (o.type == dhcp::nak &&
            (state_ == State::Rebinding || state_ == State::Rebooting || o.server == expected_server)) {
            lose();
            return;
        }
        if (o.type != dhcp::ack || yiaddr == Ipv4Address::any() ||
            (state_ == State::Renewing && o.server != expected_server) ||
            (state_ == State::Requesting && yiaddr != offered_) ||
            (state_ == State::Rebooting && yiaddr != lease_.config.address)) {
            return;
        }
        accept(yiaddr, o, renewing || state_ == State::Rebooting);
    }

    void accept(Ipv4Address address, const dhcp::Options& o, bool renewed) noexcept {
        lease_.config.address = address;
        lease_.config.netmask = o.netmask != Ipv4Address::any() ? o.netmask : Ipv4Address::from(255, 255, 255, 0);
        lease_.config.gateway = o.router;
        lease_.server = o.server;
        lease_.dns = o.dns;
        lease_.ntp = o.ntp;
        lease_.lease_s = o.lease_s;
        // Times count from the transmission of the request (RFC 2131 4.4.1).
        const auto ms = [](std::uint32_t s) noexcept {
            return static_cast<std::uint32_t>(std::min<std::uint64_t>(s * 1000ull, max_lease_ms));
        };
        const std::uint32_t lease_ms = o.lease_s == 0 ? 3'600'000 : ms(o.lease_s);
        std::uint32_t t1 = o.t1_s != 0 ? ms(o.t1_s) : lease_ms / 2;
        std::uint32_t t2 = o.t2_s != 0 ? ms(o.t2_s) : lease_ms / 8 * 7;
        if (!(t1 < t2 && t2 <= lease_ms)) {
            t1 = lease_ms / 2;
            t2 = lease_ms / 8 * 7;
        }
        expiry_ms_ = request_sent_ms_ + lease_ms;
        t2_ms_ = request_sent_ms_ + t2;
        t1_ms_ = request_sent_ms_ + t1;
        if (stack_.local_address(interface_) == address) {
            bound(renewed);  // the address is ours already (renewal, reboot)
            return;
        }
        state_ = State::Checking;  // probe first (RFC 5227); the stack reports back
        stack_.claim_address(interface_, lease_.config);
    }

    void bound(bool renewed) noexcept {
        state_ = State::Bound;
        next_ms_ = t1_ms_;
        stack_.set_config(interface_, lease_.config);  // router etc. may have changed
        if (handler_.bound != nullptr) {
            handler_.bound(handler_.context, lease_, renewed);
        }
    }

    // ---- stack events ---------------------------------------------------------------

    static void on_link(void* self, InterfaceId id, bool up) noexcept {
        auto& c = *static_cast<DhcpClient*>(self);
        if (id != c.interface_ || !up || c.state_ == State::Stopped) {
            return;
        }
        if (c.state_ == State::Bound || c.state_ == State::Renewing || c.state_ == State::Rebinding) {
            c.state_ = State::Rebooting;  // possibly another network now: verify the lease
            c.new_transaction();
            c.next_ms_ = c.stack_.now_ms();
        } else if (c.state_ == State::Selecting || c.state_ == State::Requesting) {
            c.next_ms_ = c.stack_.now_ms();  // send at once, same transaction (offers in flight stay valid)
        }
    }

    static void on_address(void* self, InterfaceId id, const Ipv4Config& config) noexcept {
        auto& c = *static_cast<DhcpClient*>(self);
        if (id == c.interface_ && c.state_ == State::Checking && config.address == c.lease_.config.address) {
            c.bound(false);
        }
    }

    static void on_conflict(void* self, InterfaceId id, Ipv4Address address) noexcept {
        auto& c = *static_cast<DhcpClient*>(self);
        if (id != c.interface_ || address != c.lease_.config.address || c.state_ == State::Stopped) {
            return;
        }
        const bool was_bound = c.state_ != State::Checking;
        c.send_decline(address, c.lease_.server);
        if (c.handler_.declined != nullptr) {
            c.handler_.declined(c.handler_.context, address);
        }
        if (was_bound && c.handler_.lost != nullptr) {
            c.handler_.lost(c.handler_.context);  // the stack gave the address up
        }
        c.lease_ = {};
        c.restart(decline_wait_ms);
    }

    Stack& stack_;
    InterfaceId interface_;
    std::string_view host_name_;
    Handler handler_;
    UdpHandler udp_handler_{this, &DhcpClient::received};
    NetEventHandler events_{this, &DhcpClient::on_link, &DhcpClient::on_address, &DhcpClient::on_conflict};
    State state_ = State::Stopped;
    std::uint32_t xid_ = 0;
    std::uint8_t attempts_ = 0;
    std::uint32_t next_ms_ = 0;
    std::uint32_t acquisition_start_ms_ = 0;
    std::uint32_t request_sent_ms_ = 0;
    std::uint32_t t1_ms_ = 0;
    std::uint32_t t2_ms_ = 0;
    std::uint32_t expiry_ms_ = 0;
    Ipv4Address offered_{};
    Ipv4Address offer_server_{};
    DhcpLease lease_{};
};

}  // namespace rtos::net
