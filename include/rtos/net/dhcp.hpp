// SPDX-License-Identifier: MIT
//
// DHCP client (RFC 2131, options RFC 2132) for the stack's single interface.
//
//   Selecting   DHCPDISCOVER (broadcast, source 0.0.0.0, BROADCAST flag: the stack
//               cannot receive unicast without an address)
//   Requesting  DHCPREQUEST for the first offer (requested address + server id)
//   Bound       Stack::set_config(); handler.bound() gets router, DNS and NTP
//               servers; renewal at T1 (option 58, default 0.5 x lease)
//   Renewing    DHCPREQUEST unicast to the leasing server (ciaddr = our address)
//   Rebinding   from T2 (option 59, default 0.875 x lease) broadcast to any server
//   expiry/NAK  address removed (handler.lost()), back to Selecting
//
// Retransmission: 4, 8, 16, 32, 64 s (+-1 s random) while acquiring; in Renewing and
// Rebinding half the remaining time, at least 60 s (RFC 2131 4.4.5). Every exchange
// uses a fresh random xid; replies must match xid and our hardware address.
// Leases longer than 24 days are renewed after 24 days (32-bit millisecond clock).
//
// Not implemented: address conflict detection (ARP probe of the offered address,
// DHCPDECLINE), INIT-REBOOT with a remembered address, option overload, relay agents.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
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

template <class Stack>
class DhcpClient {
public:
    enum class State : std::uint8_t { Stopped, Selecting, Requesting, Bound, Renewing, Rebinding };

    struct Handler {
        void* context = nullptr;
        void (*bound)(void* context, const DhcpLease& lease, bool renewed) noexcept = nullptr;
        void (*lost)(void* context) noexcept = nullptr;  ///< lease expired or NAKed
    };

    static constexpr std::uint16_t client_port = 68;
    static constexpr std::uint16_t server_port = 67;

    DhcpClient(Stack& stack, std::string_view host_name, const Handler& handler) noexcept
        : stack_(stack), host_name_(host_name.substr(0, 63)), handler_(handler) {}

    DhcpClient(const DhcpClient&) = delete;
    DhcpClient& operator=(const DhcpClient&) = delete;

    bool start() noexcept {
        if (!stack_.udp_bind(client_port, udp_handler_)) {
            return false;
        }
        restart();
        return true;
    }

    /// Stop the protocol (e.g. to fall back to a static address); the stack's current
    /// configuration is left as it is.
    void stop() noexcept {
        stack_.udp_unbind(client_port);
        state_ = State::Stopped;
    }

    void service() noexcept {
        if (state_ == State::Stopped || static_cast<std::int32_t>(stack_.now_ms() - next_ms_) < 0) {
            return;
        }
        switch (state_) {
            case State::Selecting: send(message_discover); break;
            case State::Requesting:
                if (attempts_ >= 4) {
                    restart();  // the server stopped answering: start over
                    return;
                }
                send(message_request);
                break;
            case State::Bound:
                state_ = State::Renewing;
                new_transaction();
                send(message_request);
                break;
            case State::Renewing:
                if (reached(t2_ms_)) {
                    state_ = State::Rebinding;
                    new_transaction();
                }
                send(message_request);
                break;
            case State::Rebinding:
                if (reached(expiry_ms_)) {
                    lose();
                    return;
                }
                send(message_request);
                break;
            case State::Stopped: return;
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return state_ == State::Stopped ? 0 : next_ms_; }
    [[nodiscard]] State state() const noexcept { return state_; }
    [[nodiscard]] bool bound() const noexcept {
        return state_ == State::Bound || state_ == State::Renewing || state_ == State::Rebinding;
    }
    [[nodiscard]] const DhcpLease& lease() const noexcept { return lease_; }

private:
    static constexpr std::uint8_t message_discover = 1, message_offer = 2, message_request = 3, message_ack = 5,
                                  message_nak = 6;
    static constexpr std::size_t bootp_size = 236;
    static constexpr std::size_t options_offset = bootp_size + 4;  // after the magic cookie
    static constexpr std::size_t min_message = 300;                // RFC 1542: BOOTP relays
    static constexpr std::uint32_t magic_cookie = 0x6382'5363;
    static constexpr std::uint32_t max_lease_ms = 24u * 24 * 3600 * 1000;

    [[nodiscard]] bool reached(std::uint32_t t) const noexcept {
        return static_cast<std::int32_t>(stack_.now_ms() - t) >= 0;
    }

    void restart() noexcept {
        state_ = State::Selecting;
        new_transaction();
        acquisition_start_ms_ = stack_.now_ms();
        next_ms_ = stack_.now_ms();
    }

    void new_transaction() noexcept {
        xid_ = stack_.random32();
        attempts_ = 0;
    }

    void lose() noexcept {
        stack_.set_config({});
        lease_ = {};
        if (handler_.lost != nullptr) {
            handler_.lost(handler_.context);
        }
        restart();
    }

    /// Next retransmission after `attempts_` sends.
    void schedule_retry() noexcept {
        const std::uint32_t now = stack_.now_ms();
        if (state_ == State::Selecting || state_ == State::Requesting) {
            const std::uint32_t base = std::min<std::uint32_t>(4000u << std::min<std::uint8_t>(attempts_ - 1, 4), 64000);
            next_ms_ = now + base - 1000 + stack_.random32() % 2001;  // +-1 s
            return;
        }
        const std::uint32_t until = state_ == State::Renewing ? t2_ms_ : expiry_ms_;
        const std::uint32_t remaining = static_cast<std::int32_t>(until - now) > 0 ? until - now : 0;
        next_ms_ = now + std::min(remaining, std::max<std::uint32_t>(remaining / 2, 60'000));
    }

    void send(std::uint8_t type) noexcept {
        ++attempts_;
        schedule_retry();
        PacketRef p = stack_.allocate();
        if (!p || p->tailroom() < min_message) {
            return;
        }
        std::byte* m = p->append(options_offset);
        std::fill(m, m + options_offset, std::byte{0});
        m[0] = std::byte{1};  // BOOTREQUEST
        m[1] = std::byte{1};  // Ethernet
        m[2] = std::byte{6};
        store_be32(m + 4, xid_);
        const std::uint32_t secs = (stack_.now_ms() - acquisition_start_ms_) / 1000;
        store_be16(m + 8, static_cast<std::uint16_t>(std::min<std::uint32_t>(secs, 0xFFFF)));
        const bool renewing = state_ == State::Renewing || state_ == State::Rebinding;
        if (!renewing) {
            store_be16(m + 10, 0x8000);  // BROADCAST: we cannot receive unicast yet
        } else {
            lease_.config.address.store(m + 12);  // ciaddr
        }
        stack_.mac().store(m + 28);
        store_be32(m + bootp_size, magic_cookie);

        auto option = [&p](std::uint8_t code, std::span<const std::byte> value) noexcept {
            std::byte* o = p->append(2 + value.size());
            o[0] = std::byte{code};
            o[1] = static_cast<std::byte>(value.size());
            std::copy(value.begin(), value.end(), o + 2);
        };
        const std::array<std::byte, 1> message_type{std::byte{type}};
        option(53, message_type);
        std::array<std::byte, 7> client_id{std::byte{1}};  // hardware type Ethernet + MAC
        stack_.mac().store(client_id.data() + 1);
        option(61, client_id);
        option(12, std::as_bytes(std::span(host_name_.data(), host_name_.size())));
        if (state_ == State::Requesting) {
            std::array<std::byte, 4> v{};
            offered_.store(v.data());
            option(50, v);  // requested address
            offer_server_.store(v.data());
            option(54, v);  // server identifier
        }
        constexpr std::array<std::byte, 7> parameters{std::byte{1}, std::byte{3}, std::byte{6}, std::byte{42},
                                                      std::byte{51}, std::byte{58}, std::byte{59}};
        option(55, parameters);
        *p->append(1) = std::byte{255};
        if (p->size() < min_message) {
            const std::size_t pad = min_message - p->size();
            std::fill_n(p->append(pad), pad, std::byte{0});
        }
        // Renewing: unicast to the leasing server; otherwise broadcast.
        const Ipv4Address to = state_ == State::Renewing ? lease_.server : Ipv4Address::broadcast();
        if (type == message_request && !(state_ == State::Requesting && attempts_ > 1)) {
            request_sent_ms_ = stack_.now_ms();  // the lease counts from the (first) request
        }
        stack_.udp_send(std::move(p), to, server_port, client_port);
    }

    static void received(void* self, const UdpDatagram& d) noexcept { static_cast<DhcpClient*>(self)->reply(d); }

    struct Options {
        std::uint8_t type = 0;
        Ipv4Address server{};
        Ipv4Address netmask{};
        Ipv4Address router{};
        std::array<Ipv4Address, 2> dns{};
        std::array<Ipv4Address, 2> ntp{};
        std::uint32_t lease_s = 0, t1_s = 0, t2_s = 0;
    };

    static bool parse(std::span<const std::byte> o, Options& out) noexcept {
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
            switch (code) {
                case 53: out.type = len == 1 ? std::to_integer<std::uint8_t>(v[0]) : 0; break;
                case 54: out.server = four ? Ipv4Address::load(v.data()) : Ipv4Address{}; break;
                case 1: out.netmask = four ? Ipv4Address::load(v.data()) : Ipv4Address{}; break;
                case 3: out.router = four ? Ipv4Address::load(v.data()) : Ipv4Address{}; break;
                case 6: address_list(v, out.dns); break;
                case 42: address_list(v, out.ntp); break;
                case 51: out.lease_s = four ? load_be32(v.data()) : 0; break;
                case 58: out.t1_s = four ? load_be32(v.data()) : 0; break;
                case 59: out.t2_s = four ? load_be32(v.data()) : 0; break;
                default: break;
            }
            pos += 2 + len;
        }
        return false;  // no end option
    }

    void reply(const UdpDatagram& d) noexcept {
        const auto m = d.payload;
        if (state_ == State::Stopped || state_ == State::Bound || d.source_port != server_port ||
            m.size() < options_offset || m[0] != std::byte{2} || m[1] != std::byte{1} || m[2] != std::byte{6} ||
            load_be32(m.data() + 4) != xid_ || MacAddress::load(m.data() + 28) != stack_.mac() ||
            load_be32(m.data() + bootp_size) != magic_cookie) {
            return;
        }
        Options o;
        if (!parse(m.subspan(options_offset), o) || o.server == Ipv4Address::any()) {
            return;
        }
        const Ipv4Address yiaddr = Ipv4Address::load(m.data() + 16);
        const bool renewing = state_ == State::Renewing || state_ == State::Rebinding;
        if (o.type == message_offer && state_ == State::Selecting) {
            if (yiaddr == Ipv4Address::any() || yiaddr.is_broadcast() || yiaddr.is_multicast()) {
                return;
            }
            offered_ = yiaddr;
            offer_server_ = o.server;
            state_ = State::Requesting;
            attempts_ = 0;
            send(message_request);
            return;
        }
        if (state_ != State::Requesting && !renewing) {
            return;
        }
        const Ipv4Address expected_server = renewing ? lease_.server : offer_server_;
        if (o.type == message_nak && (state_ == State::Rebinding || o.server == expected_server)) {
            lose();
            return;
        }
        if (o.type != message_ack || yiaddr == Ipv4Address::any() || (state_ == State::Renewing && o.server != expected_server) ||
            (state_ == State::Requesting && yiaddr != offered_)) {
            return;
        }
        bind(yiaddr, o, renewing);
    }

    void bind(Ipv4Address address, const Options& o, bool renewed) noexcept {
        lease_.config.address = address;
        lease_.config.netmask = o.netmask != Ipv4Address::any() ? o.netmask : Ipv4Address::from(255, 255, 255, 0);
        lease_.config.gateway = o.router;
        lease_.server = o.server;
        lease_.dns = o.dns;
        lease_.ntp = o.ntp;
        lease_.lease_s = o.lease_s;
        // Times count from the transmission of the request (RFC 2131 4.4.1).
        const std::uint32_t lease_ms =
            o.lease_s == 0 ? 3'600'000 : static_cast<std::uint32_t>(std::min<std::uint64_t>(o.lease_s * 1000ull, max_lease_ms));
        std::uint32_t t1 = o.t1_s != 0 ? static_cast<std::uint32_t>(std::min<std::uint64_t>(o.t1_s * 1000ull, max_lease_ms)) : lease_ms / 2;
        std::uint32_t t2 = o.t2_s != 0 ? static_cast<std::uint32_t>(std::min<std::uint64_t>(o.t2_s * 1000ull, max_lease_ms))
                                       : lease_ms / 8 * 7;
        if (!(t1 < t2 && t2 <= lease_ms)) {
            t1 = lease_ms / 2;
            t2 = lease_ms / 8 * 7;
        }
        expiry_ms_ = request_sent_ms_ + lease_ms;
        t2_ms_ = request_sent_ms_ + t2;
        next_ms_ = request_sent_ms_ + t1;
        state_ = State::Bound;
        stack_.set_config(lease_.config);
        if (handler_.bound != nullptr) {
            handler_.bound(handler_.context, lease_, renewed);
        }
    }

    Stack& stack_;
    std::string_view host_name_;
    Handler handler_;
    UdpHandler udp_handler_{this, &DhcpClient::received};
    State state_ = State::Stopped;
    std::uint32_t xid_ = 0;
    std::uint8_t attempts_ = 0;
    std::uint32_t next_ms_ = 0;
    std::uint32_t acquisition_start_ms_ = 0;
    std::uint32_t request_sent_ms_ = 0;
    std::uint32_t t2_ms_ = 0;
    std::uint32_t expiry_ms_ = 0;
    Ipv4Address offered_{};
    Ipv4Address offer_server_{};
    DhcpLease lease_{};
};

}  // namespace rtos::net
