// SPDX-License-Identifier: MIT
//
// mDNS responder (RFC 6762) for one host name: answers A queries for
// "<hostname>.local" with the interface address.
//
//   * multicast queries                -> multicast response (ID 0, cache-flush bit)
//   * QU questions (unicast-response)  -> unicast response to the querier, port 5353
//   * legacy unicast (src port != 5353)-> unicast response with the query ID and the
//                                         question repeated, no cache-flush bit (6.7)
//   * two announcements at start-up, one second apart (8.3)
// Not implemented: probing/conflict resolution, known-answer suppression, AAAA.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/net/dns.hpp"
#include "rtos/net/stack.hpp"

namespace rtos::net {

inline constexpr std::uint16_t mdns_port = 5353;
inline constexpr Ipv4Address mdns_group = Ipv4Address::from(224, 0, 0, 251);

template <class Stack>
class MdnsResponder {
public:
    static constexpr std::size_t max_name = 64;

    /// `hostname` without ".local"; the object stores a copy of the full name.
    MdnsResponder(Stack& stack, std::string_view hostname, std::uint32_t ttl_s = 120) noexcept
        : stack_(stack), ttl_s_(ttl_s) {
        std::size_t n = 0;
        for (char c : hostname) {
            if (n < max_name - 7) {
                name_[n++] = c;
            }
        }
        for (char c : std::string_view(".local")) {
            name_[n++] = c;
        }
        name_len_ = n;
    }

    MdnsResponder(const MdnsResponder&) = delete;
    MdnsResponder& operator=(const MdnsResponder&) = delete;

    bool start() noexcept {
        if (!stack_.join(mdns_group) || !stack_.udp_bind(mdns_port, handler_)) {
            return false;
        }
        announcements_left_ = 2;
        next_announcement_ms_ = stack_.now_ms();
        return true;
    }

    /// Announce the (new) address again, e.g. after a DHCP lease (RFC 6762 8.3).
    void announce() noexcept {
        announcements_left_ = 2;
        next_announcement_ms_ = stack_.now_ms();
    }

    void service() noexcept {
        if (announcements_left_ != 0 && stack_.configured() &&
            static_cast<std::int32_t>(stack_.now_ms() - next_announcement_ms_) >= 0) {
            send_answer(mdns_group, mdns_port, false, 0, {});
            --announcements_left_;
            next_announcement_ms_ = stack_.now_ms() + 1000;
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        return announcements_left_ != 0 && stack_.configured() ? next_announcement_ms_ : 0;
    }
    [[nodiscard]] std::string_view name() const noexcept { return {name_.data(), name_len_}; }
    [[nodiscard]] std::uint32_t answered() const noexcept { return answered_; }

private:
    static void received(void* self, const UdpDatagram& d) noexcept { static_cast<MdnsResponder*>(self)->query(d); }

    void query(const UdpDatagram& d) noexcept {
        const auto msg = d.payload;
        if (msg.size() < dns::header_size) {
            return;
        }
        const std::uint16_t id = load_be16(msg.data());
        const std::uint16_t flags = load_be16(msg.data() + 2);
        const std::uint16_t questions = load_be16(msg.data() + 4);
        if ((flags & dns::flag_qr) != 0 || ((flags >> 11) & 0xF) != 0) {
            return;  // responses and non-standard opcodes are ignored
        }
        std::size_t pos = dns::header_size;
        for (std::uint16_t q = 0; q < questions; ++q) {
            const auto matched = dns::match_name(msg, pos, name());
            const auto after = matched ? matched : dns::skip_name(msg, pos);
            if (!after || *after + 4 > msg.size()) {
                return;
            }
            const std::uint16_t type = load_be16(msg.data() + *after);
            const std::uint16_t klass = load_be16(msg.data() + *after + 2);
            if (matched && (type == dns::type_a || type == dns::type_any) &&
                (klass & dns::class_mask) == dns::class_in) {
                ++answered_;
                const bool legacy = d.source_port != mdns_port;
                if (legacy) {
                    send_answer(d.source, d.source_port, true, id, msg.subspan(pos, *after + 4 - pos));
                } else if ((klass & 0x8000) != 0) {  // QU
                    send_answer(d.source, mdns_port, false, 0, {});
                } else {
                    send_answer(mdns_group, mdns_port, false, 0, {});
                }
                return;
            }
            pos = *after + 4;
        }
    }

    void send_answer(Ipv4Address to, std::uint16_t port, bool legacy, std::uint16_t id,
                     std::span<const std::byte> question) noexcept {
        if (!stack_.configured()) {
            return;  // no address to announce yet
        }
        PacketRef p = stack_.allocate();
        if (!p) {
            return;
        }
        std::byte* h = p->append(dns::header_size);
        store_be16(h, id);
        store_be16(h + 2, dns::flag_qr | dns::flag_aa);
        store_be16(h + 4, question.empty() ? 0 : 1);
        store_be16(h + 6, 1);
        store_be16(h + 8, 0);
        store_be16(h + 10, 0);
        if (!question.empty()) {
            if (question.size() > p->tailroom()) {
                return;
            }
            p->append(question);
        }
        std::byte* a = p->data() + p->size();
        const std::size_t name_bytes = dns::write_name(a, p->tailroom() - 10, name());
        if (name_bytes == 0) {
            return;
        }
        p->append(name_bytes);
        std::byte* rr = p->append(14);
        store_be16(rr, dns::type_a);
        store_be16(rr + 2, legacy ? dns::class_in : static_cast<std::uint16_t>(dns::class_in | 0x8000));  // cache flush
        store_be32(rr + 4, ttl_s_);
        store_be16(rr + 8, 4);
        stack_.local_address().store(rr + 10);
        stack_.udp_send(std::move(p), to, port, mdns_port);
    }

    Stack& stack_;
    std::array<char, max_name> name_{};
    std::size_t name_len_ = 0;
    std::uint32_t ttl_s_;
    std::uint32_t next_announcement_ms_ = 0;
    std::uint8_t announcements_left_ = 0;
    std::uint32_t answered_ = 0;
    UdpHandler handler_{this, &MdnsResponder::received};
};

}  // namespace rtos::net
