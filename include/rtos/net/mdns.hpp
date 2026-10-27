// SPDX-License-Identifier: MIT
//
// mDNS responder (RFC 6762) for one host name on every interface of the stack:
// answers A queries for "<hostname>.local" with the address of the interface the
// query arrived on.
//
//   * probing (8.1): before an interface uses the name - and again whenever its
//     address changes (stack event) - three probes 250 ms apart (QU query for the
//     name, our A record in the authority section)
//   * simultaneous probes (8.2): another host probing the same name with a
//     lexicographically later A record wins; we probe again after one second
//   * conflict (9): a response claiming the name for another address renames the
//     host to "<hostname>-2", "-3", ... and probes again on all interfaces
//   * announcing (8.3): two unsolicited responses one second apart
//   * multicast queries                -> multicast response (ID 0, cache-flush bit)
//   * QU questions (unicast-response)  -> unicast response to the querier, port 5353
//   * legacy unicast (src port != 5353)-> unicast response with the query ID and the
//                                         question repeated, no cache-flush bit (6.7)
// Not implemented: known-answer suppression, AAAA, services (DNS-SD).
#pragma once

#include <algorithm>
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
    static constexpr std::uint32_t probe_interval_ms = 250;
    static constexpr std::uint32_t announce_interval_ms = 1000;
    static constexpr std::uint32_t defer_ms = 1000;  ///< lost a simultaneous probe
    static constexpr std::uint8_t probes = 3, announcements = 2;

    /// Notified when a conflict renamed the host (e.g. to log the new name).
    using RenameCallback = void (*)(void* context, std::string_view name) noexcept;

    /// `hostname` without ".local"; the object stores a copy.
    MdnsResponder(Stack& stack, std::string_view hostname, std::uint32_t ttl_s = 120) noexcept
        : stack_(stack), ttl_s_(ttl_s) {
        base_len_ = std::min(hostname.size(), max_name - 12);  // room for "-NN.local"
        std::copy_n(hostname.begin(), base_len_, base_.begin());
        make_name();
    }

    MdnsResponder(const MdnsResponder&) = delete;
    MdnsResponder& operator=(const MdnsResponder&) = delete;

    /// Answer on the interfaces in `on` (default: all): join the group there and start
    /// probing where an address exists.
    bool start(InterfaceSet on = InterfaceSet::all(), RenameCallback renamed = nullptr, void* context = nullptr) noexcept {
        on_ = on;
        renamed_ = renamed;
        rename_context_ = context;
        for (std::size_t n = 0; n < stack_.interface_count(); ++n) {
            const auto id = static_cast<InterfaceId>(n);
            if (on_.contains(id)) {
                if (!stack_.join(id, mdns_group)) {
                    return false;
                }
                restart(id, 0);
            }
        }
        return stack_.udp_bind(mdns_port, udp_handler_, on_) && stack_.subscribe(events_);
    }

    void service() noexcept {
        const std::uint32_t now = stack_.now_ms();
        for (std::size_t n = 0; n < stack_.interface_count(); ++n) {
            const auto id = static_cast<InterfaceId>(n);
            State& s = states_[n];
            if ((s.phase != Phase::Probing && s.phase != Phase::Announcing) ||
                static_cast<std::int32_t>(now - s.next_ms) < 0) {
                continue;
            }
            if (s.phase == Phase::Probing) {
                if (s.sent < probes) {
                    send_probe(id, s.sent == 0);
                    ++s.sent;
                    s.next_ms = now + probe_interval_ms;
                    continue;
                }
                s.phase = Phase::Announcing;  // nobody objected: the name is ours here
                s.sent = 0;
            }
            send_answer(id, mdns_group, mdns_port, false, 0, {});
            s.next_ms = now + announce_interval_ms;
            if (++s.sent == announcements) {
                s.phase = Phase::Ready;
            }
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = 0;
        for (std::size_t n = 0; n < stack_.interface_count(); ++n) {
            const State& s = states_[n];
            if ((s.phase == Phase::Probing || s.phase == Phase::Announcing) &&
                (best == 0 || static_cast<std::int32_t>(s.next_ms - best) < 0)) {
                best = s.next_ms;
            }
        }
        return best;
    }

    [[nodiscard]] std::string_view name() const noexcept { return {name_.data(), name_len_}; }
    /// The name is probed and in use on interface `id`.
    [[nodiscard]] bool ready(InterfaceId id) const noexcept { return states_[id].phase == Phase::Ready; }
    [[nodiscard]] std::uint32_t answered() const noexcept { return answered_; }
    [[nodiscard]] std::uint32_t conflicts() const noexcept { return conflicts_; }

private:
    enum class Phase : std::uint8_t { Idle, Probing, Announcing, Ready };
    struct State {
        Phase phase = Phase::Idle;
        std::uint8_t sent = 0;
        std::uint32_t next_ms = 0;
    };

    void make_name() noexcept {
        std::size_t n = base_len_;
        std::copy_n(base_.begin(), base_len_, name_.begin());
        if (suffix_ > 1) {  // "-2", "-3", ...
            name_[n++] = '-';
            std::array<char, 10> digits{};
            std::size_t d = 0;
            for (std::uint32_t v = suffix_; v != 0; v /= 10) {
                digits[d++] = static_cast<char>('0' + v % 10);
            }
            while (d != 0) {
                name_[n++] = digits[--d];
            }
        }
        for (char c : std::string_view(".local")) {
            name_[n++] = c;
        }
        name_len_ = n;
    }

    /// (Re)start probing on `id` after `delay_ms` (Idle without an address).
    void restart(InterfaceId id, std::uint32_t delay_ms) noexcept {
        State& s = states_[id];
        s.sent = 0;
        if (!on_.contains(id) || !stack_.configured(id)) {
            s.phase = Phase::Idle;
            return;
        }
        s.phase = Phase::Probing;
        s.next_ms = stack_.now_ms() + delay_ms + stack_.random32() % probe_interval_ms;
    }

    void rename() noexcept {
        ++conflicts_;
        ++suffix_;
        make_name();
        if (renamed_ != nullptr) {
            renamed_(rename_context_, name());
        }
        for (std::size_t n = 0; n < stack_.interface_count(); ++n) {
            restart(static_cast<InterfaceId>(n), 0);
        }
    }

    static void on_address(void* self, InterfaceId id, const Ipv4Config&) noexcept {
        static_cast<MdnsResponder*>(self)->restart(id, 0);
    }

    static void received(void* self, const UdpDatagram& d) noexcept { static_cast<MdnsResponder*>(self)->input(d); }

    void input(const UdpDatagram& d) noexcept {
        const auto msg = d.payload;
        if (msg.size() < dns::header_size || d.interface >= stack_.interface_count() ||
            d.source == stack_.local_address(d.interface)) {
            return;  // malformed, or our own packet looped back
        }
        const std::uint16_t flags = load_be16(msg.data() + 2);
        if (((flags >> 11) & 0xF) != 0) {
            return;  // non-standard opcode
        }
        if ((flags & dns::flag_qr) != 0) {
            response(d);
        } else {
            query(d);
        }
    }

    /// Iterate over the resource records of a message from section `first`
    /// (0 answers, 1 authority, 2 additional) on; f(owner offset, type, rdata span)
    /// returns false to stop. Returns false on a malformed message.
    template <class F>
    static bool records(std::span<const std::byte> msg, int first, F&& f) noexcept {
        std::size_t pos = dns::header_size;
        for (std::uint16_t q = 0; q < load_be16(msg.data() + 4); ++q) {
            const auto after = dns::skip_name(msg, pos);
            if (!after || *after + 4 > msg.size()) {
                return false;
            }
            pos = *after + 4;
        }
        for (int section = 0; section < 3; ++section) {
            const std::uint16_t count = load_be16(msg.data() + 6 + 2 * section);
            for (std::uint16_t r = 0; r < count; ++r) {
                const auto after = dns::skip_name(msg, pos);
                if (!after || *after + 10 > msg.size()) {
                    return false;
                }
                const std::size_t length = load_be16(msg.data() + *after + 8);
                if (*after + 10 + length > msg.size()) {
                    return false;
                }
                if (section >= first &&
                    !f(pos, load_be16(msg.data() + *after), msg.subspan(*after + 10, length))) {
                    return true;
                }
                pos = *after + 10 + length;
            }
        }
        return true;
    }

    /// Another host answers for our name: a conflict unless it has our address.
    void response(const UdpDatagram& d) noexcept {
        const State& s = states_[d.interface];
        if (s.phase == Phase::Idle) {
            return;
        }
        std::array<std::byte, 4> ours{};
        stack_.local_address(d.interface).store(ours.data());
        bool conflict = false;
        records(d.payload, 0, [&](std::size_t owner, std::uint16_t type, std::span<const std::byte> rdata) noexcept {
            if (!dns::match_name(d.payload, owner, name())) {
                return true;
            }
            // While probing any record for the name counts (8.1); afterwards only a
            // different address (9).
            conflict = s.phase == Phase::Probing || type != dns::type_a || rdata.size() != 4 ||
                       !std::equal(rdata.begin(), rdata.end(), ours.begin());
            return !conflict;
        });
        if (conflict) {
            rename();
        }
    }

    void query(const UdpDatagram& d) noexcept {
        const auto msg = d.payload;
        State& s = states_[d.interface];
        if (s.phase == Phase::Probing && load_be16(msg.data() + 8) != 0) {
            // Simultaneous probe (8.2): compare the A records in the authority sections.
            std::array<std::byte, 4> ours{};
            stack_.local_address(d.interface).store(ours.data());
            bool lost = false;
            records(msg, 1, [&](std::size_t owner, std::uint16_t type, std::span<const std::byte> rdata) noexcept {
                if (type == dns::type_a && rdata.size() == 4 && dns::match_name(msg, owner, name())) {
                    lost = std::lexicographical_compare(ours.begin(), ours.end(), rdata.begin(), rdata.end());
                    return false;
                }
                return true;
            });
            if (lost) {
                restart(d.interface, defer_ms);
            }
            return;
        }
        if (s.phase != Phase::Announcing && s.phase != Phase::Ready) {
            return;
        }
        const std::uint16_t id = load_be16(msg.data());
        const std::uint16_t questions = load_be16(msg.data() + 4);
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
                    send_answer(d.interface, d.source, d.source_port, true, id, msg.subspan(pos, *after + 4 - pos));
                } else if ((klass & 0x8000) != 0) {  // QU
                    send_answer(d.interface, d.source, mdns_port, false, 0, {});
                } else {
                    send_answer(d.interface, mdns_group, mdns_port, false, 0, {});
                }
                return;
            }
            pos = *after + 4;
        }
    }

    /// Our A record: name, type A, class IN (+ cache flush), TTL, address of `id`.
    bool append_record(InterfaceId id, PacketBuffer& p, bool cache_flush, bool compressed_name) noexcept {
        std::size_t name_bytes = 2;
        if (compressed_name) {
            if (p.tailroom() < 2 + 14) {
                return false;
            }
            std::byte* n = p.append(2);
            n[0] = std::byte{0xC0};  // pointer to the question name at offset 12
            n[1] = std::byte{dns::header_size};
        } else {
            name_bytes = dns::write_name(p.data() + p.size(), p.tailroom() - 14, name());
            if (name_bytes == 0) {
                return false;
            }
            p.append(name_bytes);
        }
        std::byte* rr = p.append(14);
        store_be16(rr, dns::type_a);
        store_be16(rr + 2, cache_flush ? static_cast<std::uint16_t>(dns::class_in | 0x8000) : dns::class_in);
        store_be32(rr + 4, ttl_s_);
        store_be16(rr + 8, 4);
        stack_.local_address(id).store(rr + 10);
        return true;
    }

    void send_probe(InterfaceId id, bool unicast_response) noexcept {
        PacketRef p = stack_.allocate();
        if (!p) {
            return;
        }
        std::byte* h = p->append(dns::header_size);
        std::fill(h, h + dns::header_size, std::byte{0});
        store_be16(h + 4, 1);  // one question
        store_be16(h + 8, 1);  // one authority record (the proposed one)
        const std::size_t n = dns::write_name(p->data() + p->size(), p->tailroom() - 4, name());
        if (n == 0) {
            return;
        }
        p->append(n);
        std::byte* q = p->append(4);
        store_be16(q, dns::type_any);
        store_be16(q + 2, unicast_response ? static_cast<std::uint16_t>(dns::class_in | 0x8000) : dns::class_in);
        if (append_record(id, *p, false, true)) {
            stack_.udp_send_on(id, std::move(p), mdns_group, mdns_port, mdns_port);
        }
    }

    void send_answer(InterfaceId id, Ipv4Address to, std::uint16_t port, bool legacy, std::uint16_t query_id,
                     std::span<const std::byte> question) noexcept {
        if (!stack_.configured(id)) {
            return;
        }
        PacketRef p = stack_.allocate();
        if (!p) {
            return;
        }
        std::byte* h = p->append(dns::header_size);
        store_be16(h, query_id);
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
        if (append_record(id, *p, !legacy, false)) {
            stack_.udp_send_on(id, std::move(p), to, port, mdns_port);
        }
    }

    Stack& stack_;
    std::array<char, max_name> base_{};
    std::size_t base_len_ = 0;
    std::array<char, max_name> name_{};
    std::size_t name_len_ = 0;
    std::uint32_t suffix_ = 1;
    std::uint32_t ttl_s_;
    std::array<State, Stack::max_interfaces> states_{};
    std::uint32_t answered_ = 0;
    std::uint32_t conflicts_ = 0;
    InterfaceSet on_ = InterfaceSet::none();
    RenameCallback renamed_ = nullptr;
    void* rename_context_ = nullptr;
    UdpHandler udp_handler_{this, &MdnsResponder::received};
    NetEventHandler events_{this, nullptr, &MdnsResponder::on_address, nullptr};
};

}  // namespace rtos::net
