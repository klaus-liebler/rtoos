// SPDX-License-Identifier: MIT
//
// DNS stub resolver (RFC 1035) for IPv4 addresses (A records).
//
//   resolve(name, callback)  asks the configured servers (e.g. from DHCP option 6);
//                            the callback gets the address or nullopt (NXDOMAIN,
//                            no answer, timeout). Cache hits answer immediately.
//   lookup(name)             cache only
//
// Spoofing resistance (RFC 5452): every query uses a random 16-bit ID and its own
// random source port (49152..65535); a reply must come from the queried server's
// port 53, match ID and question, and is parsed with bounded name decompression.
// CNAME chains inside the answer are followed. Retries: 2 s per attempt, two
// attempts per server, alternating between up to two servers.
// Cache: positive entries for their TTL (clamped to 10 s .. 1 day), NXDOMAIN 30 s.
// Not implemented: TCP fallback for truncated replies (fails), AAAA, DNSSEC.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

#include "rtos/net/dns.hpp"
#include "rtos/net/stack.hpp"

namespace rtos::net {

template <class Stack, std::size_t CacheEntries = 4, std::size_t MaxQueries = 2>
class DnsResolver {
public:
    static constexpr std::size_t max_name = 127;
    static constexpr std::uint16_t server_port = 53;
    static constexpr std::uint32_t attempt_ms = 2000;
    static constexpr std::uint8_t attempts_per_server = 2;
    static constexpr std::uint32_t min_ttl_s = 10, max_ttl_s = 86'400, negative_ttl_s = 30;

    using Callback = void (*)(void* context, std::string_view name, std::optional<Ipv4Address> address) noexcept;

    explicit DnsResolver(Stack& stack) noexcept : stack_(stack) {
        for (auto& q : queries_) {
            q.owner = this;
            q.handler = {&q, &DnsResolver::received};
        }
    }

    DnsResolver(const DnsResolver&) = delete;
    DnsResolver& operator=(const DnsResolver&) = delete;

    /// Up to two servers; 0.0.0.0 entries are ignored.
    void set_servers(std::span<const Ipv4Address> servers) noexcept {
        server_count_ = 0;
        for (Ipv4Address s : servers) {
            if (s != Ipv4Address::any() && server_count_ < servers_.size()) {
                servers_[server_count_++] = s;
            }
        }
    }
    [[nodiscard]] std::size_t servers() const noexcept { return server_count_; }

    /// Cached address of `name` (nullopt: unknown, expired or cached NXDOMAIN).
    [[nodiscard]] std::optional<Ipv4Address> lookup(std::string_view name) const noexcept {
        const CacheEntry* e = find(name);
        if (e == nullptr || e->address == Ipv4Address::any()) {
            return std::nullopt;
        }
        return e->address;
    }

    /// Resolve `name`; `callback` runs from service()/poll() (or right here for a cache
    /// hit). False if the name is invalid, no server is known or all query slots are busy.
    bool resolve(std::string_view name, Callback callback, void* context) noexcept {
        if (name.empty() || name.size() > max_name) {
            return false;
        }
        if (const CacheEntry* e = find(name)) {
            callback(context, name, e->address == Ipv4Address::any() ? std::nullopt : std::optional(e->address));
            return true;
        }
        if (server_count_ == 0 || !stack_.configured()) {
            return false;
        }
        for (auto& q : queries_) {
            if (!q.active) {
                q.name_len = name.size();
                std::copy(name.begin(), name.end(), q.name.begin());
                q.callback = callback;
                q.context = context;
                q.attempt = 0;
                q.active = true;
                send(q);
                return true;
            }
        }
        return false;
    }

    void service() noexcept {
        for (auto& q : queries_) {
            if (q.active && static_cast<std::int32_t>(stack_.now_ms() - q.deadline_ms) >= 0) {
                if (q.attempt >= attempts_per_server * server_count_) {
                    finish(q, std::nullopt, 0);
                } else {
                    send(q);
                }
            }
        }
    }

    [[nodiscard]] std::uint32_t next_deadline() const noexcept {
        std::uint32_t best = 0;
        for (const auto& q : queries_) {
            if (q.active && (best == 0 || static_cast<std::int32_t>(q.deadline_ms - best) < 0)) {
                best = q.deadline_ms;
            }
        }
        return best;
    }

private:
    static constexpr std::uint16_t type_cname = 5;

    struct Query {
        DnsResolver* owner = nullptr;
        UdpHandler handler{};
        std::array<char, max_name> name{};
        std::size_t name_len = 0;
        Callback callback = nullptr;
        void* context = nullptr;
        std::uint16_t id = 0;
        std::uint16_t port = 0;   ///< 0: not bound
        Ipv4Address server{};
        std::uint8_t attempt = 0;
        std::uint32_t deadline_ms = 0;
        bool active = false;
        [[nodiscard]] std::string_view view() const noexcept { return {name.data(), name_len}; }
    };

    struct CacheEntry {
        std::array<char, max_name> name{};
        std::size_t name_len = 0;
        Ipv4Address address{};  ///< 0.0.0.0: negative entry
        std::uint32_t expires_ms = 0;
        bool valid = false;
    };

    static bool same_name(std::string_view a, std::string_view b) noexcept {
        if (a.size() != b.size()) {
            return false;
        }
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (dns::lower(a[i]) != dns::lower(b[i])) {
                return false;
            }
        }
        return true;
    }

    const CacheEntry* find(std::string_view name) const noexcept {
        for (const auto& e : cache_) {
            if (e.valid && static_cast<std::int32_t>(e.expires_ms - stack_.now_ms()) > 0 &&
                same_name({e.name.data(), e.name_len}, name)) {
                return &e;
            }
        }
        return nullptr;
    }

    void store(std::string_view name, Ipv4Address address, std::uint32_t ttl_s) noexcept {
        CacheEntry* slot = &cache_[0];
        for (auto& e : cache_) {
            if (!e.valid || same_name({e.name.data(), e.name_len}, name)) {
                slot = &e;
                break;
            }
            if (static_cast<std::int32_t>(e.expires_ms - slot->expires_ms) < 0) {
                slot = &e;  // evict the entry that expires first
            }
        }
        std::copy(name.begin(), name.end(), slot->name.begin());
        slot->name_len = name.size();
        slot->address = address;
        slot->expires_ms = stack_.now_ms() + std::clamp(ttl_s, min_ttl_s, max_ttl_s) * 1000;
        slot->valid = true;
    }

    void unbind(Query& q) noexcept {
        if (q.port != 0) {
            stack_.udp_unbind(q.port);
            q.port = 0;
        }
    }

    void send(Query& q) noexcept {
        q.server = servers_[(q.attempt / attempts_per_server) % server_count_];
        ++q.attempt;
        q.deadline_ms = stack_.now_ms() + attempt_ms;
        // Fresh ID and source port for every attempt.
        unbind(q);
        for (int tries = 0; tries < 8 && q.port == 0; ++tries) {
            const auto port = static_cast<std::uint16_t>(49152 + stack_.random32() % 16384);
            if (stack_.udp_bind(port, q.handler)) {
                q.port = port;
            }
        }
        if (q.port == 0) {
            return;  // no free UDP socket: the attempt times out
        }
        q.id = static_cast<std::uint16_t>(stack_.random32());
        PacketRef p = stack_.allocate();
        if (!p) {
            return;
        }
        std::byte* h = p->append(dns::header_size);
        std::fill(h, h + dns::header_size, std::byte{0});
        store_be16(h, q.id);
        store_be16(h + 2, 0x0100);  // standard query, recursion desired
        store_be16(h + 4, 1);       // one question
        const std::size_t n = dns::write_name(p->data() + p->size(), p->tailroom(), q.view());
        if (n == 0) {
            return;
        }
        p->append(n);
        std::byte* t = p->append(4);
        store_be16(t, dns::type_a);
        store_be16(t + 2, dns::class_in);
        stack_.udp_send(std::move(p), q.server, server_port, q.port);
    }

    void finish(Query& q, std::optional<Ipv4Address> address, std::uint32_t ttl_s) noexcept {
        unbind(q);
        q.active = false;
        if (address || ttl_s != 0) {
            store(q.view(), address.value_or(Ipv4Address::any()), address ? ttl_s : negative_ttl_s);
        }
        q.callback(q.context, q.view(), address);
    }

    static void received(void* self, const UdpDatagram& d) noexcept {
        auto& q = *static_cast<Query*>(self);
        q.owner->reply(q, d);
    }

    void reply(Query& q, const UdpDatagram& d) noexcept {
        const auto m = d.payload;
        if (!q.active || d.source != q.server || d.source_port != server_port || m.size() < dns::header_size ||
            load_be16(m.data()) != q.id) {
            return;  // not the answer to this query: ignore (spoofing, late reply)
        }
        const std::uint16_t flags = load_be16(m.data() + 2);
        if ((flags & dns::flag_qr) == 0 || ((flags >> 11) & 0xF) != 0 || load_be16(m.data() + 4) != 1) {
            return;
        }
        // Question must be ours.
        const auto qend = dns::match_name(m, dns::header_size, q.view());
        if (!qend || *qend + 4 > m.size() || load_be16(m.data() + *qend) != dns::type_a ||
            load_be16(m.data() + *qend + 2) != dns::class_in) {
            return;
        }
        const std::uint16_t rcode = flags & 0xF;
        if (rcode == 3) {
            finish(q, std::nullopt, negative_ttl_s);  // NXDOMAIN
            return;
        }
        if (rcode != 0 || (flags & 0x0200) != 0) {  // server failure, refused, truncated
            q.deadline_ms = stack_.now_ms();         // next attempt / server right away
            return;
        }
        // Answers: follow CNAMEs from the question name to an A record.
        std::array<char, max_name> target{};
        std::size_t target_len = q.name_len;
        std::copy(q.name.begin(), q.name.begin() + static_cast<std::ptrdiff_t>(q.name_len), target.begin());
        const std::uint16_t answers = load_be16(m.data() + 6);
        for (int pass = 0; pass < 8; ++pass) {
            bool followed = false;
            std::size_t pos = *qend + 4;
            for (std::uint16_t i = 0; i < answers; ++i) {
                const auto owner_end = dns::skip_name(m, pos);
                if (!owner_end || *owner_end + 10 > m.size()) {
                    return;
                }
                const std::size_t r = *owner_end;
                const std::uint16_t type = load_be16(m.data() + r);
                const std::uint16_t klass = load_be16(m.data() + r + 2);
                const std::uint32_t ttl = load_be32(m.data() + r + 4);
                const std::size_t rdlength = load_be16(m.data() + r + 8);
                const std::size_t rdata = r + 10;
                if (rdata + rdlength > m.size()) {
                    return;
                }
                const bool owner_matches = dns::match_name(m, pos, {target.data(), target_len}).has_value();
                if (owner_matches && klass == dns::class_in && type == dns::type_a && rdlength == 4) {
                    finish(q, Ipv4Address::load(m.data() + rdata), ttl);
                    return;
                }
                if (owner_matches && klass == dns::class_in && type == type_cname) {
                    const auto len = dns::read_name(m, rdata, target);
                    if (!len) {
                        return;
                    }
                    target_len = *len;
                    followed = true;
                    break;  // look for the new name from the start
                }
                pos = rdata + rdlength;
            }
            if (!followed) {
                break;
            }
        }
        finish(q, std::nullopt, negative_ttl_s);  // no A record (NODATA)
    }

    Stack& stack_;
    std::array<Ipv4Address, 2> servers_{};
    std::size_t server_count_ = 0;
    std::array<Query, MaxQueries> queries_{};
    std::array<CacheEntry, CacheEntries> cache_{};
};

}  // namespace rtos::net
