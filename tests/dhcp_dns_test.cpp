// SPDX-License-Identifier: MIT
//
// Host unit tests for the DHCP client and the DNS resolver (fixture: net_fixture.hpp).
#include <gtest/gtest.h>

#include <optional>
#include <string>
#include <vector>

#include "net_fixture.hpp"
#include "rtos/net/dhcp.hpp"
#include "rtos/net/dns.hpp"
#include "rtos/net/dns_resolver.hpp"

namespace {

using namespace test;

constexpr Ipv4Address server_ip = peer_ip;  // 192.168.77.1
constexpr Ipv4Address router_ip = Ipv4Address::from(192, 168, 77, 254);
constexpr Ipv4Address dns2_ip = Ipv4Address::from(192, 168, 77, 53);

// Fixture: random32() == 100000 -> retransmission jitter +951 ms, DNS port 50848, ID 34464
constexpr std::uint32_t jitter_ms = 951;

// ---------------------------------------------------------------------------
// DHCP
// ---------------------------------------------------------------------------

/// Parsed DHCP message as sent by the client.
struct Dhcp {
    std::vector<std::byte> m;
    std::uint32_t xid() const { return load_be32(m.data() + 4); }
    std::uint16_t flags() const { return load_be16(m.data() + 10); }
    Ipv4Address ciaddr() const { return Ipv4Address::load(m.data() + 12); }
    std::optional<std::vector<std::byte>> option(std::uint8_t code) const {
        for (std::size_t pos = 240; pos < m.size();) {
            const auto c = std::to_integer<std::uint8_t>(m[pos]);
            if (c == 0) {
                ++pos;
                continue;
            }
            if (c == 255) {
                break;
            }
            const auto len = std::to_integer<std::size_t>(m[pos + 1]);
            if (c == code) {
                return std::vector<std::byte>(m.begin() + static_cast<std::ptrdiff_t>(pos + 2),
                                              m.begin() + static_cast<std::ptrdiff_t>(pos + 2 + len));
            }
            pos += 2 + len;
        }
        return std::nullopt;
    }
    std::uint8_t type() const { return std::to_integer<std::uint8_t>((*option(53))[0]); }
};

struct ServerReply {
    std::uint8_t type;
    Ipv4Address yiaddr = device_ip;
    Ipv4Address server = server_ip;
    std::uint32_t lease_s = 3600;
    bool ntp = true;
    // malformed or unusual replies
    bool type_option = true;      ///< option 53
    bool server_id = true;        ///< option 54
    std::uint32_t t1_s = 0;       ///< option 58 (0: absent)
    std::uint32_t t2_s = 0;       ///< option 59 (0: absent)
    bool truncated = false;       ///< last option runs past the end, no end option
};

class DhcpTest : public NetTest {
protected:
    void SetUp() override {
        NetTest::SetUp();
        stack.set_config(eth, {});  // DHCP starts unconfigured
    }

    void poll(std::uint32_t advance_ms = 0) {
        NetTest::poll(advance_ms);
        client.service();
    }

    std::vector<Dhcp> dhcp_sent() {
        std::vector<Dhcp> out;
        for (const Frame& f : sent()) {
            if (f.ethertype() == ethertype_ipv4 && f.ip_protocol() == ip_proto_udp && f.udp_dst_port() == 67) {
                const auto p = f.udp_payload();
                out.push_back({std::vector<std::byte>(reinterpret_cast<const std::byte*>(p.data()),
                                                      reinterpret_cast<const std::byte*>(p.data()) + p.size())});
            }
        }
        return out;
    }

    void reply(std::uint32_t xid, const ServerReply& r, Ipv4Address dst = Ipv4Address::broadcast()) {
        std::vector<std::byte> m(240);
        m[0] = std::byte{2};
        m[1] = std::byte{1};
        m[2] = std::byte{6};
        store_be32(m.data() + 4, xid);
        r.yiaddr.store(m.data() + 16);
        device_mac.store(m.data() + 28);
        store_be32(m.data() + 236, 0x63825363);
        auto option = [&m](std::uint8_t code, std::vector<std::byte> v) {
            m.push_back(std::byte{code});
            m.push_back(static_cast<std::byte>(v.size()));
            m.insert(m.end(), v.begin(), v.end());
        };
        auto addr = [](std::initializer_list<Ipv4Address> list) {
            std::vector<std::byte> v;
            for (Ipv4Address a : list) {
                v.resize(v.size() + 4);
                a.store(v.data() + v.size() - 4);
            }
            return v;
        };
        if (r.type_option) {
            option(53, {std::byte{r.type}});
        }
        if (r.server_id) {
            option(54, addr({r.server}));
        }
        option(1, addr({Ipv4Address::from(255, 255, 255, 0)}));
        option(3, addr({router_ip}));
        option(6, addr({server_ip, dns2_ip}));
        if (r.ntp) {
            option(42, addr({server_ip}));
        }
        std::vector<std::byte> lease(4);
        store_be32(lease.data(), r.lease_s);
        option(51, lease);
        for (auto [code, t] : {std::pair{58, r.t1_s}, std::pair{59, r.t2_s}}) {
            if (t != 0) {
                std::vector<std::byte> v(4);
                store_be32(v.data(), t);
                option(static_cast<std::uint8_t>(code), v);
            }
        }
        if (r.truncated) {
            m.push_back(std::byte{15});  // domain name, 40 bytes announced, 2 present
            m.push_back(std::byte{40});
            m.push_back(std::byte{'l'});
            m.push_back(std::byte{'a'});
        } else {
            m.push_back(std::byte{255});
        }
        deliver(udp(67, 68, m, dst, r.server));
    }

    /// ACKed address probed with ARP (RFC 5227) until the client is bound.
    void finish_check() {
        for (int i = 0; i < 200 && client.state() == DhcpClient<TestStack>::State::Checking; ++i) {
            poll(100);
        }
        EXPECT_TRUE(client.bound());
    }

    /// DISCOVER -> OFFER -> REQUEST -> ACK -> probe. Returns the time of the REQUEST
    /// (lease timers count from it).
    std::uint32_t acquire(std::uint32_t lease_s = 3600) {
        client.start();
        poll();
        auto d = dhcp_sent();
        EXPECT_EQ(d.size(), 1u);
        reply(d.at(0).xid(), {.type = 2, .lease_s = lease_s});
        const std::uint32_t requested_at = now;
        auto r = dhcp_sent();
        EXPECT_EQ(r.size(), 1u);
        reply(r.at(0).xid(), {.type = 5, .lease_s = lease_s});
        finish_check();
        (void)sent();
        return requested_at;
    }

    /// Advance the clock to `t` (polling once at t).
    void advance_to(std::uint32_t t) { poll(t - now); }

    static void on_bound(void* self, const DhcpLease& lease, bool renewed) noexcept {
        auto& t = *static_cast<DhcpTest*>(self);
        t.last_lease = lease;
        renewed ? ++t.renewals : ++t.bindings;
    }
    static void on_lost(void* self) noexcept { ++static_cast<DhcpTest*>(self)->losses; }
    static void on_declined(void* self, Ipv4Address) noexcept { ++static_cast<DhcpTest*>(self)->declines; }

    DhcpClient<TestStack>::Handler handler{this, &on_bound, &on_lost, &on_declined};
    DhcpClient<TestStack> client{stack, eth, "fcu", handler};
    std::optional<DhcpLease> last_lease;
    int bindings = 0, renewals = 0, losses = 0, declines = 0;
};

TEST_F(DhcpTest, DiscoverIsBroadcastFromUnconfiguredStack) {
    client.start();
    poll();
    const auto frames = sent();
    ASSERT_EQ(frames.size(), 1u);
    const Frame& f = frames[0];
    EXPECT_EQ(f.dst_mac(), MacAddress::broadcast());
    EXPECT_EQ(f.ip_src(), Ipv4Address::any());
    EXPECT_EQ(f.ip_dst(), Ipv4Address::broadcast());
    EXPECT_EQ(f.udp_src_port(), 68);
    const auto p = f.udp_payload();
    Dhcp d{std::vector<std::byte>(reinterpret_cast<const std::byte*>(p.data()),
                                  reinterpret_cast<const std::byte*>(p.data()) + p.size())};
    EXPECT_GE(d.m.size(), 300u);  // BOOTP minimum
    EXPECT_EQ(d.type(), 1);
    EXPECT_EQ(d.flags(), 0x8000);  // BROADCAST
    EXPECT_EQ(MacAddress::load(d.m.data() + 28), device_mac);
    const auto host = *d.option(12);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(host.data()), host.size()), "fcu");
    EXPECT_TRUE(d.option(55).has_value());
    EXPECT_FALSE(d.option(50).has_value());
}

TEST_F(DhcpTest, OfferRequestAckConfiguresStack) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid, {.type = 2});
    const auto r = dhcp_sent();
    ASSERT_EQ(r.size(), 1u);
    EXPECT_EQ(r[0].type(), 3);
    EXPECT_EQ(r[0].xid(), xid);
    std::vector<std::byte> requested(4), server(4);
    device_ip.store(requested.data());
    server_ip.store(server.data());
    EXPECT_EQ(r[0].option(50), requested);
    EXPECT_EQ(r[0].option(54), server);
    EXPECT_FALSE(stack.configured(eth));

    reply(xid, {.type = 5});
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Checking);
    EXPECT_FALSE(stack.configured(eth));  // probing (RFC 5227) before use
    finish_check();
    EXPECT_EQ(stack.local_address(eth), device_ip);
    EXPECT_EQ(stack.config(eth).netmask, Ipv4Address::from(255, 255, 255, 0));
    EXPECT_EQ(stack.config(eth).gateway, router_ip);
    ASSERT_TRUE(last_lease);
    EXPECT_EQ(bindings, 1);
    EXPECT_EQ(last_lease->dns[0], server_ip);
    EXPECT_EQ(last_lease->dns[1], dns2_ip);
    EXPECT_EQ(last_lease->ntp[0], server_ip);
    EXPECT_EQ(last_lease->lease_s, 3600u);
}

TEST_F(DhcpTest, ReplyForOtherClientIsIgnored) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid + 1, {.type = 2});
    EXPECT_TRUE(dhcp_sent().empty());
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
}

TEST_F(DhcpTest, DiscoverIsRetransmittedWithBackoff) {
    client.start();
    poll();
    ASSERT_EQ(dhcp_sent().size(), 1u);
    poll(4000 + jitter_ms - 1);
    EXPECT_TRUE(dhcp_sent().empty());
    poll(1);
    ASSERT_EQ(dhcp_sent().size(), 1u);  // after 4 s (+jitter)
    poll(8000 + jitter_ms - 1);
    EXPECT_TRUE(dhcp_sent().empty());
    poll(1);
    EXPECT_EQ(dhcp_sent().size(), 1u);  // then 8 s
}

TEST_F(DhcpTest, RenewsAtT1WithUnicastToServer) {
    const std::uint32_t t0 = acquire(100);  // T1 = 50 s after the request
    prime_arp();  // the new address flushed the ARP cache: the unicast needs the server's MAC
    advance_to(t0 + 50'000 - 1);
    EXPECT_TRUE(dhcp_sent().empty());
    poll(1);
    const auto r = dhcp_sent();
    ASSERT_EQ(r.size(), 1u);
    EXPECT_EQ(r[0].type(), 3);
    EXPECT_EQ(r[0].ciaddr(), device_ip);
    EXPECT_EQ(r[0].flags(), 0);
    EXPECT_FALSE(r[0].option(54).has_value());  // no server id while renewing
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Renewing);
    reply(r[0].xid(), {.type = 5, .lease_s = 100}, device_ip);
    EXPECT_EQ(renewals, 1);
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Bound);
}

// Malformed and unusual server replies (cases from the NetX Duo dhcp_test and the
// FreeRTOS-Plus-TCP ProcessDHCPReplies proofs).

TEST_F(DhcpTest, OfferWithoutTypeOrServerIdIsIgnored) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid, {.type = 2, .type_option = false});
    reply(xid, {.type = 2, .server_id = false});
    EXPECT_TRUE(dhcp_sent().empty());
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
    reply(xid, {.type = 2});
    EXPECT_EQ(dhcp_sent().size(), 1u);  // still answers a proper offer
}

TEST_F(DhcpTest, TruncatedOptionsAreRejected) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid, {.type = 2, .truncated = true});
    EXPECT_TRUE(dhcp_sent().empty());
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
}

TEST_F(DhcpTest, InfiniteLeaseIsCappedNotOverflowed) {
    const std::uint32_t t0 = acquire(0xFFFF'FFFFu);  // RFC 2131: infinite
    prime_arp();
    constexpr std::uint32_t t1_ms = 12u * 24 * 3600 * 1000;  // half of the 24-day cap
    advance_to(t0 + t1_ms - 1);
    EXPECT_TRUE(dhcp_sent().empty());
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Bound);
    poll(1);
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Renewing);
    (void)sent();  // ARP request: the server's entry expired during the 12 days
    deliver(arp(2, peer_mac, server_ip, device_ip, device_mac));
    EXPECT_EQ(dhcp_sent().size(), 1u);  // the parked REQUEST
}

TEST_F(DhcpTest, InconsistentT1T2FallBackToDefaults) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid, {.type = 2, .lease_s = 100, .t1_s = 90, .t2_s = 30});  // T1 after T2
    const std::uint32_t t0 = now;
    (void)dhcp_sent();
    reply(xid, {.type = 5, .lease_s = 100, .t1_s = 90, .t2_s = 30});
    finish_check();
    prime_arp();
    advance_to(t0 + 50'000 - 1);  // default T1 = 0.5 x lease
    EXPECT_TRUE(dhcp_sent().empty());
    poll(1);
    EXPECT_EQ(dhcp_sent().size(), 1u);
}

TEST_F(DhcpTest, LeaseExpiresWithoutAnswer) {
    acquire(100);  // T1 50 s, T2 87.5 s, expiry 100 s
    for (int i = 0; i < 200 && losses == 0; ++i) {
        poll(1000);
        (void)sent();
    }
    EXPECT_EQ(losses, 1);
    EXPECT_FALSE(stack.configured(eth));
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
    EXPECT_GE(now, 1000u + 100'000u);
}

TEST_F(DhcpTest, AddressInUseIsDeclined) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid, {.type = 2});
    (void)dhcp_sent();
    reply(xid, {.type = 5});
    ASSERT_EQ(client.state(), DhcpClient<TestStack>::State::Checking);
    poll(1500);
    deliver(arp(2, MacAddress::from(0x02, 0, 0, 0, 0, 0x99), device_ip, Ipv4Address::any()));  // in use
    const auto d = dhcp_sent();
    ASSERT_EQ(d.size(), 1u);
    EXPECT_EQ(d[0].type(), 4);  // DHCPDECLINE
    std::vector<std::byte> requested(4);
    device_ip.store(requested.data());
    EXPECT_EQ(d[0].option(50), requested);
    EXPECT_EQ(declines, 1);
    EXPECT_FALSE(stack.configured(eth));
    poll(DhcpClient<TestStack>::decline_wait_ms - 1);
    EXPECT_TRUE(dhcp_sent().empty());  // wait 10 s (RFC 2131 3.1.5)
    poll(1);
    EXPECT_EQ(dhcp_sent().at(0).type(), 1);
}

TEST_F(DhcpTest, LinkBackVerifiesLeaseWithInitReboot) {
    (void)acquire();
    driver.link = false;
    poll();
    driver.link = true;
    poll();
    const auto r = dhcp_sent();
    ASSERT_EQ(r.size(), 1u);
    EXPECT_EQ(r[0].type(), 3);
    EXPECT_EQ(r[0].ciaddr(), Ipv4Address::any());
    EXPECT_EQ(r[0].flags(), 0x8000);
    EXPECT_FALSE(r[0].option(54).has_value());
    std::vector<std::byte> requested(4);
    device_ip.store(requested.data());
    EXPECT_EQ(r[0].option(50), requested);
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Rebooting);
    reply(r[0].xid(), {.type = 5});
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Bound);
    EXPECT_EQ(renewals, 1);
}

TEST_F(DhcpTest, InitRebootNakStartsOver) {
    (void)acquire();
    driver.link = false;
    poll();
    driver.link = true;
    poll();
    reply(dhcp_sent().at(0).xid(), {.type = 6});
    EXPECT_EQ(losses, 1);
    EXPECT_FALSE(stack.configured(eth));
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
}

TEST_F(DhcpTest, InitRebootWithoutAnswerKeepsLease) {
    (void)acquire();
    driver.link = false;
    poll();
    driver.link = true;
    for (int i = 0; i < 40 && client.state() == DhcpClient<TestStack>::State::Rebooting; ++i) {
        poll(1000);
        (void)sent();
    }
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Bound);
    EXPECT_TRUE(stack.configured(eth));
    EXPECT_EQ(losses, 0);
}

TEST_F(DhcpTest, LostAddressDefenceDeclinesAndStartsOver) {
    (void)acquire();
    const auto intruder = MacAddress::from(0x02, 0, 0, 0, 0, 0x99);
    deliver(arp(1, intruder, device_ip, server_ip));
    deliver(arp(1, intruder, device_ip, server_ip));  // second within 10 s: give up
    EXPECT_EQ(losses, 1);
    EXPECT_EQ(declines, 1);
    EXPECT_FALSE(stack.configured(eth));
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
}

TEST_F(DhcpTest, NakRestartsDiscovery) {
    client.start();
    poll();
    const std::uint32_t xid = dhcp_sent().at(0).xid();
    reply(xid, {.type = 2});
    (void)dhcp_sent();
    reply(xid, {.type = 6});  // DHCPNAK
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Selecting);
    EXPECT_EQ(losses, 1);
    poll();
    const auto d = dhcp_sent();
    ASSERT_EQ(d.size(), 1u);
    EXPECT_EQ(d[0].type(), 1);
}

TEST_F(DhcpTest, UnconfiguredStackAnswersNoArp) {
    std::array<std::byte, 28> a{};
    store_be16(a.data(), 1);
    store_be16(a.data() + 2, ethertype_ipv4);
    a[4] = std::byte{6};
    a[5] = std::byte{4};
    store_be16(a.data() + 6, 1);  // request for 0.0.0.0
    peer_mac.store(a.data() + 8);
    peer_ip.store(a.data() + 14);
    deliver(frame(ethertype_arp, a, MacAddress::broadcast()));
    EXPECT_TRUE(sent().empty());
}

// ---------------------------------------------------------------------------
// DNS resolver
// ---------------------------------------------------------------------------

using Resolver = DnsResolver<TestStack>;
constexpr std::uint16_t query_port = 49152 + 100000 % 16384;
constexpr std::uint16_t query_id = static_cast<std::uint16_t>(100000);

class DnsTest : public NetTest {
protected:
    void SetUp() override {
        NetTest::SetUp();
        const std::array servers{server_ip, dns2_ip};
        resolver.set_servers(servers);
        prime_arp(dns2_ip);  // second server on the link, MAC known (as for the peer)
    }

    void poll(std::uint32_t advance_ms = 0) {
        NetTest::poll(advance_ms);
        resolver.service();
    }

    static void done(void* self, std::string_view, std::optional<Ipv4Address> a) noexcept {
        auto& t = *static_cast<DnsTest*>(self);
        t.result = a;
        ++t.callbacks;
    }

    bool resolve(std::string_view name) { return resolver.resolve(name, &DnsTest::done, this); }

    /// Answer: question + records [(owner, type, rdata)].
    struct Record {
        std::string owner;
        std::uint16_t type;
        std::vector<std::byte> rdata;
        std::uint32_t ttl = 300;
    };
    std::vector<std::byte> response(std::string_view qname, std::vector<Record> records, std::uint16_t rcode = 0,
                                    std::uint16_t id = query_id) {
        std::vector<std::byte> m(12);
        store_be16(m.data(), id);
        store_be16(m.data() + 2, static_cast<std::uint16_t>(0x8180 | rcode));
        store_be16(m.data() + 4, 1);
        store_be16(m.data() + 6, static_cast<std::uint16_t>(records.size()));
        auto name = [&m](std::string_view n) {
            std::vector<std::byte> b(n.size() + 2);
            b.resize(dns::write_name(b.data(), b.size(), n));
            m.insert(m.end(), b.begin(), b.end());
        };
        name(qname);
        m.resize(m.size() + 4);
        store_be16(m.data() + m.size() - 4, dns::type_a);
        store_be16(m.data() + m.size() - 2, dns::class_in);
        for (const auto& r : records) {
            name(r.owner);
            m.resize(m.size() + 10);
            std::byte* h = m.data() + m.size() - 10;
            store_be16(h, r.type);
            store_be16(h + 2, dns::class_in);
            store_be32(h + 4, r.ttl);
            store_be16(h + 8, static_cast<std::uint16_t>(r.rdata.size()));
            m.insert(m.end(), r.rdata.begin(), r.rdata.end());
        }
        return m;
    }
    static std::vector<std::byte> a(Ipv4Address ip) {
        std::vector<std::byte> v(4);
        ip.store(v.data());
        return v;
    }
    static std::vector<std::byte> name_rdata(std::string_view n) {
        std::vector<std::byte> b(n.size() + 2);
        b.resize(dns::write_name(b.data(), b.size(), n));
        return b;
    }

    Resolver resolver{stack};
    std::optional<Ipv4Address> result;
    int callbacks = 0;
};

TEST_F(DnsTest, QueryUsesRandomIdAndSourcePort) {
    ASSERT_TRUE(resolve("pool.ntp.org"));
    const auto frames = sent();
    ASSERT_EQ(frames.size(), 1u);
    EXPECT_EQ(frames[0].ip_dst(), server_ip);
    EXPECT_EQ(frames[0].udp_dst_port(), 53);
    EXPECT_EQ(frames[0].udp_src_port(), query_port);
    const auto p = frames[0].udp_payload();
    const std::span<const std::byte> q(reinterpret_cast<const std::byte*>(p.data()), p.size());
    EXPECT_EQ(load_be16(q.data()), query_id);
    EXPECT_EQ(load_be16(q.data() + 2), 0x0100);  // RD
    EXPECT_TRUE(dns::match_name(q, 12, "pool.ntp.org").has_value());
}

TEST_F(DnsTest, AnswerIsDeliveredAndCached) {
    ASSERT_TRUE(resolve("pool.ntp.org"));
    (void)sent();
    const auto ntp = Ipv4Address::from(10, 0, 0, 123);
    deliver(udp(53, query_port, response("pool.ntp.org", {{"pool.ntp.org", dns::type_a, a(ntp)}})));
    EXPECT_EQ(callbacks, 1);
    EXPECT_EQ(result, ntp);
    EXPECT_EQ(resolver.lookup("POOL.ntp.org"), ntp);  // case-insensitive cache
    ASSERT_TRUE(resolve("pool.ntp.org"));             // cache hit: no query
    EXPECT_EQ(callbacks, 2);
    EXPECT_TRUE(sent().empty());
    poll(300'000);  // TTL 300 s
    EXPECT_FALSE(resolver.lookup("pool.ntp.org"));
}

TEST_F(DnsTest, CnameChainIsFollowed) {
    ASSERT_TRUE(resolve("www.example.com"));
    (void)sent();
    const auto ip = Ipv4Address::from(93, 184, 216, 34);
    deliver(udp(53, query_port,
                response("www.example.com", {{"www.example.com", 5, name_rdata("edge.cdn.net")},
                                             {"edge.cdn.net", 5, name_rdata("node7.cdn.net")},
                                             {"node7.cdn.net", dns::type_a, a(ip)}})));
    EXPECT_EQ(result, ip);
}

TEST_F(DnsTest, SpoofedRepliesAreIgnored) {
    ASSERT_TRUE(resolve("pool.ntp.org"));
    (void)sent();
    const auto rec = std::vector<Record>{{"pool.ntp.org", dns::type_a, a(Ipv4Address::from(6, 6, 6, 6))}};
    deliver(udp(53, query_port, response("pool.ntp.org", rec, 0, query_id + 1)));      // wrong ID
    deliver(udp(5353, query_port, response("pool.ntp.org", rec)));                     // wrong source port
    deliver(udp(53, query_port, response("pool.ntp.org", rec), device_ip, dns2_ip));   // wrong server
    deliver(udp(53, query_port, response("evil.org", {{"evil.org", dns::type_a, a(Ipv4Address::from(6, 6, 6, 6))}})));
    EXPECT_EQ(callbacks, 0);
    EXPECT_FALSE(resolver.lookup("pool.ntp.org"));
}

TEST_F(DnsTest, NxdomainIsCachedNegatively) {
    ASSERT_TRUE(resolve("nonexistent.test"));
    (void)sent();
    deliver(udp(53, query_port, response("nonexistent.test", {}, 3)));
    EXPECT_EQ(callbacks, 1);
    EXPECT_FALSE(result);
    ASSERT_TRUE(resolve("nonexistent.test"));  // negative cache
    EXPECT_EQ(callbacks, 2);
    EXPECT_TRUE(sent().empty());
}

TEST_F(DnsTest, RetriesBothServersThenFails) {
    ASSERT_TRUE(resolve("pool.ntp.org"));
    std::vector<Ipv4Address> targets;
    for (int i = 0; i < 10 && callbacks == 0; ++i) {
        for (const Frame& f : sent()) {
            if (f.ethertype() == ethertype_ipv4) {
                targets.push_back(f.ip_dst());
            }
        }
        poll(Resolver::attempt_ms);
    }
    EXPECT_EQ(targets, (std::vector<Ipv4Address>{server_ip, server_ip, dns2_ip, dns2_ip}));
    EXPECT_EQ(callbacks, 1);
    EXPECT_FALSE(result);
    EXPECT_FALSE(resolver.lookup("pool.ntp.org"));  // timeouts are not cached
}

TEST_F(DnsTest, NothingToAskWithoutServers) {
    resolver.set_servers({});
    EXPECT_FALSE(resolve("pool.ntp.org"));
}

}  // namespace
