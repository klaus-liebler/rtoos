// SPDX-License-Identifier: MIT
//
// Host unit tests for mDNS and SNTP on top of the stack (fixture: net_fixture.hpp).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "net_fixture.hpp"
#include "rtos/net/dns.hpp"
#include "rtos/net/mdns.hpp"
#include "rtos/net/sntp.hpp"

namespace {

using namespace test;

// ---------------------------------------------------------------------------
// DNS helpers
// ---------------------------------------------------------------------------

std::vector<std::byte> encode(std::string_view name) {
    std::vector<std::byte> out(name.size() + 2);
    out.resize(dns::write_name(out.data(), out.size(), name));
    return out;
}

TEST(Dns, NameRoundTripAndCaseInsensitiveMatch) {
    const auto n = encode("fcu.local");
    ASSERT_EQ(n.size(), 11u);  // 3 'fcu' 5 'local' 0
    EXPECT_EQ(dns::match_name(n, 0, "FCU.Local"), std::optional<std::size_t>(11));
    EXPECT_FALSE(dns::match_name(n, 0, "fcu.lan"));
    EXPECT_FALSE(dns::match_name(n, 0, "fcu"));
    EXPECT_FALSE(dns::match_name(n, 0, "fcu.local.x"));
}

TEST(Dns, CompressionPointerIsFollowed) {
    // offset 0: "local" ; offset 7: "fcu" + pointer to 0
    std::vector<std::byte> m = encode("local");
    m.push_back(std::byte{3});
    for (char c : std::string_view("fcu")) {
        m.push_back(static_cast<std::byte>(c));
    }
    m.push_back(std::byte{0xC0});
    m.push_back(std::byte{0x00});
    EXPECT_EQ(dns::match_name(m, 7, "fcu.local"), std::optional<std::size_t>(13));
}

TEST(Dns, PointerLoopIsRejected) {
    const std::vector<std::byte> m{std::byte{0xC0}, std::byte{0x00}};
    EXPECT_FALSE(dns::match_name(m, 0, "x"));
}

// ---------------------------------------------------------------------------
// mDNS
// ---------------------------------------------------------------------------

std::vector<std::byte> mdns_query(std::string_view name, std::uint16_t id = 0, std::uint16_t klass = dns::class_in,
                                  std::uint16_t type = dns::type_a) {
    std::vector<std::byte> q(12);
    store_be16(q.data(), id);
    store_be16(q.data() + 4, 1);
    const auto n = encode(name);
    q.insert(q.end(), n.begin(), n.end());
    q.resize(q.size() + 4);
    store_be16(q.data() + q.size() - 4, type);
    store_be16(q.data() + q.size() - 2, klass);
    return q;
}

class MdnsTest : public NetTest {
protected:
    MdnsResponder<TestStack> mdns{stack, "fcu"};

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(mdns.start());
    }

    /// The A record at the end of a response: address and class field.
    static std::pair<Ipv4Address, std::uint16_t> answer(const Frame& f) {
        const std::string_view p = f.udp_payload();
        const auto* end = reinterpret_cast<const std::byte*>(p.data() + p.size());
        return {Ipv4Address::load(end - 4), load_be16(end - 12)};
    }
};

TEST_F(MdnsTest, AnnouncesTwiceAtStartup) {
    mdns.service();
    auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ip_dst(), mdns_group);
    EXPECT_EQ(f[0].dst_mac(), mdns_group.multicast_mac());
    EXPECT_EQ(answer(f[0]).first, device_ip);
    poll(999);
    mdns.service();
    EXPECT_TRUE(sent().empty());
    poll(1);
    mdns.service();
    EXPECT_EQ(sent().size(), 1u);
    poll(5000);
    mdns.service();
    EXPECT_TRUE(sent().empty());  // exactly two
}

TEST_F(MdnsTest, AnswersMulticastQueryOnTheGroup) {
    deliver(udp(mdns_port, mdns_port, mdns_query("fcu.local"), mdns_group));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ip_dst(), mdns_group);
    const std::string_view p = f[0].udp_payload();
    EXPECT_EQ(load_be16(reinterpret_cast<const std::byte*>(p.data())), 0);  // ID 0
    EXPECT_EQ(answer(f[0]).first, device_ip);
    EXPECT_EQ(answer(f[0]).second, dns::class_in | 0x8000);  // cache flush
    EXPECT_EQ(mdns.answered(), 1u);
}

TEST_F(MdnsTest, LegacyUnicastQueryGetsUnicastAnswerWithIdAndQuestion) {
    deliver(udp(54321, mdns_port, mdns_query("FCU.local", 0xBEEF), mdns_group));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ip_dst(), peer_ip);
    EXPECT_EQ(load_be16(f[0].l4() + 2), 54321);  // back to the querier's port
    const auto* m = reinterpret_cast<const std::byte*>(f[0].udp_payload().data());
    EXPECT_EQ(load_be16(m), 0xBEEF);
    EXPECT_EQ(load_be16(m + 4), 1);  // question repeated
    EXPECT_EQ(answer(f[0]).second, dns::class_in);  // no cache-flush bit
}

TEST_F(MdnsTest, QuQuestionIsAnsweredByUnicast) {
    deliver(udp(mdns_port, mdns_port, mdns_query("fcu.local", 0, dns::class_in | 0x8000), mdns_group));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ip_dst(), peer_ip);
    EXPECT_EQ(load_be16(f[0].l4() + 2), mdns_port);
}

TEST_F(MdnsTest, IgnoresOtherNamesTypesAndResponses) {
    deliver(udp(mdns_port, mdns_port, mdns_query("other.local"), mdns_group));
    deliver(udp(mdns_port, mdns_port, mdns_query("fcu.local", 0, dns::class_in, 28 /* AAAA */), mdns_group));
    auto response = mdns_query("fcu.local");
    store_be16(response.data() + 2, dns::flag_qr);
    deliver(udp(mdns_port, mdns_port, response, mdns_group));
    EXPECT_TRUE(sent().empty());
}

// ---------------------------------------------------------------------------
// SNTP
// ---------------------------------------------------------------------------

class SntpTest : public NetTest {
protected:
    static constexpr std::uint16_t server_port = 12345;
    SntpClient<TestStack> sntp{stack, peer_ip, server_port, 64'000, 12'300};

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(sntp.start());
    }

    /// Server reply to `request`, with receive time `t2` and transmit time `t3` (unix seconds).
    std::vector<std::byte> reply(const Frame& request, std::uint32_t t2, std::uint32_t t3, std::uint8_t stratum = 2) {
        std::vector<std::byte> m(48);
        m[0] = std::byte{0x24};  // version 4, mode 4
        m[1] = std::byte{stratum};
        const auto* req = reinterpret_cast<const std::byte*>(request.udp_payload().data());
        std::memcpy(m.data() + 24, req + 40, 8);  // originate = client transmit
        store_be32(m.data() + 32, static_cast<std::uint32_t>(t2 + ntp_unix_offset_s));
        store_be32(m.data() + 40, static_cast<std::uint32_t>(t3 + ntp_unix_offset_s));
        return m;
    }
};

TEST_F(SntpTest, SynchronizesWithDelayCompensation) {
    sntp.service();
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ip_dst(), peer_ip);
    EXPECT_EQ(load_be16(f[0].l4() + 2), server_port);
    EXPECT_EQ(f[0].udp_payload()[0], '\x23');

    const std::uint32_t t = 1'800'000'000;  // 2027-01-15
    now += 40;                             // 40 ms round trip, server holds 0
    deliver(udp(server_port, 12'300, reply(f[0], t, t)));
    ASSERT_TRUE(sntp.synchronized());
    EXPECT_EQ(sntp.unix_ms(now), std::uint64_t{t} * 1000 + 20);
    EXPECT_EQ(sntp.unix_ms(now + 1000), std::uint64_t{t} * 1000 + 1020);
}

TEST_F(SntpTest, RejectsReplyWithWrongOriginate) {
    sntp.service();
    auto f = sent();
    auto m = reply(f[0], 1'800'000'000, 1'800'000'000);
    m[24] ^= std::byte{1};
    deliver(udp(server_port, 12'300, m));
    EXPECT_FALSE(sntp.synchronized());
}

TEST_F(SntpTest, RejectsUnsynchronizedServer) {
    sntp.service();
    auto f = sent();
    deliver(udp(server_port, 12'300, reply(f[0], 1'800'000'000, 1'800'000'000, 0)));
    EXPECT_FALSE(sntp.synchronized());
}

TEST_F(SntpTest, RetriesThenBacksOff) {
    sntp.service();
    EXPECT_EQ(sent().size(), 1u);
    for (int i = 0; i < 2; ++i) {
        poll(2000);
        sntp.service();
        EXPECT_EQ(sent().size(), 1u);  // retry
    }
    poll(2000);
    sntp.service();
    EXPECT_TRUE(sent().empty());  // gave up until the next interval
    poll(64'000);
    sntp.service();
    EXPECT_EQ(sent().size(), 1u);
}

}  // namespace
