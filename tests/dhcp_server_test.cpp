// SPDX-License-Identifier: MIT
//
// DHCP server: end to end against our DHCP client over a back-to-back link between
// two stacks (the shape of a USB CDC-NCM link), and the server's rules with
// hand-made client messages.
#include <gtest/gtest.h>

#include <deque>
#include <optional>
#include <vector>

#include "net_fixture.hpp"
#include "rtos/net/dhcp.hpp"
#include "rtos/net/dhcp_server.hpp"
#include "rtos/net/service.hpp"

namespace {

using namespace test;

/// One end of a back-to-back Ethernet link: what it transmits, the peer receives
/// (copied into the peer's pool, like a real link).
struct PipeEnd {
    PipeEnd(MacAddress mac_address, TestPool* receive_pool) : address(mac_address), pool(receive_pool) {}

    MacAddress address;
    TestPool* pool = nullptr;
    PipeEnd* peer = nullptr;
    std::deque<PacketRef> rx;
    bool link = true;

    MacAddress mac() const noexcept { return address; }
    bool link_up() const noexcept { return link; }
    bool transmit(PacketRef frame) noexcept {
        if (!link || !peer->link) {
            return true;  // lost on the wire
        }
        PacketRef copy = peer->pool->allocate(0);
        if (copy) {
            copy->append(frame->bytes());
            peer->rx.push_back(std::move(copy));
        }
        return true;
    }
    PacketRef receive() noexcept {
        if (rx.empty()) {
            return {};
        }
        PacketRef f = std::move(rx.front());
        rx.pop_front();
        return f;
    }
};
static_assert(NetDriver<PipeEnd>);

constexpr Ipv4Address device_usb_ip = Ipv4Address::from(192, 168, 7, 1);
constexpr Ipv4Address pool_first = Ipv4Address::from(192, 168, 7, 10);

class DhcpOverLinkTest : public ::testing::Test {
protected:
    TestPool pool_device, pool_pc;
    PipeEnd end_device{MacAddress::from(0x02, 0, 0, 0, 0x07, 0x01), &pool_device};
    PipeEnd end_pc{MacAddress::from(0x02, 0, 0, 0, 0x07, 0x02), &pool_pc};
    TestStack device{pool_device};
    TestStack pc{pool_pc};
    InterfaceId device_if = 0, pc_if = 0;
    DhcpServer<TestStack> server{device, 0, {.first = pool_first, .count = 2, .lease_s = 60}};
    DhcpClient<TestStack>::Handler handler{this, &on_bound, nullptr, nullptr};
    DhcpClient<TestStack> client{pc, 0, "pc", handler};
    NetRuntime<TestStack> device_loop{device}, pc_loop{pc};
    std::uint32_t now = 1000;
    std::optional<DhcpLease> lease;
    int renewals = 0;

    void SetUp() override {
        end_device.peer = &end_pc;
        end_pc.peer = &end_device;
        device_if = device.attach(end_device, {device_usb_ip, Ipv4Address::from(255, 255, 255, 0), {}});
        pc_if = pc.attach(end_pc);
        device.set_random([](void*) noexcept -> std::uint32_t { return 4711; }, nullptr);
        pc.set_random([](void*) noexcept -> std::uint32_t { return 100000; }, nullptr);
        device.poll(now);
        ASSERT_TRUE(server.start());
        ASSERT_TRUE(client.start());
        ASSERT_TRUE(device_loop.add(server));
        ASSERT_TRUE(pc_loop.add(client));
    }

    static void on_bound(void* self, const DhcpLease& l, bool renewed) noexcept {
        auto& t = *static_cast<DhcpOverLinkTest*>(self);
        t.lease = l;
        t.renewals += renewed;
    }

    /// Run both stacks for `ms` in 100 ms steps.
    void run(std::uint32_t ms) {
        for (std::uint32_t t = 0; t < ms; t += 100) {
            now += 100;
            (void)device_loop.run_once(now);
            (void)pc_loop.run_once(now);
        }
    }
};

TEST_F(DhcpOverLinkTest, PcGetsAnAddressAndNoDefaultRoute) {
    run(15'000);  // DISCOVER .. ACK, then the client's ARP probe (RFC 5227)
    ASSERT_TRUE(client.bound());
    ASSERT_TRUE(lease);
    EXPECT_EQ(lease->config.address, pool_first);
    EXPECT_EQ(lease->config.netmask, Ipv4Address::from(255, 255, 255, 0));
    EXPECT_EQ(lease->config.gateway, Ipv4Address::any());  // the device is no router
    EXPECT_EQ(lease->server, device_usb_ip);
    EXPECT_EQ(lease->lease_s, 60u);
    EXPECT_EQ(pc.local_address(pc_if), pool_first);
    EXPECT_EQ(server.address_of(end_pc.address), pool_first);
}

TEST_F(DhcpOverLinkTest, RenewalAndRebootKeepTheAddress) {
    run(15'000);
    ASSERT_TRUE(client.bound());
    run(35'000);  // T1 = 30 s: renewal by unicast
    EXPECT_GE(renewals, 1);
    end_pc.link = false;
    run(500);
    end_pc.link = true;
    run(1000);  // INIT-REBOOT: the server confirms
    EXPECT_EQ(client.state(), DhcpClient<TestStack>::State::Bound);
    EXPECT_GE(renewals, 2);
    EXPECT_EQ(pc.local_address(pc_if), pool_first);
}

// ---- server rules with hand-made client messages --------------------------------

class DhcpServerRules : public NetTest {
protected:
    static constexpr Ipv4Address first = Ipv4Address::from(192, 168, 77, 100);
    DhcpServer<TestStack, 2> server{stack, eth, {.first = first, .count = 2}};

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(server.start());
    }

    /// Client message from `mac`; returns the server's reply (type, yiaddr) if any.
    std::optional<std::pair<std::uint8_t, Ipv4Address>> ask(std::uint8_t type, MacAddress mac,
                                                            Ipv4Address requested = {}, Ipv4Address server_id = {},
                                                            Ipv4Address ciaddr = {}) {
        std::vector<std::byte> m(240);
        m[0] = std::byte{1};
        m[1] = std::byte{1};
        m[2] = std::byte{6};
        store_be32(m.data() + 4, 0x1234);
        ciaddr.store(m.data() + 12);
        mac.store(m.data() + 28);
        store_be32(m.data() + 236, dhcp::magic_cookie);
        auto option = [&m](std::uint8_t code, Ipv4Address a) {
            m.push_back(std::byte{code});
            m.push_back(std::byte{4});
            m.resize(m.size() + 4);
            a.store(m.data() + m.size() - 4);
        };
        m.push_back(std::byte{53});
        m.push_back(std::byte{1});
        m.push_back(std::byte{type});
        if (requested != Ipv4Address::any()) {
            option(50, requested);
        }
        if (server_id != Ipv4Address::any()) {
            option(54, server_id);
        }
        m.push_back(std::byte{255});
        deliver(udp(68, 67, m, Ipv4Address::broadcast(), Ipv4Address::any()));
        for (const Frame& f : sent()) {
            if (f.ethertype() == ethertype_ipv4 && f.ip_protocol() == ip_proto_udp && f.udp_dst_port() == 68) {
                const auto p = f.udp_payload();
                const auto* b = reinterpret_cast<const std::byte*>(p.data());
                dhcp::Options o;
                EXPECT_TRUE(dhcp::parse_options({b + 240, p.size() - 240}, o));
                return std::pair{o.type, Ipv4Address::load(b + 16)};
            }
        }
        return std::nullopt;
    }

    static constexpr MacAddress a = MacAddress::from(0x02, 0, 0, 0, 0, 0xA1);
    static constexpr MacAddress b = MacAddress::from(0x02, 0, 0, 0, 0, 0xB1);
    static constexpr MacAddress c = MacAddress::from(0x02, 0, 0, 0, 0, 0xC1);
};

TEST_F(DhcpServerRules, DifferentClientsGetDifferentAddresses) {
    const auto oa = ask(dhcp::discover, a);
    const auto ob = ask(dhcp::discover, b);
    ASSERT_TRUE(oa && ob);
    EXPECT_EQ(oa->first, dhcp::offer);
    EXPECT_NE(oa->second, ob->second);
    EXPECT_FALSE(ask(dhcp::discover, c));  // pool of two exhausted: silent
    EXPECT_EQ(ask(dhcp::discover, a)->second, oa->second);  // same client, same offer
}

TEST_F(DhcpServerRules, RequestAcksOwnOfferAndNaksOthers) {
    const auto oa = ask(dhcp::discover, a);
    (void)ask(dhcp::discover, b);
    EXPECT_EQ(ask(dhcp::request, a, oa->second, device_ip)->first, dhcp::ack);
    EXPECT_EQ(server.address_of(a), oa->second);
    EXPECT_EQ(ask(dhcp::request, b, oa->second, device_ip)->first, dhcp::nak);  // a's address
    EXPECT_EQ(ask(dhcp::request, c, oa->second)->first, dhcp::nak);            // INIT-REBOOT, not c's
    EXPECT_EQ(ask(dhcp::request, a, oa->second)->first, dhcp::ack);            // INIT-REBOOT, a's own
}

TEST_F(DhcpServerRules, OtherServerChosenDropsTheOffer) {
    const auto oa = ask(dhcp::discover, a);
    EXPECT_FALSE(ask(dhcp::request, a, oa->second, Ipv4Address::from(192, 168, 77, 1)));
    EXPECT_EQ(ask(dhcp::discover, c)->second, oa->second);  // the address is free again
}

TEST_F(DhcpServerRules, DeclinedAddressIsQuarantined) {
    const auto oa = ask(dhcp::discover, a);
    (void)ask(dhcp::request, a, oa->second, device_ip);
    EXPECT_FALSE(ask(dhcp::decline, a, oa->second, device_ip));
    const auto ob = ask(dhcp::discover, b);
    ASSERT_TRUE(ob);
    EXPECT_NE(ob->second, oa->second);
    EXPECT_FALSE(ask(dhcp::discover, c));  // the declined one is not handed out
}

TEST_F(DhcpServerRules, ReleaseFreesTheAddress) {
    const auto oa = ask(dhcp::discover, a);
    (void)ask(dhcp::request, a, oa->second, device_ip);
    (void)ask(dhcp::discover, b);
    EXPECT_FALSE(ask(dhcp::release, a, {}, device_ip, oa->second));
    EXPECT_EQ(server.bound_count(), 0u);
    EXPECT_TRUE(ask(dhcp::discover, c));
}

}  // namespace
