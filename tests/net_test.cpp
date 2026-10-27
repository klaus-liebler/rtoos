// SPDX-License-Identifier: MIT
//
// Host unit tests for the IPv4 stack: checksum, ARP, ICMP, UDP, TCP.
#include <gtest/gtest.h>

#include <string>

#include "net_fixture.hpp"

namespace {

using namespace test;

// ---------------------------------------------------------------------------
// Checksum
// ---------------------------------------------------------------------------

TEST(Checksum, Rfc1071Example) {
    // RFC 1071 section 3: 00 01 f2 03 f4 f5 f6 f7 -> sum 0xddf2, checksum 0x220d
    const std::array<std::byte, 8> data{std::byte{0x00}, std::byte{0x01}, std::byte{0xf2}, std::byte{0x03},
                                        std::byte{0xf4}, std::byte{0xf5}, std::byte{0xf6}, std::byte{0xf7}};
    EXPECT_EQ(checksum(data), 0x220d);
}

TEST(Checksum, OddLengthPadsWithZero) {
    const std::array<std::byte, 3> data{std::byte{0x01}, std::byte{0x02}, std::byte{0x03}};
    EXPECT_EQ(checksum(data), static_cast<std::uint16_t>(~(0x0102 + 0x0300)));
}

// ---------------------------------------------------------------------------
// ARP / ICMP
// ---------------------------------------------------------------------------

TEST_F(NetTest, AnswersArpRequestInPlace) {
    std::array<std::byte, 28> a{};
    store_be16(a.data(), 1);
    store_be16(a.data() + 2, ethertype_ipv4);
    a[4] = std::byte{6};
    a[5] = std::byte{4};
    store_be16(a.data() + 6, 1);
    peer_mac.store(a.data() + 8);
    peer_ip.store(a.data() + 14);
    device_ip.store(a.data() + 24);
    PacketRef request = frame(ethertype_arp, a, MacAddress::broadcast());
    PacketBuffer* buffer = request.get();
    deliver(std::move(request));

    ASSERT_EQ(driver.tx.size(), 1u);
    EXPECT_EQ(driver.tx[0].get(), buffer);  // zero copy: the request buffer is the reply
    const auto f = sent();
    EXPECT_EQ(f[0].ethertype(), ethertype_arp);
    EXPECT_EQ(f[0].dst_mac(), peer_mac);
    const std::byte* r = f[0].bytes.data() + 14;
    EXPECT_EQ(load_be16(r + 6), 2);
    EXPECT_EQ(MacAddress::load(r + 8), device_mac);
    EXPECT_EQ(Ipv4Address::load(r + 14), device_ip);
}

TEST_F(NetTest, AnswersPingInPlace) {
    std::vector<std::byte> echo(8 + 4);
    echo[0] = std::byte{8};
    store_be16(echo.data() + 4, 0x1234);  // id
    store_be16(echo.data() + 6, 7);       // seq
    echo[8] = std::byte{'p'};
    store_be16(echo.data() + 2, checksum(echo));
    PacketRef request = ipv4(ip_proto_icmp, echo);
    PacketBuffer* buffer = request.get();
    deliver(std::move(request));

    ASSERT_EQ(driver.tx.size(), 1u);
    EXPECT_EQ(driver.tx[0].get(), buffer);
    const auto f = sent();
    EXPECT_EQ(f[0].ip_protocol(), ip_proto_icmp);
    EXPECT_EQ(f[0].ip_dst(), peer_ip);
    EXPECT_EQ(f[0].l4()[0], std::byte{0});                     // echo reply
    EXPECT_EQ(load_be16(f[0].l4() + 4), 0x1234);
    EXPECT_EQ(checksum({f[0].l4(), f[0].l4_size()}), 0);         // valid ICMP checksum
    EXPECT_EQ(checksum({f[0].ip(), 20}), 0);                     // valid IP checksum
}

TEST_F(NetTest, UnknownNextHopIsResolvedByArpThenSent) {
    const Ipv4Address other = Ipv4Address::from(192, 168, 77, 9);
    PacketRef p = stack.allocate();
    p->append(as_bytes("hi"));
    stack.udp_send(std::move(p), other, 9, 9);
    auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ethertype(), ethertype_arp);  // request first, datagram parked

    std::array<std::byte, 28> a{};
    store_be16(a.data(), 1);
    store_be16(a.data() + 2, ethertype_ipv4);
    a[4] = std::byte{6};
    a[5] = std::byte{4};
    store_be16(a.data() + 6, 2);
    MacAddress::from(2, 0, 0, 0, 0, 9).store(a.data() + 8);
    other.store(a.data() + 14);
    device_mac.store(a.data() + 18);
    device_ip.store(a.data() + 24);
    deliver(frame(ethertype_arp, a));
    f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ethertype(), ethertype_ipv4);
    EXPECT_EQ(f[0].udp_payload(), "hi");
}

// ---------------------------------------------------------------------------
// UDP
// ---------------------------------------------------------------------------

TEST_F(NetTest, UdpDispatchAndReply) {
    struct Echo {
        TestStack* stack;
        std::string got;
        static void received(void* self, const UdpDatagram& d) noexcept {
            auto& e = *static_cast<Echo*>(self);
            e.got.assign(reinterpret_cast<const char*>(d.payload.data()), d.payload.size());
            PacketRef reply = e.stack->allocate();
            reply->append(d.payload);
            e.stack->udp_send(std::move(reply), d.source, d.source_port, d.destination_port);
        }
    } echo{&stack, {}};
    const UdpHandler handler{&echo, &Echo::received};
    ASSERT_TRUE(stack.udp_bind(7, handler));

    deliver(udp(40000, 7, as_bytes("hello")));
    EXPECT_EQ(echo.got, "hello");
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(load_be16(f[0].l4()), 7);
    EXPECT_EQ(load_be16(f[0].l4() + 2), 40000);
    EXPECT_EQ(f[0].udp_payload(), "hello");
    EXPECT_EQ(checksum_finish(checksum_add({f[0].l4(), f[0].l4_size()},
                                           TestStack::pseudo_sum(device_ip, peer_ip, ip_proto_udp, f[0].l4_size()))),
              0);
}

TEST_F(NetTest, MulticastNeedsJoin) {
    int count = 0;
    const UdpHandler handler{&count, [](void* c, const UdpDatagram&) noexcept { ++*static_cast<int*>(c); }};
    stack.udp_bind(5353, handler);
    const Ipv4Address mdns = Ipv4Address::from(224, 0, 0, 251);
    deliver(udp(5353, 5353, as_bytes("x"), mdns));
    EXPECT_EQ(count, 0);
    stack.join(eth, mdns);
    deliver(udp(5353, 5353, as_bytes("x"), mdns));
    EXPECT_EQ(count, 1);
}

// ---------------------------------------------------------------------------
// TCP
// ---------------------------------------------------------------------------

struct EchoServer {
    std::string received;
    int accepted = 0;
    int closed = 0;
    rtos::net::TcpCloseReason reason{};
    bool echo = true;
    bool close_on_eof = true;

    TcpConnectionBase* connection = nullptr;

    static void on_accepted(void* self, TcpConnectionBase& c) noexcept {
        auto& s = *static_cast<EchoServer*>(self);
        ++s.accepted;
        s.connection = &c;
    }
    static void on_received(void* self, TcpConnectionBase& base) noexcept {
        auto& s = *static_cast<EchoServer*>(self);
        auto& c = static_cast<TestStack::TcpConnection&>(base);
        std::array<std::byte, 2048> buf{};
        const std::size_t n = c.read(buf);
        s.received.append(reinterpret_cast<const char*>(buf.data()), n);
        if (s.echo && n != 0) {
            c.write({buf.data(), n});
        }
        if (c.eof() && s.close_on_eof) {
            c.close();
        }
    }
    static void on_closed(void* self, TcpConnectionBase&, TcpCloseReason r) noexcept {
        auto& s = *static_cast<EchoServer*>(self);
        ++s.closed;
        s.reason = r;
    }
    TcpHandler handler() { return {this, &on_accepted, &on_received, nullptr, &on_closed}; }
};

class TcpTest : public NetTest {
protected:
    EchoServer server;
    TcpHandler handler = server.handler();
    static constexpr std::uint16_t port = 7;
    static constexpr std::uint16_t peer_port = 50000;
    std::uint32_t iss = 0;

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(stack.tcp().listen(port, handler));
    }

    /// Three-way handshake; afterwards peer_seq is the next peer sequence number.
    void connect() {
        deliver(tcp(peer_port, port, peer_seq, 0, tcp_flag::syn, {}, 8192, 1460));
        auto f = sent();
        ASSERT_EQ(f.size(), 1u);
        ASSERT_EQ(f[0].flags(), tcp_flag::syn | tcp_flag::ack);
        ASSERT_EQ(f[0].ack(), peer_seq + 1);
        iss = f[0].seq();
        ++peer_seq;
        deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::ack));
        ASSERT_EQ(server.accepted, 1);
    }
};

// ---------------------------------------------------------------------------
// Address claim (RFC 5227)
// ---------------------------------------------------------------------------

constexpr MacAddress other_mac = MacAddress::from(0x02, 0, 0, 0, 0, 0x99);
constexpr Ipv4Address claimed_ip = Ipv4Address::from(192, 168, 77, 5);

struct ArpFrame {
    std::uint16_t op;
    Ipv4Address sender, target;
};

class ClaimTest : public NetTest {
protected:
    void SetUp() override {
        NetTest::SetUp();
        ASSERT_TRUE(stack.subscribe(events));
    }

    std::vector<ArpFrame> arp_sent() {
        std::vector<ArpFrame> out;
        for (const Frame& f : sent()) {
            if (f.ethertype() == ethertype_arp) {
                const std::byte* a = f.bytes.data() + 14;
                out.push_back({load_be16(a + 6), Ipv4Address::load(a + 14), Ipv4Address::load(a + 24)});
            }
        }
        return out;
    }

    /// Poll in 100 ms steps for `ms`, collecting the ARP frames sent.
    std::vector<ArpFrame> run(std::uint32_t ms) {
        std::vector<ArpFrame> out;
        for (std::uint32_t t = 0; t < ms; t += 100) {
            poll(100);
            for (const auto& a : arp_sent()) {
                out.push_back(a);
            }
        }
        return out;
    }

    static void on_address(void* self, InterfaceId, const Ipv4Config& c) noexcept {
        static_cast<ClaimTest*>(self)->addresses.push_back(c.address);
    }
    static void on_conflict(void* self, InterfaceId, Ipv4Address a) noexcept {
        static_cast<ClaimTest*>(self)->conflicts.push_back(a);
    }
    NetEventHandler events{this, nullptr, &on_address, &on_conflict};
    std::vector<Ipv4Address> addresses, conflicts;
};

TEST_F(ClaimTest, ProbesThenAnnouncesAndUsesTheAddress) {
    stack.claim_address(eth, {claimed_ip, Ipv4Address::from(255, 255, 255, 0), {}});
    EXPECT_FALSE(stack.configured(eth));
    EXPECT_TRUE(stack.claiming(eth));
    const auto frames = run(12'000);
    ASSERT_EQ(frames.size(), 5u);
    for (int i = 0; i < 3; ++i) {  // probes: sender 0.0.0.0
        EXPECT_EQ(frames[i].op, 1);
        EXPECT_EQ(frames[i].sender, Ipv4Address::any());
        EXPECT_EQ(frames[i].target, claimed_ip);
    }
    for (int i = 3; i < 5; ++i) {  // announcements: sender = target = address
        EXPECT_EQ(frames[i].sender, claimed_ip);
        EXPECT_EQ(frames[i].target, claimed_ip);
    }
    EXPECT_EQ(stack.local_address(eth), claimed_ip);
    EXPECT_EQ(addresses.back(), claimed_ip);
    EXPECT_TRUE(conflicts.empty());
}

TEST_F(ClaimTest, ReplyDuringProbingIsAConflict) {
    stack.claim_address(eth, {claimed_ip, Ipv4Address::from(255, 255, 255, 0), {}});
    (void)run(1500);
    deliver(arp(2, other_mac, claimed_ip, Ipv4Address::any()));
    ASSERT_EQ(conflicts.size(), 1u);
    EXPECT_EQ(conflicts[0], claimed_ip);
    EXPECT_TRUE(run(12'000).empty());  // no more probes, no announcement
    EXPECT_FALSE(stack.configured(eth));
}

TEST_F(ClaimTest, SimultaneousProbeIsAConflict) {
    stack.claim_address(eth, {claimed_ip, Ipv4Address::from(255, 255, 255, 0), {}});
    (void)run(1500);
    deliver(arp(1, other_mac, Ipv4Address::any(), claimed_ip));
    EXPECT_EQ(conflicts.size(), 1u);
}

TEST_F(ClaimTest, DefendsOnceThenGivesTheAddressUp) {
    deliver(arp(1, other_mac, device_ip, peer_ip));  // someone else uses our address
    auto a = arp_sent();
    ASSERT_EQ(a.size(), 1u);                          // defence: announcement
    EXPECT_EQ(a[0].sender, device_ip);
    EXPECT_TRUE(stack.configured(eth));
    EXPECT_TRUE(conflicts.empty());
    poll(5000);
    deliver(arp(1, other_mac, device_ip, peer_ip));   // again within DEFEND_INTERVAL
    ASSERT_EQ(conflicts.size(), 1u);
    EXPECT_FALSE(stack.configured(eth));
}

TEST_F(ClaimTest, DefendsAgainAfterTheInterval) {
    deliver(arp(1, other_mac, device_ip, peer_ip));
    (void)sent();
    poll(TestStack::defend_interval_ms + 1);
    deliver(arp(1, other_mac, device_ip, peer_ip));
    EXPECT_EQ(arp_sent().size(), 1u);
    EXPECT_TRUE(conflicts.empty());
}

// ---------------------------------------------------------------------------
// Several interfaces
// ---------------------------------------------------------------------------

constexpr Ipv4Address usb_ip = Ipv4Address::from(192, 168, 7, 1);
constexpr Ipv4Address usb_peer = Ipv4Address::from(192, 168, 7, 2);

class TwoInterfaceTest : public NetTest {
protected:
    MockDriver usb;
    InterfaceId usb_if = [this] {
        usb.address = MacAddress::from(0x02, 0, 0, 0, 0, 0x07);
        return stack.attach(usb, {usb_ip, Ipv4Address::from(255, 255, 255, 0), {}});
    }();

    std::vector<Frame> usb_sent() {
        std::vector<Frame> out;
        for (auto& p : usb.tx) {
            out.push_back({std::vector<std::byte>(p->data(), p->data() + p->size())});
        }
        usb.complete_tx();
        return out;
    }

    void send_udp(Ipv4Address to) {
        PacketRef p = stack.allocate();
        p->append(as_bytes("x"));
        stack.udp_send(std::move(p), to, 9, 9);
    }
};

TEST_F(TwoInterfaceTest, RoutesByNetworkElseDefaultGateway) {
    EXPECT_EQ(stack.route(usb_peer), usb_if);
    EXPECT_EQ(stack.route(peer_ip), eth);
    EXPECT_EQ(stack.route(Ipv4Address::from(8, 8, 8, 8)), eth);  // only eth has a gateway
    send_udp(usb_peer);
    const auto u = usb_sent();
    ASSERT_EQ(u.size(), 1u);  // ARP request on the USB link
    EXPECT_EQ(u[0].ethertype(), ethertype_arp);
    EXPECT_TRUE(sent().empty());
}

TEST_F(TwoInterfaceTest, EachInterfaceAcceptsOnlyItsOwnAddress) {
    std::vector<std::byte> echo(8);
    echo[0] = std::byte{8};
    store_be16(echo.data() + 2, checksum(echo));
    usb.rx.push_back(ipv4(ip_proto_icmp, echo, usb_ip, usb_peer));
    usb.rx.push_back(ipv4(ip_proto_icmp, echo, device_ip, usb_peer));  // eth's address on the USB link
    poll();
    const auto u = usb_sent();
    ASSERT_EQ(u.size(), 1u);  // one echo reply, on the interface it came in (after ARP: parked)
    EXPECT_TRUE(sent().empty());
}

TEST_F(TwoInterfaceTest, UdpSocketsPerInterface) {
    struct Counter {
        int n = 0;
        static void received(void* self, const UdpDatagram&) noexcept { ++static_cast<Counter*>(self)->n; }
    } on_eth, on_usb;
    const UdpHandler h_eth{&on_eth, &Counter::received}, h_usb{&on_usb, &Counter::received};
    ASSERT_TRUE(stack.udp_bind(67, h_eth, InterfaceSet::of({eth})));
    ASSERT_TRUE(stack.udp_bind(67, h_usb, InterfaceSet::of({usb_if})));
    EXPECT_FALSE(stack.udp_bind(67, h_eth));  // all interfaces would overlap
    deliver(udp(68, 67, as_bytes("a"), Ipv4Address::broadcast()));
    usb.rx.push_back(udp(68, 67, as_bytes("b"), Ipv4Address::broadcast(), usb_peer));
    poll();
    EXPECT_EQ(on_eth.n, 1);
    EXPECT_EQ(on_usb.n, 1);
}

TEST_F(TwoInterfaceTest, TcpListenerOnChosenInterfacesOnly) {
    struct Counter {
        int accepted = 0;
        static void on(void* self, TcpConnectionBase&) noexcept { ++static_cast<Counter*>(self)->accepted; }
    } modbus;
    const TcpHandler h{&modbus, &Counter::on, nullptr, nullptr, nullptr};
    ASSERT_TRUE(stack.tcp().listen(502, h, InterfaceSet::of({eth})));
    EXPECT_FALSE(stack.tcp().listen(502, h));  // overlaps on eth
    usb.rx.push_back(tcp(5000, 502, 1, 0, tcp_flag::syn, {}, 8192, 1460, usb_ip, usb_peer));
    poll();
    EXPECT_EQ(modbus.accepted, 0);
    EXPECT_FALSE(usb_sent().empty());  // not served on USB: RST (after ARP for the peer)
    deliver(tcp(5000, 502, 1, 0, tcp_flag::syn, {}, 8192, 1460));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].flags(), tcp_flag::syn | tcp_flag::ack);  // eth: accepted
}

TEST_F(TwoInterfaceTest, SynOnInterfaceWithoutListenerIsReset) {
    const TcpHandler h{nullptr, nullptr, nullptr, nullptr, nullptr};
    ASSERT_TRUE(stack.tcp().listen(502, h, InterfaceSet::of({usb_if})));
    deliver(tcp(5000, 502, 1, 0, tcp_flag::syn, {}, 8192, 1460));  // on eth
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_TRUE(f[0].flags() & tcp_flag::rst);
}

TEST_F(TwoInterfaceTest, ListenersShareAPortOnDisjointInterfaces) {
    const TcpHandler a{nullptr, nullptr, nullptr, nullptr, nullptr}, b{nullptr, nullptr, nullptr, nullptr, nullptr};
    EXPECT_TRUE(stack.tcp().listen(80, a, InterfaceSet::of({eth})));
    EXPECT_TRUE(stack.tcp().listen(80, b, InterfaceSet::of({usb_if})));
    EXPECT_FALSE(stack.tcp().listen(80, a, InterfaceSet::of({eth, usb_if})));
}

TEST_F(TwoInterfaceTest, LinkChangesAreReported) {
    std::vector<std::pair<InterfaceId, bool>> events;
    const NetEventHandler h{&events,
                            [](void* c, InterfaceId i, bool up) noexcept {
                                static_cast<std::vector<std::pair<InterfaceId, bool>>*>(c)->emplace_back(i, up);
                            },
                            nullptr, nullptr};
    ASSERT_TRUE(stack.subscribe(h));  // both links are up already (first poll in SetUp)
    usb.link = false;
    poll();
    usb.link = true;
    poll();
    ASSERT_EQ(events.size(), 2u);
    EXPECT_EQ(events[0], std::make_pair(usb_if, false));
    EXPECT_EQ(events[1], std::make_pair(usb_if, true));
}

TEST_F(NetTest, SendersLeaveTheReceiveReserve) {
    std::vector<PacketRef> held;
    while (PacketRef p = stack.allocate()) {
        held.push_back(std::move(p));
    }
    EXPECT_EQ(pool.available(), TestStack::rx_reserve);
    EXPECT_GT(stack.stats().tx_no_buffer, 0u);

    // Receiving still works (the driver refills its ring from the reserve). The echo
    // reply reuses the request buffer, so it goes out even now.
    std::vector<std::byte> echo(8);
    echo[0] = std::byte{8};
    store_be16(echo.data() + 2, checksum(echo));
    deliver(ipv4(ip_proto_icmp, echo));
    EXPECT_EQ(sent().size(), 1u);

    held.clear();  // transmit buffers acknowledged / done
    EXPECT_TRUE(stack.allocate());
}

TEST_F(TcpTest, HandshakeAdvertisesMss) {
    deliver(tcp(peer_port, port, peer_seq, 0, tcp_flag::syn, {}, 8192, 1460));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    ASSERT_EQ(f[0].tcp_header(), 24u);
    EXPECT_EQ(f[0].l4()[20], std::byte{2});
    EXPECT_EQ(load_be16(f[0].l4() + 22), 1460);
    EXPECT_EQ(server.accepted, 0);  // not before the final ACK
}

TEST_F(TcpTest, EchoesDataAndClosesGracefully) {
    connect();
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::ack | tcp_flag::psh, "ping"));
    peer_seq += 4;
    EXPECT_EQ(server.received, "ping");
    auto f = sent();
    ASSERT_EQ(f.size(), 1u);  // echo with the ACK piggybacked
    EXPECT_EQ(f[0].tcp_payload(), "ping");
    EXPECT_EQ(f[0].ack(), peer_seq);
    EXPECT_EQ(f[0].seq(), iss + 1);

    // peer acknowledges and closes
    deliver(tcp(peer_port, port, peer_seq, iss + 5, tcp_flag::ack | tcp_flag::fin));
    ++peer_seq;
    poll();  // service(): our FIN
    f = sent();
    ASSERT_FALSE(f.empty());
    EXPECT_TRUE(f.back().flags() & tcp_flag::fin);
    EXPECT_EQ(f.back().ack(), peer_seq);
    const std::uint32_t fin_seq = f.back().seq();

    deliver(tcp(peer_port, port, peer_seq, fin_seq + 1, tcp_flag::ack));
    EXPECT_EQ(server.closed, 1);
    EXPECT_EQ(server.reason, TcpCloseReason::Normal);
    EXPECT_EQ(stack.tcp().active_connections(), 0u);
}

TEST_F(TcpTest, FinWait2TimesOutWhenPeerNeverCloses) {
    connect();
    server.connection->close();  // our side closes first
    poll();
    auto f = sent();
    ASSERT_FALSE(f.empty());
    ASSERT_TRUE(f.back().flags() & tcp_flag::fin);
    deliver(tcp(peer_port, port, peer_seq, f.back().seq() + 1, tcp_flag::ack));  // FIN acked, no FIN back
    EXPECT_EQ(server.closed, 0);
    poll(TestStack::TcpLayer::fin_wait2_ms - 1);
    EXPECT_EQ(server.closed, 0);
    poll(1);
    EXPECT_EQ(server.closed, 1);
    EXPECT_EQ(server.reason, TcpCloseReason::Timeout);
    EXPECT_TRUE(sent().back().flags() & tcp_flag::rst);
    EXPECT_EQ(stack.tcp().active_connections(), 0u);
}

TEST_F(TcpTest, RetransmitsUnackedSegmentWithBackoff) {
    connect();  // handshake RTT sample ~0 ms -> RTO = minimum (200 ms, RFC 6298 with Linux-like floor)
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::ack | tcp_flag::psh, "data"));
    peer_seq += 4;
    auto first = sent();
    ASSERT_EQ(first.size(), 1u);
    EXPECT_EQ(first[0].tcp_payload(), "data");

    poll(199);
    EXPECT_TRUE(sent().empty());
    poll(1);  // RTO expired
    auto again = sent();
    ASSERT_EQ(again.size(), 1u);
    EXPECT_EQ(again[0].seq(), first[0].seq());
    EXPECT_EQ(again[0].tcp_payload(), "data");

    poll(399);
    EXPECT_TRUE(sent().empty());
    poll(1);  // backed off: doubled
    EXPECT_EQ(sent().size(), 1u);

    deliver(tcp(peer_port, port, peer_seq, iss + 5, tcp_flag::ack));
    poll(10'000);
    EXPECT_TRUE(sent().empty());  // acknowledged: no more retransmissions
}

TEST_F(TcpTest, InitialRtoIsOneSecondWithoutSample) {
    deliver(tcp(peer_port, port, peer_seq, 0, tcp_flag::syn, {}, 8192, 1460));
    ASSERT_EQ(sent().size(), 1u);  // SYN-ACK
    poll(999);
    EXPECT_TRUE(sent().empty());
    poll(1);
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].flags(), tcp_flag::syn | tcp_flag::ack);  // SYN-ACK retransmitted
}

TEST_F(TcpTest, RetransmissionWaitsForTxDma) {
    connect();
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::ack | tcp_flag::psh, "data"));
    ASSERT_EQ(driver.tx.size(), 1u);  // the "DMA" still holds the segment
    poll(200);
    EXPECT_EQ(driver.tx.size(), 1u);  // not re-sent while the buffer is in use
    driver.complete_tx();
    poll(10);
    EXPECT_EQ(driver.tx.size(), 1u);  // now it goes out
}

TEST_F(TcpTest, OutOfOrderSegmentIsDroppedWithDuplicateAck) {
    connect();
    server.echo = false;
    deliver(tcp(peer_port, port, peer_seq + 10, iss + 1, tcp_flag::ack, "late"));
    EXPECT_EQ(server.received, "");
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].ack(), peer_seq);  // still expecting the gap
}

TEST_F(TcpTest, ResetOnClosedPort) {
    deliver(tcp(peer_port, 9999, peer_seq, 0, tcp_flag::syn));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].flags(), tcp_flag::rst | tcp_flag::ack);
    EXPECT_EQ(f[0].ack(), peer_seq + 1);
}

TEST_F(TcpTest, PeerResetClosesConnection) {
    connect();
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::rst));
    EXPECT_EQ(server.closed, 1);
    EXPECT_EQ(server.reason, TcpCloseReason::Reset);
}

TEST_F(TcpTest, SendIsLimitedByPeerWindow) {
    connect();
    server.echo = false;
    // peer window 100 bytes
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::ack, {}, 100));
    auto& c = static_cast<TestStack::TcpConnection&>(*server.connection);
    EXPECT_EQ(c.writable(), 100u);
    const std::string big(300, 'a');
    EXPECT_EQ(c.write(as_bytes(big)), 100u);
    EXPECT_EQ(c.writable(), 0u);
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_EQ(f[0].tcp_payload().size(), 100u);
}

TEST_F(TcpTest, ConnectionTableFullAnswersReset) {
    for (std::uint16_t i = 0; i < 4; ++i) {
        deliver(tcp(static_cast<std::uint16_t>(peer_port + i), port, peer_seq, 0, tcp_flag::syn));
    }
    (void)sent();
    deliver(tcp(peer_port + 10, port, peer_seq, 0, tcp_flag::syn));
    const auto f = sent();
    ASSERT_EQ(f.size(), 1u);
    EXPECT_TRUE(f[0].flags() & tcp_flag::rst);
}

TEST_F(TcpTest, BuffersReturnToPoolAfterClose) {
    const std::size_t before = pool.available();
    connect();
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::ack | tcp_flag::psh, "x"));
    ++peer_seq;
    deliver(tcp(peer_port, port, peer_seq, iss + 1, tcp_flag::rst));
    (void)sent();
    poll(5000);
    (void)sent();
    EXPECT_EQ(pool.available(), before);
}

}  // namespace
