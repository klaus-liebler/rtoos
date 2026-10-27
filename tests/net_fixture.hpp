// SPDX-License-Identifier: MIT
//
// Host test fixture for the network stack: a mock driver and a "peer" that builds
// and parses frames, so tests can talk to the stack like a remote host.
#pragma once

#include <gtest/gtest.h>

#include <cstdint>
#include <cstring>
#include <deque>
#include <optional>
#include <string_view>
#include <vector>

#include "rtos/net/stack.hpp"

namespace test {

using namespace rtos::net;

inline constexpr MacAddress device_mac = MacAddress::from(0x02, 0x00, 0x00, 0x00, 0x00, 0x02);
inline constexpr MacAddress peer_mac = MacAddress::from(0x02, 0x00, 0x00, 0x00, 0x00, 0x01);
inline constexpr Ipv4Address device_ip = Ipv4Address::from(192, 168, 77, 2);
inline constexpr Ipv4Address peer_ip = Ipv4Address::from(192, 168, 77, 1);

/// Packet pool that checks on destruction - after the stack, drivers and servers
/// declared behind it are gone - that every buffer came back (NetX Duo checks the
/// same after each test): a reference-count leak fails the test that caused it.
struct TestPool : PacketPool<32> {
    ~TestPool() { EXPECT_EQ(available(), capacity()) << "packet buffers leaked"; }
};

struct MockDriver {
    std::deque<PacketRef> rx;          // frames the stack will receive
    std::vector<PacketRef> tx;         // frames the stack sent (driver keeps a reference = "DMA busy")
    bool link = true;
    MacAddress address = device_mac;

    MacAddress mac() const noexcept { return address; }
    bool link_up() const noexcept { return link; }
    bool transmit(PacketRef frame) noexcept {
        tx.push_back(frame);
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
    /// "TX DMA done": drop the driver's references.
    void complete_tx() { tx.clear(); }
};
static_assert(NetDriver<MockDriver>);

/// Parsed view of a frame the stack sent.
struct Frame {
    std::vector<std::byte> bytes;
    MacAddress dst_mac() const { return MacAddress::load(bytes.data()); }
    std::uint16_t ethertype() const { return load_be16(bytes.data() + 12); }
    const std::byte* ip() const { return bytes.data() + 14; }
    std::uint8_t ip_protocol() const { return std::to_integer<std::uint8_t>(ip()[9]); }
    Ipv4Address ip_src() const { return Ipv4Address::load(ip() + 12); }
    Ipv4Address ip_dst() const { return Ipv4Address::load(ip() + 16); }
    std::uint16_t udp_src_port() const { return load_be16(l4()); }
    std::uint16_t udp_dst_port() const { return load_be16(l4() + 2); }
    const std::byte* l4() const { return ip() + (std::to_integer<std::size_t>(ip()[0]) & 0x0F) * 4; }
    std::size_t l4_size() const { return load_be16(ip() + 2) - (l4() - ip()); }
    // TCP
    std::uint32_t seq() const { return load_be32(l4() + 4); }
    std::uint32_t ack() const { return load_be32(l4() + 8); }
    std::uint8_t flags() const { return std::to_integer<std::uint8_t>(l4()[13]); }
    std::uint16_t window() const { return load_be16(l4() + 14); }
    std::size_t tcp_header() const { return (std::to_integer<std::size_t>(l4()[12]) >> 4) * 4; }
    std::string_view tcp_payload() const {
        return {reinterpret_cast<const char*>(l4() + tcp_header()), l4_size() - tcp_header()};
    }
    std::string_view udp_payload() const {
        return {reinterpret_cast<const char*>(l4() + 8), load_be16(l4() + 4) - 8u};
    }
};

using TestStack = Stack<TestPool>;

class NetTest : public ::testing::Test {
protected:
    TestPool pool;
    MockDriver driver;
    TestStack stack{pool};
    InterfaceId eth = stack.attach(
        driver, Ipv4Config{device_ip, Ipv4Address::from(255, 255, 255, 0), Ipv4Address::from(192, 168, 77, 254)});
    std::uint32_t now = 1000;
    std::uint32_t peer_seq = 5000;

    void SetUp() override {
        stack.set_random([](void*) noexcept -> std::uint32_t { return 100000; }, nullptr);
        prime_arp();
    }

    void poll(std::uint32_t advance_ms = 0) {
        now += advance_ms;
        stack.poll(now);
    }

    /// Frames sent since the last call (copied, so the stack's buffers stay intact).
    std::vector<Frame> sent() {
        std::vector<Frame> out;
        for (auto& p : driver.tx) {
            out.push_back({std::vector<std::byte>(p->data(), p->data() + p->size())});
        }
        driver.complete_tx();
        return out;
    }

    // ---- frame builders ---------------------------------------------------------

    PacketRef frame(std::uint16_t ethertype, std::span<const std::byte> payload, MacAddress dst = device_mac) {
        PacketRef p = pool.allocate(0);
        std::byte* h = p->append(14);
        dst.store(h);
        peer_mac.store(h + 6);
        store_be16(h + 12, ethertype);
        p->append(payload);
        return p;
    }

    PacketRef ipv4(std::uint8_t protocol, std::span<const std::byte> l4, Ipv4Address dst = device_ip,
                   Ipv4Address src = peer_ip) {
        std::vector<std::byte> ip(20 + l4.size());
        ip[0] = std::byte{0x45};
        store_be16(ip.data() + 2, static_cast<std::uint16_t>(ip.size()));
        ip[8] = std::byte{64};
        ip[9] = std::byte{protocol};
        src.store(ip.data() + 12);
        dst.store(ip.data() + 16);
        store_be16(ip.data() + 10, checksum({ip.data(), 20}));
        std::memcpy(ip.data() + 20, l4.data(), l4.size());
        const MacAddress mac = dst.is_multicast() ? dst.multicast_mac() : device_mac;
        return frame(ethertype_ipv4, ip, mac);
    }

    PacketRef udp(std::uint16_t src_port, std::uint16_t dst_port, std::span<const std::byte> payload,
                  Ipv4Address dst = device_ip, Ipv4Address src = peer_ip) {
        std::vector<std::byte> u(8 + payload.size());
        store_be16(u.data(), src_port);
        store_be16(u.data() + 2, dst_port);
        store_be16(u.data() + 4, static_cast<std::uint16_t>(u.size()));
        std::memcpy(u.data() + 8, payload.data(), payload.size());
        store_be16(u.data() + 6, checksum_finish(checksum_add(u, TestStack::pseudo_sum(src, dst, ip_proto_udp, u.size()))));
        return ipv4(ip_proto_udp, u, dst, src);
    }

    PacketRef tcp(std::uint16_t src_port, std::uint16_t dst_port, std::uint32_t seq, std::uint32_t ack,
                  std::uint8_t flags, std::string_view payload = {}, std::uint16_t window = 8192,
                  std::optional<std::uint16_t> mss = std::nullopt, Ipv4Address dst = device_ip,
                  Ipv4Address src = peer_ip) {
        const std::size_t hlen = mss ? 24 : 20;
        std::vector<std::byte> t(hlen + payload.size());
        store_be16(t.data(), src_port);
        store_be16(t.data() + 2, dst_port);
        store_be32(t.data() + 4, seq);
        store_be32(t.data() + 8, ack);
        t[12] = static_cast<std::byte>((hlen / 4) << 4);
        t[13] = static_cast<std::byte>(flags);
        store_be16(t.data() + 14, window);
        if (mss) {
            t[20] = std::byte{2};
            t[21] = std::byte{4};
            store_be16(t.data() + 22, *mss);
        }
        std::memcpy(t.data() + hlen, payload.data(), payload.size());
        store_be16(t.data() + 16,
                   checksum_finish(checksum_add(t, TestStack::pseudo_sum(src, dst, ip_proto_tcp, t.size()))));
        return ipv4(ip_proto_tcp, t, dst, src);
    }

    void deliver(PacketRef f) {
        driver.rx.push_back(std::move(f));
        poll();
    }

    /// ARP packet (op 1 request, 2 reply) as received from `sender_mac`.
    PacketRef arp(std::uint16_t op, MacAddress sender_mac, Ipv4Address sender_ip, Ipv4Address target_ip,
                  MacAddress target_mac = {}) {
        std::array<std::byte, 28> a{};
        store_be16(a.data(), 1);
        store_be16(a.data() + 2, ethertype_ipv4);
        a[4] = std::byte{6};
        a[5] = std::byte{4};
        store_be16(a.data() + 6, op);
        sender_mac.store(a.data() + 8);
        sender_ip.store(a.data() + 14);
        target_mac.store(a.data() + 18);
        target_ip.store(a.data() + 24);
        PacketRef p = frame(ethertype_arp, a, MacAddress::broadcast());
        // frame() writes the peer as Ethernet source; use the ARP sender
        sender_mac.store(p->data() + 6);
        return p;
    }

    /// Teach the stack the peer's MAC so outgoing frames are not parked for ARP.
    void prime_arp() {
        deliver(arp(2, peer_mac, peer_ip, device_ip, device_mac));
        (void)sent();
    }
};

inline std::span<const std::byte> as_bytes(std::string_view s) {
    return {reinterpret_cast<const std::byte*>(s.data()), s.size()};
}

}  // namespace test
