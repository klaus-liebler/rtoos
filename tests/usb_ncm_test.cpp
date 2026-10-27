// SPDX-License-Identifier: MIT
//
// CDC-NCM over the simulated host: NTB parameters (divisor 4 for Windows), link and
// notifications (alternate setting 1: speed change, then network connection; USBX's
// cdc_ecm alternate-setting tests), NTB reception (several datagrams, chained NDPs,
// flow control) and a table of malformed NTBs (TinyUSB's NCM fuzzer targets and the
// checks of the FCU firmware's own NCM class), NTB transmission (layout, alignment,
// ZLP rule, the host's NTB input size), buffers on reset - and the whole path into
// rtos::net::Stack: ARP, ping and a DHCP lease over USB.
#include <gtest/gtest.h>

#include <string_view>

#include "net_fixture.hpp"
#include "rtos/net/dhcp_server.hpp"
#include "rtos/usb/cdc_ncm.hpp"
#include "rtos/usb/descriptors.hpp"
#include "usb_fixture.hpp"

namespace {

using namespace usbtest;
using rtos::net::MacAddress;
using rtos::net::PacketRef;

constexpr CdcNcmInfo ncm_info{.interface = 0, .notify_ep = 0x81, .out_ep = 0x02, .in_ep = 0x82, .string = 4, .mac_string = 5};
constexpr auto device_desc = device_descriptor({});
constexpr auto config_desc = configuration_descriptor({}, cdc_ncm_function(ncm_info));
static_assert(config_desc.size() == 9 + 85);
static_assert(config_desc[4] == 2);  // alternate settings count once
constexpr std::string_view strings[] = {"rtoos", "FCU", "0001", "FCU network", "020000000201"};

constexpr MacAddress device_mac = MacAddress::from(0x02, 0, 0, 0, 0x01, 0x01);
constexpr MacAddress host_mac = MacAddress::from(0x02, 0, 0, 0, 0x02, 0x01);

using Ncm = CdcNcm<test::TestPool, 4>;
static_assert(rtos::net::NetDriver<Ncm>);
using Dev = Device<MockController>;

/// NTB16 builder: datagrams in one NDP (or chained NDPs), 4-byte aligned.
struct NtbBuilder {
    std::vector<Bytes> datagrams;
    bool chain = false;  ///< one NDP per datagram, linked by wNextNdpIndex

    Bytes build() const {
        Bytes ntb(12);
        auto put16 = [&](std::size_t at, std::uint16_t v) {
            if (ntb.size() < at + 2) {
                ntb.resize(at + 2);
            }
            ntb[at] = static_cast<std::byte>(v);
            ntb[at + 1] = static_cast<std::byte>(v >> 8);
        };
        auto align = [&] {
            while (ntb.size() % 4 != 0) {
                ntb.push_back(std::byte{0});
            }
        };
        put16(0, 0x434E);
        put16(2, 0x484D);
        put16(4, 12);
        std::vector<std::size_t> offsets;
        for (const Bytes& d : datagrams) {
            align();
            offsets.push_back(ntb.size());
            ntb.insert(ntb.end(), d.begin(), d.end());
        }
        align();
        const std::size_t groups = chain ? datagrams.size() : 1;
        std::size_t previous = 10;  // NTH wNdpIndex
        for (std::size_t g = 0; g < groups; ++g) {
            const std::size_t ndp = ntb.size();
            put16(previous, static_cast<std::uint16_t>(ndp));
            const std::size_t count = chain ? 1 : datagrams.size();
            const std::size_t length = 8 + 4 * (count + 1);
            ntb.resize(ndp + length);
            put16(ndp, 0x434E);
            put16(ndp + 2, 0x304D);
            put16(ndp + 4, static_cast<std::uint16_t>(length));
            for (std::size_t i = 0; i < count; ++i) {
                const std::size_t d = chain ? g : i;
                put16(ndp + 8 + 4 * i, static_cast<std::uint16_t>(offsets[d]));
                put16(ndp + 10 + 4 * i, static_cast<std::uint16_t>(datagrams[d].size()));
            }
            previous = ndp + 6;
        }
        put16(8, static_cast<std::uint16_t>(ntb.size()));
        return ntb;
    }
};

/// Parsed NTB sent by the device.
std::vector<Bytes> datagrams_of(const Bytes& ntb, std::vector<std::size_t>* offsets = nullptr) {
    auto u16 = [&](std::size_t at) {
        return static_cast<std::size_t>(std::to_integer<int>(ntb[at]) | (std::to_integer<int>(ntb[at + 1]) << 8));
    };
    std::vector<Bytes> out;
    EXPECT_EQ(u16(0), 0x434Eu);
    EXPECT_EQ(u16(8), ntb.size());
    const std::size_t ndp = u16(10);
    EXPECT_EQ(u16(ndp), 0x434Eu);
    EXPECT_EQ(u16(ndp + 2), 0x304Du);
    for (std::size_t e = ndp + 8; e + 4 <= ndp + u16(ndp + 4); e += 4) {
        const std::size_t index = u16(e), length = u16(e + 2);
        if (index == 0 || length == 0) {
            break;
        }
        if (offsets != nullptr) {
            offsets->push_back(index);
        }
        out.emplace_back(ntb.begin() + static_cast<std::ptrdiff_t>(index),
                         ntb.begin() + static_cast<std::ptrdiff_t>(index + length));
    }
    return out;
}

Bytes frame(std::uint8_t tag, std::size_t size = 60) {
    Bytes f(size, std::byte{tag});
    device_mac.store(f.data());
    host_mac.store(f.data() + 6);
    return f;
}

class UsbNcmTest : public ::testing::Test {
protected:
    test::TestPool pool;
    MockController hw;
    Ncm ncm{ncm_info, pool, device_mac};
    std::array<Function*, 1> functions{&ncm};
    Dev dev{hw, {device_desc, config_desc, strings}, functions};
    Host<Dev> host{hw, dev};

    void SetUp() override { host.enumerate(); }

    void link_up() {
        ASSERT_FALSE(host.control(0x01, request::set_interface, 1, 1, 0).stalled);
        host.in(0x81);  // speed change
        host.in(0x81);  // network connection
    }

    std::vector<Bytes> drain() {
        std::vector<Bytes> out;
        while (PacketRef p = ncm.receive()) {
            out.emplace_back(p->data(), p->data() + p->size());
        }
        host.run();  // service(): re-arm OUT
        return out;
    }
};

TEST_F(UsbNcmTest, NtbParametersUseDivisorFourForWindows) {
    const auto r = host.control(0xA1, ncm::get_ntb_parameters, 0, 0, 28);
    ASSERT_EQ(r.data.size(), 28u);
    EXPECT_EQ(r.data[2], std::byte{1});  // NTB16 only
    EXPECT_EQ(r.data[8], std::byte{4});  // wNdpInDivisor
    EXPECT_EQ(r.data[12], std::byte{4});  // wNdpInAlignment
    EXPECT_EQ(r.data[20], std::byte{4});  // wNdpOutDivisor
    EXPECT_EQ(host.control(0xA1, ncm::get_ntb_input_size, 0, 0, 4).data, bytes({0x00, 0x08, 0, 0}));
    EXPECT_EQ(host.control(0xA1, ncm::get_ntb_format, 0, 0, 2).data, bytes({0, 0}));
    EXPECT_TRUE(host.control(0x21, ncm::set_ntb_format, 1, 0, 0).stalled);  // NTB32
    EXPECT_FALSE(host.control(0x21, ncm::set_ethernet_packet_filter, 0x0C, 0, 0).stalled);
    EXPECT_TRUE(host.control(0xA1, 0x87, 0, 0, 2).stalled);  // GET_MAX_DATAGRAM_SIZE: not advertised
}

TEST_F(UsbNcmTest, AlternateSettingOneBringsTheLinkUpWithNotifications) {
    EXPECT_FALSE(ncm.link_up());
    EXPECT_FALSE(hw.at(0x02).active);
    ASSERT_FALSE(host.control(0x01, request::set_interface, 1, 1, 0).stalled);
    EXPECT_TRUE(ncm.link_up());
    EXPECT_TRUE(hw.at(0x02).active);  // reception armed
    const auto speed = host.in(0x81);
    ASSERT_TRUE(speed.has_value());
    EXPECT_EQ(*speed, bytes({0xA1, 0x2A, 0, 0, 0, 0, 8, 0, 0x00, 0x1B, 0xB7, 0x00, 0x00, 0x1B, 0xB7, 0x00}));
    EXPECT_EQ(*host.in(0x81), bytes({0xA1, 0x00, 1, 0, 0, 0, 0, 0}));  // connected
    EXPECT_EQ(host.control(0x81, request::get_interface, 0, 1, 1).data, bytes({1}));

    ASSERT_FALSE(host.control(0x01, request::set_interface, 0, 1, 0).stalled);
    EXPECT_FALSE(ncm.link_up());
    EXPECT_EQ(*host.in(0x81), bytes({0xA1, 0x00, 0, 0, 0, 0, 0, 0}));  // disconnected
    EXPECT_TRUE(host.control(0x01, request::set_interface, 2, 1, 0).stalled);
}

TEST_F(UsbNcmTest, ReceivesSeveralDatagramsAndRearmsWhenDone) {
    link_up();
    ASSERT_TRUE(host.out(0x02, NtbBuilder{{frame(1), frame(2, 61), frame(3, 1514)}}.build()));
    EXPECT_FALSE(hw.at(0x02).active) << "the NTB is still being taken apart";
    const auto got = drain();
    ASSERT_EQ(got.size(), 3u);
    EXPECT_EQ(got[0], frame(1));
    EXPECT_EQ(got[1], frame(2, 61));
    EXPECT_EQ(got[2].size(), 1514u);
    EXPECT_TRUE(hw.at(0x02).active);
}

TEST_F(UsbNcmTest, ChainedNdpsAreFollowed) {
    link_up();
    ASSERT_TRUE(host.out(0x02, NtbBuilder{{frame(1), frame(2), frame(3)}, true}.build()));
    EXPECT_EQ(drain().size(), 3u);
}

TEST_F(UsbNcmTest, MalformedNtbsAreRejectedAndTheNextOneWorks) {
    link_up();
    const Bytes good = NtbBuilder{{frame(7)}}.build();
    const std::size_t ndp = std::to_integer<std::size_t>(good[10]) | (std::to_integer<std::size_t>(good[11]) << 8);
    struct Case {
        const char* name;
        std::function<void(Bytes&)> mutate;
        std::size_t frames;  ///< still delivered
    };
    const auto set16 = [](Bytes& b, std::size_t at, std::uint16_t v) {
        b[at] = static_cast<std::byte>(v);
        b[at + 1] = static_cast<std::byte>(v >> 8);
    };
    const Case cases[] = {
        {"short", [](Bytes& b) { b.resize(10); }, 0},
        {"NTH signature", [](Bytes& b) { b[0] = std::byte{'X'}; }, 0},
        {"NTH header length", [&](Bytes& b) { set16(b, 4, 16); }, 0},
        {"block longer than the transfer", [&](Bytes& b) { set16(b, 8, static_cast<std::uint16_t>(b.size() + 4)); }, 0},
        {"NDP index 0", [&](Bytes& b) { set16(b, 10, 0); }, 0},
        {"NDP index unaligned", [&](Bytes& b) { set16(b, 10, static_cast<std::uint16_t>(ndp + 2)); }, 0},
        {"NDP index beyond the block", [&](Bytes& b) { set16(b, 10, 0xFFF0); }, 0},
        {"NDP with CRC (NCM1)", [&](Bytes& b) { b[ndp + 3] = std::byte{'1'}; }, 0},
        {"NDP length too small", [&](Bytes& b) { set16(b, ndp + 4, 12); }, 0},
        {"NDP length beyond the block", [&](Bytes& b) { set16(b, ndp + 4, 0x400); }, 0},
        {"datagram beyond the block", [&](Bytes& b) { set16(b, ndp + 10, 0x800); }, 0},
        {"datagram index in the header", [&](Bytes& b) { set16(b, ndp + 8, 4); }, 0},
        {"datagram shorter than Ethernet", [&](Bytes& b) { set16(b, ndp + 10, 13); }, 0},
        {"NDP loop", [&](Bytes& b) { set16(b, ndp + 6, static_cast<std::uint16_t>(ndp)); }, 1},
    };
    for (const Case& c : cases) {
        SCOPED_TRACE(c.name);
        Bytes ntb = good;
        c.mutate(ntb);
        ASSERT_TRUE(host.out(0x02, ntb));
        EXPECT_LE(drain().size(), c.frames);
        ASSERT_TRUE(host.out(0x02, good));
        EXPECT_EQ(drain(), std::vector<Bytes>{frame(7)});
    }
    EXPECT_GT(ncm.stats().rx_bad_ntbs, 10u);
}

TEST_F(UsbNcmTest, NoBufferKeepsTheDatagramForLater) {
    link_up();
    std::vector<PacketRef> hog;
    while (PacketRef p = pool.allocate(0)) {
        hog.push_back(std::move(p));
    }
    ASSERT_TRUE(host.out(0x02, NtbBuilder{{frame(1), frame(2)}}.build()));
    EXPECT_FALSE(ncm.receive());
    hog.clear();
    EXPECT_EQ(drain().size(), 2u);
}

TEST_F(UsbNcmTest, TransmitsQueuedFramesAsOneAlignedNtb) {
    link_up();
    EXPECT_TRUE(ncm.transmit(PacketRef{}) == false);
    for (std::uint8_t tag : {1, 2, 3}) {
        PacketRef p = pool.allocate(0);
        const Bytes f = frame(tag, 61 + tag);
        p->append(f);
        ASSERT_TRUE(ncm.transmit(std::move(p)));
    }
    host.run();
    const auto ntb = host.in(0x82);
    ASSERT_TRUE(ntb.has_value());
    std::vector<std::size_t> offsets;
    const auto got = datagrams_of(*ntb, &offsets);
    ASSERT_EQ(got.size(), 3u);
    EXPECT_EQ(got[1], frame(2, 63));
    for (std::size_t o : offsets) {
        EXPECT_EQ(o % 4, 0u) << "wNdpInDivisor 4";
    }
    EXPECT_EQ(pool.available(), pool.capacity());  // copied into the NTB, buffers back
    EXPECT_EQ(ncm.stats().tx_ntbs, 1u);
}

TEST_F(UsbNcmTest, NtbsRespectTheHostsInputSize) {
    link_up();
    EXPECT_FALSE(host.control(0x21, ncm::set_ntb_input_size, 0, 0, 4, bytes({0x00, 0x06, 0, 0})).stalled);  // 1536
    EXPECT_EQ(ncm.ntb_input_size(), 1578u);  // at least one maximal datagram + headers
    for (std::uint8_t tag : {1, 2}) {
        PacketRef p = pool.allocate(0);
        p->append(frame(tag, 1000));
        ncm.transmit(std::move(p));
    }
    host.run();
    const auto first = host.in(0x82);
    ASSERT_TRUE(first.has_value());
    EXPECT_LE(first->size(), 1578u);
    EXPECT_EQ(datagrams_of(*first).size(), 1u);
    const auto second = host.in(0x82);
    ASSERT_TRUE(second.has_value());
    EXPECT_EQ(datagrams_of(*second).size(), 1u);
}

TEST_F(UsbNcmTest, ZlpRuleForNtbsOnAPacketBoundary) {
    link_up();
    PacketRef p = pool.allocate(0);
    // NTB = 12 + 16 (NDP for one datagram) + 100 = 128: a multiple of 64
    p->append(frame(1, 100));
    ncm.transmit(std::move(p));
    host.run();
    EXPECT_TRUE(host.in_zlp(0x82));
    EXPECT_EQ(host.in(0x82)->size(), 128u);
}

TEST_F(UsbNcmTest, LinkDownDropsAndResetReleasesQueuedFrames) {
    PacketRef p = pool.allocate(0);
    p->append(frame(1));
    EXPECT_FALSE(ncm.transmit(std::move(p)));  // link down
    link_up();
    for (int i = 0; i < 4; ++i) {
        PacketRef q = pool.allocate(0);
        q->append(frame(2));
        EXPECT_TRUE(ncm.transmit(std::move(q)));
    }
    PacketRef full = pool.allocate(0);
    full->append(frame(3));
    EXPECT_FALSE(ncm.transmit(std::move(full)));  // queue of 4 full
    host.reset();
    EXPECT_FALSE(ncm.link_up());
    EXPECT_EQ(pool.available(), pool.capacity());
}

// ---------------------------------------------------------------------------
// NCM as an interface of the IP stack
// ---------------------------------------------------------------------------

constexpr auto device_ip = rtos::net::Ipv4Address::from(192, 168, 7, 1);
constexpr auto host_ip = rtos::net::Ipv4Address::from(192, 168, 7, 10);

class UsbNcmStackTest : public UsbNcmTest {
protected:
    rtos::net::Stack<test::TestPool> stack{pool};
    rtos::net::InterfaceId usb_if =
        stack.attach(ncm, {device_ip, rtos::net::Ipv4Address::from(255, 255, 255, 0), rtos::net::Ipv4Address::any()});
    std::uint32_t now = 1000;

    /// One pass of the network thread: stack, then the USB device.
    void pass() {
        stack.poll(++now);
        host.run();
    }

    Bytes ethernet(std::uint16_t type, const Bytes& payload, MacAddress dst = device_mac) {
        Bytes f(14);
        dst.store(f.data());
        host_mac.store(f.data() + 6);
        rtos::net::store_be16(f.data() + 12, type);
        f.insert(f.end(), payload.begin(), payload.end());
        return f;
    }

    std::vector<Bytes> sent_frames() {
        std::vector<Bytes> out;
        while (const auto ntb = host.in(0x82)) {
            for (Bytes& d : datagrams_of(*ntb)) {
                out.push_back(std::move(d));
            }
        }
        return out;
    }
};

TEST_F(UsbNcmStackTest, LinkStateReachesTheStack) {
    pass();
    EXPECT_FALSE(stack.link_up(usb_if));
    link_up();
    pass();
    EXPECT_TRUE(stack.link_up(usb_if));
}

TEST_F(UsbNcmStackTest, AnswersArpAndPingOverUsb) {
    link_up();
    pass();
    // ARP request "who has 192.168.7.1"
    Bytes arp(28);
    rtos::net::store_be16(arp.data(), 1);
    rtos::net::store_be16(arp.data() + 2, 0x0800);
    arp[4] = std::byte{6};
    arp[5] = std::byte{4};
    rtos::net::store_be16(arp.data() + 6, 1);
    host_mac.store(arp.data() + 8);
    host_ip.store(arp.data() + 14);
    device_ip.store(arp.data() + 24);
    // ICMP echo request
    Bytes icmp(12);
    icmp[0] = std::byte{8};
    rtos::net::store_be16(icmp.data() + 4, 0x4242);
    rtos::net::store_be16(icmp.data() + 2, rtos::net::checksum(icmp));
    Bytes ip(20);
    ip[0] = std::byte{0x45};
    rtos::net::store_be16(ip.data() + 2, static_cast<std::uint16_t>(20 + icmp.size()));
    ip[8] = std::byte{64};
    ip[9] = std::byte{1};
    host_ip.store(ip.data() + 12);
    device_ip.store(ip.data() + 16);
    rtos::net::store_be16(ip.data() + 10, rtos::net::checksum(ip));
    ip.insert(ip.end(), icmp.begin(), icmp.end());

    ASSERT_TRUE(host.out(0x02, NtbBuilder{{ethernet(0x0806, arp, MacAddress::broadcast()), ethernet(0x0800, ip)}}.build()));
    pass();
    pass();
    const auto frames = sent_frames();
    ASSERT_EQ(frames.size(), 2u);
    EXPECT_EQ(rtos::net::load_be16(frames[0].data() + 12), 0x0806);  // ARP reply
    EXPECT_EQ(rtos::net::load_be16(frames[0].data() + 14 + 6), 2);
    EXPECT_EQ(rtos::net::load_be16(frames[1].data() + 12), 0x0800);  // echo reply
    EXPECT_EQ(frames[1][14 + 20], std::byte{0});
    EXPECT_EQ(MacAddress::load(frames[1].data()), host_mac);
    EXPECT_EQ(MacAddress::load(frames[1].data() + 6), device_mac);
}

TEST_F(UsbNcmStackTest, DhcpServerLeasesAnAddressOverUsb) {
    rtos::net::DhcpServer<rtos::net::Stack<test::TestPool>> dhcpd{stack, usb_if, {.first = host_ip, .count = 1}};
    ASSERT_TRUE(dhcpd.start());
    link_up();
    pass();
    // DHCPDISCOVER from 0.0.0.0:68 to 255.255.255.255:67
    Bytes m(240);
    m[0] = std::byte{1};
    m[1] = std::byte{1};
    m[2] = std::byte{6};
    rtos::net::store_be32(m.data() + 4, 0xCAFE0001);
    host_mac.store(m.data() + 28);
    rtos::net::store_be32(m.data() + 236, 0x63825363);
    for (int b : {53, 1, 1, 255}) {
        m.push_back(static_cast<std::byte>(b));
    }
    Bytes udp(8);
    rtos::net::store_be16(udp.data(), 68);
    rtos::net::store_be16(udp.data() + 2, 67);
    rtos::net::store_be16(udp.data() + 4, static_cast<std::uint16_t>(8 + m.size()));
    udp.insert(udp.end(), m.begin(), m.end());  // checksum 0: none
    Bytes ip(20);
    ip[0] = std::byte{0x45};
    rtos::net::store_be16(ip.data() + 2, static_cast<std::uint16_t>(20 + udp.size()));
    ip[8] = std::byte{64};
    ip[9] = std::byte{17};
    rtos::net::Ipv4Address::broadcast().store(ip.data() + 16);
    rtos::net::store_be16(ip.data() + 10, rtos::net::checksum(ip));
    ip.insert(ip.end(), udp.begin(), udp.end());
    ASSERT_TRUE(host.out(0x02, NtbBuilder{{ethernet(0x0800, ip, MacAddress::broadcast())}}.build()));
    pass();
    dhcpd.service();
    pass();
    const auto frames = sent_frames();
    ASSERT_EQ(frames.size(), 1u);
    const Bytes& offer = frames[0];
    EXPECT_EQ(rtos::net::load_be16(offer.data() + 14 + 20 + 2), 68);              // to the client port
    EXPECT_EQ(rtos::net::Ipv4Address::load(offer.data() + 14 + 28 + 16), host_ip);  // yiaddr
}

}  // namespace
