// SPDX-License-Identifier: MIT
//
// Mutation fuzzing of the USB device stack (TinyUSB fuzzes usbd, cdc and net_ncm
// the same way): mutated SETUP packets (standard, class, vendor; every recipient)
// against a composite NCM + CDC-ACM device, with random bus resets, and mutated NTBs
// against the NCM receive path. Invariants: no contract violation or crash, every
// accepted control request runs its data and status stages (the simulated host
// checks them), no packet buffer is lost, and the device still enumerates and
// passes data afterwards.
#include <gtest/gtest.h>

#include <string_view>

#include "fuzz_support.hpp"
#include "net_fixture.hpp"
#include "rtos/usb/cdc_acm.hpp"
#include "rtos/usb/cdc_ncm.hpp"
#include "rtos/usb/descriptors.hpp"
#include "usb_fixture.hpp"

namespace {

using namespace usbtest;
using fuzz::Mutator;

constexpr CdcNcmInfo ncm_info{.interface = 0, .notify_ep = 0x81, .out_ep = 0x02, .in_ep = 0x82, .string = 4, .mac_string = 5};
constexpr CdcAcmInfo acm_info{.interface = 2, .notify_ep = 0x83, .out_ep = 0x04, .in_ep = 0x84, .string = 6};
constexpr auto device_desc = device_descriptor({});
constexpr auto config_desc = configuration_descriptor({}, cdc_ncm_function(ncm_info), cdc_acm_function(acm_info));
constexpr std::string_view strings[] = {"rtoos", "FCU", "0001", "FCU network", "020000000201", "FCU console"};

using Ncm = CdcNcm<test::TestPool, 4>;
using Acm = CdcAcm<256, 256>;
using Dev = Device<MockController>;

class UsbFuzz : public ::testing::Test {
protected:
    test::TestPool pool;
    MockController hw;
    Ncm ncm{ncm_info, pool, rtos::net::MacAddress::from(2, 0, 0, 0, 1, 1)};
    Acm acm{acm_info};
    std::array<Function*, 2> functions{&ncm, &acm};
    Dev dev{hw, {device_desc, config_desc, strings}, functions};
    Host<Dev> host{hw, dev};
};

TEST_F(UsbFuzz, SetupPackets) {
    Mutator m{fuzz::seed(0x5EED'0101u)};
    // Valid requests as seeds: standard, CDC-ACM, NCM.
    const std::vector<std::array<std::uint8_t, 8>> seeds = {
        {0x80, request::get_descriptor, 0x00, 0x01, 0, 0, 64, 0},
        {0x80, request::get_descriptor, 0x00, 0x02, 0, 0, 0xFF, 0},
        {0x80, request::get_descriptor, 0x02, 0x03, 0x09, 0x04, 0xFF, 0},
        {0x00, request::set_address, 9, 0, 0, 0, 0, 0},
        {0x00, request::set_configuration, 1, 0, 0, 0, 0, 0},
        {0x01, request::set_interface, 1, 0, 1, 0, 0, 0},
        {0x81, request::get_interface, 0, 0, 1, 0, 1, 0},
        {0x02, request::clear_feature, 0, 0, 0x82, 0, 0, 0},
        {0x82, request::get_status, 0, 0, 0x84, 0, 2, 0},
        {0xA1, ncm::get_ntb_parameters, 0, 0, 0, 0, 28, 0},
        {0x21, ncm::set_ntb_input_size, 0, 0, 0, 0, 4, 0},
        {0x21, cdc_request::set_line_coding, 0, 0, 2, 0, 7, 0},
        {0xA1, cdc_request::get_line_coding, 0, 0, 2, 0, 7, 0},
        {0x21, cdc_request::set_control_line_state, 3, 0, 2, 0, 0, 0},
    };
    const std::size_t n = fuzz::iterations(20'000);
    host.enumerate();
    for (std::size_t i = 0; i < n; ++i) {
        if (m.chance(200)) {
            host.reset();
        }
        if (m.chance(100)) {
            host.enumerate();
        }
        const auto& s = seeds[m.below(static_cast<std::uint32_t>(seeds.size()))];
        Bytes b(s.size());
        std::transform(s.begin(), s.end(), b.begin(), [](std::uint8_t x) { return std::byte{x}; });
        m.mutate(b, 0, 8);
        b.resize(8);
        const auto u8 = [&](int k) { return std::to_integer<std::uint8_t>(b[k]); };
        const auto u16 = [&](int k) { return static_cast<std::uint16_t>(u8(k) | (u8(k + 1) << 8)); };
        // OUT data for a host-to-device data stage (at most what the host would send)
        Bytes out(std::min<std::size_t>(u16(6), 300));
        for (auto& x : out) {
            x = static_cast<std::byte>(m.below(256));
        }
        host.control(u8(0), u8(1), u16(2), u16(4), u16(6), out);
        ASSERT_FALSE(HasFailure()) << "iteration " << i << ": setup " << std::hex << int{u8(0)} << ' ' << int{u8(1)}
                                   << ' ' << u16(2) << ' ' << u16(4) << ' ' << u16(6) << ", state "
                                   << static_cast<int>(dev.state());
        // Bulk traffic now and then, whatever state the device is in.
        if (m.chance(8)) {
            host.out(0x04, Bytes(1 + m.below(64), std::byte{'x'}));
            acm.write(Bytes(m.below(100), std::byte{'y'}));
            host.run();
            (void)host.in(0x84);
            std::array<std::byte, 64> sink{};
            while (acm.read(sink) != 0) {
            }
        }
        while (rtos::net::PacketRef p = ncm.receive()) {
        }
    }
    // Still a working device.
    host.enumerate();
    EXPECT_TRUE(dev.configured());
    ASSERT_FALSE(host.control(0x01, request::set_interface, 1, 1, 0).stalled);
    EXPECT_TRUE(ncm.link_up());
    host.run();
    while (host.in(0x84)) {  // what the fuzz loop left in the transmit ring
    }
    acm.write(Bytes(3, std::byte{'z'}));
    host.run();
    EXPECT_EQ(host.in(0x84)->size(), 3u);
}

TEST_F(UsbFuzz, NcmNtbs) {
    Mutator m{fuzz::seed(0x5EED'0102u)};
    host.enumerate();
    ASSERT_FALSE(host.control(0x01, request::set_interface, 1, 1, 0).stalled);
    host.in(0x81);
    host.in(0x81);
    // A valid NTB with three datagrams as the seed.
    Bytes seed(12);
    const auto put16 = [](Bytes& b, std::size_t at, std::uint16_t v) {
        b[at] = static_cast<std::byte>(v);
        b[at + 1] = static_cast<std::byte>(v >> 8);
    };
    put16(seed, 0, 0x434E);
    put16(seed, 2, 0x484D);
    put16(seed, 4, 12);
    std::vector<std::size_t> offsets;
    for (std::size_t size : {60, 61, 90}) {
        while (seed.size() % 4 != 0) {
            seed.push_back(std::byte{0});
        }
        offsets.push_back(seed.size());
        seed.resize(seed.size() + size, std::byte{0x5A});
    }
    while (seed.size() % 4 != 0) {
        seed.push_back(std::byte{0});
    }
    const std::size_t ndp = seed.size();
    seed.resize(ndp + 8 + 4 * 4);
    put16(seed, 10, static_cast<std::uint16_t>(ndp));
    put16(seed, ndp, 0x434E);
    put16(seed, ndp + 2, 0x304D);
    put16(seed, ndp + 4, 8 + 4 * 4);
    const std::size_t sizes[] = {60, 61, 90};
    for (std::size_t i = 0; i < 3; ++i) {
        put16(seed, ndp + 8 + 4 * i, static_cast<std::uint16_t>(offsets[i]));
        put16(seed, ndp + 10 + 4 * i, static_cast<std::uint16_t>(sizes[i]));
    }
    put16(seed, 8, static_cast<std::uint16_t>(seed.size()));

    const std::size_t n = fuzz::iterations(20'000);
    std::size_t delivered = 0;
    for (std::size_t i = 0; i < n; ++i) {
        Bytes ntb = seed;
        m.mutate(ntb, 0, Ncm::ntb_size);
        ASSERT_TRUE(host.out(0x02, ntb)) << "iteration " << i << ": reception stuck";
        while (rtos::net::PacketRef p = ncm.receive()) {
            ASSERT_GE(p->size(), 14u);
            ASSERT_LE(p->size(), Ncm::max_datagram);
            ++delivered;
        }
        host.run();
    }
    EXPECT_GT(delivered, n / 4);  // the mutations do not reject everything
    EXPECT_EQ(pool.available(), pool.capacity());
    ASSERT_TRUE(host.out(0x02, seed));
    std::size_t frames = 0;
    while (rtos::net::PacketRef p = ncm.receive()) {
        ++frames;
    }
    EXPECT_EQ(frames, 3u);
}

}  // namespace
