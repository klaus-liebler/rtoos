// SPDX-License-Identifier: MIT
//
// CDC-ACM over the simulated host: class requests (line coding, DTR/RTS), reception
// with flow control (OUT is not re-armed while the ring is full), transmission with
// the zero-length packet after a full last packet (TinyUSB
// test_usbd_stream_write_zlp_after_full_packet), DTR/RTS reset on bus reset (USBX
// usbx_cdc_acm_device_dtr_rts_reset_on_disconnect_test), and Modbus RTU served over
// the port (RtuService).
#include <gtest/gtest.h>

#include <string_view>

#include "rtos/modbus/register_bank.hpp"
#include "rtos/modbus/rtu_service.hpp"
#include "rtos/usb/cdc_acm.hpp"
#include "rtos/usb/descriptors.hpp"
#include "usb_fixture.hpp"

namespace {

using namespace usbtest;

constexpr CdcAcmInfo acm_info{.interface = 0, .notify_ep = 0x81, .out_ep = 0x02, .in_ep = 0x82, .string = 4};
constexpr auto device_desc = device_descriptor({});
constexpr auto config_desc = configuration_descriptor({}, cdc_acm_function(acm_info));
static_assert(config_desc.size() == 9 + 66);
static_assert(config_desc[4] == 2);  // two interfaces
constexpr std::string_view strings[] = {"rtoos", "FCU", "0001", "Modbus RTU"};

using Acm = CdcAcm<256, 256>;
using Dev = Device<MockController>;

struct Clock {
    std::uint32_t ms = 1000;
    [[nodiscard]] std::uint32_t now_ms() const noexcept { return ms; }
};

class UsbCdcTest : public ::testing::Test {
protected:
    MockController hw;
    Acm acm{acm_info};
    std::array<Function*, 1> functions{&acm};
    Dev dev{hw, {device_desc, config_desc, strings}, functions};
    Host<Dev> host{hw, dev};

    void SetUp() override { host.enumerate(); }

    static Bytes text(std::string_view s) {
        Bytes b;
        for (char c : s) {
            b.push_back(static_cast<std::byte>(c));
        }
        return b;
    }
    std::string read_all() {
        std::string out;
        std::array<std::byte, 64> buf{};
        while (const std::size_t n = acm.read(buf)) {
            out.append(reinterpret_cast<const char*>(buf.data()), n);
        }
        return out;
    }
};

TEST_F(UsbCdcTest, LineCodingRoundTrip) {
    // 9600 baud, 2 stop bits, even parity, 7 data bits
    EXPECT_FALSE(host.control(0x21, cdc_request::set_line_coding, 0, 0, 7, bytes({0x80, 0x25, 0, 0, 2, 2, 7})).stalled);
    EXPECT_EQ(acm.line_coding().baud, 9600u);
    EXPECT_EQ(acm.line_coding().data_bits, 7);
    EXPECT_EQ(host.control(0xA1, cdc_request::get_line_coding, 0, 0, 7).data, bytes({0x80, 0x25, 0, 0, 2, 2, 7}));
    EXPECT_TRUE(host.control(0x21, cdc_request::set_line_coding, 0, 0, 6, bytes({1, 2, 3, 4, 5, 6})).stalled);
}

TEST_F(UsbCdcTest, DtrMeansConnectedAndBusResetClearsIt) {
    EXPECT_FALSE(acm.connected());
    host.control(0x21, cdc_request::set_control_line_state, 0x0003, 0, 0);  // DTR + RTS
    EXPECT_TRUE(acm.connected());
    EXPECT_EQ(acm.line_state(), 3);
    host.reset();
    EXPECT_FALSE(acm.connected());
    EXPECT_EQ(acm.line_state(), 0);
}

TEST_F(UsbCdcTest, UnknownClassRequestStalls) {
    EXPECT_TRUE(host.control(0xA1, 0x7F, 0, 0, 4).stalled);
    EXPECT_FALSE(host.control(0x21, cdc_request::send_break, 0xFFFF, 0, 0).stalled);
}

TEST_F(UsbCdcTest, ReceivesPackets) {
    ASSERT_TRUE(host.out(0x02, text("hello ")));
    ASSERT_TRUE(host.out(0x02, text("world")));
    EXPECT_EQ(read_all(), "hello world");
}

TEST_F(UsbCdcTest, FullReceiveRingPausesTheHostUntilRead) {
    const Bytes packet(64, std::byte{'x'});
    int accepted = 0;
    while (host.out(0x02, packet) && accepted < 10) {
        ++accepted;
    }
    EXPECT_EQ(accepted, 4);  // 256-byte ring: the 5th packet would not fit -> NAK
    EXPECT_FALSE(host.out(0x02, packet));
    std::array<std::byte, 64> buf{};
    EXPECT_EQ(acm.read(buf), 64u);
    host.run();  // service(): room again
    EXPECT_TRUE(host.out(0x02, packet));
    EXPECT_EQ(read_all().size(), 4u * 64u);
}

TEST_F(UsbCdcTest, SendsWrittenData) {
    EXPECT_EQ(acm.write(text("abc")), 3u);
    host.run();
    const auto in = host.in(0x82);
    ASSERT_TRUE(in.has_value());
    EXPECT_EQ(*in, text("abc"));
    EXPECT_FALSE(host.in(0x82).has_value());  // nothing more
}

TEST_F(UsbCdcTest, ZeroLengthPacketAfterAFullLastPacket) {
    acm.write(Bytes(64, std::byte{'a'}));
    host.run();
    EXPECT_TRUE(host.in_zlp(0x82));  // ring empty after exactly one full packet
    ASSERT_EQ(host.in(0x82)->size(), 64u);
    acm.write(Bytes(100, std::byte{'b'}));
    host.run();
    ASSERT_EQ(host.in(0x82)->size(), 100u);  // one transfer of two packets, short end
}

TEST_F(UsbCdcTest, LargeWritesAreChunkedAndBoundedByTheRing) {
    EXPECT_EQ(acm.write(Bytes(600, std::byte{'c'})), 256u);  // ring size
    host.run();
    std::size_t total = 0;
    while (const auto in = host.in(0x82)) {
        total += in->size();
    }
    EXPECT_EQ(total, 256u);
    EXPECT_EQ(acm.write_space(), 256u);
}

TEST_F(UsbCdcTest, NothingIsSentBeforeConfiguration) {
    host.reset();
    acm.write(text("early"));
    host.run();
    EXPECT_FALSE(host.in(0x82).has_value());
    host.enumerate();
    EXPECT_EQ(*host.in(0x82), text("early"));  // queued data goes out once configured
}

// ---------------------------------------------------------------------------
// Modbus RTU over the CDC-ACM port
// ---------------------------------------------------------------------------

Bytes rtu_frame(std::initializer_list<int> body) {
    Bytes f = bytes(body);
    const std::uint16_t crc = rtos::modbus::crc16(f);
    f.push_back(static_cast<std::byte>(crc & 0xFF));
    f.push_back(static_cast<std::byte>(crc >> 8));
    return f;
}

class UsbModbusRtuTest : public UsbCdcTest {
protected:
    Clock clock;
    rtos::modbus::RegisterBank<16, 4> registers;
    rtos::modbus::RtuService<Acm, Clock> rtu{acm, clock, registers, 1, 5};

    void pass() {
        host.run();      // USB device (first in the network thread)
        rtu.service();   // then the RTU service
        host.run();      // its response goes out
    }
};

TEST_F(UsbModbusRtuTest, RequestInTwoPacketsIsAnswered) {
    registers.set_holding(2, 0x4242);
    const Bytes request = rtu_frame({1, 0x03, 0x00, 0x02, 0x00, 0x01});
    host.out(0x02, Bytes(request.begin(), request.begin() + 3));
    pass();
    EXPECT_FALSE(host.in(0x82).has_value());
    host.out(0x02, Bytes(request.begin() + 3, request.end()));
    pass();
    const auto response = host.in(0x82);
    ASSERT_TRUE(response.has_value());
    EXPECT_EQ(*response, rtu_frame({1, 0x03, 0x02, 0x42, 0x42}));
}

TEST_F(UsbModbusRtuTest, SilenceEndsAnUnknownFunctionWithException01) {
    host.out(0x02, rtu_frame({1, 0x2B, 0x0E, 0x01, 0x00}));
    pass();
    EXPECT_FALSE(host.in(0x82).has_value());
    EXPECT_EQ(rtu.next_deadline(), clock.ms + 5);
    clock.ms += 5;
    pass();
    EXPECT_EQ(*host.in(0x82), rtu_frame({1, 0xAB, 0x01}));
    EXPECT_EQ(rtu.next_deadline(), 0u);
}

TEST_F(UsbModbusRtuTest, SilenceDropsAHalfFrame) {
    const Bytes request = rtu_frame({1, 0x06, 0x00, 0x00, 0x00, 0x07});
    host.out(0x02, Bytes(request.begin(), request.begin() + 4));
    pass();
    clock.ms += 10;
    pass();
    EXPECT_EQ(rtu.server().buffered(), 0u);
    host.out(0x02, request);
    pass();
    EXPECT_EQ(*host.in(0x82), request);  // FC06 echo: the next frame is in sync
    EXPECT_EQ(registers.holding(0), 7);
}

}  // namespace
