// SPDX-License-Identifier: MIT
//
// usb::Device: enumeration and the standard requests of USB 2.0 chapter 9, control
// transfer stages (cases from TinyUSB's test_usbd.c: zero-length packet on a packet
// boundary, OUT data longer than requested), class/vendor dispatch, endpoint halt,
// alternate settings, bus reset. Driven by the simulated host of usb_fixture.hpp.
#include <gtest/gtest.h>

#include <string_view>

#include "rtos/usb/descriptors.hpp"
#include "usb_fixture.hpp"

namespace {

using namespace usbtest;

/// One vendor interface (0) with bulk OUT 0x01 / IN 0x81 and alternate settings 0, 1.
struct TestFunction final : Function {
    int configured_calls = 0, resets = 0;
    std::uint8_t alternate = 0;
    Bytes vendor_in = bytes({1, 2, 3});
    Bytes vendor_out;
    std::array<std::byte, 64> rx{};
    Bytes received;

    std::uint8_t first_interface() const noexcept override { return 0; }
    std::uint8_t interface_count() const noexcept override { return 1; }
    bool owns_endpoint(std::uint8_t ep) const noexcept override { return ep == 0x01 || ep == 0x81; }
    void configured(Endpoints& eps) noexcept override {
        ++configured_calls;
        eps.open({0x01, TransferType::Bulk, 64});
        eps.open({0x81, TransferType::Bulk, 64});
        eps.transfer(0x01, rx);
    }
    void reset() noexcept override {
        ++resets;
        alternate = 0;
    }
    bool set_interface(Endpoints&, std::uint8_t, std::uint8_t alt) noexcept override {
        if (alt > 1) {
            return false;
        }
        alternate = alt;
        return true;
    }
    std::uint8_t interface_alternate(std::uint8_t) const noexcept override { return alternate; }
    ControlResult control_setup(const Setup& s, std::span<std::byte> buffer, std::size_t& in_length) noexcept override {
        if (s.type() != Setup::Type::Vendor) {
            return ControlResult::Stall;
        }
        if (s.request == 0x01 && s.device_to_host()) {
            in_length = std::min(vendor_in.size(), buffer.size());
            std::copy_n(vendor_in.begin(), in_length, buffer.begin());
            return ControlResult::Accept;
        }
        if (s.request == 0x02 && !s.device_to_host()) {
            return ControlResult::Accept;
        }
        return ControlResult::Stall;
    }
    void control_data(const Setup&, std::span<const std::byte> data) noexcept override {
        vendor_out.assign(data.begin(), data.end());
    }
    void transfer_complete(Endpoints& eps, std::uint8_t ep, std::size_t length) noexcept override {
        if (ep == 0x01) {
            received.insert(received.end(), rx.begin(), rx.begin() + static_cast<std::ptrdiff_t>(length));
            eps.transfer(0x01, rx);
        }
    }
};

constexpr auto device_desc = device_descriptor({.vendor = 0x1209, .product = 0x0001});
constexpr auto config_desc = configuration_descriptor(
    {.max_power_ma = 100},
    detail::join(detail::interface(0, 0, 0, 0xFF, 0, 0, 0), detail::interface(0, 1, 2, 0xFF, 0, 0, 0),
                 detail::endpoint(0x01, TransferType::Bulk, 64, 0), detail::endpoint(0x81, TransferType::Bulk, 64, 0)));
static_assert(config_desc.size() == 9 + 9 + 9 + 7 + 7);
static_assert(config_desc[2] == config_desc.size() && config_desc[4] == 1);  // wTotalLength, bNumInterfaces
constexpr std::string_view strings[] = {"rtoos", "FCU test", "0001"};

using Dev = Device<MockController>;

class UsbDeviceTest : public ::testing::Test {
protected:
    MockController hw;
    TestFunction fn;
    std::array<Function*, 1> functions{&fn};
    Dev dev{hw, {device_desc, config_desc, strings}, functions};
    Host<Dev> host{hw, dev};
};

TEST_F(UsbDeviceTest, DeviceDescriptorAtAddressZeroIsClampedToItsLength) {
    host.reset();
    const auto r = host.control(0x80, request::get_descriptor, 0x0100, 0, 64);
    ASSERT_FALSE(r.stalled);
    ASSERT_EQ(r.data.size(), 18u);
    EXPECT_EQ(r.data[0], std::byte{18});
    EXPECT_EQ(r.data[7], std::byte{64});  // bMaxPacketSize0
    EXPECT_EQ(host.control(0x80, request::get_descriptor, 0x0100, 0, 8).data.size(), 8u);
}

TEST_F(UsbDeviceTest, SetAddressTakesEffectAfterTheStatusStage) {  // 9.4.6
    host.reset();
    Event e{EventType::Setup};
    const std::uint8_t raw[8] = {0x00, request::set_address, 7, 0, 0, 0, 0, 0};
    std::copy_n(reinterpret_cast<const std::byte*>(raw), 8, e.setup.begin());
    hw.events.push_back(e);
    host.run();
    EXPECT_EQ(hw.address, 0) << "still address 0 until the status stage is done";
    ASSERT_TRUE(hw.at(0x80).active);
    host.complete(0x80, 0);
    EXPECT_EQ(hw.address, 7);
    EXPECT_EQ(dev.state(), DeviceState::Address);
}

TEST_F(UsbDeviceTest, EnumerationConfiguresTheFunction) {
    host.enumerate();
    EXPECT_TRUE(dev.configured());
    EXPECT_EQ(fn.configured_calls, 1);
    EXPECT_TRUE(hw.open_eps[MockController::slot(0x81)].has_value());
    EXPECT_TRUE(hw.at(0x01).active);  // reception started
    EXPECT_EQ(host.control(0x80, request::get_configuration, 0, 0, 1).data, bytes({1}));
}

TEST_F(UsbDeviceTest, ConfigurationDescriptorHeaderThenAll) {
    host.enumerate();
    const auto head = host.control(0x80, request::get_descriptor, 0x0200, 0, 9);
    EXPECT_EQ(head.data.size(), 9u);
    const auto all = host.control(0x80, request::get_descriptor, 0x0200, 0, 0xFF);
    EXPECT_EQ(all.data.size(), config_desc.size());
    EXPECT_TRUE(host.control(0x80, request::get_descriptor, 0x0201, 0, 9).stalled);  // only configuration 0
}

TEST_F(UsbDeviceTest, StringDescriptorsAreUtf16) {
    host.enumerate();
    EXPECT_EQ(host.control(0x80, request::get_descriptor, 0x0300, 0, 255).data, bytes({4, 3, 0x09, 0x04}));
    EXPECT_EQ(host.control(0x80, request::get_descriptor, 0x0301, 0x0409, 255).data,
              bytes({12, 3, 'r', 0, 't', 0, 'o', 0, 'o', 0, 's', 0}));
    EXPECT_TRUE(host.control(0x80, request::get_descriptor, 0x0304, 0x0409, 255).stalled);
}

TEST_F(UsbDeviceTest, FullSpeedDeviceStallsDeviceQualifier) {  // 9.6.2
    host.enumerate();
    EXPECT_TRUE(host.control(0x80, request::get_descriptor, 0x0600, 0, 10).stalled);
    EXPECT_TRUE(host.control(0x80, request::get_descriptor, 0x0700, 0, 9).stalled);
}

TEST_F(UsbDeviceTest, StallIsClearedByTheNextSetup) {
    host.enumerate();
    EXPECT_TRUE(host.control(0x80, 0x55, 0, 0, 4).stalled);  // unknown standard request
    EXPECT_FALSE(host.control(0x80, request::get_configuration, 0, 0, 1).stalled);
    EXPECT_GE(dev.stats().stalls, 1u);
}

// TinyUSB test_usbd_control_in_zlp: a shorter answer than requested that ends on a
// packet boundary needs a zero-length packet; an exact answer does not.
TEST_F(UsbDeviceTest, ControlInRequestsZlpOnlyWhenShorterThanRequested) {
    host.enumerate();
    fn.vendor_in = Bytes(64, std::byte{0xAB});
    auto r = host.control(0xC1, 0x01, 0, 0, 100);
    EXPECT_EQ(r.data.size(), 64u);
    EXPECT_TRUE(r.zlp);
    r = host.control(0xC1, 0x01, 0, 0, 64);
    EXPECT_EQ(r.data.size(), 64u);
    EXPECT_FALSE(r.zlp);
    r = host.control(0xC1, 0x01, 0, 0, 10);  // clamped to wLength
    EXPECT_EQ(r.data.size(), 10u);
    EXPECT_FALSE(r.zlp);
}

TEST_F(UsbDeviceTest, ClassOutDataStageReachesTheFunction) {
    host.enumerate();
    EXPECT_FALSE(host.control(0x41, 0x02, 0, 0, 7, bytes({1, 2, 3, 4, 5, 6, 7})).stalled);
    EXPECT_EQ(fn.vendor_out, bytes({1, 2, 3, 4, 5, 6, 7}));
}

// TinyUSB test_usbd_control_out_overrun_clamp: the host sends more than wLength.
TEST_F(UsbDeviceTest, ControlOutLongerThanRequestedIsClamped) {
    host.enumerate();
    EXPECT_FALSE(host.control(0x41, 0x02, 0, 0, 3, bytes({1, 2, 3, 4, 5, 6})).stalled);
    EXPECT_EQ(fn.vendor_out, bytes({1, 2, 3}));
}

TEST_F(UsbDeviceTest, ControlOutLargerThanTheBufferStalls) {
    host.enumerate();
    EXPECT_TRUE(host.control(0x41, 0x02, 0, 0, 300, Bytes(300)).stalled);
}

TEST_F(UsbDeviceTest, ClassRequestsBeforeConfigurationStall) {
    host.reset();
    EXPECT_TRUE(host.control(0xC1, 0x01, 0, 0, 3).stalled);
}

TEST_F(UsbDeviceTest, SetConfigurationRules) {
    host.reset();
    EXPECT_TRUE(host.control(0x00, request::set_configuration, 1, 0, 0).stalled);  // Default state
    host.control(0x00, request::set_address, 3, 0, 0);
    EXPECT_TRUE(host.control(0x00, request::set_configuration, 2, 0, 0).stalled);  // no such configuration
    EXPECT_FALSE(host.control(0x00, request::set_configuration, 1, 0, 0).stalled);
    EXPECT_TRUE(dev.configured());
    EXPECT_TRUE(host.control(0x00, request::set_address, 4, 0, 0).stalled);  // not while configured
    EXPECT_FALSE(host.control(0x00, request::set_configuration, 0, 0, 0).stalled);
    EXPECT_EQ(dev.state(), DeviceState::Address);
    EXPECT_FALSE(hw.open_eps[MockController::slot(0x81)].has_value());
    EXPECT_EQ(host.control(0x80, request::get_configuration, 0, 0, 1).data, bytes({0}));
}

TEST_F(UsbDeviceTest, DeviceStatusAndRemoteWakeup) {
    host.enumerate();
    EXPECT_EQ(host.control(0x80, request::get_status, 0, 0, 2).data, bytes({0, 0}));  // bus powered
    host.control(0x00, request::set_feature, feature::remote_wakeup, 0, 0);
    EXPECT_EQ(host.control(0x80, request::get_status, 0, 0, 2).data, bytes({2, 0}));
    host.control(0x00, request::clear_feature, feature::remote_wakeup, 0, 0);
    EXPECT_EQ(host.control(0x80, request::get_status, 0, 0, 2).data, bytes({0, 0}));
    EXPECT_TRUE(host.control(0x00, request::set_feature, feature::test_mode, 0, 0).stalled);
}

TEST_F(UsbDeviceTest, EndpointHalt) {
    host.enumerate();
    EXPECT_FALSE(host.control(0x02, request::set_feature, feature::endpoint_halt, 0x81, 0).stalled);
    EXPECT_TRUE(hw.stalled[MockController::slot(0x81)]);
    EXPECT_EQ(host.control(0x82, request::get_status, 0, 0x81, 2).data, bytes({1, 0}));
    EXPECT_FALSE(host.control(0x02, request::clear_feature, feature::endpoint_halt, 0x81, 0).stalled);
    EXPECT_FALSE(hw.stalled[MockController::slot(0x81)]);
    EXPECT_EQ(host.control(0x82, request::get_status, 0, 0x81, 2).data, bytes({0, 0}));
    EXPECT_TRUE(host.control(0x82, request::get_status, 0, 0x83, 2).stalled);  // not ours
}

TEST_F(UsbDeviceTest, AlternateSettings) {
    host.enumerate();
    EXPECT_FALSE(host.control(0x01, request::set_interface, 1, 0, 0).stalled);
    EXPECT_EQ(fn.alternate, 1);
    EXPECT_EQ(host.control(0x81, request::get_interface, 0, 0, 1).data, bytes({1}));
    EXPECT_TRUE(host.control(0x01, request::set_interface, 2, 0, 0).stalled);
    EXPECT_TRUE(host.control(0x01, request::set_interface, 0, 3, 0).stalled);  // no interface 3
    EXPECT_EQ(host.control(0x81, request::get_status, 0, 0, 2).data, bytes({0, 0}));
}

TEST_F(UsbDeviceTest, BusResetDeconfigures) {
    host.enumerate();
    const int resets = fn.resets;
    host.reset();
    EXPECT_EQ(dev.state(), DeviceState::Default);
    EXPECT_EQ(fn.resets, resets + 1);
    EXPECT_FALSE(hw.open_eps[MockController::slot(0x01)].has_value());
    EXPECT_EQ(dev.address(), 0);
}

TEST_F(UsbDeviceTest, BulkCompletionsGoToTheOwningFunction) {
    host.enumerate();
    ASSERT_TRUE(host.out(0x01, bytes({9, 8, 7})));
    ASSERT_TRUE(host.out(0x01, bytes({6})));
    EXPECT_EQ(fn.received, bytes({9, 8, 7, 6}));
}

TEST_F(UsbDeviceTest, NewSetupAbortsAControlTransferInProgress) {
    host.enumerate();
    Event e{EventType::Setup};
    const std::uint8_t raw[8] = {0x80, request::get_descriptor, 0, 1, 0, 0, 18, 0};
    std::copy_n(reinterpret_cast<const std::byte*>(raw), 8, e.setup.begin());
    hw.events.push_back(e);
    host.run();  // data stage armed, never completed by the host
    EXPECT_EQ(host.control(0x80, request::get_configuration, 0, 0, 1).data, bytes({1}));
}

// Found by usb_fuzz_test.cpp: the direction and data stage of standard requests are
// fixed (9.4); a request that breaks them is a request error, not a status stage.
TEST_F(UsbDeviceTest, StandardRequestsWithTheWrongDirectionOrDataStageStall) {
    host.enumerate();
    EXPECT_TRUE(host.control(0x81, request::set_interface, 0, 0, 1).stalled);           // SET with IN
    EXPECT_TRUE(host.control(0x00, request::set_configuration, 1, 0, 4, Bytes(4)).stalled);  // OUT data
    EXPECT_TRUE(host.control(0x00, request::get_descriptor, 0x0100, 0, 0).stalled);     // GET with OUT
    EXPECT_FALSE(host.control(0x80, request::get_configuration, 0, 0, 1).stalled);      // still fine
}

TEST_F(UsbDeviceTest, InRequestWithZeroLengthHasOnlyTheStatusStage) {  // 8.5.3
    host.enumerate();
    const auto r = host.control(0x80, request::get_status, 0, 0, 0);  // the host checks: IN status, no data
    EXPECT_FALSE(r.stalled);
    EXPECT_TRUE(r.data.empty());
}

TEST_F(UsbDeviceTest, SuspendAndResume) {
    host.enumerate();
    hw.events.push_back({EventType::Suspend});
    host.run();
    EXPECT_TRUE(dev.suspended());
    hw.events.push_back({EventType::Resume});
    host.run();
    EXPECT_FALSE(dev.suspended());
    EXPECT_TRUE(dev.configured());
}

}  // namespace
