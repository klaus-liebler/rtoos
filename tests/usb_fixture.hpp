// SPDX-License-Identifier: MIT
//
// Host test fixture for the USB device stack (the idea of USBX's regression tests:
// a simulated host drives the real device stack in-process):
//   MockController  a DeviceController that records endpoints, transfers, stalls
//                   and the address; the test completes transfers like the hardware
//   Host            performs control transfers (setup, data and status stage) and
//                   bulk/interrupt transfers against it
#pragma once

#include <gtest/gtest.h>

#include <array>
#include <deque>
#include <optional>
#include <vector>

#include "rtos/usb/device.hpp"
#include "rtos/usb/usb.hpp"

namespace usbtest {

using namespace rtos::usb;
using Bytes = std::vector<std::byte>;

inline Bytes bytes(std::initializer_list<int> v) {
    Bytes out;
    for (int b : v) {
        out.push_back(static_cast<std::byte>(b));
    }
    return out;
}

struct MockController {
    struct Transfer {
        std::span<std::byte> buffer;
        bool zlp = false;
        bool active = false;
    };

    std::deque<Event> events;
    std::array<Transfer, 32> transfers{};  // [number + 16 * in]
    std::array<std::optional<EndpointConfig>, 32> open_eps{};
    std::array<bool, 32> stalled{};
    std::uint8_t address = 0;
    bool connected = false;
    int close_alls = 0;

    static std::size_t slot(std::uint8_t ep) { return endpoint_number(ep) + (is_in(ep) ? 16 : 0); }

    std::optional<Event> poll() noexcept {
        if (events.empty()) {
            return std::nullopt;
        }
        Event e = events.front();
        events.pop_front();
        return e;
    }
    bool open(const EndpointConfig& c) noexcept {
        open_eps[slot(c.address)] = c;
        return true;
    }
    void close_all() noexcept {
        ++close_alls;
        for (std::size_t i = 0; i < 32; ++i) {
            if (i % 16 != 0) {
                open_eps[i].reset();
                transfers[i] = {};
                stalled[i] = false;
            }
        }
    }
    bool transfer(std::uint8_t ep, std::span<std::byte> buffer, bool zlp) noexcept {
        Transfer& t = transfers[slot(ep)];
        if (t.active || (endpoint_number(ep) != 0 && !open_eps[slot(ep)])) {
            return false;
        }
        t = {buffer, zlp, true};
        return true;
    }
    void stall(std::uint8_t ep) noexcept { stalled[slot(ep)] = true; }
    void clear_stall(std::uint8_t ep) noexcept { stalled[slot(ep)] = false; }
    void set_address(std::uint8_t a) noexcept { address = a; }
    void connect() noexcept { connected = true; }
    void disconnect() noexcept { connected = false; }

    Transfer& at(std::uint8_t ep) { return transfers[slot(ep)]; }
};
static_assert(DeviceController<MockController>);

/// The simulated host.
template <class Dev>
struct Host {
    MockController& hw;
    Dev& dev;

    struct Result {
        bool stalled = false;
        Bytes data;      ///< IN data stage
        bool zlp = false;  ///< the device asked for a terminating zero-length packet
    };

    void run() { dev.service(); }

    void reset() {
        hw.events.push_back({EventType::BusReset});
        run();
    }

    Result control(std::uint8_t type, std::uint8_t request, std::uint16_t value, std::uint16_t index,
                   std::uint16_t length, const Bytes& out = {}) {
        Event e{EventType::Setup};
        const std::uint8_t raw[8] = {type, request, static_cast<std::uint8_t>(value), static_cast<std::uint8_t>(value >> 8),
                                     static_cast<std::uint8_t>(index), static_cast<std::uint8_t>(index >> 8),
                                     static_cast<std::uint8_t>(length), static_cast<std::uint8_t>(length >> 8)};
        for (int i = 0; i < 8; ++i) {
            e.setup[i] = static_cast<std::byte>(raw[i]);
        }
        hw.stalled[0] = hw.stalled[16] = false;  // a SETUP clears the EP0 stall
        hw.at(0x00) = {};
        hw.at(0x80) = {};
        hw.events.push_back(e);
        run();
        Result r;
        if (hw.stalled[0] || hw.stalled[16]) {
            r.stalled = true;
            return r;
        }
        if ((type & 0x80) != 0 && length != 0) {  // IN data stage
            auto& t = hw.at(0x80);
            EXPECT_TRUE(t.active) << "no IN data stage";
            if (!t.active) {
                return r;
            }
            r.data.assign(t.buffer.begin(), t.buffer.end());
            r.zlp = t.zlp;
            complete(0x80, t.buffer.size());
            auto& status = hw.at(0x00);
            EXPECT_TRUE(status.active && status.buffer.empty()) << "no OUT status stage";
            complete(0x00, 0);
            return r;
        }
        if (length != 0) {  // OUT data stage
            auto& t = hw.at(0x00);
            EXPECT_TRUE(t.active) << "no OUT data stage";
            if (!t.active) {
                return r;
            }
            const std::size_t n = std::min(out.size(), t.buffer.size());
            std::copy_n(out.begin(), n, t.buffer.begin());
            complete(0x00, n);
            if (hw.stalled[0] || hw.stalled[16]) {
                r.stalled = true;
                return r;
            }
        }
        auto& status = hw.at(0x80);  // IN status stage
        EXPECT_TRUE(status.active && status.buffer.empty()) << "no IN status stage";
        complete(0x80, 0);
        return r;
    }

    void complete(std::uint8_t ep, std::size_t length) {
        hw.at(ep).active = false;
        hw.events.push_back({EventType::TransferComplete, ep, static_cast<std::uint16_t>(length)});
        run();
    }

    /// Bulk/interrupt IN: the data of the armed transfer (nullopt: nothing armed).
    std::optional<Bytes> in(std::uint8_t ep) {
        auto& t = hw.at(ep);
        if (!t.active) {
            return std::nullopt;
        }
        Bytes data(t.buffer.begin(), t.buffer.end());
        complete(ep, data.size());
        return data;
    }
    [[nodiscard]] bool in_zlp(std::uint8_t ep) { return hw.at(ep).zlp; }

    /// Bulk OUT: false if the device has no transfer armed (it NAKs).
    bool out(std::uint8_t ep, const Bytes& data) {
        auto& t = hw.at(ep);
        if (!t.active) {
            return false;
        }
        const std::size_t n = std::min(data.size(), t.buffer.size());
        std::copy_n(data.begin(), n, t.buffer.begin());
        complete(ep, n);
        return true;
    }

    /// The usual enumeration (Windows-like): 64-byte device descriptor at address 0,
    /// reset, SET_ADDRESS, descriptors, SET_CONFIGURATION 1.
    void enumerate(std::uint8_t address = 5) {
        reset();
        control(0x80, request::get_descriptor, 0x0100, 0, 64);
        reset();
        EXPECT_FALSE(control(0x00, request::set_address, address, 0, 0).stalled);
        control(0x80, request::get_descriptor, 0x0100, 0, 18);
        const auto head = control(0x80, request::get_descriptor, 0x0200, 0, 9).data;
        const std::uint16_t total = head.size() >= 4 ? static_cast<std::uint16_t>(std::to_integer<int>(head[2]) |
                                                                                   (std::to_integer<int>(head[3]) << 8))
                                                     : 0;
        control(0x80, request::get_descriptor, 0x0200, 0, total);
        EXPECT_FALSE(control(0x00, request::set_configuration, 1, 0, 0).stalled);
    }
};

}  // namespace usbtest
