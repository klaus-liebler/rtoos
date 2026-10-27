// SPDX-License-Identifier: MIT
//
// Descriptors built at compile time (constexpr std::array<std::uint8_t, N>):
//
//   constexpr auto device = usb::device_descriptor({.vendor = 0x1209, .product = 0x0001, ...});
//   constexpr auto config = usb::configuration_descriptor(
//       {.max_power_ma = 100},
//       usb::cdc_ncm_function({.interface = 0, .notify_ep = 0x81, ...}),
//       usb::cdc_acm_function({.interface = 2, ...}));
//
// configuration_descriptor() fills in wTotalLength and bNumInterfaces. Composite
// devices use IADs (bDeviceClass 0xEF/2/1).
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>

#include "rtos/usb/usb.hpp"

namespace rtos::usb {

template <std::size_t N>
using Descriptor = std::array<std::uint8_t, N>;

namespace detail {
template <std::size_t... Ns>
constexpr auto join(const Descriptor<Ns>&... parts) noexcept {
    Descriptor<(Ns + ... + 0)> out{};
    std::size_t at = 0;
    ((std::copy(parts.begin(), parts.end(), out.begin() + static_cast<std::ptrdiff_t>(at)), at += Ns), ...);
    return out;
}
constexpr std::uint8_t lo(std::uint16_t v) noexcept { return static_cast<std::uint8_t>(v & 0xFF); }
constexpr std::uint8_t hi(std::uint16_t v) noexcept { return static_cast<std::uint8_t>(v >> 8); }

constexpr Descriptor<7> endpoint(std::uint8_t address, TransferType type, std::uint16_t max_packet,
                                 std::uint8_t interval) noexcept {
    return {7, descriptor::endpoint, address, static_cast<std::uint8_t>(type), lo(max_packet), hi(max_packet), interval};
}
constexpr Descriptor<9> interface(std::uint8_t number, std::uint8_t alternate, std::uint8_t endpoints, std::uint8_t cls,
                                  std::uint8_t subclass, std::uint8_t protocol, std::uint8_t string) noexcept {
    return {9, descriptor::interface, number, alternate, endpoints, cls, subclass, protocol, string};
}
constexpr Descriptor<8> iad(std::uint8_t first, std::uint8_t count, std::uint8_t cls, std::uint8_t subclass,
                            std::uint8_t protocol, std::uint8_t string) noexcept {
    return {8, descriptor::iad, first, count, cls, subclass, protocol, string};
}
}  // namespace detail

struct DeviceInfo {
    std::uint16_t vendor = 0x1209;   ///< pid.codes
    std::uint16_t product = 0x0001;  ///< pid.codes test PID: replace for a product
    std::uint16_t release = 0x0100;  ///< bcdDevice
    std::uint8_t manufacturer = 1;   ///< string indices (0: none)
    std::uint8_t product_name = 2;
    std::uint8_t serial = 3;
    std::uint8_t ep0_size = 64;
};

/// Device descriptor of a composite device with IADs (USB 2.0, full speed).
constexpr Descriptor<18> device_descriptor(const DeviceInfo& d) noexcept {
    using detail::hi, detail::lo;
    return {18,
            descriptor::device,
            lo(0x0200), hi(0x0200),  // bcdUSB 2.00
            0xEF, 0x02, 0x01,        // miscellaneous / common class / IAD
            d.ep0_size,
            lo(d.vendor), hi(d.vendor),
            lo(d.product), hi(d.product),
            lo(d.release), hi(d.release),
            d.manufacturer, d.product_name, d.serial,
            1};  // one configuration
}

struct ConfigurationInfo {
    std::uint16_t max_power_ma = 100;
    bool self_powered = false;
};

/// Configuration 1: header + the functions' descriptors.
template <std::size_t... Ns>
constexpr auto configuration_descriptor(const ConfigurationInfo& c, const Descriptor<Ns>&... functions) noexcept {
    auto all = detail::join(Descriptor<9>{}, functions...);
    std::uint8_t interfaces = 0;
    for (std::size_t i = 9; i < all.size(); i += all[i]) {
        if (all[i + 1] == descriptor::interface && all[i + 3] == 0) {  // alternate 0 counts once
            ++interfaces;
        }
    }
    const auto total = static_cast<std::uint16_t>(all.size());
    const Descriptor<9> header{9,
                               descriptor::configuration,
                               detail::lo(total), detail::hi(total),
                               interfaces,
                               1,  // bConfigurationValue
                               0,
                               static_cast<std::uint8_t>(0x80 | (c.self_powered ? 0x40 : 0)),
                               static_cast<std::uint8_t>(c.max_power_ma / 2)};
    std::copy(header.begin(), header.end(), all.begin());
    return all;
}

// ---- CDC (CDC 1.2, PSTN 1.2, NCM 1.0) ----------------------------------------------------

namespace cdc {
inline constexpr std::uint8_t communication = 0x02, data = 0x0A;
inline constexpr std::uint8_t subclass_acm = 0x02, subclass_ncm = 0x0D;
inline constexpr std::uint8_t protocol_ntb = 0x01;  ///< data interface of NCM
inline constexpr std::uint8_t header = 0x00, call_management = 0x01, acm = 0x02, union_ = 0x06,
                              ethernet_networking = 0x0F, ncm = 0x1A;  ///< functional descriptor subtypes
}  // namespace cdc

struct CdcAcmInfo {
    std::uint8_t interface = 0;      ///< communication interface; data = interface + 1
    std::uint8_t notify_ep = 0x81;   ///< interrupt IN
    std::uint8_t out_ep = 0x02;      ///< bulk OUT
    std::uint8_t in_ep = 0x82;       ///< bulk IN
    std::uint8_t string = 0;         ///< function name
    std::uint16_t packet = 64;
};

/// CDC-ACM function (IAD + 2 interfaces, 3 endpoints): 66 bytes.
constexpr Descriptor<66> cdc_acm_function(const CdcAcmInfo& a) noexcept {
    using namespace detail;
    const std::uint8_t c = a.interface, d = static_cast<std::uint8_t>(a.interface + 1);
    return join(iad(c, 2, cdc::communication, cdc::subclass_acm, 0, a.string),
                interface(c, 0, 1, cdc::communication, cdc::subclass_acm, 0, a.string),
                Descriptor<5>{5, descriptor::cs_interface, cdc::header, lo(0x0120), hi(0x0120)},
                Descriptor<5>{5, descriptor::cs_interface, cdc::call_management, 0x00, d},
                Descriptor<4>{4, descriptor::cs_interface, cdc::acm, 0x02},  // SET/GET_LINE_CODING, SET_CONTROL_LINE_STATE
                Descriptor<5>{5, descriptor::cs_interface, cdc::union_, c, d},
                endpoint(a.notify_ep, TransferType::Interrupt, 8, 16),
                interface(d, 0, 2, cdc::data, 0, 0, 0),
                endpoint(a.out_ep, TransferType::Bulk, a.packet, 0),
                endpoint(a.in_ep, TransferType::Bulk, a.packet, 0));
}

struct CdcNcmInfo {
    std::uint8_t interface = 0;     ///< communication interface; data = interface + 1
    std::uint8_t notify_ep = 0x81;  ///< interrupt IN (16 bytes: speed change notification)
    std::uint8_t out_ep = 0x02;
    std::uint8_t in_ep = 0x82;
    std::uint8_t string = 0;
    std::uint8_t mac_string = 0;    ///< iMACAddress: the HOST's MAC, 12 hex digits
    std::uint16_t packet = 64;
    std::uint16_t max_segment = 1514;
};

/// CDC-NCM function (IAD, communication interface, data interface with alternates
/// 0 (no endpoints) and 1 (bulk IN/OUT)): 85 bytes.
constexpr Descriptor<85> cdc_ncm_function(const CdcNcmInfo& n) noexcept {
    using namespace detail;
    const std::uint8_t c = n.interface, d = static_cast<std::uint8_t>(n.interface + 1);
    return join(iad(c, 2, cdc::communication, cdc::subclass_ncm, 0, n.string),
                interface(c, 0, 1, cdc::communication, cdc::subclass_ncm, 0, n.string),
                Descriptor<5>{5, descriptor::cs_interface, cdc::header, lo(0x0120), hi(0x0120)},
                Descriptor<5>{5, descriptor::cs_interface, cdc::union_, c, d},
                Descriptor<13>{13, descriptor::cs_interface, cdc::ethernet_networking, n.mac_string, 0, 0, 0, 0,
                               lo(n.max_segment), hi(n.max_segment), 0, 0, 0},
                Descriptor<6>{6, descriptor::cs_interface, cdc::ncm, lo(0x0100), hi(0x0100), 0x00},
                endpoint(n.notify_ep, TransferType::Interrupt, 16, 16),
                interface(d, 0, 0, cdc::data, 0, cdc::protocol_ntb, 0),
                interface(d, 1, 2, cdc::data, 0, cdc::protocol_ntb, 0),
                endpoint(n.in_ep, TransferType::Bulk, n.packet, 0),
                endpoint(n.out_ep, TransferType::Bulk, n.packet, 0));
}

}  // namespace rtos::usb
