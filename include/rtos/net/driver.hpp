// SPDX-License-Identifier: MIT
//
// Network interface driver concept. Implementations:
//   * rtos::port::stm32h5::Ethernet  (STM32H5 ETH, Synopsys DWC EQoS, RMII PHY)
//   * test::MockDriver               (host unit tests)
//
// All calls happen in the network thread; the driver's interrupt only wakes it.
#pragma once

#include <concepts>

#include "rtos/net/address.hpp"
#include "rtos/net/packet.hpp"

namespace rtos::net {

template <class D>
concept NetDriver = requires(D& d, PacketRef frame) {
    /// Station address.
    { d.mac() } noexcept -> std::same_as<MacAddress>;
    /// Physical link state.
    { d.link_up() } noexcept -> std::same_as<bool>;
    /// Queue a complete frame (Ethernet header at frame->data(), without FCS). The
    /// driver keeps its own reference until the DMA has sent it. false: TX ring full.
    { d.transmit(frame) } noexcept -> std::same_as<bool>;
    /// Next received frame (Ethernet header at data(), FCS stripped), or empty.
    { d.receive() } noexcept -> std::same_as<PacketRef>;
};

}  // namespace rtos::net
