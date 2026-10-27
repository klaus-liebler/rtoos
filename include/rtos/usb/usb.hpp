// SPDX-License-Identifier: MIT
//
// USB device stack: definitions (USB 2.0 chapter 9), the controller concept and the
// interface of a function (class driver).
//
//   DeviceController   the hardware port (STM32H5 USB_DRD_FS, a mock in the host
//                      tests): endpoints, transfers, stalls, address; its interrupt
//                      handler queues Events that the stack takes with poll()
//   Device             (device.hpp) enumeration, standard requests, control
//                      transfers; dispatches to the functions
//   Function           a class driver owning interfaces and endpoints: CDC-ACM
//                      (cdc_acm.hpp), CDC-NCM (cdc_ncm.hpp)
//
// Execution model: everything runs in one thread - the network thread (Device is a
// NetService) - which the USB interrupt wakes. No class has its own thread.
#pragma once

#include <array>
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rtos::usb {

namespace descriptor {
inline constexpr std::uint8_t device = 1, configuration = 2, string = 3, interface = 4, endpoint = 5,
                              device_qualifier = 6, other_speed = 7, iad = 11, cs_interface = 0x24;
}

namespace request {
inline constexpr std::uint8_t get_status = 0, clear_feature = 1, set_feature = 3, set_address = 5, get_descriptor = 6,
                              set_descriptor = 7, get_configuration = 8, set_configuration = 9, get_interface = 10,
                              set_interface = 11, synch_frame = 12;
}

namespace feature {
inline constexpr std::uint16_t endpoint_halt = 0, remote_wakeup = 1, test_mode = 2;
}

enum class TransferType : std::uint8_t { Control = 0, Isochronous = 1, Bulk = 2, Interrupt = 3 };

[[nodiscard]] constexpr bool is_in(std::uint8_t endpoint) noexcept { return (endpoint & 0x80) != 0; }
[[nodiscard]] constexpr std::uint8_t endpoint_number(std::uint8_t endpoint) noexcept { return endpoint & 0x0F; }

/// The 8-byte SETUP packet.
struct Setup {
    std::uint8_t request_type = 0;
    std::uint8_t request = 0;
    std::uint16_t value = 0;
    std::uint16_t index = 0;
    std::uint16_t length = 0;

    enum class Type : std::uint8_t { Standard = 0, Class = 1, Vendor = 2, Reserved = 3 };
    enum class Recipient : std::uint8_t { Device = 0, Interface = 1, Endpoint = 2, Other = 3 };

    [[nodiscard]] static constexpr Setup parse(std::span<const std::byte, 8> b) noexcept {
        const auto u8 = [&](std::size_t i) { return std::to_integer<std::uint8_t>(b[i]); };
        return {u8(0), u8(1), static_cast<std::uint16_t>(u8(2) | (u8(3) << 8)),
                static_cast<std::uint16_t>(u8(4) | (u8(5) << 8)), static_cast<std::uint16_t>(u8(6) | (u8(7) << 8))};
    }
    [[nodiscard]] constexpr bool device_to_host() const noexcept { return (request_type & 0x80) != 0; }
    [[nodiscard]] constexpr Type type() const noexcept { return static_cast<Type>((request_type >> 5) & 3); }
    [[nodiscard]] constexpr Recipient recipient() const noexcept {
        const auto r = request_type & 0x1F;
        return r <= 3 ? static_cast<Recipient>(r) : Recipient::Other;
    }
};

struct EndpointConfig {
    std::uint8_t address = 0;  ///< with direction bit (0x80 = IN)
    TransferType type = TransferType::Bulk;
    std::uint16_t max_packet = 64;
};

enum class EventType : std::uint8_t {
    BusReset,          ///< USB reset: back to the default state, address 0
    Setup,             ///< SETUP packet on EP0 (`setup`)
    TransferComplete,  ///< a transfer of `endpoint` finished, `length` bytes moved
    Suspend,
    Resume,
};

struct Event {
    EventType type = EventType::BusReset;
    std::uint8_t endpoint = 0;
    std::uint16_t length = 0;
    std::array<std::byte, 8> setup{};
};

/// The device controller port. All calls from the stack's thread; the controller's
/// interrupt handler only queues events (taken with poll()).
template <class D>
concept DeviceController = requires(D& d, const EndpointConfig& config, std::uint8_t ep, std::span<std::byte> buffer,
                                    bool zlp) {
    /// Next event from the interrupt handler, if any.
    { d.poll() } noexcept -> std::same_as<std::optional<Event>>;
    /// Configure and enable an endpoint (not EP0, which is always open).
    { d.open(config) } noexcept -> std::same_as<bool>;
    /// Disable every endpoint but EP0 (reset, new configuration), cancelling transfers.
    { d.close_all() } noexcept;
    /// Start a transfer on `ep`. IN: send `buffer` (and a zero-length packet after it if
    /// `zlp` and its size is a multiple of the packet size, or if it is empty); OUT:
    /// receive up to buffer.size() bytes (ends early with a short packet). The buffer
    /// stays in use until the TransferComplete event. False if `ep` is busy or closed.
    { d.transfer(ep, buffer, zlp) } noexcept -> std::same_as<bool>;
    /// Halt / un-halt `ep` (EP0: protocol stall of both directions until the next SETUP).
    { d.stall(ep) } noexcept;
    { d.clear_stall(ep) } noexcept;  // also resets the data toggle
    /// Take the new address (called after the status stage of SET_ADDRESS).
    { d.set_address(ep) } noexcept;
    /// Pull-up on / off.
    { d.connect() } noexcept;
    { d.disconnect() } noexcept;
};

/// What a function may do with the controller: its endpoints only (type-erased, so
/// class drivers do not depend on the controller type).
class Endpoints {
public:
    template <DeviceController D>
    explicit Endpoints(D& d) noexcept
        : self_(&d),
          open_([](void* s, const EndpointConfig& c) noexcept { return static_cast<D*>(s)->open(c); }),
          transfer_([](void* s, std::uint8_t ep, std::span<std::byte> b, bool zlp) noexcept {
              return static_cast<D*>(s)->transfer(ep, b, zlp);
          }),
          stall_([](void* s, std::uint8_t ep) noexcept { static_cast<D*>(s)->stall(ep); }) {}

    bool open(const EndpointConfig& config) noexcept { return open_(self_, config); }
    bool transfer(std::uint8_t ep, std::span<std::byte> buffer, bool zlp = false) noexcept {
        return transfer_(self_, ep, buffer, zlp);
    }
    void stall(std::uint8_t ep) noexcept { stall_(self_, ep); }

private:
    void* self_;
    bool (*open_)(void*, const EndpointConfig&) noexcept;
    bool (*transfer_)(void*, std::uint8_t, std::span<std::byte>, bool) noexcept;
    void (*stall_)(void*, std::uint8_t) noexcept;
};

/// Answer of a function to a class or vendor SETUP.
enum class ControlResult : std::uint8_t {
    Stall,   ///< not supported: request error
    Accept,  ///< IN: `in_length` bytes are in the buffer; OUT: control_data() follows
};

/// A class driver: owns interfaces first_interface() .. +interface_count()-1 and the
/// endpoints for which owns_endpoint() is true.
class Function {
public:
    [[nodiscard]] virtual std::uint8_t first_interface() const noexcept = 0;
    [[nodiscard]] virtual std::uint8_t interface_count() const noexcept = 0;
    [[nodiscard]] virtual bool owns_endpoint(std::uint8_t ep) const noexcept = 0;

    /// SET_CONFIGURATION: open the endpoints, start reception.
    virtual void configured(Endpoints& eps) noexcept = 0;
    /// Bus reset or configuration 0: everything is gone (transfers cancelled).
    virtual void reset() noexcept = 0;

    /// SET_INTERFACE; false: no such alternate setting (request error).
    virtual bool set_interface(Endpoints&, std::uint8_t /*interface*/, std::uint8_t alternate) noexcept {
        return alternate == 0;
    }
    [[nodiscard]] virtual std::uint8_t interface_alternate(std::uint8_t /*interface*/) const noexcept { return 0; }

    /// Class/vendor request for one of its interfaces or endpoints. IN requests put
    /// at most buffer.size() bytes into `buffer` and set `in_length`.
    virtual ControlResult control_setup(const Setup&, std::span<std::byte> /*buffer*/, std::size_t& /*in_length*/) noexcept {
        return ControlResult::Stall;
    }
    /// Data stage of an accepted OUT request (at most setup.length bytes).
    virtual void control_data(const Setup&, std::span<const std::byte>) noexcept {}

    /// A transfer started with Endpoints::transfer() finished.
    virtual void transfer_complete(Endpoints& eps, std::uint8_t ep, std::size_t length) noexcept = 0;
    /// Every pass while configured: start transfers for queued data.
    virtual void service(Endpoints&) noexcept {}

protected:
    ~Function() = default;
};

}  // namespace rtos::usb
