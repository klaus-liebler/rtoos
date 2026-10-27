// SPDX-License-Identifier: MIT
//
// usb::Device: the device side of USB 2.0 chapter 9 on any DeviceController.
//
//   * states Default -> Address -> Configured (bus reset back to Default)
//   * standard requests: GET_DESCRIPTOR (device, configuration, string; a full-speed
//     device stalls DEVICE_QUALIFIER / OTHER_SPEED, 9.6.2), SET_ADDRESS (applied
//     after its status stage), SET/GET_CONFIGURATION, GET_STATUS, CLEAR/SET_FEATURE
//     (endpoint halt, remote wakeup), SET/GET_INTERFACE; class and vendor requests
//     go to the function owning the interface (wIndex) or endpoint
//   * control transfers: IN data clamped to wLength with a zero-length packet when a
//     shorter answer ends on a packet boundary; OUT data longer than the buffer is a
//     request error; unsupported requests stall EP0 until the next SETUP
//   * transfer completions of other endpoints go to the function owning them
//
// Device is a NetService: it runs in the network thread, woken by the USB interrupt
// (the controller queues events; service() drains them).
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

#include "rtos/usb/usb.hpp"

namespace rtos::usb {

struct DeviceDescriptors {
    std::span<const std::uint8_t> device;         ///< 18 bytes
    std::span<const std::uint8_t> configuration;  ///< the whole configuration 1
    std::span<const std::string_view> strings;    ///< string 1, 2, ... (ASCII)
    std::uint16_t language = 0x0409;              ///< string 0: English (US)
};

enum class DeviceState : std::uint8_t { Default, Address, Configured };

struct DeviceStats {
    std::uint32_t resets = 0;
    std::uint32_t setups = 0;
    std::uint32_t stalls = 0;  ///< request errors (EP0 stalled)
};

template <DeviceController Controller, std::size_t ControlBuffer = 256>
class Device {
public:
    Device(Controller& controller, const DeviceDescriptors& descriptors, std::span<Function* const> functions) noexcept
        : dcd_(controller), endpoints_(controller), descriptors_(descriptors), functions_(functions) {}

    Device(const Device&) = delete;
    Device& operator=(const Device&) = delete;

    void connect() noexcept { dcd_.connect(); }
    void disconnect() noexcept {
        dcd_.disconnect();
        reset();
    }

    // ---- NetService --------------------------------------------------------------
    void service() noexcept {
        while (const auto event = dcd_.poll()) {
            handle(*event);
        }
        if (state_ == DeviceState::Configured) {
            for (Function* f : functions_) {
                f->service(endpoints_);
            }
        }
    }
    [[nodiscard]] std::uint32_t next_deadline() const noexcept { return 0; }  // interrupt driven

    [[nodiscard]] DeviceState state() const noexcept { return state_; }
    [[nodiscard]] bool configured() const noexcept { return state_ == DeviceState::Configured; }
    [[nodiscard]] bool suspended() const noexcept { return suspended_; }
    [[nodiscard]] std::uint8_t address() const noexcept { return address_; }
    [[nodiscard]] const DeviceStats& stats() const noexcept { return stats_; }

private:
    enum class Control : std::uint8_t { Idle, DataIn, StatusOut, DataOut, StatusIn };

    void handle(const Event& e) noexcept {
        switch (e.type) {
            case EventType::BusReset:
                ++stats_.resets;
                reset();
                break;
            case EventType::Setup: setup(Setup::parse(e.setup)); break;
            case EventType::TransferComplete:
                if (endpoint_number(e.endpoint) == 0) {
                    control_complete(e.endpoint, e.length);
                } else if (Function* f = owner_of_endpoint(e.endpoint)) {
                    f->transfer_complete(endpoints_, e.endpoint, e.length);
                }
                break;
            case EventType::Suspend: suspended_ = true; break;
            case EventType::Resume: suspended_ = false; break;
        }
    }

    void reset() noexcept {
        dcd_.close_all();
        for (Function* f : functions_) {
            f->reset();
        }
        state_ = DeviceState::Default;
        address_ = 0;
        pending_address_ = -1;
        control_ = Control::Idle;
        halted_ = 0;
        remote_wakeup_ = suspended_ = false;
    }

    // ---- control transfers -----------------------------------------------------------

    void setup(const Setup& s) noexcept {
        ++stats_.setups;
        setup_ = s;
        control_ = Control::Idle;  // a SETUP aborts whatever control transfer was running
        pending_address_ = -1;
        switch (s.type()) {
            case Setup::Type::Standard: standard(s); return;
            case Setup::Type::Class:
            case Setup::Type::Vendor: {
                Function* f = s.recipient() == Setup::Recipient::Interface ? owner_of_interface(s.index & 0xFF)
                              : s.recipient() == Setup::Recipient::Endpoint ? owner_of_endpoint(s.index & 0xFF)
                                                                             : nullptr;
                if (f == nullptr || state_ != DeviceState::Configured) {
                    stall();
                    return;
                }
                std::size_t in_length = 0;
                if (f->control_setup(s, buffer(s), in_length) == ControlResult::Stall) {
                    stall();
                    return;
                }
                data_owner_ = f;
                s.device_to_host() ? send(in_length) : receive_or_ack();
                return;
            }
            case Setup::Type::Reserved: stall(); return;
        }
    }

    /// Buffer for the data stage: never more than the request asks for.
    std::span<std::byte> buffer(const Setup& s) noexcept {
        return std::span(buffer_).first(std::min<std::size_t>(s.length, buffer_.size()));
    }

    void send(std::span<const std::uint8_t> data) noexcept {
        const std::size_t n = std::min<std::size_t>({data.size(), setup_.length, buffer_.size()});
        std::transform(data.begin(), data.begin() + static_cast<std::ptrdiff_t>(n), buffer_.begin(),
                       [](std::uint8_t b) { return std::byte{b}; });
        send(n);
    }

    /// IN data stage of `length` bytes from buffer_ (zero-length packet if a shorter
    /// answer than requested ends on a packet boundary).
    void send(std::size_t length) noexcept {
        if (setup_.length == 0) {
            status_in();  // wLength 0: no data stage, the status stage is IN (8.5.3)
            return;
        }
        length = std::min<std::size_t>(length, setup_.length);
        const bool zlp = length < setup_.length;
        control_ = Control::DataIn;
        dcd_.transfer(0x80, std::span(buffer_).first(length), zlp);
    }

    /// No data stage: status IN now. OUT data stage: receive it first.
    void receive_or_ack() noexcept {
        if (setup_.length == 0) {
            status_in();
            return;
        }
        if (setup_.length > buffer_.size()) {
            stall();
            return;
        }
        control_ = Control::DataOut;
        dcd_.transfer(0x00, std::span(buffer_).first(setup_.length), false);
    }

    void status_in() noexcept {
        control_ = Control::StatusIn;
        dcd_.transfer(0x80, {}, false);
    }

    void stall() noexcept {
        ++stats_.stalls;
        control_ = Control::Idle;
        data_owner_ = nullptr;
        dcd_.stall(0x80);
        dcd_.stall(0x00);
    }

    void control_complete(std::uint8_t ep, std::size_t length) noexcept {
        switch (control_) {
            case Control::DataIn:
                if (is_in(ep)) {
                    control_ = Control::StatusOut;
                    dcd_.transfer(0x00, {}, false);
                }
                break;
            case Control::StatusOut: control_ = Control::Idle; break;
            case Control::DataOut:
                if (!is_in(ep)) {
                    if (data_owner_ != nullptr) {
                        data_owner_->control_data(
                            setup_, std::span<const std::byte>(buffer_).first(std::min<std::size_t>(length, setup_.length)));
                    }
                    status_in();
                }
                break;
            case Control::StatusIn:
                control_ = Control::Idle;
                if (pending_address_ >= 0) {
                    address_ = static_cast<std::uint8_t>(pending_address_);
                    dcd_.set_address(address_);
                    state_ = address_ != 0 ? DeviceState::Address : DeviceState::Default;
                    pending_address_ = -1;
                }
                break;
            case Control::Idle: break;
        }
    }

    // ---- standard requests (9.4) --------------------------------------------------------

    void standard(const Setup& s) noexcept {
        data_owner_ = nullptr;
        // 9.4: GET_* requests are IN, all others OUT without data; anything else is a
        // request error.
        const bool get = s.request == request::get_status || s.request == request::get_descriptor ||
                         s.request == request::get_configuration || s.request == request::get_interface ||
                         s.request == request::synch_frame;
        if (s.device_to_host() != get || (!get && s.length != 0)) {
            stall();
            return;
        }
        switch (s.recipient()) {
            case Setup::Recipient::Device: standard_device(s); return;
            case Setup::Recipient::Interface: standard_interface(s); return;
            case Setup::Recipient::Endpoint: standard_endpoint(s); return;
            case Setup::Recipient::Other: stall(); return;
        }
    }

    void standard_device(const Setup& s) noexcept {
        switch (s.request) {
            case request::get_descriptor: get_descriptor(s); return;
            case request::set_address:
                if (s.value > 127 || s.index != 0 || s.length != 0 || state_ == DeviceState::Configured) {
                    stall();
                    return;
                }
                pending_address_ = s.value;  // after the status stage (9.4.6)
                status_in();
                return;
            case request::get_configuration: {
                const std::uint8_t v = state_ == DeviceState::Configured ? 1 : 0;
                send(std::span(&v, 1));
                return;
            }
            case request::set_configuration: set_configuration(s); return;
            case request::get_status: {
                const std::uint8_t status[2] = {
                    static_cast<std::uint8_t>((self_powered() ? 1 : 0) | (remote_wakeup_ ? 2 : 0)), 0};
                send(status);
                return;
            }
            case request::set_feature:
            case request::clear_feature:
                if (s.value != feature::remote_wakeup) {
                    stall();  // TEST_MODE is for high-speed devices
                    return;
                }
                remote_wakeup_ = s.request == request::set_feature;
                status_in();
                return;
            default: stall(); return;
        }
    }

    void get_descriptor(const Setup& s) noexcept {
        const auto type = static_cast<std::uint8_t>(s.value >> 8);
        const auto index = static_cast<std::uint8_t>(s.value & 0xFF);
        switch (type) {
            case descriptor::device: send(descriptors_.device); return;
            case descriptor::configuration:
                if (index != 0) {
                    stall();
                    return;
                }
                send(descriptors_.configuration);
                return;
            case descriptor::string: send_string(index); return;
            default: stall(); return;  // DEVICE_QUALIFIER, OTHER_SPEED: full-speed only (9.6.2)
        }
    }

    void send_string(std::uint8_t index) noexcept {
        if (index == 0) {
            const std::uint8_t languages[4] = {4, descriptor::string, static_cast<std::uint8_t>(descriptors_.language),
                                               static_cast<std::uint8_t>(descriptors_.language >> 8)};
            send(languages);
            return;
        }
        if (index > descriptors_.strings.size()) {
            stall();
            return;
        }
        const std::string_view text = descriptors_.strings[index - 1];
        const std::size_t chars = std::min<std::size_t>(text.size(), (std::min<std::size_t>(buffer_.size(), 255) - 2) / 2);
        buffer_[0] = static_cast<std::byte>(2 + 2 * chars);
        buffer_[1] = static_cast<std::byte>(descriptor::string);
        for (std::size_t i = 0; i < chars; ++i) {  // ASCII -> UTF-16LE
            buffer_[2 + 2 * i] = static_cast<std::byte>(text[i]);
            buffer_[3 + 2 * i] = std::byte{0};
        }
        send(2 + 2 * chars);
    }

    void set_configuration(const Setup& s) noexcept {
        if (state_ == DeviceState::Default || (s.value != 0 && s.value != configuration_value())) {
            stall();
            return;
        }
        dcd_.close_all();
        for (Function* f : functions_) {
            f->reset();
        }
        halted_ = 0;
        if (s.value == 0) {
            state_ = DeviceState::Address;
        } else {
            state_ = DeviceState::Configured;
            for (Function* f : functions_) {
                f->configured(endpoints_);
            }
        }
        status_in();
    }

    void standard_interface(const Setup& s) noexcept {
        const auto itf = static_cast<std::uint8_t>(s.index & 0xFF);
        Function* f = owner_of_interface(itf);
        if (f == nullptr || state_ != DeviceState::Configured) {
            stall();
            return;
        }
        switch (s.request) {
            case request::get_status: {
                const std::uint8_t status[2] = {0, 0};
                send(status);
                return;
            }
            case request::get_interface: {
                const std::uint8_t alt = f->interface_alternate(itf);
                send(std::span(&alt, 1));
                return;
            }
            case request::set_interface:
                if (!f->set_interface(endpoints_, itf, static_cast<std::uint8_t>(s.value))) {
                    stall();
                    return;
                }
                status_in();
                return;
            default: stall(); return;
        }
    }

    void standard_endpoint(const Setup& s) noexcept {
        const auto ep = static_cast<std::uint8_t>(s.index & 0x8F);
        const bool valid = endpoint_number(ep) == 0 ||
                           (state_ == DeviceState::Configured && owner_of_endpoint(ep) != nullptr);
        if (!valid) {
            stall();
            return;
        }
        switch (s.request) {
            case request::get_status: {
                const std::uint8_t status[2] = {static_cast<std::uint8_t>(halted(ep) ? 1 : 0), 0};
                send(status);
                return;
            }
            case request::clear_feature:
            case request::set_feature:
                if (s.value != feature::endpoint_halt) {
                    stall();
                    return;
                }
                if (endpoint_number(ep) != 0) {
                    const bool halt = s.request == request::set_feature;
                    halt ? dcd_.stall(ep) : dcd_.clear_stall(ep);
                    set_halted(ep, halt);
                }
                status_in();
                return;
            default: stall(); return;
        }
    }

    // ---- helpers ----------------------------------------------------------------------

    [[nodiscard]] std::uint8_t configuration_value() const noexcept {
        return descriptors_.configuration.size() > 5 ? descriptors_.configuration[5] : 1;
    }
    [[nodiscard]] bool self_powered() const noexcept {
        return descriptors_.configuration.size() > 7 && (descriptors_.configuration[7] & 0x40) != 0;
    }

    Function* owner_of_interface(std::uint8_t itf) const noexcept {
        for (Function* f : functions_) {
            if (itf >= f->first_interface() && itf < f->first_interface() + f->interface_count()) {
                return f;
            }
        }
        return nullptr;
    }
    Function* owner_of_endpoint(std::uint8_t ep) const noexcept {
        for (Function* f : functions_) {
            if (f->owns_endpoint(ep)) {
                return f;
            }
        }
        return nullptr;
    }

    [[nodiscard]] static std::uint32_t halt_bit(std::uint8_t ep) noexcept {
        return 1u << (endpoint_number(ep) + (is_in(ep) ? 16 : 0));
    }
    [[nodiscard]] bool halted(std::uint8_t ep) const noexcept { return (halted_ & halt_bit(ep)) != 0; }
    void set_halted(std::uint8_t ep, bool on) noexcept { on ? halted_ |= halt_bit(ep) : halted_ &= ~halt_bit(ep); }

    Controller& dcd_;
    Endpoints endpoints_;
    DeviceDescriptors descriptors_;
    std::span<Function* const> functions_;
    DeviceState state_ = DeviceState::Default;
    std::uint8_t address_ = 0;
    int pending_address_ = -1;
    bool remote_wakeup_ = false;
    bool suspended_ = false;
    std::uint32_t halted_ = 0;
    Control control_ = Control::Idle;
    Setup setup_{};
    Function* data_owner_ = nullptr;
    alignas(4) std::array<std::byte, ControlBuffer> buffer_{};
    DeviceStats stats_{};
};

}  // namespace rtos::usb
