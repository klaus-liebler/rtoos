// SPDX-License-Identifier: MIT
//
// RegisterBank: register storage for Modbus (and OPC UA, which maps the same
// registers): Holding holding registers, Input input registers, shared by the
// network thread and application threads (I/O, control loops).
//
// Consistency: every access - a protocol request (a block of up to 125 registers)
// or the application's get/set/read/write - runs under one `Guard`, so a request
// never sees half of a multi-register value that a control loop is updating. The
// guard is an RAII type:
//     target: rtos::Kernel<Port>::Lock (BASEPRI; copying 125 registers takes well
//             under a microsecond at 250 MHz)
//     single-threaded use and host tests: NoGuard
//
// on_write (optional) runs after a protocol write, outside the guard, in the
// protocol's thread (e.g. wake the thread that acts on a new set point).
//
// A WriteFilter (optional) decides under the guard whether a protocol write may
// change the registers at all, e.g. while a PLC program owns the outputs: a
// refused block changes no register (no partial write) and the protocol answers
// IllegalDataAddress (Modbus exception 0x02, OPC UA Bad_NotWritable). The filter
// runs with the guard held (BASEPRI on the target): short, no kernel calls. The
// application side (set_holding, ...) is never filtered.
#pragma once

#include <algorithm>
#include <array>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/modbus/modbus.hpp"

namespace rtos::modbus {

struct NoGuard {};

struct WriteHook {
    void* context = nullptr;
    void (*written)(void* context, std::uint16_t start, std::uint16_t count) noexcept = nullptr;
};

struct WriteFilter {
    void* context = nullptr;
    /// May a protocol write the holding registers start .. start+count-1?
    bool (*permit)(void* context, std::uint16_t start, std::uint16_t count) noexcept = nullptr;
};

template <std::size_t Holding, std::size_t Input, class Guard = NoGuard>
class RegisterBank final : public RegisterMap {
    static_assert(Holding <= 65536 && Input <= 65536);

public:
    RegisterBank() noexcept = default;
    explicit RegisterBank(WriteHook hook) noexcept : hook_(hook) {}

    /// Installs (or with {} removes) the protocol write filter; at any time.
    void set_write_filter(WriteFilter filter) noexcept {
        [[maybe_unused]] const Guard guard{};
        filter_ = filter;
    }
    RegisterBank(const RegisterBank&) = delete;
    RegisterBank& operator=(const RegisterBank&) = delete;

    // ---- application side --------------------------------------------------------

    [[nodiscard]] std::uint16_t holding(std::uint16_t address) const noexcept {
        contract_assert(address < Holding);
        [[maybe_unused]] const Guard guard{};
        return holding_[address];
    }
    [[nodiscard]] std::uint16_t input(std::uint16_t address) const noexcept {
        contract_assert(address < Input);
        [[maybe_unused]] const Guard guard{};
        return input_[address];
    }
    void set_holding(std::uint16_t address, std::uint16_t value) noexcept {
        contract_assert(address < Holding);
        [[maybe_unused]] const Guard guard{};
        holding_[address] = value;
    }
    void set_input(std::uint16_t address, std::uint16_t value) noexcept {
        contract_assert(address < Input);
        [[maybe_unused]] const Guard guard{};
        input_[address] = value;
    }
    /// Consistent block update/read (e.g. a 32-bit value in two registers).
    void set_inputs(std::uint16_t start, std::span<const std::uint16_t> values) noexcept {
        contract_assert(start + values.size() <= Input);
        [[maybe_unused]] const Guard guard{};
        std::ranges::copy(values, input_.begin() + start);
    }
    void set_holdings(std::uint16_t start, std::span<const std::uint16_t> values) noexcept {
        contract_assert(start + values.size() <= Holding);
        [[maybe_unused]] const Guard guard{};
        std::ranges::copy(values, holding_.begin() + start);
    }
    void read_holdings(std::uint16_t start, std::span<std::uint16_t> out) const noexcept {
        contract_assert(start + out.size() <= Holding);
        [[maybe_unused]] const Guard guard{};
        std::copy_n(holding_.begin() + start, out.size(), out.begin());
    }
    void read_inputs(std::uint16_t start, std::span<std::uint16_t> out) const noexcept {
        contract_assert(start + out.size() <= Input);
        [[maybe_unused]] const Guard guard{};
        std::copy_n(input_.begin() + start, out.size(), out.begin());
    }

    // ---- RegisterMap (protocol side) ------------------------------------------------

    [[nodiscard]] std::uint16_t size(Table table) const noexcept override {
        return static_cast<std::uint16_t>(table == Table::Holding ? Holding : Input);
    }

    Exception read(Table table, std::uint16_t start, std::span<std::uint16_t> out) noexcept override {
        [[maybe_unused]] const Guard guard{};
        const auto& regs = table == Table::Holding ? std::span<const std::uint16_t>(holding_)
                                                   : std::span<const std::uint16_t>(input_);
        std::copy_n(regs.begin() + start, out.size(), out.begin());
        return Exception::None;
    }

    Exception write(std::uint16_t start, std::span<const std::uint16_t> values) noexcept override {
        {
            [[maybe_unused]] const Guard guard{};
            if (filter_.permit != nullptr &&
                !filter_.permit(filter_.context, start, static_cast<std::uint16_t>(values.size()))) {
                return Exception::IllegalDataAddress;
            }
            std::ranges::copy(values, holding_.begin() + start);
        }
        if (hook_.written != nullptr) {
            hook_.written(hook_.context, start, static_cast<std::uint16_t>(values.size()));
        }
        return Exception::None;
    }

private:
    std::array<std::uint16_t, Holding> holding_{};
    std::array<std::uint16_t, Input> input_{};
    WriteHook hook_{};
    WriteFilter filter_{};
};

}  // namespace rtos::modbus
