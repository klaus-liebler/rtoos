// SPDX-License-Identifier: MIT
//
// Modbus application protocol (Modbus Application Protocol Specification V1.1b3):
// the PDU - function code + data - shared by Modbus TCP (modbus_tcp_server.hpp) and
// Modbus RTU (rtu.hpp). Server side.
//
// Function codes: 03 Read Holding Registers, 04 Read Input Registers, 06 Write
// Single Register, 16 Write Multiple Registers. Others answer exception 01.
// Checks in the order of the specification's state diagrams (6.3 - 6.12): quantity
// (exception 03), then address range (02), then the register map (04 on failure).
//
// RegisterMap is the application's register storage (register_bank.hpp, or generated
// code). Requests reach it as whole blocks, so it can keep a multi-register value
// consistent (one lock per request, not per register).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "rtos/net/bytes.hpp"

namespace rtos::modbus {

using net::load_be16;
using net::store_be16;

enum class Exception : std::uint8_t {
    None = 0,
    IllegalFunction = 1,
    IllegalDataAddress = 2,
    IllegalDataValue = 3,
    ServerDeviceFailure = 4,
};

enum class Table : std::uint8_t { Holding, Input };

namespace function {
inline constexpr std::uint8_t read_holding = 0x03;
inline constexpr std::uint8_t read_input = 0x04;
inline constexpr std::uint8_t write_single = 0x06;
inline constexpr std::uint8_t write_multiple = 0x10;
}  // namespace function

inline constexpr std::size_t max_pdu = 253;
inline constexpr std::uint16_t max_read = 125;   ///< registers per read (spec 6.3/6.4)
inline constexpr std::uint16_t max_write = 123;  ///< registers per write multiple (6.12)

/// The application's registers. Called from the protocol's thread (the network
/// thread for Modbus TCP); implementations synchronise with other threads.
class RegisterMap {
public:
    /// Number of registers in `table` (addresses 0 .. size-1).
    [[nodiscard]] virtual std::uint16_t size(Table table) const noexcept = 0;
    /// Copy registers start .. start+out.size()-1 (range already checked).
    virtual Exception read(Table table, std::uint16_t start, std::span<std::uint16_t> out) noexcept = 0;
    /// Write holding registers start .. start+values.size()-1 (range already checked).
    virtual Exception write(std::uint16_t start, std::span<const std::uint16_t> values) noexcept = 0;

protected:
    ~RegisterMap() = default;
};

/// Answer request PDU `request` (function code first) into `response` (at least
/// max_pdu bytes). Returns the response PDU length (an exception response is a
/// response too).
inline std::size_t process_pdu(RegisterMap& map, std::span<const std::byte> request,
                               std::span<std::byte> response) noexcept {
    if (request.empty() || response.size() < max_pdu) {
        return 0;
    }
    const auto fc = std::to_integer<std::uint8_t>(request[0]);
    const auto exception = [&](Exception e) noexcept -> std::size_t {
        response[0] = static_cast<std::byte>(fc | 0x80);
        response[1] = static_cast<std::byte>(e);
        return 2;
    };
    switch (fc) {
        case function::read_holding:
        case function::read_input: {
            if (request.size() != 5) {
                return exception(Exception::IllegalDataValue);
            }
            const std::uint16_t start = load_be16(&request[1]);
            const std::uint16_t count = load_be16(&request[3]);
            if (count == 0 || count > max_read) {
                return exception(Exception::IllegalDataValue);
            }
            const Table table = fc == function::read_holding ? Table::Holding : Table::Input;
            if (std::uint32_t{start} + count > map.size(table)) {
                return exception(Exception::IllegalDataAddress);
            }
            std::array<std::uint16_t, max_read> values{};
            if (const Exception e = map.read(table, start, {values.data(), count}); e != Exception::None) {
                return exception(e);
            }
            response[0] = static_cast<std::byte>(fc);
            response[1] = static_cast<std::byte>(count * 2);
            for (std::uint16_t i = 0; i < count; ++i) {
                store_be16(&response[2 + 2 * i], values[i]);
            }
            return 2 + 2 * std::size_t{count};
        }
        case function::write_single: {
            if (request.size() != 5) {
                return exception(Exception::IllegalDataValue);
            }
            const std::uint16_t address = load_be16(&request[1]);
            if (address >= map.size(Table::Holding)) {
                return exception(Exception::IllegalDataAddress);
            }
            const std::uint16_t value = load_be16(&request[3]);
            if (const Exception e = map.write(address, {&value, 1}); e != Exception::None) {
                return exception(e);
            }
            std::copy(request.begin(), request.end(), response.begin());  // echo
            return 5;
        }
        case function::write_multiple: {
            if (request.size() < 6) {
                return exception(Exception::IllegalDataValue);
            }
            const std::uint16_t start = load_be16(&request[1]);
            const std::uint16_t count = load_be16(&request[3]);
            const auto bytes = std::to_integer<std::size_t>(request[5]);
            if (count == 0 || count > max_write || bytes != 2 * std::size_t{count} || request.size() != 6 + bytes) {
                return exception(Exception::IllegalDataValue);
            }
            if (std::uint32_t{start} + count > map.size(Table::Holding)) {
                return exception(Exception::IllegalDataAddress);
            }
            std::array<std::uint16_t, max_write> values{};
            for (std::uint16_t i = 0; i < count; ++i) {
                values[i] = load_be16(&request[6 + 2 * i]);
            }
            if (const Exception e = map.write(start, {values.data(), count}); e != Exception::None) {
                return exception(e);
            }
            std::copy_n(request.begin(), 5, response.begin());  // fc, start, quantity
            return 5;
        }
        default: return exception(Exception::IllegalFunction);
    }
}

/// Requests that only write (no data to report): executed for broadcasts (RTU address 0).
[[nodiscard]] constexpr bool is_write(std::uint8_t fc) noexcept {
    return fc == function::write_single || fc == function::write_multiple;
}

/// CRC-16/MODBUS (poly 0xA001 reflected, init 0xFFFF); sent low byte first.
[[nodiscard]] constexpr std::uint16_t crc16(std::span<const std::byte> data) noexcept {
    std::uint16_t crc = 0xFFFF;
    for (const std::byte b : data) {
        crc ^= std::to_integer<std::uint16_t>(b);
        for (int i = 0; i < 8; ++i) {
            crc = (crc & 1) ? static_cast<std::uint16_t>((crc >> 1) ^ 0xA001) : static_cast<std::uint16_t>(crc >> 1);
        }
    }
    return crc;
}

}  // namespace rtos::modbus
