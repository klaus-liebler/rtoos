// SPDX-License-Identifier: MIT
// AFL++ harness: Modbus - the PDU server (rtos/modbus/modbus.hpp: Modbus TCP, RTU, the FCU's USB
// CDC) and the RTU framing (rtos/modbus/rtu.hpp). Input: byte 0 bit 7 = RTU (else one PDU), bits
// 0-3 = the RTU segment sizes; the rest = the PDU or the RTU byte stream. In the RTU stream a byte
// 0xFF after a byte 0xFE is inter-frame silence (t3.5).
#include <array>

#include "fuzz_main.hpp"
#include "rtos/modbus/modbus.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/modbus/rtu.hpp"

namespace mb = rtos::modbus;

void fuzz_one(std::span<const std::uint8_t> input) {
    if (input.empty()) return;
    static mb::RegisterBank<100, 50> bank;
    const auto data = std::as_bytes(input.subspan(1));
    if ((input[0] & 0x80) == 0) {
        std::array<std::byte, mb::max_pdu> response{};
        const std::size_t len = mb::process_pdu(bank, data, response);
        if (len > mb::max_pdu || (!data.empty() && len < 2)) __builtin_trap();
        return;
    }
    mb::RtuServer rtu{bank, 1};
    const std::size_t segment = 1 + (input[0] & 0x0F);
    std::size_t start = 0;
    const auto feed = [&](std::span<const std::byte> part) {
        for (std::size_t pos = 0; pos < part.size(); pos += segment) {
            if (rtu.receive(part.subspan(pos, std::min(segment, part.size() - pos))).size() > mb::RtuServer::max_adu) __builtin_trap();
        }
        while (!rtu.receive({}).empty()) {
        }
        if (rtu.buffered() > mb::RtuServer::max_adu) __builtin_trap();
    };
    for (std::size_t i = 0; i + 1 < data.size(); ++i) {
        if (data[i] == std::byte{0xFE} && data[i + 1] == std::byte{0xFF}) {
            feed(data.subspan(start, i - start));
            if (rtu.silence().size() > mb::RtuServer::max_adu) __builtin_trap();
            start = i + 2;
            ++i;
        }
    }
    feed(data.subspan(start));
    if (rtu.silence().size() > mb::RtuServer::max_adu) __builtin_trap();
}
