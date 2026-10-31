// SPDX-License-Identifier: MIT
//
// Feetech STS/SCS serial bus servos (e.g. ST3215 in the Waveshare RoArm-M3): packets
// FF FF <id> <length> <instruction> <parameters> <checksum = ~(id + length + instruction +
// parameters)>, memory table of the STS series (Feetech "STS memory table").
//
//   rtos::devices::FeetechBus<Uart, Clock> bus{uart};      // 1 Mbit/s, no echo on the line
//   bus.sync_write_positions(ids, positions, speeds, accel);
//   if (auto f = bus.feedback(11)) { f->position ... }
//
// `Uart`: a byte channel (read/write, BufferedUart); `Clock`: static now_ms() and sleep_ms().
// Ported from the FCU firmware's libs/hw/feetech_sts.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rtos::devices {

namespace feetech {
enum class Instruction : std::uint8_t { Ping = 0x01, Read = 0x02, Write = 0x03, SyncWrite = 0x83 };
inline constexpr std::uint8_t broadcast = 0xFE;
// memory table (STS)
inline constexpr std::uint8_t torque_enable = 40, acceleration = 41, goal_position = 42, lock = 55, present_position = 56;
inline constexpr std::uint8_t feedback_length = 15;  ///< present position (56) .. present current (69, 70)

/// Sign-magnitude 16-bit values (bit 15 = negative), as the STS series encodes them.
[[nodiscard]] constexpr std::uint16_t encode(std::int16_t v) noexcept {
    return v >= 0 ? static_cast<std::uint16_t>(v) : static_cast<std::uint16_t>(-static_cast<std::int32_t>(v) | 0x8000);
}
[[nodiscard]] constexpr std::int16_t decode(std::uint8_t lo, std::uint8_t hi) noexcept {
    const std::uint16_t raw = static_cast<std::uint16_t>(lo | (hi << 8));
    const auto magnitude = static_cast<std::int16_t>(raw & 0x7FFF);
    return (raw & 0x8000) != 0 ? static_cast<std::int16_t>(-magnitude) : magnitude;
}

inline constexpr std::size_t max_packet = 128;

/// One packet into `out`; returns its length (0: too long).
constexpr std::size_t packet(std::span<std::uint8_t, max_packet> out, std::uint8_t id, Instruction instruction,
                             std::span<const std::uint8_t> params) noexcept {
    const std::size_t total = 6 + params.size();
    if (total > out.size()) {
        return 0;
    }
    const auto length = static_cast<std::uint8_t>(params.size() + 2);
    out[0] = out[1] = 0xFF;
    out[2] = id;
    out[3] = length;
    out[4] = static_cast<std::uint8_t>(instruction);
    std::uint8_t sum = static_cast<std::uint8_t>(id + length + out[4]);
    for (std::size_t i = 0; i < params.size(); ++i) {
        out[5 + i] = params[i];
        sum = static_cast<std::uint8_t>(sum + params[i]);
    }
    out[5 + params.size()] = static_cast<std::uint8_t>(~sum);
    return total;
}
}  // namespace feetech

struct FeetechFeedback {
    std::int16_t position = 0;  ///< 0..4095 per turn
    std::int16_t speed = 0;
    std::int16_t load = 0;
    std::uint8_t voltage_dv = 0;  ///< 0.1 V
    std::uint8_t temperature_c = 0;
    bool moving = false;
    std::int16_t current = 0;
};

template <class Uart, class Clock>
class FeetechBus {
public:
    static constexpr std::size_t max_servos = 8;
    static constexpr std::uint32_t reply_timeout_ms = 5;  ///< at 1 Mbit/s a reply takes < 0.3 ms

    explicit FeetechBus(Uart& uart) noexcept : uart_(uart) {}

    /// Goal position, speed and acceleration for several servos in one broadcast packet.
    bool sync_write_positions(std::span<const std::uint8_t> ids, std::span<const std::int16_t> positions,
                              std::span<const std::uint16_t> speeds, std::uint8_t accel) noexcept {
        if (ids.empty() || ids.size() > max_servos || positions.size() != ids.size() || speeds.size() != ids.size()) {
            return false;
        }
        constexpr std::uint8_t per_servo = 7;  // acceleration, position (2), time (2, unused), speed (2)
        std::array<std::uint8_t, 2 + max_servos * (1 + per_servo)> params{};
        params[0] = feetech::acceleration;
        params[1] = per_servo;
        std::size_t n = 2;
        for (std::size_t i = 0; i < ids.size(); ++i) {
            const std::uint16_t p = feetech::encode(positions[i]);
            for (const std::uint8_t b : {ids[i], accel, static_cast<std::uint8_t>(p), static_cast<std::uint8_t>(p >> 8),
                                         std::uint8_t{0}, std::uint8_t{0}, static_cast<std::uint8_t>(speeds[i]),
                                         static_cast<std::uint8_t>(speeds[i] >> 8)}) {
                params[n++] = b;
            }
        }
        return send(feetech::broadcast, feetech::Instruction::SyncWrite, std::span(params).first(n));
    }

    [[nodiscard]] std::optional<FeetechFeedback> feedback(std::uint8_t id) noexcept {
        std::array<std::uint8_t, feetech::feedback_length> d{};
        if (!read(id, feetech::present_position, d)) {
            return std::nullopt;
        }
        return FeetechFeedback{feetech::decode(d[0], d[1]), feetech::decode(d[2], d[3]), feetech::decode(d[4], d[5]),
                               d[6],  d[7], d[10] != 0, feetech::decode(d[13], d[14])};
    }

    bool torque(std::uint8_t id, bool on) noexcept {
        const std::array<std::uint8_t, 2> p{feetech::torque_enable, static_cast<std::uint8_t>(on ? 1 : 0)};
        return send(id, feetech::Instruction::Write, p) && reply(id, {});
    }

    bool ping(std::uint8_t id) noexcept { return send(id, feetech::Instruction::Ping, {}) && reply(id, {}); }

    bool read(std::uint8_t id, std::uint8_t address, std::span<std::uint8_t> out) noexcept {
        const std::array<std::uint8_t, 2> p{address, static_cast<std::uint8_t>(out.size())};
        return send(id, feetech::Instruction::Read, p) && reply(id, out);
    }

private:
    bool send(std::uint8_t id, feetech::Instruction instruction, std::span<const std::uint8_t> params) noexcept {
        std::array<std::byte, 16> junk{};
        while (uart_.read(junk) != 0) {  // a late reply of an earlier request
        }
        std::array<std::uint8_t, feetech::max_packet> frame{};
        const std::size_t n = feetech::packet(frame, id, instruction, params);
        return n != 0 && uart_.write(std::as_bytes(std::span(frame).first(n))) == n;
    }

    /// The status packet of `id` with `out.size()` parameters and a valid checksum.
    bool reply(std::uint8_t id, std::span<std::uint8_t> out) noexcept {
        std::array<std::uint8_t, 6 + 32> r{};
        const std::size_t want = 6 + out.size();
        if (want > r.size()) {
            return false;
        }
        std::size_t got = 0;
        const std::uint32_t start = Clock::now_ms();
        while (got < want) {
            got += uart_.read(std::as_writable_bytes(std::span(r).subspan(got, want - got)));
            if (got < want) {
                if (Clock::now_ms() - start > reply_timeout_ms) {
                    return false;
                }
                Clock::sleep_ms(1);
            }
        }
        if (r[0] != 0xFF || r[1] != 0xFF || r[2] != id || r[3] != out.size() + 2) {
            return false;
        }
        std::uint8_t sum = 0;
        for (std::size_t i = 2; i < want - 1; ++i) {
            sum = static_cast<std::uint8_t>(sum + r[i]);
        }
        if (static_cast<std::uint8_t>(~sum) != r[want - 1]) {
            return false;
        }
        for (std::size_t i = 0; i < out.size(); ++i) {
            out[i] = r[5 + i];
        }
        return true;
    }

    Uart& uart_;
};

}  // namespace rtos::devices
