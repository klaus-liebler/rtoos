// SPDX-License-Identifier: MIT
//
// Trinamic (ADI) TMC2209 stepper driver over its single-wire UART (datasheet rev. 1.09, 4.1):
// write datagrams of 8 bytes, read requests of 4 bytes answered by 8 bytes, CRC-8 (x^8+x^2+x+1).
// TX and RX share the line, so every byte sent comes back as an echo first.
//
//   rtos::devices::Tmc2209<Uart, Clock> sorter{uart, 0b10};
//   sorter.init(Tmc2209Microsteps::M16);
//   sorter.set_current(4, 12, 6);
//   sorter.start_homing({.speed = -200});   ... sorter.tick_homing(now) until done
//
// `Uart`: a byte channel (read/write, BufferedUart); `Clock`: static now_ms() and sleep_ms().
// Transactions block the calling thread for about 1-3 ms (115200 baud plus the driver's
// reply delay) and are for a slow I/O thread, not for the network thread.
//
// Ported from the FCU firmware's stm32_libs/tmc2209 (the functions the FCU uses).
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rtos::devices {

enum class Tmc2209Microsteps : std::uint8_t { M256 = 0, M128, M64, M32, M16, M8, M4, M2, Full };

namespace tmc2209 {
// registers (datasheet 5)
inline constexpr std::uint8_t gconf = 0x00, gstat = 0x01, ifcnt = 0x02, ioin = 0x06, ihold_irun = 0x10, tstep = 0x12,
                              tcoolthrs = 0x14, vactual = 0x22, sgthrs = 0x40, sg_result = 0x41, coolconf = 0x42,
                              mscnt = 0x6A, chopconf = 0x6C, drv_status = 0x6F;
// GCONF bits
inline constexpr std::uint32_t gconf_i_scale_analog = 1u << 0, gconf_internal_rsense = 1u << 1,
                               gconf_en_spreadcycle = 1u << 2, gconf_shaft = 1u << 3, gconf_pdn_disable = 1u << 6,
                               gconf_mstep_reg_select = 1u << 7, gconf_multistep_filt = 1u << 8;
inline constexpr std::uint32_t chopconf_mres_pos = 24, chopconf_mres_mask = 0xFu << chopconf_mres_pos;
inline constexpr std::uint8_t sync = 0x05, master_address = 0xFF;
inline constexpr std::uint8_t version = 0x21;  ///< IOIN[31:24]
inline constexpr std::uint32_t clock_hz = 12'000'000;  ///< internal oscillator

[[nodiscard]] constexpr std::uint8_t crc8(std::span<const std::uint8_t> bytes) noexcept {
    std::uint8_t crc = 0;
    for (std::uint8_t b : bytes) {
        for (int j = 0; j < 8; ++j) {
            crc = ((crc >> 7) ^ (b & 0x01)) != 0 ? static_cast<std::uint8_t>((crc << 1) ^ 0x07)
                                                 : static_cast<std::uint8_t>(crc << 1);
            b >>= 1;
        }
    }
    return crc;
}

[[nodiscard]] constexpr std::array<std::uint8_t, 8> write_datagram(std::uint8_t node, std::uint8_t reg,
                                                                   std::uint32_t value) noexcept {
    std::array<std::uint8_t, 8> d{sync, node, static_cast<std::uint8_t>(reg | 0x80), static_cast<std::uint8_t>(value >> 24),
                                  static_cast<std::uint8_t>(value >> 16), static_cast<std::uint8_t>(value >> 8),
                                  static_cast<std::uint8_t>(value), 0};
    d[7] = crc8(std::span(d).first(7));
    return d;
}

[[nodiscard]] constexpr std::array<std::uint8_t, 4> read_request(std::uint8_t node, std::uint8_t reg) noexcept {
    std::array<std::uint8_t, 4> d{sync, node, reg, 0};
    d[3] = crc8(std::span(d).first(3));
    return d;
}

/// The register value from a reply datagram (sync, 0xFF, register, 4 data bytes, CRC).
[[nodiscard]] constexpr std::optional<std::uint32_t> parse_reply(std::span<const std::uint8_t, 8> r, std::uint8_t reg) noexcept {
    if (r[0] != sync || r[1] != master_address || r[2] != reg || r[7] != crc8(r.first(7))) {
        return std::nullopt;
    }
    return (std::uint32_t{r[3]} << 24) | (std::uint32_t{r[4]} << 16) | (std::uint32_t{r[5]} << 8) | r[6];
}

/// VACTUAL for `full_steps_per_s`: velocity in 1/256 microsteps per t = 2^24 / f_clk
/// (datasheet 5.2).
[[nodiscard]] constexpr std::uint32_t vactual_value(std::int32_t full_steps_per_s) noexcept {
    const std::int64_t v = static_cast<std::int64_t>(full_steps_per_s) * 256 * (std::int64_t{1} << 24) / clock_hz;
    return static_cast<std::uint32_t>(static_cast<std::int32_t>(v)) & 0x00FF'FFFFu;  // 24-bit two's complement
}
}  // namespace tmc2209

struct Tmc2209Homing {
    std::int32_t speed = -200;          ///< full steps/s towards the end stop (sign: direction)
    std::uint8_t stall_threshold = 100; ///< SGTHRS: stall when SG_RESULT < 2 x threshold
    std::uint32_t timeout_ms = 10'000;
    std::uint32_t back_off_ms = 800;    ///< first travel away from the end stop
    std::uint32_t dead_time_ms = 300;   ///< after the start: no stall detection yet
};

template <class Uart, class Clock>
class Tmc2209 {
public:
    enum class HomingResult : std::uint8_t { InProgress, Done, Timeout, Error };

    Tmc2209(Uart& uart, std::uint8_t node) noexcept : uart_(uart), node_(node) {}

    /// Check the chip, then GCONF (StealthChop, register micro steps, PDN_UART off, step
    /// filter) and the micro step resolution.
    bool init(Tmc2209Microsteps steps) noexcept {
        using namespace tmc2209;
        const auto io = read(ioin);
        const auto chop = read(chopconf);
        if (!io || (*io >> 24) != version || !chop) {
            return false;
        }
        gconf_ = gconf_pdn_disable | gconf_mstep_reg_select | gconf_multistep_filt;
        chopconf_ = (*chop & ~chopconf_mres_mask) | (static_cast<std::uint32_t>(steps) << chopconf_mres_pos);
        return write(gstat, 0x07) && write(gconf, gconf_) && read(gconf) == gconf_ && write(chopconf, chopconf_);
    }

    /// IHOLD, IRUN (0..31 of the full scale) and IHOLDDELAY (0..15).
    bool set_current(std::uint8_t hold, std::uint8_t run, std::uint8_t hold_delay) noexcept {
        return write(tmc2209::ihold_irun, (std::uint32_t{hold_delay & 0x0Fu} << 16) | (std::uint32_t{run & 0x1Fu} << 8) |
                                              (hold & 0x1Fu));
    }

    /// Internal step generator (0: STEP/DIR control again).
    bool velocity(std::int32_t full_steps_per_s) noexcept { return write(tmc2209::vactual, tmc2209::vactual_value(full_steps_per_s)); }

    // ---- sensorless homing (StallGuard4, needs SpreadCycle for CoolStep) --------------------
    bool start_homing(const Tmc2209Homing& h, std::uint32_t now_ms) noexcept {
        using namespace tmc2209;
        homing_ = h;
        // SpreadCycle, TCOOLTHRS 450, the stall threshold, CoolStep SEMIN 5, SEUP 1, SEMAX 2, SEDN 1
        constexpr std::uint32_t cool = 5u | (1u << 5) | (2u << 8) | (1u << 13);
        if (!write(gconf, gconf_ | gconf_en_spreadcycle) || !write(tcoolthrs, 450) || !write(sgthrs, h.stall_threshold) ||
            !write(coolconf, cool) || !velocity(-h.speed)) {
            phase_ = Phase::Idle;
            return false;
        }
        phase_ = Phase::BackOff;
        phase_start_ = now_ms;
        return true;
    }

    /// Advance the homing (call every few ms). Done: the end stop is reached, the motor stops.
    HomingResult tick_homing(std::uint32_t now_ms) noexcept {
        const std::uint32_t in_phase = now_ms - phase_start_;
        switch (phase_) {
            case Phase::Idle: return HomingResult::Error;
            case Phase::BackOff:
                if (in_phase >= homing_.back_off_ms) {
                    (void)velocity(0);
                    next(Phase::Settle, now_ms);
                }
                return HomingResult::InProgress;
            case Phase::Settle:
                if (in_phase >= 50) {
                    if (!velocity(homing_.speed)) {
                        return finish(HomingResult::Error);
                    }
                    next(Phase::DeadTime, now_ms);
                }
                return HomingResult::InProgress;
            case Phase::DeadTime:
                if (in_phase >= homing_.dead_time_ms) {
                    next(Phase::StallDetect, now_ms);
                }
                return HomingResult::InProgress;
            case Phase::StallDetect: {
                const auto sg = read(tmc2209::sg_result);
                if (sg && (*sg & 0x3FF) < 2u * homing_.stall_threshold) {
                    return finish(HomingResult::Done);
                }
                return in_phase >= homing_.timeout_ms ? finish(HomingResult::Timeout) : HomingResult::InProgress;
            }
        }
        return HomingResult::Error;
    }
    [[nodiscard]] bool homing() const noexcept { return phase_ != Phase::Idle; }

    // ---- register access ----------------------------------------------------------------
    bool write(std::uint8_t reg, std::uint32_t value) noexcept {
        drain();
        const auto d = tmc2209::write_datagram(node_, reg, value);
        if (!send(d)) {
            return false;
        }
        (void)receive(d.size());  // the echo of the datagram (and the time it takes to go out)
        Clock::sleep_ms(inter_transaction_ms);
        return true;
    }

    [[nodiscard]] std::optional<std::uint32_t> read(std::uint8_t reg) noexcept {
        drain();
        const auto q = tmc2209::read_request(node_, reg);
        if (!send(q)) {
            return std::nullopt;
        }
        const auto r = receive(q.size() + 8);  // echo, then the reply
        Clock::sleep_ms(inter_transaction_ms);
        if (!r) {
            return std::nullopt;
        }
        return tmc2209::parse_reply(std::span(*r).template subspan<4, 8>(), reg);
    }

private:
    enum class Phase : std::uint8_t { Idle, BackOff, Settle, DeadTime, StallDetect };
    static constexpr std::uint32_t inter_transaction_ms = 2;  ///< the product firmware's spacing
    static constexpr std::uint32_t reply_timeout_ms = 20;

    void drain() noexcept {
        std::array<std::byte, 16> junk{};
        while (uart_.read(junk) != 0) {
        }
    }
    template <std::size_t N>
    bool send(const std::array<std::uint8_t, N>& d) noexcept {
        return uart_.write(std::as_bytes(std::span(d))) == N;
    }
    /// `n` bytes within the timeout.
    std::optional<std::array<std::uint8_t, 12>> receive(std::size_t n) noexcept {
        std::array<std::uint8_t, 12> buf{};
        std::size_t got = 0;
        const std::uint32_t start = Clock::now_ms();
        while (got < n) {
            got += uart_.read(std::as_writable_bytes(std::span(buf).subspan(got, n - got)));
            if (got < n) {
                if (Clock::now_ms() - start > reply_timeout_ms) {
                    return std::nullopt;
                }
                Clock::sleep_ms(1);
            }
        }
        return buf;
    }
    HomingResult finish(HomingResult r) noexcept {
        (void)velocity(0);
        (void)write(tmc2209::gconf, gconf_);  // StealthChop again
        phase_ = Phase::Idle;
        return r;
    }
    void next(Phase p, std::uint32_t now_ms) noexcept {
        phase_ = p;
        phase_start_ = now_ms;
    }

    Uart& uart_;
    std::uint8_t node_;
    std::uint32_t gconf_ = 0, chopconf_ = 0;
    Phase phase_ = Phase::Idle;
    std::uint32_t phase_start_ = 0;
    Tmc2209Homing homing_{};
};

}  // namespace rtos::devices
