// SPDX-License-Identifier: MIT
//
// Avia HX711 24-bit load cell ADC, clocked through an SPI master without its SCK pin: MOSI
// drives the HX711's PD_SCK, MISO samples DOUT. Every byte 0x55 (01010101) gives four clock
// pulses; MISO is sampled once per SPI bit, so the HX711's data bit is the sample in the
// second half of each pulse (bits 6, 4, 2, 0 of each received byte). Six bytes clock the 24
// data bits; the seventh selects channel and gain for the next conversion (1, 2 or 3 extra
// pulses), the eighth keeps MOSI low (datasheet: PD_SCK high > 60 us powers the chip down).
//
//   using Scale = rtos::devices::Hx711<Spi2, data_ready_pin_reader, Hx711Gain::A128, 8>;
//   Scale::init();
//   Scale::poll();                       // when DOUT is low: one transfer (64 SPI bits)
//   if (auto w = Scale::average()) ...   // mean of the last 8 readings
//
// The SPI bit rate must give pulses of 0.2 .. 50 us (datasheet table 3): 20 kHz .. 2.5 MHz.
// Ported from the FCU firmware's stm32_libs/hx711.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

namespace rtos::devices {

enum class Hx711Gain : std::uint8_t {
    A128 = 0x40,  ///< channel A, gain 128: 1 extra pulse
    B32 = 0x50,   ///< channel B, gain 32: 2 extra pulses
    A64 = 0x54,   ///< channel A, gain 64: 3 extra pulses
};

namespace hx711 {
/// The 24-bit two's complement value from the 8 received bytes (data in bits 6, 4, 2, 0 of
/// bytes 0..5, MSB first), sign-extended.
[[nodiscard]] constexpr std::int32_t decode(std::span<const std::byte, 8> rx) noexcept {
    std::uint32_t v = 0;
    for (std::size_t i = 0; i < 6; ++i) {
        const auto b = std::to_integer<std::uint32_t>(rx[i]);
        for (int bit = 6; bit >= 0; bit -= 2) {
            v = (v << 1) | ((b >> bit) & 1u);
        }
    }
    return static_cast<std::int32_t>(v << 8) / 256;
}

[[nodiscard]] constexpr std::array<std::byte, 8> clock_pattern(Hx711Gain gain) noexcept {
    return {std::byte{0x55}, std::byte{0x55}, std::byte{0x55}, std::byte{0x55}, std::byte{0x55},
            std::byte{0x55}, static_cast<std::byte>(gain), std::byte{0x00}};
}
}  // namespace hx711

/// `Spi`: static transfer(tx, rx) and write(tx); `DataReady`: a function returning true while
/// DOUT is low (a conversion is ready).
template <class Spi, bool (*DataReady)() noexcept, Hx711Gain Gain = Hx711Gain::A128, std::size_t Average = 8>
class Hx711 {
    static_assert(Average >= 1);

public:
    /// Drive MOSI low (some SPI blocks leave it high between transfers until the first one).
    static bool init() noexcept {
        constexpr std::array<std::byte, 4> zeros{};
        count_ = next_ = 0;
        return Spi::write(zeros);
    }

    /// Read a conversion if one is ready. Returns the new reading, or nullopt.
    static std::optional<std::int32_t> poll() noexcept {
        if (!DataReady()) {
            return std::nullopt;
        }
        static constexpr auto tx = hx711::clock_pattern(Gain);
        std::array<std::byte, 8> rx{};
        if (!Spi::transfer(tx, rx)) {
            return std::nullopt;
        }
        const std::int32_t v = hx711::decode(rx);
        readings_[next_] = v;
        next_ = (next_ + 1) % Average;
        count_ = count_ < Average ? count_ + 1 : Average;
        return v;
    }

    /// Mean of the last `Average` readings, once that many were read.
    [[nodiscard]] static std::optional<std::int32_t> average() noexcept {
        if (count_ < Average) {
            return std::nullopt;
        }
        std::int64_t sum = 0;
        for (const std::int32_t r : readings_) {
            sum += r;
        }
        return static_cast<std::int32_t>(sum / static_cast<std::int64_t>(Average));
    }

private:
    static inline std::array<std::int32_t, Average> readings_{};
    static inline std::size_t next_ = 0, count_ = 0;
};

}  // namespace rtos::devices
