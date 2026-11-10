// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <vector>

#include "rtos/serial/frame.hpp"

namespace {

using rtos::serial::crc16_ccitt;
using rtos::serial::encode_frame;
using rtos::serial::FrameDecoder;
using rtos::serial::max_frame_size;
using Bytes = std::vector<std::uint8_t>;

Bytes frame(const Bytes& payload) {
    Bytes out(max_frame_size(payload.size()));
    out.resize(encode_frame(payload, out));
    return out;
}

/// Feeds the bytes; the payloads of the frames that came out.
template <std::size_t N>
std::vector<Bytes> feed(FrameDecoder<N>& d, const Bytes& stream) {
    std::vector<Bytes> got;
    for (const std::uint8_t b : stream) {
        if (const auto p = d.push(b)) {
            got.emplace_back(p->begin(), p->end());
        }
    }
    return got;
}

TEST(SerialFrame, Crc16CcittCheckValue) {
    const Bytes check{'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    EXPECT_EQ(crc16_ccitt(check), 0x29B1);
}

TEST(SerialFrame, NoZeroInsideAFrameAndOneAtItsEnd) {
    const Bytes payload{0x00, 0x11, 0x00, 0x00, 0x22, 0x00};
    const Bytes f = frame(payload);
    ASSERT_FALSE(f.empty());
    EXPECT_EQ(f.back(), 0x00);
    EXPECT_EQ(std::count(f.begin(), f.end(), 0x00), 1);
}

TEST(SerialFrame, RoundTripOfAllLengthsAroundTheBlockSize) {
    FrameDecoder<max_frame_size(600)> d;
    for (std::size_t n : {0u, 1u, 2u, 252u, 253u, 254u, 255u, 256u, 507u, 508u, 509u, 600u}) {
        for (const std::uint8_t fill : {std::uint8_t{0x00}, std::uint8_t{0x5A}}) {
            Bytes payload(n, fill);
            for (std::size_t i = 0; i < n; i += 7) {
                payload[i] = static_cast<std::uint8_t>(i);
            }
            const auto got = feed(d, frame(payload));
            ASSERT_EQ(got.size(), 1u) << n;
            EXPECT_EQ(got[0], payload) << n;
        }
    }
    EXPECT_EQ(d.errors().crc + d.errors().framing + d.errors().overflow, 0u);
}

TEST(SerialFrame, EncoderRefusesATooSmallBuffer) {
    const Bytes payload(10, 1);
    Bytes out(max_frame_size(10) - 1);
    EXPECT_EQ(encode_frame(payload, out), 0u);
}

TEST(SerialFrame, AGarbledFrameIsDroppedAndTheNextOneComesThrough) {
    FrameDecoder<64> d;
    const Bytes a{1, 2, 3, 4}, b{5, 6, 7};
    Bytes bad = frame(a);
    bad[2] ^= 0x10;  // a flipped bit (no zero made)
    Bytes stream = bad;
    const Bytes fb = frame(b);
    stream.insert(stream.end(), fb.begin(), fb.end());
    const auto got = feed(d, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0], b);
    EXPECT_EQ(d.errors().crc, 1u);
}

TEST(SerialFrame, StartingInTheMiddleOfAFrameResynchronisesAtTheDelimiter) {
    FrameDecoder<64> d;
    const Bytes a{9, 8, 7, 6, 5}, b{0, 1, 0};
    const Bytes fa = frame(a), fb = frame(b);
    Bytes stream(fa.begin() + 3, fa.end());  // the receiver came up late
    stream.insert(stream.end(), fb.begin(), fb.end());
    const auto got = feed(d, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0], b);
    EXPECT_EQ(d.errors().crc + d.errors().framing, 1u);
}

TEST(SerialFrame, AnOverlongFrameIsDroppedWithoutWritingPastTheBuffer) {
    FrameDecoder<16> d;
    const Bytes big(40, 0x33), small{1, 2};
    Bytes stream = frame(big);
    const Bytes fs = frame(small);
    stream.insert(stream.end(), fs.begin(), fs.end());
    const auto got = feed(d, stream);
    ASSERT_EQ(got.size(), 1u);
    EXPECT_EQ(got[0], small);
    EXPECT_EQ(d.errors().overflow, 1u);
}

TEST(SerialFrame, EmptyFramesAreIgnored) {
    FrameDecoder<16> d;
    EXPECT_TRUE(feed(d, Bytes{0, 0, 0}).empty());
    EXPECT_EQ(d.errors().crc + d.errors().framing + d.errors().overflow, 0u);
}

}  // namespace
