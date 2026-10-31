// SPDX-License-Identifier: MIT
//
// USB PD sink against a simulated source: the test plays the source's protocol layer
// (GoodCRC, message IDs) and policy (capabilities, Accept, PS_RDY), the PHY and the
// microsecond timer are test doubles.
#include <gtest/gtest.h>

#include <vector>

#include "rtos/usb_pd/sink.hpp"

namespace {

using namespace rtos::usb_pd;

struct FakePhy {
    std::vector<Message> sent;
    unsigned hard_resets = 0;
    bool line_free = true;
    bool transmit(const Message& m) {
        if (!line_free) {
            return false;
        }
        sent.push_back(m);
        return true;
    }
    bool transmit_hard_reset() {
        ++hard_resets;
        return true;
    }
};

struct FakeTimer {
    std::uint32_t time = 0xFFFF'0000u;  // wraps during the tests
    bool armed = false;
    std::uint32_t at = 0;
    std::uint32_t now() const { return time; }
    void arm(std::uint32_t t) {
        armed = true;
        at = t;
    }
    void disarm() { armed = false; }
};

constexpr std::uint32_t fixed_pdo(int mv, int ma) { return static_cast<std::uint32_t>((mv / 50) << 10 | (ma / 10)); }
constexpr std::uint32_t pps_pdo(int min_mv, int max_mv, int ma) {
    return 3u << 30 | static_cast<std::uint32_t>((max_mv / 100) << 17 | (min_mv / 100) << 8 | (ma / 50));
}

class SinkTest : public ::testing::Test {
protected:
    FakePhy phy;
    FakeTimer timer;
    Sink<FakePhy, FakeTimer> sink{phy, timer};
    unsigned source_id = 0;
    std::vector<SinkEvent> events;

    /// Advance the clock, firing the deadline like the timer interrupt.
    void advance(std::uint32_t us) {
        for (std::uint32_t step = 0; step < us; step += 100) {
            timer.time += 100;
            if (timer.armed && static_cast<std::int32_t>(timer.time - timer.at) >= 0) {
                timer.armed = false;
                sink.on_timer();
            }
        }
    }

    /// A message from the source (revision 3.0, source, DFP).
    Message from_source(MessageType type, std::initializer_list<std::uint32_t> objects = {}, int id = -1) {
        Message m;
        const unsigned mid = id >= 0 ? static_cast<unsigned>(id) : source_id++ & 7u;
        m.header = static_cast<std::uint16_t>(objects.size() << 12 | mid << 9 | 1u << 8 | 2u << 6 | 1u << 5 |
                                              (static_cast<unsigned>(type) & 0x1Fu));
        std::ranges::copy(objects, m.objects.begin());
        return m;
    }

    /// The source sends `m`; the sink must answer with GoodCRC, which then goes out.
    void deliver(const Message& m) {
        const auto before = phy.sent.size();
        sink.on_message(m);
        ASSERT_EQ(phy.sent.size(), before + 1) << "GoodCRC";
        EXPECT_EQ(phy.sent.back().type(), MessageType::GoodCrc);
        EXPECT_EQ(phy.sent.back().message_id(), m.message_id());
        sink.on_transmitted(true);
    }

    /// The last message of the sink went out and the source confirms it.
    const Message& acknowledge() {
        const Message& m = phy.sent.back();
        EXPECT_NE(m.type(), MessageType::GoodCrc);
        sink.on_transmitted(true);
        Message crc;
        crc.header = static_cast<std::uint16_t>(m.message_id() << 9 | 1u << 8 | 2u << 6 | 0x01u);
        sink.on_message(crc);
        return m;
    }

    void collect() {
        sink.poll([this](SinkEvent e) { events.push_back(e); });
    }

    void attach_with_capabilities() {
        sink.on_attach(1);
        deliver(from_source(MessageType::SourceCapabilities,
                            {fixed_pdo(5000, 3000), fixed_pdo(9000, 3000), fixed_pdo(12000, 2250),
                             pps_pdo(3300, 11000, 3000)}));
    }

    void contract() {
        acknowledge();
        deliver(from_source(MessageType::Accept));
        deliver(from_source(MessageType::PsReady));
    }
};

TEST(UsbPdCapabilities, ParsesFixedAndPpsObjects) {
    Message m;
    m.init(MessageType::SourceCapabilities, 3, 1);
    m.objects[0] = fixed_pdo(5000, 3000) | 1u << 26;  // flags in the upper bits are ignored
    m.objects[1] = pps_pdo(3300, 21000, 5000);
    m.objects[2] = 3u << 30 | 1u << 28;               // AVS: skipped
    std::array<SourceCapability, Message::max_objects> caps{};
    ASSERT_EQ(parse_source_capabilities(m, caps), 2u);
    EXPECT_EQ(caps[0].type, SupplyType::Fixed);
    EXPECT_EQ(caps[0].min_mv, 5000);
    EXPECT_EQ(caps[0].max_mv, 5000);
    EXPECT_EQ(caps[0].max_ma, 3000);
    EXPECT_EQ(caps[1].type, SupplyType::Pps);
    EXPECT_EQ(caps[1].position, 2);
    EXPECT_EQ(caps[1].min_mv, 3300);
    EXPECT_EQ(caps[1].max_mv, 21000);
    EXPECT_EQ(caps[1].max_ma, 5000);
}

TEST(UsbPdCapabilities, RequestObjects) {
    const std::array<SourceCapability, 2> caps{{{SupplyType::Fixed, 1, 3000, 5000, 5000},
                                                {SupplyType::Pps, 2, 3000, 3300, 11000}}};
    const auto fixed = power_request(caps, 5000, 1234);
    ASSERT_TRUE(fixed.valid());
    EXPECT_EQ(fixed.object, 1u << 28 | 1u << 24 | 123u << 10 | 123u);  // 10 mA units
    EXPECT_EQ(fixed.ma, 1230);
    const auto pps = power_request(caps, 7510);
    ASSERT_TRUE(pps.valid());
    EXPECT_EQ(pps.index, 1u);
    EXPECT_EQ(pps.object, 2u << 28 | 1u << 24 | 376u << 9 | 60u);  // 20 mV and 50 mA units
    EXPECT_EQ(pps.mv, 7520);
    EXPECT_EQ(pps.ma, 3000);
    EXPECT_FALSE(power_request(caps, 20000).valid());
    EXPECT_FALSE(power_request(caps, 5000, 3500).valid()) << "more current than offered";
}

TEST_F(SinkTest, NegotiatesTheDesiredVoltage) {
    EXPECT_FALSE(sink.request_power(12000)) << "no source yet, remembered";
    attach_with_capabilities();
    ASSERT_EQ(phy.sent.back().type(), MessageType::Request);
    EXPECT_EQ(phy.sent.back().objects[0] >> 28, 3u) << "third object: 12 V";
    EXPECT_EQ(phy.sent.back().spec_revision(), 1u) << "revision 2.0 headers";
    EXPECT_EQ(phy.sent.back().message_id(), 0u);
    contract();
    const auto s = sink.status();
    EXPECT_TRUE(s.connected);
    EXPECT_EQ(s.active_mv, 12000);
    EXPECT_EQ(s.active_ma, 2250);
    EXPECT_EQ(s.capability_count, 4u);
    collect();
    EXPECT_NE(std::ranges::find(events, SinkEvent::SourceCapabilitiesChanged), events.end());
    EXPECT_NE(std::ranges::find(events, SinkEvent::VoltageChanged), events.end());
    EXPECT_EQ(sink.tx_failures(), 0u);
}

TEST_F(SinkTest, FallsBackToFiveVoltsAndRequestsLater) {
    sink.request_power(20000);
    attach_with_capabilities();
    EXPECT_EQ(phy.sent.back().objects[0] >> 28, 1u) << "5 V";
    contract();
    EXPECT_EQ(sink.status().active_mv, 5000);
    EXPECT_TRUE(sink.request_power(9000));
    EXPECT_EQ(phy.sent.back().objects[0] >> 28, 2u);
    EXPECT_EQ(phy.sent.back().message_id(), 1u) << "next message ID";
    contract();
    EXPECT_EQ(sink.status().active_mv, 9000);
}

TEST_F(SinkTest, RetriesWithoutGoodCrcThenGivesUp) {
    attach_with_capabilities();
    const auto before = phy.sent.size();
    sink.on_transmitted(true);  // Request out, no GoodCRC follows
    for (int i = 0; i < 3; ++i) {
        advance(1'200);
        if (i < 2) {
            ASSERT_EQ(phy.sent.size(), before + i + 1u);
            EXPECT_EQ(phy.sent.back().type(), MessageType::Request);
            EXPECT_EQ(phy.sent.back().message_id(), 0u) << "a retry keeps the ID";
            sink.on_transmitted(true);
        }
    }
    EXPECT_EQ(phy.sent.size(), before + 2u) << "nRetryCount = 2";
    EXPECT_EQ(sink.tx_failures(), 1u);
}

TEST_F(SinkTest, RepeatedMessageIsConfirmedButHandledOnce) {
    sink.on_attach(1);
    const auto caps = from_source(MessageType::SourceCapabilities, {fixed_pdo(5000, 3000)});
    deliver(caps);
    const auto requests = std::ranges::count_if(phy.sent, [](const Message& m) { return m.type() == MessageType::Request; });
    acknowledge();
    deliver(caps);  // the source missed our GoodCRC and repeats
    EXPECT_EQ(std::ranges::count_if(phy.sent, [](const Message& m) { return m.type() == MessageType::Request; }), requests);
}

TEST_F(SinkTest, RejectRaisesEventAndKeepsTheActiveContract) {
    sink.request_power(9000);
    attach_with_capabilities();
    acknowledge();
    deliver(from_source(MessageType::Reject));
    collect();
    EXPECT_NE(std::ranges::find(events, SinkEvent::PowerRejected), events.end());
    EXPECT_EQ(sink.status().requested_mv, sink.status().active_mv);
}

TEST_F(SinkTest, WaitRepeatsTheRequest) {
    sink.request_power(9000);
    attach_with_capabilities();
    acknowledge();
    deliver(from_source(MessageType::Wait));
    const auto before = phy.sent.size();
    advance(100'000);
    ASSERT_EQ(phy.sent.size(), before + 1);
    EXPECT_EQ(phy.sent.back().type(), MessageType::Request);
    EXPECT_EQ(phy.sent.back().objects[0] >> 28, 2u);
}

TEST_F(SinkTest, AsksForCapabilitiesThenHardResets) {
    sink.on_attach(2);
    for (unsigned i = 0; i < 3; ++i) {
        advance(i == 0 ? 100'000 : 200'000);
        ASSERT_EQ(phy.sent.back().type(), MessageType::GetSourceCap) << i;
        acknowledge();
    }
    EXPECT_EQ(phy.hard_resets, 0u);
    advance(200'000);
    EXPECT_EQ(phy.hard_resets, 1u);
    sink.on_hard_reset();  // sent
    deliver(from_source(MessageType::SourceCapabilities, {fixed_pdo(5000, 3000)}, 0));
    EXPECT_EQ(phy.sent.back().type(), MessageType::Request);
    EXPECT_EQ(phy.sent.back().message_id(), 0u) << "IDs restart after a Hard Reset";
}

TEST_F(SinkTest, RequestWhileTransmittingIsDeferred) {
    attach_with_capabilities();
    contract();
    sink.request_power(9000);  // goes out
    const auto before = phy.sent.size();
    EXPECT_TRUE(sink.request_power(12000)) << "busy: deferred";
    EXPECT_EQ(phy.sent.size(), before);
    acknowledge();  // the 9 V request completes, the 12 V one follows
    ASSERT_EQ(phy.sent.size(), before + 1);
    EXPECT_EQ(phy.sent.back().objects[0] >> 28, 3u);
}

TEST_F(SinkTest, PpsContractIsRefreshed) {
    sink.request_power(7500, 2000);
    attach_with_capabilities();
    EXPECT_EQ(phy.sent.back().objects[0] >> 28, 4u);
    contract();
    EXPECT_EQ(sink.status().active_mv, 7500);
    const auto before = phy.sent.size();
    advance(7'900'000);
    EXPECT_EQ(phy.sent.size(), before);
    advance(200'000);
    ASSERT_EQ(phy.sent.size(), before + 1);
    EXPECT_EQ(phy.sent.back().type(), MessageType::Request);
    const auto first = std::ranges::find_if(phy.sent, [](const Message& m) { return m.type() == MessageType::Request; });
    EXPECT_EQ(phy.sent.back().objects[0], first->objects[0]) << "same PPS request";
}

TEST_F(SinkTest, SoftResetIsAccepted) {
    attach_with_capabilities();
    contract();
    deliver(from_source(MessageType::SoftReset, {}, 0));
    EXPECT_EQ(phy.sent.back().type(), MessageType::Accept);
    acknowledge();
    deliver(from_source(MessageType::SourceCapabilities, {fixed_pdo(5000, 3000)}, 0));
    EXPECT_EQ(phy.sent.back().type(), MessageType::Request) << "ID 0 again is a new message";
}

TEST_F(SinkTest, UnsupportedRequestsAreRejected) {
    attach_with_capabilities();
    contract();
    deliver(from_source(MessageType::GetSinkCap));
    EXPECT_EQ(phy.sent.back().type(), MessageType::Reject);
}

TEST_F(SinkTest, LostGoodCrcMakesTheRepeatNew) {
    sink.on_attach(1);
    const auto caps = from_source(MessageType::SourceCapabilities, {fixed_pdo(5000, 3000)});
    sink.on_message(caps);
    sink.on_transmitted(false);  // our GoodCRC was discarded
    deliver(caps);
    EXPECT_EQ(phy.sent.back().type(), MessageType::Request);
}

TEST_F(SinkTest, DetachDropsTheContract) {
    attach_with_capabilities();
    contract();
    collect();
    events.clear();
    sink.on_attach(0);
    const auto s = sink.status();
    EXPECT_FALSE(s.attached);
    EXPECT_FALSE(s.connected);
    EXPECT_EQ(s.active_mv, 0);
    collect();
    EXPECT_NE(std::ranges::find(events, SinkEvent::VoltageChanged), events.end());
    EXPECT_FALSE(timer.armed);
}

}  // namespace
