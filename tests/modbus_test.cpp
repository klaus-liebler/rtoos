// SPDX-License-Identifier: MIT
//
// Modbus: the PDU against the examples of the Modbus Application Protocol
// Specification V1.1b3 and its exception rules, RegisterBank, RTU framing (Modbus
// over Serial Line V1.02: CRC, addresses, broadcast, frames in pieces, silence),
// and Modbus TCP over the stack (MBAP framing, pipelining, unit ids, timeouts).
#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "net_fixture.hpp"
#include "rtos/modbus/modbus.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/modbus/rtu.hpp"
#include "rtos/net/modbus_tcp_server.hpp"

namespace {

using namespace rtos::modbus;
using Bytes = std::vector<std::byte>;

Bytes bytes(std::initializer_list<int> v) {
    Bytes out;
    for (int b : v) {
        out.push_back(static_cast<std::byte>(b));
    }
    return out;
}

using Bank = RegisterBank<128, 16>;

Bytes pdu(RegisterMap& bank, const Bytes& request) {
    std::array<std::byte, max_pdu> response{};
    const std::size_t n = process_pdu(bank, request, response);
    return {response.begin(), response.begin() + static_cast<std::ptrdiff_t>(n)};
}

// ---------------------------------------------------------------------------
// PDU: specification examples
// ---------------------------------------------------------------------------

TEST(ModbusPdu, ReadHoldingRegistersSpecExample) {  // spec 6.3
    Bank bank;
    // "registers 108 to 110" of the example are numbered from 1: PDU addresses 107..109
    bank.set_holding(107, 0x022B);
    bank.set_holding(108, 0x0000);
    bank.set_holding(109, 0x0064);
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 0x6B, 0x00, 0x03})),
              bytes({0x03, 0x06, 0x02, 0x2B, 0x00, 0x00, 0x00, 0x64}));
}

TEST(ModbusPdu, ReadInputRegisterSpecExample) {  // spec 6.4
    Bank bank;
    bank.set_input(8, 0x000A);
    EXPECT_EQ(pdu(bank, bytes({0x04, 0x00, 0x08, 0x00, 0x01})), bytes({0x04, 0x02, 0x00, 0x0A}));
}

TEST(ModbusPdu, WriteSingleRegisterSpecExample) {  // spec 6.6
    Bank bank;
    EXPECT_EQ(pdu(bank, bytes({0x06, 0x00, 0x01, 0x00, 0x03})), bytes({0x06, 0x00, 0x01, 0x00, 0x03}));
    EXPECT_EQ(bank.holding(1), 3);
}

TEST(ModbusPdu, WriteMultipleRegistersSpecExample) {  // spec 6.12
    Bank bank;
    EXPECT_EQ(pdu(bank, bytes({0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02})),
              bytes({0x10, 0x00, 0x01, 0x00, 0x02}));
    EXPECT_EQ(bank.holding(1), 0x000A);
    EXPECT_EQ(bank.holding(2), 0x0102);
}

TEST(ModbusPdu, ReadsTheLastRegistersAndTheMaximumCount) {
    Bank bank;
    bank.set_holding(127, 0xBEEF);
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 127, 0x00, 0x01})), bytes({0x03, 0x02, 0xBE, 0xEF}));
    RegisterBank<200, 0> big;
    const Bytes r = pdu(big, bytes({0x03, 0x00, 0x00, 0x00, 125}));
    ASSERT_EQ(r.size(), 2u + 250u);
    EXPECT_EQ(r[1], std::byte{250});
}

// ---------------------------------------------------------------------------
// PDU: exceptions (spec 7 and the state diagrams of 6.x)
// ---------------------------------------------------------------------------

TEST(ModbusPdu, UnsupportedFunctionIsException01) {
    Bank bank;
    EXPECT_EQ(pdu(bank, bytes({0x07})), bytes({0x87, 0x01}));
    EXPECT_EQ(pdu(bank, bytes({0x2B, 0x0E, 0x01, 0x00})), bytes({0xAB, 0x01}));
    EXPECT_EQ(pdu(bank, bytes({0x01, 0x00, 0x00, 0x00, 0x01})), bytes({0x81, 0x01}));  // coils: none
}

TEST(ModbusPdu, QuantityOutOfRangeIsException03) {
    Bank bank;
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 0x00, 0x00, 0x00})), bytes({0x83, 0x03}));
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 0x00, 0x00, 126})), bytes({0x83, 0x03}));
    EXPECT_EQ(pdu(bank, bytes({0x04, 0x00, 0x00, 0xFF, 0xFF})), bytes({0x84, 0x03}));
    EXPECT_EQ(pdu(bank, bytes({0x10, 0x00, 0x00, 0x00, 124, 248})), bytes({0x90, 0x03}));
}

TEST(ModbusPdu, AddressOutOfRangeIsException02) {
    Bank bank;
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 120, 0x00, 9})), bytes({0x83, 0x02}));  // 120..128
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 119, 0x00, 9})).size(), 2u + 18u);      // 119..127: fine
    EXPECT_EQ(pdu(bank, bytes({0x03, 0xFF, 0xFF, 0x00, 0x02})), bytes({0x83, 0x02}));  // no 16-bit wrap
    EXPECT_EQ(pdu(bank, bytes({0x04, 0x00, 16, 0x00, 0x01})), bytes({0x84, 0x02}));
    EXPECT_EQ(pdu(bank, bytes({0x06, 0x00, 128, 0x00, 0x01})), bytes({0x86, 0x02}));
    EXPECT_EQ(pdu(bank, bytes({0x10, 0x00, 127, 0x00, 0x02, 0x04, 0, 1, 0, 2})), bytes({0x90, 0x02}));
    EXPECT_EQ(bank.holding(127), 0);  // nothing written
}

TEST(ModbusPdu, MalformedRequestsAreException03) {
    Bank bank;
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 0x00, 0x00})), bytes({0x83, 0x03}));           // short
    EXPECT_EQ(pdu(bank, bytes({0x03, 0x00, 0x00, 0x00, 0x01, 0x00})), bytes({0x83, 0x03}));  // long
    EXPECT_EQ(pdu(bank, bytes({0x06, 0x00, 0x01})), bytes({0x86, 0x03}));
    EXPECT_EQ(pdu(bank, bytes({0x10, 0x00, 0x01, 0x00, 0x02, 0x03, 0, 1, 0})), bytes({0x90, 0x03}));  // count != 2 x qty
    EXPECT_EQ(pdu(bank, bytes({0x10, 0x00, 0x01, 0x00, 0x02, 0x04, 0, 1, 0})), bytes({0x90, 0x03}));  // truncated
    EXPECT_EQ(pdu(bank, bytes({})), Bytes{});
}

// ---------------------------------------------------------------------------
// RegisterBank
// ---------------------------------------------------------------------------

struct CountingGuard {
    static inline int entered = 0;
    CountingGuard() noexcept { ++entered; }
};

TEST(RegisterBank, OneGuardPerRequestBlock) {
    RegisterBank<64, 64, CountingGuard> bank;
    CountingGuard::entered = 0;
    std::array<std::byte, max_pdu> response{};
    process_pdu(bank, bytes({0x03, 0x00, 0x00, 0x00, 50}), response);
    EXPECT_EQ(CountingGuard::entered, 1);  // 50 registers, one consistent copy
    process_pdu(bank, bytes({0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0, 1, 0, 2}), response);
    EXPECT_EQ(CountingGuard::entered, 2);
    process_pdu(bank, bytes({0x03, 0x00, 0x00, 0x00, 200}), response);  // rejected before the map
    EXPECT_EQ(CountingGuard::entered, 2);
}

TEST(RegisterBank, WriteHookAfterSuccessfulWritesOnly) {
    struct Seen {
        int calls = 0;
        std::uint16_t start = 0, count = 0;
    } seen;
    RegisterBank<8, 0> bank{WriteHook{&seen, [](void* s, std::uint16_t start, std::uint16_t count) noexcept {
                                          auto& x = *static_cast<Seen*>(s);
                                          ++x.calls;
                                          x.start = start;
                                          x.count = count;
                                      }}};
    std::array<std::byte, max_pdu> response{};
    process_pdu(bank, bytes({0x10, 0x00, 0x02, 0x00, 0x03, 0x06, 0, 1, 0, 2, 0, 3}), response);
    EXPECT_EQ(seen.calls, 1);
    EXPECT_EQ(seen.start, 2);
    EXPECT_EQ(seen.count, 3);
    process_pdu(bank, bytes({0x06, 0x00, 0x08, 0x00, 0x01}), response);  // exception 02
    process_pdu(bank, bytes({0x03, 0x00, 0x00, 0x00, 0x01}), response);  // read
    EXPECT_EQ(seen.calls, 1);
    std::array<std::uint16_t, 3> values{};
    bank.read_holdings(2, values);
    EXPECT_EQ(values, (std::array<std::uint16_t, 3>{1, 2, 3}));
}

// ---------------------------------------------------------------------------
// RTU
// ---------------------------------------------------------------------------

TEST(ModbusRtu, Crc16KnownVector) {
    // Modbus over Serial Line: 01 03 00 00 00 0A -> CRC C5 CD (low byte first)
    EXPECT_EQ(crc16(bytes({0x01, 0x03, 0x00, 0x00, 0x00, 0x0A})), 0xCDC5);
}

Bytes rtu_frame(std::initializer_list<int> body) {
    Bytes f = bytes(body);
    const std::uint16_t crc = crc16(f);
    f.push_back(static_cast<std::byte>(crc & 0xFF));
    f.push_back(static_cast<std::byte>(crc >> 8));
    return f;
}

Bytes to_vector(std::span<const std::byte> s) { return {s.begin(), s.end()}; }

TEST(ModbusRtu, AnswersItsAddressWithValidCrc) {
    Bank bank;
    bank.set_holding(0, 0x1234);
    bank.set_holding(1, 0x5678);
    RtuServer rtu{bank, 17};
    const Bytes response = to_vector(rtu.receive(rtu_frame({17, 0x03, 0x00, 0x00, 0x00, 0x02})));
    EXPECT_EQ(response, rtu_frame({17, 0x03, 0x04, 0x12, 0x34, 0x56, 0x78}));
    EXPECT_EQ(rtu.stats().frames, 1u);
}

TEST(ModbusRtu, FrameInSingleBytesAnswersOnlyAtTheEnd) {
    Bank bank;
    RtuServer rtu{bank, 1};
    const Bytes frame = rtu_frame({1, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x0A, 0x01, 0x02});
    for (std::size_t i = 0; i + 1 < frame.size(); ++i) {
        EXPECT_TRUE(rtu.receive({&frame[i], 1}).empty()) << "byte " << i;
    }
    EXPECT_EQ(to_vector(rtu.receive({&frame.back(), 1})), rtu_frame({1, 0x10, 0x00, 0x00, 0x00, 0x02}));
    EXPECT_EQ(bank.holding(1), 0x0102);
}

TEST(ModbusRtu, BadCrcAndOtherAddressesGetNoAnswer) {
    Bank bank;
    RtuServer rtu{bank, 1};
    Bytes bad = rtu_frame({1, 0x06, 0x00, 0x00, 0x00, 0x07});
    bad.back() ^= std::byte{1};
    EXPECT_TRUE(rtu.receive(bad).empty());
    EXPECT_EQ(rtu.stats().crc_errors, 1u);
    EXPECT_TRUE(rtu.receive(rtu_frame({2, 0x06, 0x00, 0x00, 0x00, 0x07})).empty());
    EXPECT_EQ(rtu.stats().other_address, 1u);
    EXPECT_EQ(bank.holding(0), 0);
}

TEST(ModbusRtu, BroadcastWritesWithoutAnswerAndIgnoresReads) {
    Bank bank;
    RtuServer rtu{bank, 1};
    EXPECT_TRUE(rtu.receive(rtu_frame({0, 0x06, 0x00, 0x03, 0x00, 0x2A})).empty());
    EXPECT_EQ(bank.holding(3), 42);
    EXPECT_TRUE(rtu.receive(rtu_frame({0, 0x03, 0x00, 0x03, 0x00, 0x01})).empty());
    EXPECT_EQ(rtu.stats().broadcasts, 1u);
}

TEST(ModbusRtu, UnknownFunctionEndsAtSilenceWithException01) {
    Bank bank;
    RtuServer rtu{bank, 1};
    EXPECT_TRUE(rtu.receive(rtu_frame({1, 0x2B, 0x0E, 0x01, 0x00})).empty());  // length unknown
    EXPECT_EQ(to_vector(rtu.silence()), rtu_frame({1, 0xAB, 0x01}));
}

TEST(ModbusRtu, TwoFramesInOneChunk) {
    Bank bank;
    RtuServer rtu{bank, 1};
    Bytes chunk = rtu_frame({1, 0x06, 0x00, 0x00, 0x00, 0x01});
    const Bytes second = rtu_frame({1, 0x06, 0x00, 0x01, 0x00, 0x02});
    chunk.insert(chunk.end(), second.begin(), second.end());
    EXPECT_EQ(to_vector(rtu.receive(chunk)), rtu_frame({1, 0x06, 0x00, 0x00, 0x00, 0x01}));
    EXPECT_EQ(to_vector(rtu.receive({})), second);  // echo of the second request
    EXPECT_EQ(bank.holding(1), 2);
}

TEST(ModbusRtu, SilenceDropsAnIncompleteFrameAndOverrunsAreCounted) {
    Bank bank;
    RtuServer rtu{bank, 1};
    const Bytes frame = rtu_frame({1, 0x03, 0x00, 0x00, 0x00, 0x01});
    EXPECT_TRUE(rtu.receive({frame.data(), 5}).empty());
    EXPECT_TRUE(rtu.silence().empty());
    EXPECT_EQ(rtu.buffered(), 0u);
    EXPECT_FALSE(rtu.receive(frame).empty());  // the next frame is clean
    const Bytes noise(300, std::byte{0x2B});
    EXPECT_TRUE(rtu.receive(noise).empty());
    EXPECT_TRUE(rtu.silence().empty());
    EXPECT_EQ(rtu.stats().overruns, 1u);
    EXPECT_FALSE(rtu.receive(frame).empty());
}

// ---------------------------------------------------------------------------
// Modbus TCP over the stack
// ---------------------------------------------------------------------------

using namespace test;
using Server = rtos::net::ModbusTcpServer<TestStack, 2>;

class ModbusTcpTest : public NetTest {
protected:
    static constexpr std::uint16_t port = 502;
    Bank bank;
    rtos::net::ModbusTcpConfig config{};
    std::unique_ptr<Server> server;  // replaced by start(): the old one stops listening

    struct Client {
        std::uint16_t port;
        std::uint32_t seq;  ///< next sequence number to send
        std::uint32_t iss = 0;
        std::uint32_t received = 0;  ///< payload bytes received
    };

    void start(const rtos::net::ModbusTcpConfig& cfg = {}) {
        config = cfg;
        server.reset();
        server = std::make_unique<Server>(stack, bank, config);
        ASSERT_TRUE(server->start(port));
    }

    void SetUp() override {
        NetTest::SetUp();
        start();
    }

    Client connect(std::uint16_t client_port) {
        Client c{client_port, 7000};
        deliver(tcp(client_port, port, c.seq, 0, tcp_flag::syn, {}, 8192, 1460));
        const auto f = sent();
        EXPECT_EQ(f.size(), 1u);
        c.iss = f.at(0).seq();
        ++c.seq;
        deliver(tcp(client_port, port, c.seq, c.iss + 1, tcp_flag::ack));
        return c;
    }

    /// Send raw bytes, run the server, return the payload it sent back.
    std::string exchange(Client& c, const Bytes& data) {
        deliver(tcp(c.port, port, c.seq, c.iss + 1 + c.received, tcp_flag::ack | tcp_flag::psh,
                    {reinterpret_cast<const char*>(data.data()), data.size()}));
        c.seq += static_cast<std::uint32_t>(data.size());
        server->service();
        std::string out;
        for (const Frame& f : sent()) {
            out += f.tcp_payload();
        }
        c.received += static_cast<std::uint32_t>(out.size());
        return out;
    }

    static Bytes adu(std::uint16_t tid, std::uint8_t unit, const Bytes& pdu_bytes, std::uint16_t protocol = 0) {
        Bytes a(7);
        store_be16(a.data(), tid);
        store_be16(a.data() + 2, protocol);
        store_be16(a.data() + 4, static_cast<std::uint16_t>(pdu_bytes.size() + 1));
        a[6] = static_cast<std::byte>(unit);
        a.insert(a.end(), pdu_bytes.begin(), pdu_bytes.end());
        return a;
    }
    static std::string str(const Bytes& b) { return {reinterpret_cast<const char*>(b.data()), b.size()}; }

    bool closed_by_server() {
        for (const Frame& f : sent()) {
            if (f.flags() & tcp_flag::fin) {
                return true;
            }
        }
        return false;
    }
};

TEST_F(ModbusTcpTest, EchoesTransactionAndUnitId) {
    bank.set_holding(4, 0xCAFE);
    Client c = connect(40000);
    EXPECT_EQ(exchange(c, adu(0x1234, 0x11, bytes({0x03, 0x00, 0x04, 0x00, 0x01}))),
              str(adu(0x1234, 0x11, bytes({0x03, 0x02, 0xCA, 0xFE}))));
    EXPECT_EQ(server->stats().requests, 1u);
}

TEST_F(ModbusTcpTest, RequestSplitOverSegmentsIsReassembled) {
    Client c = connect(40000);
    const Bytes request = adu(7, 1, bytes({0x06, 0x00, 0x02, 0x00, 0x09}));
    EXPECT_EQ(exchange(c, Bytes(request.begin(), request.begin() + 3)), "");
    EXPECT_EQ(exchange(c, Bytes(request.begin() + 3, request.begin() + 9)), "");
    EXPECT_EQ(exchange(c, Bytes(request.begin() + 9, request.end())), str(request));
    EXPECT_EQ(bank.holding(2), 9);
}

TEST_F(ModbusTcpTest, PipelinedRequestsAreAnsweredInOrder) {
    Client c = connect(40000);
    Bytes two = adu(1, 1, bytes({0x06, 0x00, 0x00, 0x00, 0x05}));
    const Bytes second = adu(2, 1, bytes({0x03, 0x00, 0x00, 0x00, 0x01}));
    two.insert(two.end(), second.begin(), second.end());
    EXPECT_EQ(exchange(c, two),
              str(adu(1, 1, bytes({0x06, 0x00, 0x00, 0x00, 0x05}))) + str(adu(2, 1, bytes({0x03, 0x02, 0x00, 0x05}))));
}

TEST_F(ModbusTcpTest, ExceptionResponsesAreCounted) {
    Client c = connect(40000);
    EXPECT_EQ(exchange(c, adu(3, 1, bytes({0x03, 0x00, 0x00, 0x00, 0x00}))), str(adu(3, 1, bytes({0x83, 0x03}))));
    EXPECT_EQ(server->stats().exceptions, 1u);
}

TEST_F(ModbusTcpTest, BadProtocolIdOrLengthClosesTheConnection) {
    Client c = connect(40000);
    deliver(tcp(c.port, port, c.seq, c.iss + 1, tcp_flag::ack | tcp_flag::psh,
                str(adu(1, 1, bytes({0x03, 0x00, 0x00, 0x00, 0x01}), 0x0001))));
    server->service();
    poll();
    EXPECT_TRUE(closed_by_server());
    EXPECT_EQ(server->stats().protocol_errors, 1u);

    Client d = connect(40001);
    Bytes huge = adu(1, 1, bytes({0x03}));
    store_be16(huge.data() + 4, 300);
    deliver(tcp(d.port, port, d.seq, d.iss + 1, tcp_flag::ack | tcp_flag::psh, str(huge)));
    server->service();
    poll();
    EXPECT_TRUE(closed_by_server());
    EXPECT_EQ(server->stats().protocol_errors, 2u);
}

TEST_F(ModbusTcpTest, UnitIdFilter) {
    rtos::net::ModbusTcpConfig cfg;
    cfg.unit_id = 5;
    start(cfg);
    Client c = connect(40000);
    EXPECT_EQ(exchange(c, adu(1, 6, bytes({0x03, 0x00, 0x00, 0x00, 0x01}))), "");
    EXPECT_EQ(server->stats().other_unit, 1u);
    EXPECT_FALSE(exchange(c, adu(2, 5, bytes({0x03, 0x00, 0x00, 0x00, 0x01}))).empty());
    EXPECT_FALSE(exchange(c, adu(3, 0xFF, bytes({0x03, 0x00, 0x00, 0x00, 0x01}))).empty());
}

TEST_F(ModbusTcpTest, IdleClientIsClosed) {
    rtos::net::ModbusTcpConfig cfg;
    cfg.idle_timeout_ms = 5000;
    start(cfg);
    Client c = connect(40000);
    EXPECT_FALSE(exchange(c, adu(1, 1, bytes({0x03, 0x00, 0x00, 0x00, 0x01}))).empty());
    poll(4999);
    server->service();
    EXPECT_FALSE(closed_by_server());
    poll(1);
    server->service();
    poll();
    EXPECT_TRUE(closed_by_server());
}

TEST_F(ModbusTcpTest, ServesSeveralClients) {
    Client a = connect(40000);
    Client b = connect(40001);
    EXPECT_EQ(server->clients(), 2u);
    EXPECT_EQ(exchange(a, adu(1, 1, bytes({0x06, 0x00, 0x00, 0x00, 0x0A}))).size(), 12u);
    EXPECT_EQ(exchange(b, adu(1, 1, bytes({0x03, 0x00, 0x00, 0x00, 0x01}))),
              str(adu(1, 1, bytes({0x03, 0x02, 0x00, 0x0A}))));
}

}  // namespace
