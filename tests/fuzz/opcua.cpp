// SPDX-License-Identifier: MIT
// AFL++ harness: the OPC UA server's protocol side (rtos/opcua: Hello, SecureChannel, Session, the
// services, subscriptions) - one Connection as rtos::net::OpcUaServer drives it. Input: the client's
// byte stream, cut into UA TCP messages by their header exactly as the server does (MessageSize
// 8..8192, else the connection ends). Between the messages the clock advances 100 ms and Tick()
// runs (sampling, publishing). The server identity is the test device's (certs/, RSA-2048).
// Seeds: tests/fuzz/corpus/opcua - the client streams of opcua_test.cpp (RTOS_FUZZ_CORPUS).
#include <wolfssl/wolfcrypt/settings.h>

#include <array>
#include <cstddef>
#include <vector>

#include "fuzz_main.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/net/tls.hpp"
#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/connection.hpp"
#include "rtos/opcua/ns0_base.hpp"
#include "rtos/opcua/registers.hpp"
#include "rtos/opcua/security_crypto.hpp"

namespace {

namespace ua = rtos::opcua;

constexpr unsigned char server_cert[] = {
#embed "opcua.der"
};
constexpr unsigned char server_key[] = {
#embed "opcua.key.der"
};
constexpr unsigned char ca_cert[] = {
#embed "ca.der"
};

// the application nodes: a register (read/write through the map), a read-only constant
using Bank = rtos::modbus::RegisterBank<8, 8>;
constexpr ua::NodeId kFcu(1, 1000), kLed(1, 5000), kUptime(1, 5001), kSerial(1, 5003);
constexpr ua::registers::Register kLedRegister{rtos::modbus::Table::Holding, 0};
constexpr ua::registers::Register kUptimeRegister{rtos::modbus::Table::Input, 0};
constexpr std::array kAppNodes = {
    ua::Node{.nodeId = kFcu, .nodeClass = ua::NodeClass::Object, .browseName = ua::QualifiedName(1, "FCU"),
             .displayName = ua::LocalizedText("en-US", "FCU")},
    ua::Node{.nodeId = kLed, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "Led"),
             .displayName = ua::LocalizedText("en-US", "Led"), .dataType = ua::NodeId(0, ua::ns0::DataType_UInt16),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE,
             .readCallback = &ua::registers::ReadUInt16, .writeCallback = &ua::registers::WriteUInt16,
             .userContext = ua::registers::Context(kLedRegister)},
    ua::Node{.nodeId = kUptime, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "Uptime"),
             .displayName = ua::LocalizedText("en-US", "Uptime"), .dataType = ua::NodeId(0, ua::ns0::DataType_UInt32),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ, .readCallback = &ua::registers::ReadUInt32,
             .userContext = ua::registers::Context(kUptimeRegister)},
    ua::Node{.nodeId = kSerial, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "Serial"),
             .displayName = ua::LocalizedText("en-US", "Serial"), .dataType = ua::NodeId(0, ua::ns0::DataType_UInt16),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ, .staticValue = ua::Variant::Of(ua::UInt16{4711})},
};
constexpr std::array kAppReferences = {
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasTypeDefinition), true, ua::NodeId(0, ua::ns0::FolderType)},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kLed},
    ua::ReferenceEntry{kLed, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kUptime},
    ua::ReferenceEntry{kUptime, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kSerial},
    ua::ReferenceEntry{kSerial, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
};
constexpr auto kNodes = ua::JoinNodes(ua::kBaseNodes, kAppNodes);
constexpr auto kReferences = ua::JoinReferences(ua::kBaseReferences, kAppReferences);
constexpr ua::AddressSpace kSpace(kNodes, kReferences);

// Deterministic: the same input takes the same path (nonces, timestamps).
std::uint64_t g_now_ms = 0;
std::uint32_t g_entropy = 1;

bool entropy(std::span<std::byte> out) noexcept {
    for (auto& b : out) {
        g_entropy = g_entropy * 1103515245u + 12345u;
        b = static_cast<std::byte>(g_entropy >> 16);
    }
    return true;
}

struct Transport final : ua::ITransport {
    bool closed = false;
    std::size_t sent = 0;
    bool Send(std::span<const ua::Byte> data) override {
        if (data.size() < ua::TCP_HEADER_SIZE) __builtin_trap();  // every response is a framed message
        sent += data.size();
        return true;
    }
    void RequestClose() override { closed = true; }
    bool CanSend() const override { return true; }
};

struct Server {
    Bank bank;
    ua::ServerIdentity identity;
    alignas(16) std::array<std::byte, 48 * 1024> crypto_memory{};

    Server() {
        if (!rtos::net::TlsServerContext::initialize_library(&entropy, []() noexcept -> std::uint32_t { return 0; })) __builtin_trap();
        if (!ua::security::InitCrypto(crypto_memory)) __builtin_trap();
        identity.certificateDer = {server_cert, sizeof server_cert};
        identity.caCertificateDer = {ca_cert, sizeof ca_cert};
        if (!ua::security::LoadServerKey({server_key, sizeof server_key}, identity.privateKey)) __builtin_trap();
        if (!ua::security::SetTrustedCa({ca_cert, sizeof ca_cert})) __builtin_trap();
        ua::SetServerIdentity(identity);
        ua::SetClock([]() noexcept -> std::uint64_t { return 1'790'000'000'000ull + g_now_ms; },
                     []() noexcept -> std::uint64_t { return g_now_ms; });
        ua::registers::Bind(bank);
    }
};

}  // namespace

void fuzz_one(std::span<const std::uint8_t> input) {
    static Server server;
    static Transport transport;
    static ua::Connection connection;  // 8 KB send buffer + session state: one, reused like a server slot
    static std::array<ua::Byte, 8192> rx{};
    g_now_ms = 0;
    g_entropy = 1;
    transport = Transport{};
    connection.Reset(0, &transport, &kSpace, "opc.tcp://192.168.77.2:4840");

    std::size_t pos = 0;
    while (!transport.closed && input.size() - pos >= ua::TCP_HEADER_SIZE) {
        ua::TcpHeader header;
        if (!ua::PeekTcpHeader(input.subspan(pos), header) || header.messageSize < ua::TCP_HEADER_SIZE ||
            header.messageSize > rx.size() || header.messageSize > input.size() - pos) {
            break;  // the server drops the connection here
        }
        // the server hands over its receive buffer (decrypted in place)
        std::copy_n(input.begin() + static_cast<std::ptrdiff_t>(pos), header.messageSize, rx.begin());
        connection.HandleMessage(std::span(rx).first(header.messageSize));
        pos += header.messageSize;
        g_now_ms += 100;
        connection.Tick();
    }
    for (int i = 0; i < 5 && !transport.closed; ++i) {  // publishing cycles after the last request
        g_now_ms += 1000;
        connection.Tick();
    }
}
