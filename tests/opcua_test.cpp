// SPDX-License-Identifier: MIT
//
// OPC UA server (rtos::net::OpcUaServer + the FCU's opcua_native library) over the stack,
// against a client written here: Hello/Acknowledge, SecureChannel with SecurityPolicy None
// (read-only) and Basic256Sha256/Sign (client side crypto with wolfCrypt: OAEP-SHA1,
// PKCS#1 v1.5 SHA-256, P_SHA256, HMAC-SHA256), sessions, Read/Write/Browse/GetEndpoints,
// variables shared with Modbus registers and variables of their own, trust (client certificates must chain to the CA), tampered
// messages, framing limits, and the OpenSecureChannel as background work.
#include <gtest/gtest.h>

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/asn_public.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssl/wolfcrypt/random.h>
#include <wolfssl/wolfcrypt/rsa.h>

#include <array>
#include <cstring>
#include <ctime>
#include <functional>
#include <optional>
#include <random>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "fuzz_support.hpp"
#include "net_fixture.hpp"
#include "rtos/modbus/register_bank.hpp"
#include "rtos/net/opcua_server.hpp"
#include "rtos/net/tls.hpp"
#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/ns0_base.hpp"
#include "rtos/opcua/registers.hpp"
#include "rtos/opcua/service_header.hpp"

namespace {

using namespace test;
namespace ua = rtos::opcua;
using ua::Byte;
using UaBytes = std::vector<Byte>;

constexpr unsigned char server_cert[] = {
#embed "opcua.der"
};
constexpr unsigned char server_key[] = {
#embed "opcua.key.der"
};
constexpr unsigned char ca_cert[] = {
#embed "ca.der"
};
constexpr unsigned char client_cert[] = {
#embed "opcua_client.der"
};
constexpr unsigned char client_key[] = {
#embed "opcua_client.key.der"
};
constexpr unsigned char rogue_cert[] = {
#embed "opcua_rogue.der"
};
constexpr unsigned char rogue_key[] = {
#embed "opcua_rogue.key.der"
};

template <std::size_t N>
std::span<const std::byte> der(const unsigned char (&a)[N]) {
    return {reinterpret_cast<const std::byte*>(a), N};
}

// ---- address space: Objects/FCU --------------------------------------------------------------
// Shared with Modbus (registers.hpp): a writable LED (holding 0) and a read-only 32-bit uptime
// (inputs 0..1). OPC UA only: a writable setpoint in its own variable and a constant serial
// number - a server does not have to map anything onto Modbus.

using Bank = rtos::modbus::RegisterBank<8, 8>;
constexpr ua::NodeId kFcu(1, 1000), kLed(1, 5000), kUptime(1, 5001), kSetpoint(1, 5002), kSerial(1, 5003);
constexpr ua::registers::Register kLedRegister{rtos::modbus::Table::Holding, 0};
constexpr ua::registers::Register kUptimeRegister{rtos::modbus::Table::Input, 0};

ua::UInt16 g_setpoint = 0;  ///< the setpoint's own storage
constexpr ua::UInt16 kSerialNumber = 4711;

ua::Variant read_setpoint(void* context) { return ua::Variant::Of(*static_cast<ua::UInt16*>(context)); }
bool write_setpoint(void* context, const ua::Variant& value) {
    const ua::UInt16* v = value.GetIf<ua::UInt16>();
    if (v == nullptr || *v > 1000) {
        return false;  // the application's own range check
    }
    *static_cast<ua::UInt16*>(context) = *v;
    return true;
}

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
    ua::Node{.nodeId = kSetpoint, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "Setpoint"),
             .displayName = ua::LocalizedText("en-US", "Setpoint"),
             .dataType = ua::NodeId(0, ua::ns0::DataType_UInt16),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE,
             .readCallback = &read_setpoint, .writeCallback = &write_setpoint, .userContext = &g_setpoint},
    ua::Node{.nodeId = kSerial, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "Serial"),
             .displayName = ua::LocalizedText("en-US", "Serial"), .dataType = ua::NodeId(0, ua::ns0::DataType_UInt16),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ, .staticValue = ua::Variant::Of(kSerialNumber)},
};
constexpr std::array kAppReferences = {
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasTypeDefinition), true, ua::NodeId(0, ua::ns0::FolderType)},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kLed},
    ua::ReferenceEntry{kLed, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kUptime},
    ua::ReferenceEntry{kUptime, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kSetpoint},
    ua::ReferenceEntry{kSetpoint, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
    ua::ReferenceEntry{kFcu, ua::NodeId(0, ua::ns0::HasComponent), true, kSerial},
    ua::ReferenceEntry{kSerial, ua::NodeId(0, ua::ns0::HasComponent), false, kFcu},
};
constexpr auto kNodes = ua::JoinNodes(ua::kBaseNodes, kAppNodes);
constexpr auto kReferences = ua::JoinReferences(ua::kBaseReferences, kAppReferences);
static_assert(ua::AllTypedNodesHaveDataType(kNodes));
static_assert(ua::AllDataTypesResolve(kNodes));
static_assert(ua::AllReferencesResolve(kNodes, kReferences));
constexpr ua::AddressSpace kSpace(kNodes, kReferences);

constexpr std::string_view kEndpoint = "opc.tcp://192.168.77.2:4840";
constexpr char kBasic256Sha256[] = "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";
constexpr char kNone[] = "http://opcfoundation.org/UA/SecurityPolicy#None";

bool host_entropy(std::span<std::byte> out) noexcept {
    static std::mt19937 rng(7);
    for (auto& b : out) {
        b = static_cast<std::byte>(rng());
    }
    return true;
}

// ---- client side crypto (wolfCrypt) ------------------------------------------------------------

class ClientCrypto {
public:
    static ClientCrypto& instance() {
        static ClientCrypto c;
        return c;
    }
    WOLFSSL_HEAP_HINT* heap = nullptr;
    WC_RNG rng{};

    /// RSA public key from a certificate.
    bool public_key(std::span<const Byte> cert, RsaKey& key) {
        std::array<Byte, 512> spki{};
        word32 size = spki.size();
        word32 idx = 0;
        return wc_InitRsaKey_ex(&key, heap, INVALID_DEVID) == 0 &&
               wc_GetSubjectPubKeyInfoDerFromCert(cert.data(), (word32)cert.size(), spki.data(), &size) == 0 &&
               wc_RsaPublicKeyDecode(spki.data(), &idx, &key, size) == 0;
    }
    bool private_key(std::span<const Byte> der, RsaKey& key) {
        word32 idx = 0;
        if (wc_GetPkcs8TraditionalOffset(const_cast<Byte*>(der.data()), &idx, (word32)der.size()) < 0) {
            idx = 0;
        }
        return wc_InitRsaKey_ex(&key, heap, INVALID_DEVID) == 0 &&
               wc_RsaPrivateKeyDecode(der.data(), &idx, &key, (word32)der.size()) == 0 && wc_RsaSetRNG(&key, &rng) == 0;
    }

private:
    ClientCrypto() {
        alignas(16) static std::array<unsigned char, 64 * 1024> memory{};
        wc_LoadStaticMemory(&heap, memory.data(), memory.size(), WOLFMEM_GENERAL, 1);
        if (wolfSSL_GetGlobalHeapHint() == nullptr) {
            wolfSSL_SetGlobalHeapHint(heap);  // the DRBG's seed test allocates without a hint
        }
        EXPECT_EQ(wc_InitRng_ex(&rng, heap, INVALID_DEVID), 0);
    }
};

/// DigestInfo(SHA-256) for PKCS#1 v1.5.
UaBytes digest_info(std::span<const Byte> message) {
    UaBytes out = {0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65,
                   0x03, 0x04, 0x02, 0x01, 0x05, 0x00, 0x04, 0x20};
    out.resize(out.size() + 32);
    ua::security::Sha256(message, out.data() + 19);
    return out;
}

// ---- message encoding helpers ---------------------------------------------------------------

bool is_message(const UaBytes& msg, const char* type) {
    ua::TcpHeader h;
    return ua::PeekTcpHeader(msg, h) && h.Is(type);
}

void request_header(ua::ByteWriter& w, const ua::NodeId& auth, ua::UInt32 handle) {
    ua::EncodeNodeId(w, auth);
    w.WriteDateTime(0);
    w.WriteUInt32(handle);
    w.WriteUInt32(0);
    w.WriteString(ua::String::Null());
    w.WriteUInt32(10000);
    ua::EncodeNullExtensionObject(w);
}

/// A decoded service response: TypeId, ServiceResult, and a reader at the service body.
struct Response {
    UaBytes bytes;
    ua::NodeId type;
    ua::StatusCode result = ua::StatusCode::BadInternalError;
    std::size_t body = 0;
    ua::ByteReader reader() const { return ua::ByteReader(std::span(bytes).subspan(body)); }
};

bool parse_response(UaBytes body, Response& out) {
    out.bytes = std::move(body);
    ua::ByteReader r(out.bytes);
    ua::DateTime ts = 0;
    ua::UInt32 handle = 0;
    Byte diagnostics = 0;
    ua::Int32 strings = 0;
    if (!ua::DecodeNodeId(r, out.type) || !r.ReadDateTime(ts) || !r.ReadUInt32(handle) ||
        !r.ReadStatusCode(out.result) || !r.ReadByte(diagnostics) || !r.ReadInt32(strings) ||
        !ua::DecodeAndDiscardExtensionObject(r)) {
        return false;
    }
    out.body = r.Position();
    return true;
}

// ---- fixture ------------------------------------------------------------------------------------

using Server = rtos::net::OpcUaServer<TestStack, 2>;

class OpcUaTest : public NetTest {
protected:
    static constexpr std::uint16_t port = 4840;
    alignas(16) static inline std::array<std::byte, 48 * 1024> crypto_memory{};
    Bank bank;
    Server server{stack, kSpace, kEndpoint};

    static void SetUpTestSuite() {
        ASSERT_TRUE(rtos::net::TlsServerContext::initialize_library(&host_entropy,
                                                                    []() noexcept -> std::uint32_t { return 0; }));
        (void)ClientCrypto::instance();
    }

    void SetUp() override {
        NetTest::SetUp();
        ASSERT_EQ(server.init({der(server_cert), der(server_key), der(ca_cert)}, crypto_memory),
                  rtos::net::OpcUaInitResult::Ok);
        ua::registers::Bind(bank);
        g_setpoint = 0;
        ASSERT_TRUE(server.start(port));
        connect();
    }

    // ---- TCP --------------------------------------------------------------------------------
    std::uint16_t client_port = 41000;
    std::uint32_t snd = 9000, iss = 0, received = 0;
    UaBytes rx;
    bool closed = false;

    void connect() {
        ++client_port;
        snd = 9000;
        received = 0;
        rx.clear();
        closed = false;
        deliver(tcp(client_port, port, snd, 0, tcp_flag::syn, {}, 65535, 1460));
        iss = sent().at(0).seq();
        ++snd;
        deliver(tcp(client_port, port, snd, iss + 1, tcp_flag::ack, {}, 65535));
    }

    /// Fuzzing: mutates every outgoing message (signed MSGs before signing, see call()).
    std::function<void(UaBytes&)> fuzz;

    void send(std::span<const Byte> data, bool mutate = true) {
        if (fuzz && mutate) {
            UaBytes m(data.begin(), data.end());
            fuzz(m);
            send(m, false);
            return;
        }
        for (std::size_t done = 0; done < data.size();) {
            const std::size_t n = std::min<std::size_t>(data.size() - done, 1400);
            deliver(tcp(client_port, port, snd, iss + 1 + received, tcp_flag::ack | tcp_flag::psh,
                        {reinterpret_cast<const char*>(data.data() + done), n}, 65535));
            snd += static_cast<std::uint32_t>(n);
            done += n;
        }
    }

    /// Run the server (background steps included) until a whole message arrived.
    std::optional<UaBytes> receive() {
        for (int round = 0; round < 200; ++round) {
            server.service();
            while (server.background_pending()) {
                server.background_step();
            }
            poll(1);
            bool any = false;
            for (const Frame& f : sent()) {
                if (f.ethertype() != ethertype_ipv4 || f.ip_protocol() != ip_proto_tcp || f.udp_dst_port() != client_port) {
                    continue;
                }
                closed = closed || (f.flags() & (tcp_flag::fin | tcp_flag::rst)) != 0;
                const auto p = f.tcp_payload();
                if (!p.empty() && f.seq() == iss + 1 + received) {
                    rx.insert(rx.end(), p.begin(), p.end());
                    received += static_cast<std::uint32_t>(p.size());
                    any = true;
                }
            }
            if (any) {
                deliver(tcp(client_port, port, snd, iss + 1 + received, tcp_flag::ack, {}, 65535));
            }
            ua::TcpHeader h;
            if (rx.size() >= 8 && ua::PeekTcpHeader(rx, h) && rx.size() >= h.messageSize) {
                UaBytes msg(rx.begin(), rx.begin() + h.messageSize);
                rx.erase(rx.begin(), rx.begin() + h.messageSize);
                return msg;
            }
            if (closed && !any) {
                return std::nullopt;
            }
        }
        return std::nullopt;
    }

    void reset_connection() { deliver(tcp(client_port, port, snd, iss + 1 + received, tcp_flag::rst, {}, 0)); }

    /// A second client: swap() exchanges the connection and channel state with the fixture's.
    struct Peer {
        std::uint16_t client_port = 0;
        std::uint32_t snd = 0, iss = 0, received = 0;
        UaBytes rx;
        bool closed = false, secured = false;
        ua::UInt32 channel = 0, token = 0, sequence = 0, request = 0;
        std::array<Byte, 32> client_signing{}, server_signing{};
        ua::NodeId auth_token;
    };
    void swap(Peer& o) {
        using std::swap;
        swap(o.client_port, client_port);
        swap(o.snd, snd);
        swap(o.iss, iss);
        swap(o.received, received);
        swap(o.rx, rx);
        swap(o.closed, closed);
        swap(o.secured, secured);
        swap(o.channel, channel);
        swap(o.token, token);
        swap(o.sequence, sequence);
        swap(o.request, request);
        swap(o.client_signing, client_signing);
        swap(o.server_signing, server_signing);
        swap(o.auth_token, auth_token);
    }

    // ---- UA TCP ------------------------------------------------------------------------------
    std::array<Byte, 16384> buffer{};
    ua::UInt32 channel = 0, token = 0, sequence = 0, request = 0;
    bool secured = false;
    std::array<Byte, 32> client_signing{}, server_signing{};
    ua::NodeId auth_token;

    std::optional<UaBytes> hello(ua::UInt32 receive_size = 65535) {
        ua::ByteWriter w(buffer);
        ua::EncodeTcpHeader(w, "HEL", ua::ChunkType::Final, 0);
        w.WriteUInt32(0);
        w.WriteUInt32(receive_size);
        w.WriteUInt32(65535);
        w.WriteUInt32(0);
        w.WriteUInt32(0);
        w.WriteString(kEndpoint);
        w.PatchUInt32(4, static_cast<ua::UInt32>(w.Position()));
        send(w.Written());
        return receive();
    }

    /// The OpenSecureChannelRequest body (after the sequence header).
    void open_body(ua::ByteWriter& w, ua::Int32 mode, std::span<const Byte> nonce) {
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::OpenSecureChannelRequest));
        request_header(w, ua::NodeId(), 1);
        w.WriteUInt32(0);  // protocol version
        w.WriteInt32(0);   // Issue
        w.WriteInt32(mode);
        w.WriteByteString(std::string_view(reinterpret_cast<const char*>(nonce.data()), nonce.size()), nonce.empty());
        w.WriteUInt32(600000);
    }

    bool open_none() {
        ua::ByteWriter w(buffer);
        ua::EncodeTcpHeader(w, "OPN", ua::ChunkType::Final, 0);
        w.WriteUInt32(0);
        w.WriteString(kNone);
        w.WriteByteString({}, true);
        w.WriteByteString({}, true);
        w.WriteUInt32(++sequence);
        w.WriteUInt32(++request);
        open_body(w, 1, {});
        w.PatchUInt32(4, static_cast<ua::UInt32>(w.Position()));
        send(w.Written());
        const auto reply = receive();
        if (!reply || !is_message(*reply, "OPN")) {
            return false;
        }
        ua::ByteReader r(*reply);
        r.Skip(8);
        r.ReadUInt32(channel);
        ua::String policy;
        std::string_view cert, thumb;
        bool null1 = true, null2 = true;
        r.ReadString(policy);
        r.ReadByteString(cert, null1);
        r.ReadByteString(thumb, null2);
        r.Skip(8);  // sequence header
        UaBytes rest(r.RemainingSpan().begin(), r.RemainingSpan().end());
        Response resp;
        if (!parse_response(rest, resp) || !ua::IsGood(resp.result)) {
            return false;
        }
        ua::ByteReader b = resp.reader();
        ua::UInt32 version = 0;
        b.ReadUInt32(version);
        b.ReadUInt32(channel);
        b.ReadUInt32(token);
        secured = false;
        return b.Ok();
    }

    /// Basic256Sha256 / Sign with `cert`/`key` as the client identity.
    /// `after_send` runs between sending the OPN and receiving the answer.
    bool open_basic(std::span<const unsigned char> cert, std::span<const unsigned char> key_der,
                    const std::function<void()>& after_send = {}) {
        auto& cc = ClientCrypto::instance();
        RsaKey server_pub, client_priv, client_pub;
        if (!cc.public_key(std::span<const Byte>(server_cert, sizeof server_cert), server_pub) ||
            !cc.private_key(std::span<const Byte>(key_der.data(), key_der.size()), client_priv) ||
            !cc.public_key(std::span<const Byte>(cert.data(), cert.size()), client_pub)) {
            return false;
        }
        std::array<Byte, 32> client_nonce{};
        wc_RNG_GenerateBlock(&cc.rng, client_nonce.data(), client_nonce.size());

        // header + asymmetric security header
        ua::ByteWriter w(buffer);
        ua::EncodeTcpHeader(w, "OPN", ua::ChunkType::Final, 0);
        w.WriteUInt32(0);
        w.WriteString(kBasic256Sha256);
        w.WriteByteString(std::string_view(reinterpret_cast<const char*>(cert.data()), cert.size()), false);
        std::array<Byte, 20> thumbprint{};
        ua::security::Sha1(std::span<const Byte>(server_cert, sizeof server_cert), thumbprint.data());
        w.WriteByteString(std::string_view(reinterpret_cast<const char*>(thumbprint.data()), 20), false);
        const std::size_t header_len = w.Position();

        // plaintext: sequence header + body + padding + signature (client key: 256 bytes)
        std::array<Byte, 1024> body_buf{};
        ua::ByteWriter b(body_buf);
        b.WriteUInt32(++sequence);
        b.WriteUInt32(++request);
        open_body(b, 2, client_nonce);
        const std::size_t plain_block = 256 - 42;
        const std::size_t base = b.Position() + 1 + 256;
        const std::size_t pad = (plain_block - base % plain_block) % plain_block;
        const std::size_t plain_len = base + pad;
        const std::size_t blocks = plain_len / plain_block;
        w.PatchUInt32(4, static_cast<ua::UInt32>(header_len + blocks * 256));
        UaBytes plain(body_buf.begin(), body_buf.begin() + static_cast<std::ptrdiff_t>(b.Position()));
        plain.push_back(static_cast<Byte>(pad));
        plain.insert(plain.end(), pad, static_cast<Byte>(pad));
        UaBytes to_sign(w.Written().begin(), w.Written().end());
        to_sign.insert(to_sign.end(), plain.begin(), plain.end());
        const UaBytes info = digest_info(to_sign);
        std::array<Byte, 256> signature{};
        if (wc_RsaSSL_Sign(info.data(), (word32)info.size(), signature.data(), 256, &client_priv, &cc.rng) != 256) {
            return false;
        }
        plain.insert(plain.end(), signature.begin(), signature.end());
        for (std::size_t i = 0; i < blocks; ++i) {
            std::span<Byte> out = w.Reserve(256);
            if (wc_RsaPublicEncrypt_ex(plain.data() + i * plain_block, (word32)plain_block, out.data(), 256, &server_pub,
                                       &cc.rng, WC_RSA_OAEP_PAD, WC_HASH_TYPE_SHA, WC_MGF1SHA1, nullptr, 0) != 256) {
                return false;
            }
        }
        send(w.Written());
        if (after_send) {
            after_send();
        }
        const auto reply = receive();
        if (!reply || !is_message(*reply, "OPN")) {
            return false;
        }

        // response: decrypt with our key, verify the server's signature, derive the keys
        ua::ByteReader r(*reply);
        r.Skip(8);
        ua::UInt32 ch = 0;
        ua::String policy;
        std::string_view their_cert, their_thumb;
        bool n1 = true, n2 = true;
        r.ReadUInt32(ch);
        r.ReadString(policy);
        r.ReadByteString(their_cert, n1);
        r.ReadByteString(their_thumb, n2);
        const std::size_t clear_len = r.Position();
        const auto cipher = r.RemainingSpan();
        if (cipher.size() % 256 != 0) {
            return false;
        }
        UaBytes clear(cipher.size() / 256 * plain_block);
        for (std::size_t i = 0; i < cipher.size() / 256; ++i) {
            std::array<Byte, 256> block{};
            if (wc_RsaPrivateDecrypt_ex(cipher.data() + i * 256, 256, block.data(), (word32)block.size(),
                                        &client_priv, WC_RSA_OAEP_PAD, WC_HASH_TYPE_SHA, WC_MGF1SHA1, nullptr,
                                        0) != (int)plain_block) {
                return false;
            }
            std::memcpy(clear.data() + i * plain_block, block.data(), plain_block);
        }
        const std::size_t sig_start = clear.size() - 256;
        UaBytes signed_part(reply->begin(), reply->begin() + static_cast<std::ptrdiff_t>(clear_len));
        signed_part.insert(signed_part.end(), clear.begin(), clear.begin() + static_cast<std::ptrdiff_t>(sig_start));
        const UaBytes expected = digest_info(signed_part);
        std::array<Byte, 256> decoded{};
        if (wc_RsaSSL_Verify(clear.data() + sig_start, 256, decoded.data(), 256, &server_pub) != (int)expected.size() ||
            std::memcmp(decoded.data(), expected.data(), expected.size()) != 0) {
            return false;
        }
        const std::size_t padding = clear[sig_start - 1];
        UaBytes body(clear.begin() + 8, clear.begin() + static_cast<std::ptrdiff_t>(sig_start - 1 - padding));
        Response resp;
        if (!parse_response(body, resp) || !ua::IsGood(resp.result)) {
            return false;
        }
        ua::ByteReader rb = resp.reader();
        ua::UInt32 version = 0;
        ua::DateTime created = 0;
        ua::UInt32 lifetime = 0;
        std::string_view server_nonce;
        bool nonce_null = true;
        rb.ReadUInt32(version);
        rb.ReadUInt32(channel);
        rb.ReadUInt32(token);
        rb.ReadDateTime(created);
        rb.ReadUInt32(lifetime);
        rb.ReadByteString(server_nonce, nonce_null);
        if (!rb.Ok() || server_nonce.size() != 32) {
            return false;
        }
        const auto sn = std::span<const Byte>(reinterpret_cast<const Byte*>(server_nonce.data()), 32);
        ua::security::PSha256(sn, client_nonce, client_signing);
        ua::security::PSha256(client_nonce, sn, server_signing);
        wc_FreeRsaKey(&server_pub);
        wc_FreeRsaKey(&client_priv);
        wc_FreeRsaKey(&client_pub);
        secured = true;
        return true;
    }

    /// One service call (MSG); `tamper`: flip a byte of the signed request; `wait`: false
    /// only sends the request.
    std::optional<Response> call(const std::function<void(ua::ByteWriter&)>& body, bool tamper = false,
                                 bool wait = true) {
        ua::ByteWriter w(buffer);
        ua::EncodeTcpHeader(w, "MSG", ua::ChunkType::Final, 0);
        w.WriteUInt32(channel);
        w.WriteUInt32(token);
        w.WriteUInt32(++sequence);
        w.WriteUInt32(++request);
        body(w);
        UaBytes msg(w.Written().begin(), w.Written().end());
        if (secured) {
            if (fuzz) {  // a mutated body that is signed properly reaches the decoder
                fuzz(msg);
                msg.resize(std::max<std::size_t>(msg.size(), 24));
            }
            ua::ByteWriter size{std::span(msg).subspan(4, 4)};
            size.WriteUInt32(static_cast<ua::UInt32>(msg.size() + 32));
            std::array<Byte, 32> mac{};
            ua::security::HmacSha256(client_signing, msg, mac.data());
            msg.insert(msg.end(), mac.begin(), mac.end());
        } else {
            ua::ByteWriter size{std::span(msg).subspan(4, 4)};
            size.WriteUInt32(static_cast<ua::UInt32>(msg.size()));
        }
        if (tamper) {
            msg[30] ^= 0x01;
        }
        send(msg, !secured);
        if (!wait) {
            return std::nullopt;
        }
        const auto reply = receive();
        if (!reply || !is_message(*reply, "MSG")) {
            return std::nullopt;
        }
        std::size_t end = reply->size();
        if (secured) {
            end -= 32;
            std::array<Byte, 32> mac{};
            ua::security::HmacSha256(server_signing, std::span(*reply).first(end), mac.data());
            EXPECT_EQ(std::memcmp(mac.data(), reply->data() + end, 32), 0) << "server signature";
        }
        Response resp;
        if (!parse_response(UaBytes(reply->begin() + 24, reply->begin() + static_cast<std::ptrdiff_t>(end)), resp)) {
            return std::nullopt;
        }
        return resp;
    }

    bool session() {
        const auto created = call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::CreateSessionRequest));
            request_header(w, ua::NodeId(), 2);
            w.WriteString("urn:rtoos:test-client");
            w.WriteString("urn:rtoos:test-client");
            ua::EncodeLocalizedText(w, ua::LocalizedText("en-US", "test client"));
            w.WriteInt32(1);  // Client
            w.WriteString(ua::String::Null());
            w.WriteString(ua::String::Null());
            w.WriteInt32(-1);
            w.WriteString(ua::String::Null());  // serverUri
            w.WriteString(kEndpoint);
            w.WriteString("session");
            std::array<Byte, 32> nonce{};
            w.WriteByteString(std::string_view(reinterpret_cast<const char*>(nonce.data()), 32), false);
            if (secured) {
                w.WriteByteString(std::string_view(reinterpret_cast<const char*>(client_cert), sizeof client_cert), false);
            } else {
                w.WriteByteString({}, true);
            }
            w.WriteDouble(60000);
            w.WriteUInt32(0);
        });
        if (!created || !ua::IsGood(created->result)) {
            return false;
        }
        ua::ByteReader r = created->reader();
        ua::NodeId session_id;
        ua::DecodeNodeId(r, session_id);
        ua::DecodeNodeId(r, auth_token);
        const auto activated = call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::ActivateSessionRequest));
            request_header(w, auth_token, 3);
            w.WriteString(ua::String::Null());  // client signature
            w.WriteByteString({}, true);
            w.WriteInt32(0);   // software certificates
            w.WriteInt32(0);   // locale ids
            ua::EncodeNodeId(w, ua::NodeId(0, 321));  // AnonymousIdentityToken, binary
            w.WriteByte(0x01);
            std::array<Byte, 64> token_body{};
            ua::ByteWriter t(token_body);
            t.WriteString("anonymous");
            w.WriteByteString(std::string_view(reinterpret_cast<const char*>(token_body.data()), t.Position()), false);
            w.WriteString(ua::String::Null());  // user token signature
            w.WriteByteString({}, true);
        });
        return activated && ua::IsGood(activated->result);
    }

    /// Read the Value attribute: status and value.
    std::pair<ua::StatusCode, ua::Variant> read(const ua::NodeId& node) {
        const auto r = call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::ReadRequest));
            request_header(w, auth_token, 4);
            w.WriteDouble(0);
            w.WriteInt32(0);
            w.WriteInt32(1);
            ua::EncodeNodeId(w, node);
            w.WriteUInt32(13);  // Value
            w.WriteString(ua::String::Null());
            ua::EncodeQualifiedName(w, ua::QualifiedName(0, ""));
        });
        if (!r) {
            return {ua::StatusCode::BadInternalError, {}};
        }
        if (!ua::IsGood(r->result)) {
            return {r->result, {}};
        }
        ua::ByteReader b = r->reader();
        ua::Int32 count = 0;
        Byte mask = 0;
        b.ReadInt32(count);
        b.ReadByte(mask);
        ua::Variant v;
        ua::StatusCode status = ua::StatusCode::Good;
        if (mask & 0x01) {
            ua::DecodeVariant(b, v);
        }
        if (mask & 0x02) {
            b.ReadStatusCode(status);
        }
        return {status, v};
    }

    /// Write a UInt16 Value: the service result, else the result of the one operation.
    ua::StatusCode write(const ua::NodeId& node, ua::UInt16 value) {
        const auto r = call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::WriteRequest));
            request_header(w, auth_token, 5);
            w.WriteInt32(1);
            ua::EncodeNodeId(w, node);
            w.WriteUInt32(13);
            w.WriteString(ua::String::Null());
            w.WriteByte(0x01);
            ua::EncodeVariant(w, ua::Variant::Of(value));
        });
        if (!r) {
            return ua::StatusCode::BadInternalError;
        }
        if (!ua::IsGood(r->result)) {
            return r->result;
        }
        ua::ByteReader b = r->reader();
        ua::Int32 count = 0;
        ua::StatusCode status = ua::StatusCode::BadInternalError;
        b.ReadInt32(count);
        b.ReadStatusCode(status);
        return status;
    }
};

// The numeric values of OPC UA Part 6 / StatusCode.csv (the FCU library had 16 of them
// wrong, e.g. BadSecurityModeInsufficient as 0x80770000 = BadTooManySubscriptions).
TEST(OpcUaStatusCodes, HaveTheSpecificationValues) {
    static_assert(std::to_underlying(ua::StatusCode::Good) == 0x00000000u);
    static_assert(std::to_underlying(ua::StatusCode::BadInternalError) == 0x80020000u);
    static_assert(std::to_underlying(ua::StatusCode::BadOutOfMemory) == 0x80030000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNotImplemented) == 0x80400000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNotSupported) == 0x803D0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadInvalidArgument) == 0x80AB0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadOutOfRange) == 0x803C0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNodeIdInvalid) == 0x80330000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNodeIdUnknown) == 0x80340000u);
    static_assert(std::to_underlying(ua::StatusCode::BadAttributeIdInvalid) == 0x80350000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNotReadable) == 0x803A0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNotWritable) == 0x803B0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTypeMismatch) == 0x80740000u);
    static_assert(std::to_underlying(ua::StatusCode::BadUserAccessDenied) == 0x801F0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecurityChecksFailed) == 0x80130000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecurityPolicyRejected) == 0x80550000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecurityModeRejected) == 0x80540000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecurityModeInsufficient) == 0x80E60000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecureChannelIdInvalid) == 0x80220000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecureChannelClosed) == 0x80860000u);
    static_assert(std::to_underlying(ua::StatusCode::BadRequestTypeInvalid) == 0x80530000u);
    static_assert(std::to_underlying(ua::StatusCode::BadRequestHeaderInvalid) == 0x802A0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTimestampsToReturnInvalid) == 0x802B0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSessionIdInvalid) == 0x80250000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSessionClosed) == 0x80260000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSessionNotActivated) == 0x80270000u);
    static_assert(std::to_underlying(ua::StatusCode::BadIdentityTokenInvalid) == 0x80200000u);
    static_assert(std::to_underlying(ua::StatusCode::BadIdentityTokenRejected) == 0x80210000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTooManySessions) == 0x80560000u);
    static_assert(std::to_underlying(ua::StatusCode::BadServiceUnsupported) == 0x800B0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadDecodingError) == 0x80070000u);
    static_assert(std::to_underlying(ua::StatusCode::BadEncodingError) == 0x80060000u);
    static_assert(std::to_underlying(ua::StatusCode::BadEncodingLimitsExceeded) == 0x80080000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNothingToDo) == 0x800F0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNoMatch) == 0x806F0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTcpMessageTypeInvalid) == 0x807E0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTcpMessageTooLarge) == 0x80800000u);
    static_assert(std::to_underlying(ua::StatusCode::BadConnectionClosed) == 0x80AE0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadUnexpectedError) == 0x80010000u);
    static_assert(std::to_underlying(ua::StatusCode::UncertainInitialValue) == 0x40920000u);
}

// The monotonic clock is 64 bits: past 2^32 ms (49.7 days) it keeps counting forward.
TEST(OpcUaClock, MonotonicClockDoesNotWrapAfter49Days) {
    static std::uint64_t ms = 0;
    ua::SetClock(nullptr, []() noexcept { return ms; });
    ms = 0xFFFF'FFFFull;
    const ua::DateTime before = ua::NowMonotonic();
    ms = 0x1'0000'0005ull;
    const ua::DateTime after = ua::NowMonotonic();
    EXPECT_EQ(after - before, 6 * 10'000) << "6 ms in 100 ns units";
    EXPECT_EQ(after, static_cast<ua::DateTime>(0x1'0000'0005ull) * 10'000);
    ua::SetClock(nullptr, nullptr);
}

TEST_F(OpcUaTest, ClientCertificatesAreCheckedAgainstTheCa) {
    namespace sec = ua::security;
    std::array<Byte, 256> modulus{};
    std::array<Byte, 8> exponent{};
    sec::RsaPublicKey key;
    EXPECT_EQ(sec::CheckClientCertificate(std::span<const Byte>(client_cert, sizeof client_cert), 0, modulus, exponent,
                                          key),
              sec::CertificateCheck::Ok);
    EXPECT_EQ(key.modulus.size(), 256u);
    EXPECT_EQ(sec::CheckClientCertificate(std::span<const Byte>(rogue_cert, sizeof rogue_cert), 0, modulus, exponent, key),
              sec::CertificateCheck::Untrusted);
    // Validity: now (certs/ is made by tools/make_certs.py, valid for 13 months), before
    // notBefore (1990) and long after notAfter (2100)
    EXPECT_EQ(sec::CheckClientCertificate(std::span<const Byte>(client_cert, sizeof client_cert),
                                          static_cast<ua::UInt64>(std::time(nullptr)), modulus, exponent, key),
              sec::CertificateCheck::Ok);
    EXPECT_EQ(sec::CheckClientCertificate(std::span<const Byte>(client_cert, sizeof client_cert), 631152000, modulus,
                                          exponent, key),
              sec::CertificateCheck::TimeInvalid);
    EXPECT_EQ(sec::CheckClientCertificate(std::span<const Byte>(client_cert, sizeof client_cert), 4102444800, modulus,
                                          exponent, key),
              sec::CertificateCheck::TimeInvalid);
}

TEST_F(OpcUaTest, HelloIsAcknowledgedWithTheSmallerBuffers) {
    const auto ack = hello(16384);
    ASSERT_TRUE(ack.has_value());
    ASSERT_TRUE(is_message(*ack, "ACK"));
    ua::ByteReader r(*ack);
    r.Skip(8);
    ua::UInt32 version = 0, receive_size = 0, send_size = 0;
    r.ReadUInt32(version);
    r.ReadUInt32(receive_size);
    r.ReadUInt32(send_size);
    EXPECT_EQ(receive_size, 8192u);
    EXPECT_EQ(send_size, 8192u);
}

TEST_F(OpcUaTest, HelloWithTooSmallBuffersIsAnError) {
    const auto err = hello(1024);
    ASSERT_TRUE(err.has_value());
    EXPECT_TRUE(is_message(*err, "ERR"));
}

TEST_F(OpcUaTest, NoneChannelReadsButCannotWrite) {
    bank.set_holding(0, 7);
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const auto [status, value] = read(kLed);
    EXPECT_TRUE(ua::IsGood(status));
    ASSERT_NE(value.GetIf<ua::UInt16>(), nullptr);
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), 7);
    EXPECT_EQ(write(kLed, 1), ua::StatusCode::BadSecurityModeInsufficient);
    EXPECT_EQ(bank.holding(0), 7);
}

TEST_F(OpcUaTest, GetEndpointsOffersBasic256Sha256AndNone) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    const auto r = call([&](ua::ByteWriter& w) {
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::GetEndpointsRequest));
        request_header(w, ua::NodeId(), 9);
        w.WriteString(kEndpoint);
        w.WriteInt32(0);
        w.WriteInt32(0);
    });
    ASSERT_TRUE(r.has_value());
    const std::string text(reinterpret_cast<const char*>(r->bytes.data()), r->bytes.size());
    EXPECT_NE(text.find(kBasic256Sha256), std::string::npos);
    EXPECT_NE(text.find(kNone), std::string::npos);
    EXPECT_NE(text.find("urn:factory-control-unit:opcua-native"), std::string::npos);
}

TEST_F(OpcUaTest, TrustedClientOnASignedChannelWrites) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    EXPECT_EQ(server.stats().opens, 1u);
    ASSERT_TRUE(session());
    EXPECT_TRUE(ua::IsGood(write(kLed, 1)));
    EXPECT_EQ(bank.holding(0), 1);  // Modbus sees the same register
    const auto [status, value] = read(kLed);
    EXPECT_TRUE(ua::IsGood(status));
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), 1);
}

// Shared: one register, two protocols - writes from either side are seen by the other.
TEST_F(OpcUaTest, SharedRegisterIsTheSameValueForModbusAndOpcUa) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    ASSERT_TRUE(session());
    const std::array<std::uint16_t, 1> on{1};
    ASSERT_EQ(bank.write(0, on), rtos::modbus::Exception::None);  // as a Modbus write would
    auto [status, value] = read(kLed);
    ASSERT_TRUE(ua::IsGood(status));
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), 1);
    EXPECT_TRUE(ua::IsGood(write(kLed, 0)));
    EXPECT_EQ(bank.holding(0), 0);
}

// Own values: OPC UA variables need no Modbus register; their writes leave the bank alone.
TEST_F(OpcUaTest, VariablesOfTheirOwnAreIndependentOfModbus) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    ASSERT_TRUE(session());
    std::array<std::uint16_t, 8> holding_before{}, holding_after{};
    ASSERT_EQ(bank.read(rtos::modbus::Table::Holding, 0, holding_before), rtos::modbus::Exception::None);

    EXPECT_TRUE(ua::IsGood(write(kSetpoint, 250)));
    EXPECT_EQ(g_setpoint, 250);
    auto [status, value] = read(kSetpoint);
    ASSERT_TRUE(ua::IsGood(status));
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), 250);
    EXPECT_FALSE(ua::IsGood(write(kSetpoint, 2000))) << "the application's range check rejects it";
    EXPECT_EQ(g_setpoint, 250);

    std::tie(status, value) = read(kSerial);
    ASSERT_TRUE(ua::IsGood(status));
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), kSerialNumber);
    EXPECT_FALSE(ua::IsGood(write(kSerial, 1))) << "a constant is read-only";

    ASSERT_EQ(bank.read(rtos::modbus::Table::Holding, 0, holding_after), rtos::modbus::Exception::None);
    EXPECT_EQ(holding_before, holding_after) << "no Modbus register changed";
}

TEST_F(OpcUaTest, ThirtyTwoBitValueFromTwoRegisters) {
    bank.set_inputs(0, std::array<std::uint16_t, 2>{0x0001, 0x0002});
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const auto [status, value] = read(kUptime);
    ASSERT_NE(value.GetIf<ua::UInt32>(), nullptr);
    EXPECT_EQ(*value.GetIf<ua::UInt32>(), 0x0001'0002u);
    EXPECT_FALSE(ua::IsGood(write(kUptime, 5)));  // read-only node
}

TEST_F(OpcUaTest, UntrustedClientCertificateIsRejected) {
    ASSERT_TRUE(hello());
    EXPECT_FALSE(open_basic(rogue_cert, rogue_key));
    EXPECT_EQ(server.stats().untrusted_clients, 1u);
}

TEST_F(OpcUaTest, TamperedSignedMessageClosesTheChannel) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    ASSERT_TRUE(session());
    const auto r = call(
        [&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::ReadRequest));
            request_header(w, auth_token, 4);
            w.WriteDouble(0);
            w.WriteInt32(0);
            w.WriteInt32(0);
        },
        true);
    EXPECT_FALSE(r.has_value()) << "no answer to a message with a wrong signature";
    (void)receive();
    EXPECT_TRUE(closed);
}

TEST_F(OpcUaTest, MessageLargerThanTheBufferClosesTheConnection) {
    ASSERT_TRUE(hello());
    UaBytes big(16, 0);
    ua::ByteWriter w(big);
    ua::EncodeTcpHeader(w, "MSG", ua::ChunkType::Final, 9000);
    send(big);
    EXPECT_FALSE(receive().has_value());
    EXPECT_TRUE(closed);
    EXPECT_EQ(server.stats().oversized, 1u);
}

TEST_F(OpcUaTest, OpenSecureChannelIsBackgroundWork) {
    ASSERT_TRUE(hello());
    // Deliver an OPN by hand and look at the server before it runs its step.
    ua::ByteWriter w(buffer);
    ua::EncodeTcpHeader(w, "OPN", ua::ChunkType::Final, 0);
    w.WriteUInt32(0);
    w.WriteString(kNone);
    w.WriteByteString({}, true);
    w.WriteByteString({}, true);
    w.WriteUInt32(1);
    w.WriteUInt32(1);
    open_body(w, 1, {});
    w.PatchUInt32(4, static_cast<ua::UInt32>(w.Position()));
    send(w.Written());
    server.service();
    EXPECT_TRUE(server.background_pending()) << "the OPN waits for a background step";
    EXPECT_EQ(server.stats().opens, 0u);
    const auto reply = receive();
    ASSERT_TRUE(reply.has_value());
    EXPECT_TRUE(is_message(*reply, "OPN"));
    EXPECT_EQ(server.stats().opens, 1u);
}

// A Read on one channel is answered in the foreground while an RSA OpenSecureChannel of
// another client still waits for its background step (the RSA work does not delay it).
TEST_F(OpcUaTest, ReadIsAnsweredWhileAnRsaOpenWaits) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    Peer reader;
    swap(reader);  // the fixture now is the second client
    connect();
    ASSERT_TRUE(hello());
    bool answered = false, opn_waited = false;
    ASSERT_TRUE(open_basic(client_cert, client_key,
                           [&] {
                               swap(reader);
                               (void)call(
                                   [&](ua::ByteWriter& w) {
                                       ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::ReadRequest));
                                       request_header(w, auth_token, 4);
                                       w.WriteDouble(0);
                                       w.WriteInt32(0);
                                       w.WriteInt32(1);
                                       ua::EncodeNodeId(w, kLed);
                                       w.WriteUInt32(13);
                                       w.WriteString(ua::String::Null());
                                       ua::EncodeQualifiedName(w, ua::QualifiedName(0, ""));
                                   },
                                   false, false);
                               server.service();  // foreground only
                               opn_waited = server.background_pending();
                               for (const Frame& f : sent()) {
                                   if (f.ethertype() == ethertype_ipv4 && f.ip_protocol() == ip_proto_tcp &&
                                       f.udp_dst_port() == client_port && !f.tcp_payload().empty()) {
                                       answered = true;
                                   }
                               }
                               swap(reader);
                           }));
    EXPECT_TRUE(opn_waited) << "the RSA OpenSecureChannel is background work";
    EXPECT_TRUE(answered) << "the Read was answered before the OpenSecureChannel";
    EXPECT_EQ(server.stats().opens, 2u);
}

// Mutated messages (Hello, OpenSecureChannel, sessions, Read, Write; on None channels and, a
// few times, on signed Basic256Sha256 channels with the mutation signed properly) never crash
// the server, and it keeps serving clients. RTOS_FUZZ_ITERATIONS / RTOS_FUZZ_SEED vary the run.
TEST_F(OpcUaTest, MutatedMessagesKeepTheServerHealthy) {
    fuzz::Mutator m(fuzz::seed(0x0C0A));
    std::uint32_t target = 0, count = 0;
    bool armed = false;  // the signed OPN itself is not mutated: RSA dominates the run time
    fuzz = [&](UaBytes& msg) {
        if (!armed || (count++ != target && !m.chance(8))) {
            return;
        }
        fuzz::Bytes b(reinterpret_cast<const std::byte*>(msg.data()),
                      reinterpret_cast<const std::byte*>(msg.data()) + msg.size());
        m.mutate(b, 0, 9000);
        msg.assign(reinterpret_cast<const Byte*>(b.data()), reinterpret_cast<const Byte*>(b.data()) + b.size());
    };
    const auto run = [&](bool signed_channel) {
        channel = token = sequence = request = 0;
        secured = false;
        auth_token = {};
        target = m.below(signed_channel ? 4 : 6);
        count = 0;
        armed = !signed_channel;
        connect();
        const bool opened = signed_channel ? (hello() && open_basic(client_cert, client_key)) : (hello() && open_none());
        armed = true;
        if (opened && session()) {
            (void)read(kLed);
            (void)write(kLed, 1);
            (void)read(kUptime);
        }
        reset_connection();
        poll(1);
    };
    const std::size_t n = fuzz::iterations(1500);
    std::uint32_t primed = now;
    for (std::size_t i = 0; i < n; ++i) {
        if (now - primed > TestStack::arp_timeout_ms) {
            prime_arp();  // long runs pass the ARP timeout; the stack asks again
            primed = now;
        }
        run(false);
    }
    for (std::size_t i = 0; i < std::max<std::size_t>(n / 300, 1); ++i) {
        run(true);
    }

    fuzz = nullptr;
    channel = token = sequence = request = 0;
    secured = false;
    connect();
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const std::array<std::uint16_t, 1> seven{7};
    bank.write(0, seven);
    const auto [status, value] = read(kLed);
    EXPECT_TRUE(ua::IsGood(status));
    EXPECT_EQ(std::get<ua::UInt16>(value.storage), 7);
}

}  // namespace

