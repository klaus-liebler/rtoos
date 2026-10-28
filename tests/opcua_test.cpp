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

// ---- session steps ------------------------------------------------------------------------
struct Created {
    std::optional<Response> response;
    ua::NodeId token;
    UaBytes server_nonce;
    bool server_certificate_null = true;
};
struct ActivateOptions {
    bool sign = true;                     // a ClientSignature on a secured channel
    bool corrupt_signature = false;       // ... with one bit flipped
    ua::UInt32 token_type = 321;          // AnonymousIdentityToken
    std::string_view policy_id = "anonymous";
    bool null_token = false;              // no UserIdentityToken at all
    ua::Int32 locale_ids = 0;
};

using Server = rtos::net::OpcUaServer<TestStack, 2>;

class OpcUaTest : public NetTest {
public:
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
        std::array<Byte, 80> client_keys{}, server_keys{};
        bool encrypted = false;
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
        swap(o.client_keys, client_keys);
        swap(o.server_keys, server_keys);
        swap(o.encrypted, encrypted);
        swap(o.auth_token, auth_token);
    }

    // ---- UA TCP ------------------------------------------------------------------------------
    std::array<Byte, 16384> buffer{};
    ua::UInt32 channel = 0, token = 0, sequence = 0, request = 0;
    bool secured = false;
    // signing key (32) | encrypting key (32) | IV (16) per direction (Part 6 6.7.5)
    std::array<Byte, 80> client_keys{}, server_keys{};
    bool encrypted = false;  // SignAndEncrypt
    ua::StatusCode last_error = ua::StatusCode::Good;  // of an ERR answer to open_basic
    ua::Int32 request_type = -1;  // OPN SecurityTokenRequestType; -1: Issue, or Renew once open
    bool fixed_nonce = false;     // open_basic: always the same client nonce
    ua::NodeId auth_token;
    // the identity of the last signed channel (open_basic): CreateSession sends the
    // certificate, ActivateSession signs with the key
    std::span<const unsigned char> identity_cert, identity_key;

    /// RSASSA-PKCS1-v1_5 SHA-256 signature with `key_der` (client side).
    UaBytes rsa_sign(std::span<const unsigned char> key_der, std::span<const Byte> message) {
        auto& cc = ClientCrypto::instance();
        RsaKey key;
        UaBytes signature(256);
        const UaBytes info = digest_info(message);
        const bool ok = cc.private_key(std::span<const Byte>(key_der.data(), key_der.size()), key) &&
                        wc_RsaSSL_Sign(info.data(), (word32)info.size(), signature.data(), 256, &key, &cc.rng) == 256;
        wc_FreeRsaKey(&key);
        return ok ? signature : UaBytes{};
    }

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
        w.WriteInt32(request_type >= 0 ? request_type : (channel != 0 ? 1 : 0));  // Issue, or Renew once open
        w.WriteInt32(mode);
        w.WriteByteString(std::string_view(reinterpret_cast<const char*>(nonce.data()), nonce.size()), nonce.empty());
        w.WriteUInt32(600000);
    }

    bool open_none(ua::Int32 mode = 1) {
        ua::ByteWriter w(buffer);
        ua::EncodeTcpHeader(w, "OPN", ua::ChunkType::Final, 0);
        w.WriteUInt32(0);
        w.WriteString(kNone);
        w.WriteByteString({}, true);
        w.WriteByteString({}, true);
        w.WriteUInt32(++sequence);
        w.WriteUInt32(++request);
        open_body(w, mode, {});
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
        encrypted = false;
        return b.Ok();
    }

    /// Basic256Sha256 / Sign with `cert`/`key` as the client identity.
    /// `after_send` runs between sending the OPN and receiving the answer.
    bool open_basic(std::span<const unsigned char> cert, std::span<const unsigned char> key_der,
                    const std::function<void()>& after_send = {}, ua::Int32 mode = 2) {
        identity_cert = cert;
        identity_key = key_der;
        auto& cc = ClientCrypto::instance();
        RsaKey server_pub, client_priv, client_pub;
        if (!cc.public_key(std::span<const Byte>(server_cert, sizeof server_cert), server_pub) ||
            !cc.private_key(std::span<const Byte>(key_der.data(), key_der.size()), client_priv) ||
            !cc.public_key(std::span<const Byte>(cert.data(), cert.size()), client_pub)) {
            return false;
        }
        std::array<Byte, 32> client_nonce{};
        wc_RNG_GenerateBlock(&cc.rng, client_nonce.data(), client_nonce.size());
        if (fixed_nonce) client_nonce.fill(0x42);

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
        open_body(b, mode, client_nonce);
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
        if (reply && is_message(*reply, "ERR")) {
            ua::ByteReader e(*reply);
            e.Skip(8);
            e.ReadStatusCode(last_error);
        }
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
        ua::security::PSha256(sn, client_nonce, client_keys);
        ua::security::PSha256(client_nonce, sn, server_keys);
        encrypted = mode == 3;
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
            if (encrypted) {  // padding to whole AES blocks (with the signature)
                const std::size_t pad = (16 - (msg.size() - 16 + 1 + 32) % 16) % 16;
                msg.insert(msg.end(), pad + 1, static_cast<Byte>(pad));
            }
            ua::ByteWriter size{std::span(msg).subspan(4, 4)};
            size.WriteUInt32(static_cast<ua::UInt32>(msg.size() + 32));
            std::array<Byte, 32> mac{};
            ua::security::HmacSha256(std::span(client_keys).first(32), msg, mac.data());
            msg.insert(msg.end(), mac.begin(), mac.end());
            if (encrypted) {
                EXPECT_TRUE(ua::security::AesCbcEncrypt(std::span(client_keys).subspan<32, 32>(),
                                                        std::span(client_keys).subspan<64, 16>(),
                                                        std::span(msg).subspan(16)));
            }
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
        return wait_response();
    }

    /// The next MSG from the server, verified (and decrypted) and decoded.
    std::optional<Response> wait_response() {
        const auto reply = receive();
        if (!reply || !is_message(*reply, "MSG")) {
            return std::nullopt;
        }
        UaBytes plain = *reply;
        if (secured && encrypted) {
            EXPECT_EQ((plain.size() - 16) % 16, 0u) << "whole AES blocks";
            EXPECT_TRUE(ua::security::AesCbcDecrypt(std::span(server_keys).subspan<32, 32>(),
                                                    std::span(server_keys).subspan<64, 16>(),
                                                    std::span(plain).subspan(16)));
        }
        std::size_t end = plain.size();
        if (secured) {
            end -= 32;
            std::array<Byte, 32> mac{};
            ua::security::HmacSha256(std::span(server_keys).first(32), std::span(plain).first(end), mac.data());
            EXPECT_EQ(std::memcmp(mac.data(), plain.data() + end, 32), 0) << "server signature";
            if (encrypted) {
                const std::size_t pad = plain[end - 1];
                for (std::size_t i = end - 1 - pad; i < end; i++) {
                    EXPECT_EQ(plain[i], pad) << "padding";
                }
                end -= pad + 1;
            }
        }
        Response resp;
        if (!parse_response(UaBytes(plain.begin() + 24, plain.begin() + static_cast<std::ptrdiff_t>(end)), resp)) {
            return std::nullopt;
        }
        return resp;
    }

    // ---- sessions -------------------------------------------------------------------------
    /// CreateSession; `certificate`: the client certificate to send (default: the channel's).
    Created create_session(std::optional<std::span<const unsigned char>> certificate = std::nullopt,
                           std::size_t nonce_size = 32) {
        Created out;
        const auto cert = certificate.value_or(identity_cert);
        out.response = call([&](ua::ByteWriter& w) {
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
            std::vector<Byte> nonce(nonce_size, 0x5A);
            w.WriteByteString(std::string_view(reinterpret_cast<const char*>(nonce.data()), nonce.size()), false);
            if (!cert.empty()) {
                w.WriteByteString(std::string_view(reinterpret_cast<const char*>(cert.data()), cert.size()), false);
            } else {
                w.WriteByteString({}, true);
            }
            w.WriteDouble(60000);
            w.WriteUInt32(0);
        });
        if (!out.response || !ua::IsGood(out.response->result) || out.response->type != ua::NodeId(0, ua::ns0::CreateSessionResponse)) {
            return out;
        }
        ua::ByteReader r = out.response->reader();
        ua::NodeId session_id;
        ua::Double timeout = 0;
        std::string_view nonce, server_certificate;
        bool nonce_null = true;
        ua::DecodeNodeId(r, session_id);
        ua::DecodeNodeId(r, out.token);
        r.ReadDouble(timeout);
        r.ReadByteString(nonce, nonce_null);
        r.ReadByteString(server_certificate, out.server_certificate_null);
        out.server_nonce.assign(nonce.begin(), nonce.end());
        return out;
    }

    std::optional<Response> activate_session(const ua::NodeId& token, std::span<const Byte> server_nonce,
                                             const ActivateOptions& o = {}) {
        // ClientSignature (Part 4 5.6.3): server certificate (leaf) || server nonce
        UaBytes signature;
        if (secured && o.sign) {
            UaBytes signed_data(std::begin(server_cert), std::end(server_cert));
            signed_data.insert(signed_data.end(), server_nonce.begin(), server_nonce.end());
            signature = rsa_sign(identity_key, signed_data);
            if (o.corrupt_signature && !signature.empty()) signature[10] ^= 0x01;
        }
        return call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::ActivateSessionRequest));
            request_header(w, token, 3);
            if (!signature.empty()) {
                w.WriteString("http://www.w3.org/2001/04/xmldsig-more#rsa-sha256");
                w.WriteByteString(std::string_view(reinterpret_cast<const char*>(signature.data()), signature.size()),
                                  false);
            } else {
                w.WriteString(ua::String::Null());  // client signature
                w.WriteByteString({}, true);
            }
            w.WriteInt32(0);  // software certificates
            w.WriteInt32(o.locale_ids);
            for (ua::Int32 i = 0; i < o.locale_ids; i++) w.WriteString("en-US");
            if (o.null_token) {
                ua::EncodeNullExtensionObject(w);
            } else {
                ua::EncodeNodeId(w, ua::NodeId(0, o.token_type));
                w.WriteByte(0x01);
                std::array<Byte, 64> token_body{};
                ua::ByteWriter t(token_body);
                t.WriteString(o.policy_id);
                w.WriteByteString(std::string_view(reinterpret_cast<const char*>(token_body.data()), t.Position()), false);
            }
            w.WriteString(ua::String::Null());  // user token signature
            w.WriteByteString({}, true);
        });
    }

    std::optional<Response> close_session(const ua::NodeId& token) {
        return call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::CloseSessionRequest));
            request_header(w, token, 6);
            w.WriteBoolean(true);  // deleteSubscriptions
        });
    }

    /// CreateSession + ActivateSession; auth_token is the session's token afterwards.
    bool session(const ActivateOptions& o = {}) {
        const Created created = create_session();
        if (!created.response || !ua::IsGood(created.response->result)) {
            return false;
        }
        auth_token = created.token;
        const auto activated = activate_session(created.token, created.server_nonce, o);
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
    static_assert(std::to_underlying(ua::StatusCode::BadNonceInvalid) == 0x80240000u);
    static_assert(std::to_underlying(ua::StatusCode::BadCertificateInvalid) == 0x80120000u);
    static_assert(std::to_underlying(ua::StatusCode::BadApplicationSignatureInvalid) == 0x80580000u);
    static_assert(std::to_underlying(ua::StatusCode::BadCertificateUriInvalid) == 0x80170000u);
    static_assert(std::to_underlying(ua::StatusCode::BadMaxAgeInvalid) == 0x80700000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTooManyOperations) == 0x80100000u);
    static_assert(std::to_underlying(ua::StatusCode::BadIndexRangeInvalid) == 0x80360000u);
    static_assert(std::to_underlying(ua::StatusCode::BadIndexRangeNoData) == 0x80370000u);
    static_assert(std::to_underlying(ua::StatusCode::BadDataEncodingInvalid) == 0x80380000u);
    static_assert(std::to_underlying(ua::StatusCode::BadDataEncodingUnsupported) == 0x80390000u);
    static_assert(std::to_underlying(ua::StatusCode::BadWriteNotSupported) == 0x80730000u);
    static_assert(std::to_underlying(ua::StatusCode::BadBrowseDirectionInvalid) == 0x804D0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadViewIdUnknown) == 0x806B0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadReferenceTypeIdInvalid) == 0x804C0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadContinuationPointInvalid) == 0x804A0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNoContinuationPoints) == 0x804B0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadBrowseNameInvalid) == 0x80600000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSubscriptionIdInvalid) == 0x80280000u);
    static_assert(std::to_underlying(ua::StatusCode::BadMonitoredItemIdInvalid) == 0x80420000u);
    static_assert(std::to_underlying(ua::StatusCode::BadMonitoringModeInvalid) == 0x80410000u);
    static_assert(std::to_underlying(ua::StatusCode::BadMonitoredItemFilterInvalid) == 0x80430000u);
    static_assert(std::to_underlying(ua::StatusCode::BadMonitoredItemFilterUnsupported) == 0x80440000u);
    static_assert(std::to_underlying(ua::StatusCode::BadFilterNotAllowed) == 0x80450000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTooManySubscriptions) == 0x80770000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTooManyMonitoredItems) == 0x80DB0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTooManyPublishRequests) == 0x80780000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNoSubscription) == 0x80790000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSequenceNumberUnknown) == 0x807A0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadMessageNotAvailable) == 0x807B0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadDeadbandFilterInvalid) == 0x808E0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTimeout) == 0x800A0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadRequestTooLarge) == 0x80B80000u);
    static_assert(std::to_underlying(ua::StatusCode::BadResponseTooLarge) == 0x80B90000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSequenceNumberInvalid) == 0x80880000u);
    static_assert(std::to_underlying(ua::StatusCode::BadSecureChannelTokenUnknown) == 0x80870000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNodeClassInvalid) == 0x805F0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNodeAttributesInvalid) == 0x80620000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTcpEndpointUrlInvalid) == 0x80830000u);
    static_assert(std::to_underlying(ua::StatusCode::GoodSubscriptionTransferred) == 0x002D0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTooManyArguments) == 0x80E50000u);
    static_assert(std::to_underlying(ua::StatusCode::GoodCompletesAsynchronously) == 0x002E0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadNotFound) == 0x803E0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadInvalidState) == 0x80AF0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadTimestampNotSupported) == 0x80A10000u);
    static_assert(std::to_underlying(ua::StatusCode::BadWaitingForInitialData) == 0x80320000u);
    static_assert(std::to_underlying(ua::StatusCode::BadServerNotConnected) == 0x800D0000u);
    static_assert(std::to_underlying(ua::StatusCode::BadCertificateTimeInvalid) == 0x80140000u);
    static_assert(std::to_underlying(ua::StatusCode::BadCertificateUntrusted) == 0x801A0000u);
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

// open62541: check_securechannel_unit.c processHELACK_happyPath_returnsGood
// open62541: check_securechannel_unit.c processHELACK_remoteReceiveCapped_capsSendBuffer
// open62541: check_securechannel_unit.c processHELACK_remoteSendCapped_capsRecvBuffer
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

// open62541: check_securechannel_unit.c processHELACK_bufferTooSmall_returnsBad
TEST_F(OpcUaTest, HelloWithTooSmallBuffersIsAnError) {
    const auto err = hello(1024);
    ASSERT_TRUE(err.has_value());
    EXPECT_TRUE(is_message(*err, "ERR"));
}

// open62541: check_securechannel.c SecureChannel_sendSymmetricMessage_modeNone
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

// open62541: server/check_server_getendpoints.c testServerCertificateInGetEndpoints
// open62541: server/check_discovery.c Server_getEndpoints_emptyPolicyId
// open62541: server/check_server.c checkNoneEndpointUserTokens
// open62541: server/check_server_client_readwrite.c client_get_endpoints
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
    // SignAndEncrypt, Sign (Basic256Sha256) and None, the most secure first
    ua::ByteReader b = r->reader();
    ua::Int32 count = 0;
    ASSERT_TRUE(b.ReadInt32(count));
    ASSERT_EQ(count, 3);
    const ua::Int32 modes[] = {3, 2, 1};
    const ua::Byte levels[] = {2, 1, 0};
    for (ua::Int32 i = 0; i < count; i++) {
        ua::String url, policy, transport;
        ua::ApplicationDescription app;
        std::string_view certificate;
        bool certificate_null = true;
        ua::Int32 mode = 0, tokens = 0;
        ua::Byte level = 0;
        ASSERT_TRUE(b.ReadString(url) && ua::DecodeApplicationDescription(b, app) &&
                    b.ReadByteString(certificate, certificate_null) && b.ReadInt32(mode) && b.ReadString(policy) &&
                    b.ReadInt32(tokens));
        for (ua::Int32 t = 0; t < tokens; t++) {
            ua::String id, issued, issuer, token_policy;
            ua::Int32 type = -1;
            ASSERT_TRUE(b.ReadString(id) && b.ReadInt32(type) && b.ReadString(issued) && b.ReadString(issuer) &&
                        b.ReadString(token_policy));
            EXPECT_EQ(id.value, "anonymous");
            EXPECT_EQ(type, 0);
        }
        ASSERT_TRUE(b.ReadString(transport) && b.ReadByte(level));
        EXPECT_EQ(url.value, kEndpoint);
        EXPECT_EQ(app.applicationUri.value, "urn:factory-control-unit:opcua-native");
        EXPECT_FALSE(certificate_null);
        EXPECT_EQ(mode, modes[i]);
        EXPECT_EQ(policy.value, i < 2 ? std::string_view(kBasic256Sha256) : std::string_view(kNone));
        EXPECT_EQ(level, levels[i]);
    }
}

// open62541: check_securechannel.c SecureChannel_sendSymmetricMessage_modeSign
// open62541: check_securechannel.c SecureChannel_sendSymmetricMessage
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
// ---- sessions (ported from open62541 tests/server/check_session.c, check_session_errorpaths.c,
// tests/client/check_activateSession.c; the tests about open62541's internals -- session
// attributes, callbacks, access-control plugins, the local server API -- are listed with their
// reasons in tests/open62541/rules.tsv) --------------------------------------------------------


// open62541: server/check_session.c Session_close_before_activate
TEST_F(OpcUaTest, Session_close_before_activate) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    const Created created = create_session();
    ASSERT_TRUE(created.response && ua::IsGood(created.response->result));
    const auto closed = close_session(created.token);
    ASSERT_TRUE(closed);
    EXPECT_EQ(closed->result, ua::StatusCode::Good);
}

// open62541: server/check_session.c Session_activate_then_close
// open62541: server/check_accesscontrol.c Client_anonymous
// open62541: client/check_activateSession.c Client_activateSession
// open62541: client/check_activateSession.c Client_read
// open62541: client/check_activateSession.c Client_activateSessionClose
TEST_F(OpcUaTest, Session_activate_then_close) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const auto [status, value] = read(ua::NodeId(0, ua::ns0::Server_ServerStatus_CurrentTime));
    EXPECT_TRUE(ua::IsGood(status));
    EXPECT_NE(value.GetIf<ua::DateTimeValue>(), nullptr);
    const auto closed = close_session(auth_token);
    ASSERT_TRUE(closed);
    EXPECT_EQ(closed->result, ua::StatusCode::Good);
}

// After CloseSession, its token is unknown. (open62541 closes the SecureChannel first; a
// closed channel ends the connection here, which the TCP tests cover.)
// open62541: server/check_session.c Session_readAfterClose
// open62541: server/check_session_errorpaths.c Session_closeTwiceFails
TEST_F(OpcUaTest, Session_readAfterClose) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    ASSERT_TRUE(close_session(auth_token));
    EXPECT_EQ(read(kLed).first, ua::StatusCode::BadSessionIdInvalid);
    const auto again = close_session(auth_token);
    ASSERT_TRUE(again);
    EXPECT_EQ(again->result, ua::StatusCode::BadSessionIdInvalid);
}

// open62541: server/check_session.c Session_browse
TEST_F(OpcUaTest, Session_browse) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const auto r = call([&](ua::ByteWriter& w) {
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::BrowseRequest));
        request_header(w, auth_token, 8);
        ua::EncodeNodeId(w, ua::NodeId());
        w.WriteDateTime(0);
        w.WriteUInt32(0);
        w.WriteUInt32(0);
        w.WriteInt32(1);
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::RootFolder));
        w.WriteInt32(0);
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::HierarchicalReferences));
        w.WriteBoolean(true);
        w.WriteUInt32(0);
        w.WriteUInt32(0x3F);
    });
    ASSERT_TRUE(r);
    EXPECT_EQ(r->result, ua::StatusCode::Good);
    ua::ByteReader b = r->reader();
    ua::Int32 results = 0, references = 0;
    ua::StatusCode status{};
    std::string_view cp;
    bool cp_null = true;
    ASSERT_TRUE(b.ReadInt32(results) && results == 1 && b.ReadStatusCode(status) && b.ReadByteString(cp, cp_null) &&
                b.ReadInt32(references));
    EXPECT_EQ(status, ua::StatusCode::Good);
    EXPECT_GT(references, 0) << "Objects, Types, Views";
}

// open62541: server/check_session.c Session_write_read_value
TEST_F(OpcUaTest, Session_write_read_value) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    ASSERT_TRUE(session());
    EXPECT_TRUE(ua::IsGood(write(kLed, 123)));
    const auto [status, value] = read(kLed);
    EXPECT_TRUE(ua::IsGood(status));
    ASSERT_NE(value.GetIf<ua::UInt16>(), nullptr);
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), 123);
}

// open62541: server/check_session.c Session_multiple_reads
TEST_F(OpcUaTest, Session_multiple_reads) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    for (int i = 0; i < 10; i++) {
        EXPECT_TRUE(ua::IsGood(read(ua::NodeId(0, ua::ns0::Server_ServerStatus_State)).first)) << i;
    }
}

// open62541: server/check_session.c Session_reconnect_after_disconnect
TEST_F(OpcUaTest, Session_reconnect_after_disconnect) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    EXPECT_TRUE(ua::IsGood(read(kLed).first));
    reset_connection();
    poll(1);
    channel = token = sequence = request = 0;
    connect();
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    EXPECT_TRUE(ua::IsGood(read(kLed).first));
}

// Two clients, two sessions; a session's token is not valid on another connection.
// open62541: server/check_session.c Session_create_multiple
// open62541: server/check_session.c Session_secureChannel_mismatch
// open62541: server/check_session.c Session_closeSession_secureChannel_mismatch
TEST_F(OpcUaTest, Session_create_multiple) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const ua::NodeId first_token = auth_token;
    Peer first;
    swap(first);
    connect();
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    EXPECT_NE(auth_token, first_token);
    EXPECT_TRUE(ua::IsGood(read(kLed).first));
    // the first client's token on the second connection
    const ua::NodeId own = auth_token;
    auth_token = first_token;
    EXPECT_EQ(read(kLed).first, ua::StatusCode::BadSessionIdInvalid);
    const auto closed = close_session(first_token);
    ASSERT_TRUE(closed);
    EXPECT_EQ(closed->result, ua::StatusCode::BadSessionIdInvalid);
    auth_token = own;
    swap(first);
    EXPECT_TRUE(ua::IsGood(read(kLed).first)) << "the first session still works";
}

// open62541: server/check_session.c Session_createSession_certIgnored_on_none_policy
// open62541: server/check_session.c Session_createSession_noServerCertOnNonePolicy
TEST_F(OpcUaTest, Session_createSession_certIgnored_on_none_policy) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    static constexpr unsigned char dummy[] = {0x30, 0x82, 0x01, 0x00};
    const Created created = create_session(std::span<const unsigned char>(dummy));
    ASSERT_TRUE(created.response);
    EXPECT_EQ(created.response->result, ua::StatusCode::Good);
    EXPECT_TRUE(created.server_certificate_null) << "no server certificate on SecurityPolicy None";
}

// open62541: server/check_session_errorpaths.c Session_activateWithBadToken
TEST_F(OpcUaTest, Session_activateWithBadToken) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    const Created created = create_session();
    ASSERT_TRUE(created.response && ua::IsGood(created.response->result));
    const auto r = activate_session(ua::NodeId(1, 4711), created.server_nonce);
    ASSERT_TRUE(r);
    EXPECT_EQ(r->result, ua::StatusCode::BadSessionIdInvalid);
}

// open62541: server/check_session_errorpaths.c Session_activate_withLocaleIds_succeeds
// open62541: client/check_activateSession.c Client_activateSessionLocaleIds
TEST_F(OpcUaTest, Session_activate_withLocaleIds_succeeds) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    EXPECT_TRUE(session({.locale_ids = 2}));
}

// open62541: client/check_activateSession.c Client_activateSession_nullToken
TEST_F(OpcUaTest, Client_activateSession_nullToken) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    EXPECT_TRUE(session({.null_token = true}));
}

// A session without requests for its timeout is closed.
// open62541: client/check_activateSession.c Client_activateSessionTimeout
TEST_F(OpcUaTest, Client_activateSessionTimeout) {
    static std::uint64_t ms = 1'000'000;
    ua::SetClock(nullptr, []() noexcept { return ms; });
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());  // requested 60 s
    ms += 59'000;
    EXPECT_TRUE(ua::IsGood(read(kLed).first)) << "a request is activity";
    ms += 59'000;
    EXPECT_TRUE(ua::IsGood(read(kLed).first));
    ms += 61'000;
    EXPECT_EQ(read(kLed).first, ua::StatusCode::BadSessionIdInvalid) << "timed out";
    ua::SetClock(nullptr, nullptr);
}

// A renewed SecureChannel (a second OpenSecureChannel) keeps the session.
// open62541: client/check_activateSession.c Client_renewSecureChannel
TEST_F(OpcUaTest, Client_renewSecureChannel) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const ua::UInt32 old_token = token, old_channel = channel;
    ASSERT_TRUE(open_none());
    EXPECT_EQ(channel, old_channel);
    EXPECT_NE(token, old_token);
    EXPECT_TRUE(ua::IsGood(read(kLed).first));
}

// ---- rtoos: session checks ---------------------------------------------------------------------

TEST_F(OpcUaTest, ServicesNeedAnActivatedSession) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    EXPECT_EQ(read(kLed).first, ua::StatusCode::BadSessionIdInvalid) << "no session";
    const Created created = create_session();
    ASSERT_TRUE(created.response && ua::IsGood(created.response->result));
    auth_token = created.token;
    EXPECT_EQ(read(kLed).first, ua::StatusCode::BadSessionNotActivated);
}

TEST_F(OpcUaTest, UnknownServiceIsAFaultWithTheRequestHandle) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    const auto r = call([&](ua::ByteWriter& w) {
        ua::EncodeNodeId(w, ua::NodeId(0, 712));  // HistoryReadRequest
        request_header(w, auth_token, 4242);
    });
    ASSERT_TRUE(r);
    EXPECT_EQ(r->type, ua::NodeId(0, ua::ns0::ServiceFault));
    EXPECT_EQ(r->result, ua::StatusCode::BadServiceUnsupported);
}

TEST_F(OpcUaTest, OnlyTheAnonymousTokenOfTheOfferedPolicyIsAccepted) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    for (const ActivateOptions o : {ActivateOptions{.token_type = 324}, ActivateOptions{.token_type = 327},
                                    ActivateOptions{.policy_id = "username"}}) {
        const Created created = create_session();
        ASSERT_TRUE(created.response && ua::IsGood(created.response->result));
        const auto r = activate_session(created.token, created.server_nonce, o);
        ASSERT_TRUE(r);
        EXPECT_EQ(r->result, ua::StatusCode::BadIdentityTokenInvalid);
    }
}

TEST_F(OpcUaTest, ActivateSessionChecksTheClientSignature) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    for (const ActivateOptions o : {ActivateOptions{.sign = false}, ActivateOptions{.corrupt_signature = true}}) {
        const Created created = create_session();
        ASSERT_TRUE(created.response && ua::IsGood(created.response->result));
        EXPECT_EQ(created.server_nonce.size(), 32u);
        EXPECT_FALSE(created.server_certificate_null);
        const auto r = activate_session(created.token, created.server_nonce, o);
        ASSERT_TRUE(r);
        EXPECT_EQ(r->result, ua::StatusCode::BadApplicationSignatureInvalid);
    }
    // a signature over an old nonce does not count either: every activation gets a new one
    const Created created = create_session();
    ASSERT_TRUE(created.response && ua::IsGood(created.response->result));
    ASSERT_TRUE(activate_session(created.token, created.server_nonce));
    const auto replay = activate_session(created.token, created.server_nonce);
    ASSERT_TRUE(replay);
    EXPECT_EQ(replay->result, ua::StatusCode::BadApplicationSignatureInvalid);
}

TEST_F(OpcUaTest, CreateSessionOnASignedChannelNeedsItsClientAndANonce) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    Created created = create_session(std::span<const unsigned char>(rogue_cert));
    ASSERT_TRUE(created.response);
    EXPECT_EQ(created.response->result, ua::StatusCode::BadCertificateInvalid) << "not the channel's certificate";
    created = create_session(std::nullopt, 16);
    ASSERT_TRUE(created.response);
    EXPECT_EQ(created.response->result, ua::StatusCode::BadNonceInvalid);
}

// ---- SignAndEncrypt, crypto (ported from open62541 tests/encryption) ------------------------------

// open62541: encryption/check_encryption_basic256sha256.c encryption_connect
// open62541: check_securechannel.c SecureChannel_sendSymmetricMessage_modeSignAndEncrypt
// open62541: encryption/check_encryption_basic256sha256_with_config.c encryption_connect
// open62541: encryption/check_ca_chain.c encryption_connect_Basic256Sha256_ca
TEST_F(OpcUaTest, TrustedClientOnASignAndEncryptChannel) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key, {}, 3));
    ASSERT_TRUE(session());
    EXPECT_TRUE(ua::IsGood(write(kLed, 7)));
    EXPECT_EQ(bank.holding(0), 7);
    const auto [status, value] = read(kLed);
    EXPECT_TRUE(ua::IsGood(status));
    ASSERT_NE(value.GetIf<ua::UInt16>(), nullptr);
    EXPECT_EQ(*value.GetIf<ua::UInt16>(), 7);
    // many sizes: the padding to whole AES blocks
    for (int i = 0; i < 20; i++) {
        EXPECT_TRUE(ua::IsGood(read(i % 2 ? kLed : kUptime).first)) << i;
    }
}

// The client learns that the channel was refused, not why (no certificate oracle).
// open62541: encryption/check_cert_validation_client_response.c testOpenSecureChannelCertificateFailuresHidden
TEST_F(OpcUaTest, testOpenSecureChannelCertificateFailuresHidden) {
    ASSERT_TRUE(hello());
    EXPECT_FALSE(open_basic(rogue_cert, rogue_key, {}, 3));
    EXPECT_EQ(last_error, ua::StatusCode::BadSecurityChecksFailed);
    EXPECT_TRUE(closed);
}

TEST_F(OpcUaTest, TamperedEncryptedMessageClosesTheChannel) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key, {}, 3));
    ASSERT_TRUE(session());
    const auto r = call(
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
        true);
    EXPECT_FALSE(r.has_value());
    (void)receive();
    EXPECT_TRUE(closed);
}

// open62541: encryption/check_hmac_verify_constanttime.c opensslHmacSha256VerifyRejectsTamperedAndShortSignatures
// open62541: encryption/check_hmac_verify_constanttime.c mbedtlsHmacSha256VerifyRejectsTamperedAndShortSignatures
TEST(OpcUaCrypto, HmacSha256VerifyRejectsTamperedAndShortSignatures) {
    const std::array<Byte, 32> key{1, 2, 3};
    const std::string_view text = "message to sign";
    const std::span<const Byte> message(reinterpret_cast<const Byte*>(text.data()), text.size());
    std::array<Byte, 32> mac{}, again{};
    ASSERT_TRUE(ua::security::HmacSha256(key, message, mac.data()));
    ASSERT_TRUE(ua::security::HmacSha256(key, message, again.data()));
    EXPECT_TRUE(ua::security::ConstantTimeEqual(mac, again));
    for (std::size_t i : {0u, 15u, 31u}) {
        auto tampered = again;
        tampered[i] ^= 0x80;
        EXPECT_FALSE(ua::security::ConstantTimeEqual(mac, tampered)) << i;
    }
    EXPECT_FALSE(ua::security::ConstantTimeEqual(mac, std::span(again).first(31))) << "short signature";
    EXPECT_FALSE(ua::security::ConstantTimeEqual(mac, std::span<const Byte>())) << "empty signature";
}

// The building blocks of Basic256Sha256, each against an independent reference.
// open62541: encryption/check_securitypolicy_crypto.c securitypolicy_basic256sha256
// open62541: check_securechannel.c SecureChannel_derivedKeyLength
TEST(OpcUaCrypto, Basic256Sha256Primitives) {
    // AES-256-CBC: NIST SP 800-38A F.2.5
    std::array<Byte, 32> key{};
    const char* hex = "603deb1015ca71be2b73aef0857d77811f352c073b6108d72d9810a30914dff4";
    for (std::size_t i = 0; i < 32; i++) key[i] = static_cast<Byte>(std::stoi(std::string(hex + 2 * i, 2), nullptr, 16));
    std::array<Byte, 16> iv{};
    for (std::size_t i = 0; i < 16; i++) iv[i] = static_cast<Byte>(i);
    std::array<Byte, 16> block = {0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
                                  0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a};
    const std::array<Byte, 16> plain = block;
    ASSERT_TRUE(ua::security::AesCbcEncrypt(key, iv, block));
    EXPECT_EQ(block, (std::array<Byte, 16>{0xf5, 0x8c, 0x4c, 0x04, 0xd6, 0xe5, 0xf1, 0xba, 0x77, 0x9e, 0xab, 0xfb,
                                           0x5f, 0x7b, 0xfb, 0xd6}));
    ASSERT_TRUE(ua::security::AesCbcDecrypt(key, iv, block));
    EXPECT_EQ(block, plain);
    std::array<Byte, 15> partial{};
    EXPECT_FALSE(ua::security::AesCbcEncrypt(key, iv, partial)) << "whole blocks only";

    // P_SHA256 (Part 6 6.7.5 / RFC 5246 5): reference computed with Python's hmac
    std::array<Byte, 32> secret{};
    for (std::size_t i = 0; i < 32; i++) secret[i] = static_cast<Byte>(i + 1);
    const std::string_view seed = "rtoos P_SHA256 known answer";
    std::array<Byte, 80> material{};
    ASSERT_TRUE(ua::security::PSha256(secret, std::span<const Byte>(reinterpret_cast<const Byte*>(seed.data()), seed.size()),
                                      material));
    const std::array<Byte, 80> expected = {
        0x8c, 0x14, 0x7f, 0xb6, 0xb2, 0xef, 0x63, 0xa9, 0x6c, 0x8f, 0x9d, 0xf5, 0xf7, 0x18, 0x02, 0xad,
        0x29, 0xa4, 0x6c, 0x1d, 0x90, 0x79, 0x48, 0xf8, 0x25, 0x04, 0x2c, 0x15, 0x35, 0x27, 0x59, 0xc3,
        0xde, 0x16, 0xa6, 0x5a, 0xd0, 0x16, 0xc1, 0xcd, 0x46, 0x87, 0xf0, 0x8f, 0x45, 0x1a, 0xcf, 0xf6,
        0xc3, 0x10, 0xa3, 0x39, 0x48, 0xe9, 0xcd, 0x87, 0x04, 0x3c, 0xf8, 0x6c, 0xe4, 0xd1, 0xd3, 0xda,
        0xce, 0x7f, 0x1e, 0x35, 0x4e, 0x76, 0xe5, 0x1b, 0xf9, 0xed, 0xb8, 0xa8, 0x28, 0xbe, 0x1d, 0x12};
    EXPECT_EQ(material, expected);
}

// ---- subscriptions over TCP: Publish responses sent from the connection's tick -----------------

// The OPC UA monotonic clock follows the test's stack clock.
std::uint32_t* g_stack_now = nullptr;

TEST_F(OpcUaTest, SubscriptionPublishesOverAnEncryptedChannel) {
    g_stack_now = &now;
    ua::SetClock(nullptr, []() noexcept { return std::uint64_t{*g_stack_now}; });
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key, {}, 3));
    ASSERT_TRUE(session());
    auto created = call([&](ua::ByteWriter& w) {
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::CreateSubscriptionRequest));
        request_header(w, auth_token, 20);
        w.WriteDouble(100);
        w.WriteUInt32(30);
        w.WriteUInt32(10);
        w.WriteUInt32(0);
        w.WriteBoolean(true);
        w.WriteByte(0);
    });
    ASSERT_TRUE(created && ua::IsGood(created->result));
    ua::UInt32 sub = 0;
    ua::ByteReader cr = created->reader();
    ASSERT_TRUE(cr.ReadUInt32(sub));
    auto items = call([&](ua::ByteWriter& w) {
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::CreateMonitoredItemsRequest));
        request_header(w, auth_token, 21);
        w.WriteUInt32(sub);
        w.WriteInt32(2);
        w.WriteInt32(1);
        ua::EncodeNodeId(w, kLed);
        w.WriteUInt32(13);
        w.WriteString(ua::String::Null());
        ua::EncodeQualifiedName(w, ua::QualifiedName{});
        w.WriteInt32(2);
        w.WriteUInt32(77);  // client handle
        w.WriteDouble(50);
        ua::EncodeNullExtensionObject(w);
        w.WriteUInt32(1);
        w.WriteBoolean(true);
    });
    ASSERT_TRUE(items && ua::IsGood(items->result));

    // Publish, then the first notification's value
    const auto publish_value = [&]() -> std::optional<ua::UInt16> {
        (void)call(
            [&](ua::ByteWriter& w) {
                ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::PublishRequest));
                request_header(w, auth_token, 30);
                w.WriteInt32(0);
            },
            false, false);
        const auto r = wait_response();
        if (!r || !ua::IsGood(r->result) || r->type != ua::NodeId(0, ua::ns0::PublishResponse)) return std::nullopt;
        ua::ByteReader b = r->reader();
        ua::UInt32 id = 0, seq = 0, handle = 0;
        ua::Int32 available = 0, notifications = 0, count = 0;
        ua::Boolean more = false;
        ua::DateTime time = 0;
        if (!b.ReadUInt32(id) || id != sub || !b.ReadInt32(available)) return std::nullopt;
        for (ua::Int32 i = 0; i < available; i++) b.ReadUInt32(seq);
        if (!b.ReadBoolean(more) || !b.ReadUInt32(seq) || !b.ReadDateTime(time) || !b.ReadInt32(notifications) ||
            notifications != 1)
            return std::nullopt;
        ua::NodeId type;
        Byte encoding = 0;
        std::string_view body;
        bool null = true;
        if (!ua::DecodeNodeId(b, type) || type != ua::NodeId(0, ua::ns0::DataChangeNotification) || !b.ReadByte(encoding) ||
            !b.ReadByteString(body, null))
            return std::nullopt;
        ua::ByteReader d(std::span<const Byte>(reinterpret_cast<const Byte*>(body.data()), body.size()));
        Byte mask = 0;
        ua::Variant value;
        if (!d.ReadInt32(count) || count != 1 || !d.ReadUInt32(handle) || handle != 77 || !d.ReadByte(mask) ||
            !(mask & 0x01) || !ua::DecodeVariant(d, value) || value.GetIf<ua::UInt16>() == nullptr)
            return std::nullopt;
        return *value.GetIf<ua::UInt16>();
    };

    const auto initial = publish_value();
    ASSERT_TRUE(initial);
    EXPECT_EQ(*initial, 0) << "the initial value";
    const std::array<std::uint16_t, 1> on{1};
    ASSERT_EQ(bank.write(0, on), rtos::modbus::Exception::None);  // as Modbus would
    const auto changed = publish_value();
    ASSERT_TRUE(changed);
    EXPECT_EQ(*changed, 1) << "the change, via Modbus";
    ua::SetClock(nullptr, nullptr);
}

// ---- Hello and SecureChannel states (ported from open62541 tests/server/check_server.c,
// check_services_securechannel.c) ------------------------------------------------------------------

/// A Hello with an EndpointUrl of `url_length` bytes and `trailing` extra bytes: the answer's type.
std::string hello_answer(OpcUaTest& t, std::size_t url_length, std::size_t trailing, ua::StatusCode& error) {
    UaBytes msg(8 + 20 + 4 + url_length + trailing, 0);
    ua::ByteWriter w(msg);
    ua::EncodeTcpHeader(w, "HEL", ua::ChunkType::Final, static_cast<ua::UInt32>(msg.size()));
    w.WriteUInt32(0);
    w.WriteUInt32(65535);
    w.WriteUInt32(65535);
    w.WriteUInt32(0);
    w.WriteUInt32(0);
    w.WriteString(std::string(url_length, 'x'));
    t.send(msg);
    const auto reply = t.receive();
    if (!reply) return "none";
    if (is_message(*reply, "ERR")) {
        ua::ByteReader r(*reply);
        r.Skip(8);
        r.ReadStatusCode(error);
    }
    return std::string(reinterpret_cast<const char*>(reply->data()), 3);
}

// open62541: server/check_server.c helloEndpointUrlLimit
TEST_F(OpcUaTest, helloEndpointUrlLimit) {
    ua::StatusCode error = ua::StatusCode::Good;
    EXPECT_EQ(hello_answer(*this, 4095, 0, error), "ACK");
    for (std::size_t length : {4096u, 4097u, 5000u}) {
        reset_connection();
        poll(1);
        connect();
        EXPECT_EQ(hello_answer(*this, length, 0, error), "ERR") << length;
        EXPECT_EQ(error, ua::StatusCode::BadTcpEndpointUrlInvalid);
    }
}

// open62541: server/check_server.c helloTrailingData
TEST_F(OpcUaTest, helloTrailingData) {
    ua::StatusCode error = ua::StatusCode::Good;
    for (std::size_t trailing : {1u, 8u}) {
        reset_connection();
        poll(1);
        connect();
        EXPECT_EQ(hello_answer(*this, 32, trailing, error), "ERR") << trailing;
        EXPECT_EQ(error, ua::StatusCode::BadDecodingError);
    }
}

// An Issue on an open channel, a Renew without one and an unknown request type are refused
// (open62541: BadInternalError from its state machine; here the ERR of a refused OPN).
// open62541: server/check_services_securechannel.c OpenSecureChannel_issue_wrongState_rejected
// open62541: server/check_services_securechannel.c OpenSecureChannel_renew_wrongState_rejected
// open62541: server/check_services_securechannel.c OpenSecureChannel_unknownRequestType_rejected
TEST_F(OpcUaTest, OpenSecureChannelRequestTypeMustFitTheState) {
    ASSERT_TRUE(hello());
    request_type = 1;  // Renew before any channel
    EXPECT_FALSE(open_none());
    EXPECT_TRUE(closed);
    for (const ua::Int32 type : {0, 99}) {
        reset_connection();
        poll(1);
        channel = token = 0;
        connect();
        ASSERT_TRUE(hello());
        request_type = 0;
        ASSERT_TRUE(open_none());
        request_type = type;  // Issue again on the open channel; an unknown type
        EXPECT_FALSE(open_none()) << type;
        EXPECT_TRUE(closed) << type;
    }
}

// open62541: server/check_services_securechannel.c OpenSecureChannel_renew_nonceReuse_rejected
TEST_F(OpcUaTest, OpenSecureChannel_renew_nonceReuse_rejected) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_basic(client_cert, client_key));
    ASSERT_TRUE(open_basic(client_cert, client_key)) << "a Renew with a fresh nonce";
    fixed_nonce = true;  // the next two opens use the same nonce
    ASSERT_TRUE(open_basic(client_cert, client_key));
    EXPECT_FALSE(open_basic(client_cert, client_key));
    EXPECT_EQ(last_error, ua::StatusCode::BadSecurityChecksFailed);
}

// open62541: server/check_services_securechannel.c CloseSecureChannel_smoke_doesNotCrash
TEST_F(OpcUaTest, CloseSecureChannel_smoke_doesNotCrash) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ua::ByteWriter w(buffer);
    ua::EncodeTcpHeader(w, "CLO", ua::ChunkType::Final, 0);
    w.WriteUInt32(channel);
    w.WriteUInt32(token);
    w.WriteUInt32(++sequence);
    w.WriteUInt32(++request);
    ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::CloseSecureChannelRequest));
    request_header(w, ua::NodeId(), 99);
    w.PatchUInt32(4, static_cast<ua::UInt32>(w.Position()));
    send(w.Written());
    EXPECT_FALSE(receive().has_value()) << "no response (Part 4 5.5.3)";
    EXPECT_TRUE(closed);
}

// SecurityPolicy None with mode Sign, Basic256Sha256 with mode None or an unknown mode:
// rejected (BadSecurityModeRejected).
// open62541: check_securechannel.c SecureChannel_open_invalidSecurityMode
// open62541: check_securechannel_unit.c setSecurityMode_nonePolicyWithSign_rejected
// open62541: check_securechannel_unit.c setSecurityMode_invalidMode_rejected
// open62541: check_securechannel.c SecureChannel_open_validSecurityMode
// open62541: check_securechannel_unit.c setSecurityMode_noneMode_accepted
// open62541: check_securechannel_unit.c setSecurityMode_outOfRange_rejected
TEST_F(OpcUaTest, SecureChannel_open_invalidSecurityMode) {
    ASSERT_TRUE(hello());
    EXPECT_FALSE(open_none(2)) << "None with Sign";
    EXPECT_FALSE(open_none(4)) << "an unknown mode";
    for (const ua::Int32 mode : {1, 4}) {
        reset_connection();
        poll(1);
        channel = token = 0;
        connect();
        ASSERT_TRUE(hello());
        EXPECT_FALSE(open_basic(client_cert, client_key, {}, mode)) << mode;
    }
    reset_connection();
    poll(1);
    channel = token = 0;
    connect();
    ASSERT_TRUE(hello());
    EXPECT_TRUE(open_none(1));
}

// A MSG with another TokenId closes the channel.
// open62541: check_securechannel.c SecureChannel_validateMessageToken
TEST_F(OpcUaTest, SecureChannel_validateMessageToken) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    ASSERT_TRUE(session());
    token += 7;
    EXPECT_EQ(read(kLed).first, ua::StatusCode::BadInternalError) << "no answer";
    (void)receive();
    EXPECT_TRUE(closed);
}

// FindServers: this server, unless the ServerUris name others only.
// open62541: server/check_server_client_readwrite.c client_find_servers
// open62541: server/check_server_client_readwrite.c client_find_servers_with_filter
TEST_F(OpcUaTest, client_find_servers) {
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    const auto find = [&](std::optional<std::string_view> uri) -> ua::Int32 {
        const auto r = call([&](ua::ByteWriter& w) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::FindServersRequest));
            request_header(w, ua::NodeId(), 11);
            w.WriteString(kEndpoint);
            w.WriteInt32(0);
            if (uri) {
                w.WriteInt32(1);
                w.WriteString(*uri);
            } else {
                w.WriteInt32(0);
            }
        });
        if (!r || !ua::IsGood(r->result)) return -1;
        ua::ByteReader b = r->reader();
        ua::Int32 count = -1;
        b.ReadInt32(count);
        if (count == 1) {
            ua::ApplicationDescription app;
            EXPECT_TRUE(ua::DecodeApplicationDescription(b, app));
            EXPECT_EQ(app.applicationUri.value, "urn:factory-control-unit:opcua-native");
        }
        return count;
    };
    EXPECT_EQ(find(std::nullopt), 1);
    EXPECT_EQ(find("urn:factory-control-unit:opcua-native"), 1);
    EXPECT_EQ(find("urn:somewhere:else"), 0);
}

// A product names its own application: GetEndpoints, FindServers and the NamespaceArray follow.
TEST_F(OpcUaTest, ApplicationIdentityIsConfigurable) {
    const ua::ApplicationIdentity product{.applicationUri = "urn:example:press-7",
                                          .productUri = "urn:example:press",
                                          .applicationName = "Press 7"};
    ua::SetApplicationIdentity(product);
    ASSERT_TRUE(hello());
    ASSERT_TRUE(open_none());
    const auto r = call([&](ua::ByteWriter& w) {
        ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::FindServersRequest));
        request_header(w, ua::NodeId(), 11);
        w.WriteString(kEndpoint);
        w.WriteInt32(0);
        w.WriteInt32(1);
        w.WriteString(std::string_view("urn:example:press-7"));
    });
    ua::SetApplicationIdentity({});  // back to the defaults for the other tests
    ASSERT_TRUE(r && ua::IsGood(r->result));
    ua::ByteReader b = r->reader();
    ua::Int32 count = -1;
    ASSERT_TRUE(b.ReadInt32(count));
    ASSERT_EQ(count, 1);
    ua::ApplicationDescription app;
    ASSERT_TRUE(ua::DecodeApplicationDescription(b, app));
    EXPECT_EQ(app.applicationUri.value, "urn:example:press-7");
    EXPECT_EQ(app.productUri.value, "urn:example:press");
    EXPECT_EQ(app.applicationName.text, "Press 7");
}

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

