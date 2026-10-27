#pragma once
// SecureChannel layer (Part 6 7.1.2/7.2, Part 4 5.5): OPN/MSG/CLO message framing plus the
// OpenSecureChannel/CloseSecureChannel service handlers.
//
// SecurityPolicy#Basic256Sha256, SecurityMode Sign or SignAndEncrypt: every OPN message is BOTH
// signed (RSASSA-PKCS1-v1_5-SHA256) and encrypted (RSAES-OAEP-SHA1) with asymmetric keys,
// regardless of the negotiated MessageSecurityMode -- Part 6 6.7.4 mandates this for the channel
// handshake itself. MSG/CLO messages are HMAC-SHA256 signed (Sign) and, with SignAndEncrypt,
// AES-256-CBC encrypted from the sequence header on (Part 6 6.7.2): padding to the block size,
// signature over the plaintext, then encryption -- keys and IVs derived per direction with
// P_SHA256 from the nonces (6.7.5).
//
// rtoos: client certificates must chain to the trusted CA (rtoosCA) and be valid now
// (security::CheckClientCertificate) -- only such clients get a signed channel. A channel may
// also be opened with SecurityPolicy#None: no crypto at all, and READ-ONLY (GetEndpoints,
// sessions, Read, Browse; Write is rejected -- connection.cpp). Real clients, UAExpert
// included, start with None for GetEndpoints (Part 4 5.4.4) -- see SecureChannel::IsUnsecured().
#include <span>

#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/security_crypto.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/tcp_message.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

constexpr char SECURITY_POLICY_BASIC256SHA256_URI[] = "http://opcfoundation.org/UA/SecurityPolicy#Basic256Sha256";
constexpr char SECURITY_POLICY_NONE_URI[] = "http://opcfoundation.org/UA/SecurityPolicy#None";
constexpr Int32 SECURITY_MODE_NONE = 1; // MessageSecurityMode, Part 4 7.15
constexpr Int32 SECURITY_MODE_SIGN = 2;
constexpr Int32 SECURITY_MODE_SIGN_AND_ENCRYPT = 3;

// This server's own certificate + private key -- set ONCE at startup (OpcUaServer) and shared by
// every Connection/SecureChannel. rtoos: an RSA-2048 application instance certificate of its
// own (the HTTPS certificate is ECDSA P-256); the key is loaded into wolfCrypt
// (security::LoadServerKey). The spans point at the embedded certificate blobs.
struct ServerIdentity {
    std::span<const Byte> certificateDer; // leaf only -- used to extract this server's RSA key
    // Root CA cert (DER), appended after the leaf whenever the certificate is sent over the wire
    // (SenderCertificate/serverCertificate), so a client (e.g. UAExpert) can build a complete
    // trust chain without a separate out-of-band CA fetch -- found necessary via live UAExpert
    // testing 2026-08-19 (BadCertificateChainIncomplete when only the leaf was sent).
    std::span<const Byte> caCertificateDer;
    security::RsaPrivateKey privateKey;
};
void SetServerIdentity(const ServerIdentity &identity);
const ServerIdentity &GetServerIdentity();

// Writes the server's certificate CHAIN (leaf + root CA) as a single ByteString, matching how
// serverCertificate/SenderCertificate is encoded on the wire (Part 6 7.10) -- built from two
// separately-embedded DER blobs without needing to physically concatenate them in memory first.
bool WriteServerCertificateChain(ByteWriter &w);

// A fully verified/decrypted MSG or CLO service body, ready for TypeId dispatch.
struct ParsedSecureMessage {
    TcpHeader header;
    UInt32 channelId = 0;
    UInt32 tokenId = 0;
    UInt32 sequenceNumber = 0;
    UInt32 requestId = 0;
    std::span<const Byte> body; // plaintext: TypeId (NodeId) + the service struct's own fields
};

// Writes a SymmetricAlgorithmSecurityHeader'd MSG/CLO frame header, leaving "w" positioned for
// the caller to encode the service body -- pair with SecureChannel::SignAndFinishSymmetric().
bool BeginSymmetricMessage(ByteWriter &w, const char messageType[3], UInt32 channelId,
                           UInt32 tokenId, UInt32 sequenceNumber, UInt32 requestId,
                           size_t &startPosOut);

// Per-connection SecureChannel state (Part 4 5.5.2/5.5.3). One instance per Connection -- this
// server ties exactly one SecureChannel to one TCP connection (no channel migration/renewal
// across connections, see connection.hpp).
class SecureChannel {
public:
    bool IsOpen() const { return isOpen_; }
    // True if this channel was opened with SecurityPolicy#None -- read-only (Connection::
    // HandleServiceMessage enforces this), see the file header comment.
    bool IsUnsecured() const { return isDiscoveryOnly_; }
    // Result of the last client certificate check (Basic256Sha256 OPN).
    security::CertificateCheck LastCertificateCheck() const { return certificateCheck_; }
    // MessageSecurityMode of the open channel (SECURITY_MODE_*).
    Int32 SecurityMode() const { return securityMode_; }
    // The client's application certificate of a secured channel: SHA-1 thumbprint of its leaf
    // and its RSA-2048 public key (CreateSession/ActivateSession check against these).
    std::span<const Byte, security::SHA1_DIGEST_SIZE> ClientThumbprint() const { return clientThumbprint_; }
    security::RsaPublicKey ClientKey() const {
        return {std::span<const Byte>(clientModulus_, clientModulusSize_),
                std::span<const Byte>(clientExponent_, clientExponentSize_)};
    }
    UInt32 ChannelId() const { return channelId_; }
    UInt32 TokenId() const { return tokenId_; }
    // Pre-increment, not post: serverSequenceNumber_ starts at 0 (zero-init, see the .bss/.data
    // comment on SessionTransport::bound_ in net_transport.hpp for why that matters), but Part 6
    // 6.7.5 still wants the first sequence number a channel ever sends to be 1 -- pre-increment
    // gives the same 1, 2, 3, ... sequence a post-increment-from-1 default would have.
    UInt32 NextServerSequenceNumber() { return ++serverSequenceNumber_; }

    // "fullOpnMessage" is exactly one complete OPN message (header.messageSize == size()).
    // Accepts SecurityPolicy#Basic256Sha256 (parses+decrypts+verifies the request using the
    // client certificate embedded in the message itself, plus this server's own identity,
    // derives fresh HMAC-SHA256 signing keys from the ClientNonce/ServerNonce exchange per Part 6
    // 6.7.5, and writes a fully signed+encrypted OPN response) or SecurityPolicy#None (no crypto
    // at all, IsDiscoveryOnly() becomes true) into "w". Assigns a fresh ChannelId+TokenId on
    // first call (Issue), or revises the token on a later call for the same channel (Renew) --
    // Renew re-derives keys from a fresh nonce exchange but otherwise just re-issues the same
    // ChannelId/TokenId, since this server has no multi-token overlap/rollover logic yet (an
    // accepted simplification, see the project's OPC UA design notes).
    bool HandleOpen(std::span<const Byte> fullOpnMessage, UInt32 assignedChannelId, ByteWriter &w);

    // "fullCloMessage" is exactly one complete CLO message. There is no response either way
    // (Part 4 5.5.3); returns false only if the message didn't even verify/decode (caller closes
    // the connection regardless).
    bool HandleClose(std::span<const Byte> fullCloMessage);

    // Verifies the HMAC-SHA256 signature and strips padding from a complete MSG/CLO message,
    // exposing the plaintext service body via "out". Requires IsOpen().
    bool VerifyAndUnwrapSymmetric(std::span<const Byte> fullMessage, ParsedSecureMessage &out);

    // Pairs with BeginSymmetricMessage(): appends PaddingSize+Padding+HMAC-SHA256 signature to
    // the response service body already written into "w" from "startPos", then backfills the
    // message-size field. Replaces the old policy-agnostic FinishSecureMessage for MSG/CLO.
    bool SignAndFinishSymmetric(ByteWriter &w, size_t startPos);

private:
    bool isOpen_ = false;
    bool isDiscoveryOnly_ = false;
    UInt32 channelId_ = 0;
    UInt32 tokenId_ = 0;
    DateTime createdAt_ = 0;
    UInt32 revisedLifetimeMs_ = 0;
    UInt32 serverSequenceNumber_ = 0; // see NextServerSequenceNumber() comment: pre-incremented
    security::CertificateCheck certificateCheck_ = security::CertificateCheck::Ok;
    Int32 securityMode_ = 0;
    Byte clientThumbprint_[security::SHA1_DIGEST_SIZE] = {};
    Byte clientModulus_[security::MAX_RSA_MODULUS_BYTES] = {};
    Byte clientExponent_[8] = {};
    size_t clientModulusSize_ = 0;
    size_t clientExponentSize_ = 0;

    // Keys derived from the OPN nonce exchange (Part 6 6.7.5): per direction the HMAC-SHA256
    // signing key, the AES-256 encrypting key and the IV (the latter two: SignAndEncrypt).
    struct Keys {
        Byte signing[security::SHA256_DIGEST_SIZE] = {};
        Byte encrypting[security::AES_KEY_SIZE] = {};
        Byte iv[security::AES_BLOCK_SIZE] = {};
    };
    Keys client_{};  // what the client sends: verify, decrypt
    Keys server_{};  // what this server sends: sign, encrypt
    bool Encrypted() const { return securityMode_ == SECURITY_MODE_SIGN_AND_ENCRYPT; }
};

}  // namespace rtos::opcua
