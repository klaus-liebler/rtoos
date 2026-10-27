#include "rtos/opcua/secure_channel.hpp"

#include <cstring>

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"

namespace rtos::opcua {

namespace {

constexpr UInt32 DEFAULT_CHANNEL_LIFETIME_MS = 3600000;
constexpr size_t NONCE_SIZE = 32; // Basic256Sha256's symmetric key length, also used as nonce size

const ServerIdentity *g_serverIdentity = nullptr;

// Parsed once at startup (opcua_setup.cpp) via SetServerIdentity(); everything else below only
// ever reads it, never concurrently (single-threaded NX_TCPSERVER callback model, see
// net_transport.hpp).
using security::RsaPrivateKey;
using security::RsaPublicKey;

// rtoos: buffers of HandleOpen as one static scratch area instead of ~11 KB of stack (the
// network thread runs every OPC UA connection, one message at a time -- never concurrently).
constexpr size_t MAX_CERT_DER = 2000;
constexpr size_t MAX_PLAIN = 1536;
constexpr size_t MAX_SIGNED = MAX_CERT_DER + MAX_PLAIN + 64;
struct OpenScratch {
    Byte plainBuf[MAX_PLAIN];
    Byte signedBuf[MAX_SIGNED];
    Byte plaintext[MAX_PLAIN];
    Byte toSign[MAX_SIGNED];
    Byte clientModulus[security::MAX_RSA_MODULUS_BYTES];
    Byte clientExponent[8];
};
OpenScratch g_scratch;

// SignAndEncrypt: decrypted copy of the message being handled (one at a time).
Byte g_decrypted[8192];
// MSG/CLO: message header (8), SecureChannelId (4), TokenId (4) stay in clear text
constexpr size_t SYMMETRIC_CLEAR_HEADER = 16;
// derived key material per direction: signing key, encrypting key, IV (Part 6 6.7.5)
constexpr size_t KEY_MATERIAL = security::SHA256_DIGEST_SIZE + security::AES_KEY_SIZE + security::AES_BLOCK_SIZE;

// The client's certificate (SenderCertificate) must chain to the trusted CA and be valid
// now (security_crypto.hpp); yields its RSA-2048 public key.
security::CertificateCheck CheckClientCertificate(std::string_view certDer, RsaPublicKey &outKey) {
    const DateTime now = Now();
    const UInt64 unixNow = IsSynced() ? static_cast<UInt64>((now - DATETIME_UNIX_EPOCH) / DATETIME_SEC) : 0;
    return security::CheckClientCertificate(
        std::span<const Byte>(reinterpret_cast<const Byte *>(certDer.data()), certDer.size()), unixNow,
        std::span<Byte, security::MAX_RSA_MODULUS_BYTES>(g_scratch.clientModulus),
        std::span<Byte, 8>(g_scratch.clientExponent), outKey);
}

// HMAC-SHA256 sign/verify for symmetric (MSG/CLO) messages -- Sign mode, no padding (Part 6
// 6.7.2/6.7.3: Padding is only present when the message is also encrypted, which MSG/CLO never
// is here -- SecurityMode Sign only, see the file header comment).
bool SignSymmetric(std::span<const Byte> key, std::span<const Byte> message, Byte outSignature[security::SHA256_DIGEST_SIZE]) {
    return security::HmacSha256(key, message, outSignature);
}
bool VerifySymmetric(std::span<const Byte> key, std::span<const Byte> message, std::span<const Byte> signature) {
    if(signature.size() != security::SHA256_DIGEST_SIZE) return false;
    Byte expected[security::SHA256_DIGEST_SIZE];
    if(!security::HmacSha256(key, message, expected)) return false;
    // constant time: a byte-wise early exit would tell an attacker how much of a forged
    // signature was right
    return security::ConstantTimeEqual(expected, signature);
}

} // namespace

void SetServerIdentity(const ServerIdentity &identity) { g_serverIdentity = &identity; }
const ServerIdentity &GetServerIdentity() { return *g_serverIdentity; }

bool WriteServerCertificateChain(ByteWriter &w) {
    const auto &id = GetServerIdentity();
    size_t totalLen = id.certificateDer.size() + id.caCertificateDer.size();
    if(!w.WriteInt32((Int32)totalLen)) return false;
    if(!w.WriteRaw(id.certificateDer.data(), id.certificateDer.size())) return false;
    return w.WriteRaw(id.caCertificateDer.data(), id.caCertificateDer.size());
}

bool BeginSymmetricMessage(ByteWriter &w, const char messageType[3], UInt32 channelId,
                           UInt32 tokenId, UInt32 sequenceNumber, UInt32 requestId,
                           size_t &startPosOut) {
    startPosOut = w.Position();
    if(!EncodeTcpHeader(w, messageType, ChunkType::Final, 0)) return false;
    if(!w.WriteUInt32(channelId)) return false;
    if(!w.WriteUInt32(tokenId)) return false;
    if(!w.WriteUInt32(sequenceNumber)) return false;
    return w.WriteUInt32(requestId);
}

bool SecureChannel::SignAndFinishSymmetric(ByteWriter &w, size_t startPos) {
    if(!w.Ok()) return false;
    // SecurityPolicy#None (discovery-only channel, see the file header comment): plain framing,
    // no signature -- matches how this server behaved before Basic256Sha256 existed.
    if(isDiscoveryOnly_) {
        UInt32 totalSize = static_cast<UInt32>(w.Position() - startPos);
        return w.PatchUInt32(startPos + 4, totalSize);
    }
    // SignAndEncrypt: padding first (Part 6 6.7.2) -- PaddingSize byte plus that many bytes of
    // the same value, so that sequence header + body + padding + signature fill whole AES blocks.
    if(Encrypted()) {
        const size_t plain = w.Position() - (startPos + SYMMETRIC_CLEAR_HEADER);
        const size_t pad = (security::AES_BLOCK_SIZE -
                            (plain + 1 + security::SHA256_DIGEST_SIZE) % security::AES_BLOCK_SIZE) %
                           security::AES_BLOCK_SIZE;
        for(size_t i = 0; i <= pad; i++)
            if(!w.WriteByte(static_cast<Byte>(pad))) return false;
    }
    // The signature must cover the EXACT bytes actually sent, including the final messageSize
    // field -- so messageSize must be patched in BEFORE signing, not after (same bug/fix as
    // HandleOpen's response construction above; here it's simpler since the signature is always
    // exactly SHA256_DIGEST_SIZE bytes -- no OAEP block padding to account for).
    UInt32 totalSize = static_cast<UInt32>(w.Position() - startPos + security::SHA256_DIGEST_SIZE);
    if(!w.PatchUInt32(startPos + 4, totalSize)) return false;
    // Sign over everything written so far for this message (header through body, now with the
    // FINAL size already in place) -- the signature covers the whole on-wire message except
    // itself, per Part 6 6.7.3.
    Byte signature[security::SHA256_DIGEST_SIZE];
    if(!SignSymmetric(server_.signing, w.Written().subspan(startPos), signature))
        return false;
    if(!w.WriteRaw(signature, sizeof(signature))) return false;
    if(!Encrypted()) return true;
    // encrypt in place: everything after the cleartext header (sequence header on)
    return security::AesCbcEncrypt(server_.encrypting, server_.iv, w.WrittenFrom(startPos + SYMMETRIC_CLEAR_HEADER));
}

bool SecureChannel::VerifyAndUnwrapSymmetric(std::span<const Byte> msg, ParsedSecureMessage &out) {
    if(!isOpen_) return false;
    ByteReader r(msg);
    if(!DecodeTcpHeader(r, out.header)) return false;
    if(!(out.header.Is("MSG") || out.header.Is("CLO"))) return false;
    if(out.header.chunkType != ChunkType::Final) return false;
    if(out.header.messageSize != msg.size()) return false;
    if(!r.ReadUInt32(out.channelId)) return false;
    if(!r.ReadUInt32(out.tokenId)) return false;
    if(!r.Ok()) return false;

    // SignAndEncrypt: everything after the cleartext header is encrypted -- decrypt into the
    // scratch buffer (the network thread handles one message at a time) and go on from there.
    std::span<const Byte> plain = msg;
    if(Encrypted()) {
        if(msg.size() > sizeof g_decrypted || msg.size() <= SYMMETRIC_CLEAR_HEADER ||
           (msg.size() - SYMMETRIC_CLEAR_HEADER) % security::AES_BLOCK_SIZE != 0)
            return false;
        std::memcpy(g_decrypted, msg.data(), msg.size());
        if(!security::AesCbcDecrypt(client_.encrypting, client_.iv,
                                    std::span<Byte>(g_decrypted + SYMMETRIC_CLEAR_HEADER, msg.size() - SYMMETRIC_CLEAR_HEADER)))
            return false;
        plain = std::span<const Byte>(g_decrypted, msg.size());
    }
    ByteReader seq(plain.subspan(SYMMETRIC_CLEAR_HEADER));
    if(!seq.ReadUInt32(out.sequenceNumber)) return false;
    if(!seq.ReadUInt32(out.requestId)) return false;
    const size_t bodyStart = SYMMETRIC_CLEAR_HEADER + 8;

    if(isDiscoveryOnly_) {
        // No signature to strip -- see SignAndFinishSymmetric's mirror-image comment.
        out.body = plain.subspan(bodyStart);
        return true;
    }

    if(plain.size() < bodyStart + security::SHA256_DIGEST_SIZE) return false;
    size_t signatureStart = plain.size() - security::SHA256_DIGEST_SIZE;
    std::span<const Byte> signedPart = plain.subspan(0, signatureStart);
    std::span<const Byte> signature = plain.subspan(signatureStart);
    if(!VerifySymmetric(client_.signing, signedPart, signature))
        return false;

    size_t bodyEnd = signatureStart;
    if(Encrypted()) {
        // padding is checked only after the signature: no padding oracle (the signature covers it)
        const size_t pad = plain[signatureStart - 1];
        if(bodyStart + pad + 1 > signatureStart) return false;
        for(size_t i = signatureStart - 1 - pad; i < signatureStart; i++)
            if(plain[i] != pad) return false;
        bodyEnd = signatureStart - 1 - pad;
    }
    out.body = plain.subspan(bodyStart, bodyEnd - bodyStart);
    return true;
}

bool SecureChannel::HandleClose(std::span<const Byte> fullCloMessage) {
    ParsedSecureMessage parsed;
    bool ok = VerifyAndUnwrapSymmetric(fullCloMessage, parsed) &&
             parsed.channelId == channelId_ && parsed.tokenId == tokenId_;
    isOpen_ = false;
    if(!ok) return false;
    ByteReader body(parsed.body);
    NodeId typeId;
    return DecodeNodeId(body, typeId);
}

namespace {

// Builds the PKCS#1-style Padding region (Part 6 6.7.2 Table 61) for an ENCRYPTED message:
// PaddingSizeByte(1) + PaddingBytes(paddingCount, each = paddingCount). RSA keys <= 2048 bits
// (this project's only configuration) never need the optional ExtraPaddingSize byte.
void AppendAsymmetricPadding(Byte *dest, size_t paddingCount) {
    dest[0] = (Byte)paddingCount;
    std::memset(dest + 1, (int)paddingCount, paddingCount);
}

} // namespace

bool SecureChannel::HandleOpen(std::span<const Byte> fullOpnMessage, UInt32 assignedChannelId, ByteWriter &w) {
    // --- Parse the plaintext MessageHeader + AsymmetricAlgorithmSecurityHeader ---
    ByteReader r(fullOpnMessage);
    TcpHeader header;
    if(!DecodeTcpHeader(r, header)) return false;
    if(!header.Is("OPN") || header.chunkType != ChunkType::Final) return false;
    if(header.messageSize != fullOpnMessage.size()) return false;

    UInt32 msgChannelId = 0;
    if(!r.ReadUInt32(msgChannelId)) return false;
    String policyUri;
    std::string_view clientCertDer, receiverThumbprint;
    bool clientCertIsNull = true, receiverThumbprintIsNull = true;
    if(!r.ReadString(policyUri)) return false;
    if(!r.ReadByteString(clientCertDer, clientCertIsNull)) return false;
    if(!r.ReadByteString(receiverThumbprint, receiverThumbprintIsNull)) return false;
    if(!r.Ok()) return false;
    if(policyUri.isNull) return false;

    // SecurityPolicy#None: discovery-only channel (Part 4 5.4.4), no crypto at all -- everything
    // after the security header is the plaintext SeqHeader+OpenSecureChannelRequest body
    // directly. See the file header comment for why this exception exists.
    if(policyUri.value == SECURITY_POLICY_NONE_URI) {
        ByteReader seq(r.RemainingSpan());
        UInt32 reqSeqNum = 0, reqId = 0;
        if(!seq.ReadUInt32(reqSeqNum)) return false;
        if(!seq.ReadUInt32(reqId)) return false;
        NodeId typeId;
        if(!DecodeNodeId(seq, typeId) || typeId != NodeId(0, ns0::OpenSecureChannelRequest)) return false;
        RequestHeader reqHeader;
        if(!DecodeRequestHeader(seq, reqHeader)) return false;
        UInt32 clientProtocolVersion = 0;
        Int32 requestType = 0, securityMode = 0;
        std::string_view clientNonce;
        bool clientNonceIsNull = true;
        UInt32 requestedLifetimeMs = 0;
        if(!seq.ReadUInt32(clientProtocolVersion)) return false;
        if(!seq.ReadInt32(requestType)) return false;
        if(!seq.ReadInt32(securityMode)) return false;
        if(!seq.ReadByteString(clientNonce, clientNonceIsNull)) return false;
        if(!seq.ReadUInt32(requestedLifetimeMs)) return false;
        if(!seq.Ok()) return false;
        (void)clientProtocolVersion;
        if(!RequestTypeValid(requestType)) return false;

        StatusCode result = (securityMode == SECURITY_MODE_NONE)
            ? StatusCode::Good : StatusCode::BadSecurityModeRejected;
        if(IsGood(result)) {
            isOpen_ = true;
            isDiscoveryOnly_ = true;
            securityMode_ = SECURITY_MODE_NONE;
            channelId_ = assignedChannelId;
            tokenId_ += 1;
            createdAt_ = Now();
            revisedLifetimeMs_ = (requestedLifetimeMs != 0) ? requestedLifetimeMs : DEFAULT_CHANNEL_LIFETIME_MS;
        }

        if(!EncodeTcpHeader(w, "OPN", ChunkType::Final, 0)) return false; // patched below
        if(!w.WriteUInt32(IsGood(result) ? channelId_ : assignedChannelId)) return false;
        if(!w.WriteString(SECURITY_POLICY_NONE_URI)) return false;
        if(!w.WriteByteString(std::string_view{}, true)) return false; // SenderCertificate = null
        if(!w.WriteByteString(std::string_view{}, true)) return false; // ReceiverCertificateThumbprint = null
        if(!w.WriteUInt32(NextServerSequenceNumber())) return false;
        if(!w.WriteUInt32(reqId)) return false;
        if(!EncodeNodeId(w, NodeId(0, ns0::OpenSecureChannelResponse))) return false;
        if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, result))) return false;
        if(!w.WriteUInt32(0)) return false; // ServerProtocolVersion
        if(!w.WriteUInt32(channelId_)) return false;
        if(!w.WriteUInt32(tokenId_)) return false;
        if(!w.WriteDateTime(createdAt_)) return false;
        if(!w.WriteUInt32(revisedLifetimeMs_)) return false;
        if(!w.WriteByteString(std::string_view{}, true)) return false; // ServerNonce = null (no crypto)
        if(!w.Ok()) return false;
        UInt32 totalSize = static_cast<UInt32>(w.Position());
        return w.PatchUInt32(4, totalSize);
    }

    if(policyUri.value != SECURITY_POLICY_BASIC256SHA256_URI) return false;
    if(clientCertIsNull || clientCertDer.empty()) return false; // mandatory once policy != None
    // Defensive upper bound (real RSA-2048 certs are typically 900-1500 bytes) -- keeps the
    // fixed-size scratch buffers sized safely (see MAX_SIGNED).
    if(clientCertDer.size() > MAX_CERT_DER) return false;
    // SenderCertificate may be a chain (leaf first): the leaf is the client's identity.
    const std::span<const Byte> clientLeaf =
        security::LeafCertificate(std::span<const Byte>((const Byte *)clientCertDer.data(), clientCertDer.size()));
    if(clientLeaf.empty()) return false;
    clientCertDer = std::string_view((const char *)clientLeaf.data(), clientLeaf.size());

    RsaPublicKey clientKey;
    certificateCheck_ = CheckClientCertificate(clientCertDer, clientKey);
    if(certificateCheck_ != security::CertificateCheck::Ok) return false;
    size_t clientModLen = clientKey.modulus.size();

    // --- Decrypt the ciphertext (everything after the security header) with our private key ---
    std::span<const Byte> ciphertext = r.RemainingSpan();
    const RsaPrivateKey &ourKey = GetServerIdentity().privateKey;
    size_t ourModLen = ourKey.modulusSize;
    size_t ourPlainBlock = security::RsaOaepMaxPlaintext(ourModLen);
    if(ciphertext.empty() || ciphertext.size() % ourModLen != 0) return false;

    size_t plainLen = (ciphertext.size() / ourModLen) * ourPlainBlock;
    if(plainLen > MAX_PLAIN || plainLen < NONCE_SIZE + 1 + clientModLen) return false;
    Byte *plainBuf = g_scratch.plainBuf;
    if(!security::RsaOaepDecryptBlocks(ourKey, ciphertext, std::span<Byte>(plainBuf, plainLen))) return false;

    // --- Locate Signature/Padding/SeqHeader+Body within the decrypted plaintext ---
    size_t signatureStart = plainLen - clientModLen;
    if(signatureStart == 0) return false;
    size_t paddingCount = plainBuf[signatureStart - 1];
    if(paddingCount + 1 > signatureStart) return false;
    size_t paddingSizeByteIdx = signatureStart - 1 - paddingCount;
    for(size_t i = 0; i < paddingCount; i++) {
        if(plainBuf[paddingSizeByteIdx + 1 + i] != (Byte)paddingCount) return false;
    }
    size_t seqHeaderPlusBodyLen = paddingSizeByteIdx;
    if(seqHeaderPlusBodyLen < 8) return false;

    // --- Verify the RSA-PKCS1v15-SHA256 signature over [cleartext header/secheader][plaintext
    // up to the signature] ---
    size_t cleartextLen = r.Position(); // header + security header, still in fullOpnMessage
    // Bounds both the request-verify concatenation (cleartextLen, dominated by the CLIENT's
    // certificate bytes, capped by MAX_CERT_DER above) and the response-sign concatenation
    // further below (headerSecHeaderLen, dominated by OUR OWN certificate bytes).
    if(cleartextLen + signatureStart > MAX_SIGNED) return false;
    Byte *signedBuf = g_scratch.signedBuf;
    std::memcpy(signedBuf, fullOpnMessage.data(), cleartextLen);
    std::memcpy(signedBuf + cleartextLen, plainBuf, signatureStart);
    if(!security::RsaSha256Verify(clientKey, std::span<const Byte>(signedBuf, cleartextLen + signatureStart),
                                  std::span<const Byte>(plainBuf + signatureStart, clientModLen)))
        return false;

    // --- Decode the OpenSecureChannelRequest body ---
    ByteReader seq(std::span<const Byte>(plainBuf, seqHeaderPlusBodyLen));
    UInt32 reqSeqNum = 0, reqId = 0;
    if(!seq.ReadUInt32(reqSeqNum)) return false;
    if(!seq.ReadUInt32(reqId)) return false;

    NodeId typeId;
    if(!DecodeNodeId(seq, typeId) || typeId != NodeId(0, ns0::OpenSecureChannelRequest)) return false;
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(seq, reqHeader)) return false;
    UInt32 clientProtocolVersion = 0;
    Int32 requestType = 0, securityMode = 0;
    std::string_view clientNonce;
    bool clientNonceIsNull = true;
    UInt32 requestedLifetimeMs = 0;
    if(!seq.ReadUInt32(clientProtocolVersion)) return false;
    if(!seq.ReadInt32(requestType)) return false;
    if(!seq.ReadInt32(securityMode)) return false;
    if(!seq.ReadByteString(clientNonce, clientNonceIsNull)) return false;
    if(!seq.ReadUInt32(requestedLifetimeMs)) return false;
    if(!seq.Ok()) return false;
    (void)clientProtocolVersion;
    if(!RequestTypeValid(requestType)) return false;
    // a renewal brings a new nonce: reusing the old one would give the old keys again
    if(requestType == REQUEST_TYPE_RENEW && !clientNonceIsNull && clientNonce.size() == NONCE_SIZE &&
       security::ConstantTimeEqual(std::span<const Byte>((const Byte *)clientNonce.data(), NONCE_SIZE), lastClientNonce_))
        return false;

    StatusCode result = (securityMode == SECURITY_MODE_SIGN || securityMode == SECURITY_MODE_SIGN_AND_ENCRYPT)
        ? StatusCode::Good : StatusCode::BadSecurityModeRejected;
    if(IsGood(result) && (clientNonceIsNull || clientNonce.size() != NONCE_SIZE))
        result = StatusCode::BadSecurityChecksFailed;

    Byte serverNonce[NONCE_SIZE] = {};
    if(IsGood(result)) {
        isOpen_ = true;
        isDiscoveryOnly_ = false;
        securityMode_ = securityMode;
        std::memcpy(clientModulus_, clientKey.modulus.data(), clientKey.modulus.size());
        std::memcpy(clientExponent_, clientKey.exponent.data(), clientKey.exponent.size());
        clientModulusSize_ = clientKey.modulus.size();
        clientExponentSize_ = clientKey.exponent.size();
        if(!security::Sha1(clientLeaf, clientThumbprint_)) return false;
        channelId_ = assignedChannelId;
        tokenId_ += 1;
        createdAt_ = Now();
        revisedLifetimeMs_ = (requestedLifetimeMs != 0) ? requestedLifetimeMs : DEFAULT_CHANNEL_LIFETIME_MS;

        if(!security::Random(std::span<Byte>(serverNonce, NONCE_SIZE)))
            return false;
        std::span<const Byte> clientNonceSpan((const Byte *)clientNonce.data(), clientNonce.size());
        std::span<const Byte> serverNonceSpan(serverNonce, NONCE_SIZE);
        // Part 6 6.7.5: "Client*" keys (this server uses to VERIFY/DECRYPT what the client sends)
        // are derived with ServerNonce as secret, ClientNonce as seed; "Server*" keys (this server
        // uses to SIGN/ENCRYPT what it sends) the other way around. Each block: signing key,
        // encrypting key, initialization vector.
        const auto derive = [](std::span<const Byte> secret, std::span<const Byte> seed, Keys &keys) {
            Byte material[KEY_MATERIAL];
            if(!security::PSha256(secret, seed, material)) return false;
            std::memcpy(keys.signing, material, sizeof keys.signing);
            std::memcpy(keys.encrypting, material + sizeof keys.signing, sizeof keys.encrypting);
            std::memcpy(keys.iv, material + sizeof keys.signing + sizeof keys.encrypting, sizeof keys.iv);
            return true;
        };
        if(!derive(serverNonceSpan, clientNonceSpan, client_) || !derive(clientNonceSpan, serverNonceSpan, server_))
            return false;
        std::memcpy(lastClientNonce_, clientNonce.data(), NONCE_SIZE);
    }

    // --- Build the OpenSecureChannelResponse body (SeqHeader + Body) ---
    Byte bodyBuf[512];
    ByteWriter bw(bodyBuf);
    if(!bw.WriteUInt32(NextServerSequenceNumber())) return false;
    if(!bw.WriteUInt32(reqId)) return false;
    if(!EncodeNodeId(bw, NodeId(0, ns0::OpenSecureChannelResponse))) return false;
    if(!EncodeResponseHeader(bw, MakeResponseHeader(reqHeader, result))) return false;
    if(!bw.WriteUInt32(0)) return false; // ServerProtocolVersion
    if(!bw.WriteUInt32(IsGood(result) ? channelId_ : assignedChannelId)) return false;
    if(!bw.WriteUInt32(tokenId_)) return false;
    if(!bw.WriteDateTime(createdAt_)) return false;
    if(!bw.WriteUInt32(revisedLifetimeMs_)) return false;
    if(!bw.WriteByteString(std::string_view((const char *)serverNonce, NONCE_SIZE), !IsGood(result))) return false;
    if(!bw.Ok()) return false;
    size_t seqHeaderPlusRespBodyLen = bw.Position();

    // --- Sign + OAEP-encrypt the response (Part 6 6.7.4 -- always both, regardless of mode) ---
    size_t plainBlock = security::RsaOaepMaxPlaintext(clientModLen);
    size_t baseLen = seqHeaderPlusRespBodyLen + 1 + ourModLen; // +1 PaddingSizeByte, +signature
    size_t pad = (plainBlock - (baseLen % plainBlock)) % plainBlock;
    size_t plaintextLen = baseLen + pad;
    size_t numBlocks = plaintextLen / plainBlock;
    size_t cipherLen = numBlocks * clientModLen;

    // The signature must cover the EXACT bytes actually sent, including the final messageSize
    // field -- so messageSize has to be correct THE FIRST TIME the header is written, not
    // patched in afterward (a patch-after-sign would leave the signature covering a different
    // messageSize than the one on the wire, failing verification on the receiving end 100% of
    // the time -- found via the Python test client's very first live handshake attempt).
    // Every length below is already known before writing a single byte of the header.
    const auto &ourCertDer = GetServerIdentity().certificateDer;
    const auto &ourCaCertDer = GetServerIdentity().caCertificateDer;
    size_t headerSecHeaderLen = TCP_HEADER_SIZE + 4 /*ChannelId*/
        + 4 + sizeof(SECURITY_POLICY_BASIC256SHA256_URI) - 1 /*policyUri string*/
        + 4 + ourCertDer.size() + ourCaCertDer.size() /*SenderCertificate = leaf + CA chain*/
        + 4 + security::SHA1_DIGEST_SIZE /*ReceiverCertificateThumbprint*/;
    UInt32 finalMessageSize = (UInt32)(headerSecHeaderLen + cipherLen);

    if(!EncodeTcpHeader(w, "OPN", ChunkType::Final, finalMessageSize)) return false;
    // The response's SecureChannelId must be the (possibly newly-assigned) real channel id, NOT
    // an echo of the request's own SecureChannelId (which a first-time Issue always sets to 0,
    // "assign one" -- msgChannelId above is that REQUEST value, wrong for the response header).
    if(!w.WriteUInt32(IsGood(result) ? channelId_ : assignedChannelId)) return false;
    if(!w.WriteString(SECURITY_POLICY_BASIC256SHA256_URI)) return false;
    if(!WriteServerCertificateChain(w)) return false; // leaf + CA, see the file header comment
    Byte clientThumbprint[security::SHA1_DIGEST_SIZE];
    if(!security::Sha1(std::span<const Byte>((const Byte *)clientCertDer.data(), clientCertDer.size()), clientThumbprint))
        return false;
    if(!w.WriteByteString(std::string_view((const char *)clientThumbprint, sizeof(clientThumbprint)), false))
        return false;
    if(!w.Ok() || w.Position() != headerSecHeaderLen) return false; // sanity-check the precomputed length

    if(plaintextLen > MAX_PLAIN) return false;
    Byte *plaintext = g_scratch.plaintext;
    std::memcpy(plaintext, bodyBuf, seqHeaderPlusRespBodyLen);
    AppendAsymmetricPadding(plaintext + seqHeaderPlusRespBodyLen, pad);
    // Sign over [already-written header+secheader in w, now with the FINAL size][seqHeader+body+padding].
    size_t toSignLen = headerSecHeaderLen + seqHeaderPlusRespBodyLen + 1 + pad;
    if(toSignLen > MAX_SIGNED) return false;
    Byte *toSign = g_scratch.toSign;
    std::memcpy(toSign, w.Written().data(), headerSecHeaderLen);
    std::memcpy(toSign + headerSecHeaderLen, plaintext, seqHeaderPlusRespBodyLen + 1 + pad);
    Byte signature[security::MAX_RSA_MODULUS_BYTES];
    if(ourModLen > sizeof(signature)) return false;
    if(!security::RsaSha256Sign(ourKey, std::span<const Byte>(toSign, toSignLen), std::span<Byte>(signature, ourModLen)))
        return false;
    std::memcpy(plaintext + seqHeaderPlusRespBodyLen + 1 + pad, signature, ourModLen);

    std::span<Byte> cipherSpan = w.Reserve(cipherLen);
    if(cipherSpan.size() != cipherLen) return false;
    if(!security::RsaOaepEncryptBlocks(clientKey, std::span<const Byte>(plaintext, plaintextLen), cipherSpan))
        return false;

    return w.Ok();
}

}  // namespace rtos::opcua
