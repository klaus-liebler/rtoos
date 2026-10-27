#include "rtos/opcua/session.hpp"

#include <cmath>
#include <cstring>

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/secure_channel.hpp"
#include "rtos/opcua/service_header.hpp"

namespace rtos::opcua {

namespace {
// AnonymousIdentityToken's DefaultBinary encoding id (Part 4 7.36.4 / Opc.Ua.NodeIds.csv).
constexpr UInt32 ANONYMOUS_IDENTITY_TOKEN_TYPEID = 321;
constexpr std::string_view ANONYMOUS_POLICY_ID = "anonymous";
// Basic256Sha256's AsymmetricSignatureAlgorithm (Part 7) -- the 2001/04 URI every real stack
// uses (UAExpert rejects the 2000/09 one of the original spec text).
constexpr std::string_view RSA_SHA256_URI = "http://www.w3.org/2001/04/xmldsig-more#rsa-sha256";

struct SignatureData {
    String algorithm;
    std::string_view signature;
    bool signatureIsNull = true;
};
bool DecodeSignatureData(ByteReader &r, SignatureData &out) {
    return r.ReadString(out.algorithm) && r.ReadByteString(out.signature, out.signatureIsNull);
}

// ClientSoftwareCertificates (Part 4 7.32): array of {CertificateData: ByteString,
// SignatureData}. Real anonymous-auth clients send an empty array; skipped either way since
// this server has no software certificate policy.
bool SkipSoftwareCertificateArray(ByteReader &r) {
    Int32 count = 0;
    if(!r.ReadInt32(count)) return false;
    for(Int32 i = 0; i < count; i++) {
        std::string_view cert;
        bool certIsNull = true;
        SignatureData signature;
        if(!r.ReadByteString(cert, certIsNull)) return false;
        if(!DecodeSignatureData(r, signature)) return false;
    }
    return true;
}

std::span<const Byte> Bytes(std::string_view s) { return {reinterpret_cast<const Byte *>(s.data()), s.size()}; }

// ClientSignature check of ActivateSession: RSA-SHA256 over serverCertificate || serverNonce
// with the channel's client key. Part 4 5.6.3 names the certificate of CreateSessionResponse
// (here: leaf + CA); clients differ in whether they sign the chain or only the leaf (asyncua
// uses the leaf from the endpoint), so both are accepted -- either way the client proved it
// holds the private key of the certificate the channel was opened with.
// Static scratch: the network thread serves one message at a time.
Byte g_signInput[2600];
bool ClientSignatureValid(const SecureChannel &channel, const SignatureData &sig, std::span<const Byte> serverNonce) {
    if(sig.algorithm.isNull || sig.algorithm.value != RSA_SHA256_URI || sig.signatureIsNull) return false;
    const auto &id = GetServerIdentity();
    const auto verify = [&](bool withChain) {
        const size_t certLen = id.certificateDer.size() + (withChain ? id.caCertificateDer.size() : 0);
        if(certLen + serverNonce.size() > sizeof g_signInput) return false;
        std::memcpy(g_signInput, id.certificateDer.data(), id.certificateDer.size());
        if(withChain) std::memcpy(g_signInput + id.certificateDer.size(), id.caCertificateDer.data(), id.caCertificateDer.size());
        std::memcpy(g_signInput + certLen, serverNonce.data(), serverNonce.size());
        return security::RsaSha256Verify(channel.ClientKey(), std::span<const Byte>(g_signInput, certLen + serverNonce.size()),
                                         Bytes(sig.signature));
    };
    return verify(false) || verify(true);
}

// UserIdentityToken (ExtensionObject): Anonymous with the offered PolicyId, or null.
StatusCode CheckIdentityToken(ByteReader &r) {
    NodeId tokenTypeId;
    Byte encoding = 0;
    if(!DecodeNodeId(r, tokenTypeId) || !r.ReadByte(encoding)) return StatusCode::BadDecodingError;
    std::string_view body;
    bool bodyIsNull = true;
    if(encoding & 0x01) {
        if(!r.ReadByteString(body, bodyIsNull)) return StatusCode::BadDecodingError;
    } else if(encoding & 0x02) {
        String xml;
        if(!r.ReadString(xml)) return StatusCode::BadDecodingError;
        return StatusCode::BadIdentityTokenInvalid; // XML-encoded tokens: not supported
    }
    if(tokenTypeId.IsNull()) return StatusCode::Good; // some clients omit the token entirely
    if(tokenTypeId != NodeId(0, ANONYMOUS_IDENTITY_TOKEN_TYPEID)) return StatusCode::BadIdentityTokenInvalid;
    ByteReader token(Bytes(body));
    String policyId;
    if(!token.ReadString(policyId) || policyId.isNull || policyId.value != ANONYMOUS_POLICY_ID)
        return StatusCode::BadIdentityTokenInvalid;
    return StatusCode::Good;
}
} // namespace

StatusCode Session::Check(const NodeId &authenticationToken) {
    Expire();
    if(!isCreated_ || authenticationToken != authenticationToken_) return StatusCode::BadSessionIdInvalid;
    lastActivity_ = NowMonotonic();
    return isActive_ ? StatusCode::Good : StatusCode::BadSessionNotActivated;
}

void Session::Expire() {
    if(isCreated_ && static_cast<Double>(NowMonotonic() - lastActivity_) / (DATETIME_SEC / 1000) > timeoutMs_)
        Close();
}

bool Session::HandleCreate(const RequestHeader &header, ByteReader &requestBody, UInt32 connectionSlot,
                           std::string_view endpointUrl, const SecureChannel &channel, ByteWriter &w) {
    ApplicationDescription clientDescription;
    if(!DecodeApplicationDescription(requestBody, clientDescription)) return false;

    String serverUri, requestedEndpointUrl, sessionName;
    std::string_view clientNonce, clientCertificate;
    bool clientNonceIsNull = true, clientCertificateIsNull = true;
    Double requestedSessionTimeout = 0;
    UInt32 maxResponseMessageSize = 0;
    if(!requestBody.ReadString(serverUri)) return false;
    if(!requestBody.ReadString(requestedEndpointUrl)) return false;
    if(!requestBody.ReadString(sessionName)) return false;
    if(!requestBody.ReadByteString(clientNonce, clientNonceIsNull)) return false;
    if(!requestBody.ReadByteString(clientCertificate, clientCertificateIsNull)) return false;
    if(!requestBody.ReadDouble(requestedSessionTimeout)) return false;
    if(!requestBody.ReadUInt32(maxResponseMessageSize)) return false;
    if(!requestBody.Ok()) return false;
    (void)maxResponseMessageSize;

    // On a secured channel the session belongs to the channel's client (Part 4 5.6.2.2): the
    // same application certificate (leaf compared by thumbprint), and a nonce of >= 32 bytes.
    // On SecurityPolicy None both are ignored.
    const bool secured = !channel.IsUnsecured();
    if(secured) {
        if(clientNonceIsNull || clientNonce.size() < NONCE_SIZE) return EncodeServiceFault(w, header, StatusCode::BadNonceInvalid);
        const auto leaf = security::LeafCertificate(Bytes(clientCertificate));
        Byte thumbprint[security::SHA1_DIGEST_SIZE];
        if(clientCertificateIsNull || leaf.empty() || !security::Sha1(leaf, thumbprint) ||
           !security::ConstantTimeEqual(thumbprint, channel.ClientThumbprint()))
            return EncodeServiceFault(w, header, StatusCode::BadCertificateInvalid);
    }

    // One session per connection: a new CreateSession replaces the previous one, with a new
    // token (requests carrying the old one are answered with BadSessionIdInvalid).
    ++tokenCounter_;
    sessionId_ = NodeId(1, 10000 + connectionSlot);
    authenticationToken_ = NodeId(1, 0x0100'0000u * (connectionSlot + 1) + tokenCounter_);
    isCreated_ = true;
    isActive_ = false;
    lastActivity_ = NowMonotonic();
    timeoutMs_ = (std::isnan(requestedSessionTimeout) || requestedSessionTimeout <= 0)
                     ? DEFAULT_TIMEOUT_MS
                     : std::clamp(requestedSessionTimeout, MIN_TIMEOUT_MS, MAX_TIMEOUT_MS);
    if(!security::Random(serverNonce_)) return false;

    // Echo back whatever URL the client itself specified (rather than this server's own fixed,
    // hostless default) -- see HandleGetEndpoints (services.cpp) for why: strict clients
    // (UAExpert's .NET-based stack included) reject the session with BadInvalidArgument if the
    // returned EndpointUrl doesn't match how they connected.
    std::string_view effectiveEndpointUrl = (!requestedEndpointUrl.isNull && !requestedEndpointUrl.value.empty())
        ? requestedEndpointUrl.value : endpointUrl;

    // ServerSignature (Part 4 5.6.2.2): proof the server holds the private key matching
    // serverCertificate -- sign(serverPrivateKey, clientCertificate || clientNonce).
    constexpr size_t MAX_SIGN_INPUT = 2200;
    Byte serverSignature[security::MAX_RSA_MODULUS_BYTES];
    size_t serverSignatureLen = 0;
    if(secured && clientCertificate.size() + clientNonce.size() <= MAX_SIGN_INPUT) {
        std::memcpy(g_signInput, clientCertificate.data(), clientCertificate.size());
        std::memcpy(g_signInput + clientCertificate.size(), clientNonce.data(), clientNonce.size());
        const auto &ourKey = GetServerIdentity().privateKey;
        if(ourKey.modulusSize <= sizeof(serverSignature) &&
           security::RsaSha256Sign(ourKey,
                                   std::span<const Byte>(g_signInput, clientCertificate.size() + clientNonce.size()),
                                   std::span<Byte>(serverSignature, ourKey.modulusSize)))
            serverSignatureLen = ourKey.modulusSize;
    }

    if(!EncodeNodeId(w, NodeId(0, ns0::CreateSessionResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!EncodeNodeId(w, sessionId_)) return false;
    if(!EncodeNodeId(w, authenticationToken_)) return false;
    if(!w.WriteDouble(timeoutMs_)) return false;
    if(!w.WriteByteString(std::string_view(reinterpret_cast<const char *>(serverNonce_), NONCE_SIZE), false)) return false;
    // serverCertificate: leaf + CA on a secured channel, null on SecurityPolicy None
    if(secured ? !WriteServerCertificateChain(w) : !w.WriteByteString(std::string_view{}, true)) return false;
    if(!EncodeEndpoints(w, effectiveEndpointUrl)) return false; // ServerEndpoints
    if(!w.WriteInt32(-1)) return false;                        // ServerSoftwareCertificates: null
    if(serverSignatureLen > 0) {
        if(!w.WriteString(RSA_SHA256_URI)) return false;
        if(!w.WriteByteString(std::string_view((const char *)serverSignature, serverSignatureLen), false)) return false;
    } else {
        if(!w.WriteString(String::Null())) return false;
        if(!w.WriteByteString(std::string_view{}, true)) return false;
    }
    return w.WriteUInt32(8192); // MaxRequestMessageSize
}

bool Session::HandleActivate(const RequestHeader &header, ByteReader &requestBody, const SecureChannel &channel,
                             ByteWriter &w) {
    SignatureData clientSignature;
    if(!DecodeSignatureData(requestBody, clientSignature)) return false;
    if(!SkipSoftwareCertificateArray(requestBody)) return false; // ClientSoftwareCertificates
    if(!SkipStringArray(requestBody)) return false;              // LocaleIds
    const StatusCode token = CheckIdentityToken(requestBody);
    if(token == StatusCode::BadDecodingError) return false;
    SignatureData userTokenSignature;
    if(!DecodeSignatureData(requestBody, userTokenSignature)) return false;
    if(!requestBody.Ok()) return false;

    Expire();
    StatusCode result = StatusCode::Good;
    if(!isCreated_ || header.authenticationToken != authenticationToken_)
        result = StatusCode::BadSessionIdInvalid;
    else if(!channel.IsUnsecured() && !ClientSignatureValid(channel, clientSignature, serverNonce_))
        result = StatusCode::BadApplicationSignatureInvalid;
    else
        result = token;
    if(!IsGood(result)) return EncodeServiceFault(w, header, result);

    isActive_ = true;
    lastActivity_ = NowMonotonic();
    if(!security::Random(serverNonce_)) return false; // a fresh nonce for the next activation

    if(!EncodeNodeId(w, NodeId(0, ns0::ActivateSessionResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!w.WriteByteString(std::string_view(reinterpret_cast<const char *>(serverNonce_), NONCE_SIZE), false)) return false;
    if(!w.WriteInt32(-1)) return false; // Results: null array
    return w.WriteInt32(-1);            // DiagnosticInfos: null array
}

bool Session::HandleClose(const RequestHeader &header, ByteReader &requestBody, ByteWriter &w) {
    Boolean deleteSubscriptions = false;
    if(!requestBody.ReadBoolean(deleteSubscriptions)) return false;
    if(!requestBody.Ok()) return false;
    (void)deleteSubscriptions; // subscriptions belong to the session and end with it

    Expire();
    if(!isCreated_ || header.authenticationToken != authenticationToken_)
        return EncodeServiceFault(w, header, StatusCode::BadSessionIdInvalid);
    Close();
    if(!EncodeNodeId(w, NodeId(0, ns0::CloseSessionResponse))) return false;
    return EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good));
}

}  // namespace rtos::opcua
