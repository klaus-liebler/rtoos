#pragma once
// Cryptographic primitives for SecurityPolicy#Basic256Sha256 (Part 6 6.7.3/6.7.5): RSASSA-
// PKCS1-v1_5 signing/verification with SHA-256, RSAES-OAEP with SHA-1, HMAC-SHA256, and the
// P_SHA256 key-derivation function (the same HMAC-SHA256-based PRF construction as TLS's PRF,
// RFC 5246 5.). rtoos: built on wolfCrypt (formerly NetX Crypto), with its own static memory
// pool (InitCrypto) - no heap.
//
// Client certificates are verified against the trusted CA (SetTrustedCa): signature chain via
// wolfSSL's certificate manager, validity period against the server clock (wolfSSL is built
// without a clock, NO_ASN_TIME, so the dates are checked here once the clock is synchronised).
//
// Fixed to RSA-2048 (MAX_RSA_MODULUS_BYTES): this server's key and every client key.
#include <cstddef>
#include <span>

#include "rtos/opcua/types.hpp"

namespace rtos::opcua::security {

constexpr size_t SHA256_DIGEST_SIZE = 32;
constexpr size_t SHA1_DIGEST_SIZE = 20;
constexpr size_t MAX_RSA_MODULUS_BYTES = 256; // RSA-2048

/// `memory` (16-byte aligned, e.g. 24 KB) becomes the static pool for the random number
/// generator, the certificate manager and the client certificate checks. Empty: the existing
/// wolfSSL pool (the global heap hint, e.g. the TLS context's) is shared. Only the first call
/// sets it up; later calls report whether that worked.
bool InitCrypto(std::span<std::byte> memory);

bool Sha256(std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]);
// OAEP's MGF1 and the CertificateThumbprint field (Part 6 7.10, a SHA-1 digest of the peer's
// DER certificate).
bool Sha1(std::span<const Byte> data, Byte out[SHA1_DIGEST_SIZE]);
bool HmacSha256(std::span<const Byte> key, std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]);
bool Random(std::span<Byte> out);
/// Equal contents, in time independent of where they differ (signature checks).
bool ConstantTimeEqual(std::span<const Byte> a, std::span<const Byte> b);
/// The first DER element of `certificates` - the leaf of a chain sent as one ByteString
/// (Part 6 6.2.3: leaf followed by its issuers); empty if malformed.
std::span<const Byte> LeafCertificate(std::span<const Byte> certificates);

// AES-256-CBC (Basic256Sha256's SymmetricEncryptionAlgorithm, SignAndEncrypt): `data` (a
// multiple of 16 bytes) in place; the IV is the channel's derived one for every message (Part 6
// 6.7.5 -- the sequence header in the first block makes each message's first block unique).
constexpr size_t AES_KEY_SIZE = 32;
constexpr size_t AES_BLOCK_SIZE = 16;
bool AesCbcEncrypt(std::span<const Byte, AES_KEY_SIZE> key, std::span<const Byte, AES_BLOCK_SIZE> iv, std::span<Byte> data);
bool AesCbcDecrypt(std::span<const Byte, AES_KEY_SIZE> key, std::span<const Byte, AES_BLOCK_SIZE> iv, std::span<Byte> data);

// RFC 5246 5. / OPC UA Part 6 6.7.5 -- expands (secret, seed) into exactly out.size() bytes.
bool PSha256(std::span<const Byte> secret, std::span<const Byte> seed, std::span<Byte> out);

// Public key of a peer (big-endian modulus and exponent), e.g. from its certificate.
struct RsaPublicKey {
    std::span<const Byte> modulus;
    std::span<const Byte> exponent;
};

// This server's private key, loaded once (LoadServerKey). Opaque: the wolfCrypt key lives in
// security_crypto.cpp.
struct RsaPrivateKey {
    size_t modulusSize = 0;
};

/// PKCS#1 or PKCS#8 DER RSA-2048 private key of this server.
bool LoadServerKey(std::span<const Byte> keyDer, RsaPrivateKey &out);

/// CA certificate (DER) that client certificates must chain to.
bool SetTrustedCa(std::span<const Byte> caDer);

/// Outcome of CheckClientCertificate (mapped onto the Bad_Certificate* status codes).
enum class CertificateCheck : Byte { Ok, Invalid, Untrusted, TimeInvalid, NotRsa2048 };

/// Verify `certDer` against the trusted CA and its validity period (`now`, 0: clock not
/// synchronised - dates not checked), and extract its RSA-2048 public key into `modulus`/
/// `exponent` (storage of the caller) and `out`.
CertificateCheck CheckClientCertificate(std::span<const Byte> certDer, UInt64 unixSecondsNow,
                                        std::span<Byte, MAX_RSA_MODULUS_BYTES> modulus,
                                        std::span<Byte, 8> exponent, RsaPublicKey &out);

// RSASSA-PKCS1-v1_5 with SHA-256 ("http://www.w3.org/2001/04/xmldsig-more#rsa-sha256", used by
// Basic256Sha256 for the asymmetric OPN signature and CreateSession's ServerSignature proof of
// possession). "signatureOut" must be exactly the key's modulus size.
bool RsaSha256Sign(const RsaPrivateKey &key, std::span<const Byte> message, std::span<Byte> signatureOut);
bool RsaSha256Verify(const RsaPublicKey &key, std::span<const Byte> message, std::span<const Byte> signature);

// RSAES-OAEP (RFC 3447 7.1) with SHA-1 for the label hash and MGF1 -- Basic256Sha256's
// AsymmetricEncryptionAlgorithm (http://www.w3.org/2001/04/xmlenc#rsa-oaep), empty label. Per
// Part 6 6.7.4, OpenSecureChannel messages are ALWAYS both signed and encrypted with this
// whenever SecurityPolicy != None -- independent of the negotiated MessageSecurityMode.
constexpr size_t OAEP_SHA1_OVERHEAD_BYTES = 2 * 20 + 2; // 2*hLen + 2
constexpr size_t RsaOaepMaxPlaintext(size_t modulusLen) { return modulusLen - OAEP_SHA1_OVERHEAD_BYTES; }

// Multi-block form (Part 6 6.7.2): the plaintext is padded by the CALLER to an exact multiple
// of RsaOaepMaxPlaintext(modulus) ("PlainTextBlockSize"); each block is encrypted on its own and
// the ciphertext blocks are concatenated -- "cipherOut" is exactly (plaintext.size() /
// RsaOaepMaxPlaintext(...)) * modulus bytes.
bool RsaOaepEncryptBlocks(const RsaPublicKey &key, std::span<const Byte> plaintext, std::span<Byte> cipherOut);
// Inverse: every block must decode to EXACTLY RsaOaepMaxPlaintext(modulus) bytes (a peer that
// built its blocks correctly always produces that); "plaintextOut" is exactly
// (ciphertext.size() / modulus) * RsaOaepMaxPlaintext(...) bytes.
bool RsaOaepDecryptBlocks(const RsaPrivateKey &key, std::span<const Byte> ciphertext, std::span<Byte> plaintextOut);

}  // namespace rtos::opcua::security
