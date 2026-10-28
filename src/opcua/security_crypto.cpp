#include "rtos/opcua/security_crypto.hpp"

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/ssl.h>
#include <wolfssl/wolfcrypt/aes.h>
#include <wolfssl/wolfcrypt/asn_public.h>
#include <wolfssl/wolfcrypt/error-crypt.h>
#include <wolfssl/wolfcrypt/hmac.h>
#include <wolfssl/wolfcrypt/memory.h>
#include <wolfssl/wolfcrypt/random.h>
#include <wolfssl/wolfcrypt/rsa.h>
#include <wolfssl/wolfcrypt/sha.h>
#include <wolfssl/wolfcrypt/sha256.h>

#include <algorithm>
#include <cstring>

namespace rtos::opcua::security {

namespace {

// RFC 3447 9.2 Note 1 -- DER of DigestInfo's AlgorithmIdentifier for SHA-256, prepended to the
// raw 32-byte hash: the input of the PKCS#1 v1.5 signature operation.
constexpr Byte SHA256_DIGESTINFO_PREFIX[] = {
    0x30, 0x31, 0x30, 0x0d, 0x06, 0x09, 0x60, 0x86, 0x48, 0x01, 0x65, 0x03, 0x04, 0x02, 0x01, 0x05,
    0x00, 0x04, 0x20,
};
constexpr size_t DIGESTINFO_SIZE = sizeof(SHA256_DIGESTINFO_PREFIX) + SHA256_DIGEST_SIZE; // 51

// One network thread uses all of this (single-threaded server, see opcua_server.hpp).
WOLFSSL_HEAP_HINT *g_heap = nullptr;
WC_RNG g_rng;
bool g_rngReady = false;
RsaKey g_serverKey;
bool g_serverKeyReady = false;
WOLFSSL_CERT_MANAGER *g_trust = nullptr;
// A peer's key for one operation at a time. An RsaKey is ~4 KB (SP_INT_BITS 2048): static,
// not on the network thread's stack, which also carries wolfCrypt's modular exponentiation.
RsaKey g_peerKey;

bool DigestInfo(std::span<const Byte> message, Byte out[DIGESTINFO_SIZE]) {
    std::memcpy(out, SHA256_DIGESTINFO_PREFIX, sizeof(SHA256_DIGESTINFO_PREFIX));
    return Sha256(message, out + sizeof(SHA256_DIGESTINFO_PREFIX));
}

/// A peer's public key as a wolfCrypt key (g_peerKey) for the duration of one operation.
class PublicKey {
public:
    explicit PublicKey(const RsaPublicKey &k) {
        init_ = wc_InitRsaKey_ex(&g_peerKey, g_heap, INVALID_DEVID) == 0;
        ok_ = init_ && wc_RsaPublicKeyDecodeRaw(k.modulus.data(), (word32)k.modulus.size(), k.exponent.data(),
                                                (word32)k.exponent.size(), &g_peerKey) == 0;
    }
    ~PublicKey() {
        if(init_) wc_FreeRsaKey(&g_peerKey);
    }
    PublicKey(const PublicKey &) = delete;
    PublicKey &operator=(const PublicKey &) = delete;
    bool Ok() const { return ok_; }
    RsaKey *Get() { return &g_peerKey; }

private:
    bool init_ = false;
    bool ok_ = false;
};

// ---- minimal DER reader for the certificate validity (wolfSSL: NO_ASN_TIME) ----------------

struct Der {
    std::span<const Byte> data;
    size_t pos = 0;

    // Next TLV: its tag and content; false if malformed.
    bool Next(Byte &tag, std::span<const Byte> &content) {
        if(pos + 2 > data.size()) return false;
        tag = data[pos++];
        size_t len = data[pos++];
        if(len & 0x80) {
            const size_t bytes = len & 0x7F;
            if(bytes == 0 || bytes > 3 || pos + bytes > data.size()) return false;
            len = 0;
            for(size_t i = 0; i < bytes; i++) len = (len << 8) | data[pos++];
        }
        if(pos + len > data.size()) return false;
        content = data.subspan(pos, len);
        pos += len;
        return true;
    }
};

constexpr int64_t DaysFromCivil(int64_t y, int m, int d) {
    y -= (m <= 2) ? 1 : 0;
    const int64_t era = (y >= 0 ? y : y - 399) / 400;
    const int64_t yoe = y - era * 400;
    const int64_t doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const int64_t doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

// UTCTime (0x17, YYMMDDHHMMSSZ) or GeneralizedTime (0x18, YYYYMMDDHHMMSSZ) -> Unix seconds.
bool ParseTime(Byte tag, std::span<const Byte> t, int64_t &out) {
    const auto digits = [&](size_t at, size_t n, int &v) {
        v = 0;
        for(size_t i = 0; i < n; i++) {
            if(at + i >= t.size() || t[at + i] < '0' || t[at + i] > '9') return false;
            v = v * 10 + (t[at + i] - '0');
        }
        return true;
    };
    int year = 0, mon = 0, day = 0, h = 0, m = 0, s = 0;
    size_t at = 0;
    if(tag == 0x17 && t.size() == 13) {
        if(!digits(0, 2, year)) return false;
        year += year < 50 ? 2000 : 1900; // RFC 5280 4.1.2.5.1
        at = 2;
    } else if(tag == 0x18 && t.size() == 15) {
        if(!digits(0, 4, year)) return false;
        at = 4;
    } else {
        return false;
    }
    if(!digits(at, 2, mon) || !digits(at + 2, 2, day) || !digits(at + 4, 2, h) || !digits(at + 6, 2, m) ||
       !digits(at + 8, 2, s) || t[t.size() - 1] != 'Z')
        return false;
    out = DaysFromCivil(year, mon, day) * 86400 + h * 3600 + m * 60 + s;
    return true;
}

// Certificate -> tbsCertificate -> [0] version, serial, signature, issuer, validity.
bool Validity(std::span<const Byte> certDer, int64_t &notBefore, int64_t &notAfter) {
    Der outer{certDer};
    Byte tag = 0;
    std::span<const Byte> cert, tbs, field;
    if(!outer.Next(tag, cert) || tag != 0x30) return false;
    Der c{cert};
    if(!c.Next(tag, tbs) || tag != 0x30) return false;
    Der t{tbs};
    if(!t.Next(tag, field)) return false;
    if(tag == 0xA0 && !t.Next(tag, field)) return false; // version present: then the serial
    if(!t.Next(tag, field) || tag != 0x30) return false; // signature algorithm
    if(!t.Next(tag, field) || tag != 0x30) return false; // issuer
    std::span<const Byte> validity;
    if(!t.Next(tag, validity) || tag != 0x30) return false;
    Der v{validity};
    std::span<const Byte> time;
    return v.Next(tag, time) && ParseTime(tag, time, notBefore) && v.Next(tag, time) && ParseTime(tag, time, notAfter);
}

} // namespace

bool InitCrypto(std::span<std::byte> memory) {
    if(g_heap != nullptr) return g_rngReady; // once per program: the pool backs keys and the RNG
    if(wolfSSL_Init() != WOLFSSL_SUCCESS) return false;
    if(memory.empty()) {
        // no pool of its own: the existing one (the TLS context's, tls.cpp) serves OPC UA too
        g_heap = static_cast<WOLFSSL_HEAP_HINT *>(wolfSSL_GetGlobalHeapHint());
        if(g_heap == nullptr) return false;
        g_rngReady = wc_InitRng_ex(&g_rng, g_heap, INVALID_DEVID) == 0;
        return g_rngReady;
    }
    // Buckets for what lives here (DRBG, certificate manager and its CA, the decoded client
    // certificate during a check); RSA itself works on the stack (WOLFSSL_SP_NO_MALLOC).
    // wolfSSL's default split is made for TLS sessions and needs ~85 KB.
    static constexpr word32 sizes[] = {64, 128, 256, 512, 1024, 2048, 4096, 8192};
    static constexpr word32 counts[] = {16, 16, 8, 8, 4, 4, 2, 1};
    if(wc_LoadStaticMemory_ex(&g_heap, std::size(sizes), sizes, counts, reinterpret_cast<unsigned char *>(memory.data()),
                              (word32)memory.size(), WOLFMEM_GENERAL, 1) != 0)
        return false;
    // Some wolfCrypt paths allocate without a heap hint (e.g. the seed test of the DRBG);
    // with WOLFSSL_NO_MALLOC those use the global hint. The TLS context sets it when it
    // exists (tls.cpp); without TLS, this pool serves them.
    if(wolfSSL_GetGlobalHeapHint() == nullptr) wolfSSL_SetGlobalHeapHint(g_heap);
    g_rngReady = wc_InitRng_ex(&g_rng, g_heap, INVALID_DEVID) == 0;
    return g_rngReady;
}

bool Sha1(std::span<const Byte> data, Byte out[SHA1_DIGEST_SIZE]) {
    return wc_ShaHash(data.data(), (word32)data.size(), out) == 0;
}

bool Sha256(std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]) {
    return wc_Sha256Hash(data.data(), (word32)data.size(), out) == 0;
}

bool HmacSha256(std::span<const Byte> key, std::span<const Byte> data, Byte out[SHA256_DIGEST_SIZE]) {
    Hmac hmac;
    if(wc_HmacInit(&hmac, g_heap, INVALID_DEVID) != 0) return false;
    const bool ok = wc_HmacSetKey(&hmac, WC_SHA256, key.data(), (word32)key.size()) == 0 &&
                    wc_HmacUpdate(&hmac, data.data(), (word32)data.size()) == 0 && wc_HmacFinal(&hmac, out) == 0;
    wc_HmacFree(&hmac);
    return ok;
}

bool Random(std::span<Byte> out) {
    return g_rngReady && wc_RNG_GenerateBlock(&g_rng, out.data(), (word32)out.size()) == 0;
}

namespace {
bool AesCbc(std::span<const Byte, AES_KEY_SIZE> key, std::span<const Byte, AES_BLOCK_SIZE> iv, std::span<Byte> data,
            bool encrypt) {
    if(data.size() % AES_BLOCK_SIZE != 0) return false;
    Aes aes;
    if(wc_AesInit(&aes, g_heap, INVALID_DEVID) != 0) return false;
    bool ok = wc_AesSetKey(&aes, key.data(), AES_KEY_SIZE, iv.data(), encrypt ? AES_ENCRYPTION : AES_DECRYPTION) == 0;
    if(ok) {
        ok = (encrypt ? wc_AesCbcEncrypt(&aes, data.data(), data.data(), (word32)data.size())
                      : wc_AesCbcDecrypt(&aes, data.data(), data.data(), (word32)data.size())) == 0;
    }
    wc_AesFree(&aes);
    return ok;
}
} // namespace

bool AesCbcEncrypt(std::span<const Byte, AES_KEY_SIZE> key, std::span<const Byte, AES_BLOCK_SIZE> iv, std::span<Byte> data) {
    return AesCbc(key, iv, data, true);
}
bool AesCbcDecrypt(std::span<const Byte, AES_KEY_SIZE> key, std::span<const Byte, AES_BLOCK_SIZE> iv, std::span<Byte> data) {
    return AesCbc(key, iv, data, false);
}

bool ConstantTimeEqual(std::span<const Byte> a, std::span<const Byte> b) {
    if(a.size() != b.size()) return false;
    Byte diff = 0;
    for(size_t i = 0; i < a.size(); i++) diff |= static_cast<Byte>(a[i] ^ b[i]);
    return diff == 0;
}

std::span<const Byte> LeafCertificate(std::span<const Byte> certificates) {
    Der d{certificates};
    Byte tag = 0;
    std::span<const Byte> content;
    if(!d.Next(tag, content) || tag != 0x30) return {};
    return certificates.first(d.pos);
}

bool PSha256(std::span<const Byte> secret, std::span<const Byte> seed, std::span<Byte> out) {
    // Seeds used by this server are always small (nonce concatenations, <= ~128 bytes) --
    // MAX_SEED bounds the scratch buffer without dynamic allocation.
    constexpr size_t MAX_SEED = 256;
    if(seed.size() > MAX_SEED) return false;

    Byte a[SHA256_DIGEST_SIZE];
    if(!HmacSha256(secret, seed, a)) return false; // A(1) = HMAC(secret, seed)

    Byte combined[SHA256_DIGEST_SIZE + MAX_SEED];
    size_t filled = 0;
    while(filled < out.size()) {
        std::memcpy(combined, a, SHA256_DIGEST_SIZE);
        std::memcpy(combined + SHA256_DIGEST_SIZE, seed.data(), seed.size());
        Byte chunk[SHA256_DIGEST_SIZE];
        if(!HmacSha256(secret, std::span<const Byte>(combined, SHA256_DIGEST_SIZE + seed.size()), chunk))
            return false;
        size_t n = std::min(SHA256_DIGEST_SIZE, out.size() - filled);
        std::memcpy(out.data() + filled, chunk, n);
        filled += n;
        if(filled < out.size()) {
            if(!HmacSha256(secret, std::span<const Byte>(a, SHA256_DIGEST_SIZE), a)) return false; // A(i+1)
        }
    }
    return true;
}

bool LoadServerKey(std::span<const Byte> keyDer, RsaPrivateKey &out) {
    if(g_serverKeyReady) {
        wc_FreeRsaKey(&g_serverKey);
        g_serverKeyReady = false;
    }
    if(wc_InitRsaKey_ex(&g_serverKey, g_heap, INVALID_DEVID) != 0) return false;
    word32 idx = 0;
    if(wc_GetPkcs8TraditionalOffset(const_cast<Byte *>(keyDer.data()), &idx, (word32)keyDer.size()) < 0)
        idx = 0; // not PKCS#8: PKCS#1 RSAPrivateKey
    if(wc_RsaPrivateKeyDecode(keyDer.data(), &idx, &g_serverKey, (word32)keyDer.size()) != 0 ||
       wc_RsaEncryptSize(&g_serverKey) != (int)MAX_RSA_MODULUS_BYTES || wc_RsaSetRNG(&g_serverKey, &g_rng) != 0) {
        wc_FreeRsaKey(&g_serverKey);
        return false;
    }
    g_serverKeyReady = true;
    out.modulusSize = MAX_RSA_MODULUS_BYTES;
    return true;
}

bool SetTrustedCa(std::span<const Byte> caDer) {
    if(g_trust == nullptr) {
        g_trust = wolfSSL_CertManagerNew_ex(g_heap);
        if(g_trust == nullptr) return false;
    }
    return wolfSSL_CertManagerLoadCABuffer(g_trust, caDer.data(), (long)caDer.size(), WOLFSSL_FILETYPE_ASN1) ==
           WOLFSSL_SUCCESS;
}

CertificateCheck CheckClientCertificate(std::span<const Byte> certDer, UInt64 unixSecondsNow,
                                        std::span<Byte, MAX_RSA_MODULUS_BYTES> modulus, std::span<Byte, 8> exponent,
                                        RsaPublicKey &out) {
    if(g_trust == nullptr) return CertificateCheck::Untrusted;
    const int verified = wolfSSL_CertManagerVerifyBuffer(g_trust, certDer.data(), (long)certDer.size(),
                                                         WOLFSSL_FILETYPE_ASN1);
    if(verified != WOLFSSL_SUCCESS) {
        return verified == ASN_NO_SIGNER_E || verified == ASN_SIG_CONFIRM_E || verified == ASN_SIG_OID_E ||
                       verified == ASN_SIG_KEY_E || verified == ASN_SIG_HASH_E
                   ? CertificateCheck::Untrusted
                   : CertificateCheck::Invalid;
    }
    int64_t notBefore = 0, notAfter = 0;
    if(!Validity(certDer, notBefore, notAfter)) return CertificateCheck::Invalid;
    if(unixSecondsNow != 0 && ((int64_t)unixSecondsNow < notBefore || (int64_t)unixSecondsNow > notAfter))
        return CertificateCheck::TimeInvalid;

    Byte spki[512];
    word32 spkiSize = sizeof(spki);
    if(wc_GetSubjectPubKeyInfoDerFromCert(certDer.data(), (word32)certDer.size(), spki, &spkiSize) != 0)
        return CertificateCheck::Invalid;
    if(wc_InitRsaKey_ex(&g_peerKey, g_heap, INVALID_DEVID) != 0) return CertificateCheck::Invalid;
    word32 idx = 0;
    word32 nSize = (word32)modulus.size(), eSize = (word32)exponent.size();
    const bool rsa = wc_RsaPublicKeyDecode(spki, &idx, &g_peerKey, spkiSize) == 0 &&
                     wc_RsaFlattenPublicKey(&g_peerKey, exponent.data(), &eSize, modulus.data(), &nSize) == 0;
    wc_FreeRsaKey(&g_peerKey);
    if(!rsa || nSize != MAX_RSA_MODULUS_BYTES) return CertificateCheck::NotRsa2048;
    out.modulus = modulus.first(nSize);
    out.exponent = exponent.first(eSize);
    return CertificateCheck::Ok;
}

bool RsaSha256Sign(const RsaPrivateKey &key, std::span<const Byte> message, std::span<Byte> signatureOut) {
    if(!g_serverKeyReady || signatureOut.size() != key.modulusSize) return false;
    Byte info[DIGESTINFO_SIZE];
    if(!DigestInfo(message, info)) return false;
    return wc_RsaSSL_Sign(info, DIGESTINFO_SIZE, signatureOut.data(), (word32)signatureOut.size(), &g_serverKey,
                          &g_rng) == (int)signatureOut.size();
}

bool RsaSha256Verify(const RsaPublicKey &key, std::span<const Byte> message, std::span<const Byte> signature) {
    if(signature.size() != key.modulus.size() || signature.size() > MAX_RSA_MODULUS_BYTES) return false;
    Byte expected[DIGESTINFO_SIZE];
    if(!DigestInfo(message, expected)) return false;
    PublicKey pub(key);
    if(!pub.Ok()) return false;
    Byte decoded[MAX_RSA_MODULUS_BYTES];
    const int n = wc_RsaSSL_Verify(signature.data(), (word32)signature.size(), decoded, sizeof(decoded), pub.Get());
    return n == (int)DIGESTINFO_SIZE && std::memcmp(decoded, expected, DIGESTINFO_SIZE) == 0;
}

bool RsaOaepEncryptBlocks(const RsaPublicKey &key, std::span<const Byte> plaintext, std::span<Byte> cipherOut) {
    const size_t k = key.modulus.size();
    const size_t blockPlain = RsaOaepMaxPlaintext(k);
    if(k > MAX_RSA_MODULUS_BYTES || k < OAEP_SHA1_OVERHEAD_BYTES || plaintext.size() % blockPlain != 0) return false;
    const size_t blocks = plaintext.size() / blockPlain;
    if(cipherOut.size() != blocks * k) return false;
    PublicKey pub(key);
    if(!pub.Ok()) return false;
    for(size_t i = 0; i < blocks; i++) {
        const int n = wc_RsaPublicEncrypt_ex(plaintext.data() + i * blockPlain, (word32)blockPlain,
                                             cipherOut.data() + i * k, (word32)k, pub.Get(), &g_rng, WC_RSA_OAEP_PAD,
                                             WC_HASH_TYPE_SHA, WC_MGF1SHA1, nullptr, 0);
        if(n != (int)k) return false;
    }
    return true;
}

bool RsaOaepDecryptBlocks(const RsaPrivateKey &key, std::span<const Byte> ciphertext, std::span<Byte> plaintextOut) {
    const size_t k = key.modulusSize;
    const size_t blockPlain = RsaOaepMaxPlaintext(k);
    if(!g_serverKeyReady || k == 0 || ciphertext.empty() || ciphertext.size() % k != 0) return false;
    const size_t blocks = ciphertext.size() / k;
    if(plaintextOut.size() != blocks * blockPlain) return false;
    // With WOLFSSL_NO_MALLOC wolfCrypt decrypts in place in `out`: the output buffer must hold
    // the whole modulus, not just the plaintext - decrypting straight into plaintextOut would
    // overrun it by 2*hLen+2 bytes at the last block.
    Byte block[MAX_RSA_MODULUS_BYTES];
    if(k > sizeof block) return false;
    bool ok = true;
    for(size_t i = 0; i < blocks && ok; i++) {
        // A single generic failure path (RFC 3447 7.1.2 warns against revealing which check
        // failed: padding oracle); wolfCrypt's OAEP decoding is constant time.
        const int n = wc_RsaPrivateDecrypt_ex(ciphertext.data() + i * k, (word32)k, block, (word32)k, &g_serverKey,
                                              WC_RSA_OAEP_PAD, WC_HASH_TYPE_SHA, WC_MGF1SHA1, nullptr, 0);
        ok = n == (int)blockPlain; // not a partial-block format, see the header
        if(ok) std::memcpy(plaintextOut.data() + i * blockPlain, block, blockPlain);
    }
    wc_ForceZero(block, sizeof block);
    return ok;
}

}  // namespace rtos::opcua::security
