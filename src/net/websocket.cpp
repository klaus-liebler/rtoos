// SPDX-License-Identifier: MIT
#include "rtos/net/websocket.hpp"

#include <wolfssl/wolfcrypt/settings.h>
#include <wolfssl/wolfcrypt/coding.h>
#include <wolfssl/wolfcrypt/hash.h>
#include <wolfssl/wolfcrypt/sha.h>

#include <cstring>

namespace rtos::net {

bool websocket_accept_key(std::string_view key, std::array<char, 29>& out) noexcept {
    // RFC 6455 4.2.1: the key is the base64 encoding of 16 bytes (24 characters).
    constexpr std::string_view guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    if (key.size() != 24) {
        return false;
    }
    std::array<unsigned char, 24 + guid.size()> input{};
    std::memcpy(input.data(), key.data(), key.size());
    std::memcpy(input.data() + key.size(), guid.data(), guid.size());

    // SHA-1 context on the stack (wc_ShaHash would allocate it with WOLFSSL_SMALL_STACK).
    std::array<unsigned char, WC_SHA_DIGEST_SIZE> digest{};
    wc_Sha sha;
    if (wc_InitSha_ex(&sha, nullptr, INVALID_DEVID) != 0) {
        return false;
    }
    const bool hashed = wc_ShaUpdate(&sha, input.data(), static_cast<word32>(input.size())) == 0 &&
                        wc_ShaFinal(&sha, digest.data()) == 0;
    wc_ShaFree(&sha);
    if (!hashed) {
        return false;
    }
    word32 length = static_cast<word32>(out.size());
    if (Base64_Encode_NoNl(digest.data(), static_cast<word32>(digest.size()),
                           reinterpret_cast<unsigned char*>(out.data()), &length) != 0 ||
        length != 28) {
        return false;
    }
    out[28] = '\0';
    return true;
}

}  // namespace rtos::net
