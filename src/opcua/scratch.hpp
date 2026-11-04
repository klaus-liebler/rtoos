// SPDX-License-Identifier: MIT
//
// One static work area for the OPC UA steps of the network thread that never overlap, instead
// of ~11 KB of stack: the network thread runs every OPC UA connection, one message at a time,
// and an OPN (HandleOpen) runs to its end in one background step.
//   open        HandleOpen (OPN): the asymmetric request and response
//   signInput   ActivateSession's ClientSignature check (inside a MSG; the client key it uses
//               is a copy in the SecureChannel, not the one in `open`)
// A MSG/CLO is decrypted in place in the connection's receive buffer (VerifyAndUnwrapSymmetric),
// not here.
#pragma once

#include <cstddef>

#include "rtos/opcua/security_crypto.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua::scratch {

constexpr std::size_t MAX_CERT_DER = 2000;
constexpr std::size_t MAX_PLAIN = 1536;
constexpr std::size_t MAX_SIGNED = MAX_CERT_DER + MAX_PLAIN + 64;

struct Open {
    Byte plainBuf[MAX_PLAIN];
    Byte signedBuf[MAX_SIGNED];
    Byte plaintext[MAX_PLAIN];
    Byte toSign[MAX_SIGNED];
    Byte clientModulus[security::MAX_RSA_MODULUS_BYTES];
    Byte clientExponent[8];
};

union Area {
    Open open;
    Byte signInput[2600];
};

extern Area area;  // secure_channel.cpp

}  // namespace rtos::opcua::scratch
