#pragma once
// UA TCP transport messages (Part 6 Section 7): the 8-byte header common to every message
// (Hello/Acknowledge/Error/OpenSecureChannel/Message/CloseSecureChannel all start with it), and
// the encode/decode for the three unsecured ones (Hello/Acknowledge/Error). OPN/MSG/CLO bodies
// are handled in secure_channel.hpp/.cpp since their header additionally carries the
// (a/symmetric) security header this server needs to interpret.
#include <span>
#include <string_view>

#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

constexpr size_t TCP_HEADER_SIZE = 8;

enum class ChunkType : Byte { Final = 'F', Intermediate = 'C', Abort = 'A' };

struct TcpHeader {
    char messageType[3] = {0, 0, 0}; // "HEL"/"ACK"/"ERR"/"OPN"/"MSG"/"CLO", NOT NUL-terminated
    ChunkType chunkType = ChunkType::Final;
    UInt32 messageSize = 0; // total size including this 8-byte header

    bool Is(const char *type) const {
        return messageType[0] == type[0] && messageType[1] == type[1] && messageType[2] == type[2];
    }
};

// Peeks (does not consume) the 8-byte header from the front of a byte range -- used by the
// transport layer to learn MessageSize before a complete message has necessarily arrived yet.
bool PeekTcpHeader(std::span<const Byte> data, TcpHeader &out);

bool EncodeTcpHeader(ByteWriter &w, const char messageType[3], ChunkType chunkType, UInt32 messageSize);
bool DecodeTcpHeader(ByteReader &r, TcpHeader &out);

constexpr UInt32 PROTOCOL_VERSION = 0;

struct HelloMessage {
    UInt32 protocolVersion = 0;
    UInt32 receiveBufferSize = 0;
    UInt32 sendBufferSize = 0;
    UInt32 maxMessageSize = 0;
    UInt32 maxChunkCount = 0;
    String endpointUrl;
};
// Body only (the 8-byte TcpHeader has already been consumed by the caller).
bool DecodeHelloBody(ByteReader &r, HelloMessage &out);

struct AcknowledgeMessage {
    UInt32 protocolVersion = PROTOCOL_VERSION;
    UInt32 receiveBufferSize = 0;
    UInt32 sendBufferSize = 0;
    UInt32 maxMessageSize = 0;
    UInt32 maxChunkCount = 0;
};
// Encodes the full message (header + body) into w.
bool EncodeAcknowledge(ByteWriter &w, const AcknowledgeMessage &msg);

// Encodes a full "ERR" message (header + body) -- the last thing sent before the connection is
// closed, per Part 6 7.1.4.
bool EncodeErrorMessage(ByteWriter &w, StatusCode error, std::string_view reason);

}  // namespace rtos::opcua
