#include "rtos/opcua/tcp_message.hpp"

namespace rtos::opcua {

bool PeekTcpHeader(std::span<const Byte> data, TcpHeader &out) {
    ByteReader r(data);
    return DecodeTcpHeader(r, out);
}

bool EncodeTcpHeader(ByteWriter &w, const char messageType[3], ChunkType chunkType, UInt32 messageSize) {
    if(!w.WriteRaw(messageType, 3)) return false;
    if(!w.WriteByte(static_cast<Byte>(chunkType))) return false;
    return w.WriteUInt32(messageSize);
}

bool DecodeTcpHeader(ByteReader &r, TcpHeader &out) {
    Byte typeBytes[3];
    if(!r.ReadRaw(typeBytes, 3)) return false;
    out.messageType[0] = static_cast<char>(typeBytes[0]);
    out.messageType[1] = static_cast<char>(typeBytes[1]);
    out.messageType[2] = static_cast<char>(typeBytes[2]);
    Byte chunk = 0;
    if(!r.ReadByte(chunk)) return false;
    out.chunkType = static_cast<ChunkType>(chunk);
    return r.ReadUInt32(out.messageSize);
}

bool DecodeHelloBody(ByteReader &r, HelloMessage &out) {
    if(!r.ReadUInt32(out.protocolVersion)) return false;
    if(!r.ReadUInt32(out.receiveBufferSize)) return false;
    if(!r.ReadUInt32(out.sendBufferSize)) return false;
    if(!r.ReadUInt32(out.maxMessageSize)) return false;
    if(!r.ReadUInt32(out.maxChunkCount)) return false;
    return r.ReadString(out.endpointUrl);
}

bool EncodeAcknowledge(ByteWriter &w, const AcknowledgeMessage &msg) {
    // Body is fixed-size (5x UInt32 = 20 bytes), so the final MessageSize is known up front --
    // no need for the "reserve header, backfill size" pattern EncodeErrorMessage below uses.
    constexpr UInt32 size = TCP_HEADER_SIZE + 20;
    if(!EncodeTcpHeader(w, "ACK", ChunkType::Final, size)) return false;
    if(!w.WriteUInt32(msg.protocolVersion)) return false;
    if(!w.WriteUInt32(msg.receiveBufferSize)) return false;
    if(!w.WriteUInt32(msg.sendBufferSize)) return false;
    if(!w.WriteUInt32(msg.maxMessageSize)) return false;
    return w.WriteUInt32(msg.maxChunkCount);
}

bool EncodeErrorMessage(ByteWriter &w, StatusCode error, std::string_view reason) {
    size_t startPos = w.Position();
    // MessageSize isn't known until the (variable-length) Reason string is encoded -- write a
    // placeholder header now (size field is bytes [4,8) of it), backfill the real size once
    // the body is done via ByteWriter::PatchUInt32 (the same pattern secure_channel.cpp uses
    // for OPN/MSG/CLO framing).
    if(!EncodeTcpHeader(w, "ERR", ChunkType::Final, 0)) return false;
    if(!w.WriteStatusCode(error)) return false;
    if(!w.WriteString(reason)) return false;
    if(!w.Ok()) return false;

    UInt32 totalSize = static_cast<UInt32>(w.Position() - startPos);
    return w.PatchUInt32(startPos + 4, totalSize);
}

}  // namespace rtos::opcua
