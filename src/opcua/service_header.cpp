#include "rtos/opcua/service_header.hpp"

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"

namespace rtos::opcua {

bool EncodeNullExtensionObject(ByteWriter &w) {
    if(!EncodeNodeId(w, NodeId(0, 0))) return false;
    return w.WriteByte(0x00); // encoding mask: no body
}

bool DecodeAndDiscardExtensionObject(ByteReader &r) {
    NodeId typeId;
    if(!DecodeNodeId(r, typeId)) return false;
    Byte encoding = 0;
    if(!r.ReadByte(encoding)) return false;
    if(encoding & 0x01) {
        // Binary body: ByteString-shaped (length-prefixed raw bytes).
        std::string_view discard;
        bool isNull = false;
        return r.ReadByteString(discard, isNull);
    }
    if(encoding & 0x02) {
        // XML body: String-shaped.
        String discard;
        return r.ReadString(discard);
    }
    return true; // encoding == 0: no body to skip
}

bool DecodeRequestHeader(ByteReader &r, RequestHeader &out) {
    if(!DecodeNodeId(r, out.authenticationToken)) return false;
    if(!r.ReadDateTime(out.timestamp)) return false;
    if(!r.ReadUInt32(out.requestHandle)) return false;
    if(!r.ReadUInt32(out.returnDiagnostics)) return false;
    if(!r.ReadString(out.auditEntryId)) return false;
    if(!r.ReadUInt32(out.timeoutHint)) return false;
    return DecodeAndDiscardExtensionObject(r);
}

bool EncodeResponseHeader(ByteWriter &w, const ResponseHeader &h) {
    if(!w.WriteDateTime(h.timestamp)) return false;
    if(!w.WriteUInt32(h.requestHandle)) return false;
    if(!w.WriteStatusCode(h.serviceResult)) return false;
    if(!w.WriteByte(0x00)) return false;      // serviceDiagnostics: DiagnosticInfo, no fields present
    if(!w.WriteInt32(-1)) return false;       // stringTable: null array
    return EncodeNullExtensionObject(w);      // additionalHeader
}

ResponseHeader MakeResponseHeader(const RequestHeader &request, StatusCode result) {
    ResponseHeader h;
    h.timestamp = Now();
    h.requestHandle = request.requestHandle;
    h.serviceResult = result;
    return h;
}

}  // namespace rtos::opcua
