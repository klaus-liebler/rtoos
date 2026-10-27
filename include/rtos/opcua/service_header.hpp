#pragma once
// RequestHeader/ResponseHeader (Part 4 7.33/7.34) -- the fixed preamble every service
// request/response carries, right after the service's own TypeId NodeId. This server only ever
// DECODES a RequestHeader (client -> server) and ENCODES a ResponseHeader (server -> client),
// never the reverse, since it's a server-only implementation (no OPC UA client role).
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

struct RequestHeader {
    NodeId authenticationToken;
    DateTime timestamp = 0;
    UInt32 requestHandle = 0;
    UInt32 returnDiagnostics = 0;
    String auditEntryId;
    UInt32 timeoutHint = 0;
    // additionalHeader (ExtensionObject) is decoded and discarded -- this server defines no
    // vendor-specific request extensions.
};
bool DecodeRequestHeader(ByteReader &r, RequestHeader &out);

struct ResponseHeader {
    DateTime timestamp = 0;
    UInt32 requestHandle = 0;
    StatusCode serviceResult = StatusCode::Good;
    // serviceDiagnostics/stringTable/additionalHeader are always encoded as "none/empty" --
    // this server never returns diagnostic detail beyond the StatusCode itself.
};
bool EncodeResponseHeader(ByteWriter &w, const ResponseHeader &h);

// Builds a ResponseHeader that echoes requestHandle and carries the given result -- the
// standard shape needed by every service handler's response.
ResponseHeader MakeResponseHeader(const RequestHeader &request, StatusCode result);

// ExtensionObject (Part 6 5.2.10) encoded as "no body" (TypeId = ns=0;i=0, encoding byte 0) --
// used for RequestHeader.additionalHeader on encode, and reused as the generic "encode an
// empty extension object" building block anywhere else it's needed later (e.g. a future
// service that doesn't yet have a real payload type implemented).
bool EncodeNullExtensionObject(ByteWriter &w);
// Decodes (and discards the body of) an ExtensionObject in whatever form the client actually
// sent -- correctness here matters even though the content is unused, to keep the stream
// position in sync for whatever follows.
bool DecodeAndDiscardExtensionObject(ByteReader &r);

}  // namespace rtos::opcua
