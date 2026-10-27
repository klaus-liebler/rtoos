#pragma once
// Encode/decode for the composite built-in types (Part 6 5.2) that sit below Variant:
// NodeId, ExpandedNodeId, QualifiedName, LocalizedText, Guid. Variant itself (5.2.6) lives in
// variant.hpp/.cpp since it needs its own type-table registry, kept separate to keep this file
// focused on the "no registry needed" composite types.
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

bool EncodeGuid(ByteWriter &w, const Guid &v);
bool DecodeGuid(ByteReader &r, Guid &out);

// Always picks the most compact wire form for Numeric NodeIds (Two-Byte/Four-Byte/full
// Numeric, Part 6 5.2.2) -- this server only ever hands out Numeric NodeIds of its own, so
// String/Guid/Opaque encoding is intentionally not implemented (DecodeNodeId still accepts
// them, for robustness against an unexpected client-supplied NodeId).
bool EncodeNodeId(ByteWriter &w, const NodeId &v);
bool DecodeNodeId(ByteReader &r, NodeId &out);

// ExpandedNodeId (5.2.3): NodeId + optional NamespaceUri/ServerIndex. This server is never
// itself referenced by NamespaceUri or from another server, so those two optional fields are
// always encoded absent; DecodeExpandedNodeId still consumes them if a client sets the flags,
// to stay in sync with the stream.
bool EncodeExpandedNodeId(ByteWriter &w, const ExpandedNodeId &v);
bool DecodeExpandedNodeId(ByteReader &r, ExpandedNodeId &out);

bool EncodeQualifiedName(ByteWriter &w, const QualifiedName &v);
bool DecodeQualifiedName(ByteReader &r, QualifiedName &out);

bool EncodeLocalizedText(ByteWriter &w, const LocalizedText &v);
bool DecodeLocalizedText(ByteReader &r, LocalizedText &out);

}  // namespace rtos::opcua
