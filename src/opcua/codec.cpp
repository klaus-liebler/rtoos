#include "rtos/opcua/codec.hpp"

namespace rtos::opcua {

namespace {
// Part 6 5.2.2 Table 15
constexpr Byte NODEID_ENCODING_TWO_BYTE = 0x00;
constexpr Byte NODEID_ENCODING_FOUR_BYTE = 0x01;
constexpr Byte NODEID_ENCODING_NUMERIC = 0x02;
constexpr Byte NODEID_ENCODING_STRING = 0x03;
constexpr Byte NODEID_ENCODING_GUID = 0x04;
constexpr Byte NODEID_ENCODING_OPAQUE = 0x05;
// ExpandedNodeId (5.2.3): top two bits of the same encoding byte are flags on top of one of
// the NodeId forms above.
constexpr Byte EXPANDED_FLAG_NAMESPACE_URI = 0x80;
constexpr Byte EXPANDED_FLAG_SERVER_INDEX = 0x40;
constexpr Byte EXPANDED_ENCODING_MASK = 0x3F;
} // namespace

bool EncodeGuid(ByteWriter &w, const Guid &v) {
    if(!w.WriteUInt32(v.data1)) return false;
    if(!w.WriteUInt16(v.data2)) return false;
    if(!w.WriteUInt16(v.data3)) return false;
    return w.WriteRaw(v.data4, sizeof(v.data4));
}

bool DecodeGuid(ByteReader &r, Guid &out) {
    if(!r.ReadUInt32(out.data1)) return false;
    if(!r.ReadUInt16(out.data2)) return false;
    if(!r.ReadUInt16(out.data3)) return false;
    return r.ReadRaw(out.data4, sizeof(out.data4));
}

bool EncodeNodeId(ByteWriter &w, const NodeId &v) {
    if(v.type == NodeIdType::Numeric) {
        if(v.namespaceIndex == 0 && v.numeric <= 0xFF) {
            if(!w.WriteByte(NODEID_ENCODING_TWO_BYTE)) return false;
            return w.WriteByte(static_cast<Byte>(v.numeric));
        }
        if(v.namespaceIndex <= 0xFF && v.numeric <= 0xFFFF) {
            if(!w.WriteByte(NODEID_ENCODING_FOUR_BYTE)) return false;
            if(!w.WriteByte(static_cast<Byte>(v.namespaceIndex))) return false;
            return w.WriteUInt16(static_cast<UInt16>(v.numeric));
        }
        if(!w.WriteByte(NODEID_ENCODING_NUMERIC)) return false;
        if(!w.WriteUInt16(v.namespaceIndex)) return false;
        return w.WriteUInt32(v.numeric);
    }
    if(v.type == NodeIdType::String) {
        if(!w.WriteByte(NODEID_ENCODING_STRING)) return false;
        if(!w.WriteUInt16(v.namespaceIndex)) return false;
        return w.WriteString(v.string);
    }
    if(v.type == NodeIdType::Guid) {
        if(!w.WriteByte(NODEID_ENCODING_GUID)) return false;
        if(!w.WriteUInt16(v.namespaceIndex)) return false;
        return EncodeGuid(w, v.guid);
    }
    // Opaque
    if(!w.WriteByte(NODEID_ENCODING_OPAQUE)) return false;
    if(!w.WriteUInt16(v.namespaceIndex)) return false;
    return w.WriteByteString(v.string, false);
}

bool DecodeNodeId(ByteReader &r, NodeId &out) {
    Byte encoding = 0;
    if(!r.ReadByte(encoding)) return false;

    switch(encoding) {
    case NODEID_ENCODING_TWO_BYTE: {
        Byte id = 0;
        if(!r.ReadByte(id)) return false;
        out = NodeId(0, id);
        return true;
    }
    case NODEID_ENCODING_FOUR_BYTE: {
        Byte ns = 0;
        UInt16 id = 0;
        if(!r.ReadByte(ns)) return false;
        if(!r.ReadUInt16(id)) return false;
        out = NodeId(ns, id);
        return true;
    }
    case NODEID_ENCODING_NUMERIC: {
        UInt16 ns = 0;
        UInt32 id = 0;
        if(!r.ReadUInt16(ns)) return false;
        if(!r.ReadUInt32(id)) return false;
        out = NodeId(ns, id);
        return true;
    }
    case NODEID_ENCODING_STRING: {
        UInt16 ns = 0;
        String s;
        if(!r.ReadUInt16(ns)) return false;
        if(!r.ReadString(s)) return false;
        out = NodeId{};
        out.namespaceIndex = ns;
        out.type = NodeIdType::String;
        out.string = s.isNull ? std::string_view{} : s.value;
        return true;
    }
    case NODEID_ENCODING_GUID: {
        UInt16 ns = 0;
        Guid g;
        if(!r.ReadUInt16(ns)) return false;
        if(!DecodeGuid(r, g)) return false;
        out = NodeId{};
        out.namespaceIndex = ns;
        out.type = NodeIdType::Guid;
        out.guid = g;
        return true;
    }
    case NODEID_ENCODING_OPAQUE: {
        UInt16 ns = 0;
        std::string_view bytes;
        bool isNull = false;
        if(!r.ReadUInt16(ns)) return false;
        if(!r.ReadByteString(bytes, isNull)) return false;
        out = NodeId{};
        out.namespaceIndex = ns;
        out.type = NodeIdType::Opaque;
        out.string = bytes;
        return true;
    }
    default:
        return false; // unknown/unsupported encoding byte -- BadDecodingError upstream
    }
}

bool EncodeExpandedNodeId(ByteWriter &w, const ExpandedNodeId &v) {
    // Reuse EncodeNodeId's compact-form selection by encoding into a scratch position first is
    // not possible without a second buffer, so duplicate the (small) dispatch here with the
    // flag bits left clear -- NamespaceUri/ServerIndex are never present (see header comment).
    return EncodeNodeId(w, v);
}

bool DecodeExpandedNodeId(ByteReader &r, ExpandedNodeId &out) {
    // Peek the encoding byte to check the ExpandedNodeId-only flag bits, then let DecodeNodeId
    // consume the base NodeId form (masking the flags out first so it sees a plain, known
    // encoding byte).
    if(r.Remaining() < 1) return false;
    Byte encodingWithFlags = r.RemainingSpan()[0];
    Byte baseEncoding = encodingWithFlags & EXPANDED_ENCODING_MASK;

    // Consume+replace the peeked byte by decoding through a NodeId-shaped path: simplest
    // correct approach is a tiny local re-implementation rather than rewinding the reader
    // (ByteReader has no seek-backwards, by design -- streams are forward-only).
    Byte discard = 0;
    if(!r.ReadByte(discard)) return false;

    NodeId base;
    bool ok = true;
    switch(baseEncoding) {
    case 0x00: { Byte id = 0; ok = r.ReadByte(id); base = NodeId(0, id); break; }
    case 0x01: { Byte ns = 0; UInt16 id = 0; ok = r.ReadByte(ns) && r.ReadUInt16(id); base = NodeId(ns, id); break; }
    case 0x02: { UInt16 ns = 0; UInt32 id = 0; ok = r.ReadUInt16(ns) && r.ReadUInt32(id); base = NodeId(ns, id); break; }
    case 0x03: {
        UInt16 ns = 0; String s;
        ok = r.ReadUInt16(ns) && r.ReadString(s);
        base = NodeId{}; base.namespaceIndex = ns; base.type = NodeIdType::String;
        base.string = s.isNull ? std::string_view{} : s.value;
        break;
    }
    case 0x04: {
        UInt16 ns = 0; Guid g;
        ok = r.ReadUInt16(ns) && DecodeGuid(r, g);
        base = NodeId{}; base.namespaceIndex = ns; base.type = NodeIdType::Guid; base.guid = g;
        break;
    }
    case 0x05: {
        UInt16 ns = 0; std::string_view bytes; bool isNull = false;
        ok = r.ReadUInt16(ns) && r.ReadByteString(bytes, isNull);
        base = NodeId{}; base.namespaceIndex = ns; base.type = NodeIdType::Opaque; base.string = bytes;
        break;
    }
    default:
        return false;
    }
    if(!ok) return false;

    if(encodingWithFlags & EXPANDED_FLAG_NAMESPACE_URI) {
        String uri;
        if(!r.ReadString(uri)) return false;
    }
    if(encodingWithFlags & EXPANDED_FLAG_SERVER_INDEX) {
        UInt32 serverIndex = 0;
        if(!r.ReadUInt32(serverIndex)) return false;
    }
    out = base;
    return true;
}

bool EncodeQualifiedName(ByteWriter &w, const QualifiedName &v) {
    if(!w.WriteUInt16(v.namespaceIndex)) return false;
    return w.WriteString(v.name);
}

bool DecodeQualifiedName(ByteReader &r, QualifiedName &out) {
    String name;
    if(!r.ReadUInt16(out.namespaceIndex)) return false;
    if(!r.ReadString(name)) return false;
    out.name = name.isNull ? std::string_view{} : name.value;
    return true;
}

bool EncodeLocalizedText(ByteWriter &w, const LocalizedText &v) {
    Byte encoding = 0;
    if(!v.locale.empty()) encoding |= 0x01;
    if(!v.text.empty()) encoding |= 0x02;
    if(!w.WriteByte(encoding)) return false;
    if(encoding & 0x01) {
        if(!w.WriteString(v.locale)) return false;
    }
    if(encoding & 0x02) {
        if(!w.WriteString(v.text)) return false;
    }
    return true;
}

bool DecodeLocalizedText(ByteReader &r, LocalizedText &out) {
    Byte encoding = 0;
    if(!r.ReadByte(encoding)) return false;
    out = LocalizedText{};
    if(encoding & 0x01) {
        String s;
        if(!r.ReadString(s)) return false;
        out.locale = s.isNull ? std::string_view{} : s.value;
    }
    if(encoding & 0x02) {
        String s;
        if(!r.ReadString(s)) return false;
        out.text = s.isNull ? std::string_view{} : s.value;
    }
    return true;
}

}  // namespace rtos::opcua
