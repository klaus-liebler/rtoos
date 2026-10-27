#include "rtos/opcua/variant.hpp"

#include <type_traits>

#include "rtos/opcua/codec.hpp"

namespace rtos::opcua {

namespace {
constexpr Byte VARIANT_ARRAY_FLAG = 0x80;
constexpr Byte VARIANT_ARRAY_DIMENSIONS_FLAG = 0x40;
constexpr Byte VARIANT_ENCODING_MASK = 0x3F;
// Nesting of Variant/DataValue/DiagnosticInfo inside each other while skipping (Part 6 5.2.2:
// decoders shall limit recursion; open62541 uses 100, a few levels are plenty here).
constexpr int MAX_DEPTH = 8;
// Elements of a skipped array: a sanity bound against huge declared lengths (the reader
// fails anyway once the message ends).
constexpr Int32 MAX_ARRAY_LENGTH = 65536;

template <typename T> constexpr BuiltinTypeId TypeIdFor();
template <> constexpr BuiltinTypeId TypeIdFor<Boolean>() { return BuiltinTypeId::Boolean; }
template <> constexpr BuiltinTypeId TypeIdFor<SByte>() { return BuiltinTypeId::SByte; }
template <> constexpr BuiltinTypeId TypeIdFor<Byte>() { return BuiltinTypeId::Byte_; }
template <> constexpr BuiltinTypeId TypeIdFor<Int16>() { return BuiltinTypeId::Int16; }
template <> constexpr BuiltinTypeId TypeIdFor<UInt16>() { return BuiltinTypeId::UInt16; }
template <> constexpr BuiltinTypeId TypeIdFor<Int32>() { return BuiltinTypeId::Int32; }
template <> constexpr BuiltinTypeId TypeIdFor<UInt32>() { return BuiltinTypeId::UInt32; }
template <> constexpr BuiltinTypeId TypeIdFor<Int64>() { return BuiltinTypeId::Int64; }
template <> constexpr BuiltinTypeId TypeIdFor<UInt64>() { return BuiltinTypeId::UInt64; }
template <> constexpr BuiltinTypeId TypeIdFor<Float>() { return BuiltinTypeId::Float; }
template <> constexpr BuiltinTypeId TypeIdFor<Double>() { return BuiltinTypeId::Double; }
template <> constexpr BuiltinTypeId TypeIdFor<String>() { return BuiltinTypeId::String; }
template <> constexpr BuiltinTypeId TypeIdFor<opcua::NodeId>() { return BuiltinTypeId::NodeId; }
template <> constexpr BuiltinTypeId TypeIdFor<StatusCode>() { return BuiltinTypeId::StatusCode; }
template <> constexpr BuiltinTypeId TypeIdFor<QualifiedName>() { return BuiltinTypeId::QualifiedName; }
template <> constexpr BuiltinTypeId TypeIdFor<LocalizedText>() { return BuiltinTypeId::LocalizedText; }
template <> constexpr BuiltinTypeId TypeIdFor<DateTimeValue>() { return BuiltinTypeId::DateTime; }
template <> constexpr BuiltinTypeId TypeIdFor<ByteStringValue>() { return BuiltinTypeId::ByteString; }
template <> constexpr BuiltinTypeId TypeIdFor<Guid>() { return BuiltinTypeId::Guid; }
template <> constexpr BuiltinTypeId TypeIdFor<UInt32Array>() { return BuiltinTypeId::UInt32; }
template <> constexpr BuiltinTypeId TypeIdFor<StringArray>() { return BuiltinTypeId::String; }

bool ValidType(Byte t) { return t >= 1 && t <= 25; }

bool SkipExpandedNodeId(ByteReader &r) {
    ExpandedNodeId id;
    return DecodeExpandedNodeId(r, id);
}

bool DecodeVariantAt(ByteReader &r, Variant &out, int depth);

bool SkipDiagnosticInfo(ByteReader &r, int depth) {
    if(depth > MAX_DEPTH) return false;
    Byte mask = 0;
    if(!r.ReadByte(mask)) return false;
    Int32 i = 0;
    if((mask & 0x01) && !r.ReadInt32(i)) return false; // SymbolicId
    if((mask & 0x02) && !r.ReadInt32(i)) return false; // NamespaceUri
    if((mask & 0x08) && !r.ReadInt32(i)) return false; // LocalizedText
    if((mask & 0x04) && !r.ReadInt32(i)) return false; // Locale
    String s;
    if((mask & 0x10) && !r.ReadString(s)) return false; // AdditionalInfo
    StatusCode sc{};
    if((mask & 0x20) && !r.ReadStatusCode(sc)) return false; // InnerStatusCode
    if(mask & 0x40) return SkipDiagnosticInfo(r, depth + 1);  // InnerDiagnosticInfo
    return true;
}

bool SkipDataValue(ByteReader &r, int depth) {
    Byte mask = 0;
    if(!r.ReadByte(mask)) return false;
    if((mask & 0x01) && !SkipBuiltin(r, BuiltinTypeId::Variant, depth + 1)) return false;
    if((mask & 0x02) && !r.Skip(4)) return false;
    if((mask & 0x04) && !r.Skip(8)) return false;
    if((mask & 0x10) && !r.Skip(2)) return false;
    if((mask & 0x08) && !r.Skip(8)) return false;
    if((mask & 0x20) && !r.Skip(2)) return false;
    return true;
}

template <class T>
bool DecodeScalar(ByteReader &r, BuiltinTypeId type, Variant &out) {
    T v{};
    bool ok = false;
    if constexpr(std::is_same_v<T, Boolean>) ok = r.ReadBoolean(v);
    else if constexpr(std::is_same_v<T, SByte>) ok = r.ReadSByte(v);
    else if constexpr(std::is_same_v<T, Byte>) ok = r.ReadByte(v);
    else if constexpr(std::is_same_v<T, Int16>) ok = r.ReadInt16(v);
    else if constexpr(std::is_same_v<T, UInt16>) ok = r.ReadUInt16(v);
    else if constexpr(std::is_same_v<T, Int32>) ok = r.ReadInt32(v);
    else if constexpr(std::is_same_v<T, UInt32>) ok = r.ReadUInt32(v);
    else if constexpr(std::is_same_v<T, Int64>) ok = r.ReadInt64(v);
    else if constexpr(std::is_same_v<T, UInt64>) ok = r.ReadUInt64(v);
    else if constexpr(std::is_same_v<T, Float>) ok = r.ReadFloat(v);
    else if constexpr(std::is_same_v<T, Double>) ok = r.ReadDouble(v);
    else if constexpr(std::is_same_v<T, String>) ok = r.ReadString(v);
    else if constexpr(std::is_same_v<T, opcua::NodeId>) ok = DecodeNodeId(r, v);
    else if constexpr(std::is_same_v<T, StatusCode>) ok = r.ReadStatusCode(v);
    else if constexpr(std::is_same_v<T, QualifiedName>) ok = DecodeQualifiedName(r, v);
    else if constexpr(std::is_same_v<T, LocalizedText>) ok = DecodeLocalizedText(r, v);
    else if constexpr(std::is_same_v<T, DateTimeValue>) ok = r.ReadDateTime(v.value);
    else if constexpr(std::is_same_v<T, ByteStringValue>) ok = r.ReadByteString(v.value, v.isNull);
    else if constexpr(std::is_same_v<T, Guid>) ok = DecodeGuid(r, v);
    (void)type;
    if(ok) out = Variant::Of(v);
    return ok;
}
} // namespace

bool SkipBuiltin(ByteReader &r, BuiltinTypeId type, int depth) {
    if(depth > MAX_DEPTH) return false;
    switch(type) {
    case BuiltinTypeId::Boolean:
    case BuiltinTypeId::SByte:
    case BuiltinTypeId::Byte_: return r.Skip(1);
    case BuiltinTypeId::Int16:
    case BuiltinTypeId::UInt16: return r.Skip(2);
    case BuiltinTypeId::Int32:
    case BuiltinTypeId::UInt32:
    case BuiltinTypeId::Float:
    case BuiltinTypeId::StatusCode: return r.Skip(4);
    case BuiltinTypeId::Int64:
    case BuiltinTypeId::UInt64:
    case BuiltinTypeId::Double:
    case BuiltinTypeId::DateTime: return r.Skip(8);
    case BuiltinTypeId::Guid: return r.Skip(16);
    case BuiltinTypeId::String:
    case BuiltinTypeId::XmlElement: { String s; return r.ReadString(s); }
    case BuiltinTypeId::ByteString: { std::string_view b; bool n = true; return r.ReadByteString(b, n); }
    case BuiltinTypeId::NodeId: { opcua::NodeId id; return DecodeNodeId(r, id); }
    case BuiltinTypeId::ExpandedNodeId: return SkipExpandedNodeId(r);
    case BuiltinTypeId::QualifiedName: { QualifiedName q; return DecodeQualifiedName(r, q); }
    case BuiltinTypeId::LocalizedText: { LocalizedText t; return DecodeLocalizedText(r, t); }
    case BuiltinTypeId::ExtensionObject: {
        opcua::NodeId typeId;
        Byte encoding = 0;
        if(!DecodeNodeId(r, typeId) || !r.ReadByte(encoding)) return false;
        if(encoding == 0x01 || encoding == 0x02) { std::string_view b; bool n = true; return r.ReadByteString(b, n); }
        return encoding == 0x00;
    }
    case BuiltinTypeId::DataValue: return SkipDataValue(r, depth);
    case BuiltinTypeId::Variant: { Variant v; return DecodeVariantAt(r, v, depth + 1); }
    case BuiltinTypeId::DiagnosticInfo: return SkipDiagnosticInfo(r, depth);
    }
    return false;
}

BuiltinTypeId Variant::TypeId() const {
    return std::visit([](auto &&v) -> BuiltinTypeId {
        using T = std::decay_t<decltype(v)>;
        if constexpr(std::is_same_v<T, std::monostate>)
            return BuiltinTypeId::Boolean; // unreachable in practice -- caller checks IsEmpty() first
        else if constexpr(std::is_same_v<T, Unsupported>)
            return v.type;
        else
            return TypeIdFor<T>();
    }, storage);
}

opcua::NodeId Variant::DataTypeNodeId() const {
    return opcua::NodeId(0, static_cast<UInt32>(TypeId()));
}

bool EncodeVariant(ByteWriter &w, const Variant &v) {
    if(v.IsEmpty())
        return w.WriteByte(0); // Part 6 5.2.6: encoding byte 0 = "no value"
    if(!v.IsSupported())
        return false; // a decoded placeholder, never sent

    if(const UInt32Array *a = v.GetIf<UInt32Array>()) {
        if(!w.WriteByte(static_cast<Byte>(BuiltinTypeId::UInt32) | VARIANT_ARRAY_FLAG)) return false;
        if(!w.WriteInt32(static_cast<Int32>(a->values.size()))) return false;
        for(UInt32 x : a->values)
            if(!w.WriteUInt32(x)) return false;
        return true;
    }
    if(const StringArray *a = v.GetIf<StringArray>()) {
        if(!w.WriteByte(static_cast<Byte>(BuiltinTypeId::String) | VARIANT_ARRAY_FLAG)) return false;
        if(!w.WriteInt32(static_cast<Int32>(a->values.size()))) return false;
        for(const String &x : a->values)
            if(!w.WriteString(x)) return false;
        return true;
    }

    if(!w.WriteByte(static_cast<Byte>(v.TypeId())))
        return false;

    return std::visit([&](auto &&val) -> bool {
        using T = std::decay_t<decltype(val)>;
        if constexpr(std::is_same_v<T, std::monostate> || std::is_same_v<T, Unsupported> ||
                     std::is_same_v<T, UInt32Array> || std::is_same_v<T, StringArray>) {
            return false; // handled above
        } else if constexpr(std::is_same_v<T, Boolean>) {
            return w.WriteBoolean(val);
        } else if constexpr(std::is_same_v<T, SByte>) {
            return w.WriteSByte(val);
        } else if constexpr(std::is_same_v<T, Byte>) {
            return w.WriteByte(val);
        } else if constexpr(std::is_same_v<T, Int16>) {
            return w.WriteInt16(val);
        } else if constexpr(std::is_same_v<T, UInt16>) {
            return w.WriteUInt16(val);
        } else if constexpr(std::is_same_v<T, Int32>) {
            return w.WriteInt32(val);
        } else if constexpr(std::is_same_v<T, UInt32>) {
            return w.WriteUInt32(val);
        } else if constexpr(std::is_same_v<T, Int64>) {
            return w.WriteInt64(val);
        } else if constexpr(std::is_same_v<T, UInt64>) {
            return w.WriteUInt64(val);
        } else if constexpr(std::is_same_v<T, Float>) {
            return w.WriteFloat(val);
        } else if constexpr(std::is_same_v<T, Double>) {
            return w.WriteDouble(val);
        } else if constexpr(std::is_same_v<T, String>) {
            return w.WriteString(val);
        } else if constexpr(std::is_same_v<T, opcua::NodeId>) {
            return EncodeNodeId(w, val);
        } else if constexpr(std::is_same_v<T, StatusCode>) {
            return w.WriteStatusCode(val);
        } else if constexpr(std::is_same_v<T, QualifiedName>) {
            return EncodeQualifiedName(w, val);
        } else if constexpr(std::is_same_v<T, LocalizedText>) {
            return EncodeLocalizedText(w, val);
        } else if constexpr(std::is_same_v<T, DateTimeValue>) {
            return w.WriteDateTime(val.value);
        } else if constexpr(std::is_same_v<T, ByteStringValue>) {
            return w.WriteByteString(val.value, val.isNull);
        } else if constexpr(std::is_same_v<T, Guid>) {
            return EncodeGuid(w, val);
        } else {
            static_assert(!sizeof(T *), "EncodeVariant: missing case for a Variant::Storage alternative");
            return false;
        }
    }, v.storage);
}

bool DecodeVariant(ByteReader &r, Variant &out) { return DecodeVariantAt(r, out, 0); }

namespace {
bool DecodeVariantAt(ByteReader &r, Variant &out, int depth) {
    if(depth > MAX_DEPTH) return false;
    Byte encoding = 0;
    if(!r.ReadByte(encoding))
        return false;

    if(encoding == 0) {
        out = Variant{};
        return true;
    }
    const Byte typeByte = encoding & VARIANT_ENCODING_MASK;
    if(!ValidType(typeByte)) return false;
    const auto type = static_cast<BuiltinTypeId>(typeByte);

    if(encoding & VARIANT_ARRAY_FLAG) {
        // Arrays are skipped (see variant.hpp): the elements, then the optional dimensions.
        Int32 length = 0;
        if(!r.ReadInt32(length) || length > MAX_ARRAY_LENGTH) return false;
        for(Int32 i = 0; i < length; i++)
            if(!SkipBuiltin(r, type, depth + 1)) return false;
        if(encoding & VARIANT_ARRAY_DIMENSIONS_FLAG) {
            Int32 dims = 0;
            if(!r.ReadInt32(dims) || dims > MAX_ARRAY_LENGTH) return false;
            if(!r.Skip(static_cast<size_t>(dims < 0 ? 0 : dims) * 4)) return false;
        }
        out = Variant::Of(Unsupported{type, true});
        return true;
    }
    if(encoding & VARIANT_ARRAY_DIMENSIONS_FLAG) return false; // dimensions without an array

    switch(type) {
    case BuiltinTypeId::Boolean: return DecodeScalar<Boolean>(r, type, out);
    case BuiltinTypeId::SByte: return DecodeScalar<SByte>(r, type, out);
    case BuiltinTypeId::Byte_: return DecodeScalar<Byte>(r, type, out);
    case BuiltinTypeId::Int16: return DecodeScalar<Int16>(r, type, out);
    case BuiltinTypeId::UInt16: return DecodeScalar<UInt16>(r, type, out);
    case BuiltinTypeId::Int32: return DecodeScalar<Int32>(r, type, out);
    case BuiltinTypeId::UInt32: return DecodeScalar<UInt32>(r, type, out);
    case BuiltinTypeId::Int64: return DecodeScalar<Int64>(r, type, out);
    case BuiltinTypeId::UInt64: return DecodeScalar<UInt64>(r, type, out);
    case BuiltinTypeId::Float: return DecodeScalar<Float>(r, type, out);
    case BuiltinTypeId::Double: return DecodeScalar<Double>(r, type, out);
    case BuiltinTypeId::String: return DecodeScalar<String>(r, type, out);
    case BuiltinTypeId::NodeId: return DecodeScalar<opcua::NodeId>(r, type, out);
    case BuiltinTypeId::StatusCode: return DecodeScalar<StatusCode>(r, type, out);
    case BuiltinTypeId::QualifiedName: return DecodeScalar<QualifiedName>(r, type, out);
    case BuiltinTypeId::LocalizedText: return DecodeScalar<LocalizedText>(r, type, out);
    case BuiltinTypeId::DateTime: return DecodeScalar<DateTimeValue>(r, type, out);
    case BuiltinTypeId::ByteString: return DecodeScalar<ByteStringValue>(r, type, out);
    case BuiltinTypeId::Guid: return DecodeScalar<Guid>(r, type, out);
    default:
        // XmlElement, ExpandedNodeId, ExtensionObject, DataValue, Variant, DiagnosticInfo
        if(!SkipBuiltin(r, type, depth + 1)) return false;
        out = Variant::Of(Unsupported{type, false});
        return true;
    }
}
} // namespace

}  // namespace rtos::opcua
