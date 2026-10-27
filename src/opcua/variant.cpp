#include "rtos/opcua/variant.hpp"

#include <type_traits>

#include "rtos/opcua/codec.hpp"

namespace rtos::opcua {

namespace {
constexpr Byte VARIANT_ARRAY_FLAG = 0x80;
constexpr Byte VARIANT_ARRAY_DIMENSIONS_FLAG = 0x40;
constexpr Byte VARIANT_ENCODING_MASK = 0x3F;

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
} // namespace

BuiltinTypeId Variant::TypeId() const {
    return std::visit([](auto &&v) -> BuiltinTypeId {
        using T = std::decay_t<decltype(v)>;
        if constexpr(std::is_same_v<T, std::monostate>)
            return BuiltinTypeId::Boolean; // unreachable in practice -- caller checks IsEmpty() first
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

    if(!w.WriteByte(static_cast<Byte>(v.TypeId())))
        return false;

    return std::visit([&](auto &&val) -> bool {
        using T = std::decay_t<decltype(val)>;
        if constexpr(std::is_same_v<T, std::monostate>) {
            return false; // unreachable, IsEmpty() already handled above
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
        } else {
            static_assert(!sizeof(T *), "EncodeVariant: missing case for a Variant::Storage alternative");
            return false;
        }
    }, v.storage);
}

bool DecodeVariant(ByteReader &r, Variant &out) {
    Byte encoding = 0;
    if(!r.ReadByte(encoding))
        return false;

    if(encoding == 0) {
        out = Variant{};
        return true;
    }
    if(encoding & (VARIANT_ARRAY_FLAG | VARIANT_ARRAY_DIMENSIONS_FLAG))
        return false; // arrays not supported yet, see variant.hpp

    switch(static_cast<BuiltinTypeId>(encoding & VARIANT_ENCODING_MASK)) {
    case BuiltinTypeId::Boolean: {
        Boolean v = false;
        if(!r.ReadBoolean(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::SByte: {
        SByte v = 0;
        if(!r.ReadSByte(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::Byte_: {
        Byte v = 0;
        if(!r.ReadByte(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::Int16: {
        Int16 v = 0;
        if(!r.ReadInt16(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::UInt16: {
        UInt16 v = 0;
        if(!r.ReadUInt16(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::Int32: {
        Int32 v = 0;
        if(!r.ReadInt32(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::UInt32: {
        UInt32 v = 0;
        if(!r.ReadUInt32(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::Int64: {
        Int64 v = 0;
        if(!r.ReadInt64(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::UInt64: {
        UInt64 v = 0;
        if(!r.ReadUInt64(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::Float: {
        Float v = 0;
        if(!r.ReadFloat(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::Double: {
        Double v = 0;
        if(!r.ReadDouble(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::String: {
        String v;
        if(!r.ReadString(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::NodeId: {
        opcua::NodeId v;
        if(!DecodeNodeId(r, v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::StatusCode: {
        StatusCode v{};
        if(!r.ReadStatusCode(v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::QualifiedName: {
        QualifiedName v;
        if(!DecodeQualifiedName(r, v)) return false;
        out = Variant::Of(v);
        return true;
    }
    case BuiltinTypeId::LocalizedText: {
        LocalizedText v;
        if(!DecodeLocalizedText(r, v)) return false;
        out = Variant::Of(v);
        return true;
    }
    default:
        return false; // type not (yet) supported by this server -- see variant.hpp
    }
}

}  // namespace rtos::opcua
