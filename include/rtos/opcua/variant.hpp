#pragma once
// Variant (Part 6 5.2.6) -- the tagged-union wire type used for every Read/Write attribute
// Value. Deliberately built as a std::variant of concrete C++ types (type-safe, no manual
// tagging) instead of open62541's void*+UA_DataType-descriptor approach -- "modern C++" per
// the project brief, at zero cost here since -fno-rtti/-fno-exceptions still apply (std::visit
// with `if constexpr` compiles down to a plain jump table, no dynamic_cast/throw involved).
//
// Extending this later (adding a type from the full OPC UA canon, e.g. Guid, ByteString,
// Int64/UInt64-as-payload, arrays): three places, no rework of the surrounding codec --
//   1. Add the C++ type as a new alternative to Variant::Storage below.
//   2. Add a TypeIdFor<T>() specialization.
//   3. Add one `else if constexpr` arm each in EncodeVariant()/DecodeVariant() (variant.cpp).
// Only the types this server's current address space actually needs are wired up today.
// Note: DateTime is intentionally NOT a Variant alternative (yet) -- it's a `using ... =
// int64_t` alias identical to Int64, so it can't coexist with Int64 in the same std::variant
// without a distinct wrapper type; add that wrapper when a node actually needs a DateTime
// Value (ServerStatus does not for the minimal address space built so far).
#include <variant>

#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

// Part 6 5.1.2 Table 1 -- the standard built-in "Identifier" values, which double as this
// type's NodeId in namespace 0 (see Variant::DataTypeNodeId()). Only scalar types (bit 7/6 of
// the wire encoding byte, marking Array/ArrayDimensions, are never set by this server).
enum class BuiltinTypeId : Byte {
    Boolean = 1, SByte = 2, Byte_ = 3, Int16 = 4, UInt16 = 5, Int32 = 6, UInt32 = 7,
    Int64 = 8, UInt64 = 9, Float = 10, Double = 11, String = 12, DateTime = 13, Guid = 14,
    ByteString = 15, XmlElement = 16, NodeId = 17, ExpandedNodeId = 18, StatusCode = 19,
    QualifiedName = 20, LocalizedText = 21, ExtensionObject = 22, DataValue = 23,
    Variant = 24, DiagnosticInfo = 25,
};

class Variant {
public:
    using Storage = std::variant<std::monostate, Boolean, SByte, Byte, Int16, UInt16, Int32,
                                 UInt32, Int64, UInt64, Float, Double, String,
                                 opcua::NodeId, StatusCode, QualifiedName, LocalizedText>;
    Storage storage;

    constexpr Variant() = default;

    // constexpr: lets a constexpr Node table (address_space.hpp) embed a constant Value
    // directly, e.g. Node{.staticValue = Variant::Of(String("hello"))} -- no runtime
    // construction, placed in Flash with the rest of the enclosing table.
    template <typename T>
    static constexpr Variant Of(T v) {
        Variant result;
        result.storage = std::move(v);
        return result;
    }

    constexpr bool IsEmpty() const { return std::holds_alternative<std::monostate>(storage); }

    // Undefined or the empty variant if IsEmpty() -- check first.
    BuiltinTypeId TypeId() const;
    // ns=0;i=<TypeId> -- the standard OPC UA DataType NodeId for this value's built-in type
    // (Part 6 5.1.2's Identifier values are, by design, the same numbers as their NS0 NodeIds).
    opcua::NodeId DataTypeNodeId() const;

    template <typename T>
    const T *GetIf() const { return std::get_if<T>(&storage); }
};

bool EncodeVariant(ByteWriter &w, const Variant &v);
// Rejects (returns false) Variants with the Array or ArrayDimensions flag bits set -- this
// server has no use for array-valued Variants yet (see header comment on extending this).
bool DecodeVariant(ByteReader &r, Variant &out);

}  // namespace rtos::opcua
