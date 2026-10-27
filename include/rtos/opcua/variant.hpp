#pragma once
// Variant (Part 6 5.2.2.16): the scalar built-in types this server's variables use, plus
// DateTime, ByteString and Guid. Arrays: a UInt32 array (the ArrayDimensions attribute) and a
// String array (Server.NamespaceArray); the application's variables are scalars.
//
// Decoding never fails on a well-formed value it cannot represent (an array, a matrix, an
// ExtensionObject, a DataValue, ...): the value is skipped and the Variant holds Unsupported.
// A Write of such a value is answered with BadTypeMismatch instead of closing the connection
// over a decoding error.
#include <algorithm>
#include <span>
#include <variant>

#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

enum class BuiltinTypeId : Byte {
    Boolean = 1, SByte = 2, Byte_ = 3, Int16 = 4, UInt16 = 5, Int32 = 6, UInt32 = 7,
    Int64 = 8, UInt64 = 9, Float = 10, Double = 11, String = 12, DateTime = 13, Guid = 14,
    ByteString = 15, XmlElement = 16, NodeId = 17, ExpandedNodeId = 18, StatusCode = 19,
    QualifiedName = 20, LocalizedText = 21, ExtensionObject = 22, DataValue = 23,
    Variant = 24, DiagnosticInfo = 25,
};

/// DateTime as a Variant alternative (DateTime itself is an Int64).
struct DateTimeValue {
    DateTime value = 0;
    friend constexpr bool operator==(const DateTimeValue &, const DateTimeValue &) = default;
};

/// ByteString as a Variant alternative (a view: valid as long as its source is).
struct ByteStringValue {
    std::string_view value;
    bool isNull = true;
    friend constexpr bool operator==(const ByteStringValue &a, const ByteStringValue &b) {
        return a.isNull == b.isNull && a.value == b.value;
    }
};

/// A one-dimensional UInt32 array (the ArrayDimensions attribute).
struct UInt32Array {
    std::span<const UInt32> values;
    friend constexpr bool operator==(const UInt32Array &a, const UInt32Array &b) {
        return a.values.size() == b.values.size() && std::equal(a.values.begin(), a.values.end(), b.values.begin());
    }
};

/// A one-dimensional String array (e.g. Server.NamespaceArray).
struct StringArray {
    std::span<const String> values;
    friend constexpr bool operator==(const StringArray &a, const StringArray &b) {
        return a.values.size() == b.values.size() && std::equal(a.values.begin(), a.values.end(), b.values.begin());
    }
};

/// A decoded value this server does not represent: its type, and whether it was an array.
struct Unsupported {
    BuiltinTypeId type = BuiltinTypeId::Boolean;
    bool array = false;
    friend constexpr bool operator==(const Unsupported &, const Unsupported &) = default;
};

class Variant {
public:
    using Storage = std::variant<std::monostate, Boolean, SByte, Byte, Int16, UInt16, Int32,
                                 UInt32, Int64, UInt64, Float, Double, String,
                                 opcua::NodeId, StatusCode, QualifiedName, LocalizedText,
                                 DateTimeValue, ByteStringValue, Guid, UInt32Array, StringArray, Unsupported>;
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
    constexpr bool IsSupported() const { return !std::holds_alternative<Unsupported>(storage); }
    constexpr bool IsArray() const {
        if(const auto *u = std::get_if<Unsupported>(&storage)) return u->array;
        return std::holds_alternative<UInt32Array>(storage) || std::holds_alternative<StringArray>(storage);
    }

    // Undefined or the empty variant if IsEmpty() -- check first.
    BuiltinTypeId TypeId() const;
    // ns=0;i=<TypeId> -- the standard OPC UA DataType NodeId for this value's built-in type
    // (Part 6 5.1.2's Identifier values are, by design, the same numbers as their NS0 NodeIds).
    opcua::NodeId DataTypeNodeId() const;

    template <typename T>
    const T *GetIf() const { return std::get_if<T>(&storage); }

    friend bool operator==(const Variant &a, const Variant &b) { return a.storage == b.storage; }
};

bool EncodeVariant(ByteWriter &w, const Variant &v);
/// False only for a malformed encoding (see the file comment).
bool DecodeVariant(ByteReader &r, Variant &out);
/// Skip one encoded value of built-in type `type` (any of the 25, nested ones included).
bool SkipBuiltin(ByteReader &r, BuiltinTypeId type, int depth = 0);

}  // namespace rtos::opcua
