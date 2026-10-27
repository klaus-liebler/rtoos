// SPDX-License-Identifier: MIT
//
// Binary encoding of the built-in types (Part 6 5.2), ported from open62541
// tests/check_types_builtin_binary.c: known encodings, round trips, and the values the decoder
// skips (arrays, matrices, ExtensionObjects, DiagnosticInfos, DataValues) without losing its
// place in the message. Tests about open62541's type API (custom types, allocators, calcSize,
// JSON/XML) are listed with their reasons in tests/open62541/rules.tsv.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/variant.hpp"

namespace {

namespace ua = rtos::opcua;
using ua::Byte;
using Bytes = std::vector<Byte>;

Bytes encode(const ua::Variant& v) {
    std::array<Byte, 512> buffer{};
    ua::ByteWriter w(buffer);
    EXPECT_TRUE(ua::EncodeVariant(w, v));
    return Bytes(w.Written().begin(), w.Written().end());
}

ua::Variant decode(const Bytes& bytes, std::size_t* consumed = nullptr) {
    ua::ByteReader r(bytes);
    ua::Variant v;
    EXPECT_TRUE(ua::DecodeVariant(r, v));
    if (consumed) *consumed = r.Position();
    return v;
}

/// The Variant encodes to exactly `expected` and decodes back to itself.
void round_trip(const ua::Variant& v, const Bytes& expected) {
    const Bytes bytes = encode(v);
    EXPECT_EQ(bytes, expected);
    std::size_t consumed = 0;
    const ua::Variant back = decode(bytes, &consumed);
    EXPECT_EQ(consumed, bytes.size());
    EXPECT_EQ(back, v);
}

// open62541: check_types_builtin_binary.c binary_bool_true
// open62541: check_types_builtin_binary.c binary_bool_false
TEST(OpcUaCodec, binary_bool) {
    round_trip(ua::Variant::Of(true), {1, 0x01});
    round_trip(ua::Variant::Of(false), {1, 0x00});
}

// open62541: check_types_builtin_binary.c binary_sbyte
// open62541: check_types_builtin_binary.c binary_byte
// open62541: check_types_builtin_binary.c binary_int16
// open62541: check_types_builtin_binary.c binary_uint16
// open62541: check_types_builtin_binary.c binary_int32
// open62541: check_types_builtin_binary.c binary_uint32
// open62541: check_types_builtin_binary.c binary_int64
// open62541: check_types_builtin_binary.c binary_uint64
TEST(OpcUaCodec, binary_integers) {
    round_trip(ua::Variant::Of(ua::SByte{-2}), {2, 0xFE});
    round_trip(ua::Variant::Of(ua::Byte{0xAB}), {3, 0xAB});
    round_trip(ua::Variant::Of(ua::Int16{-2}), {4, 0xFE, 0xFF});
    round_trip(ua::Variant::Of(ua::UInt16{0x1234}), {5, 0x34, 0x12});
    round_trip(ua::Variant::Of(ua::Int32{-1}), {6, 0xFF, 0xFF, 0xFF, 0xFF});
    round_trip(ua::Variant::Of(ua::UInt32{0x12345678}), {7, 0x78, 0x56, 0x34, 0x12});
    round_trip(ua::Variant::Of(ua::Int64{std::numeric_limits<ua::Int64>::min()}), {8, 0, 0, 0, 0, 0, 0, 0, 0x80});
    round_trip(ua::Variant::Of(ua::UInt64{0x0102030405060708ull}), {9, 8, 7, 6, 5, 4, 3, 2, 1});
}

// IEEE 754, little endian (Part 6 5.2.2.3)
// open62541: check_types_builtin_binary.c binary_float_nan
// open62541: check_types_builtin_binary.c binary_float_inf
// open62541: check_types_builtin_binary.c binary_float_neginf
// open62541: check_types_builtin_binary.c binary_double_nan
// open62541: check_types_builtin_binary.c binary_double_inf
TEST(OpcUaCodec, binary_floats) {
    round_trip(ua::Variant::Of(ua::Float{1.5f}), {10, 0x00, 0x00, 0xC0, 0x3F});
    round_trip(ua::Variant::Of(std::numeric_limits<ua::Float>::infinity()), {10, 0x00, 0x00, 0x80, 0x7F});
    round_trip(ua::Variant::Of(-std::numeric_limits<ua::Float>::infinity()), {10, 0x00, 0x00, 0x80, 0xFF});
    round_trip(ua::Variant::Of(std::numeric_limits<ua::Double>::infinity()), {11, 0, 0, 0, 0, 0, 0, 0xF0, 0x7F});
    // NaN: any NaN decodes as NaN (NaN != NaN, so no round-trip equality)
    for (const Bytes& nan : {Bytes{10, 0x00, 0x00, 0xC0, 0x7F}, Bytes{11, 0, 0, 0, 0, 0, 0, 0xF8, 0x7F}}) {
        const ua::Variant v = decode(nan);
        const double x = v.GetIf<ua::Float>() ? *v.GetIf<ua::Float>() : *v.GetIf<ua::Double>();
        EXPECT_TRUE(std::isnan(x));
        EXPECT_EQ(encode(v), nan);
    }
}

// String (Part 6 5.2.2.4): Int32 length, -1 = null, 0 = empty
// open62541: check_types_builtin_binary.c binary_string_empty
// open62541: check_encoding_roundtrip.c binary_encode_decode_string
// open62541: check_types_builtin_binary.c binary_string_null
// open62541: check_types_builtin_binary.c binary_string_long
TEST(OpcUaCodec, binary_strings) {
    round_trip(ua::Variant::Of(ua::String("")), {12, 0, 0, 0, 0});
    round_trip(ua::Variant::Of(ua::String::Null()), {12, 0xFF, 0xFF, 0xFF, 0xFF});
    const std::string text(300, 'a');
    Bytes expected = {12, 0x2C, 0x01, 0, 0};
    expected.insert(expected.end(), text.begin(), text.end());
    round_trip(ua::Variant::Of(ua::String(text)), expected);
}

// open62541: check_types_builtin_binary.c binary_datetime
// open62541: check_types_builtin_binary.c binary_guid
// open62541: check_types_builtin_binary.c binary_bytestring_empty
// open62541: check_types_builtin_binary.c binary_statuscode
TEST(OpcUaCodec, binary_datetime_guid_bytestring_statuscode) {
    round_trip(ua::Variant::Of(ua::DateTimeValue{0x01D9'0000'0000'0001}), {13, 1, 0, 0, 0, 0, 0, 0xD9, 0x01});
    ua::Guid g;
    g.data1 = 0x72962B91;
    g.data2 = 0xFA75;
    g.data3 = 0x4AE6;
    const Byte d4[8] = {0x8D, 0x28, 0xB4, 0x04, 0xDC, 0x7D, 0xAF, 0x63};
    std::copy(std::begin(d4), std::end(d4), std::begin(g.data4));
    // Part 6 5.2.2.6 example: 72962B91-FA75-4AE6-8D28-B404DC7DAF63
    round_trip(ua::Variant::Of(g), {14, 0x91, 0x2B, 0x96, 0x72, 0x75, 0xFA, 0xE6, 0x4A, 0x8D, 0x28, 0xB4, 0x04, 0xDC, 0x7D,
                                    0xAF, 0x63});
    round_trip(ua::Variant::Of(ua::ByteStringValue{"", false}), {15, 0, 0, 0, 0});
    round_trip(ua::Variant::Of(ua::ByteStringValue{{}, true}), {15, 0xFF, 0xFF, 0xFF, 0xFF});
    round_trip(ua::Variant::Of(ua::StatusCode::BadNodeIdUnknown), {19, 0x00, 0x00, 0x34, 0x80});
}

// NodeId encodings (Part 6 5.2.2.9): two-byte, four-byte, numeric, string, guid, opaque
// open62541: check_types_builtin_binary.c binary_nodeid_numeric
// open62541: check_encoding_roundtrip.c binary_encode_decode_nodeId
// open62541: check_types_builtin_binary.c binary_nodeid_fourbyte
// open62541: check_types_builtin_binary.c binary_nodeid_string
// open62541: check_types_builtin_binary.c binary_nodeid_guid
// open62541: check_types_builtin_binary.c binary_nodeid_bytestring
TEST(OpcUaCodec, binary_nodeids) {
    round_trip(ua::Variant::Of(ua::NodeId(0, 72)), {17, 0x00, 72});
    round_trip(ua::Variant::Of(ua::NodeId(5, 1025)), {17, 0x01, 5, 0x01, 0x04});
    round_trip(ua::Variant::Of(ua::NodeId(300, 0x12345678)), {17, 0x02, 0x2C, 0x01, 0x78, 0x56, 0x34, 0x12});
    round_trip(ua::Variant::Of(ua::NodeId::Str(1, "Hot")), {17, 0x03, 1, 0, 3, 0, 0, 0, 'H', 'o', 't'});
    // Guid and Opaque NodeIds decode (and compare) with their identifiers
    const Bytes guid = {17, 0x04, 2, 0, 0x91, 0x2B, 0x96, 0x72, 0x75, 0xFA, 0xE6, 0x4A, 0x8D, 0x28, 0xB4, 0x04, 0xDC, 0x7D, 0xAF, 0x63};
    const ua::Variant g = decode(guid);
    ASSERT_NE(g.GetIf<ua::NodeId>(), nullptr);
    EXPECT_EQ(g.GetIf<ua::NodeId>()->type, ua::NodeIdType::Guid);
    EXPECT_EQ(g.GetIf<ua::NodeId>()->guid.data1, 0x72962B91u);
    EXPECT_EQ(encode(g), guid);
    const Bytes opaque = {17, 0x05, 3, 0, 2, 0, 0, 0, 0xAA, 0xBB};
    const ua::Variant o = decode(opaque);
    ASSERT_NE(o.GetIf<ua::NodeId>(), nullptr);
    EXPECT_EQ(o.GetIf<ua::NodeId>()->type, ua::NodeIdType::Opaque);
    EXPECT_EQ(encode(o), opaque);
}

// ExpandedNodeId with NamespaceUri and ServerIndex: decoded (the flags consumed), the URI and
// index dropped -- this server has only local nodes.
// open62541: check_types_builtin_binary.c binary_expandednodeid_with_uri
TEST(OpcUaCodec, binary_expandednodeid_with_uri) {
    const Bytes bytes = {0x80 | 0x40 | 0x01, 0, 0x2A, 0x00, 3, 0, 0, 0, 'u', 'r', 'n', 7, 0, 0, 0, 0xEE};
    ua::ByteReader r(bytes);
    ua::ExpandedNodeId id;
    ASSERT_TRUE(ua::DecodeExpandedNodeId(r, id));
    EXPECT_EQ(id, ua::NodeId(0, 42));
    Byte next = 0;
    ASSERT_TRUE(r.ReadByte(next));
    EXPECT_EQ(next, 0xEE) << "URI and server index consumed";
}

// open62541: check_types_builtin_binary.c binary_qualifiedname
// open62541: check_types_builtin_binary.c binary_localizedtext
// open62541: check_types_builtin_binary.c binary_localizedtext_nolocale
TEST(OpcUaCodec, binary_names_and_texts) {
    round_trip(ua::Variant::Of(ua::QualifiedName(2, "ab")), {20, 2, 0, 2, 0, 0, 0, 'a', 'b'});
    round_trip(ua::Variant::Of(ua::LocalizedText("de", "x")), {21, 0x03, 2, 0, 0, 0, 'd', 'e', 1, 0, 0, 0, 'x'});
    round_trip(ua::Variant::Of(ua::LocalizedText("", "x")), {21, 0x02, 1, 0, 0, 0, 'x'});
}

// open62541: check_types_builtin_binary.c binary_variant_int32
// open62541: check_encoding_roundtrip.c binary_encode_decode_variant
// open62541: check_types_builtin_binary.c binary_variant_empty
TEST(OpcUaCodec, binary_variant) {
    round_trip(ua::Variant::Of(ua::Int32{42}), {6, 42, 0, 0, 0});
    round_trip(ua::Variant{}, {0});
}

/// The decoder skips `bytes` exactly and reports the type as Unsupported.
void skipped(const Bytes& bytes, ua::BuiltinTypeId type, bool array) {
    Bytes with_tail = bytes;
    with_tail.push_back(0xEE);
    std::size_t consumed = 0;
    const ua::Variant v = decode(with_tail, &consumed);
    EXPECT_EQ(consumed, bytes.size());
    ASSERT_FALSE(v.IsSupported());
    EXPECT_EQ(v.TypeId(), type);
    EXPECT_EQ(v.IsArray(), array);
}

// Arrays, matrices and structures are decoded past (Write: BadTypeMismatch) -- the message
// stays decodable.
// open62541: check_types_builtin_binary.c binary_variant_array
// open62541: check_types_builtin_binary.c binary_variant_matrix
// open62541: check_types_builtin_binary.c binary_variant_string_array
// open62541: check_types_builtin_binary.c binary_array_strings
TEST(OpcUaCodec, binary_variant_arrays_are_skipped) {
    skipped({0x80 | 6, 2, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0}, ua::BuiltinTypeId::Int32, true);
    skipped({0xC0 | 6, 4, 0, 0, 0, 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0, 2, 0, 0, 0},
            ua::BuiltinTypeId::Int32, true);
    skipped({0x80 | 12, 2, 0, 0, 0, 1, 0, 0, 0, 'a', 0xFF, 0xFF, 0xFF, 0xFF}, ua::BuiltinTypeId::String, true);
}

// open62541: check_types_builtin_binary.c binary_extensionobject_bytestring
// open62541: check_types_builtin_binary.c binary_extensionobject_xml
// open62541: check_types_builtin_binary.c binary_extensionobject_nobody
// open62541: check_types_builtin_binary.c binary_extensionobject_decoded
// open62541: check_types_builtin_binary.c binary_variant_extensionobject
// open62541: check_types_builtin_binary.c binary_variant_array_extensionobject
TEST(OpcUaCodec, binary_extensionobjects_are_skipped) {
    skipped({22, 0x01, 0x00, 0x34, 0x01, 0x01, 3, 0, 0, 0, 1, 2, 3}, ua::BuiltinTypeId::ExtensionObject, false);  // binary body
    skipped({22, 0x01, 0x00, 0x34, 0x01, 0x02, 1, 0, 0, 0, '<'}, ua::BuiltinTypeId::ExtensionObject, false);       // XML body
    skipped({22, 0x00, 0x00, 0x00}, ua::BuiltinTypeId::ExtensionObject, false);                      // no body
    skipped({0x80 | 22, 2, 0, 0, 0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x34, 0x01, 0x01, 1, 0, 0, 0, 9}, ua::BuiltinTypeId::ExtensionObject,
            true);
}

// open62541: check_types_builtin_binary.c binary_datavalue_all_fields
// open62541: check_types_builtin_binary.c binary_diagnosticinfo_full
TEST(OpcUaCodec, binary_datavalue_and_diagnosticinfo_are_skipped) {
    // DataValue: value Int32, status, source time + picos, server time + picos
    skipped({23, 0x3F, 6, 1, 0, 0, 0, 0, 0, 0x34, 0x80, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 0},
            ua::BuiltinTypeId::DataValue, false);
    // DiagnosticInfo with every field and a nested one
    skipped({25, 0x7F, 1, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 4, 0, 0, 0, 1, 0, 0, 0, 'i', 0, 0, 0x34, 0x80, 0x01, 5, 0, 0, 0},
            ua::BuiltinTypeId::DiagnosticInfo, false);
}

// A nested Variant beyond the nesting limit is a decoding error, not a stack overflow.
TEST(OpcUaCodec, NestingIsLimited) {
    Bytes deep(64, 24);  // Variant in Variant in ...
    deep.push_back(0);
    ua::ByteReader r(deep);
    ua::Variant v;
    EXPECT_FALSE(ua::DecodeVariant(r, v));
}

// open62541: check_types_builtin_binary.c binary_decode_truncated
// open62541: check_types_builtin_binary.c binary_decode_empty
// open62541: check_types_builtin_binary.c binary_variant_invalidDimensions
TEST(OpcUaCodec, binary_decode_truncated) {
    for (const Bytes& bad : {Bytes{}, Bytes{6, 1, 2}, Bytes{12, 5, 0, 0, 0, 'a'}, Bytes{0x80 | 6, 3, 0, 0, 0, 1, 0, 0, 0},
                             Bytes{0x40 | 6, 1, 0, 0, 0}, Bytes{26}, Bytes{0x3F}}) {
        ua::ByteReader r(bad);
        ua::Variant v;
        EXPECT_FALSE(ua::DecodeVariant(r, v)) << bad.size();
    }
}

}  // namespace
