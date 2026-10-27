// SPDX-License-Identifier: MIT
//
// Attribute service set (Read, Write), ported from open62541
// tests/server/check_services_attributes.c. Differences to open62541 that follow from this
// server's design are noted at the tests: the address space is constexpr (Flash), so no
// attribute but Value is writable (WriteMask 0 -> BadNotWritable), values carry no stored
// status or source timestamp, and the variables are scalars. Tests that do not fit are listed
// with their reasons in tests/open62541/rules.tsv.
#include "opcua_service_harness.hpp"

namespace {

using namespace harness;

ua::NodeId ns0(ua::UInt32 id) { return ua::NodeId(0, id); }
constexpr ua::UInt32 attr(ua::ns0::AttributeId a) { return static_cast<ua::UInt32>(a); }
using A = ua::ns0::AttributeId;

class Open62541AttributesTest : public ServiceTest {};

// ---- Read ------------------------------------------------------------------------------------

// open62541: server/check_services_attributes.c ReadSingleAttributeValueWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeValueWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    ASSERT_NE(v.value.GetIf<ua::Int32>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Int32>(), 42);
    EXPECT_FALSE(v.hasSource());
    EXPECT_FALSE(v.hasServer());
}

// open62541: server/check_services_attributes.c ReadSingleAttributeNodeIdWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeNodeIdWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::NodeId)});
    ASSERT_NE(v.value.GetIf<ua::NodeId>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::NodeId>(), kAnswer);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeNodeClassWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeNodeClassWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::NodeClass)});
    ASSERT_NE(v.value.GetIf<ua::Int32>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Int32>(), static_cast<ua::Int32>(ua::NodeClass::Variable));
}

// open62541: server/check_services_attributes.c ReadSingleAttributeBrowseNameWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeBrowseNameWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::BrowseName)});
    ASSERT_NE(v.value.GetIf<ua::QualifiedName>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::QualifiedName>(), ua::QualifiedName(1, "the answer"));
}

// open62541: server/check_services_attributes.c ReadSingleAttributeDisplayNameWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeDisplayNameWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::DisplayName)});
    ASSERT_NE(v.value.GetIf<ua::LocalizedText>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::LocalizedText>(), ua::LocalizedText("en-US", "the answer"));
}

// Nodes have no description text here: the attribute is an empty LocalizedText.
// open62541: server/check_services_attributes.c ReadSingleAttributeDescriptionWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeDescriptionWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::Description)});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    ASSERT_NE(v.value.GetIf<ua::LocalizedText>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::LocalizedText>(), ua::LocalizedText{});
}

// open62541: server/check_services_attributes.c ReadSingleAttributeWriteMaskWithoutTimestamp
// open62541: server/check_services_attributes.c ReadSingleAttributeUserWriteMaskWithoutTimestamp
// open62541: server/check_services_attributes.c ReadSingleAttributeUserWriteMaskViaPublicAPI
TEST_F(Open62541AttributesTest, ReadSingleAttributeWriteMaskWithoutTimestamp) {
    for (A a : {A::WriteMask, A::UserWriteMask}) {
        const DataValue v = read({.node = kAnswer, .attribute = attr(a)});
        ASSERT_NE(v.value.GetIf<ua::UInt32>(), nullptr);
        EXPECT_EQ(*v.value.GetIf<ua::UInt32>(), 0u) << "no attribute but Value is writable";
    }
}

// open62541: server/check_services_attributes.c ReadSingleAttributeIsAbstractWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeIsAbstractWithoutTimestamp) {
    const DataValue v = read({.node = ns0(ua::ns0::Organizes), .attribute = attr(A::IsAbstract)});
    ASSERT_NE(v.value.GetIf<ua::Boolean>(), nullptr);
    EXPECT_FALSE(*v.value.GetIf<ua::Boolean>());
    const DataValue abstractType = read({.node = ns0(ua::ns0::HierarchicalReferences), .attribute = attr(A::IsAbstract)});
    ASSERT_NE(abstractType.value.GetIf<ua::Boolean>(), nullptr);
    EXPECT_TRUE(*abstractType.value.GetIf<ua::Boolean>());
}

// open62541: server/check_services_attributes.c ReadSingleAttributeSymmetricWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeSymmetricWithoutTimestamp) {
    const DataValue v = read({.node = ns0(ua::ns0::Organizes), .attribute = attr(A::Symmetric)});
    ASSERT_NE(v.value.GetIf<ua::Boolean>(), nullptr);
    EXPECT_FALSE(*v.value.GetIf<ua::Boolean>());
    const DataValue references = read({.node = ns0(ua::ns0::References), .attribute = attr(A::Symmetric)});
    ASSERT_NE(references.value.GetIf<ua::Boolean>(), nullptr);
    EXPECT_TRUE(*references.value.GetIf<ua::Boolean>());
}

// open62541: server/check_services_attributes.c ReadSingleAttributeInverseNameWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeInverseNameWithoutTimestamp) {
    const DataValue v = read({.node = ns0(ua::ns0::Organizes), .attribute = attr(A::InverseName)});
    ASSERT_NE(v.value.GetIf<ua::LocalizedText>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::LocalizedText>(), ua::LocalizedText("en-US", "OrganizedBy"));
}

// open62541: server/check_services_attributes.c ReadSingleAttributeEventNotifierWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeEventNotifierWithoutTimestamp) {
    const DataValue v = read({.node = kDevice, .attribute = attr(A::EventNotifier)});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    ASSERT_NE(v.value.GetIf<ua::Byte>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Byte>(), 0);
}

// The variable here is an Int32 (open62541's test variable: BaseDataType).
// open62541: server/check_services_attributes.c ReadSingleAttributeDataTypeWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeDataTypeWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::DataType)});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    ASSERT_NE(v.value.GetIf<ua::NodeId>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::NodeId>(), ns0(ua::ns0::DataType_Int32));
}

// Scalars: ValueRank -1 (open62541's test variable: -2, "any").
// open62541: server/check_services_attributes.c ReadSingleAttributeValueRankWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeValueRankWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::ValueRank)});
    ASSERT_NE(v.value.GetIf<ua::Int32>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Int32>(), -1);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeArrayDimensionsWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeArrayDimensionsWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::ArrayDimensions)});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    ASSERT_TRUE(v.value.IsArray()) << "an (empty) UInt32 array";
    EXPECT_EQ(v.value.TypeId(), ua::BuiltinTypeId::UInt32);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeAccessLevelWithoutTimestamp
// open62541: server/check_services_attributes.c ReadAccessLevel_adminSession_isFull
TEST_F(Open62541AttributesTest, ReadSingleAttributeAccessLevelWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::AccessLevel)});
    ASSERT_NE(v.value.GetIf<ua::Byte>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Byte>(), ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeAccessLevelExWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeAccessLevelExWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::AccessLevelEx)});
    ASSERT_NE(v.value.GetIf<ua::UInt32>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::UInt32>(), ua::UInt32{ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE});
}

// UserAccessLevel: AccessLevel on a secured channel; without CurrentWrite on SecurityPolicy None.
// open62541: server/check_services_attributes.c ReadSingleAttributeUserAccessLevelWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeUserAccessLevelWithoutTimestamp) {
    DataValue v = read({.node = kAnswer, .attribute = attr(A::UserAccessLevel)});
    ASSERT_NE(v.value.GetIf<ua::Byte>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Byte>(), ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE);
    ctx.readOnly = true;
    v = read({.node = kAnswer, .attribute = attr(A::UserAccessLevel)});
    ASSERT_NE(v.value.GetIf<ua::Byte>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Byte>(), ua::ns0::ACCESS_LEVEL_CURRENT_READ);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeMinimumSamplingIntervalWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeMinimumSamplingIntervalWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::MinimumSamplingInterval)});
    ASSERT_NE(v.value.GetIf<ua::Double>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Double>(), ua::MIN_SAMPLING_INTERVAL_MS);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeHistorizingWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleAttributeHistorizingWithoutTimestamp) {
    const DataValue v = read({.node = kAnswer, .attribute = attr(A::Historizing)});
    ASSERT_NE(v.value.GetIf<ua::Boolean>(), nullptr);
    EXPECT_FALSE(*v.value.GetIf<ua::Boolean>());
}

// A variable whose value comes from a callback (open62541: a "data source").
// open62541: server/check_services_attributes.c ReadSingleDataSourceAttributeValueWithoutTimestamp
// open62541: server/check_services_attributes.c ReadSingleAttributeValueWithExternalSource
TEST_F(Open62541AttributesTest, ReadSingleDataSourceAttributeValueWithoutTimestamp) {
    g_temperature = 23.25;
    const DataValue v = read({.node = kTemperature});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    ASSERT_NE(v.value.GetIf<ua::Double>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Double>(), 23.25);
}

// open62541: server/check_services_attributes.c ReadSingleDataSourceAttributeDataTypeWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleDataSourceAttributeDataTypeWithoutTimestamp) {
    const DataValue v = read({.node = kTemperature, .attribute = attr(A::DataType)});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    EXPECT_FALSE(v.hasServer()) << "TimestampsToReturn Neither";
    ASSERT_NE(v.value.GetIf<ua::NodeId>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::NodeId>(), ns0(ua::ns0::DataType_Double));
}

// open62541: server/check_services_attributes.c ReadSingleDataSourceAttributeArrayDimensionsWithoutTimestamp
TEST_F(Open62541AttributesTest, ReadSingleDataSourceAttributeArrayDimensionsWithoutTimestamp) {
    EXPECT_EQ(read({.node = kTemperature, .attribute = attr(A::ArrayDimensions)}).status, ua::StatusCode::Good);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeServerTimestampOnError
TEST_F(Open62541AttributesTest, ReadSingleAttributeServerTimestampOnError) {
    DataValue v = read({.node = kAnswer, .attribute = attr(A::NodeId), .indexRange = ua::String("0:1")}, 2);
    EXPECT_EQ(v.status, ua::StatusCode::BadIndexRangeNoData);
    EXPECT_TRUE(v.hasServer());
    v = read({.node = kAnswer, .dataEncoding = ua::QualifiedName(0, "invalid")}, 2);
    EXPECT_EQ(v.status, ua::StatusCode::BadDataEncodingInvalid);
    EXPECT_TRUE(v.hasServer());
}

// open62541: server/check_services_attributes.c ReadSingleAttributeSourceTimestampOnValueError
TEST_F(Open62541AttributesTest, ReadSingleAttributeSourceTimestampOnValueError) {
    const DataValue v = read({.node = kAnswer, .dataEncoding = ua::QualifiedName(0, "invalid")}, 2);
    EXPECT_EQ(v.status, ua::StatusCode::BadDataEncodingInvalid);
    EXPECT_TRUE(v.hasSource());
}

// open62541: server/check_services_attributes.c ReadSingleAttributeDataTypeDefinitionForSimpleType
TEST_F(Open62541AttributesTest, ReadSingleAttributeDataTypeDefinitionForSimpleType) {
    EXPECT_EQ(read({.node = ns0(ua::ns0::DataType_Int32), .attribute = attr(A::DataTypeDefinition)}).status,
              ua::StatusCode::BadAttributeIdInvalid);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeOutOfRange
TEST_F(Open62541AttributesTest, ReadSingleAttributeOutOfRange) {
    for (ua::UInt32 id : {28u, 0x8000'0000u, 0xFFFF'FFFFu}) {
        const DataValue v = read({.node = ns0(ua::ns0::Server), .attribute = id});
        EXPECT_TRUE(v.mask & 0x02) << "status present";
        EXPECT_EQ(v.status, ua::StatusCode::BadAttributeIdInvalid) << id;
    }
}

// open62541: server/check_services_attributes.c ReadSingleAttributeValue_InvalidNode
// open62541: server/check_services_attributes.c ReadUnknownAttribute_returnsError
// open62541: server/check_services_attributes.c ReadSingleAttributeRange_invalidNode_returnsError
TEST_F(Open62541AttributesTest, ReadSingleAttributeValue_InvalidNode) {
    EXPECT_EQ(read({.node = ua::NodeId(1, 999999)}).status, ua::StatusCode::BadNodeIdUnknown);
    EXPECT_EQ(read({.node = ua::NodeId(1, 99999)}).status, ua::StatusCode::BadNodeIdUnknown);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeInvalidAttributeId_returnsError
TEST_F(Open62541AttributesTest, ReadSingleAttributeInvalidAttributeId_returnsError) {
    EXPECT_EQ(read({.node = kAnswer, .attribute = 0}).status, ua::StatusCode::BadAttributeIdInvalid);
    EXPECT_EQ(read({.node = kAnswer, .attribute = 9999}).status, ua::StatusCode::BadAttributeIdInvalid);
}

// open62541: server/check_services_attributes.c ReadSingleAttributeInverseNameOnNonReference
// open62541: server/check_services_attributes.c ReadSingleAttributeContainsNoLoopsOnNonReference
// open62541: server/check_services_attributes.c ReadSingleAttributeSymmetricOnNonReference
// open62541: server/check_services_attributes.c ReadSingleAttributeIsAbstractOnNonAbstract
TEST_F(Open62541AttributesTest, AttributesOfOtherNodeClassesAreInvalid) {
    for (A a : {A::InverseName, A::ContainsNoLoops, A::Symmetric, A::IsAbstract, A::Executable, A::UserExecutable,
                A::RolePermissions, A::UserRolePermissions, A::AccessRestrictions}) {
        EXPECT_EQ(read({.node = kAnswer, .attribute = attr(a)}).status, ua::StatusCode::BadAttributeIdInvalid)
            << static_cast<ua::UInt32>(a);
    }
    EXPECT_EQ(read({.node = kDevice, .attribute = attr(A::Value)}).status, ua::StatusCode::BadAttributeIdInvalid);
    EXPECT_EQ(read({.node = kDevice, .attribute = attr(A::AccessLevel)}).status, ua::StatusCode::BadAttributeIdInvalid);
}

// A property of an object: the property's BrowseName under HasProperty, then its value.
// open62541: server/check_services_attributes.c ReadObjectProperty
TEST_F(Open62541AttributesTest, ReadObjectProperty) {
    const PathResult path = translate(ns0(ua::ns0::Server), {{.referenceType = ns0(ua::ns0::HasProperty),
                                                               .name = ua::QualifiedName(0, "ServerArray")}});
    ASSERT_EQ(path.status, ua::StatusCode::Good);
    ASSERT_EQ(path.targets.size(), 1u);
    const DataValue v = read({.node = path.targets[0]});
    EXPECT_EQ(v.status, ua::StatusCode::Good);
    EXPECT_TRUE(v.value.IsArray());
    EXPECT_EQ(v.value.TypeId(), ua::BuiltinTypeId::String);
}

// open62541: server/check_services_attributes.c ReadObjectProperty_NotFound
// open62541: server/check_services_attributes.c WriteObjectProperty_NotFound
// open62541: server/check_services_attributes.c WriteObjectProperty_scalar_NotFound_returnsError
TEST_F(Open62541AttributesTest, ReadObjectProperty_NotFound) {
    EXPECT_EQ(translate(ns0(ua::ns0::Server), {{.referenceType = ns0(ua::ns0::HasProperty),
                                                 .name = ua::QualifiedName(0, "NoSuchProperty")}})
                  .status,
              ua::StatusCode::BadNoMatch);
}

// ---- Write -----------------------------------------------------------------------------------

// open62541: server/check_services_attributes.c WriteSingleAttributeValue
// open62541: server/check_services_attributes.c WriteDataValue
TEST_F(Open62541AttributesTest, WriteSingleAttributeValue) {
    EXPECT_EQ(write({.node = kAnswer, .value = ua::Variant::Of(ua::Int32{20})}), ua::StatusCode::Good);
    const DataValue v = read({.node = kAnswer}, 0);
    ASSERT_NE(v.value.GetIf<ua::Int32>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Int32>(), 20);
    EXPECT_TRUE(v.hasSource());
}

// open62541: server/check_services_attributes.c WriteSingleAttributeValueWithServerTimestamp
TEST_F(Open62541AttributesTest, WriteSingleAttributeValueWithServerTimestamp) {
    // the server timestamp is the server's own: a client's one is ignored
    EXPECT_EQ(write({.node = kAnswer, .mask = 0x01 | 0x08, .value = ua::Variant::Of(ua::Int32{20}), .server = 1337}),
              ua::StatusCode::Good);
    const DataValue v = read({.node = kAnswer}, 1);
    ASSERT_NE(v.value.GetIf<ua::Int32>(), nullptr);
    EXPECT_EQ(*v.value.GetIf<ua::Int32>(), 20);
    EXPECT_TRUE(v.hasServer());
    EXPECT_NE(v.server, 1337);
}

// Status and source timestamp need their own AccessLevel bits (StatusWrite, TimestampWrite);
// no variable here has them. A Good status and a zero timestamp count as "not given". (The
// phases where open62541 grants the bits at run time do not exist: AccessLevel is constant.)
// open62541: server/check_services_attributes.c valueStatusAndTimestampNeedSeparatePermission
TEST_F(Open62541AttributesTest, valueStatusAndTimestampNeedSeparatePermission) {
    const ua::Variant v99 = ua::Variant::Of(ua::Int32{99});
    EXPECT_EQ(write({.node = kAnswer, .mask = 0x03, .value = v99, .status = ua::StatusCode::BadOutOfRange}),
              ua::StatusCode::BadWriteNotSupported);
    EXPECT_EQ(write({.node = kAnswer, .mask = 0x05, .value = v99, .source = ua::DATETIME_UNIX_EPOCH + ua::DATETIME_SEC}),
              ua::StatusCode::BadWriteNotSupported);
    EXPECT_EQ(g_answer, 42) << "a rejected write leaves the value";
    EXPECT_EQ(write({.node = kAnswer, .mask = 0x03, .value = v99, .status = ua::StatusCode::Good}), ua::StatusCode::Good);
    EXPECT_EQ(write({.node = kAnswer, .mask = 0x05, .value = v99, .source = 0}), ua::StatusCode::Good);
    EXPECT_EQ(g_answer, 99);
}

// No attribute but Value is writable (WriteMask 0): rejected writes leave every attribute as
// it was. (open62541 grants the WriteMask bits at run time; here the address space is Flash.)
// open62541: server/check_services_attributes.c attributeWritePermissionPreservesRejectedValue
TEST_F(Open62541AttributesTest, attributeWritePermissionPreservesRejectedValue) {
    const std::pair<A, ua::Variant> writes[] = {
        {A::DisplayName, ua::Variant::Of(ua::LocalizedText("locale", "changed text"))},
        {A::Description, ua::Variant::Of(ua::LocalizedText("locale", "changed text"))},
        {A::WriteMask, ua::Variant::Of(ua::UInt32{0x20})},
        {A::DataType, ua::Variant::Of(ns0(ua::ns0::DataType_Int32))},
        {A::ValueRank, ua::Variant::Of(ua::Int32{-1})},
        {A::AccessLevel, ua::Variant::Of(ua::Byte{3})},
        {A::AccessLevelEx, ua::Variant::Of(ua::UInt32{0x23})},
        {A::MinimumSamplingInterval, ua::Variant::Of(ua::Double{250})},
        {A::Historizing, ua::Variant::Of(true)},
    };
    for (const auto& [a, value] : writes) {
        const DataValue before = read({.node = kAnswer, .attribute = attr(a)});
        EXPECT_EQ(write({.node = kAnswer, .attribute = attr(a), .value = value}), ua::StatusCode::BadNotWritable)
            << static_cast<ua::UInt32>(a);
        const DataValue after = read({.node = kAnswer, .attribute = attr(a)});
        EXPECT_EQ(after.value, before.value) << static_cast<ua::UInt32>(a);
    }
}

// NodeId, NodeClass and BrowseName are not writable either (open62541: BadWriteNotSupported;
// here, like every other attribute but Value, BadNotWritable -- WriteMask 0).
// open62541: server/check_services_attributes.c WriteSingleAttributeNodeId
// open62541: server/check_services_attributes.c WriteSingleAttributeNodeclass
// open62541: server/check_services_attributes.c WriteSingleAttributeBrowseName
// open62541: server/check_services_attributes.c WriteSingleAttributeDataType
// open62541: server/check_services_attributes.c WriteSingleAttributeArrayDimensions
TEST_F(Open62541AttributesTest, WriteSingleAttributeNodeId) {
    EXPECT_EQ(write({.node = kAnswer, .attribute = attr(A::NodeId), .value = ua::Variant::Of(ua::NodeId(1, 5))}),
              ua::StatusCode::BadNotWritable);
    EXPECT_EQ(write({.node = kAnswer, .attribute = attr(A::NodeClass), .value = ua::Variant::Of(ua::Int32{1})}),
              ua::StatusCode::BadNotWritable);
    EXPECT_EQ(write({.node = kAnswer, .attribute = attr(A::BrowseName),
                     .value = ua::Variant::Of(ua::QualifiedName(1, "x"))}),
              ua::StatusCode::BadNotWritable);
    EXPECT_EQ(write({.node = kAnswer, .attribute = attr(A::DataType), .value = ua::Variant::Of(ua::NodeId())}),
              ua::StatusCode::BadNotWritable);
    EXPECT_EQ(write({.node = kAnswer, .attribute = attr(A::ArrayDimensions), .value = ua::Variant::Of(ua::UInt32{1})}),
              ua::StatusCode::BadNotWritable);
}

// open62541: server/check_services_attributes.c WriteSingleAttributeIsAbstract
// open62541: server/check_services_attributes.c WriteSingleAttributeSymmetric
// open62541: server/check_services_attributes.c WriteSingleAttributeInverseName
// open62541: server/check_services_attributes.c WriteSingleAttributeContainsNoLoops
// open62541: server/check_services_attributes.c WriteSingleAttributeEventNotifier
// open62541: server/check_services_attributes.c WriteSingleAttributeExecutable
TEST_F(Open62541AttributesTest, WriteAttributesOfOtherNodeClassesIsInvalid) {
    for (A a : {A::IsAbstract, A::Symmetric, A::InverseName, A::ContainsNoLoops, A::EventNotifier, A::Executable}) {
        EXPECT_EQ(write({.node = kAnswer, .attribute = attr(a), .value = ua::Variant::Of(true)}),
                  ua::StatusCode::BadAttributeIdInvalid)
            << static_cast<ua::UInt32>(a);
    }
}

// A read-only variable (open62541: a data source without write callback).
// open62541: server/check_services_attributes.c WriteSingleDataSourceAttributeValue
TEST_F(Open62541AttributesTest, WriteSingleDataSourceAttributeValue) {
    EXPECT_EQ(write({.node = kTemperature, .value = ua::Variant::Of(ua::Double{1})}), ua::StatusCode::BadNotWritable);
    EXPECT_EQ(write({.node = kReadonly, .value = ua::Variant::Of(ua::Int32{1})}), ua::StatusCode::BadNotWritable);
}

// open62541: server/check_services_attributes.c WriteSingleAttributeValue_InvalidNode
TEST_F(Open62541AttributesTest, WriteSingleAttributeValue_InvalidNode) {
    EXPECT_EQ(write({.node = ua::NodeId(1, 99999), .value = ua::Variant::Of(ua::Int32{42})}),
              ua::StatusCode::BadNodeIdUnknown);
}

// ---- rtoos: request checks of Part 4 5.10 --------------------------------------------------

TEST_F(Open62541AttributesTest, ReadRequestChecks) {
    EXPECT_EQ(read_request({}).result, ua::StatusCode::BadNothingToDo);
    EXPECT_EQ(read_request(std::vector<ReadValueId>(ua::MAX_NODES_PER_READ + 1, ReadValueId{.node = kAnswer})).result,
              ua::StatusCode::BadTooManyOperations);
    EXPECT_EQ(read_request({{.node = kAnswer}}, 4).result, ua::StatusCode::BadTimestampsToReturnInvalid);
    EXPECT_EQ(read_request({{.node = kAnswer}}, -1).result, ua::StatusCode::BadTimestampsToReturnInvalid);
    EXPECT_EQ(read_request({{.node = kAnswer}}, 3, -1.0).result, ua::StatusCode::BadMaxAgeInvalid);
    const Response ok = read_request({{.node = kAnswer}}, 3, 1000.0);
    EXPECT_EQ(ok.result, ua::StatusCode::Good);
    EXPECT_EQ(ok.handle, 77u) << "the RequestHandle comes back";
}

TEST_F(Open62541AttributesTest, TimestampsToReturn) {
    struct Case {
        ua::Int32 timestamps;
        bool source, server;
    };
    for (const Case c : {Case{0, true, false}, Case{1, false, true}, Case{2, true, true}, Case{3, false, false}}) {
        const DataValue v = read({.node = kAnswer}, c.timestamps);
        EXPECT_EQ(v.hasSource(), c.source) << c.timestamps;
        EXPECT_EQ(v.hasServer(), c.server) << c.timestamps;
        const DataValue name = read({.node = kAnswer, .attribute = attr(A::BrowseName)}, c.timestamps);
        EXPECT_FALSE(name.hasSource()) << "a source timestamp only for Value";
    }
}

TEST_F(Open62541AttributesTest, IndexRangeAndDataEncoding) {
    EXPECT_EQ(read({.node = kAnswer, .indexRange = ua::String("1")}).status, ua::StatusCode::BadIndexRangeNoData);
    EXPECT_EQ(read({.node = kAnswer, .indexRange = ua::String("2:1")}).status, ua::StatusCode::BadIndexRangeInvalid);
    EXPECT_EQ(read({.node = kAnswer, .indexRange = ua::String("a")}).status, ua::StatusCode::BadIndexRangeInvalid);
    EXPECT_EQ(read({.node = kAnswer, .dataEncoding = ua::QualifiedName(0, "Default Binary")}).status, ua::StatusCode::Good);
    EXPECT_EQ(read({.node = kAnswer, .attribute = attr(A::DisplayName), .dataEncoding = ua::QualifiedName(0, "Default Binary")})
                  .status,
              ua::StatusCode::BadDataEncodingInvalid);
    EXPECT_EQ(write({.node = kAnswer, .indexRange = ua::String("0"), .value = ua::Variant::Of(ua::Int32{1})}),
              ua::StatusCode::BadIndexRangeNoData);
    EXPECT_EQ(write({.node = kAnswer, .indexRange = ua::String("x"), .value = ua::Variant::Of(ua::Int32{1})}),
              ua::StatusCode::BadIndexRangeInvalid);
    EXPECT_TRUE(ua::NumericRangeValid("0:3,1"));
    EXPECT_FALSE(ua::NumericRangeValid("0:3,"));
    EXPECT_FALSE(ua::NumericRangeValid(":3"));
}

TEST_F(Open62541AttributesTest, WriteChecksTheDataType) {
    EXPECT_EQ(write({.node = kAnswer, .value = ua::Variant::Of(ua::UInt32{1})}), ua::StatusCode::BadTypeMismatch);
    EXPECT_EQ(write({.node = kAnswer, .value = ua::Variant::Of(ua::String("42"))}), ua::StatusCode::BadTypeMismatch);
    EXPECT_EQ(write({.node = kAnswer, .mask = 0x00}), ua::StatusCode::BadTypeMismatch) << "no value";
    EXPECT_EQ(g_answer, 42);
    EXPECT_EQ(write_request({}).result, ua::StatusCode::BadNothingToDo);
    EXPECT_EQ(write_request(std::vector<WriteValue>(ua::MAX_NODES_PER_WRITE + 1, WriteValue{.node = kAnswer})).result,
              ua::StatusCode::BadTooManyOperations);
}

// An array value (here skipped by the decoder) is a type mismatch for a scalar variable --
// and the message stays decodable.
TEST_F(Open62541AttributesTest, WriteOfAnArrayIsATypeMismatch) {
    const Response resp = run(
        [&](ua::ByteWriter& w) {
            w.WriteInt32(2);
            for (int i = 0; i < 2; i++) {
                ua::EncodeNodeId(w, kAnswer);
                w.WriteUInt32(13);
                w.WriteString(ua::String());
                w.WriteByte(0x01);
                w.WriteByte(0x80 | 6);  // Int32 array
                w.WriteInt32(3);
                w.WriteInt32(1);
                w.WriteInt32(2);
                w.WriteInt32(3);
            }
        },
        [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) { return ua::HandleWrite(ctx, h, r, w); });
    ua::ByteReader r = resp.reader();
    ua::Int32 count = 0;
    ua::StatusCode a{}, b{};
    ASSERT_TRUE(r.ReadInt32(count) && count == 2 && r.ReadStatusCode(a) && r.ReadStatusCode(b));
    EXPECT_EQ(a, ua::StatusCode::BadTypeMismatch);
    EXPECT_EQ(b, ua::StatusCode::BadTypeMismatch);
}

}  // namespace
