#pragma once
// Well-known Namespace-0 numeric identifiers (Part 6 / Opc.Ua.NodeIds.csv) that this server
// needs to reference: service Request/Response "DefaultBinary" encoding TypeIds (used as the
// leading NodeId of every OPN/MSG body), a few DataType/ObjectType/ReferenceType/well-known
// Object/Variable NodeIds for the minimal address space, and the AttributeId enum. These are
// fixed by the OPC UA specification, not something this server chooses.
#include "rtos/opcua/types.hpp"

namespace rtos::opcua::ns0 {

// --- Service Message TypeIds (the DefaultBinary encoding id of each request/response struct) ---
constexpr UInt32 OpenSecureChannelRequest = 446;
constexpr UInt32 OpenSecureChannelResponse = 449;
constexpr UInt32 CloseSecureChannelRequest = 452;
constexpr UInt32 CloseSecureChannelResponse = 455;
constexpr UInt32 GetEndpointsRequest = 428;
constexpr UInt32 GetEndpointsResponse = 431;
constexpr UInt32 CreateSessionRequest = 461;
constexpr UInt32 CreateSessionResponse = 464;
constexpr UInt32 ActivateSessionRequest = 467;
constexpr UInt32 ActivateSessionResponse = 470;
constexpr UInt32 CloseSessionRequest = 473;
constexpr UInt32 CloseSessionResponse = 476;
constexpr UInt32 ReadRequest = 631;
constexpr UInt32 ReadResponse = 634;
constexpr UInt32 WriteRequest = 673;
constexpr UInt32 WriteResponse = 676;
constexpr UInt32 BrowseRequest = 527;
constexpr UInt32 BrowseResponse = 530;
constexpr UInt32 BrowseNextRequest = 533;
constexpr UInt32 BrowseNextResponse = 536;
constexpr UInt32 TranslateBrowsePathsToNodeIdsRequest = 554;
constexpr UInt32 TranslateBrowsePathsToNodeIdsResponse = 557;
constexpr UInt32 ServiceFault = 397;
// rtoos: View and Subscription services, reference types, data types (Opc.Ua.NodeIds.csv)
constexpr UInt32 RegisterNodesRequest = 560;
constexpr UInt32 RegisterNodesResponse = 563;
constexpr UInt32 UnregisterNodesRequest = 566;
constexpr UInt32 UnregisterNodesResponse = 569;
constexpr UInt32 CreateSubscriptionRequest = 787;
constexpr UInt32 CreateSubscriptionResponse = 790;
constexpr UInt32 ModifySubscriptionRequest = 793;
constexpr UInt32 ModifySubscriptionResponse = 796;
constexpr UInt32 SetPublishingModeRequest = 799;
constexpr UInt32 SetPublishingModeResponse = 802;
constexpr UInt32 DeleteSubscriptionsRequest = 847;
constexpr UInt32 DeleteSubscriptionsResponse = 850;
constexpr UInt32 CreateMonitoredItemsRequest = 751;
constexpr UInt32 CreateMonitoredItemsResponse = 754;
constexpr UInt32 ModifyMonitoredItemsRequest = 763;
constexpr UInt32 ModifyMonitoredItemsResponse = 766;
constexpr UInt32 SetMonitoringModeRequest = 769;
constexpr UInt32 SetMonitoringModeResponse = 772;
constexpr UInt32 DeleteMonitoredItemsRequest = 781;
constexpr UInt32 DeleteMonitoredItemsResponse = 784;
constexpr UInt32 PublishRequest = 826;
constexpr UInt32 PublishResponse = 829;
constexpr UInt32 RepublishRequest = 832;
constexpr UInt32 RepublishResponse = 835;
constexpr UInt32 TransferSubscriptionsRequest = 841;
constexpr UInt32 TransferSubscriptionsResponse = 844;
constexpr UInt32 SetTriggeringRequest = 775;
constexpr UInt32 SetTriggeringResponse = 778;
constexpr UInt32 DataChangeNotification = 811;
constexpr UInt32 StatusChangeNotification = 820;
constexpr UInt32 EventNotificationList = 916;
constexpr UInt32 DataChangeFilter = 724;
constexpr UInt32 EventFilter = 727;
constexpr UInt32 AggregateFilter = 730;
constexpr UInt32 AnonymousIdentityToken = 321;
constexpr UInt32 UserNameIdentityToken = 324;
constexpr UInt32 X509IdentityToken = 327;
constexpr UInt32 IssuedIdentityToken = 940;
constexpr UInt32 NotificationMessage = 805;
constexpr UInt32 MonitoredItemNotification = 808;
constexpr UInt32 NonHierarchicalReferences = 32;
constexpr UInt32 Aggregates = 44;
constexpr UInt32 HasSubtype = 45;
constexpr UInt32 ReferenceTypesFolder = 91;
constexpr UInt32 HasModellingRule = 37;
constexpr UInt32 HasEncoding = 38;
constexpr UInt32 HasEventSource = 36;
constexpr UInt32 HasNotifier = 48;
constexpr UInt32 HasOrderedComponent = 49;
constexpr UInt32 GeneratesEvent = 41;
constexpr UInt32 HasDescription = 39;
constexpr UInt32 Server_ServerStatus_CurrentTime = 2258;
constexpr UInt32 Server_ServerStatus_StartTime = 2257;
constexpr UInt32 DataType_DateTime = 13;
constexpr UInt32 DataType_Double = 11;
constexpr UInt32 DataType_Byte = 3;
constexpr UInt32 DataType_SByte = 2;
constexpr UInt32 DataType_Int64 = 8;
constexpr UInt32 DataType_UInt64 = 9;
constexpr UInt32 DataType_Float = 10;
constexpr UInt32 DataType_ByteString = 15;
constexpr UInt32 DataType_Guid = 14;
constexpr UInt32 DataType_NodeId = 17;
constexpr UInt32 DataType_StatusCode = 19;
constexpr UInt32 DataType_QualifiedName = 20;
constexpr UInt32 DataType_Number = 26;
constexpr UInt32 DataType_Integer = 27;
constexpr UInt32 DataType_UInteger = 28;
constexpr UInt32 FindServersRequest = 422;
constexpr UInt32 FindServersResponse = 425;

// --- Well-known DataType NodeIds (Part 6 5.1.2 -- matches Variant::BuiltinTypeId numerically) ---
constexpr UInt32 DataType_Boolean = 1;
constexpr UInt32 DataType_Int16 = 4;
constexpr UInt32 DataType_UInt16 = 5;
constexpr UInt32 DataType_Int32 = 6;
constexpr UInt32 DataType_UInt32 = 7;
constexpr UInt32 DataType_String = 12;
constexpr UInt32 DataType_LocalizedText = 21;
// The abstract root of the DataType hierarchy (Part 5) -- what a VariableType whose instances
// may hold any concrete type (e.g. BaseDataVariableType) uses as its own DataType attribute.
constexpr UInt32 DataType_BaseDataType = 24;

// --- Well-known ReferenceType NodeIds (Part 3 Table 30, Part 4 releases) ---
constexpr UInt32 References = 31;
constexpr UInt32 HierarchicalReferences = 33;
constexpr UInt32 HasChild = 34;
constexpr UInt32 Organizes = 35;
constexpr UInt32 HasComponent = 47;
constexpr UInt32 HasTypeDefinition = 40;
constexpr UInt32 HasProperty = 46;

// --- Well-known Object/ObjectType/VariableType NodeIds ---
constexpr UInt32 RootFolder = 84;
constexpr UInt32 ObjectsFolder = 85;
constexpr UInt32 TypesFolder = 86;
constexpr UInt32 ViewsFolder = 87;
constexpr UInt32 Server = 2253;
constexpr UInt32 Server_ServerStatus = 2256;
constexpr UInt32 Server_ServerStatus_State = 2259;
// Added 2026-08-19: a real OPC UA client (UAExpert) periodically re-Reads exactly this trio
// (State + these two) as a liveness/"is the server still healthy" watchdog -- confirmed via
// firmware-side request tracing (see project memory). None of this server's minimal test
// address space registered ANY Server-object nodes, so every one of these Reads failed
// (BadNodeIdUnknown), and UAExpert's client stack reacted to the failing watchdog by tearing
// down and reopening the connection every few seconds -- a perpetual reconnect loop that looked
// like a transport/timing bug but was really just a missing mandatory NS0 node.
constexpr UInt32 Server_ServerStatus_SecondsTillShutdown = 2992;
constexpr UInt32 Server_ServerStatus_ShutdownReason = 2993;
// The struct DataType of the ServerStatus Variable itself -- like the trio above, a Variable's
// DataType attribute must never be left as the null NodeId (Node::dataType's default): a real
// client that reads it (as UAExpert does for every Variable node it discovers) tries to resolve
// that NodeId next, and a null one fails with BadNodeIdUnknown just like an unregistered real
// one would.
constexpr UInt32 ServerStatusDataType = 862;
constexpr UInt32 Server_NamespaceArray = 2255;
constexpr UInt32 Server_ServerArray = 2254;
constexpr UInt32 FolderType = 61;
constexpr UInt32 BaseObjectType = 58;
constexpr UInt32 BaseDataVariableType = 63;
constexpr UInt32 BaseVariableType = 62;
constexpr UInt32 PropertyType = 68;
constexpr UInt32 DataType_Structure = 22;
constexpr UInt32 ServerType = 2004;
constexpr UInt32 ServerStatusType = 2138;
constexpr UInt32 ServerState_Running = 0; // ServerState enum value, not a NodeId

// --- UserTokenType (Part 4 7.36.5) enum values (encoded as Int32 in UserTokenPolicy) ---
constexpr Int32 UserTokenType_Anonymous = 0;

// --- Attribute IDs (Part 6 Table 3) ---
enum class AttributeId : UInt32 {
    NodeId = 1,
    NodeClass = 2,
    BrowseName = 3,
    DisplayName = 4,
    Description = 5,
    WriteMask = 6,
    UserWriteMask = 7,
    IsAbstract = 8,
    Symmetric = 9,
    InverseName = 10,
    ContainsNoLoops = 11,
    EventNotifier = 12,
    Value = 13,
    DataType = 14,
    ValueRank = 15,
    ArrayDimensions = 16,
    AccessLevel = 17,
    UserAccessLevel = 18,
    MinimumSamplingInterval = 19,
    Historizing = 20,
    Executable = 21,
    UserExecutable = 22,
    DataTypeDefinition = 23,
    RolePermissions = 24,
    UserRolePermissions = 25,
    AccessRestrictions = 26,
    AccessLevelEx = 27,
};
constexpr UInt32 ATTRIBUTE_ID_MAX = 27;

// Part 3 5.6.2 -- AccessLevel/UserAccessLevel bit mask.
constexpr Byte ACCESS_LEVEL_CURRENT_READ = 0x01;
constexpr Byte ACCESS_LEVEL_CURRENT_WRITE = 0x02;
constexpr Byte ACCESS_LEVEL_STATUS_WRITE = 0x20;
constexpr Byte ACCESS_LEVEL_TIMESTAMP_WRITE = 0x40;

}  // namespace rtos::opcua::ns0
