#include "rtos/opcua/services.hpp"

#include <array>
#include <cstring>

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/log.hpp"

namespace rtos::opcua {

namespace {

constexpr UInt32 MAX_REFERENCES_PER_RESULT = 32; // per BrowseResult; more: continuation point
constexpr size_t MAX_PATH_TARGETS = 8;           // TranslateBrowsePaths: nodes per path step
constexpr int MAX_SUBTYPE_DEPTH = 16;
constexpr std::string_view DEFAULT_BINARY = "Default Binary";
constexpr std::string_view TRANSPORT_PROFILE_URI = "http://opcfoundation.org/UA-Profile/Transport/uatcp-uasc-uabinary";

// The empty ArrayDimensions of a scalar.
constexpr std::array<UInt32, 0> kNoDimensions{};

bool IsType(NodeClass nc) {
    return nc == NodeClass::ObjectType || nc == NodeClass::VariableType || nc == NodeClass::ReferenceType ||
           nc == NodeClass::DataType;
}

// ---- attributes ---------------------------------------------------------------------------------

AttributeValue Good(Variant v) { return {StatusCode::Good, std::move(v)}; }
constexpr AttributeValue kInvalid{StatusCode::BadAttributeIdInvalid, {}};

AttributeValue ReadNodeAttribute(const ServiceContext &ctx, const Node &node, UInt32 attributeId) {
    const NodeClass nc = node.nodeClass;
    const bool variable = nc == NodeClass::Variable;
    switch(static_cast<ns0::AttributeId>(attributeId)) {
    case ns0::AttributeId::NodeId: return Good(Variant::Of(node.nodeId));
    case ns0::AttributeId::NodeClass: return Good(Variant::Of(static_cast<Int32>(nc)));
    case ns0::AttributeId::BrowseName: return Good(Variant::Of(node.browseName));
    case ns0::AttributeId::DisplayName: return Good(Variant::Of(node.displayName));
    // Description/WriteMask/UserWriteMask (Part 3 5.2.4): base "Node" attributes every
    // NodeClass has -- no description text, and no attribute is writable (only Value).
    case ns0::AttributeId::Description: return Good(Variant::Of(LocalizedText{}));
    case ns0::AttributeId::WriteMask:
    case ns0::AttributeId::UserWriteMask: return Good(Variant::Of(static_cast<UInt32>(0)));
    case ns0::AttributeId::IsAbstract:
        // HasIsAbstractAttribute (address_space.hpp): one predicate shared with the table checks
        return HasIsAbstractAttribute(nc) ? Good(Variant::Of(node.isAbstract)) : kInvalid;
    case ns0::AttributeId::Symmetric:
        return nc == NodeClass::ReferenceType ? Good(Variant::Of(node.symmetric)) : kInvalid;
    case ns0::AttributeId::InverseName:
        return nc == NodeClass::ReferenceType ? Good(Variant::Of(node.inverseName)) : kInvalid;
    case ns0::AttributeId::ContainsNoLoops:
        return nc == NodeClass::View ? Good(Variant::Of(true)) : kInvalid;
    case ns0::AttributeId::EventNotifier:
        // no events: the notifier byte is 0 for every Object (and View)
        return (nc == NodeClass::Object || nc == NodeClass::View) ? Good(Variant::Of(static_cast<Byte>(0))) : kInvalid;
    case ns0::AttributeId::Value:
        if(nc == NodeClass::VariableType) return Good(node.ReadValue());
        if(!variable) return kInvalid;
        if(!(node.accessLevel & ns0::ACCESS_LEVEL_CURRENT_READ)) return {StatusCode::BadNotReadable, {}};
        return Good(node.ReadValue());
    case ns0::AttributeId::DataType:
        // HasDataTypeAttribute (address_space.hpp) -- Variable AND VariableType
        return HasDataTypeAttribute(nc) ? Good(Variant::Of(node.dataType)) : kInvalid;
    case ns0::AttributeId::ValueRank:
        return HasDataTypeAttribute(nc) ? Good(Variant::Of(node.valueRank)) : kInvalid;
    case ns0::AttributeId::ArrayDimensions:
        // scalars and one-dimensional arrays of unspecified length: no dimensions
        return HasDataTypeAttribute(nc) ? Good(Variant::Of(UInt32Array{kNoDimensions})) : kInvalid;
    case ns0::AttributeId::AccessLevel: return variable ? Good(Variant::Of(node.accessLevel)) : kInvalid;
    case ns0::AttributeId::UserAccessLevel: {
        if(!variable) return kInvalid;
        // a read-only channel (SecurityPolicy None) may not write, whatever the node allows
        const Byte writeBits = ns0::ACCESS_LEVEL_CURRENT_WRITE | ns0::ACCESS_LEVEL_STATUS_WRITE |
                               ns0::ACCESS_LEVEL_TIMESTAMP_WRITE;
        return Good(Variant::Of(static_cast<Byte>(ctx.readOnly ? node.accessLevel & ~writeBits : node.accessLevel)));
    }
    case ns0::AttributeId::AccessLevelEx:
        return variable ? Good(Variant::Of(static_cast<UInt32>(node.accessLevel))) : kInvalid;
    case ns0::AttributeId::MinimumSamplingInterval:
        return variable ? Good(Variant::Of(MIN_SAMPLING_INTERVAL_MS)) : kInvalid;
    case ns0::AttributeId::Historizing: return variable ? Good(Variant::Of(false)) : kInvalid;
    case ns0::AttributeId::Executable:
    case ns0::AttributeId::UserExecutable:
        return nc == NodeClass::Method ? Good(Variant::Of(false)) : kInvalid;
    // DataTypeDefinition (no structured types are defined here), RolePermissions,
    // UserRolePermissions, AccessRestrictions: optional attributes this server does not have
    default: return kInvalid;
    }
}

StatusCode WriteNodeAttribute(const ServiceContext &ctx, const NodeId &nodeId, UInt32 attributeId,
                              const String &indexRange, const Variant &value, Byte mask, StatusCode status,
                              DateTime sourceTimestamp) {
    const Node *node = ctx.addressSpace.FindNode(nodeId);
    if(!node) return StatusCode::BadNodeIdUnknown;
    if(attributeId == 0 || attributeId > ns0::ATTRIBUTE_ID_MAX) return StatusCode::BadAttributeIdInvalid;
    // An attribute the node class does not have is invalid; one it has is not writable
    // (WriteMask 0) -- only Value is.
    const AttributeValue current = ReadNodeAttribute(ctx, *node, attributeId);
    if(current.status == StatusCode::BadAttributeIdInvalid) return StatusCode::BadAttributeIdInvalid;
    if(static_cast<ns0::AttributeId>(attributeId) != ns0::AttributeId::Value || node->nodeClass != NodeClass::Variable)
        return StatusCode::BadNotWritable;
    if(!(node->accessLevel & ns0::ACCESS_LEVEL_CURRENT_WRITE)) return StatusCode::BadNotWritable;
    if(ctx.readOnly) return StatusCode::BadUserAccessDenied;
    // every writable variable is a scalar: a valid range addresses no data
    if(!indexRange.isNull && !indexRange.value.empty())
        return NumericRangeValid(indexRange.value) ? StatusCode::BadIndexRangeNoData : StatusCode::BadIndexRangeInvalid;
    if(!(mask & 0x01) || value.IsEmpty()) return StatusCode::BadTypeMismatch;
    // Status and source timestamp are written only with their own AccessLevel bits (Part 3
    // 5.6.2); a Good status and a zero timestamp count as "not given". The server timestamp
    // is the server's own and is ignored.
    if((mask & 0x02) && status != StatusCode::Good && !(node->accessLevel & ns0::ACCESS_LEVEL_STATUS_WRITE))
        return StatusCode::BadWriteNotSupported;
    if((mask & 0x04) && sourceTimestamp != 0 && !(node->accessLevel & ns0::ACCESS_LEVEL_TIMESTAMP_WRITE))
        return StatusCode::BadWriteNotSupported;
    // The value's built-in type must be the variable's DataType (BaseDataType: any).
    if(!value.IsSupported() || value.IsArray() || node->valueRank != -1) return StatusCode::BadTypeMismatch;
    if(node->dataType != NodeId(0, ns0::DataType_BaseDataType) && value.DataTypeNodeId() != node->dataType)
        return StatusCode::BadTypeMismatch;
    // Nodes are immutable data (see address_space.hpp) -- a Variable with the Write access bit
    // set but no WriteCallback is a node-definition bug, not a "write into staticValue" case.
    if(!node->writeCallback) return StatusCode::BadNotWritable;
    // the type is right: the application's own check (range, state) refused the value
    return node->writeCallback(node->userContext, value) ? StatusCode::Good : StatusCode::BadOutOfRange;
}

struct DecodedDataValue {
    Variant value;
    Byte mask = 0;
    StatusCode status = StatusCode::Good;
    DateTime sourceTimestamp = 0;
};

bool DecodeDataValue(ByteReader &r, DecodedDataValue &out) {
    if(!r.ReadByte(out.mask)) return false;
    if((out.mask & 0x01) && !DecodeVariant(r, out.value)) return false;
    if((out.mask & 0x02) && !r.ReadStatusCode(out.status)) return false;
    if((out.mask & 0x04) && !r.ReadDateTime(out.sourceTimestamp)) return false;
    if(out.mask & 0x10) { UInt16 p = 0; if(!r.ReadUInt16(p)) return false; } // SourcePicoseconds
    if(out.mask & 0x08) { DateTime dt = 0; if(!r.ReadDateTime(dt)) return false; }
    if(out.mask & 0x20) { UInt16 p = 0; if(!r.ReadUInt16(p)) return false; } // ServerPicoseconds
    return true;
}

// Request size checks shared by every service with an operation list.
StatusCode CheckOperationCount(Int32 count, Int32 max) {
    if(count <= 0) return StatusCode::BadNothingToDo;
    return count > max ? StatusCode::BadTooManyOperations : StatusCode::Good;
}

// ---- view ---------------------------------------------------------------------------------------

constexpr UInt32 RESULT_REFERENCE_TYPE = 0x01, RESULT_IS_FORWARD = 0x02, RESULT_NODE_CLASS = 0x04,
                 RESULT_BROWSE_NAME = 0x08, RESULT_DISPLAY_NAME = 0x10, RESULT_TYPE_DEFINITION = 0x20;

NodeId TypeDefinitionOf(const AddressSpace &as, const NodeId &id) {
    NodeId typeDefinition; // null -> encodes as an absent ExpandedNodeId
    as.ForEachReference(id, true, [&](const ReferenceEntry &r) {
        if(r.referenceTypeId == NodeId(0, ns0::HasTypeDefinition)) typeDefinition = r.targetId;
    });
    return typeDefinition;
}

bool ReferenceMatches(const AddressSpace &as, const ReferenceEntry &ref, const ViewState::Continuation &c) {
    if(c.refType != nullptr && !IsReferenceTypeMatch(as, ref.referenceTypeId, c.refType->nodeId, c.includeSubtypes))
        return false;
    if(c.nodeClassMask != 0) {
        const Node *target = as.FindNode(ref.targetId);
        if(target == nullptr || (static_cast<UInt32>(target->nodeClass) & c.nodeClassMask) == 0) return false;
    }
    return true;
}

// Every reference of c.node that the description selects, forward ones first.
template <class F>
void ForEachMatch(const AddressSpace &as, const ViewState::Continuation &c, F &&fn) {
    const auto visit = [&](bool forward) {
        as.ForEachReference(c.node->nodeId, forward, [&](const ReferenceEntry &r) {
            if(ReferenceMatches(as, r, c)) fn(r);
        });
    };
    if(c.direction == 0 || c.direction == 2) visit(true);
    if(c.direction == 1 || c.direction == 2) visit(false);
}

bool EncodeReferenceDescription(ByteWriter &w, const AddressSpace &as, const ReferenceEntry &ref, UInt32 mask) {
    const Node *target = as.FindNode(ref.targetId);
    if(!EncodeNodeId(w, (mask & RESULT_REFERENCE_TYPE) ? ref.referenceTypeId : NodeId())) return false;
    if(!w.WriteBoolean((mask & RESULT_IS_FORWARD) ? ref.isForward : false)) return false;
    if(!EncodeExpandedNodeId(w, ref.targetId)) return false;
    if(!EncodeQualifiedName(w, (target && (mask & RESULT_BROWSE_NAME)) ? target->browseName : QualifiedName{}))
        return false;
    if(!EncodeLocalizedText(w, (target && (mask & RESULT_DISPLAY_NAME)) ? target->displayName : LocalizedText{}))
        return false;
    if(!w.WriteInt32((target && (mask & RESULT_NODE_CLASS)) ? static_cast<Int32>(target->nodeClass) : 0)) return false;
    return EncodeExpandedNodeId(w, (mask & RESULT_TYPE_DEFINITION) ? TypeDefinitionOf(as, ref.targetId) : NodeId());
}

/// One BrowseResult from c (offset onward); a continuation point if more remain. `fresh`:
/// c is not yet stored in the view state (a new Browse).
bool EncodeBrowseResult(ByteWriter &w, const AddressSpace &as, ViewState &view, ViewState::Continuation c) {
    UInt32 total = 0;
    ForEachMatch(as, c, [&](const ReferenceEntry &) { ++total; });
    const UInt32 remaining = total > c.offset ? total - c.offset : 0;
    const UInt32 limit = c.maxReferences == 0 ? MAX_REFERENCES_PER_RESULT
                                              : std::min(c.maxReferences, MAX_REFERENCES_PER_RESULT);
    const UInt32 count = std::min(remaining, limit);

    ViewState::Continuation *stored = nullptr;
    if(remaining > count) {
        stored = view.Allocate();
        const UInt32 id = stored->id;
        *stored = c;
        stored->used = true;
        stored->id = id;
        stored->offset = c.offset + count;
    }
    if(!w.WriteStatusCode(StatusCode::Good)) return false;
    if(stored != nullptr) {
        Byte point[4];
        std::memcpy(point, &stored->id, 4);
        if(!w.WriteByteString(std::string_view(reinterpret_cast<const char *>(point), 4), false)) return false;
    } else if(!w.WriteByteString(std::string_view{}, true)) {
        return false;
    }
    if(!w.WriteInt32(static_cast<Int32>(count))) return false;
    UInt32 index = 0;
    bool ok = true;
    ForEachMatch(as, c, [&](const ReferenceEntry &r) {
        if(ok && index >= c.offset && index < c.offset + count) ok = EncodeReferenceDescription(w, as, r, c.resultMask);
        ++index;
    });
    return ok;
}

bool EncodeBrowseError(ByteWriter &w, StatusCode status) {
    return w.WriteStatusCode(status) && w.WriteByteString(std::string_view{}, true) && w.WriteInt32(-1);
}

} // namespace

// ---- public helpers --------------------------------------------------------------------------------

AttributeValue ReadAttribute(const ServiceContext &ctx, const NodeId &nodeId, UInt32 attributeId) {
    const Node *node = ctx.addressSpace.FindNode(nodeId);
    if(!node) return {StatusCode::BadNodeIdUnknown, {}};
    if(attributeId == 0 || attributeId > ns0::ATTRIBUTE_ID_MAX) return {StatusCode::BadAttributeIdInvalid, {}};
    return ReadNodeAttribute(ctx, *node, attributeId);
}

bool EncodeDataValue(ByteWriter &w, const AttributeValue &v, TimestampsToReturn timestamps, bool valueAttribute,
                     DateTime sourceTimestamp) {
    const bool hasValue = !v.value.IsEmpty() && IsGood(v.status);
    const bool source = valueAttribute && (timestamps == TimestampsToReturn::Source || timestamps == TimestampsToReturn::Both);
    const bool server = timestamps == TimestampsToReturn::Server || timestamps == TimestampsToReturn::Both;
    Byte mask = 0;
    if(hasValue) mask |= 0x01;
    if(v.status != StatusCode::Good) mask |= 0x02;
    if(source) mask |= 0x04;
    if(server) mask |= 0x08;
    if(!w.WriteByte(mask)) return false;
    if(hasValue && !EncodeVariant(w, v.value)) return false;
    if((mask & 0x02) && !w.WriteStatusCode(v.status)) return false;
    if(source && !w.WriteDateTime(sourceTimestamp)) return false;
    return !server || w.WriteDateTime(Now());
}

bool NumericRangeValid(std::string_view range) {
    if(range.empty()) return false;
    size_t pos = 0;
    while(pos <= range.size()) {
        const size_t end = std::min(range.find(',', pos), range.size());
        const std::string_view dim = range.substr(pos, end - pos);
        const size_t colon = dim.find(':');
        const auto number = [](std::string_view s, UInt32 &out) {
            if(s.empty() || s.size() > 9) return false;
            out = 0;
            for(char ch : s) {
                if(ch < '0' || ch > '9') return false;
                out = out * 10 + static_cast<UInt32>(ch - '0');
            }
            return true;
        };
        UInt32 a = 0, b = 0;
        if(colon == std::string_view::npos) {
            if(!number(dim, a)) return false;
        } else if(!number(dim.substr(0, colon), a) || !number(dim.substr(colon + 1), b) || a >= b) {
            return false;
        }
        if(end == range.size()) return true;
        pos = end + 1;
    }
    return false;
}

bool IsReferenceTypeMatch(const AddressSpace &as, const NodeId &type, const NodeId &base, bool includeSubtypes) {
    if(type == base) return true;
    if(!includeSubtypes) return false;
    NodeId current = type;
    for(int depth = 0; depth < MAX_SUBTYPE_DEPTH; depth++) {
        NodeId super;
        as.ForEachReference(current, false, [&](const ReferenceEntry &r) {
            if(r.referenceTypeId == NodeId(0, ns0::HasSubtype)) super = r.targetId;
        });
        if(super.IsNull()) return false;
        if(super == base) return true;
        current = super;
    }
    return false;
}

ViewState::Continuation *ViewState::Allocate() {
    Continuation *slot = nullptr;
    for(Continuation &c : points_) {
        if(!c.used) { slot = &c; break; }
        if(slot == nullptr || c.id < slot->id) slot = &c; // oldest (ids increase)
    }
    *slot = Continuation{};
    slot->id = ++nextId_;
    return slot;
}

ViewState::Continuation *ViewState::Find(std::string_view point) {
    if(point.size() != 4) return nullptr;
    UInt32 id = 0;
    std::memcpy(&id, point.data(), 4);
    for(Continuation &c : points_)
        if(c.used && c.id == id) return &c;
    return nullptr;
}

// ---- services ---------------------------------------------------------------------------------------

bool HandleGetEndpoints(const RequestHeader &header, ByteReader &requestBody, std::string_view ourEndpointUrl,
                        ByteWriter &w) {
    String requestedEndpointUrl;
    if(!requestBody.ReadString(requestedEndpointUrl)) return false;
    if(!SkipStringArray(requestBody)) return false; // LocaleIds
    // ProfileUris: when given, only endpoints with one of these transport profiles
    Int32 profiles = 0;
    bool profileMatch = false;
    if(!requestBody.ReadInt32(profiles)) return false;
    for(Int32 i = 0; i < profiles; i++) {
        String uri;
        if(!requestBody.ReadString(uri)) return false;
        profileMatch = profileMatch || (!uri.isNull && uri.value == TRANSPORT_PROFILE_URI);
    }
    if(!requestBody.Ok()) return false;

    // Echo back whatever URL the client actually used to reach us (rather than this server's
    // own fixed, hostless default) -- strict clients (e.g. UAExpert's .NET-based stack) verify
    // the EndpointUrl matches how they connected, and reject the session with
    // BadInvalidArgument if a mismatched/hostless one comes back. Falls back to our default
    // only if the client didn't specify one at all.
    std::string_view endpointUrl = (!requestedEndpointUrl.isNull && !requestedEndpointUrl.value.empty())
        ? requestedEndpointUrl.value : ourEndpointUrl;

    if(!EncodeNodeId(w, NodeId(0, ns0::GetEndpointsResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(profiles > 0 && !profileMatch) return w.WriteInt32(0);
    return EncodeEndpoints(w, endpointUrl);
}

bool HandleRead(const ServiceContext &ctx, const RequestHeader &header, ByteReader &requestBody, ByteWriter &w) {
    Double maxAge = 0;
    Int32 timestampsToReturn = 0;
    Int32 count = 0;
    if(!requestBody.ReadDouble(maxAge)) return false;
    if(!requestBody.ReadInt32(timestampsToReturn)) return false;
    if(!requestBody.ReadInt32(count)) return false;
    // maxAge: every value is read from its source anyway (no cache) -- only its sign matters
    if(maxAge < 0) return EncodeServiceFault(w, header, StatusCode::BadMaxAgeInvalid);
    if(timestampsToReturn < 0 || timestampsToReturn > 3)
        return EncodeServiceFault(w, header, StatusCode::BadTimestampsToReturnInvalid);
    if(const StatusCode s = CheckOperationCount(count, MAX_NODES_PER_READ); !IsGood(s))
        return EncodeServiceFault(w, header, s);
    const auto timestamps = static_cast<TimestampsToReturn>(timestampsToReturn);

    if(!EncodeNodeId(w, NodeId(0, ns0::ReadResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!w.WriteInt32(count)) return false;

    for(Int32 i = 0; i < count; i++) {
        NodeId nodeId;
        UInt32 attributeId = 0;
        String indexRange;
        QualifiedName dataEncoding;
        if(!DecodeNodeId(requestBody, nodeId)) return false;
        if(!requestBody.ReadUInt32(attributeId)) return false;
        if(!requestBody.ReadString(indexRange)) return false;
        if(!DecodeQualifiedName(requestBody, dataEncoding)) return false;
        if(!requestBody.Ok()) return false;

        const bool valueAttribute = attributeId == static_cast<UInt32>(ns0::AttributeId::Value);
        AttributeValue result = ReadAttribute(ctx, nodeId, attributeId);
        if(IsGood(result.status) && !dataEncoding.name.empty()) {
            // DataEncoding applies to the Value of structured types only; for built-in types
            // only the default encoding is "selected"
            if(!valueAttribute || dataEncoding.namespaceIndex != 0 || dataEncoding.name != DEFAULT_BINARY)
                result = {StatusCode::BadDataEncodingInvalid, {}};
        }
        if(IsGood(result.status) && !indexRange.isNull && !indexRange.value.empty()) {
            // sub-ranges are not supported (application variables are scalars; the few arrays
            // of ns 0 are read whole): a valid range finds no data
            result = {NumericRangeValid(indexRange.value) ? StatusCode::BadIndexRangeNoData
                                                          : StatusCode::BadIndexRangeInvalid,
                      {}};
        }
        log_debug("OPC UA: Read (ns=%u,i=%u) attr=%u -> status=0x%08X", static_cast<unsigned>(nodeId.namespaceIndex),
                 static_cast<unsigned>(nodeId.numeric), static_cast<unsigned>(attributeId),
                 static_cast<unsigned>(result.status));
        if(!EncodeDataValue(w, result, timestamps, valueAttribute, Now())) return false;
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleWrite(const ServiceContext &ctx, const RequestHeader &header, ByteReader &requestBody, ByteWriter &w) {
    Int32 count = 0;
    if(!requestBody.ReadInt32(count)) return false;
    if(const StatusCode s = CheckOperationCount(count, MAX_NODES_PER_WRITE); !IsGood(s))
        return EncodeServiceFault(w, header, s);

    if(!EncodeNodeId(w, NodeId(0, ns0::WriteResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!w.WriteInt32(count)) return false;

    for(Int32 i = 0; i < count; i++) {
        NodeId nodeId;
        UInt32 attributeId = 0;
        String indexRange;
        DecodedDataValue value;
        if(!DecodeNodeId(requestBody, nodeId)) return false;
        if(!requestBody.ReadUInt32(attributeId)) return false;
        if(!requestBody.ReadString(indexRange)) return false;
        if(!DecodeDataValue(requestBody, value)) return false;
        if(!requestBody.Ok()) return false;
        const StatusCode result = WriteNodeAttribute(ctx, nodeId, attributeId, indexRange, value.value, value.mask,
                                                     value.status, value.sourceTimestamp);
        if(!w.WriteStatusCode(result)) return false;
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleBrowse(const ServiceContext &ctx, ViewState &view, const RequestHeader &header, ByteReader &requestBody,
                  ByteWriter &w) {
    const AddressSpace &as = ctx.addressSpace;
    // ViewDescription: this server has no Views -- a non-null ViewId is unknown.
    NodeId viewId;
    DateTime viewTimestamp = 0;
    UInt32 viewVersion = 0;
    UInt32 requestedMaxPerNode = 0;
    Int32 count = 0;
    if(!DecodeNodeId(requestBody, viewId)) return false;
    if(!requestBody.ReadDateTime(viewTimestamp)) return false;
    if(!requestBody.ReadUInt32(viewVersion)) return false;
    if(!requestBody.ReadUInt32(requestedMaxPerNode)) return false;
    if(!requestBody.ReadInt32(count)) return false;
    if(!viewId.IsNull()) return EncodeServiceFault(w, header, StatusCode::BadViewIdUnknown);
    if(const StatusCode s = CheckOperationCount(count, MAX_NODES_PER_BROWSE); !IsGood(s))
        return EncodeServiceFault(w, header, s);

    if(!EncodeNodeId(w, NodeId(0, ns0::BrowseResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!w.WriteInt32(count)) return false;

    for(Int32 i = 0; i < count; i++) {
        NodeId nodeId, referenceTypeId;
        Int32 direction = 0;
        Boolean includeSubtypes = false;
        UInt32 nodeClassMask = 0, resultMask = 0;
        if(!DecodeNodeId(requestBody, nodeId)) return false;
        if(!requestBody.ReadInt32(direction)) return false;
        if(!DecodeNodeId(requestBody, referenceTypeId)) return false;
        if(!requestBody.ReadBoolean(includeSubtypes)) return false;
        if(!requestBody.ReadUInt32(nodeClassMask)) return false;
        if(!requestBody.ReadUInt32(resultMask)) return false;
        if(!requestBody.Ok()) return false;

        ViewState::Continuation c;
        c.node = as.FindNode(nodeId);
        c.refType = referenceTypeId.IsNull() ? nullptr : as.FindNode(referenceTypeId);
        c.includeSubtypes = includeSubtypes;
        c.direction = direction;
        c.nodeClassMask = nodeClassMask;
        c.resultMask = resultMask;
        c.maxReferences = requestedMaxPerNode;
        StatusCode status = StatusCode::Good;
        if(direction < 0 || direction > 2) status = StatusCode::BadBrowseDirectionInvalid;
        else if(c.node == nullptr) status = StatusCode::BadNodeIdUnknown;
        else if(!referenceTypeId.IsNull() && (c.refType == nullptr || c.refType->nodeClass != NodeClass::ReferenceType))
            status = StatusCode::BadReferenceTypeIdInvalid;
        if(!(IsGood(status) ? EncodeBrowseResult(w, as, view, c) : EncodeBrowseError(w, status))) return false;
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleBrowseNext(const ServiceContext &ctx, ViewState &view, const RequestHeader &header,
                      ByteReader &requestBody, ByteWriter &w) {
    Boolean release = false;
    Int32 count = 0;
    if(!requestBody.ReadBoolean(release)) return false;
    if(!requestBody.ReadInt32(count)) return false;
    if(const StatusCode s = CheckOperationCount(count, MAX_NODES_PER_BROWSE); !IsGood(s))
        return EncodeServiceFault(w, header, s);

    if(!EncodeNodeId(w, NodeId(0, ns0::BrowseNextResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!w.WriteInt32(count)) return false;
    for(Int32 i = 0; i < count; i++) {
        std::string_view point;
        bool isNull = true;
        if(!requestBody.ReadByteString(point, isNull)) return false;
        ViewState::Continuation *c = view.Find(point);
        if(c == nullptr) {
            if(!EncodeBrowseError(w, StatusCode::BadContinuationPointInvalid)) return false;
            continue;
        }
        const ViewState::Continuation copy = *c;
        c->used = false; // a further point (if any) is allocated anew
        if(release) {
            // releasing: no references, just the status (Part 4 5.8.3.2)
            if(!w.WriteStatusCode(StatusCode::Good) || !w.WriteByteString(std::string_view{}, true) || !w.WriteInt32(-1))
                return false;
            continue;
        }
        if(!EncodeBrowseResult(w, ctx.addressSpace, view, copy)) return false;
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleTranslateBrowsePaths(const ServiceContext &ctx, const RequestHeader &header, ByteReader &requestBody,
                                ByteWriter &w) {
    const AddressSpace &as = ctx.addressSpace;
    Int32 count = 0;
    if(!requestBody.ReadInt32(count)) return false;
    if(const StatusCode s = CheckOperationCount(count, MAX_NODES_PER_TRANSLATE); !IsGood(s))
        return EncodeServiceFault(w, header, s);

    if(!EncodeNodeId(w, NodeId(0, ns0::TranslateBrowsePathsToNodeIdsResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(!w.WriteInt32(count)) return false;

    for(Int32 i = 0; i < count; i++) {
        NodeId start;
        Int32 elements = 0;
        if(!DecodeNodeId(requestBody, start)) return false;
        if(!requestBody.ReadInt32(elements)) return false;

        // breadth first, one path element at a time; the node sets live in the address space
        std::array<const Node *, MAX_PATH_TARGETS> current{}, next{};
        size_t currentCount = 0;
        StatusCode status = StatusCode::Good;
        if(const Node *s = as.FindNode(start)) current[currentCount++] = s;
        else status = StatusCode::BadNodeIdUnknown;
        if(IsGood(status) && elements <= 0) status = StatusCode::BadNothingToDo;
        if(IsGood(status) && elements > MAX_BROWSE_PATH_ELEMENTS) status = StatusCode::BadTooManyOperations;

        for(Int32 e = 0; e < elements; e++) {
            NodeId referenceTypeId;
            Boolean isInverse = false, includeSubtypes = false;
            QualifiedName targetName;
            if(!DecodeNodeId(requestBody, referenceTypeId)) return false;
            if(!requestBody.ReadBoolean(isInverse)) return false;
            if(!requestBody.ReadBoolean(includeSubtypes)) return false;
            if(!DecodeQualifiedName(requestBody, targetName)) return false;
            if(!IsGood(status)) continue; // keep decoding the rest of the path
            if(targetName.name.empty()) { status = StatusCode::BadBrowseNameInvalid; continue; }
            const Node *refType = referenceTypeId.IsNull() ? nullptr : as.FindNode(referenceTypeId);
            if(!referenceTypeId.IsNull() && (refType == nullptr || refType->nodeClass != NodeClass::ReferenceType)) {
                status = StatusCode::BadReferenceTypeIdInvalid;
                continue;
            }
            size_t nextCount = 0;
            for(size_t n = 0; n < currentCount; n++) {
                as.ForEachReference(current[n]->nodeId, !isInverse, [&](const ReferenceEntry &r) {
                    if(refType != nullptr && !IsReferenceTypeMatch(as, r.referenceTypeId, refType->nodeId, includeSubtypes))
                        return;
                    const Node *target = as.FindNode(r.targetId);
                    if(target == nullptr || !(target->browseName == targetName) || nextCount == next.size()) return;
                    for(size_t k = 0; k < nextCount; k++)
                        if(next[k] == target) return;
                    next[nextCount++] = target;
                });
            }
            if(nextCount == 0) status = StatusCode::BadNoMatch;
            current = next;
            currentCount = nextCount;
        }
        if(!requestBody.Ok()) return false;

        if(!w.WriteStatusCode(status)) return false;
        if(!IsGood(status)) {
            if(!w.WriteInt32(-1)) return false;
            continue;
        }
        if(!w.WriteInt32(static_cast<Int32>(currentCount))) return false;
        for(size_t n = 0; n < currentCount; n++) {
            if(!EncodeExpandedNodeId(w, current[n]->nodeId)) return false;
            if(!w.WriteUInt32(0xFFFF'FFFFu)) return false; // RemainingPathIndex: the whole path matched
        }
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleRegisterNodes(const RequestHeader &header, ByteReader &requestBody, ByteWriter &w, bool unregister) {
    // Every node is always "registered": the NodeIds are returned as they are (Part 4 5.8.5).
    Int32 count = 0;
    if(!requestBody.ReadInt32(count)) return false;
    if(const StatusCode s = CheckOperationCount(count, MAX_NODES_PER_REGISTER); !IsGood(s))
        return EncodeServiceFault(w, header, s);
    if(!EncodeNodeId(w, NodeId(0, unregister ? ns0::UnregisterNodesResponse : ns0::RegisterNodesResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(unregister) {
        for(Int32 i = 0; i < count; i++) {
            NodeId id;
            if(!DecodeNodeId(requestBody, id)) return false;
        }
        return requestBody.Ok();
    }
    if(!w.WriteInt32(count)) return false;
    for(Int32 i = 0; i < count; i++) {
        NodeId id;
        if(!DecodeNodeId(requestBody, id) || !EncodeNodeId(w, id)) return false;
    }
    return requestBody.Ok();
}

}  // namespace rtos::opcua
