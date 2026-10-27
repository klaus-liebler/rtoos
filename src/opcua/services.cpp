#include "rtos/opcua/services.hpp"

#include <array>

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/log.hpp"

namespace rtos::opcua {

namespace {

struct AttributeReadResult {
    StatusCode status = StatusCode::Good;
    Variant value;
};

AttributeReadResult ReadNodeAttribute(const AddressSpace &as, const NodeId &nodeId, UInt32 attributeId) {
    const Node *node = as.FindNode(nodeId);
    if(!node)
        return {StatusCode::BadNodeIdUnknown, {}};

    switch(static_cast<ns0::AttributeId>(attributeId)) {
    case ns0::AttributeId::NodeId:
        return {StatusCode::Good, Variant::Of(node->nodeId)};
    case ns0::AttributeId::NodeClass:
        return {StatusCode::Good, Variant::Of(static_cast<Int32>(node->nodeClass))};
    case ns0::AttributeId::BrowseName:
        return {StatusCode::Good, Variant::Of(node->browseName)};
    case ns0::AttributeId::DisplayName:
        return {StatusCode::Good, Variant::Of(node->displayName)};
    case ns0::AttributeId::Value:
        if(node->nodeClass != NodeClass::Variable)
            return {StatusCode::BadAttributeIdInvalid, {}};
        if(!(node->accessLevel & ns0::ACCESS_LEVEL_CURRENT_READ))
            return {StatusCode::BadNotReadable, {}};
        return {StatusCode::Good, node->ReadValue()};
    case ns0::AttributeId::DataType:
        // HasDataTypeAttribute (address_space.hpp) is the SAME predicate the compile-time table
        // checks use (AllTypedNodesHaveDataType/AllDataTypesResolve) -- deliberately shared, not
        // a second hand-copied `nodeClass == Variable` check: that's exactly how this attribute
        // ended up wrongly gated to Variable-only in the first place (2026-08-19), missing that
        // VariableType nodes (e.g. BaseDataVariableType) have a DataType attribute too.
        if(!HasDataTypeAttribute(node->nodeClass))
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(node->dataType)};
    case ns0::AttributeId::ValueRank:
        if(!HasDataTypeAttribute(node->nodeClass))
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(static_cast<Int32>(-1))}; // scalar-only, see variant.hpp
    case ns0::AttributeId::AccessLevel:
    case ns0::AttributeId::UserAccessLevel:
        if(node->nodeClass != NodeClass::Variable)
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(node->accessLevel)};
    case ns0::AttributeId::Historizing:
        if(node->nodeClass != NodeClass::Variable)
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(false)}; // no history collection support
    case ns0::AttributeId::EventNotifier:
        if(node->nodeClass != NodeClass::Object)
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(static_cast<Byte>(0))}; // no event notification support
    // Description/WriteMask/UserWriteMask (Part 3 5.2.4): base "Node" attributes every
    // NodeClass has, not gated by class at all -- this server tracks no per-node description
    // text or attribute-write permissions, so these are universal, spec-legal defaults (an
    // absent/null Description, and "nothing is attribute-writable") rather than per-node data.
    case ns0::AttributeId::Description:
        return {StatusCode::Good, Variant::Of(LocalizedText{})};
    case ns0::AttributeId::WriteMask:
    case ns0::AttributeId::UserWriteMask:
        return {StatusCode::Good, Variant::Of(static_cast<UInt32>(0))};
    case ns0::AttributeId::IsAbstract:
        // HasIsAbstractAttribute (address_space.hpp) -- same shared-predicate reasoning as
        // HasDataTypeAttribute above.
        if(!HasIsAbstractAttribute(node->nodeClass))
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(node->isAbstract)};
    case ns0::AttributeId::Symmetric:
        if(node->nodeClass != NodeClass::ReferenceType)
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(false)}; // none of this server's ReferenceTypes are symmetric
    case ns0::AttributeId::InverseName:
        if(node->nodeClass != NodeClass::ReferenceType)
            return {StatusCode::BadAttributeIdInvalid, {}};
        return {StatusCode::Good, Variant::Of(node->inverseName)};
    default:
        return {StatusCode::BadAttributeIdInvalid, {}};
    }
}

StatusCode WriteNodeAttribute(const AddressSpace &as, const NodeId &nodeId, UInt32 attributeId, const Variant &value) {
    const Node *node = as.FindNode(nodeId);
    if(!node)
        return StatusCode::BadNodeIdUnknown;
    if(static_cast<ns0::AttributeId>(attributeId) != ns0::AttributeId::Value)
        return StatusCode::BadNotWritable; // only the Value attribute is writable, see services.hpp
    if(node->nodeClass != NodeClass::Variable)
        return StatusCode::BadAttributeIdInvalid;
    if(!(node->accessLevel & ns0::ACCESS_LEVEL_CURRENT_WRITE))
        return StatusCode::BadNotWritable;
    if(value.IsEmpty())
        return StatusCode::BadTypeMismatch;
    // Nodes are immutable data (see address_space.hpp) -- a Variable with the Write access bit
    // set but no WriteCallback is a node-definition bug, not a "write into staticValue" case.
    if(!node->writeCallback)
        return StatusCode::BadNotWritable;

    return node->writeCallback(node->userContext, value) ? StatusCode::Good : StatusCode::BadTypeMismatch;
}

bool EncodeDataValue(ByteWriter &w, StatusCode status, const Variant *value) {
    bool hasValue = value != nullptr && !value->IsEmpty() && IsGood(status);
    Byte mask = 0x02 | 0x08; // StatusCode + ServerTimestamp always present
    if(hasValue) mask |= 0x01;
    if(!w.WriteByte(mask)) return false;
    if(hasValue && !EncodeVariant(w, *value)) return false;
    if(!w.WriteStatusCode(status)) return false;
    return w.WriteDateTime(Now());
}

struct DecodedDataValue {
    Variant value;
    bool hasValue = false;
};

bool DecodeDataValue(ByteReader &r, DecodedDataValue &out) {
    Byte mask = 0;
    if(!r.ReadByte(mask)) return false;
    if(mask & 0x01) {
        if(!DecodeVariant(r, out.value)) return false;
        out.hasValue = true;
    }
    if(mask & 0x02) { StatusCode s{}; if(!r.ReadStatusCode(s)) return false; }
    if(mask & 0x04) { DateTime dt = 0; if(!r.ReadDateTime(dt)) return false; }
    if(mask & 0x08) { DateTime dt = 0; if(!r.ReadDateTime(dt)) return false; }
    if(mask & 0x10) { UInt16 p = 0; if(!r.ReadUInt16(p)) return false; } // SourcePicoseconds
    if(mask & 0x20) { UInt16 p = 0; if(!r.ReadUInt16(p)) return false; } // ServerPicoseconds
    return true;
}

} // namespace

bool HandleGetEndpoints(ByteReader &requestBody, std::string_view ourEndpointUrl, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    String requestedEndpointUrl;
    if(!requestBody.ReadString(requestedEndpointUrl)) return false;
    if(!SkipStringArray(requestBody)) return false; // LocaleIds
    if(!SkipStringArray(requestBody)) return false; // ProfileUris
    if(!requestBody.Ok()) return false;

    // Echo back whatever URL the client actually used to reach us (rather than this server's
    // own fixed, hostless default) -- strict clients (e.g. UAExpert's .NET-based stack) verify
    // the EndpointUrl matches how they connected, and reject the session with
    // BadInvalidArgument if a mismatched/hostless one comes back. Falls back to our default
    // only if the client didn't specify one at all.
    std::string_view endpointUrl = (!requestedEndpointUrl.isNull && !requestedEndpointUrl.value.empty())
        ? requestedEndpointUrl.value : ourEndpointUrl;

    if(!EncodeNodeId(w, NodeId(0, ns0::GetEndpointsResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::Good))) return false;
    return EncodeEndpoints(w, endpointUrl);
}

bool HandleRead(const AddressSpace &as, bool sessionActive, ByteReader &requestBody, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    if(!sessionActive) {
        if(!EncodeNodeId(w, NodeId(0, ns0::ReadResponse))) return false;
        if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::BadSessionNotActivated))) return false;
        if(!w.WriteInt32(-1)) return false;
        return w.WriteInt32(-1);
    }
    Double maxAge = 0;
    Int32 timestampsToReturn = 0;
    if(!requestBody.ReadDouble(maxAge)) return false;
    if(!requestBody.ReadInt32(timestampsToReturn)) return false;
    (void)maxAge; (void)timestampsToReturn; // always returns ServerTimestamp, see EncodeDataValue

    Int32 count = 0;
    if(!requestBody.ReadInt32(count)) return false;
    if(count < 0) count = 0;

    if(!EncodeNodeId(w, NodeId(0, ns0::ReadResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::Good))) return false;
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

        AttributeReadResult result = ReadNodeAttribute(as, nodeId, attributeId);
        log_debug("OPC UA: Read (ns=%u,i=%u) attr=%u -> status=0x%08X", static_cast<unsigned>(nodeId.namespaceIndex),
                 static_cast<unsigned>(nodeId.numeric), static_cast<unsigned>(attributeId),
                 static_cast<unsigned>(result.status));
        if(!EncodeDataValue(w, result.status, IsGood(result.status) ? &result.value : nullptr))
            return false;
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleWrite(const AddressSpace &as, bool sessionActive, ByteReader &requestBody, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    if(!sessionActive) {
        if(!EncodeNodeId(w, NodeId(0, ns0::WriteResponse))) return false;
        if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::BadSessionNotActivated))) return false;
        if(!w.WriteInt32(-1)) return false;
        return w.WriteInt32(-1);
    }

    Int32 count = 0;
    if(!requestBody.ReadInt32(count)) return false;
    if(count < 0) count = 0;

    if(!EncodeNodeId(w, NodeId(0, ns0::WriteResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::Good))) return false;
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

        StatusCode result = value.hasValue
            ? WriteNodeAttribute(as, nodeId, attributeId, value.value)
            : StatusCode::BadTypeMismatch;
        if(!w.WriteStatusCode(result)) return false;
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

bool HandleBrowse(const AddressSpace &as, bool sessionActive, ByteReader &requestBody, ByteWriter &w) {
    RequestHeader reqHeader;
    if(!DecodeRequestHeader(requestBody, reqHeader)) return false;
    if(!sessionActive) {
        if(!EncodeNodeId(w, NodeId(0, ns0::BrowseResponse))) return false;
        if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::BadSessionNotActivated))) return false;
        if(!w.WriteInt32(-1)) return false;
        return w.WriteInt32(-1);
    }

    // ViewDescription: ignored, this server has no Views.
    NodeId viewId;
    DateTime viewTimestamp = 0;
    UInt32 viewVersion = 0;
    if(!DecodeNodeId(requestBody, viewId)) return false;
    if(!requestBody.ReadDateTime(viewTimestamp)) return false;
    if(!requestBody.ReadUInt32(viewVersion)) return false;

    UInt32 requestedMaxPerNode = 0;
    if(!requestBody.ReadUInt32(requestedMaxPerNode)) return false;
    (void)requestedMaxPerNode; // not honored, see services.hpp

    Int32 count = 0;
    if(!requestBody.ReadInt32(count)) return false;
    if(count < 0) count = 0;

    if(!EncodeNodeId(w, NodeId(0, ns0::BrowseResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(reqHeader, StatusCode::Good))) return false;
    if(!w.WriteInt32(count)) return false;

    for(Int32 i = 0; i < count; i++) {
        NodeId nodeId;
        Int32 direction = 0;
        NodeId referenceTypeId;
        Boolean includeSubtypes = false;
        UInt32 nodeClassMask = 0;
        UInt32 resultMask = 0;
        if(!DecodeNodeId(requestBody, nodeId)) return false;
        if(!requestBody.ReadInt32(direction)) return false;
        if(!DecodeNodeId(requestBody, referenceTypeId)) return false;
        if(!requestBody.ReadBoolean(includeSubtypes)) return false;
        if(!requestBody.ReadUInt32(nodeClassMask)) return false;
        if(!requestBody.ReadUInt32(resultMask)) return false;
        if(!requestBody.Ok()) return false;
        // referenceTypeId/includeSubtypes/nodeClassMask filtering not implemented, see
        // services.hpp -- every reference from the node is returned regardless.
        (void)referenceTypeId; (void)includeSubtypes; (void)nodeClassMask; (void)resultMask;

        const Node *sourceNode = as.FindNode(nodeId);
        if(!sourceNode) {
            if(!w.WriteStatusCode(StatusCode::BadNodeIdUnknown)) return false;
            if(!w.WriteByteString(std::string_view{}, true)) return false; // ContinuationPoint
            if(!w.WriteInt32(-1)) return false;                        // References: null
            continue;
        }

        constexpr size_t MAX_BROWSE_RESULT = 64;
        // ~6 KB: static, not on the network thread's stack (one thread serves every client).
        static std::array<ReferenceEntry, MAX_BROWSE_RESULT> collected{};
        size_t collectedCount = 0;
        auto collect = [&](const ReferenceEntry &ref) {
            if(collectedCount < MAX_BROWSE_RESULT) collected[collectedCount++] = ref;
        };
        if(direction == 0 || direction == 2) as.ForEachReference(nodeId, true, collect);
        if(direction == 1 || direction == 2) as.ForEachReference(nodeId, false, collect);

        if(!w.WriteStatusCode(StatusCode::Good)) return false;
        if(!w.WriteByteString(std::string_view{}, true)) return false; // ContinuationPoint = null, see services.hpp
        if(!w.WriteInt32(static_cast<Int32>(collectedCount))) return false;

        for(size_t j = 0; j < collectedCount; j++) {
            const ReferenceEntry &ref = collected[j];
            const Node *targetNode = as.FindNode(ref.targetId);

            if(!EncodeNodeId(w, ref.referenceTypeId)) return false;
            if(!w.WriteBoolean(ref.isForward)) return false;
            if(!EncodeExpandedNodeId(w, ref.targetId)) return false;
            if(targetNode) {
                if(!EncodeQualifiedName(w, targetNode->browseName)) return false;
                if(!EncodeLocalizedText(w, targetNode->displayName)) return false;
                if(!w.WriteInt32(static_cast<Int32>(targetNode->nodeClass))) return false;
            } else {
                if(!EncodeQualifiedName(w, QualifiedName{})) return false;
                if(!EncodeLocalizedText(w, LocalizedText{})) return false;
                if(!w.WriteInt32(0)) return false;
            }

            NodeId typeDefinition; // null by default -> encodes as an absent ExpandedNodeId
            as.ForEachReference(ref.targetId, true, [&](const ReferenceEntry &tdRef) {
                if(tdRef.referenceTypeId == NodeId(0, ns0::HasTypeDefinition))
                    typeDefinition = tdRef.targetId;
            });
            if(!EncodeExpandedNodeId(w, typeDefinition)) return false;
        }
    }
    return w.WriteInt32(-1); // DiagnosticInfos: null array
}

}  // namespace rtos::opcua
