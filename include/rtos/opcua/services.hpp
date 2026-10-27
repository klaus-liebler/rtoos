#pragma once
// Attribute (Read, Write), View (Browse, BrowseNext, TranslateBrowsePathsToNodeIds,
// RegisterNodes, UnregisterNodes) and Discovery (GetEndpoints) services. Each handler gets the
// already decoded RequestHeader (connection.cpp checks the session first) and writes the
// complete response, or a ServiceFault for a request that fails as a whole.
#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/service_structs.hpp"

namespace rtos::opcua {

// Operation limits (Part 4 5.4.x ServerCapabilities.OperationLimits): larger requests are
// answered with BadTooManyOperations.
constexpr Int32 MAX_NODES_PER_READ = 64;
constexpr Int32 MAX_NODES_PER_WRITE = 64;
constexpr Int32 MAX_NODES_PER_BROWSE = 16;
constexpr Int32 MAX_NODES_PER_TRANSLATE = 16;
constexpr Int32 MAX_NODES_PER_REGISTER = 64;
constexpr Int32 MAX_BROWSE_PATH_ELEMENTS = 8;
/// The fastest a variable is sampled (MinimumSamplingInterval attribute, Subscriptions).
constexpr Double MIN_SAMPLING_INTERVAL_MS = 50;

/// What a request may do: the address space, and whether the channel may change it
/// (SecurityPolicy None: read-only, UserAccessLevel without CurrentWrite).
struct ServiceContext {
    const AddressSpace &addressSpace;
    bool readOnly = false;
};

// TimestampsToReturn (Part 4 7.40)
enum class TimestampsToReturn : Int32 { Source = 0, Server = 1, Both = 2, Neither = 3 };

/// Value of one attribute (Part 4 5.10.2 operation level): status and value.
struct AttributeValue {
    StatusCode status = StatusCode::Good;
    Variant value;
};
AttributeValue ReadAttribute(const ServiceContext &ctx, const NodeId &nodeId, UInt32 attributeId);
/// DataValue (Part 6 5.2.2.17) with the timestamps `timestamps` asks for; the source timestamp
/// only for the Value attribute.
bool EncodeDataValue(ByteWriter &w, const AttributeValue &v, TimestampsToReturn timestamps, bool valueAttribute,
                     DateTime sourceTimestamp);
/// NumericRange syntax (Part 4 7.22): "i", "i:j" (i < j), dimensions separated by ','.
bool NumericRangeValid(std::string_view range);

bool HandleGetEndpoints(const RequestHeader &header, ByteReader &requestBody, std::string_view ourEndpointUrl,
                        ByteWriter &w);
bool HandleRead(const ServiceContext &ctx, const RequestHeader &header, ByteReader &requestBody, ByteWriter &w);
bool HandleWrite(const ServiceContext &ctx, const RequestHeader &header, ByteReader &requestBody, ByteWriter &w);

/// Browse continuation points of one session (Part 4 5.8.2): a Browse that has more
/// references than requestedMaxReferencesPerNode returns the first ones and a continuation
/// point; BrowseNext continues. A Browse that needs a point while all are in use reclaims the
/// oldest one (like open62541).
class ViewState {
public:
    static constexpr size_t MAX_CONTINUATION_POINTS = 4;
    struct Continuation {
        bool used = false;
        UInt32 id = 0;
        const Node *node = nullptr;     // the browsed node (address space: static lifetime)
        const Node *refType = nullptr;  // nullptr: every reference type
        bool includeSubtypes = false;
        Int32 direction = 0;
        UInt32 nodeClassMask = 0;
        UInt32 resultMask = 0;
        UInt32 maxReferences = 0;
        UInt32 offset = 0;  // matching references already returned
    };
    Continuation *Allocate();
    Continuation *Find(std::string_view point);
    void Clear() { *this = ViewState{}; }

private:
    Continuation points_[MAX_CONTINUATION_POINTS]{};
    UInt32 nextId_ = 0;
};

bool HandleBrowse(const ServiceContext &ctx, ViewState &view, const RequestHeader &header, ByteReader &requestBody,
                  ByteWriter &w);
bool HandleBrowseNext(const ServiceContext &ctx, ViewState &view, const RequestHeader &header,
                      ByteReader &requestBody, ByteWriter &w);
bool HandleTranslateBrowsePaths(const ServiceContext &ctx, const RequestHeader &header, ByteReader &requestBody,
                                ByteWriter &w);
bool HandleRegisterNodes(const RequestHeader &header, ByteReader &requestBody, ByteWriter &w, bool unregister);

/// Is reference type `type` `base` or (with includeSubtypes) a subtype of it (HasSubtype)?
bool IsReferenceTypeMatch(const AddressSpace &as, const NodeId &type, const NodeId &base, bool includeSubtypes);

}  // namespace rtos::opcua
