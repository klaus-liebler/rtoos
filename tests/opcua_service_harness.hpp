// SPDX-License-Identifier: MIT
//
// Calls the OPC UA service handlers directly (no TCP, no SecureChannel), like open62541's
// service tests call Service_*(): encode a request body, run the handler, decode the response.
// The tests ported from open62541 (tests/opcua_o6_*_test.cpp) use it; each ported test
// carries a "// open62541: <file> <test>" marker that tools/open62541_catalog.py collects.
//
// The address space mirrors the variables of open62541's test servers that matter here:
//   Objects/the.answer   ns=1;s=the.answer  Int32 42, read + write (own storage)
//   Objects/readonly     ns=1;s=readonly    Int32 7, read only
//   Objects/Device       ns=1;i=1000        Object with Temperature (Double, ns=1;i=1001)
//   Objects/"\0", "\0\0", "\0\0\0"  ns=1;i=2001..2003  names that are prefixes of each other
#pragma once

#include <gtest/gtest.h>

#include <array>
#include <functional>
#include <vector>

#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/ns0_base.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/services.hpp"

namespace harness {

namespace ua = rtos::opcua;
using ua::Byte;
using Bytes = std::vector<Byte>;

inline ua::Int32 g_answer = 42;
inline ua::Double g_temperature = 21.5;

inline ua::Variant read_answer(void*) { return ua::Variant::Of(g_answer); }
inline bool write_answer(void*, const ua::Variant& v) {
    g_answer = *v.GetIf<ua::Int32>();  // the server checked the type
    return true;
}
inline ua::Variant read_temperature(void*) { return ua::Variant::Of(g_temperature); }

inline constexpr ua::NodeId kAnswer = ua::NodeId::Str(1, "the.answer");
inline constexpr ua::NodeId kReadonly = ua::NodeId::Str(1, "readonly");
inline constexpr ua::NodeId kDevice(1, 1000), kTemperature(1, 1001);
inline constexpr ua::NodeId kZero1(1, 2001), kZero2(1, 2002), kZero3(1, 2003);
inline constexpr char kZeros[3] = {0, 0, 0};

inline constexpr std::array kAppNodes = {
    ua::Node{.nodeId = kAnswer, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "the answer"),
             .displayName = ua::LocalizedText("en-US", "the answer"), .dataType = ua::NodeId(0, ua::ns0::DataType_Int32),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE,
             .readCallback = &read_answer, .writeCallback = &write_answer},
    ua::Node{.nodeId = kReadonly, .nodeClass = ua::NodeClass::Variable, .browseName = ua::QualifiedName(1, "readonly"),
             .displayName = ua::LocalizedText("en-US", "readonly"), .dataType = ua::NodeId(0, ua::ns0::DataType_Int32),
             .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ, .staticValue = ua::Variant::Of(ua::Int32{7})},
    ua::Node{.nodeId = kDevice, .nodeClass = ua::NodeClass::Object, .browseName = ua::QualifiedName(1, "Device"),
             .displayName = ua::LocalizedText("en-US", "Device")},
    ua::Node{.nodeId = kTemperature, .nodeClass = ua::NodeClass::Variable,
             .browseName = ua::QualifiedName(1, "Temperature"), .displayName = ua::LocalizedText("en-US", "Temperature"),
             .dataType = ua::NodeId(0, ua::ns0::DataType_Double), .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ,
             .readCallback = &read_temperature},
    ua::Node{.nodeId = kZero1, .nodeClass = ua::NodeClass::Object, .browseName = ua::QualifiedName(0, {kZeros, 1}),
             .displayName = ua::LocalizedText("", "zero1")},
    ua::Node{.nodeId = kZero2, .nodeClass = ua::NodeClass::Object, .browseName = ua::QualifiedName(0, {kZeros, 2}),
             .displayName = ua::LocalizedText("", "zero2")},
    ua::Node{.nodeId = kZero3, .nodeClass = ua::NodeClass::Object, .browseName = ua::QualifiedName(0, {kZeros, 3}),
             .displayName = ua::LocalizedText("", "zero3")},
};
inline constexpr std::array kAppReferences = {
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kAnswer},
    ua::ReferenceEntry{kAnswer, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{kAnswer, ua::NodeId(0, ua::ns0::HasTypeDefinition), true,
                       ua::NodeId(0, ua::ns0::BaseDataVariableType)},
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kReadonly},
    ua::ReferenceEntry{kReadonly, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kDevice},
    ua::ReferenceEntry{kDevice, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{kDevice, ua::NodeId(0, ua::ns0::HasTypeDefinition), true, ua::NodeId(0, ua::ns0::BaseObjectType)},
    ua::ReferenceEntry{kDevice, ua::NodeId(0, ua::ns0::HasComponent), true, kTemperature},
    ua::ReferenceEntry{kTemperature, ua::NodeId(0, ua::ns0::HasComponent), false, kDevice},
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kZero1},
    ua::ReferenceEntry{kZero1, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kZero2},
    ua::ReferenceEntry{kZero2, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
    ua::ReferenceEntry{ua::NodeId(0, ua::ns0::ObjectsFolder), ua::NodeId(0, ua::ns0::Organizes), true, kZero3},
    ua::ReferenceEntry{kZero3, ua::NodeId(0, ua::ns0::Organizes), false, ua::NodeId(0, ua::ns0::ObjectsFolder)},
};
inline constexpr auto kNodes = ua::JoinNodes(ua::kBaseNodes, kAppNodes);
inline constexpr auto kReferences = ua::JoinReferences(ua::kBaseReferences, kAppReferences);
static_assert(ua::AllTypedNodesHaveDataType(kNodes));
static_assert(ua::AllDataTypesResolve(kNodes));
static_assert(ua::AllReferenceTypesHaveInverseName(kNodes));
static_assert(ua::AllReferencesResolve(kNodes, kReferences));
inline constexpr ua::AddressSpace kSpace(kNodes, kReferences);

/// A decoded response: TypeId, ResponseHeader fields, and a reader at the body.
struct Response {
    Bytes bytes;
    ua::NodeId type;
    ua::UInt32 handle = 0;
    ua::StatusCode result = ua::StatusCode::BadInternalError;
    std::size_t body = 0;
    ua::ByteReader reader() const { return ua::ByteReader(std::span(bytes).subspan(body)); }
    bool fault() const { return type == ua::NodeId(0, ua::ns0::ServiceFault); }
};

inline Response parse(std::span<const Byte> written) {
    Response out;
    out.bytes.assign(written.begin(), written.end());
    ua::ByteReader r(out.bytes);
    ua::DateTime ts = 0;
    Byte diagnostics = 0;
    ua::Int32 strings = 0;
    EXPECT_TRUE(ua::DecodeNodeId(r, out.type) && r.ReadDateTime(ts) && r.ReadUInt32(out.handle) &&
                r.ReadStatusCode(out.result) && r.ReadByte(diagnostics) && r.ReadInt32(strings) &&
                ua::DecodeAndDiscardExtensionObject(r));
    out.body = r.Position();
    return out;
}

/// ReferenceDescription (Part 4 7.30)
struct Reference {
    ua::NodeId referenceType{};
    bool forward = false;
    ua::NodeId target;
    ua::QualifiedName browseName;
    ua::LocalizedText displayName;
    ua::Int32 nodeClass = 0;
    ua::NodeId typeDefinition;
};
/// BrowseResult (Part 4 7.6)
struct BrowseResult {
    ua::StatusCode status = ua::StatusCode::BadInternalError;
    Bytes continuation;
    bool continuationNull = true;
    std::vector<Reference> references;
};
/// BrowsePathResult (Part 4 7.7)
struct PathResult {
    ua::StatusCode status = ua::StatusCode::BadInternalError;
    std::vector<ua::NodeId> targets;
};
/// DataValue (Part 6 5.2.2.17)
struct DataValue {
    Byte mask = 0;
    ua::Variant value;
    ua::StatusCode status = ua::StatusCode::Good;
    ua::DateTime source = 0, server = 0;
    bool hasValue() const { return mask & 0x01; }
    bool hasSource() const { return mask & 0x04; }
    bool hasServer() const { return mask & 0x08; }
};

struct BrowseDescription {
    ua::NodeId node{};
    ua::Int32 direction = 0;  // forward
    ua::NodeId referenceType{};
    bool includeSubtypes = false;
    ua::UInt32 nodeClassMask = 0;
    ua::UInt32 resultMask = 0x3F;  // all
};

struct PathElement {
    ua::NodeId referenceType{};
    bool inverse = false;
    bool includeSubtypes = false;
    ua::QualifiedName name{};
};

struct ReadValueId {
    ua::NodeId node{};
    ua::UInt32 attribute = 13;  // Value
    ua::String indexRange{};
    ua::QualifiedName dataEncoding{};
};

struct WriteValue {
    ua::NodeId node{};
    ua::UInt32 attribute = 13;
    ua::String indexRange{};
    Byte mask = 0x01;
    ua::Variant value{};
    ua::StatusCode status = ua::StatusCode::Good;
    ua::DateTime source = 0, server = 0;
};

class ServiceTest : public ::testing::Test {
public:
    ua::ViewState view;
    ua::ServiceContext ctx{kSpace, false};
    std::array<Byte, 4096> request{};
    std::array<Byte, 16384> response{};

    void SetUp() override {
        g_answer = 42;
        g_temperature = 21.5;
    }

    /// Run `handler` on the request body `encode` writes.
    Response run(const std::function<void(ua::ByteWriter&)>& encode,
                 const std::function<bool(const ua::RequestHeader&, ua::ByteReader&, ua::ByteWriter&)>& handler) {
        ua::ByteWriter rw(request);
        encode(rw);
        EXPECT_TRUE(rw.Ok());
        ua::ByteReader rr(rw.Written());
        ua::ByteWriter w(response);
        ua::RequestHeader header;
        header.requestHandle = 77;
        EXPECT_TRUE(handler(header, rr, w)) << "handler failed (malformed request?)";
        EXPECT_TRUE(w.Ok());
        return parse(w.Written());
    }

    // ---- View ---------------------------------------------------------------------------
    static void encode_description(ua::ByteWriter& w, const BrowseDescription& d) {
        ua::EncodeNodeId(w, d.node);
        w.WriteInt32(d.direction);
        ua::EncodeNodeId(w, d.referenceType);
        w.WriteBoolean(d.includeSubtypes);
        w.WriteUInt32(d.nodeClassMask);
        w.WriteUInt32(d.resultMask);
    }
    static BrowseResult decode_browse_result(ua::ByteReader& r) {
        BrowseResult out;
        std::string_view cp;
        ua::Int32 count = 0;
        EXPECT_TRUE(r.ReadStatusCode(out.status) && r.ReadByteString(cp, out.continuationNull) && r.ReadInt32(count));
        out.continuation.assign(cp.begin(), cp.end());
        for (ua::Int32 i = 0; i < count; i++) {
            Reference ref;
            EXPECT_TRUE(ua::DecodeNodeId(r, ref.referenceType) && r.ReadBoolean(ref.forward) &&
                        ua::DecodeExpandedNodeId(r, ref.target) && ua::DecodeQualifiedName(r, ref.browseName) &&
                        ua::DecodeLocalizedText(r, ref.displayName) && r.ReadInt32(ref.nodeClass) &&
                        ua::DecodeExpandedNodeId(r, ref.typeDefinition));
            out.references.push_back(ref);
        }
        return out;
    }
    std::vector<BrowseResult> results(Response& resp) {
        std::vector<BrowseResult> out;
        if (resp.fault()) return out;
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0;
        EXPECT_TRUE(r.ReadInt32(count));
        for (ua::Int32 i = 0; i < count; i++) out.push_back(decode_browse_result(r));
        return out;
    }

    Response browse_request(const std::vector<BrowseDescription>& d, ua::UInt32 maxReferences = 0,
                            ua::NodeId viewId = {}) {
        return run(
            [&](ua::ByteWriter& w) {
                ua::EncodeNodeId(w, viewId);
                w.WriteDateTime(0);
                w.WriteUInt32(0);
                w.WriteUInt32(maxReferences);
                w.WriteInt32(static_cast<ua::Int32>(d.size()));
                for (const auto& x : d) encode_description(w, x);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return ua::HandleBrowse(ctx, view, h, r, w);
            });
    }
    BrowseResult browse(const BrowseDescription& d, ua::UInt32 maxReferences = 0) {
        Response resp = browse_request({d}, maxReferences);
        auto all = results(resp);
        EXPECT_EQ(all.size(), 1u);
        return all.empty() ? BrowseResult{} : all.front();
    }
    Response browse_next_request(const std::vector<Bytes>& points, bool release) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteBoolean(release);
                w.WriteInt32(static_cast<ua::Int32>(points.size()));
                for (const auto& p : points)
                    w.WriteByteString(std::string_view(reinterpret_cast<const char*>(p.data()), p.size()), false);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return ua::HandleBrowseNext(ctx, view, h, r, w);
            });
    }
    BrowseResult browse_next(const Bytes& point, bool release = false) {
        Response resp = browse_next_request({point}, release);
        auto all = results(resp);
        EXPECT_EQ(all.size(), 1u);
        return all.empty() ? BrowseResult{} : all.front();
    }

    Response translate_request(const std::vector<std::pair<ua::NodeId, std::vector<PathElement>>>& paths) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteInt32(static_cast<ua::Int32>(paths.size()));
                for (const auto& [start, elements] : paths) {
                    ua::EncodeNodeId(w, start);
                    w.WriteInt32(static_cast<ua::Int32>(elements.size()));
                    for (const auto& e : elements) {
                        ua::EncodeNodeId(w, e.referenceType);
                        w.WriteBoolean(e.inverse);
                        w.WriteBoolean(e.includeSubtypes);
                        ua::EncodeQualifiedName(w, e.name);
                    }
                }
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return ua::HandleTranslateBrowsePaths(ctx, h, r, w);
            });
    }
    PathResult translate(const ua::NodeId& start, const std::vector<PathElement>& elements) {
        Response resp = translate_request({{start, elements}});
        PathResult out;
        if (resp.fault()) return out;
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0, targets = 0;
        EXPECT_TRUE(r.ReadInt32(count) && count == 1 && r.ReadStatusCode(out.status) && r.ReadInt32(targets));
        for (ua::Int32 i = 0; i < targets; i++) {
            ua::NodeId id;
            ua::UInt32 remaining = 0;
            EXPECT_TRUE(ua::DecodeExpandedNodeId(r, id) && r.ReadUInt32(remaining));
            out.targets.push_back(id);
        }
        return out;
    }

    Response register_request(const std::vector<ua::NodeId>& ids, bool unregister) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteInt32(static_cast<ua::Int32>(ids.size()));
                for (const auto& id : ids) ua::EncodeNodeId(w, id);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return ua::HandleRegisterNodes(h, r, w, unregister);
            });
    }

    // ---- Attributes ---------------------------------------------------------------------
    static DataValue decode_data_value(ua::ByteReader& r) {
        DataValue v;
        EXPECT_TRUE(r.ReadByte(v.mask));
        if (v.mask & 0x01) {
            EXPECT_TRUE(ua::DecodeVariant(r, v.value));
        }
        if (v.mask & 0x02) {
            EXPECT_TRUE(r.ReadStatusCode(v.status));
        }
        if (v.mask & 0x04) {
            EXPECT_TRUE(r.ReadDateTime(v.source));
        }
        if (v.mask & 0x10) {
            EXPECT_TRUE(r.Skip(2));
        }
        if (v.mask & 0x08) {
            EXPECT_TRUE(r.ReadDateTime(v.server));
        }
        if (v.mask & 0x20) {
            EXPECT_TRUE(r.Skip(2));
        }
        return v;
    }
    Response read_request(const std::vector<ReadValueId>& ids, ua::Int32 timestamps = 3, ua::Double maxAge = 0) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteDouble(maxAge);
                w.WriteInt32(timestamps);
                w.WriteInt32(static_cast<ua::Int32>(ids.size()));
                for (const auto& id : ids) {
                    ua::EncodeNodeId(w, id.node);
                    w.WriteUInt32(id.attribute);
                    w.WriteString(id.indexRange);
                    ua::EncodeQualifiedName(w, id.dataEncoding);
                }
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) { return ua::HandleRead(ctx, h, r, w); });
    }
    std::vector<DataValue> values(Response& resp) {
        std::vector<DataValue> out;
        if (resp.fault()) return out;
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0;
        EXPECT_TRUE(r.ReadInt32(count));
        for (ua::Int32 i = 0; i < count; i++) out.push_back(decode_data_value(r));
        return out;
    }
    DataValue read(const ReadValueId& id, ua::Int32 timestamps = 3) {
        Response resp = read_request({id}, timestamps);
        auto all = values(resp);
        EXPECT_EQ(all.size(), 1u);
        return all.empty() ? DataValue{} : all.front();
    }

    Response write_request(const std::vector<WriteValue>& values) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteInt32(static_cast<ua::Int32>(values.size()));
                for (const auto& v : values) {
                    ua::EncodeNodeId(w, v.node);
                    w.WriteUInt32(v.attribute);
                    w.WriteString(v.indexRange);
                    w.WriteByte(v.mask);
                    if (v.mask & 0x01) ua::EncodeVariant(w, v.value);
                    if (v.mask & 0x02) w.WriteStatusCode(v.status);
                    if (v.mask & 0x04) w.WriteDateTime(v.source);
                    if (v.mask & 0x08) w.WriteDateTime(v.server);
                }
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) { return ua::HandleWrite(ctx, h, r, w); });
    }
    ua::StatusCode write(const WriteValue& v) {
        Response resp = write_request({v});
        if (resp.fault()) return resp.result;
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0;
        ua::StatusCode s = ua::StatusCode::BadInternalError;
        EXPECT_TRUE(r.ReadInt32(count) && count == 1 && r.ReadStatusCode(s));
        return s;
    }
};

}  // namespace harness
