// SPDX-License-Identifier: MIT
//
// View service set (Browse, BrowseNext, TranslateBrowsePathsToNodeIds, RegisterNodes,
// UnregisterNodes), ported from open62541 tests/server/check_services_view.c. The tests that
// do not fit (server-side APIs without a service) are listed with their reasons in
// tests/open62541/rules.tsv.
#include "opcua_service_harness.hpp"

namespace {

using namespace harness;

ua::NodeId ns0(ua::UInt32 id) { return ua::NodeId(0, id); }

bool is_hierarchical(const ua::NodeId& type) {
    return ua::IsReferenceTypeMatch(kSpace, type, ns0(ua::ns0::HierarchicalReferences), true);
}

// open62541: server/check_services_view.c Service_Browse_CheckSubTypes
TEST(Open62541View, Service_Browse_CheckSubTypes) {
    const ua::NodeId hierarchical = ns0(ua::ns0::HierarchicalReferences);
    for (ua::UInt32 sub : {ua::ns0::Organizes, ua::ns0::HasProperty, ua::ns0::HasChild, ua::ns0::Aggregates,
                           ua::ns0::HasComponent, ua::ns0::HasSubtype}) {
        EXPECT_TRUE(ua::IsReferenceTypeMatch(kSpace, ns0(sub), hierarchical, true)) << sub;
    }
    EXPECT_FALSE(ua::IsReferenceTypeMatch(kSpace, ns0(ua::ns0::HasTypeDefinition), hierarchical, true));
    EXPECT_FALSE(ua::IsReferenceTypeMatch(kSpace, ns0(ua::ns0::NonHierarchicalReferences), hierarchical, true));
    EXPECT_FALSE(ua::IsReferenceTypeMatch(kSpace, ns0(ua::ns0::Organizes), hierarchical, false))
        << "without includeSubtypes only the type itself";
}

class Open62541ViewTest : public ServiceTest {};

// Browse with requestedMaxReferencesPerNode = max, then BrowseNext until the end.
std::size_t browse_with_max_results(Open62541ViewTest& t, const BrowseDescription& d, ua::UInt32 max) {
    BrowseResult r = t.browse(d, max);
    EXPECT_EQ(r.status, ua::StatusCode::Good);
    std::size_t total = r.references.size();
    while (!r.continuationNull) {
        EXPECT_LE(r.references.size(), max);
        r = t.browse_next(r.continuation);
        EXPECT_EQ(r.status, ua::StatusCode::Good);
        total += r.references.size();
    }
    return total;
}

// open62541: server/check_services_view.c Service_Browse_WithMaxResults
TEST_F(Open62541ViewTest, Service_Browse_WithMaxResults) {
    const BrowseDescription d{.node = ns0(ua::ns0::Server)};
    const BrowseResult all = browse(d);
    ASSERT_EQ(all.status, ua::StatusCode::Good);
    ASSERT_GT(all.references.size(), 0u);
    EXPECT_TRUE(all.continuationNull);
    for (ua::UInt32 i = 1; i <= all.references.size(); i++) {
        EXPECT_EQ(browse_with_max_results(*this, d, i), all.references.size()) << "max " << i;
    }
}

// open62541: server/check_services_view.c Service_Browse_ReclaimsOldestContinuationPoint
TEST_F(Open62541ViewTest, Service_Browse_ReclaimsOldestContinuationPoint) {
    const BrowseDescription d{.node = ns0(ua::ns0::Server)};
    std::vector<Bytes> points;
    for (std::size_t i = 0; i < ua::ViewState::MAX_CONTINUATION_POINTS; i++) {
        const BrowseResult r = browse(d, 1);
        ASSERT_EQ(r.status, ua::StatusCode::Good);
        ASSERT_FALSE(r.continuationNull);
        points.push_back(r.continuation);
    }
    const BrowseResult r = browse(d, 1);  // reclaims the oldest point
    EXPECT_EQ(r.status, ua::StatusCode::Good);
    EXPECT_FALSE(r.continuationNull);
    EXPECT_EQ(browse_next(points[0]).status, ua::StatusCode::BadContinuationPointInvalid);
    EXPECT_EQ(browse_next(points[1]).status, ua::StatusCode::Good) << "a newer point was not reclaimed";
}

// open62541: server/check_services_view.c Service_Browse_WithBrowseName
TEST_F(Open62541ViewTest, Service_Browse_WithBrowseName) {
    const BrowseResult r = browse({.node = ns0(ua::ns0::ObjectsFolder), .referenceType = ns0(ua::ns0::Organizes),
                                   .resultMask = 0x08});
    ASSERT_EQ(r.status, ua::StatusCode::Good);
    ASSERT_GT(r.references.size(), 0u);
    EXPECT_FALSE(r.references[0].browseName.name.empty());
    EXPECT_TRUE(r.references[0].referenceType.IsNull()) << "not requested by the ResultMask";
    EXPECT_TRUE(r.references[0].displayName.text.empty()) << "not requested by the ResultMask";
}

// open62541: server/check_services_view.c Service_Browse_ClassMask
TEST_F(Open62541ViewTest, Service_Browse_ClassMask) {
    const auto with_mask = [&](ua::UInt32 mask) {
        const BrowseResult r = browse({.node = ns0(ua::ns0::ObjectsFolder), .nodeClassMask = mask});
        EXPECT_EQ(r.status, ua::StatusCode::Good);
        for (const Reference& ref : r.references) {
            EXPECT_NE(static_cast<ua::UInt32>(ref.nodeClass) & mask, 0u);
        }
        return r.references.size();
    };
    const std::size_t objects = with_mask(static_cast<ua::UInt32>(ua::NodeClass::Object));
    const std::size_t variables = with_mask(static_cast<ua::UInt32>(ua::NodeClass::Variable));
    EXPECT_GT(objects, 0u);
    EXPECT_GT(variables, 0u);
    EXPECT_EQ(with_mask(static_cast<ua::UInt32>(ua::NodeClass::Object) | static_cast<ua::UInt32>(ua::NodeClass::Variable)),
              objects + variables);
}

// open62541: server/check_services_view.c Service_Browse_ReferenceTypes
TEST_F(Open62541ViewTest, Service_Browse_ReferenceTypes) {
    const BrowseResult all = browse({.node = ns0(ua::ns0::ObjectsFolder), .resultMask = 0x01});
    ASSERT_EQ(all.status, ua::StatusCode::Good);
    std::size_t hierarchical = 0;
    for (const Reference& ref : all.references) hierarchical += is_hierarchical(ref.referenceType) ? 1 : 0;
    EXPECT_LT(hierarchical, all.references.size()) << "Objects also has a HasTypeDefinition reference";
    const BrowseResult filtered = browse({.node = ns0(ua::ns0::ObjectsFolder),
                                          .referenceType = ns0(ua::ns0::HierarchicalReferences),
                                          .includeSubtypes = true, .resultMask = 0x01});
    EXPECT_EQ(filtered.references.size(), hierarchical);
}

// open62541: server/check_services_view.c ServiceTest_TranslateBrowsePathsToNodeIds
TEST_F(Open62541ViewTest, ServiceTest_TranslateBrowsePathsToNodeIds) {
    // /Objects/Server/ServerStatus/State = 85/2253/2256/2259
    const PathResult r = translate(ns0(ua::ns0::ObjectsFolder),
                                   {{.referenceType = ns0(ua::ns0::Organizes), .name = ua::QualifiedName(0, "Server")},
                                    {.referenceType = ns0(ua::ns0::HasComponent), .name = ua::QualifiedName(0, "ServerStatus")},
                                    {.referenceType = ns0(ua::ns0::HasComponent), .name = ua::QualifiedName(0, "State")}});
    ASSERT_EQ(r.status, ua::StatusCode::Good);
    ASSERT_EQ(r.targets.size(), 1u);
    EXPECT_EQ(r.targets[0], ns0(ua::ns0::Server_ServerStatus_State));
}

// open62541: server/check_services_view.c Service_TranslateBrowsePathsWithHashCollision
TEST_F(Open62541ViewTest, Service_TranslateBrowsePathsWithHashCollision) {
    // names "\0", "\0\0", "\0\0\0": each one matches exactly one node, not its prefixes
    const ua::NodeId expected[] = {kZero1, kZero2, kZero3};
    for (std::size_t i = 0; i < 3; i++) {
        const PathResult r = translate(ns0(ua::ns0::ObjectsFolder),
                                       {{.referenceType = ns0(ua::ns0::Organizes),
                                         .name = ua::QualifiedName(0, std::string_view(kZeros, i + 1))}});
        ASSERT_EQ(r.status, ua::StatusCode::Good);
        ASSERT_EQ(r.targets.size(), 1u);
        EXPECT_EQ(r.targets[0], expected[i]);
    }
}

// open62541: server/check_services_view.c Service_TranslateBrowsePathsNoMatches
TEST_F(Open62541ViewTest, Service_TranslateBrowsePathsNoMatches) {
    const PathResult r = translate(ns0(ua::ns0::ObjectsFolder),
                                   {{.referenceType = ns0(ua::ns0::Organizes),
                                     .name = ua::QualifiedName(0, "NoMatchToBeFound")}});
    EXPECT_EQ(r.status, ua::StatusCode::BadNoMatch);
    EXPECT_TRUE(r.targets.empty());
}

// The "simplified browse path" of open62541 is a TranslateBrowsePaths over hierarchical
// references (with subtypes).
// open62541: server/check_services_view.c BrowseSimplifiedBrowsePath
TEST_F(Open62541ViewTest, BrowseSimplifiedBrowsePath) {
    const PathResult r = translate(ns0(ua::ns0::RootFolder),
                                   {{.referenceType = ns0(ua::ns0::HierarchicalReferences), .includeSubtypes = true,
                                     .name = ua::QualifiedName(0, "Objects")}});
    ASSERT_EQ(r.status, ua::StatusCode::Good);
    ASSERT_EQ(r.targets.size(), 1u);
    EXPECT_EQ(r.targets[0], ns0(ua::ns0::ObjectsFolder));
}

// open62541 limits the simplified path to 50 elements (BadInternalError); here a path is at
// most MAX_BROWSE_PATH_ELEMENTS long and a longer one is BadTooManyOperations.
// open62541: server/check_services_view.c BrowseSimplifiedBrowsePath_tooLong_returnsInternalError
TEST_F(Open62541ViewTest, BrowsePathTooLongIsRejected) {
    std::vector<PathElement> path(ua::MAX_BROWSE_PATH_ELEMENTS + 1,
                                  PathElement{.referenceType = ns0(ua::ns0::HierarchicalReferences),
                                              .includeSubtypes = true, .name = ua::QualifiedName(0, "Anything")});
    const PathResult r = translate(ns0(ua::ns0::RootFolder), path);
    EXPECT_EQ(r.status, ua::StatusCode::BadTooManyOperations);
    EXPECT_TRUE(r.targets.empty());
}

// open62541: server/check_services_view.c Service_Browse_EmptyRefType
TEST_F(Open62541ViewTest, Service_Browse_EmptyRefType) {
    const BrowseResult r = browse({.node = ns0(ua::ns0::ObjectsFolder), .includeSubtypes = true});
    EXPECT_EQ(r.status, ua::StatusCode::Good);
    EXPECT_GT(r.references.size(), 0u);
}

// open62541: server/check_services_view.c Service_Browse_InvalidRefTypeId
TEST_F(Open62541ViewTest, Service_Browse_InvalidRefTypeId) {
    const BrowseResult r = browse({.node = ns0(ua::ns0::ObjectsFolder), .referenceType = ua::NodeId(1, 99999),
                                   .includeSubtypes = true});
    EXPECT_EQ(r.status, ua::StatusCode::BadReferenceTypeIdInvalid);
}

// open62541: server/check_services_view.c Service_Browse_NonReferenceTypeAsRefType
TEST_F(Open62541ViewTest, Service_Browse_NonReferenceTypeAsRefType) {
    const BrowseResult r = browse({.node = ns0(ua::ns0::ObjectsFolder), .referenceType = ns0(ua::ns0::ObjectsFolder),
                                   .includeSubtypes = true});
    EXPECT_EQ(r.status, ua::StatusCode::BadReferenceTypeIdInvalid);
}

// open62541: server/check_services_view.c View_BrowseUnknownNode_returnsError
TEST_F(Open62541ViewTest, View_BrowseUnknownNode_returnsError) {
    const BrowseResult r = browse({.node = ua::NodeId(1, 999999), .referenceType = ns0(ua::ns0::HierarchicalReferences),
                                   .includeSubtypes = true});
    EXPECT_EQ(r.status, ua::StatusCode::BadNodeIdUnknown);
}

// open62541: server/check_services_view.c View_BrowseNextUnknownContinuationPoint
TEST_F(Open62541ViewTest, View_BrowseNextUnknownContinuationPoint) {
    const std::string_view bad = "not-a-real-continuation-point";
    EXPECT_EQ(browse_next(Bytes(bad.begin(), bad.end())).status, ua::StatusCode::BadContinuationPointInvalid);
}

// open62541: server/check_services_view.c View_TranslateBrowsePath_unknownStartNode
TEST_F(Open62541ViewTest, View_TranslateBrowsePath_unknownStartNode) {
    const PathResult r = translate(ua::NodeId(1, 999999),
                                   {{.referenceType = ns0(ua::ns0::HierarchicalReferences), .includeSubtypes = true,
                                     .name = ua::QualifiedName(0, "Objects")}});
    EXPECT_EQ(r.status, ua::StatusCode::BadNodeIdUnknown);
}

// open62541: server/check_services_view.c View_TranslateBrowsePath_emptyPath
TEST_F(Open62541ViewTest, View_TranslateBrowsePath_emptyPath) {
    EXPECT_EQ(translate(ns0(ua::ns0::Server), {}).status, ua::StatusCode::BadNothingToDo);
}

// open62541: server/check_services_view.c View_BrowseBothDirections_includesInverse
TEST_F(Open62541ViewTest, View_BrowseBothDirections_includesInverse) {
    const BrowseResult r = browse({.node = ns0(ua::ns0::ObjectsFolder), .direction = 2,
                                   .referenceType = ns0(ua::ns0::HierarchicalReferences), .includeSubtypes = true});
    ASSERT_EQ(r.status, ua::StatusCode::Good);
    bool forward = false, inverse = false;
    for (const Reference& ref : r.references) (ref.forward ? forward : inverse) = true;
    EXPECT_TRUE(forward);
    EXPECT_TRUE(inverse) << "Objects is organized by Root";
}

// open62541: server/check_services_view.c Service_Browse_maxNodesPerBrowse_exceeded
TEST_F(Open62541ViewTest, Service_Browse_maxNodesPerBrowse_exceeded) {
    const std::vector<BrowseDescription> d(ua::MAX_NODES_PER_BROWSE + 1, BrowseDescription{.node = ns0(ua::ns0::RootFolder)});
    const Response resp = browse_request(d);
    EXPECT_TRUE(resp.fault());
    EXPECT_EQ(resp.result, ua::StatusCode::BadTooManyOperations);
    EXPECT_EQ(resp.handle, 77u);
}

// open62541: server/check_services_view.c Service_Browse_viewIdUnknown
TEST_F(Open62541ViewTest, Service_Browse_viewIdUnknown) {
    const Response resp = browse_request({{.node = ns0(ua::ns0::RootFolder)}}, 0, ua::NodeId(1, 99999));
    EXPECT_EQ(resp.result, ua::StatusCode::BadViewIdUnknown);
}

// open62541: server/check_services_view.c Service_RegisterNodes_emptyList_returnsNothingToDo
TEST_F(Open62541ViewTest, Service_RegisterNodes_emptyList_returnsNothingToDo) {
    EXPECT_EQ(register_request({}, false).result, ua::StatusCode::BadNothingToDo);
}

// open62541: server/check_services_view.c Service_RegisterNodes_tooMany
TEST_F(Open62541ViewTest, Service_RegisterNodes_tooMany) {
    const std::vector<ua::NodeId> ids(ua::MAX_NODES_PER_REGISTER + 1, ns0(ua::ns0::RootFolder));
    EXPECT_EQ(register_request(ids, false).result, ua::StatusCode::BadTooManyOperations);
}

// open62541: server/check_services_view.c Service_UnregisterNodes_emptyList_returnsNothingToDo
TEST_F(Open62541ViewTest, Service_UnregisterNodes_emptyList_returnsNothingToDo) {
    EXPECT_EQ(register_request({}, true).result, ua::StatusCode::BadNothingToDo);
}

// open62541: server/check_services_view.c Service_UnregisterNodes_tooMany
TEST_F(Open62541ViewTest, Service_UnregisterNodes_tooMany) {
    const std::vector<ua::NodeId> ids(ua::MAX_NODES_PER_REGISTER + 1, ns0(ua::ns0::RootFolder));
    EXPECT_EQ(register_request(ids, true).result, ua::StatusCode::BadTooManyOperations);
}

// Registered NodeIds are the NodeIds themselves (no aliases), string ones included.
TEST_F(Open62541ViewTest, RegisterNodesReturnsTheNodeIds) {
    Response resp = register_request({kAnswer, kDevice}, false);
    ASSERT_EQ(resp.result, ua::StatusCode::Good);
    ua::ByteReader r = resp.reader();
    ua::Int32 count = 0;
    ua::NodeId a, b;
    ASSERT_TRUE(r.ReadInt32(count) && count == 2 && ua::DecodeNodeId(r, a) && ua::DecodeNodeId(r, b));
    EXPECT_EQ(a, kAnswer);
    EXPECT_EQ(b, kDevice);
}

// open62541: server/check_services_view.c Service_TranslateBrowsePathsToNodeIds_maxNodesPerTranslate
TEST_F(Open62541ViewTest, Service_TranslateBrowsePathsToNodeIds_maxNodesPerTranslate) {
    std::vector<std::pair<ua::NodeId, std::vector<PathElement>>> paths(ua::MAX_NODES_PER_TRANSLATE + 1,
                                                                       {ns0(ua::ns0::RootFolder), {}});
    EXPECT_EQ(translate_request(paths).result, ua::StatusCode::BadTooManyOperations);
}

// BrowseNext with releaseContinuationPoints frees the point without returning references.
TEST_F(Open62541ViewTest, BrowseNextReleasesContinuationPoints) {
    const BrowseResult first = browse({.node = ns0(ua::ns0::Server)}, 1);
    ASSERT_FALSE(first.continuationNull);
    const BrowseResult released = browse_next(first.continuation, true);
    EXPECT_EQ(released.status, ua::StatusCode::Good);
    EXPECT_TRUE(released.references.empty());
    EXPECT_EQ(browse_next(first.continuation).status, ua::StatusCode::BadContinuationPointInvalid);
}

// An invalid BrowseDirection is rejected per operation (Part 4 5.8.2.2).
TEST_F(Open62541ViewTest, BrowseDirectionMustBeValid) {
    EXPECT_EQ(browse({.node = ns0(ua::ns0::RootFolder), .direction = 3}).status, ua::StatusCode::BadBrowseDirectionInvalid);
}

}  // namespace
