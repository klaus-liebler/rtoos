#pragma once
// The address space is a compile-time-fixed, non-owning view over externally-defined constexpr
// tables (project requirement, 2026-08-19) -- NOT built at runtime via AddObjectNode()/
// AddVariableNode()/AddReference()-style function calls. AddressSpace itself is just two spans
// plus lookup helpers; the actual Node/ReferenceEntry data lives wherever the caller defines it
// (e.g. opcua_test_address_space.cpp today, a register-map.json-generated table later) as a
// `constexpr Node kNodes[] = {...}` array with designated initializers.
//
// Nodes are immutable by design: a genuinely writable Variable node MUST go through a
// WriteCallback pointing at externally-owned, explicitly-mutable RAM (e.g. a future
// ModbusRegisterModel-backed register) -- there is no "static value the server quietly mutates
// in place" path. This keeps every constexpr node table trivially placeable in Flash (.rodata)
// with zero RAM cost, and makes "is this node's storage mutable" a question answered entirely
// by whether it has a WriteCallback, not by inspecting AddressSpace internals.
#include <span>

#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/types.hpp"
#include "rtos/opcua/variant.hpp"

// Apply to every `constexpr Node kNodes[] = {...}` table that contains a ReadCallback/
// WriteCallback (a function pointer): GCC's default section-selection heuristic treats any
// const object holding a pointer-to-code as needing load-time relocation (correct for
// PIC/shared-library targets) and silently places it in .data instead of .rodata -- meaning
// it would still cost RAM plus a startup copy, exactly what constexpr was meant to avoid. On
// this bare-metal, statically-linked target there is no runtime relocation step at all (the
// linker resolves every function address to its final Flash location at link time), so it's
// always safe to force true .rodata placement here. Without this, `nm`/`size` on the object
// file would show the table as `d` (.data) instead of `r` (.rodata) -- verify with those tools
// if in doubt, `constexpr` alone does not guarantee Flash placement once function pointers are
// involved.
#define OPCUA_RODATA __attribute__((section(".rodata")))

namespace rtos::opcua {

class AddressSpace;

using ReadCallback = Variant (*)(void *context);
// Returns false if the value's type/range is unacceptable (-> BadTypeMismatch/BadOutOfRange
// upstream in the Write service handler).
using WriteCallback = bool (*)(void *context, const Variant &value);

struct Node {
    NodeId nodeId{};
    NodeClass nodeClass = NodeClass::Object;
    QualifiedName browseName{};
    LocalizedText displayName{};

    // --- Variable-only fields (ignored for Object nodes) ---
    NodeId dataType{};
    Byte accessLevel = 0;
    // Constant Value for a node with no ReadCallback (e.g. a fixed test/status string) --
    // ignored if readCallback is set. Never mutated at runtime; see the file header comment
    // for why a truly dynamic value must be a ReadCallback instead.
    Variant staticValue{};
    ReadCallback readCallback = nullptr;
    // Set together with a WriteCallback for a genuinely writable Variable -- a node with the
    // Write access bit set but no WriteCallback is a node-definition bug and is rejected at
    // Write time (see services.cpp WriteNodeAttribute), not silently allowed to "work" via
    // staticValue mutation.
    WriteCallback writeCallback = nullptr;
    void *userContext = nullptr;
    // ValueRank (Part 3 5.6.2): -1 scalar (every application variable), 1 a one-dimensional
    // array (e.g. Server.NamespaceArray).
    Int32 valueRank = -1;

    // --- ObjectType/VariableType/ReferenceType/DataType-only field (Part 3 5.6.2ff) ---
    bool isAbstract = false;
    // --- ReferenceType-only: symmetric (References itself; Part 3 5.3.3) -- no InverseName ---
    bool symmetric = false;
    // --- ReferenceType-only field -- mandatory (non-null) for an asymmetric ReferenceType per
    // spec (Part 3 5.5.2); none of this server's ReferenceTypes are symmetric, see
    // ReadNodeAttribute's Symmetric case (services.cpp).
    LocalizedText inverseName{};

    constexpr Variant ReadValue() const { return readCallback ? readCallback(userContext) : staticValue; }
};

struct ReferenceEntry {
    NodeId sourceId;
    NodeId referenceTypeId;
    bool isForward = true; // true: sourceId --refType--> targetId; false: the inverse direction
    NodeId targetId;
};

class AddressSpace {
public:
    constexpr AddressSpace(std::span<const Node> nodes, std::span<const ReferenceEntry> references)
        : nodes_(nodes), references_(references) {}

    constexpr const Node *FindNode(const NodeId &id) const {
        for(const Node &n : nodes_) {
            if(n.nodeId == id) return &n;
        }
        return nullptr;
    }

    // Invokes fn(const ReferenceEntry&) for every reference where sourceId matches "nodeId" in
    // the requested direction (isForward == wantForward) -- the core of the Browse service.
    template <typename F>
    constexpr void ForEachReference(const NodeId &nodeId, bool wantForward, F &&fn) const {
        for(const ReferenceEntry &ref : references_) {
            if(ref.sourceId == nodeId && ref.isForward == wantForward)
                fn(ref);
        }
    }

private:
    std::span<const Node> nodes_;
    std::span<const ReferenceEntry> references_;
};

// --- Compile-time table validation -----------------------------------------------------------
// All three checks below exist because of real bugs (2026-08-19, see project memory): a kNodes/
// kReferences table is hand-written data with no runtime construction step to catch mistakes in
// -- an omitted field silently keeps its default instead of failing to compile, and a NodeId
// used as a reference's type/target (or a Variable's DataType) before its own `Node{...}` entry
// exists is just as silent. Every node-table bug found that day (missing Root/Server/
// ServerStatus/*Type/builtin-DataType nodes, an unset Node::dataType defaulting to the null
// NodeId) was one of these three categories, discovered only via live OPC UA client testing --
// including one, the builtin DataType nodes, that only surfaced when a human actually clicked
// the node in UAExpert rather than during its automatic startup walk, i.e. NOT something you can
// assume "a quick test client run" will exercise for you. Add all four static_asserts below to
// any new `constexpr Node kNodes[] = {...}` / `constexpr ReferenceEntry kReferences[] = {...}`
// table:
//
//   static_assert(AllTypedNodesHaveDataType(kNodes), "...");
//   static_assert(AllDataTypesResolve(kNodes), "...");
//   static_assert(AllReferenceTypesHaveInverseName(kNodes), "...");
//   static_assert(AllReferencesResolve(kNodes, kReferences), "...");

constexpr bool IsNullNodeId(const NodeId &id) {
    return id.namespaceIndex == 0 && id.type == NodeIdType::Numeric && id.numeric == 0;
}

// Which NodeClasses carry a DataType/ValueRank attribute per spec (Part 3 5.6.2/5.6.5): Variable
// AND VariableType both do -- a VariableType's DataType constrains what type its own instances
// must use (e.g. BaseDataVariableType's DataType is the abstract BaseDataType). Getting this
// wrong once (2026-08-19: services.cpp's ReadNodeAttribute gated DataType/ValueRank to
// `nodeClass == Variable` only, and this file's table checks independently made the exact same
// too-narrow assumption -- two copies of one wrong rule, so neither caught the other's mistake)
// is why this predicate exists as the ONE place both the runtime attribute handler
// (ReadNodeAttribute, services.cpp) and the table checks below call -- change what "has a
// DataType" means here and both update together, instead of two hand-copied conditions silently
// drifting apart again.
constexpr bool HasDataTypeAttribute(NodeClass nc) {
    return nc == NodeClass::Variable || nc == NodeClass::VariableType;
}

// Which NodeClasses carry an IsAbstract attribute (Part 3 5.6.2/5.6.5/5.5.2/5.4.2): the four
// "Type" classes only. Shared with ReadNodeAttribute (services.cpp) for the same reason as
// HasDataTypeAttribute above.
constexpr bool HasIsAbstractAttribute(NodeClass nc) {
    return nc == NodeClass::ObjectType || nc == NodeClass::VariableType ||
           nc == NodeClass::ReferenceType || nc == NodeClass::DataType;
}

// A Variable/VariableType's DataType attribute must never be the null NodeId -- a real client
// (UAExpert confirmed) reads it for every such node it discovers and tries to resolve it as if
// it were a genuine DataType reference, failing with BadNodeIdUnknown if it's null (Node::dataType's
// default -- easy to forget to set for a Node{...} entry with many other fields to fill in).
constexpr bool AllTypedNodesHaveDataType(std::span<const Node> nodes) {
    for(const Node &n : nodes) {
        if(HasDataTypeAttribute(n.nodeClass) && IsNullNodeId(n.dataType))
            return false;
    }
    return true;
}

// A DataType attribute value is itself a NodeId a client reads NodeClass/BrowseName/DisplayName
// of (the moment it inspects that node, not just during startup discovery -- see the file header
// comment) -- being non-null (AllTypedNodesHaveDataType above) isn't enough, it must actually
// resolve to a registered Node, exactly like a reference's target/type does below.
constexpr bool AllDataTypesResolve(std::span<const Node> nodes) {
    auto exists = [&](const NodeId &id) {
        for(const Node &n : nodes)
            if(n.nodeId == id) return true;
        return false;
    };
    for(const Node &n : nodes) {
        if(HasDataTypeAttribute(n.nodeClass) && !exists(n.dataType))
            return false;
    }
    return true;
}

// Every ReferenceType node must have a non-empty InverseName (Part 3 5.5.2 -- mandatory for an
// asymmetric reference type, which every ReferenceType this server defines is; see
// ReadNodeAttribute's Symmetric case, services.cpp). Same omission-prone shape as
// AllTypedNodesHaveDataType above: an easy field to forget on a Node{...} entry with many others
// to fill in, and one this server actually shipped without at least once (2026-08-19).
constexpr bool AllReferenceTypesHaveInverseName(std::span<const Node> nodes) {
    for(const Node &n : nodes) {
        if(n.nodeClass == NodeClass::ReferenceType && !n.symmetric && n.inverseName.text.empty())
            return false;
    }
    return true;
}

// Every sourceId/referenceTypeId/targetId a ReferenceEntry mentions must be a NodeId that's
// actually present in kNodes -- e.g. using a ReferenceType/ObjectType constant from node_ids.hpp
// as a reference's referenceTypeId or target without a matching Node{...} entry for it compiles
// fine (NodeId is just data) but fails at runtime the moment a real client tries to read that
// NodeId's own attributes.
constexpr bool AllReferencesResolve(std::span<const Node> nodes, std::span<const ReferenceEntry> references) {
    auto exists = [&](const NodeId &id) {
        for(const Node &n : nodes)
            if(n.nodeId == id) return true;
        return false;
    };
    for(const ReferenceEntry &ref : references) {
        if(!exists(ref.sourceId) || !exists(ref.referenceTypeId) || !exists(ref.targetId))
            return false;
    }
    return true;
}

}  // namespace rtos::opcua
