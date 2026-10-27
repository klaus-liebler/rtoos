#pragma once
// The namespace 0 skeleton every address space of this server starts with (Root, Objects,
// Server/ServerStatus, the reference, object, variable and data types its nodes use) -- the
// fixed part of the FCU's generated address space (GeneratedOpcUa, UniversalRegisterAccess).
// An application joins its own nodes and references onto it at compile time:
//
//   constexpr auto kNodes = opcua::JoinNodes(opcua::kBaseNodes, myNodes);
//   constexpr auto kReferences = opcua::JoinReferences(opcua::kBaseReferences, myReferences);
//   static_assert(opcua::AllReferencesResolve(kNodes, kReferences));
//   constexpr opcua::AddressSpace space(kNodes, kReferences);
//
// Application objects are organized under Objects (ns0::ObjectsFolder).
#include <algorithm>
#include <array>
#include <cstddef>

#include "rtos/opcua/address_space.hpp"

namespace rtos::opcua {

namespace detail {
template <class T, std::size_t A, std::size_t B>
constexpr std::array<T, A + B> Join(const std::array<T, A> &a, const std::array<T, B> &b) {
    std::array<T, A + B> out{};
    std::copy(a.begin(), a.end(), out.begin());
    std::copy(b.begin(), b.end(), out.begin() + A);
    return out;
}
} // namespace detail

template <std::size_t A, std::size_t B>
constexpr std::array<Node, A + B> JoinNodes(const std::array<Node, A> &a, const std::array<Node, B> &b) {
    return detail::Join(a, b);
}
template <std::size_t A, std::size_t B>
constexpr std::array<ReferenceEntry, A + B> JoinReferences(const std::array<ReferenceEntry, A> &a,
                                                           const std::array<ReferenceEntry, B> &b) {
    return detail::Join(a, b);
}

inline constexpr std::array kBaseNodes = {
    Node{.nodeId = NodeId(0, ns0::RootFolder), .nodeClass = NodeClass::Object,
         .browseName = QualifiedName(0, "Root"), .displayName = LocalizedText("en-US", "Root")},
    Node{.nodeId = NodeId(0, ns0::ObjectsFolder), .nodeClass = NodeClass::Object,
         .browseName = QualifiedName(0, "Objects"), .displayName = LocalizedText("en-US", "Objects")},
    Node{.nodeId = NodeId(0, ns0::Organizes), .nodeClass = NodeClass::ReferenceType,
         .browseName = QualifiedName(0, "Organizes"), .displayName = LocalizedText("en-US", "Organizes"),
         .inverseName = LocalizedText("en-US", "OrganizedBy")},
    Node{.nodeId = NodeId(0, ns0::HasComponent), .nodeClass = NodeClass::ReferenceType,
         .browseName = QualifiedName(0, "HasComponent"), .displayName = LocalizedText("en-US", "HasComponent"),
         .inverseName = LocalizedText("en-US", "ComponentOf")},
    Node{.nodeId = NodeId(0, ns0::HasTypeDefinition), .nodeClass = NodeClass::ReferenceType,
         .browseName = QualifiedName(0, "HasTypeDefinition"), .displayName = LocalizedText("en-US", "HasTypeDefinition"),
         .inverseName = LocalizedText("en-US", "TypeDefinitionOf")},
    Node{.nodeId = NodeId(0, ns0::FolderType), .nodeClass = NodeClass::ObjectType,
         .browseName = QualifiedName(0, "FolderType"), .displayName = LocalizedText("en-US", "FolderType")},
    Node{.nodeId = NodeId(0, ns0::BaseDataVariableType), .nodeClass = NodeClass::VariableType,
         .browseName = QualifiedName(0, "BaseDataVariableType"), .displayName = LocalizedText("en-US", "BaseDataVariableType"),
         .dataType = NodeId(0, ns0::DataType_BaseDataType)},
    Node{.nodeId = NodeId(0, ns0::ServerType), .nodeClass = NodeClass::ObjectType,
         .browseName = QualifiedName(0, "ServerType"), .displayName = LocalizedText("en-US", "ServerType")},
    Node{.nodeId = NodeId(0, ns0::ServerStatusType), .nodeClass = NodeClass::VariableType,
         .browseName = QualifiedName(0, "ServerStatusType"), .displayName = LocalizedText("en-US", "ServerStatusType"),
         .dataType = NodeId(0, ns0::ServerStatusDataType)},
    Node{.nodeId = NodeId(0, ns0::ServerStatusDataType), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "ServerStatusDataType"), .displayName = LocalizedText("en-US", "ServerStatusDataType")},
    Node{.nodeId = NodeId(0, ns0::DataType_Int16), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "Int16"), .displayName = LocalizedText("en-US", "Int16")},
    Node{.nodeId = NodeId(0, ns0::DataType_UInt16), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "UInt16"), .displayName = LocalizedText("en-US", "UInt16")},
    Node{.nodeId = NodeId(0, ns0::DataType_Int32), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "Int32"), .displayName = LocalizedText("en-US", "Int32")},
    Node{.nodeId = NodeId(0, ns0::DataType_UInt32), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "UInt32"), .displayName = LocalizedText("en-US", "UInt32")},
    Node{.nodeId = NodeId(0, ns0::DataType_String), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "String"), .displayName = LocalizedText("en-US", "String")},
    Node{.nodeId = NodeId(0, ns0::DataType_LocalizedText), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "LocalizedText"), .displayName = LocalizedText("en-US", "LocalizedText")},
    Node{.nodeId = NodeId(0, ns0::DataType_BaseDataType), .nodeClass = NodeClass::DataType,
         .browseName = QualifiedName(0, "BaseDataType"), .displayName = LocalizedText("en-US", "BaseDataType"),
         .isAbstract = true},
    Node{.nodeId = NodeId(0, ns0::Server), .nodeClass = NodeClass::Object,
         .browseName = QualifiedName(0, "Server"), .displayName = LocalizedText("en-US", "Server")},
    Node{.nodeId = NodeId(0, ns0::Server_ServerStatus), .nodeClass = NodeClass::Variable,
         .browseName = QualifiedName(0, "ServerStatus"), .displayName = LocalizedText("en-US", "ServerStatus"),
         .dataType = NodeId(0, ns0::ServerStatusDataType), .accessLevel = ns0::ACCESS_LEVEL_CURRENT_READ},
    Node{.nodeId = NodeId(0, ns0::Server_ServerStatus_State), .nodeClass = NodeClass::Variable,
         .browseName = QualifiedName(0, "State"), .displayName = LocalizedText("en-US", "State"),
         .dataType = NodeId(0, ns0::DataType_Int32), .accessLevel = ns0::ACCESS_LEVEL_CURRENT_READ,
         .staticValue = Variant::Of(static_cast<Int32>(ns0::ServerState_Running))},
    Node{.nodeId = NodeId(0, ns0::Server_ServerStatus_SecondsTillShutdown), .nodeClass = NodeClass::Variable,
         .browseName = QualifiedName(0, "SecondsTillShutdown"), .displayName = LocalizedText("en-US", "SecondsTillShutdown"),
         .dataType = NodeId(0, ns0::DataType_UInt32), .accessLevel = ns0::ACCESS_LEVEL_CURRENT_READ,
         .staticValue = Variant::Of(static_cast<UInt32>(0))},
    Node{.nodeId = NodeId(0, ns0::Server_ServerStatus_ShutdownReason), .nodeClass = NodeClass::Variable,
         .browseName = QualifiedName(0, "ShutdownReason"), .displayName = LocalizedText("en-US", "ShutdownReason"),
         .dataType = NodeId(0, ns0::DataType_LocalizedText), .accessLevel = ns0::ACCESS_LEVEL_CURRENT_READ,
         .staticValue = Variant::Of(LocalizedText("en-US", ""))},
};

inline constexpr std::array kBaseReferences = {
    ReferenceEntry{NodeId(0, ns0::RootFolder), NodeId(0, ns0::Organizes), true, NodeId(0, ns0::ObjectsFolder)},
    ReferenceEntry{NodeId(0, ns0::ObjectsFolder), NodeId(0, ns0::Organizes), false, NodeId(0, ns0::RootFolder)},
    ReferenceEntry{NodeId(0, ns0::RootFolder), NodeId(0, ns0::HasTypeDefinition), true, NodeId(0, ns0::FolderType)},
    ReferenceEntry{NodeId(0, ns0::ObjectsFolder), NodeId(0, ns0::HasTypeDefinition), true, NodeId(0, ns0::FolderType)},
    ReferenceEntry{NodeId(0, ns0::ObjectsFolder), NodeId(0, ns0::Organizes), true, NodeId(0, ns0::Server)},
    ReferenceEntry{NodeId(0, ns0::Server), NodeId(0, ns0::Organizes), false, NodeId(0, ns0::ObjectsFolder)},
    ReferenceEntry{NodeId(0, ns0::Server), NodeId(0, ns0::HasComponent), true, NodeId(0, ns0::Server_ServerStatus)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus), NodeId(0, ns0::HasComponent), false, NodeId(0, ns0::Server)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus), NodeId(0, ns0::HasTypeDefinition), true,
                   NodeId(0, ns0::ServerStatusType)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus), NodeId(0, ns0::HasComponent), true,
                   NodeId(0, ns0::Server_ServerStatus_State)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus_State), NodeId(0, ns0::HasComponent), false,
                   NodeId(0, ns0::Server_ServerStatus)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus_State), NodeId(0, ns0::HasTypeDefinition), true,
                   NodeId(0, ns0::BaseDataVariableType)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus), NodeId(0, ns0::HasComponent), true,
                   NodeId(0, ns0::Server_ServerStatus_SecondsTillShutdown)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus_SecondsTillShutdown), NodeId(0, ns0::HasComponent), false,
                   NodeId(0, ns0::Server_ServerStatus)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus_SecondsTillShutdown), NodeId(0, ns0::HasTypeDefinition), true,
                   NodeId(0, ns0::BaseDataVariableType)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus), NodeId(0, ns0::HasComponent), true,
                   NodeId(0, ns0::Server_ServerStatus_ShutdownReason)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus_ShutdownReason), NodeId(0, ns0::HasComponent), false,
                   NodeId(0, ns0::Server_ServerStatus)},
    ReferenceEntry{NodeId(0, ns0::Server_ServerStatus_ShutdownReason), NodeId(0, ns0::HasTypeDefinition), true,
                   NodeId(0, ns0::BaseDataVariableType)},
};

} // namespace rtos::opcua
