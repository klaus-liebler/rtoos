#pragma once
// The server's application identity (Part 4 7.2 ApplicationDescription): GetEndpoints,
// FindServers, Server.NamespaceArray (namespace 1 = the ApplicationUri) and Server.ServerArray.
// The ApplicationUri must equal the URI in the SubjectAltName of the application certificate
// (tools/make_certs.py --opcua-uri), or clients reject the server.
//
// The defaults are those of the rtoos example device (the Factory Control Unit). A product
// sets its own before OpcUaServer::start(); the strings must outlive the server.
//
// rtoos: set once from the network thread at start-up -- no lock.
#include <string_view>

#include "rtos/opcua/types.hpp"
#include "rtos/opcua/variant.hpp"

namespace rtos::opcua {

struct ApplicationIdentity {
    std::string_view applicationUri = "urn:factory-control-unit:opcua-native";
    std::string_view productUri = "urn:factory-control-unit:opcua-native:product";
    std::string_view applicationName = "Factory Control Unit OPC UA Server";
};

void SetApplicationIdentity(const ApplicationIdentity &identity) noexcept;
const ApplicationIdentity &Application() noexcept;

/// ReadCallbacks of Server.NamespaceArray and Server.ServerArray (ns0_base.hpp).
Variant ReadNamespaceArray(void *);
Variant ReadServerArray(void *);

}  // namespace rtos::opcua
