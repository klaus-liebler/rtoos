# OPC UA

rtoos includes an OPC UA server (UA Binary over opc.tcp) for small devices: the address space is a `constexpr` table in flash, all state is static, and the crypto is wolfCrypt from the same static pool as TLS. The protocol lives in [include/rtos/opcua](../include/rtos/opcua) and [src/opcua](../src/opcua). [`OpcUaServer`](../include/rtos/net/opcua_server.hpp) is the transport on `TcpServer`, with one session per client connection.

## Features

| Topic | Implementation |
|---|---|
| Services | Hello/Acknowledge, OpenSecureChannel/CloseSecureChannel, CreateSession/ActivateSession/CloseSession, Read, Write, Browse, BrowseNext, TranslateBrowsePathsToNodeIds, RegisterNodes/UnregisterNodes, GetEndpoints, FindServers, subscription services |
| Security | Basic256Sha256 in the modes Sign and SignAndEncrypt (AES-256-CBC). The client certificate must chain to the trusted CA and be valid (dates checked against SNTP time); such clients get full access. CreateSession requires the same certificate, ActivateSession its signature over the server certificate and nonce. SecurityPolicy None is read-only: Write returns `BadSecurityModeInsufficient`. GetEndpoints offers all three endpoints. |
| Sessions | one session per connection. Every service checks the AuthenticationToken and the session timeout (64-bit clock). Only the Anonymous user token is offered. |
| View | Browse with reference type (including subtypes), NodeClassMask, ResultMask and continuation points; TranslateBrowsePathsToNodeIds; RegisterNodes |
| Attributes | all attributes 1..27 per node class; scalar values of the built-in types, String and UInt32 arrays for server properties |
| Subscriptions | data-change subscriptions ([subscriptions.hpp](../include/rtos/opcua/subscriptions.hpp)): Create/Modify/Delete, SetPublishingMode, MonitoredItems with a queue (overflow bit), DataChangeFilter (trigger, absolute deadband), SetMonitoringMode, SetTriggering, Publish with keep-alive and lifetime, Republish |
| Crypto | wolfCrypt: RSA-2048 (OAEP-SHA1, PKCS#1 v1.5 with SHA-256), HMAC-SHA256, P_SHA256, AES-256-CBC. No heap. |
| Fairness | an OpenSecureChannel (three RSA-2048 private-key operations) is a background step; every other message is answered at once |

Not implemented: TransferSubscriptions, events, methods (Call), history, user tokens other than Anonymous, other security policies, arrays as application variables, token overlap on renew.

## Usage

```cpp
namespace ua = rtos::opcua;

// address space: base nodes (Server object, types) + your nodes, checked at compile time
constexpr std::array app_nodes = { /* ua::Node{...} */ };
constexpr std::array app_references = { /* ua::ReferenceEntry{...} */ };
constexpr auto nodes = ua::JoinNodes(ua::kBaseNodes, app_nodes);
constexpr auto references = ua::JoinReferences(ua::kBaseReferences, app_references);
static_assert(ua::AllTypedNodesHaveDataType(nodes));
static_assert(ua::AllDataTypesResolve(nodes));
static_assert(ua::AllReferencesResolve(nodes, references));
constexpr ua::AddressSpace space(nodes, references);

rtos::net::OpcUaServer<Stack, 2> opcua{stack, space, "opc.tcp://my-device.local:4840"};

// in the network thread, after TlsServerContext::initialize_library():
ua::SetApplicationIdentity({.applicationUri = "urn:my-company:my-device",
                            .productUri = "urn:my-company:my-product",
                            .applicationName = "My Device"});
ua::SetClock([] noexcept { return sntp.unix_ms(kernel.now()); }, [] noexcept { return kernel.now64(); });
opcua.init({opcua_cert_der, opcua_key_der, ca_der}, {});   // {}: share the TLS pool
ua::registers::Bind(registers);
opcua.start();                                             // port 4840, all interfaces
runtime.add(opcua, "opcua");
```

### Application identity

`SetApplicationIdentity` ([application.hpp](../include/rtos/opcua/application.hpp)) sets the ApplicationUri, ProductUri and ApplicationName that GetEndpoints and FindServers report. Namespace 1 of `Server.NamespaceArray` is the ApplicationUri. The ApplicationUri must match the `URI:` entry in the SubjectAltName of the application certificate (`make_certs.py --opcua-uri`, `rtos_certificates(... OPCUA_URI ...)`), or clients reject the server. The defaults are those of the example device, the Factory Control Unit.

### Address space

Nodes are `constexpr` `ua::Node` values: NodeId, class, BrowseName, DisplayName, data type, access level, and either a constant value or a `readCallback` / `writeCallback` with a `userContext`. [registers.hpp](../include/rtos/opcua/registers.hpp) binds variables to a Modbus `RegisterMap`, so OPC UA and Modbus see the same values:

```cpp
constexpr ua::registers::Register led{rtos::modbus::Table::Holding, 0};
ua::Node{.nodeId = ua::NodeId(1, 5000), .nodeClass = ua::NodeClass::Variable,
         .browseName = ua::QualifiedName(1, "Led"), .displayName = ua::LocalizedText("en-US", "Led"),
         .dataType = ua::NodeId(0, ua::ns0::DataType_UInt16),
         .accessLevel = ua::ns0::ACCESS_LEVEL_CURRENT_READ | ua::ns0::ACCESS_LEVEL_CURRENT_WRITE,
         .readCallback = &ua::registers::ReadUInt16, .writeCallback = &ua::registers::WriteUInt16,
         .userContext = ua::registers::Context(led)};
```

`ReadUInt32` combines two registers (high word first). [apps/net/main.cpp](../apps/net/main.cpp) builds a complete `Objects/FCU` folder this way. Callbacks run in the network thread and must be short.

### Clock

`SetClock(unix_ms, monotonic_ms)` connects the server to SNTP time (for timestamps and certificate dates) and to `Kernel::now64()` (session and subscription timeouts that never wrap). Before the first SNTP sync, the server time is anchored to the build timestamp, so clients don't see expired channel tokens.

### Memory

| Item | Size |
|---|---|
| per session: subscriptions | `MAX_SUBSCRIPTIONS` 2, `MAX_MONITORED_ITEMS` 12, queue 4, 4 publish requests, 1 KB per notification message (plus its copy for Republish) |
| per connection | message buffer of one UA TCP message (single chunk) |
| crypto | wolfCrypt allocations from the shared wolfSSL pool; RSA-2048 needs about 13.5 KB of stack in the network thread |

The limits are constants in [subscriptions.hpp](../include/rtos/opcua/subscriptions.hpp) and [session.hpp](../include/rtos/opcua/session.hpp).

## Tests

- [opcua_test.cpp](../tests/opcua_test.cpp) has its own client with wolfCrypt (None, Sign, SignAndEncrypt). It checks the Hello limits, read-only None and GetEndpoints, a signed channel with write access, rejected foreign certificates and tampered signatures, the message limit, 32-bit values from two registers, the OpenSecureChannel as a background step and a Read answered while another client's RSA open waits, and the status codes against the specification.
- [opcua_o6_view_test.cpp](../tests/opcua_o6_view_test.cpp), [opcua_o6_attributes_test.cpp](../tests/opcua_o6_attributes_test.cpp), [opcua_o6_subscriptions_test.cpp](../tests/opcua_o6_subscriptions_test.cpp) drive the services through a harness without TCP ([opcua_service_harness.hpp](../tests/opcua_service_harness.hpp)); [opcua_codec_test.cpp](../tests/opcua_codec_test.cpp) tests the codec.
- A mutation fuzzer varies messages on None and signed channels.
- Renode: `net_test.py` uses asyncua: Browse and Read over None, a rejected Write, Write with the client certificate `certs/opcua_client.*` (LED and Modbus registers), rejection of a self-signed client, SignAndEncrypt, and Reads during parallel downloads.

### Tests from open62541

The open62541 test suite (4503 test cases) is mapped completely. 309 are ported, and each one carries the marker `// open62541: <file> <test>`. For all others, [tests/open62541/rules.tsv](../tests/open62541/rules.tsv) gives the reason. Most of them test library functions without a counterpart here: the client, PubSub, JSON/XML, the node store, the run-time server API. [tools/open62541_catalog.py](../tools/open62541_catalog.py) writes [tests/open62541/catalog.md](../tests/open62541/catalog.md) and fails if a test is neither ported nor explained:

```
python tools/open62541_catalog.py <open62541>/tests          # rewrite the catalog
python tools/open62541_catalog.py <open62541>/tests --check  # check only
```

Run it after every OPC UA change and commit the regenerated catalog.
