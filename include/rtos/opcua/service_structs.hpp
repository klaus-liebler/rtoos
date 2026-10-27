#pragma once
// Composite structs shared by more than one service (Part 4 7.1 ApplicationDescription,
// 7.10 EndpointDescription, 7.37 UserTokenPolicy) plus small array-skip helpers used
// throughout request decoding for fields this server reads but never acts on (LocaleIds,
// ProfileUris, ...).
#include <span>

#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

// Part 4 7.36.3 ApplicationType enum.
constexpr Int32 APPLICATION_TYPE_SERVER = 0;

struct ApplicationDescription {
    String applicationUri;
    String productUri;
    LocalizedText applicationName;
    Int32 applicationType = APPLICATION_TYPE_SERVER;
    String gatewayServerUri;
    String discoveryProfileUri;
    // discoveryUrls (array of String): always encoded as a null array (no discovery service,
    // see the project's decision to leave UA_ENABLE_DISCOVERY-equivalent functionality out).
};
bool EncodeApplicationDescription(ByteWriter &w, const ApplicationDescription &v);
// discoveryUrls is decoded and discarded (only relevant for a CreateSessionRequest's
// ClientDescription, which this server never acts on beyond having decoded it correctly).
bool DecodeApplicationDescription(ByteReader &r, ApplicationDescription &out);

struct UserTokenPolicy {
    String policyId;
    Int32 tokenType = 0; // UserTokenType, Part 4 7.36.5 -- 0 = Anonymous
    String issuedTokenType;
    String issuerEndpointUrl;
    String securityPolicyUri;
};
bool EncodeUserTokenPolicy(ByteWriter &w, const UserTokenPolicy &v);

struct EndpointDescription {
    String endpointUrl;
    ApplicationDescription server;
    // serverCertificate is not stored here -- EncodeEndpointDescription writes the server's
    // certificate CHAIN (leaf + CA) directly via WriteServerCertificateChain(), see
    // secure_channel.hpp.
    Int32 securityMode = 2; // MessageSecurityMode, Part 4 7.15 -- 2 = Sign
    String securityPolicyUri;
    // userIdentityTokens: exactly one policy, Anonymous (set by BuildEndpoint()).
    UserTokenPolicy userIdentityToken;
    String transportProfileUri;
    Byte securityLevel = 0;
};
bool EncodeEndpointDescription(ByteWriter &w, const EndpointDescription &v);

// Builds this server's one and only endpoint (opc.tcp://<host>:<port>,
// SecurityPolicy#Basic256Sha256, SecurityMode Sign, anonymous auth) -- used by both
// GetEndpoints and CreateSession (Part 4 5.6.2.3: the session response carries the same
// endpoint list as GetEndpoints would).
// secure: Basic256Sha256 / Sign (security level 1); else None / None (0, read-only, see
// secure_channel.hpp).
EndpointDescription BuildEndpoint(std::string_view endpointUrl, bool secure);
// Both endpoints of this server (GetEndpoints, CreateSession's ServerEndpoints).
bool EncodeEndpoints(ByteWriter &w, std::string_view endpointUrl);

// Reads and discards a length-prefixed array of String -- LocaleIds/ProfileUris/... wherever
// this server accepts but ignores the filter.
bool SkipStringArray(ByteReader &r);

}  // namespace rtos::opcua
