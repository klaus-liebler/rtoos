#include "rtos/opcua/service_structs.hpp"

#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/secure_channel.hpp"
#include "rtos/opcua/service_header.hpp"

namespace rtos::opcua {

namespace {
constexpr std::string_view APPLICATION_URI = "urn:factory-control-unit:opcua-native";
constexpr std::string_view PRODUCT_URI = "urn:factory-control-unit:opcua-native:product";
constexpr std::string_view APPLICATION_NAME = "Factory Control Unit OPC UA Server";
constexpr std::string_view TRANSPORT_PROFILE_URI =
    "http://opcfoundation.org/UA-Profile/Transport/uatcp-uasc-uabinary";
constexpr std::string_view ANONYMOUS_POLICY_ID = "anonymous";
} // namespace

bool EncodeApplicationDescription(ByteWriter &w, const ApplicationDescription &v) {
    if(!w.WriteString(v.applicationUri)) return false;
    if(!w.WriteString(v.productUri)) return false;
    if(!EncodeLocalizedText(w, v.applicationName)) return false;
    if(!w.WriteInt32(v.applicationType)) return false;
    if(!w.WriteString(v.gatewayServerUri)) return false;
    if(!w.WriteString(v.discoveryProfileUri)) return false;
    if(v.discoveryUrl.empty()) return w.WriteInt32(-1); // discoveryUrls: null array
    return w.WriteInt32(1) && w.WriteString(v.discoveryUrl);
}

bool DecodeApplicationDescription(ByteReader &r, ApplicationDescription &out) {
    if(!r.ReadString(out.applicationUri)) return false;
    if(!r.ReadString(out.productUri)) return false;
    if(!DecodeLocalizedText(r, out.applicationName)) return false;
    if(!r.ReadInt32(out.applicationType)) return false;
    if(!r.ReadString(out.gatewayServerUri)) return false;
    if(!r.ReadString(out.discoveryProfileUri)) return false;
    return SkipStringArray(r); // discoveryUrls
}

bool EncodeUserTokenPolicy(ByteWriter &w, const UserTokenPolicy &v) {
    if(!w.WriteString(v.policyId)) return false;
    if(!w.WriteInt32(v.tokenType)) return false;
    if(!w.WriteString(v.issuedTokenType)) return false;
    if(!w.WriteString(v.issuerEndpointUrl)) return false;
    return w.WriteString(v.securityPolicyUri);
}

bool EncodeEndpointDescription(ByteWriter &w, const EndpointDescription &v) {
    if(!w.WriteString(v.endpointUrl)) return false;
    if(!EncodeApplicationDescription(w, v.server)) return false;
    if(!WriteServerCertificateChain(w)) return false; // leaf + CA, see secure_channel.hpp
    if(!w.WriteInt32(v.securityMode)) return false;
    if(!w.WriteString(v.securityPolicyUri)) return false;
    // userIdentityTokens: exactly one entry (Anonymous).
    if(!w.WriteInt32(1)) return false;
    if(!EncodeUserTokenPolicy(w, v.userIdentityToken)) return false;
    if(!w.WriteString(v.transportProfileUri)) return false;
    return w.WriteByte(v.securityLevel);
}

EndpointDescription BuildEndpoint(std::string_view endpointUrl, Int32 securityMode) {
    const bool secure = securityMode != SECURITY_MODE_NONE;
    EndpointDescription ep;
    ep.endpointUrl = String(endpointUrl);
    ep.server.applicationUri = String(APPLICATION_URI);
    ep.server.productUri = String(PRODUCT_URI);
    ep.server.applicationName = LocalizedText("en-US", APPLICATION_NAME);
    ep.server.applicationType = APPLICATION_TYPE_SERVER;
    ep.securityMode = securityMode;
    ep.securityPolicyUri = String(secure ? SECURITY_POLICY_BASIC256SHA256_URI : SECURITY_POLICY_NONE_URI);
    ep.userIdentityToken.policyId = String(ANONYMOUS_POLICY_ID);
    ep.userIdentityToken.tokenType = 0; // Anonymous
    ep.userIdentityToken.securityPolicyUri = String::Null();
    ep.transportProfileUri = String(TRANSPORT_PROFILE_URI);
    // higher is more secure (Part 4 7.14): SignAndEncrypt 2, Sign 1, None 0
    ep.securityLevel = static_cast<Byte>(securityMode - SECURITY_MODE_NONE);
    return ep;
}

bool EncodeEndpoints(ByteWriter &w, std::string_view endpointUrl) {
    return w.WriteInt32(3) && EncodeEndpointDescription(w, BuildEndpoint(endpointUrl, SECURITY_MODE_SIGN_AND_ENCRYPT)) &&
           EncodeEndpointDescription(w, BuildEndpoint(endpointUrl, SECURITY_MODE_SIGN)) &&
           EncodeEndpointDescription(w, BuildEndpoint(endpointUrl, SECURITY_MODE_NONE));
}

bool HandleFindServers(const RequestHeader &header, ByteReader &requestBody, std::string_view endpointUrl,
                       ByteWriter &w) {
    String requestedUrl;
    if(!requestBody.ReadString(requestedUrl)) return false;
    if(!SkipStringArray(requestBody)) return false; // LocaleIds
    Int32 uris = 0;
    bool ours = false;
    if(!requestBody.ReadInt32(uris)) return false;
    for(Int32 i = 0; i < uris; i++) {
        String uri;
        if(!requestBody.ReadString(uri)) return false;
        ours = ours || (!uri.isNull && uri.value == APPLICATION_URI);
    }
    if(!requestBody.Ok()) return false;
    if(!EncodeNodeId(w, NodeId(0, ns0::FindServersResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(header, StatusCode::Good))) return false;
    if(uris > 0 && !ours) return w.WriteInt32(0);
    ApplicationDescription self = BuildEndpoint(endpointUrl, SECURITY_MODE_NONE).server;
    self.discoveryUrl = (!requestedUrl.isNull && !requestedUrl.value.empty()) ? requestedUrl.value : endpointUrl;
    return w.WriteInt32(1) && EncodeApplicationDescription(w, self);
}

bool SkipStringArray(ByteReader &r) {
    Int32 count = 0;
    if(!r.ReadInt32(count)) return false;
    for(Int32 i = 0; i < count; i++) {
        String s;
        if(!r.ReadString(s)) return false;
    }
    return true;
}

}  // namespace rtos::opcua
