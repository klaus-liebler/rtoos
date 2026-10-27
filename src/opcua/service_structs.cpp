#include "rtos/opcua/service_structs.hpp"

#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/secure_channel.hpp"

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
    return w.WriteInt32(-1); // discoveryUrls: null array
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

EndpointDescription BuildEndpoint(std::string_view endpointUrl, bool secure) {
    EndpointDescription ep;
    ep.endpointUrl = String(endpointUrl);
    ep.server.applicationUri = String(APPLICATION_URI);
    ep.server.productUri = String(PRODUCT_URI);
    ep.server.applicationName = LocalizedText("en-US", APPLICATION_NAME);
    ep.server.applicationType = APPLICATION_TYPE_SERVER;
    ep.securityMode = secure ? SECURITY_MODE_SIGN : SECURITY_MODE_NONE;
    ep.securityPolicyUri = String(secure ? SECURITY_POLICY_BASIC256SHA256_URI : SECURITY_POLICY_NONE_URI);
    ep.userIdentityToken.policyId = String(ANONYMOUS_POLICY_ID);
    ep.userIdentityToken.tokenType = 0; // Anonymous
    ep.userIdentityToken.securityPolicyUri = String::Null();
    ep.transportProfileUri = String(TRANSPORT_PROFILE_URI);
    ep.securityLevel = secure ? 1 : 0;
    return ep;
}

bool EncodeEndpoints(ByteWriter &w, std::string_view endpointUrl) {
    return w.WriteInt32(2) && EncodeEndpointDescription(w, BuildEndpoint(endpointUrl, true)) &&
           EncodeEndpointDescription(w, BuildEndpoint(endpointUrl, false));
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
