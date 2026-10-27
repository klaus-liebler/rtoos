#pragma once
// Session (Part 4 5.6): one per connection, bound to that connection's SecureChannel.
//
// rtoos:
//   * every service except CreateSession/ActivateSession/GetEndpoints carries the session's
//     AuthenticationToken; Check() answers it (unknown, closed or timed-out session:
//     BadSessionIdInvalid; created but not activated: BadSessionNotActivated)
//   * a session that sees no request for its revised timeout is closed (Part 4 5.6.2)
//   * on a secured channel, CreateSession requires a ClientNonce of >= 32 bytes and the
//     channel's own client certificate; ActivateSession requires the ClientSignature over
//     serverCertificate || serverNonce (Part 4 5.6.3), made with that certificate's key
//   * identity: Anonymous only (the one UserTokenPolicy GetEndpoints offers, PolicyId
//     "anonymous"); a null token counts as Anonymous
#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/security_crypto.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/service_structs.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

class SecureChannel;

class Session {
public:
    static constexpr Double MIN_TIMEOUT_MS = 10'000;
    static constexpr Double MAX_TIMEOUT_MS = 3'600'000;
    static constexpr Double DEFAULT_TIMEOUT_MS = 60'000;
    static constexpr size_t NONCE_SIZE = 32;

    bool IsActive() const { return isActive_; }
    bool IsCreated() const { return isCreated_; }
    const NodeId &AuthenticationToken() const { return authenticationToken_; }

    /// Is `authenticationToken` this (activated, not timed-out) session? Counts as activity.
    StatusCode Check(const NodeId &authenticationToken);
    /// Close the session if its timeout passed without a request (also done by Check()).
    void Expire();

    // connectionSlot: this connection's fixed index in the server's connection table -- used
    // to derive a SessionId/AuthenticationToken that's unique across the whole (small, fixed)
    // connection pool without needing a separate counter/allocator.
    bool HandleCreate(const RequestHeader &header, ByteReader &requestBody, UInt32 connectionSlot,
                      std::string_view endpointUrl, const SecureChannel &channel, ByteWriter &w);
    bool HandleActivate(const RequestHeader &header, ByteReader &requestBody, const SecureChannel &channel,
                        ByteWriter &w);
    bool HandleClose(const RequestHeader &header, ByteReader &requestBody, ByteWriter &w);

private:
    void Close() { isCreated_ = isActive_ = false; }

    bool isCreated_ = false;
    bool isActive_ = false;
    NodeId sessionId_;
    NodeId authenticationToken_;
    UInt32 tokenCounter_ = 0;  ///< a new token per CreateSession on this connection
    Double timeoutMs_ = DEFAULT_TIMEOUT_MS;
    DateTime lastActivity_ = 0;  ///< NowMonotonic() of the last request
    Byte serverNonce_[NONCE_SIZE] = {};
};

}  // namespace rtos::opcua
