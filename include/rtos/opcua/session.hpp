#pragma once
// Session lifecycle (Part 4 5.6): CreateSession/ActivateSession/CloseSession. Anonymous
// authentication only (project decision -- "SecurityPolicy None" implies no certificate-based
// or username/password auth infrastructure either for this first version).
//
// One Session per Connection (see connection.hpp) -- this server does not support
// reactivating a session on a different SecureChannel/TCP connection (Part 4 5.6.3's
// "transfer" case), which is legal for a minimal server to omit. A client that reconnects
// simply creates a new session, same as if the old one had timed out.
//
// Unlike SecureChannel::HandleOpen (which frames its own OPN response, since the OPN/MSG
// distinction is handled one level up in connection.cpp), these Handle* methods write ONLY the
// response's TypeId + struct fields into an already-positioned ByteWriter -- connection.cpp
// wraps every regular (post-channel-open) service call in one shared
// BeginSymmetricMessage(...)/FinishSecureMessage(...) pair, so the framing boilerplate isn't
// repeated per service.
#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/service_structs.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

class Session {
public:
    bool IsActive() const { return isActive_; }
    bool IsCreated() const { return isCreated_; }
    const NodeId &AuthenticationToken() const { return authenticationToken_; }

    // connectionSlot: this connection's fixed index in the server's connection table -- used
    // to derive a SessionId/AuthenticationToken that's unique across the whole (small, fixed)
    // connection pool without needing a separate counter/allocator.
    bool HandleCreate(ByteReader &requestBody, UInt32 connectionSlot,
                      std::string_view endpointUrl, ByteWriter &w);
    // Validates the RequestHeader.authenticationToken (decoded by the caller, connection.cpp,
    // before dispatch -- see its comment on why) already matches; only the UserIdentityToken
    // is inspected here (must be Anonymous).
    bool HandleActivate(ByteReader &requestBody, ByteWriter &w);
    bool HandleClose(ByteReader &requestBody, ByteWriter &w);

private:
    bool isCreated_ = false;
    bool isActive_ = false;
    NodeId sessionId_;
    NodeId authenticationToken_;
};

}  // namespace rtos::opcua
