#pragma once
// Per-TCP-connection state machine: Hello handshake -> SecureChannel (Open/Close) -> Session
// (Create/Activate/Close) -> Read/Write/Browse/GetEndpoints, all driven purely by
// HandleMessage() receiving one complete UA TCP message at a time (see transport.hpp/ITransport
// for how responses go back out, and the future net_transport.hpp/.cpp for how "one complete
// message" gets assembled from NX_TCPSERVER's byte stream).
//
// One Connection == one TCP connection == at most one SecureChannel == at most one Session
// (see session.hpp for why that's an acceptable simplification for a minimal server).
#include <array>
#include <span>
#include <string_view>

#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/secure_channel.hpp"
#include "rtos/opcua/services.hpp"
#include "rtos/opcua/subscriptions.hpp"
#include "rtos/opcua/session.hpp"
#include "rtos/opcua/transport.hpp"
#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

class Connection {
public:
    // Rebinds this (reused, e.g. from a fixed connection-table slot) Connection object to a
    // fresh TCP connection -- must be called once before the first HandleMessage().
    // endpointUrl must outlive this Connection (points at a server-owned, unchanging string).
    void Reset(UInt32 connectionSlot, ITransport *transport, const AddressSpace *addressSpace,
              std::string_view endpointUrl);

    // Processes exactly one complete UA TCP message (header.messageSize == msg.size(),
    // ChunkType checked internally). May call transport->Send()/RequestClose() any number of
    // times (typically 0 or 1 Send() and at most one RequestClose()).
    void HandleMessage(std::span<const Byte> msg);
    /// Periodic work: sampling, publishing cycles, Publish responses (when the transport can send).
    void Tick();
    /// NowMonotonic() of the next Tick() with work, 0: none.
    DateTime NextDeadline() const;

    // Result of the last client certificate check (OpenSecureChannel, Basic256Sha256).
    security::CertificateCheck LastCertificateCheck() const { return secureChannel_.LastCertificateCheck(); }

private:
    void HandleHello(std::span<const Byte> msg);
    void HandleOpenSecureChannel(std::span<const Byte> msg);
    void HandleCloseSecureChannel(std::span<const Byte> msg);
    void HandleServiceMessage(std::span<const Byte> msg);
    void SendErrorAndClose(StatusCode error, std::string_view reason);

    UInt32 connectionSlot_ = 0;
    ITransport *transport_ = nullptr;
    const AddressSpace *addressSpace_ = nullptr;
    std::string_view endpointUrl_;

    bool helloReceived_ = false;
    SecureChannel secureChannel_;
    Session session_;
    ViewState view_;  // browse continuation points of the session
    Subscriptions subscriptions_;  // the session's subscriptions

    // Scratch buffer every response is built into before handing it to transport_->Send() --
    // one per connection so concurrent connections never share/race on it (NX_TCPSERVER
    // serializes all its callbacks on one thread anyway, but keeping this non-shared avoids
    // relying on that as an invariant here).
    static constexpr size_t SEND_BUFFER_SIZE = 8192;
    std::array<Byte, SEND_BUFFER_SIZE> sendBuffer_{};
};

}  // namespace rtos::opcua
