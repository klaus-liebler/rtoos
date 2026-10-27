#pragma once
// Abstraction the protocol/service layer (connection.hpp, secure_channel.hpp, session.hpp)
// sends framed UA TCP messages through, independent of the network stack. rtoos: the one
// implementation is the session of rtos::net::OpcUaServer (rtos/net/opcua_server.hpp).
#include <span>

#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

class ITransport {
public:

    // Sends exactly one complete, already-framed UA TCP message (its own 8-byte
    // MessageType/ChunkType/MessageSize header included). May be split across multiple
    // underlying packets by the implementation, but must be delivered as one ordered byte
    // range on the TCP stream -- callers never call Send() twice for what must arrive as a
    // single message.
    virtual bool Send(std::span<const Byte> data) = 0;

    // Requests the underlying TCP connection be torn down (e.g. after CloseSecureChannel, or
    // a fatal decode/protocol error). Asynchronous: no further Send() calls will be made
    // afterward, but the connection object itself is only reset once the transport's own
    // disconnect callback fires.
    virtual void RequestClose() = 0;

protected:
    // Never deleted through this interface: no virtual (deleting) destructor, which would
    // pull operator delete into a heap-free image.
    ~ITransport() = default;
};

}  // namespace rtos::opcua
