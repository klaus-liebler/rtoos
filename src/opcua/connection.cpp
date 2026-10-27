#include "rtos/opcua/connection.hpp"

#include <algorithm>

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"
#include "rtos/opcua/services.hpp"
#include "rtos/opcua/tcp_message.hpp"
#include "rtos/opcua/log.hpp"

namespace rtos::opcua {

void Connection::Reset(UInt32 connectionSlot, ITransport *transport, const AddressSpace *addressSpace,
                       std::string_view endpointUrl) {
    connectionSlot_ = connectionSlot;
    transport_ = transport;
    addressSpace_ = addressSpace;
    endpointUrl_ = endpointUrl;
    helloReceived_ = false;
    secureChannel_ = SecureChannel{};
    session_ = Session{};
    view_.Clear();
    subscriptions_.Clear();
}

void Connection::Tick() {
    if(!secureChannel_.IsOpen()) return;
    session_.Expire();
    if(!session_.IsActive()) {
        // subscriptions belong to the session: they end with it (no transfer, Part 4 5.13.7)
        if(subscriptions_.SubscriptionCount() != 0) subscriptions_.Clear();
        return;
    }
    const ServiceContext ctx{*addressSpace_, secureChannel_.IsUnsecured()};
    subscriptions_.Tick(ctx);
    while(transport_->CanSend() && subscriptions_.PublishDue()) {
        ByteWriter w(sendBuffer_);
        size_t startPos = 0;
        UInt32 requestId = 0;
        // the RequestId is known once the response is chosen: patched into the sequence header
        if(!BeginSymmetricMessage(w, "MSG", secureChannel_.ChannelId(), secureChannel_.TokenId(),
                                  secureChannel_.NextServerSequenceNumber(), 0, startPos) ||
           !subscriptions_.WritePublish(w, requestId) || !w.PatchUInt32(startPos + 20, requestId) ||
           !secureChannel_.SignAndFinishSymmetric(w, startPos) || !transport_->Send(w.Written())) {
            transport_->RequestClose();
            return;
        }
    }
}

DateTime Connection::NextDeadline() const {
    return session_.IsActive() ? subscriptions_.NextDeadline() : 0;
}

void Connection::SendErrorAndClose(StatusCode error, std::string_view reason) {
    log_debug("OPC UA: slot %u closing: %.*s (0x%08X)", connectionSlot_,
             static_cast<int>(reason.size()), reason.data(), static_cast<unsigned>(error));
    ByteWriter w(sendBuffer_);
    if(EncodeErrorMessage(w, error, reason) && w.Ok())
        transport_->Send(w.Written());
    transport_->RequestClose();
}

void Connection::HandleMessage(std::span<const Byte> msg) {
    TcpHeader header;
    if(!PeekTcpHeader(msg, header)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed message header");
        return;
    }

    if(!helloReceived_) {
        if(header.Is("HEL")) { HandleHello(msg); return; }
        SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "expected Hello");
        return;
    }
    if(header.Is("HEL")) {
        SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "duplicate Hello");
        return;
    }
    if(header.Is("OPN")) { HandleOpenSecureChannel(msg); return; }
    if(header.Is("CLO")) { HandleCloseSecureChannel(msg); return; }
    if(header.Is("MSG")) {
        if(!secureChannel_.IsOpen()) {
            SendErrorAndClose(StatusCode::BadSecureChannelClosed, "no open SecureChannel");
            return;
        }
        HandleServiceMessage(msg);
        return;
    }
    SendErrorAndClose(StatusCode::BadTcpMessageTypeInvalid, "unknown message type");
}

void Connection::HandleHello(std::span<const Byte> msg) {
    ByteReader r(msg);
    TcpHeader header;
    if(!DecodeTcpHeader(r, header)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed Hello header");
        return;
    }
    HelloMessage hello;
    if(!DecodeHelloBody(r, hello) || !r.Ok() || r.Remaining() != 0) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed Hello body");
        return;
    }
    // Part 6 7.1.2.3: the EndpointUrl is shorter than 4096 bytes
    if(!hello.endpointUrl.isNull && hello.endpointUrl.value.size() >= 4096) {
        SendErrorAndClose(StatusCode::BadTcpEndpointUrlInvalid, "EndpointUrl too long");
        return;
    }

    helloReceived_ = true;

    AcknowledgeMessage ack;
    ack.protocolVersion = PROTOCOL_VERSION;
    // Part 6 7.1.2.4: never more than the client can take (its ReceiveBufferSize), never more
    // than it announced to send; 8192 is the minimum every party must support.
    if(hello.receiveBufferSize < 8192 || hello.sendBufferSize < 8192) {
        SendErrorAndClose(StatusCode::BadTcpMessageTooLarge, "buffer sizes below 8192");
        return;
    }
    ack.receiveBufferSize = static_cast<UInt32>(std::min<size_t>(SEND_BUFFER_SIZE, hello.sendBufferSize));
    ack.sendBufferSize = static_cast<UInt32>(std::min<size_t>(SEND_BUFFER_SIZE, hello.receiveBufferSize));
    ack.maxMessageSize = static_cast<UInt32>(SEND_BUFFER_SIZE);
    // Single-chunk-only transport (see the project's scope notes / secure_channel.hpp) -- a
    // compliant client is required to honor this and never send a multi-chunk request.
    ack.maxChunkCount = 1;

    ByteWriter w(sendBuffer_);
    if(!EncodeAcknowledge(w, ack) || !w.Ok() || !transport_->Send(w.Written()))
        transport_->RequestClose();
}

void Connection::HandleOpenSecureChannel(std::span<const Byte> msg) {
    // Assigns a fresh ChannelId on first Open, keeps the existing one across a Renew --
    // (connectionSlot_ + 1) is guaranteed unique among this server's small, fixed connection
    // table (see the future net_transport.hpp wiring) without needing a separate allocator.
    UInt32 channelId = secureChannel_.IsOpen() ? secureChannel_.ChannelId() : (connectionSlot_ + 1);

    ByteWriter w(sendBuffer_);
    if(!secureChannel_.HandleOpen(msg, channelId, w) || !w.Ok()) {
        SendErrorAndClose(StatusCode::BadSecurityChecksFailed, "OpenSecureChannel handling failed");
        return;
    }
    if(!transport_->Send(w.Written()))
        transport_->RequestClose();
}

void Connection::HandleCloseSecureChannel(std::span<const Byte> msg) {
    // Best-effort: a CloseSecureChannelRequest that doesn't even verify still results in the
    // connection closing (Part 4 5.5.3: there is no response either way).
    secureChannel_.HandleClose(msg);
    transport_->RequestClose();
}

void Connection::HandleServiceMessage(std::span<const Byte> msg) {
    ParsedSecureMessage parsed;
    if(!secureChannel_.VerifyAndUnwrapSymmetric(msg, parsed)) {
        SendErrorAndClose(StatusCode::BadSecurityChecksFailed, "malformed or unsigned service message");
        return;
    }
    if(parsed.channelId != secureChannel_.ChannelId() || parsed.tokenId != secureChannel_.TokenId()) {
        log_debug("OPC UA: slot %u channel/token mismatch: got channel=%u token=%u, expected channel=%u token=%u",
                 connectionSlot_, static_cast<unsigned>(parsed.channelId), static_cast<unsigned>(parsed.tokenId),
                 static_cast<unsigned>(secureChannel_.ChannelId()), static_cast<unsigned>(secureChannel_.TokenId()));
        SendErrorAndClose(StatusCode::BadSecureChannelIdInvalid, "channel/token mismatch");
        return;
    }

    ByteReader body(parsed.body);
    NodeId typeId;
    if(!DecodeNodeId(body, typeId)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed service TypeId");
        return;
    }
    log_debug("OPC UA: slot %u service typeId=(ns=%u,i=%u)", connectionSlot_,
             static_cast<unsigned>(typeId.namespaceIndex), static_cast<unsigned>(typeId.numeric));

    // Every request starts with a RequestHeader (Part 4 7.28), unknown services included.
    RequestHeader header;
    if(!DecodeRequestHeader(body, header)) {
        SendErrorAndClose(StatusCode::BadDecodingError, "malformed RequestHeader");
        return;
    }

    // Publish: queued, answered later by Tick() -- unless it fails at once (no sequence number
    // is used for a request without an immediate response).
    StatusCode publishFault = StatusCode::Good;
    if(typeId == NodeId(0, ns0::PublishRequest)) {
        publishFault = session_.Check(header.authenticationToken);
        if(IsGood(publishFault)) publishFault = subscriptions_.QueuePublish(header, parsed.requestId, body);
        if(publishFault == StatusCode::BadDecodingError) {
            SendErrorAndClose(StatusCode::BadDecodingError, "malformed PublishRequest");
            return;
        }
        if(IsGood(publishFault)) return;
    }

    ByteWriter w(sendBuffer_);
    size_t startPos = 0;
    if(!BeginSymmetricMessage(w, "MSG", secureChannel_.ChannelId(), secureChannel_.TokenId(),
                              secureChannel_.NextServerSequenceNumber(), parsed.requestId, startPos)) {
        transport_->RequestClose();
        return;
    }

    const auto is = [&](UInt32 id) { return typeId == NodeId(0, id); };
    const ServiceContext ctx{*addressSpace_, secureChannel_.IsUnsecured()};
    bool handled;
    if(is(ns0::PublishRequest)) {
        handled = EncodeServiceFault(w, header, publishFault);
    } else if(is(ns0::GetEndpointsRequest)) {
        handled = HandleGetEndpoints(header, body, endpointUrl_, w);
    } else if(is(ns0::FindServersRequest)) {
        handled = HandleFindServers(header, body, endpointUrl_, w);
    } else if(is(ns0::CreateSessionRequest)) {
        view_.Clear();
        subscriptions_.Clear();
        handled = session_.HandleCreate(header, body, connectionSlot_, endpointUrl_, secureChannel_, w);
    } else if(is(ns0::ActivateSessionRequest)) {
        handled = session_.HandleActivate(header, body, secureChannel_, w);
    } else if(is(ns0::CloseSessionRequest)) {
        view_.Clear();
        subscriptions_.Clear();
        handled = session_.HandleClose(header, body, w);
    } else if(const StatusCode s = session_.Check(header.authenticationToken); !IsGood(s)) {
        // every other service needs the activated session of this connection
        handled = EncodeServiceFault(w, header, s);
    } else if(is(ns0::WriteRequest) && secureChannel_.IsUnsecured()) {
        // rtoos: a SecurityPolicy#None channel is read-only (see secure_channel.hpp's file
        // header comment) -- writing needs a signed channel of a trusted client.
        handled = EncodeServiceFault(w, header, StatusCode::BadSecurityModeInsufficient);
    } else if(is(ns0::ReadRequest)) {
        handled = HandleRead(ctx, header, body, w);
    } else if(is(ns0::WriteRequest)) {
        handled = HandleWrite(ctx, header, body, w);
    } else if(is(ns0::BrowseRequest)) {
        handled = HandleBrowse(ctx, view_, header, body, w);
    } else if(is(ns0::BrowseNextRequest)) {
        handled = HandleBrowseNext(ctx, view_, header, body, w);
    } else if(is(ns0::TranslateBrowsePathsToNodeIdsRequest)) {
        handled = HandleTranslateBrowsePaths(ctx, header, body, w);
    } else if(is(ns0::RegisterNodesRequest) || is(ns0::UnregisterNodesRequest)) {
        handled = HandleRegisterNodes(header, body, w, is(ns0::UnregisterNodesRequest));
    } else if(is(ns0::CreateSubscriptionRequest)) {
        handled = subscriptions_.HandleCreateSubscription(header, body, w);
    } else if(is(ns0::ModifySubscriptionRequest)) {
        handled = subscriptions_.HandleModifySubscription(header, body, w);
    } else if(is(ns0::SetPublishingModeRequest)) {
        handled = subscriptions_.HandleSetPublishingMode(header, body, w);
    } else if(is(ns0::DeleteSubscriptionsRequest)) {
        handled = subscriptions_.HandleDeleteSubscriptions(header, body, w);
    } else if(is(ns0::CreateMonitoredItemsRequest)) {
        handled = subscriptions_.HandleCreateMonitoredItems(ctx, header, body, w);
    } else if(is(ns0::ModifyMonitoredItemsRequest)) {
        handled = subscriptions_.HandleModifyMonitoredItems(header, body, w);
    } else if(is(ns0::SetMonitoringModeRequest)) {
        handled = subscriptions_.HandleSetMonitoringMode(header, body, w);
    } else if(is(ns0::DeleteMonitoredItemsRequest)) {
        handled = subscriptions_.HandleDeleteMonitoredItems(header, body, w);
    } else if(is(ns0::SetTriggeringRequest)) {
        handled = subscriptions_.HandleSetTriggering(header, body, w);
    } else if(is(ns0::RepublishRequest)) {
        handled = subscriptions_.HandleRepublish(header, body, w);
    } else {
        // a service this server does not implement (Part 4 7.33: ServiceFault)
        handled = EncodeServiceFault(w, header, StatusCode::BadServiceUnsupported);
    }

    if(!handled || !w.Ok() || !secureChannel_.SignAndFinishSymmetric(w, startPos)) {
        log_debug("OPC UA: slot %u handler failed for typeId=(ns=%u,i=%u) handled=%d w.Ok=%d",
                 connectionSlot_, static_cast<unsigned>(typeId.namespaceIndex),
                 static_cast<unsigned>(typeId.numeric), handled ? 1 : 0, w.Ok() ? 1 : 0);
        transport_->RequestClose();
        return;
    }
    if(!transport_->Send(w.Written())) {
        log_debug("OPC UA: slot %u Send() failed for typeId=(ns=%u,i=%u)", connectionSlot_,
                 static_cast<unsigned>(typeId.namespaceIndex), static_cast<unsigned>(typeId.numeric));
        transport_->RequestClose();
    }
}

}  // namespace rtos::opcua
