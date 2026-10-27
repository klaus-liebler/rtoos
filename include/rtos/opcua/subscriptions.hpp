#pragma once
// Subscriptions and MonitoredItems (Part 4 5.12, 5.13): data change notifications of variable
// attributes -- the "Standard DataChange Subscription" facet without events, aggregates and
// durable/transferable subscriptions.
//
// One Subscriptions object belongs to a session (connection.hpp). Everything is static: at most
// MAX_SUBSCRIPTIONS subscriptions with MAX_MONITORED_ITEMS items in all, a queue of up to
// MAX_QUEUE_SIZE values per item, MAX_PUBLISH_REQUESTS queued Publish requests, and per
// subscription the last NotificationMessage for Republish (the retransmission queue, depth 1).
//
// Timing runs on the 64-bit monotonic clock (clock.hpp). Tick() samples the items that are due
// (ReadAttribute of the node, compared per the DataChangeFilter) and decides per subscription
// and publishing interval: notifications -> a PublishResponse, nothing for MaxKeepAliveCount
// intervals -> a keep-alive, no Publish request for LifetimeCount intervals -> the
// subscription ends (StatusChangeNotification BadTimeout). Responses are written when the
// connection can send (Connection::Tick); Publish requests wait in the queue until then.
#include <array>
#include <span>

#include "rtos/opcua/address_space.hpp"
#include "rtos/opcua/byte_stream.hpp"
#include "rtos/opcua/service_header.hpp"
#include "rtos/opcua/services.hpp"
#include "rtos/opcua/types.hpp"
#include "rtos/opcua/variant.hpp"

namespace rtos::opcua {

class Subscriptions {
public:
    static constexpr size_t MAX_SUBSCRIPTIONS = 2;
    static constexpr size_t MAX_MONITORED_ITEMS = 12;  ///< per session, all subscriptions
    static constexpr UInt32 MAX_QUEUE_SIZE = 4;
    static constexpr size_t MAX_PUBLISH_REQUESTS = 4;
    static constexpr size_t MAX_ACKNOWLEDGEMENTS = 8;  ///< per Publish request
    static constexpr size_t MAX_TRIGGER_LINKS = 4;     ///< per triggering item
    static constexpr Int32 MAX_ITEMS_PER_CALL = static_cast<Int32>(MAX_MONITORED_ITEMS);
    static constexpr Double MIN_PUBLISHING_INTERVAL_MS = 50;
    static constexpr Double MAX_PUBLISHING_INTERVAL_MS = 3'600'000;
    static constexpr UInt32 DEFAULT_KEEPALIVE_COUNT = 10;
    static constexpr UInt32 MAX_KEEPALIVE_COUNT = 10'000;
    static constexpr UInt32 MAX_LIFETIME_COUNT = 30'000;
    static constexpr size_t MAX_MESSAGE_SIZE = 1024;  ///< one NotificationMessage (and its copy for Republish)

    // Part 4 7.20.1 MonitoringMode, 7.22.2 DataChangeTrigger, DeadbandType
    enum class MonitoringMode : Int32 { Disabled = 0, Sampling = 1, Reporting = 2 };
    enum class Trigger : UInt32 { Status = 0, StatusValue = 1, StatusValueTimestamp = 2 };
    enum class Deadband : UInt32 { None = 0, Absolute = 1, Percent = 2 };

    // ---- services (each writes the complete response, or a ServiceFault) ------------------
    bool HandleCreateSubscription(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleModifySubscription(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleSetPublishingMode(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleDeleteSubscriptions(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleCreateMonitoredItems(const ServiceContext &ctx, const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleModifyMonitoredItems(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleSetMonitoringMode(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleDeleteMonitoredItems(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleSetTriggering(const RequestHeader &h, ByteReader &r, ByteWriter &w);
    bool HandleRepublish(const RequestHeader &h, ByteReader &r, ByteWriter &w);

    /// A Publish request: acknowledgements now, the response later (PublishDue/WritePublish).
    /// Returns Good when queued, else the ServiceFault status to answer it with at once.
    StatusCode QueuePublish(const RequestHeader &h, UInt32 secureChannelRequestId, ByteReader &r);

    // ---- periodic -------------------------------------------------------------------------
    /// Sample due items and advance the publishing cycles (counters, expiry).
    void Tick(const ServiceContext &ctx);
    /// Is a response ready to be written (and a Publish request to carry it)?
    bool PublishDue() const;
    /// Write the next response (PublishResponse or ServiceFault of a timed-out request) and
    /// return the SecureChannel requestId it answers.
    bool WritePublish(ByteWriter &w, UInt32 &secureChannelRequestId);
    /// The next time (NowMonotonic) Tick() has something to do; 0: nothing scheduled.
    DateTime NextDeadline() const;

    /// End every subscription (in place: no temporary of this size on the network stack).
    void Clear();
    size_t SubscriptionCount() const;
    size_t MonitoredItemCount() const;

private:
    struct Sample {
        Variant value;
        StatusCode status = StatusCode::Good;
        DateTime source = 0, server = 0;
    };
    struct MonitoredItem {
        bool used = false;
        UInt32 id = 0;
        UInt32 subscriptionId = 0;
        const Node *node = nullptr;
        UInt32 attributeId = 0;
        UInt32 clientHandle = 0;
        MonitoringMode mode = MonitoringMode::Reporting;
        TimestampsToReturn timestamps = TimestampsToReturn::Both;
        Double samplingInterval = 0;
        UInt32 queueSize = 1;
        bool discardOldest = true;
        Trigger trigger = Trigger::StatusValue;
        Deadband deadband = Deadband::None;
        Double deadbandValue = 0;
        DateTime nextSample = 0;
        bool sampled = false;  // lastValue/lastStatus valid
        Variant lastValue;
        StatusCode lastStatus = StatusCode::Good;
        std::array<Sample, MAX_QUEUE_SIZE> queue{};
        UInt32 queued = 0;
        std::array<UInt32, MAX_TRIGGER_LINKS> links{};
        UInt32 linkCount = 0;
        bool triggered = false;  // a linked Sampling item reports with its triggering item
    };
    struct Subscription {
        bool used = false;
        UInt32 id = 0;
        Double interval = 0;
        UInt32 lifetimeCount = 0, keepAliveCount = 0, maxNotifications = 0;
        Byte priority = 0;
        bool enabled = true;
        DateTime nextCycle = 0;
        UInt32 lifetimeCounter = 0, keepAliveCounter = 0;
        UInt32 nextSequence = 1;
        bool firstCycle = true;   // the first cycle always publishes (initial values or keep-alive)
        bool dataDue = false;     // notifications ready, waiting for a Publish request
        bool keepAliveDue = false;
        StatusCode statusChange = StatusCode::Good;  // BadTimeout: ends with this notification
        std::array<Byte, MAX_MESSAGE_SIZE> retransmit{};
        size_t retransmitSize = 0;
        UInt32 retransmitSequence = 0;  // 0: nothing retained
    };
    struct PublishRequest {
        UInt32 requestId = 0;  // SecureChannel request id
        RequestHeader header;
        DateTime received = 0;
        std::array<StatusCode, MAX_ACKNOWLEDGEMENTS> ackResults{};
        UInt32 acks = 0;
    };

    static void Reset(Subscription &s);  // in place, the retransmission buffer is not copied
    Subscription *FindSubscription(UInt32 id);
    MonitoredItem *FindItem(UInt32 subscriptionId, UInt32 itemId);
    void DeleteSubscription(Subscription &s);
    void DeleteItem(MonitoredItem &item);
    static void Revise(Subscription &s, Double interval, UInt32 lifetime, UInt32 keepAlive);
    Double ReviseSampling(const Subscription &s, Double requested) const;
    StatusCode ApplyParameters(MonitoredItem &item, const Subscription &s, ByteReader &r, Double &samplingOut,
                               UInt32 &queueOut, bool &decodeFailed);
    void SampleItem(const ServiceContext &ctx, MonitoredItem &item, DateTime now);
    void Enqueue(MonitoredItem &item, const Sample &sample);
    bool HasNotifications(const Subscription &s) const;
    bool WriteNotificationMessage(ByteWriter &w, Subscription &s, bool keepAlive, DateTime now, bool &more);

    std::array<Subscription, MAX_SUBSCRIPTIONS> subscriptions_{};
    std::array<MonitoredItem, MAX_MONITORED_ITEMS> items_{};
    std::array<PublishRequest, MAX_PUBLISH_REQUESTS> publish_{};
    size_t publishCount_ = 0;  // FIFO: publish_[0] is the oldest
    UInt32 nextSubscriptionId_ = 0, nextItemId_ = 0;
};

}  // namespace rtos::opcua
