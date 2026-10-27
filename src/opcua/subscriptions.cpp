#include "rtos/opcua/subscriptions.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/codec.hpp"
#include "rtos/opcua/node_ids.hpp"

namespace rtos::opcua {

namespace {

constexpr DateTime MS = DATETIME_SEC / 1000;  // DateTime ticks (100 ns) per millisecond
// Part 4 7.20.1 / Part 6: InfoType DataValue (0x400) + Overflow (0x80) on a Good status
constexpr UInt32 OVERFLOW_BITS = 0x0000'0480u;

DateTime Ticks(Double ms) { return static_cast<DateTime>(ms * static_cast<Double>(MS)); }

StatusCode CheckCount(Int32 count, Int32 max) {
    if(count <= 0) return StatusCode::BadNothingToDo;
    return count > max ? StatusCode::BadTooManyOperations : StatusCode::Good;
}

bool WriteStatusArrayHeader(ByteWriter &w, const RequestHeader &h, UInt32 typeId, Int32 count) {
    return EncodeNodeId(w, NodeId(0, typeId)) && EncodeResponseHeader(w, MakeResponseHeader(h, StatusCode::Good)) &&
           w.WriteInt32(count);
}

bool IsNumeric(const Variant &v) {
    return std::visit([](auto &&x) {
        using T = std::decay_t<decltype(x)>;
        return std::is_same_v<T, SByte> || std::is_same_v<T, Byte> || std::is_same_v<T, Int16> ||
               std::is_same_v<T, UInt16> || std::is_same_v<T, Int32> || std::is_same_v<T, UInt32> ||
               std::is_same_v<T, Int64> || std::is_same_v<T, UInt64> || std::is_same_v<T, Float> ||
               std::is_same_v<T, Double>;
    }, v.storage);
}

Double AsDouble(const Variant &v) {
    return std::visit([](auto &&x) -> Double {
        using T = std::decay_t<decltype(x)>;
        if constexpr(std::is_arithmetic_v<T> && !std::is_same_v<T, Boolean>) return static_cast<Double>(x);
        else return 0;
    }, v.storage);
}

bool IsNumericDataType(const AddressSpace &as, const NodeId &type) {
    // the numeric built-in types are subtypes of Number (i=26)
    for(NodeId t = type; !t.IsNull();) {
        if(t == NodeId(0, ns0::DataType_Number)) return true;
        NodeId super;
        as.ForEachReference(t, false, [&](const ReferenceEntry &r) {
            if(r.referenceTypeId == NodeId(0, ns0::HasSubtype)) super = r.targetId;
        });
        t = super;
    }
    return false;
}

} // namespace

// ---- bookkeeping --------------------------------------------------------------------------------

void Subscriptions::Reset(Subscription &s) {
    s.used = false;
    s.id = 0;
    s.interval = 0;
    s.lifetimeCount = s.keepAliveCount = s.maxNotifications = 0;
    s.priority = 0;
    s.enabled = true;
    s.nextCycle = 0;
    s.lifetimeCounter = s.keepAliveCounter = 0;
    s.nextSequence = 1;
    s.firstCycle = true;
    s.dataDue = s.keepAliveDue = false;
    s.statusChange = StatusCode::Good;
    s.retransmitSize = 0;
    s.retransmitSequence = 0;
}

void Subscriptions::Clear() {
    for(auto &s : subscriptions_) Reset(s);
    for(auto &i : items_) i = MonitoredItem{};
    publishCount_ = 0;
}

Subscriptions::Subscription *Subscriptions::FindSubscription(UInt32 id) {
    for(auto &s : subscriptions_)
        if(s.used && s.id == id) return &s;
    return nullptr;
}

Subscriptions::MonitoredItem *Subscriptions::FindItem(UInt32 subscriptionId, UInt32 itemId) {
    for(auto &i : items_)
        if(i.used && i.subscriptionId == subscriptionId && i.id == itemId) return &i;
    return nullptr;
}

size_t Subscriptions::SubscriptionCount() const {
    return static_cast<size_t>(std::count_if(subscriptions_.begin(), subscriptions_.end(), [](auto &s) { return s.used; }));
}
size_t Subscriptions::MonitoredItemCount() const {
    return static_cast<size_t>(std::count_if(items_.begin(), items_.end(), [](auto &i) { return i.used; }));
}

void Subscriptions::DeleteItem(MonitoredItem &item) {
    const UInt32 id = item.id, sub = item.subscriptionId;
    item = MonitoredItem{};
    // links to the deleted item go with it (Part 4 5.12.5)
    for(auto &other : items_) {
        if(!other.used || other.subscriptionId != sub) continue;
        auto end = std::remove(other.links.begin(), other.links.begin() + other.linkCount, id);
        other.linkCount = static_cast<UInt32>(end - other.links.begin());
    }
}

void Subscriptions::DeleteSubscription(Subscription &s) {
    for(auto &i : items_)
        if(i.used && i.subscriptionId == s.id) i = MonitoredItem{};
    Reset(s);
}

// Part 4 5.13.2.2: interval within the server's limits, lifetime >= 3 x keep-alive.
void Subscriptions::Revise(Subscription &s, Double interval, UInt32 lifetime, UInt32 keepAlive) {
    s.interval = (std::isnan(interval) || interval < MIN_PUBLISHING_INTERVAL_MS)
                     ? MIN_PUBLISHING_INTERVAL_MS
                     : std::min(interval, MAX_PUBLISHING_INTERVAL_MS);
    s.keepAliveCount = keepAlive == 0 ? DEFAULT_KEEPALIVE_COUNT : std::min(keepAlive, MAX_KEEPALIVE_COUNT);
    s.lifetimeCount = std::min(std::max(lifetime, 3 * s.keepAliveCount), MAX_LIFETIME_COUNT);
}

// Part 4 5.12.1.2: -1 -> the publishing interval; below the fastest rate -> the fastest rate.
Double Subscriptions::ReviseSampling(const Subscription &s, Double requested) const {
    if(std::isnan(requested) || requested < 0) return s.interval;
    return std::max(requested, MIN_SAMPLING_INTERVAL_MS);
}

// ---- subscription services ------------------------------------------------------------------------

bool Subscriptions::HandleCreateSubscription(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    Double interval = 0;
    UInt32 lifetime = 0, keepAlive = 0, maxNotifications = 0;
    Boolean enabled = false;
    Byte priority = 0;
    if(!r.ReadDouble(interval) || !r.ReadUInt32(lifetime) || !r.ReadUInt32(keepAlive) ||
       !r.ReadUInt32(maxNotifications) || !r.ReadBoolean(enabled) || !r.ReadByte(priority))
        return false;
    Subscription *s = nullptr;
    for(auto &candidate : subscriptions_)
        if(!candidate.used) { s = &candidate; break; }
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadTooManySubscriptions);
    Reset(*s);
    s->used = true;
    s->id = ++nextSubscriptionId_;
    Revise(*s, interval, lifetime, keepAlive);
    s->maxNotifications = maxNotifications;
    s->priority = priority;
    s->enabled = enabled;
    s->nextCycle = NowMonotonic() + Ticks(s->interval);
    return EncodeNodeId(w, NodeId(0, ns0::CreateSubscriptionResponse)) &&
           EncodeResponseHeader(w, MakeResponseHeader(h, StatusCode::Good)) && w.WriteUInt32(s->id) &&
           w.WriteDouble(s->interval) && w.WriteUInt32(s->lifetimeCount) && w.WriteUInt32(s->keepAliveCount);
}

bool Subscriptions::HandleModifySubscription(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    UInt32 id = 0, lifetime = 0, keepAlive = 0, maxNotifications = 0;
    Double interval = 0;
    Byte priority = 0;
    if(!r.ReadUInt32(id) || !r.ReadDouble(interval) || !r.ReadUInt32(lifetime) || !r.ReadUInt32(keepAlive) ||
       !r.ReadUInt32(maxNotifications) || !r.ReadByte(priority))
        return false;
    Subscription *s = FindSubscription(id);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    Revise(*s, interval, lifetime, keepAlive);
    s->maxNotifications = maxNotifications;
    s->priority = priority;
    s->lifetimeCounter = 0;
    return EncodeNodeId(w, NodeId(0, ns0::ModifySubscriptionResponse)) &&
           EncodeResponseHeader(w, MakeResponseHeader(h, StatusCode::Good)) && w.WriteDouble(s->interval) &&
           w.WriteUInt32(s->lifetimeCount) && w.WriteUInt32(s->keepAliveCount);
}

bool Subscriptions::HandleSetPublishingMode(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    Boolean enabled = false;
    Int32 count = 0;
    if(!r.ReadBoolean(enabled) || !r.ReadInt32(count)) return false;
    if(const StatusCode s = CheckCount(count, static_cast<Int32>(MAX_SUBSCRIPTIONS) * 8); !IsGood(s))
        return EncodeServiceFault(w, h, s);
    if(!WriteStatusArrayHeader(w, h, ns0::SetPublishingModeResponse, count)) return false;
    for(Int32 i = 0; i < count; i++) {
        UInt32 id = 0;
        if(!r.ReadUInt32(id)) return false;
        Subscription *s = FindSubscription(id);
        if(s != nullptr) s->enabled = enabled;
        if(!w.WriteStatusCode(s != nullptr ? StatusCode::Good : StatusCode::BadSubscriptionIdInvalid)) return false;
    }
    return w.WriteInt32(-1);
}

bool Subscriptions::HandleDeleteSubscriptions(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    Int32 count = 0;
    if(!r.ReadInt32(count)) return false;
    if(const StatusCode s = CheckCount(count, static_cast<Int32>(MAX_SUBSCRIPTIONS) * 8); !IsGood(s))
        return EncodeServiceFault(w, h, s);
    if(!WriteStatusArrayHeader(w, h, ns0::DeleteSubscriptionsResponse, count)) return false;
    for(Int32 i = 0; i < count; i++) {
        UInt32 id = 0;
        if(!r.ReadUInt32(id)) return false;
        Subscription *s = FindSubscription(id);
        if(s != nullptr) DeleteSubscription(*s);
        if(!w.WriteStatusCode(s != nullptr ? StatusCode::Good : StatusCode::BadSubscriptionIdInvalid)) return false;
    }
    // Publish requests outlive their subscriptions; without any, they fail (Part 4 5.13.5)
    return w.WriteInt32(-1);
}

// ---- monitored items ----------------------------------------------------------------------------

// MonitoringParameters (Part 4 7.21): clientHandle, samplingInterval, filter, queueSize,
// discardOldest. The filter is a DataChangeFilter (or null); others are not supported.
StatusCode Subscriptions::ApplyParameters(MonitoredItem &item, const Subscription &s, ByteReader &r, Double &samplingOut,
                                          UInt32 &queueOut, bool &decodeFailed) {
    UInt32 clientHandle = 0, queueSize = 0;
    Double sampling = 0;
    Boolean discardOldest = true;
    NodeId filterType;
    Byte filterEncoding = 0;
    std::string_view filterBody;
    bool filterNull = true;
    decodeFailed = !(r.ReadUInt32(clientHandle) && r.ReadDouble(sampling) && DecodeNodeId(r, filterType) &&
                     r.ReadByte(filterEncoding));
    if(!decodeFailed && (filterEncoding & 0x03)) decodeFailed = !r.ReadByteString(filterBody, filterNull);
    if(!decodeFailed) decodeFailed = !(r.ReadUInt32(queueSize) && r.ReadBoolean(discardOldest));
    if(decodeFailed) return StatusCode::BadDecodingError;

    Trigger trigger = Trigger::StatusValue;
    Deadband deadband = Deadband::None;
    Double deadbandValue = 0;
    if(!filterType.IsNull()) {
        if(filterType == NodeId(0, ns0::EventFilter)) return StatusCode::BadFilterNotAllowed;
        if(filterType != NodeId(0, ns0::DataChangeFilter) || filterEncoding != 0x01)
            return StatusCode::BadMonitoredItemFilterUnsupported;
        ByteReader f(std::span<const Byte>(reinterpret_cast<const Byte *>(filterBody.data()), filterBody.size()));
        UInt32 t = 0, d = 0;
        if(!f.ReadUInt32(t) || !f.ReadUInt32(d) || !f.ReadDouble(deadbandValue)) return StatusCode::BadMonitoredItemFilterInvalid;
        if(t > 2 || d > 2) return StatusCode::BadMonitoredItemFilterInvalid;
        trigger = static_cast<Trigger>(t);
        deadband = static_cast<Deadband>(d);
        // a DataChangeFilter applies to the Value attribute only (Part 4 7.22.2)
        if(item.attributeId != static_cast<UInt32>(ns0::AttributeId::Value)) return StatusCode::BadFilterNotAllowed;
        // Percent needs the EURange property: no variable here has one
        if(deadband == Deadband::Percent) return StatusCode::BadMonitoredItemFilterUnsupported;
        if(deadband == Deadband::Absolute && (std::isnan(deadbandValue) || deadbandValue < 0))
            return StatusCode::BadDeadbandFilterInvalid;
    }
    item.clientHandle = clientHandle;
    item.samplingInterval = ReviseSampling(s, sampling);
    item.queueSize = std::clamp<UInt32>(queueSize, 1, MAX_QUEUE_SIZE);
    item.discardOldest = discardOldest;
    item.trigger = trigger;
    item.deadband = deadband;
    item.deadbandValue = deadbandValue;
    // a smaller queue keeps the newest values (with the overflow bit, Part 4 5.12.1.5)
    if(item.queued > item.queueSize) {
        const UInt32 drop = item.queued - item.queueSize;
        std::move(item.queue.begin() + drop, item.queue.begin() + item.queued, item.queue.begin());
        item.queued = item.queueSize;
        if(item.queueSize > 1)
            item.queue[0].status = static_cast<StatusCode>(static_cast<UInt32>(item.queue[0].status) | OVERFLOW_BITS);
    }
    if(item.queueSize == 1) // the overflow bit is only set for queues larger than one
        for(UInt32 i = 0; i < item.queued; i++)
            item.queue[i].status = static_cast<StatusCode>(static_cast<UInt32>(item.queue[i].status) & ~OVERFLOW_BITS);
    samplingOut = item.samplingInterval;
    queueOut = item.queueSize;
    return StatusCode::Good;
}

bool Subscriptions::HandleCreateMonitoredItems(const ServiceContext &ctx, const RequestHeader &h, ByteReader &r,
                                               ByteWriter &w) {
    UInt32 subscriptionId = 0;
    Int32 timestamps = 0, count = 0;
    if(!r.ReadUInt32(subscriptionId) || !r.ReadInt32(timestamps) || !r.ReadInt32(count)) return false;
    Subscription *s = FindSubscription(subscriptionId);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    if(timestamps < 0 || timestamps > 3) return EncodeServiceFault(w, h, StatusCode::BadTimestampsToReturnInvalid);
    if(const StatusCode st = CheckCount(count, MAX_ITEMS_PER_CALL); !IsGood(st)) return EncodeServiceFault(w, h, st);
    if(!WriteStatusArrayHeader(w, h, ns0::CreateMonitoredItemsResponse, count)) return false;
    const DateTime now = NowMonotonic();
    for(Int32 i = 0; i < count; i++) {
        NodeId nodeId;
        UInt32 attributeId = 0;
        String indexRange;
        QualifiedName dataEncoding;
        Int32 mode = 0;
        if(!DecodeNodeId(r, nodeId) || !r.ReadUInt32(attributeId) || !r.ReadString(indexRange) ||
           !DecodeQualifiedName(r, dataEncoding) || !r.ReadInt32(mode))
            return false;

        MonitoredItem candidate;
        candidate.node = ctx.addressSpace.FindNode(nodeId);
        candidate.attributeId = attributeId;
        candidate.timestamps = static_cast<TimestampsToReturn>(timestamps);
        Double sampling = 0;
        UInt32 queue = 0;
        bool decodeFailed = false;
        StatusCode status = ApplyParameters(candidate, *s, r, sampling, queue, decodeFailed);
        if(decodeFailed) return false;
        if(IsGood(status)) {
            if(candidate.node == nullptr) status = StatusCode::BadNodeIdUnknown;
            else if(mode < 0 || mode > 2) status = StatusCode::BadMonitoringModeInvalid;
            else if(ReadAttribute(ctx, nodeId, attributeId).status == StatusCode::BadAttributeIdInvalid)
                status = StatusCode::BadAttributeIdInvalid;
            else if(!indexRange.isNull && !indexRange.value.empty())
                status = NumericRangeValid(indexRange.value) ? StatusCode::BadIndexRangeNoData : StatusCode::BadIndexRangeInvalid;
            else if(!dataEncoding.name.empty())
                status = StatusCode::BadDataEncodingInvalid;
            else if(candidate.deadband == Deadband::Absolute && !IsNumericDataType(ctx.addressSpace, candidate.node->dataType))
                status = StatusCode::BadFilterNotAllowed;
        }
        MonitoredItem *slot = nullptr;
        if(IsGood(status)) {
            for(auto &it : items_)
                if(!it.used) { slot = &it; break; }
            if(slot == nullptr) status = StatusCode::BadTooManyMonitoredItems;
        }
        UInt32 id = 0;
        if(IsGood(status)) {
            *slot = candidate;
            slot->used = true;
            slot->id = id = ++nextItemId_;
            slot->subscriptionId = s->id;
            slot->mode = static_cast<MonitoringMode>(mode);
            slot->nextSample = now; // the first sample at once: the initial value
        }
        if(!w.WriteStatusCode(status) || !w.WriteUInt32(id) || !w.WriteDouble(IsGood(status) ? sampling : 0) ||
           !w.WriteUInt32(IsGood(status) ? queue : 0) || !EncodeNullExtensionObject(w))
            return false;
    }
    return w.WriteInt32(-1);
}

bool Subscriptions::HandleModifyMonitoredItems(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    UInt32 subscriptionId = 0;
    Int32 timestamps = 0, count = 0;
    if(!r.ReadUInt32(subscriptionId) || !r.ReadInt32(timestamps) || !r.ReadInt32(count)) return false;
    Subscription *s = FindSubscription(subscriptionId);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    if(timestamps < 0 || timestamps > 3) return EncodeServiceFault(w, h, StatusCode::BadTimestampsToReturnInvalid);
    if(const StatusCode st = CheckCount(count, MAX_ITEMS_PER_CALL); !IsGood(st)) return EncodeServiceFault(w, h, st);
    if(!WriteStatusArrayHeader(w, h, ns0::ModifyMonitoredItemsResponse, count)) return false;
    for(Int32 i = 0; i < count; i++) {
        UInt32 itemId = 0;
        if(!r.ReadUInt32(itemId)) return false;
        MonitoredItem *item = FindItem(s->id, itemId);
        MonitoredItem scratch;  // parameters of an unknown item are decoded, then dropped
        MonitoredItem &target = item != nullptr ? *item : scratch;
        MonitoredItem before = target;
        Double sampling = 0;
        UInt32 queue = 0;
        bool decodeFailed = false;
        StatusCode status = ApplyParameters(target, *s, r, sampling, queue, decodeFailed);
        if(decodeFailed) return false;
        if(item == nullptr) status = StatusCode::BadMonitoredItemIdInvalid;
        else if(!IsGood(status)) *item = before;  // a rejected modification changes nothing
        else item->timestamps = static_cast<TimestampsToReturn>(timestamps);
        if(!w.WriteStatusCode(status) || !w.WriteDouble(IsGood(status) ? sampling : 0) ||
           !w.WriteUInt32(IsGood(status) ? queue : 0) || !EncodeNullExtensionObject(w))
            return false;
    }
    return w.WriteInt32(-1);
}

bool Subscriptions::HandleSetMonitoringMode(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    UInt32 subscriptionId = 0;
    Int32 mode = 0, count = 0;
    if(!r.ReadUInt32(subscriptionId) || !r.ReadInt32(mode) || !r.ReadInt32(count)) return false;
    Subscription *s = FindSubscription(subscriptionId);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    if(mode < 0 || mode > 2) return EncodeServiceFault(w, h, StatusCode::BadMonitoringModeInvalid);
    if(const StatusCode st = CheckCount(count, MAX_ITEMS_PER_CALL); !IsGood(st)) return EncodeServiceFault(w, h, st);
    if(!WriteStatusArrayHeader(w, h, ns0::SetMonitoringModeResponse, count)) return false;
    for(Int32 i = 0; i < count; i++) {
        UInt32 itemId = 0;
        if(!r.ReadUInt32(itemId)) return false;
        MonitoredItem *item = FindItem(s->id, itemId);
        if(item != nullptr) {
            item->mode = static_cast<MonitoringMode>(mode);
            if(item->mode == MonitoringMode::Disabled) {
                item->queued = 0;      // Part 4 5.12.4: disabling discards the queue
                item->sampled = false; // re-enabling reports the current value again
            }
        }
        if(!w.WriteStatusCode(item != nullptr ? StatusCode::Good : StatusCode::BadMonitoredItemIdInvalid)) return false;
    }
    return w.WriteInt32(-1);
}

bool Subscriptions::HandleDeleteMonitoredItems(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    UInt32 subscriptionId = 0;
    Int32 count = 0;
    if(!r.ReadUInt32(subscriptionId) || !r.ReadInt32(count)) return false;
    Subscription *s = FindSubscription(subscriptionId);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    if(const StatusCode st = CheckCount(count, MAX_ITEMS_PER_CALL); !IsGood(st)) return EncodeServiceFault(w, h, st);
    if(!WriteStatusArrayHeader(w, h, ns0::DeleteMonitoredItemsResponse, count)) return false;
    for(Int32 i = 0; i < count; i++) {
        UInt32 itemId = 0;
        if(!r.ReadUInt32(itemId)) return false;
        MonitoredItem *item = FindItem(s->id, itemId);
        if(item != nullptr) DeleteItem(*item);
        if(!w.WriteStatusCode(item != nullptr ? StatusCode::Good : StatusCode::BadMonitoredItemIdInvalid)) return false;
    }
    return w.WriteInt32(-1);
}

// Part 4 5.12.5: when the triggering item reports, its linked items (also in Sampling mode)
// report their queued values too.
bool Subscriptions::HandleSetTriggering(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    UInt32 subscriptionId = 0, triggeringId = 0;
    Int32 adds = 0;
    if(!r.ReadUInt32(subscriptionId) || !r.ReadUInt32(triggeringId) || !r.ReadInt32(adds)) return false;
    std::array<UInt32, MAX_MONITORED_ITEMS> toAdd{}, toRemove{};
    if(adds > MAX_ITEMS_PER_CALL) return EncodeServiceFault(w, h, StatusCode::BadTooManyOperations);
    for(Int32 i = 0; i < adds; i++)
        if(!r.ReadUInt32(toAdd[static_cast<size_t>(i)])) return false;
    Int32 removes = 0;
    if(!r.ReadInt32(removes)) return false;
    if(removes > MAX_ITEMS_PER_CALL) return EncodeServiceFault(w, h, StatusCode::BadTooManyOperations);
    for(Int32 i = 0; i < removes; i++)
        if(!r.ReadUInt32(toRemove[static_cast<size_t>(i)])) return false;
    if(adds <= 0 && removes <= 0) return EncodeServiceFault(w, h, StatusCode::BadNothingToDo);
    Subscription *s = FindSubscription(subscriptionId);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    MonitoredItem *trigger = FindItem(s->id, triggeringId);
    if(trigger == nullptr) return EncodeServiceFault(w, h, StatusCode::BadMonitoredItemIdInvalid);

    if(!EncodeNodeId(w, NodeId(0, ns0::SetTriggeringResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(h, StatusCode::Good))) return false;
    if(!w.WriteInt32(std::max<Int32>(adds, 0))) return false;
    for(Int32 i = 0; i < adds; i++) {
        const UInt32 id = toAdd[static_cast<size_t>(i)];
        StatusCode status = StatusCode::Good;
        if(FindItem(s->id, id) == nullptr) status = StatusCode::BadMonitoredItemIdInvalid;
        else if(std::find(trigger->links.begin(), trigger->links.begin() + trigger->linkCount, id) ==
                trigger->links.begin() + trigger->linkCount) {
            if(trigger->linkCount == MAX_TRIGGER_LINKS) status = StatusCode::BadTooManyMonitoredItems;
            else trigger->links[trigger->linkCount++] = id;
        }
        if(!w.WriteStatusCode(status)) return false;
    }
    if(!w.WriteInt32(-1)) return false;
    if(!w.WriteInt32(std::max<Int32>(removes, 0))) return false;
    for(Int32 i = 0; i < removes; i++) {
        const UInt32 id = toRemove[static_cast<size_t>(i)];
        auto end = trigger->links.begin() + trigger->linkCount;
        auto it = std::find(trigger->links.begin(), end, id);
        if(it != end) {
            std::move(it + 1, end, it);
            --trigger->linkCount;
        }
        if(!w.WriteStatusCode(it != end ? StatusCode::Good : StatusCode::BadMonitoredItemIdInvalid)) return false;
    }
    return w.WriteInt32(-1);
}

bool Subscriptions::HandleRepublish(const RequestHeader &h, ByteReader &r, ByteWriter &w) {
    UInt32 subscriptionId = 0, sequence = 0;
    if(!r.ReadUInt32(subscriptionId) || !r.ReadUInt32(sequence)) return false;
    Subscription *s = FindSubscription(subscriptionId);
    if(s == nullptr) return EncodeServiceFault(w, h, StatusCode::BadSubscriptionIdInvalid);
    if(s->retransmitSequence == 0 || s->retransmitSequence != sequence)
        return EncodeServiceFault(w, h, StatusCode::BadMessageNotAvailable);
    return EncodeNodeId(w, NodeId(0, ns0::RepublishResponse)) &&
           EncodeResponseHeader(w, MakeResponseHeader(h, StatusCode::Good)) &&
           w.WriteRaw(s->retransmit.data(), s->retransmitSize);
}

// ---- Publish -------------------------------------------------------------------------------------------

StatusCode Subscriptions::QueuePublish(const RequestHeader &h, UInt32 secureChannelRequestId, ByteReader &r) {
    Int32 acks = 0;
    if(!r.ReadInt32(acks)) return StatusCode::BadDecodingError;
    PublishRequest p;
    p.requestId = secureChannelRequestId;
    p.header = h;
    p.received = NowMonotonic();
    for(Int32 i = 0; i < acks; i++) {
        UInt32 subscriptionId = 0, sequence = 0;
        if(!r.ReadUInt32(subscriptionId) || !r.ReadUInt32(sequence)) return StatusCode::BadDecodingError;
        Subscription *s = FindSubscription(subscriptionId);
        StatusCode result = StatusCode::BadSubscriptionIdInvalid;
        if(s != nullptr) {
            result = (s->retransmitSequence != 0 && s->retransmitSequence == sequence) ? StatusCode::Good
                                                                                      : StatusCode::BadSequenceNumberUnknown;
            if(IsGood(result)) s->retransmitSequence = 0; // acknowledged: no longer retained
        }
        if(p.acks < MAX_ACKNOWLEDGEMENTS) p.ackResults[p.acks++] = result;
    }
    if(SubscriptionCount() == 0) return StatusCode::BadNoSubscription;
    if(publishCount_ == MAX_PUBLISH_REQUESTS) return StatusCode::BadTooManyPublishRequests;
    publish_[publishCount_++] = p;
    for(auto &s : subscriptions_)
        if(s.used) s.lifetimeCounter = 0; // a Publish request keeps every subscription alive
    return StatusCode::Good;
}

// ---- sampling ------------------------------------------------------------------------------------

void Subscriptions::Enqueue(MonitoredItem &item, const Sample &sample) {
    if(item.queued < item.queueSize) {
        item.queue[item.queued++] = sample;
        return;
    }
    // full (Part 4 5.12.1.5): discard the oldest (overflow bit on the new oldest) or replace
    // the newest (overflow bit on it); queues of one carry no overflow bit
    if(item.discardOldest) {
        std::move(item.queue.begin() + 1, item.queue.begin() + item.queued, item.queue.begin());
        item.queue[item.queued - 1] = sample;
        if(item.queueSize > 1)
            item.queue[0].status = static_cast<StatusCode>(static_cast<UInt32>(item.queue[0].status) | OVERFLOW_BITS);
    } else {
        item.queue[item.queued - 1] = sample;
        if(item.queueSize > 1)
            item.queue[item.queued - 1].status =
                static_cast<StatusCode>(static_cast<UInt32>(sample.status) | OVERFLOW_BITS);
    }
}

void Subscriptions::SampleItem(const ServiceContext &ctx, MonitoredItem &item, DateTime now) {
    const AttributeValue v = ReadAttribute(ctx, item.node->nodeId, item.attributeId);
    bool changed = !item.sampled || v.status != item.lastStatus;
    if(!changed && item.trigger != Trigger::Status) {
        if(item.deadband == Deadband::Absolute && IsNumeric(v.value) && IsNumeric(item.lastValue))
            changed = std::fabs(AsDouble(v.value) - AsDouble(item.lastValue)) > item.deadbandValue;
        else
            changed = !(v.value == item.lastValue);
        // StatusValueTimestamp: every sample has a new source timestamp here (read time)
        if(item.trigger == Trigger::StatusValueTimestamp) changed = true;
    }
    if(!changed) return;
    item.sampled = true;
    item.lastValue = v.value;
    item.lastStatus = v.status;
    const DateTime wall = Now();
    Enqueue(item, Sample{v.value, v.status, wall, wall});
    (void)now;
}

bool Subscriptions::HasNotifications(const Subscription &s) const {
    for(const auto &i : items_) {
        if(!i.used || i.subscriptionId != s.id || i.queued == 0) continue;
        if(i.mode == MonitoringMode::Reporting || (i.mode == MonitoringMode::Sampling && i.triggered)) return true;
    }
    return false;
}

void Subscriptions::Tick(const ServiceContext &ctx) {
    const DateTime now = NowMonotonic();
    for(auto &item : items_) {
        if(!item.used || item.mode == MonitoringMode::Disabled || now < item.nextSample) continue;
        const UInt32 before = item.queued;
        SampleItem(ctx, item, now);
        // a reporting triggering item with a new value triggers its links
        if(item.queued != before && item.mode == MonitoringMode::Reporting)
            for(UInt32 k = 0; k < item.linkCount; k++)
                if(MonitoredItem *linked = FindItem(item.subscriptionId, item.links[k])) linked->triggered = true;
        const DateTime period = std::max<DateTime>(Ticks(item.samplingInterval), 1);
        item.nextSample = std::max(item.nextSample + period, now - period + 1);  // no catch-up bursts
        if(item.nextSample <= now) item.nextSample = now + period;
    }
    // publishing cycles (Part 4 5.13.1.2 state table, simplified to one cycle per interval)
    for(auto &s : subscriptions_) {
        if(!s.used || now < s.nextCycle) continue;
        s.nextCycle = std::max(s.nextCycle + Ticks(s.interval), now + 1);
        const bool data = s.enabled && HasNotifications(s);
        if(data || s.firstCycle) { // the first cycle always answers: data or a keep-alive
            s.dataDue = data;
            s.keepAliveDue = !data;
        } else if(++s.keepAliveCounter >= s.keepAliveCount) {
            s.keepAliveDue = true;
        }
        s.firstCycle = false;
        if((s.dataDue || s.keepAliveDue) && publishCount_ == 0) {
            // late: no Publish request to carry the response
            if(++s.lifetimeCounter >= s.lifetimeCount) {
                // expired: the items go now, the subscription ends with a StatusChangeNotification
                // answering the next Publish request (or with the session)
                s.statusChange = StatusCode::BadTimeout;
                s.dataDue = s.keepAliveDue = false;
                for(auto &i : items_)
                    if(i.used && i.subscriptionId == s.id) i = MonitoredItem{};
            }
        }
    }
    // Publish requests whose TimeoutHint passed are answered with BadTimeout (WritePublish)
}

bool Subscriptions::PublishDue() const {
    if(publishCount_ == 0) return false;
    const DateTime now = NowMonotonic();
    const PublishRequest &oldest = publish_[0];
    if(oldest.header.timeoutHint != 0 && now - oldest.received > static_cast<DateTime>(oldest.header.timeoutHint) * MS)
        return true;
    for(const auto &s : subscriptions_)
        if(s.used && (s.dataDue || s.keepAliveDue || s.statusChange != StatusCode::Good)) return true;
    return false;
}

DateTime Subscriptions::NextDeadline() const {
    DateTime best = 0;
    const auto consider = [&](DateTime t) {
        if(t != 0 && (best == 0 || t < best)) best = t;
    };
    for(const auto &i : items_)
        if(i.used && i.mode != MonitoringMode::Disabled) consider(i.nextSample);
    for(const auto &s : subscriptions_)
        if(s.used) consider(s.nextCycle);
    if(publishCount_ > 0 && publish_[0].header.timeoutHint != 0)
        consider(publish_[0].received + static_cast<DateTime>(publish_[0].header.timeoutHint) * MS + 1);
    return best;
}

// NotificationMessage (Part 4 7.24): sequence number, publish time, notification data.
bool Subscriptions::WriteNotificationMessage(ByteWriter &w, Subscription &s, bool keepAlive, DateTime now, bool &more) {
    (void)now;
    more = false;
    // a keep-alive carries the next sequence number without using it (Part 4 5.13.1.1)
    const UInt32 sequence = s.nextSequence;
    if(!w.WriteUInt32(sequence) || !w.WriteDateTime(Now())) return false;
    if(keepAlive) return w.WriteInt32(0);

    if(s.statusChange != StatusCode::Good) {
        // StatusChangeNotification: the subscription ends with this message
        if(!w.WriteInt32(1) || !EncodeNodeId(w, NodeId(0, ns0::StatusChangeNotification)) || !w.WriteByte(0x01)) return false;
        Byte body[16];
        ByteWriter b(body);
        if(!b.WriteStatusCode(s.statusChange) || !b.WriteByte(0)) return false; // + DiagnosticInfo: empty
        return w.WriteByteString(std::string_view(reinterpret_cast<const char *>(body), b.Position()), false);
    }

    // DataChangeNotification: MonitoredItemNotification[] {clientHandle, DataValue}, DiagnosticInfos
    if(!w.WriteInt32(1) || !EncodeNodeId(w, NodeId(0, ns0::DataChangeNotification)) || !w.WriteByte(0x01)) return false;
    const size_t lengthAt = w.Position();
    if(!w.WriteInt32(0)) return false; // ByteString length, patched below
    const size_t countAt = w.Position();
    if(!w.WriteInt32(0)) return false; // MonitoredItemNotification count, patched below
    const UInt32 limit = s.maxNotifications == 0 ? 0xFFFF'FFFFu : s.maxNotifications;
    UInt32 written = 0;
    for(auto &item : items_) {
        if(!item.used || item.subscriptionId != s.id || item.queued == 0) continue;
        const bool reports = item.mode == MonitoringMode::Reporting || (item.mode == MonitoringMode::Sampling && item.triggered);
        if(!reports) continue;
        UInt32 taken = 0;
        while(taken < item.queued) {
            // keep room for the trailing DiagnosticInfos, the signature and padding
            if(written == limit || w.Remaining() < 160) {
                more = true;
                break;
            }
            const Sample &v = item.queue[taken];
            if(!w.WriteUInt32(item.clientHandle)) return false;
            if(!EncodeDataValue(w, AttributeValue{v.status, v.value}, item.timestamps,
                                item.attributeId == static_cast<UInt32>(ns0::AttributeId::Value), v.source))
                return false;
            ++taken;
            ++written;
        }
        std::move(item.queue.begin() + taken, item.queue.begin() + item.queued, item.queue.begin());
        item.queued -= taken;
        if(item.queued == 0) item.triggered = false;
        if(more) break;
    }
    if(!w.WriteInt32(-1)) return false; // DiagnosticInfos
    if(!w.PatchUInt32(countAt, written)) return false;
    return w.PatchUInt32(lengthAt, static_cast<UInt32>(w.Position() - countAt));
}

bool Subscriptions::WritePublish(ByteWriter &w, UInt32 &secureChannelRequestId) {
    if(publishCount_ == 0) return false;
    PublishRequest p = publish_[0];
    std::move(publish_.begin() + 1, publish_.begin() + publishCount_, publish_.begin());
    --publishCount_;
    secureChannelRequestId = p.requestId;
    const DateTime now = NowMonotonic();
    if(p.header.timeoutHint != 0 && now - p.received > static_cast<DateTime>(p.header.timeoutHint) * MS)
        return EncodeServiceFault(w, p.header, StatusCode::BadTimeout);

    // the subscription to serve: a status change first, then data, then keep-alives
    Subscription *s = nullptr;
    for(auto &c : subscriptions_)
        if(c.used && c.statusChange != StatusCode::Good) { s = &c; break; }
    if(s == nullptr)
        for(auto &c : subscriptions_)
            if(c.used && c.dataDue) { s = &c; break; }
    if(s == nullptr)
        for(auto &c : subscriptions_)
            if(c.used && c.keepAliveDue) { s = &c; break; }
    if(s == nullptr) { // nothing due after all: back to the queue
        std::move_backward(publish_.begin(), publish_.begin() + publishCount_, publish_.begin() + publishCount_ + 1);
        publish_[0] = p;
        ++publishCount_;
        return false;
    }
    const bool keepAlive = !s->dataDue && s->statusChange == StatusCode::Good;

    if(!EncodeNodeId(w, NodeId(0, ns0::PublishResponse))) return false;
    if(!EncodeResponseHeader(w, MakeResponseHeader(p.header, StatusCode::Good))) return false;
    if(!w.WriteUInt32(s->id)) return false;
    // AvailableSequenceNumbers: the retained message (if any) plus the one sent now
    const bool retain = !keepAlive;
    const UInt32 available = (s->retransmitSequence != 0 ? 1 : 0) + (retain ? 1 : 0);
    if(!w.WriteInt32(static_cast<Int32>(available))) return false;
    if(s->retransmitSequence != 0 && !w.WriteUInt32(s->retransmitSequence)) return false;
    if(retain && !w.WriteUInt32(s->nextSequence)) return false;
    const size_t moreAt = w.Position();
    if(!w.WriteBoolean(false)) return false;
    const size_t messageAt = w.Position();
    bool more = false;
    if(!WriteNotificationMessage(w, *s, keepAlive, now, more)) return false;
    const size_t messageSize = w.Position() - messageAt;
    if(more && !w.PatchByte(moreAt, 1)) return false;
    if(!w.WriteInt32(static_cast<Int32>(p.acks))) return false; // Results: the acknowledgements
    for(UInt32 i = 0; i < p.acks; i++)
        if(!w.WriteStatusCode(p.ackResults[i])) return false;
    if(!w.WriteInt32(-1)) return false; // DiagnosticInfos

    if(retain) {
        // the retransmission queue keeps this message for Republish until it is acknowledged
        const auto written = w.Written();
        if(messageSize <= s->retransmit.size()) {
            std::memcpy(s->retransmit.data(), written.data() + messageAt, messageSize);
            s->retransmitSize = messageSize;
            s->retransmitSequence = s->nextSequence;
        }
        s->nextSequence = s->nextSequence == 0xFFFF'FFFFu ? 1 : s->nextSequence + 1;
    }
    s->keepAliveCounter = 0;
    s->lifetimeCounter = 0;
    s->keepAliveDue = false;
    s->dataDue = more;
    if(s->statusChange != StatusCode::Good) DeleteSubscription(*s);
    return w.Ok();
}

}  // namespace rtos::opcua
