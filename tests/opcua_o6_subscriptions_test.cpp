// SPDX-License-Identifier: MIT
//
// Subscription service set (Subscriptions, MonitoredItems, Publish), ported from open62541
// tests/server/check_services_subscriptions.c, check_services_subscriptions_modify.c,
// check_monitoreditem_filter.c and check_datachange_deadband_typekinds.c. Time is a test clock
// (the OPC UA monotonic clock source); Tick() and WritePublish() are what the connection calls.
// Tests that do not fit (callbacks, transfer, diagnostics) are listed in tests/open62541/rules.tsv.
#include <cmath>
#include <optional>

#include "opcua_service_harness.hpp"
#include "rtos/opcua/clock.hpp"
#include "rtos/opcua/subscriptions.hpp"

namespace {

using namespace harness;
using Subs = ua::Subscriptions;

std::uint64_t g_ms = 1'000'000;  // the monotonic clock of these tests

struct Filter {
    ua::UInt32 trigger = 1;  // StatusValue
    ua::UInt32 deadband = 0;
    ua::Double value = 0;
};
struct Item {
    ua::NodeId node{};
    ua::UInt32 attribute = 13;
    ua::Int32 mode = 2;  // Reporting
    ua::UInt32 handle = 1;
    ua::Double sampling = 0;
    ua::UInt32 queue = 1;
    bool discardOldest = true;
    std::optional<Filter> filter{};
    ua::String indexRange{};
    ua::NodeId filterType{};  // other filter types (EventFilter, ...): an empty body
};
struct ItemResult {
    ua::StatusCode status = ua::StatusCode::BadInternalError;
    ua::UInt32 id = 0;
    ua::Double sampling = 0;
    ua::UInt32 queue = 0;
};
struct Created {
    ua::StatusCode status = ua::StatusCode::BadInternalError;
    ua::UInt32 id = 0;
    ua::Double interval = 0;
    ua::UInt32 lifetime = 0, keepAlive = 0;
};
struct Notification {
    ua::UInt32 handle = 0;
    DataValue value;
};
struct Published {
    bool fault = false;
    ua::StatusCode status = ua::StatusCode::Good;
    ua::UInt32 subscription = 0;
    std::vector<ua::UInt32> available;
    bool more = false;
    ua::UInt32 sequence = 0;
    bool keepAlive = true;  // no notification data
    std::vector<Notification> data;
    std::optional<ua::StatusCode> statusChange;
    std::vector<ua::StatusCode> acks;
    ua::UInt32 requestId = 0;
};

class SubscriptionTest : public ServiceTest {
public:
    Subs subs;

    void SetUp() override {
        ServiceTest::SetUp();
        g_ms = 1'000'000;
        ua::SetClock(nullptr, []() noexcept { return g_ms; });
    }
    void TearDown() override { ua::SetClock(nullptr, nullptr); }

    Created create(ua::Double interval = 100, ua::UInt32 lifetime = 30, ua::UInt32 keepAlive = 10,
                   ua::UInt32 maxNotifications = 0, bool enabled = true) {
        Response resp = run(
            [&](ua::ByteWriter& w) {
                w.WriteDouble(interval);
                w.WriteUInt32(lifetime);
                w.WriteUInt32(keepAlive);
                w.WriteUInt32(maxNotifications);
                w.WriteBoolean(enabled);
                w.WriteByte(0);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return subs.HandleCreateSubscription(h, r, w);
            });
        Created c;
        c.status = resp.result;
        if (!resp.fault()) {
            ua::ByteReader r = resp.reader();
            EXPECT_TRUE(r.ReadUInt32(c.id) && r.ReadDouble(c.interval) && r.ReadUInt32(c.lifetime) &&
                        r.ReadUInt32(c.keepAlive));
        }
        return c;
    }

    Response modify_subscription(ua::UInt32 id, ua::Double interval, ua::UInt32 lifetime, ua::UInt32 keepAlive,
                                 ua::UInt32 maxNotifications = 0) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteUInt32(id);
                w.WriteDouble(interval);
                w.WriteUInt32(lifetime);
                w.WriteUInt32(keepAlive);
                w.WriteUInt32(maxNotifications);
                w.WriteByte(10);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return subs.HandleModifySubscription(h, r, w);
            });
    }

    static void encode_parameters(ua::ByteWriter& w, const Item& i) {
        w.WriteUInt32(i.handle);
        w.WriteDouble(i.sampling);
        if (i.filter) {
            ua::EncodeNodeId(w, ua::NodeId(0, ua::ns0::DataChangeFilter));
            w.WriteByte(0x01);
            std::array<Byte, 16> body{};
            ua::ByteWriter b(body);
            b.WriteUInt32(i.filter->trigger);
            b.WriteUInt32(i.filter->deadband);
            b.WriteDouble(i.filter->value);
            w.WriteByteString(std::string_view(reinterpret_cast<const char*>(body.data()), b.Position()), false);
        } else if (!i.filterType.IsNull()) {
            ua::EncodeNodeId(w, i.filterType);
            w.WriteByte(0x01);
            w.WriteByteString(std::string_view{}, false);
        } else {
            ua::EncodeNullExtensionObject(w);
        }
        w.WriteUInt32(i.queue);
        w.WriteBoolean(i.discardOldest);
    }

    static std::vector<ua::StatusCode> statuses(Response& resp) {
        std::vector<ua::StatusCode> out;
        if (resp.fault()) return out;
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0;
        EXPECT_TRUE(r.ReadInt32(count));
        for (ua::Int32 i = 0; i < count; i++) {
            ua::StatusCode s{};
            EXPECT_TRUE(r.ReadStatusCode(s));
            out.push_back(s);
        }
        return out;
    }

    Response create_items_request(ua::UInt32 sub, const std::vector<Item>& items, ua::Int32 timestamps = 2) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteUInt32(sub);
                w.WriteInt32(timestamps);
                w.WriteInt32(static_cast<ua::Int32>(items.size()));
                for (const Item& i : items) {
                    ua::EncodeNodeId(w, i.node);
                    w.WriteUInt32(i.attribute);
                    w.WriteString(i.indexRange);
                    ua::EncodeQualifiedName(w, ua::QualifiedName{});
                    w.WriteInt32(i.mode);
                    encode_parameters(w, i);
                }
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return subs.HandleCreateMonitoredItems(ctx, h, r, w);
            });
    }
    std::vector<ItemResult> create_items(ua::UInt32 sub, const std::vector<Item>& items, ua::Int32 timestamps = 2) {
        Response resp = create_items_request(sub, items, timestamps);
        std::vector<ItemResult> out;
        if (resp.fault()) return out;
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0;
        EXPECT_TRUE(r.ReadInt32(count));
        for (ua::Int32 i = 0; i < count; i++) {
            ItemResult x;
            EXPECT_TRUE(r.ReadStatusCode(x.status) && r.ReadUInt32(x.id) && r.ReadDouble(x.sampling) &&
                        r.ReadUInt32(x.queue) && ua::DecodeAndDiscardExtensionObject(r));
            out.push_back(x);
        }
        return out;
    }
    ItemResult create_item(ua::UInt32 sub, const Item& item, ua::Int32 timestamps = 2) {
        auto all = create_items(sub, {item}, timestamps);
        return all.empty() ? ItemResult{} : all.front();
    }

    Response modify_items_request(ua::UInt32 sub, const std::vector<std::pair<ua::UInt32, Item>>& items,
                                  ua::Int32 timestamps = 2) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteUInt32(sub);
                w.WriteInt32(timestamps);
                w.WriteInt32(static_cast<ua::Int32>(items.size()));
                for (const auto& [id, i] : items) {
                    w.WriteUInt32(id);
                    encode_parameters(w, i);
                }
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                return subs.HandleModifyMonitoredItems(h, r, w);
            });
    }
    ItemResult modify_item(ua::UInt32 sub, ua::UInt32 id, const Item& item, ua::Int32 timestamps = 2) {
        Response resp = modify_items_request(sub, {{id, item}}, timestamps);
        ItemResult x;
        if (resp.fault()) {
            x.status = resp.result;
            return x;
        }
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0;
        EXPECT_TRUE(r.ReadInt32(count) && count == 1 && r.ReadStatusCode(x.status) && r.ReadDouble(x.sampling) &&
                    r.ReadUInt32(x.queue) && ua::DecodeAndDiscardExtensionObject(r));
        return x;
    }

    Response ids_request(const std::function<void(ua::ByteWriter&)>& prefix, const std::vector<ua::UInt32>& ids,
                         const std::function<bool(const ua::RequestHeader&, ua::ByteReader&, ua::ByteWriter&)>& handler) {
        return run(
            [&](ua::ByteWriter& w) {
                prefix(w);
                w.WriteInt32(static_cast<ua::Int32>(ids.size()));
                for (ua::UInt32 id : ids) w.WriteUInt32(id);
            },
            handler);
    }
    Response set_mode(ua::UInt32 sub, ua::Int32 mode, const std::vector<ua::UInt32>& ids) {
        return ids_request([&](ua::ByteWriter& w) { w.WriteUInt32(sub); w.WriteInt32(mode); }, ids,
                           [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                               return subs.HandleSetMonitoringMode(h, r, w);
                           });
    }
    Response delete_items(ua::UInt32 sub, const std::vector<ua::UInt32>& ids) {
        return ids_request([&](ua::ByteWriter& w) { w.WriteUInt32(sub); }, ids,
                           [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                               return subs.HandleDeleteMonitoredItems(h, r, w);
                           });
    }
    Response delete_subscriptions(const std::vector<ua::UInt32>& ids) {
        return ids_request([](ua::ByteWriter&) {}, ids, [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
            return subs.HandleDeleteSubscriptions(h, r, w);
        });
    }
    Response set_publishing(bool enabled, const std::vector<ua::UInt32>& ids) {
        return ids_request([&](ua::ByteWriter& w) { w.WriteBoolean(enabled); }, ids,
                           [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) {
                               return subs.HandleSetPublishingMode(h, r, w);
                           });
    }
    Response set_triggering(ua::UInt32 sub, ua::UInt32 trigger, const std::vector<ua::UInt32>& adds,
                            const std::vector<ua::UInt32>& removes) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteUInt32(sub);
                w.WriteUInt32(trigger);
                w.WriteInt32(static_cast<ua::Int32>(adds.size()));
                for (ua::UInt32 id : adds) w.WriteUInt32(id);
                w.WriteInt32(static_cast<ua::Int32>(removes.size()));
                for (ua::UInt32 id : removes) w.WriteUInt32(id);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) { return subs.HandleSetTriggering(h, r, w); });
    }
    Response republish(ua::UInt32 sub, ua::UInt32 sequence) {
        return run(
            [&](ua::ByteWriter& w) {
                w.WriteUInt32(sub);
                w.WriteUInt32(sequence);
            },
            [&](const ua::RequestHeader& h, ua::ByteReader& r, ua::ByteWriter& w) { return subs.HandleRepublish(h, r, w); });
    }

    /// A Publish request (with acknowledgements); requestId identifies its answer.
    ua::StatusCode publish(const std::vector<std::pair<ua::UInt32, ua::UInt32>>& acks = {}, ua::UInt32 requestId = 500,
                           ua::UInt32 timeoutHint = 0) {
        ua::ByteWriter rw(request);
        rw.WriteInt32(static_cast<ua::Int32>(acks.size()));
        for (const auto& [sub, seq] : acks) {
            rw.WriteUInt32(sub);
            rw.WriteUInt32(seq);
        }
        ua::ByteReader rr(rw.Written());
        ua::RequestHeader h;
        h.requestHandle = requestId;
        h.timeoutHint = timeoutHint;
        return subs.QueuePublish(h, requestId, rr);
    }

    void advance(std::uint64_t ms) {
        // tick every 10 ms like a busy network thread
        for (std::uint64_t t = 0; t < ms; t += 10) {
            g_ms += std::min<std::uint64_t>(10, ms - t);
            subs.Tick(ctx);
        }
    }

    /// The next response the connection would send, if one is due.
    std::optional<Published> take() {
        subs.Tick(ctx);
        if (!subs.PublishDue()) return std::nullopt;
        ua::ByteWriter w(response);
        Published p;
        if (!subs.WritePublish(w, p.requestId)) return std::nullopt;
        Response resp = parse(w.Written());
        if (resp.fault()) {
            p.fault = true;
            p.status = resp.result;
            return p;
        }
        ua::ByteReader r = resp.reader();
        ua::Int32 count = 0, items = 0;
        ua::DateTime time = 0;
        EXPECT_TRUE(r.ReadUInt32(p.subscription) && r.ReadInt32(count));
        for (ua::Int32 i = 0; i < count; i++) {
            ua::UInt32 seq = 0;
            EXPECT_TRUE(r.ReadUInt32(seq));
            p.available.push_back(seq);
        }
        EXPECT_TRUE(r.ReadBoolean(p.more) && r.ReadUInt32(p.sequence) && r.ReadDateTime(time) && r.ReadInt32(items));
        for (ua::Int32 i = 0; i < items; i++) {
            ua::NodeId type;
            Byte encoding = 0;
            std::string_view body;
            bool null = true;
            EXPECT_TRUE(ua::DecodeNodeId(r, type) && r.ReadByte(encoding) && r.ReadByteString(body, null));
            ua::ByteReader b(std::span<const Byte>(reinterpret_cast<const Byte*>(body.data()), body.size()));
            if (type == ua::NodeId(0, ua::ns0::DataChangeNotification)) {
                p.keepAlive = false;
                ua::Int32 n = 0, diagnostics = 0;
                EXPECT_TRUE(b.ReadInt32(n));
                for (ua::Int32 k = 0; k < n; k++) {
                    Notification note;
                    EXPECT_TRUE(b.ReadUInt32(note.handle));
                    note.value = decode_data_value(b);
                    p.data.push_back(note);
                }
                EXPECT_TRUE(b.ReadInt32(diagnostics));
            } else if (type == ua::NodeId(0, ua::ns0::StatusChangeNotification)) {
                p.keepAlive = false;
                ua::StatusCode s{};
                EXPECT_TRUE(b.ReadStatusCode(s));
                p.statusChange = s;
            } else {
                ADD_FAILURE() << "unexpected notification type";
            }
        }
        ua::Int32 results = 0;
        EXPECT_TRUE(r.ReadInt32(results));
        for (ua::Int32 i = 0; i < results; i++) {
            ua::StatusCode s{};
            EXPECT_TRUE(r.ReadStatusCode(s));
            p.acks.push_back(s);
        }
        return p;
    }

    /// Publish and take the response (after `ms` of time).
    std::optional<Published> publish_and_take(std::uint64_t ms = 0) {
        EXPECT_EQ(publish(), ua::StatusCode::Good);
        advance(ms);
        return take();
    }

    static ua::Int32 int32(const Notification& n) {
        const ua::Int32* v = n.value.value.GetIf<ua::Int32>();
        return v ? *v : -999;
    }
};

// ---- subscriptions --------------------------------------------------------------------------------

// open62541: server/check_services_subscriptions.c Server_createSubscription
// open62541: server/check_services_subscriptions_modify.c createModifySubscription
TEST_F(SubscriptionTest, Server_createSubscription) {
    const Created c = create(250, 30, 10);
    EXPECT_EQ(c.status, ua::StatusCode::Good);
    EXPECT_GT(c.id, 0u);
    EXPECT_EQ(c.interval, 250);
    EXPECT_EQ(c.keepAlive, 10u);
    EXPECT_EQ(c.lifetime, 30u);
    EXPECT_EQ(subs.SubscriptionCount(), 1u);
}

// open62541: server/check_services_subscriptions.c Server_modifySubscription
TEST_F(SubscriptionTest, Server_modifySubscription) {
    const Created c = create();
    Response resp = modify_subscription(c.id, 500, 1000, 1000, 1);
    ASSERT_EQ(resp.result, ua::StatusCode::Good);
    ua::ByteReader r = resp.reader();
    ua::Double interval = 0;
    ua::UInt32 lifetime = 0, keepAlive = 0;
    ASSERT_TRUE(r.ReadDouble(interval) && r.ReadUInt32(lifetime) && r.ReadUInt32(keepAlive));
    EXPECT_EQ(interval, 500);
    EXPECT_EQ(keepAlive, 1000u);
    EXPECT_EQ(lifetime, 3000u) << "lifetime >= 3 x keep-alive";
}

// open62541: server/check_services_subscriptions.c Server_modifySubscription_invalid
TEST_F(SubscriptionTest, Server_modifySubscription_invalid) {
    EXPECT_EQ(modify_subscription(99999, 100, 30, 10).result, ua::StatusCode::BadSubscriptionIdInvalid);
}

// open62541: server/check_services_subscriptions.c Server_setPublishingMode
// open62541: server/check_services_subscriptions_modify.c setPublishingMode
TEST_F(SubscriptionTest, Server_setPublishingMode) {
    const Created c = create();
    ASSERT_GT(create_item(c.id, {.node = kAnswer}).id, 0u);
    Response resp = set_publishing(false, {c.id, 4711});
    EXPECT_EQ(statuses(resp), (std::vector<ua::StatusCode>{ua::StatusCode::Good, ua::StatusCode::BadSubscriptionIdInvalid}));
    // disabled: only keep-alives
    auto p = publish_and_take(200);
    ASSERT_TRUE(p);
    EXPECT_TRUE(p->keepAlive);
    resp = set_publishing(true, {c.id});
    p = publish_and_take(200);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u) << "the queued initial value";
    EXPECT_EQ(int32(p->data[0]), 42);
}

// open62541: server/check_services_subscriptions.c Server_deleteSubscription
// open62541: server/check_services_subscriptions.c Server_deleteSubscription_invalid
// open62541: server/check_services_subscriptions_modify.c deleteNonExistentSubscription
TEST_F(SubscriptionTest, Server_deleteSubscription) {
    const Created c = create();
    ASSERT_GT(create_item(c.id, {.node = kAnswer}).id, 0u);
    Response resp = delete_subscriptions({c.id, 99999});
    EXPECT_EQ(statuses(resp),
              (std::vector<ua::StatusCode>{ua::StatusCode::Good, ua::StatusCode::BadSubscriptionIdInvalid}));
    EXPECT_EQ(subs.SubscriptionCount(), 0u);
    EXPECT_EQ(subs.MonitoredItemCount(), 0u) << "its items went with it";
    EXPECT_EQ(publish(), ua::StatusCode::BadNoSubscription);
}

// open62541: server/check_services_subscriptions.c Server_invalidPublishingInterval
TEST_F(SubscriptionTest, Server_invalidPublishingInterval) {
    EXPECT_EQ(create(-5.0).interval, Subs::MIN_PUBLISHING_INTERVAL_MS);
    EXPECT_EQ(create(std::nan("")).interval, Subs::MIN_PUBLISHING_INTERVAL_MS);
}

// open62541: server/check_services_subscriptions.c Server_createSubscription_maxSubscriptionsPerSession
TEST_F(SubscriptionTest, Server_createSubscription_maxSubscriptionsPerSession) {
    for (std::size_t i = 0; i < Subs::MAX_SUBSCRIPTIONS; i++) EXPECT_EQ(create().status, ua::StatusCode::Good);
    EXPECT_EQ(create().status, ua::StatusCode::BadTooManySubscriptions);
}

// LifetimeCount >= 3 x MaxKeepAliveCount; without Publish requests the subscription ends after
// LifetimeCount intervals, with a StatusChangeNotification BadTimeout.
// open62541: server/check_services_subscriptions.c Server_lifeTimeCount
TEST_F(SubscriptionTest, Server_lifeTimeCount) {
    Created c = create(100, 3, 1);
    EXPECT_EQ(c.keepAlive, 1u);
    EXPECT_EQ(c.lifetime, 3u);
    const Created c2 = create(100, 4, 2);
    EXPECT_EQ(c2.keepAlive, 2u);
    EXPECT_EQ(c2.lifetime, 6u);
    ASSERT_GT(create_item(c.id, {.node = kAnswer}).id, 0u);
    advance(250);
    EXPECT_EQ(subs.MonitoredItemCount(), 1u) << "within the lifetime";
    advance(150);
    EXPECT_EQ(subs.MonitoredItemCount(), 0u) << "3 intervals without a Publish request: expired";
    const auto p = publish_and_take();
    ASSERT_TRUE(p);
    EXPECT_EQ(p->subscription, c.id);
    ASSERT_TRUE(p->statusChange);
    EXPECT_EQ(*p->statusChange, ua::StatusCode::BadTimeout);
    EXPECT_EQ(subs.SubscriptionCount(), 1u) << "the other subscription is kept alive by the request";
}

// A keep-alive after MaxKeepAliveCount intervals without notifications; it carries the next
// sequence number without using it.
// open62541: server/check_services_subscriptions.c Server_publishCallback
TEST_F(SubscriptionTest, Server_publishCallback) {
    const Created c = create(100, 30, 3);
    auto first = publish_and_take(100);
    ASSERT_TRUE(first) << "the first cycle answers at once";
    EXPECT_TRUE(first->keepAlive);
    EXPECT_EQ(publish(), ua::StatusCode::Good);
    advance(250);
    EXPECT_FALSE(take()) << "2 of 3 intervals";
    advance(100);
    const auto p = take();
    ASSERT_TRUE(p);
    EXPECT_EQ(p->subscription, c.id);
    EXPECT_TRUE(p->keepAlive);
    EXPECT_EQ(p->sequence, 1u);
    EXPECT_TRUE(p->available.empty());
}

// open62541: server/check_services_subscriptions.c Server_republish
// open62541: server/check_services_subscriptions.c Server_republish_invalid
// open62541: server/check_services_subscriptions.c Server_republish_unknownSequenceNumber
// open62541: server/check_services_subscriptions_modify.c republish_test
TEST_F(SubscriptionTest, Server_republish) {
    const Created c = create();
    EXPECT_EQ(republish(c.id, 0).result, ua::StatusCode::BadMessageNotAvailable);
    EXPECT_EQ(republish(99999, 0).result, ua::StatusCode::BadSubscriptionIdInvalid);
    ASSERT_GT(create_item(c.id, {.node = kAnswer}).id, 0u);
    const auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u);
    EXPECT_EQ(p->available, std::vector<ua::UInt32>{p->sequence});
    EXPECT_EQ(republish(c.id, p->sequence + 5).result, ua::StatusCode::BadMessageNotAvailable);
    Response again = republish(c.id, p->sequence);
    ASSERT_EQ(again.result, ua::StatusCode::Good);
    ua::ByteReader r = again.reader();
    ua::UInt32 seq = 0;
    ASSERT_TRUE(r.ReadUInt32(seq));
    EXPECT_EQ(seq, p->sequence);
    // acknowledged: no longer available
    EXPECT_EQ(publish({{c.id, p->sequence}, {c.id, 777}, {99999, 1}}), ua::StatusCode::Good);
    EXPECT_EQ(republish(c.id, p->sequence).result, ua::StatusCode::BadMessageNotAvailable);
    g_answer = 1;
    advance(100);
    const auto next = take();
    ASSERT_TRUE(next);
    EXPECT_EQ(next->acks, (std::vector<ua::StatusCode>{ua::StatusCode::Good, ua::StatusCode::BadSequenceNumberUnknown,
                                                        ua::StatusCode::BadSubscriptionIdInvalid}));
    EXPECT_EQ(next->sequence, p->sequence + 1);
}

// ---- monitored items ------------------------------------------------------------------------------

// open62541: server/check_services_subscriptions.c Server_createMonitoredItems
TEST_F(SubscriptionTest, Server_createMonitoredItems) {
    const Created c = create();
    const auto r = create_items(c.id, {{.node = kAnswer, .handle = 7}, {.node = ua::NodeId(1, 99999)},
                                       {.node = kAnswer, .attribute = 99}, {.node = kAnswer, .mode = 7},
                                       {.node = kAnswer, .indexRange = ua::String("1")}});
    ASSERT_EQ(r.size(), 5u);
    EXPECT_EQ(r[0].status, ua::StatusCode::Good);
    EXPECT_GT(r[0].id, 0u);
    EXPECT_EQ(r[0].sampling, ua::MIN_SAMPLING_INTERVAL_MS) << "0: the fastest rate";
    EXPECT_EQ(r[1].status, ua::StatusCode::BadNodeIdUnknown);
    EXPECT_EQ(r[2].status, ua::StatusCode::BadAttributeIdInvalid);
    EXPECT_EQ(r[3].status, ua::StatusCode::BadMonitoringModeInvalid);
    EXPECT_EQ(r[4].status, ua::StatusCode::BadIndexRangeNoData);
    const auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u);
    EXPECT_EQ(p->data[0].handle, 7u);
    EXPECT_EQ(int32(p->data[0]), 42) << "the initial value";
}

// open62541: server/check_services_subscriptions.c Server_createMonitoredItems_invalidSubscription
TEST_F(SubscriptionTest, Server_createMonitoredItems_invalidSubscription) {
    EXPECT_EQ(create_items_request(99999, {{.node = kAnswer}}).result, ua::StatusCode::BadSubscriptionIdInvalid);
}

// open62541: server/check_services_subscriptions.c Server_createMonitoredItems_maxPerSubscription
// open62541: server/check_services_subscriptions.c Server_createMonitoredItems_maxPerServer
TEST_F(SubscriptionTest, Server_createMonitoredItems_maxPerSubscription) {
    const Created c = create();
    const Created other = create();
    std::size_t created = 0;
    for (std::size_t i = 0; i < Subs::MAX_MONITORED_ITEMS; i++)
        created += create_item(i % 2 ? c.id : other.id, {.node = kAnswer}).status == ua::StatusCode::Good ? 1 : 0;
    EXPECT_EQ(created, Subs::MAX_MONITORED_ITEMS);
    EXPECT_EQ(create_item(c.id, {.node = kAnswer}).status, ua::StatusCode::BadTooManyMonitoredItems);
}

// open62541: server/check_services_subscriptions.c Server_createMonitoredItems_maxMonitoredItemsPerCall
TEST_F(SubscriptionTest, Server_createMonitoredItems_maxMonitoredItemsPerCall) {
    const Created c = create();
    const std::vector<Item> items(Subs::MAX_ITEMS_PER_CALL + 1, Item{.node = kAnswer});
    EXPECT_EQ(create_items_request(c.id, items).result, ua::StatusCode::BadTooManyOperations);
    EXPECT_EQ(create_items_request(c.id, {}).result, ua::StatusCode::BadNothingToDo);
    EXPECT_EQ(create_items_request(c.id, {{.node = kAnswer}}, 7).result, ua::StatusCode::BadTimestampsToReturnInvalid);
}

// open62541: server/check_services_subscriptions.c Server_negativeSamplingInterval
// open62541: server/check_services_subscriptions.c Server_dataSourceSamplingIntervalZero
TEST_F(SubscriptionTest, Server_negativeSamplingInterval) {
    const Created c = create(1000);
    EXPECT_EQ(create_item(c.id, {.node = kTemperature, .sampling = -1}).sampling, 1000) << "-1: publishing interval";
    EXPECT_EQ(create_item(c.id, {.node = kTemperature, .sampling = 0}).sampling, ua::MIN_SAMPLING_INTERVAL_MS);
    EXPECT_EQ(create_item(c.id, {.node = kTemperature, .sampling = 10}).sampling, ua::MIN_SAMPLING_INTERVAL_MS);
    EXPECT_EQ(create_item(c.id, {.node = kTemperature, .sampling = 200}).sampling, 200);
}

// open62541: server/check_services_subscriptions.c Server_nanModifiedSamplingInterval
TEST_F(SubscriptionTest, Server_nanModifiedSamplingInterval) {
    const Created c = create(300);
    const ItemResult item = create_item(c.id, {.node = kAnswer, .sampling = 100});
    const ItemResult m = modify_item(c.id, item.id, {.sampling = std::nan("")});
    EXPECT_EQ(m.status, ua::StatusCode::Good);
    EXPECT_TRUE(std::isfinite(m.sampling));
    EXPECT_EQ(m.sampling, 300);
}

// open62541: server/check_services_subscriptions.c Server_modifyMonitoredItems
// open62541: server/check_services_subscriptions_modify.c modifyMonitoredItem
TEST_F(SubscriptionTest, Server_modifyMonitoredItems) {
    const Created c = create();
    const ItemResult item = create_item(c.id, {.node = kAnswer, .handle = 1});
    const ItemResult m = modify_item(c.id, item.id, {.handle = 9, .sampling = 200, .queue = 3});
    EXPECT_EQ(m.status, ua::StatusCode::Good);
    EXPECT_EQ(m.sampling, 200);
    EXPECT_EQ(m.queue, 3u);
    EXPECT_EQ(modify_item(c.id, 4711, {.queue = 1}).status, ua::StatusCode::BadMonitoredItemIdInvalid);
    const auto p = publish_and_take(200);
    ASSERT_TRUE(p);
    ASSERT_FALSE(p->data.empty());
    EXPECT_EQ(p->data[0].handle, 9u) << "the new client handle";
}

// open62541: server/check_services_subscriptions.c Server_modifyMonitoredItems_invalidSubscription
// open62541: server/check_services_subscriptions.c Server_ModifyMonitoredItems_invalidTimestampsToReturn
TEST_F(SubscriptionTest, Server_modifyMonitoredItems_invalidSubscription) {
    const Created c = create();
    const ItemResult item = create_item(c.id, {.node = kAnswer});
    EXPECT_EQ(modify_items_request(99999, {{item.id, {}}}).result, ua::StatusCode::BadSubscriptionIdInvalid);
    EXPECT_EQ(modify_items_request(c.id, {{item.id, {}}}, 4).result, ua::StatusCode::BadTimestampsToReturnInvalid);
}

// Queue overflow (Part 4 5.12.1.5): discardOldest drops the oldest value and sets the
// overflow bit on the new oldest; a queue of one never carries the bit.
// open62541: server/check_services_subscriptions.c Server_overflow
TEST_F(SubscriptionTest, Server_overflow) {
    const Created c = create(1000);
    const ItemResult item = create_item(c.id, {.node = kAnswer, .sampling = 50, .queue = 3});
    EXPECT_EQ(item.queue, 3u);
    for (ua::Int32 v = 1; v <= 4; v++) {  // initial 42, then 1..4: five values for three places
        g_answer = v;
        advance(50);
    }
    auto p = publish_and_take(1000);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 3u);
    EXPECT_EQ(int32(p->data[0]), 2);
    EXPECT_EQ(static_cast<ua::UInt32>(p->data[0].value.status) & 0x480u, 0x480u) << "overflow bit on the oldest";
    EXPECT_EQ(int32(p->data[2]), 4);
    EXPECT_EQ(p->data[2].value.status, ua::StatusCode::Good);

    // discardNewest: the newest value is replaced, with the overflow bit
    ASSERT_EQ(modify_item(c.id, item.id, {.sampling = 50, .queue = 2, .discardOldest = false}).status, ua::StatusCode::Good);
    for (ua::Int32 v = 10; v <= 12; v++) {
        g_answer = v;
        advance(50);
    }
    p = publish_and_take(1000);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 2u);
    EXPECT_EQ(int32(p->data[0]), 10);
    EXPECT_EQ(int32(p->data[1]), 12);
    EXPECT_EQ(static_cast<ua::UInt32>(p->data[1].value.status) & 0x480u, 0x480u);

    // queue size 1: never an overflow bit
    ASSERT_EQ(modify_item(c.id, item.id, {.sampling = 50, .queue = 1}).status, ua::StatusCode::Good);
    for (ua::Int32 v = 20; v <= 22; v++) {
        g_answer = v;
        advance(50);
    }
    p = publish_and_take(1000);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u);
    EXPECT_EQ(int32(p->data[0]), 22);
    EXPECT_EQ(p->data[0].value.status, ua::StatusCode::Good);
}

// open62541: server/check_services_subscriptions.c Server_setMonitoringMode
// open62541: server/check_services_subscriptions.c Server_setMonitoringMode_invalid
// open62541: server/check_services_subscriptions_modify.c setMonitoringMode
TEST_F(SubscriptionTest, Server_setMonitoringMode) {
    const Created c = create();
    const ItemResult item = create_item(c.id, {.node = kAnswer});
    Response resp = set_mode(c.id, 0, {item.id, 4711});
    EXPECT_EQ(statuses(resp),
              (std::vector<ua::StatusCode>{ua::StatusCode::Good, ua::StatusCode::BadMonitoredItemIdInvalid}));
    auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    EXPECT_TRUE(p->keepAlive) << "disabled: no sampling, no notification";
    resp = set_mode(c.id, 2, {item.id});
    p = publish_and_take(200);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u) << "re-enabled: the current value";
    EXPECT_EQ(int32(p->data[0]), 42);
    EXPECT_EQ(set_mode(c.id, 5, {item.id}).result, ua::StatusCode::BadMonitoringModeInvalid);
    EXPECT_EQ(set_mode(99999, 2, {item.id}).result, ua::StatusCode::BadSubscriptionIdInvalid);
}

// open62541: server/check_services_subscriptions.c Server_SetMonitoringMode_maxMonitoredItemsPerCall
TEST_F(SubscriptionTest, Server_SetMonitoringMode_maxMonitoredItemsPerCall) {
    const Created c = create();
    const std::vector<ua::UInt32> ids(Subs::MAX_ITEMS_PER_CALL + 1, 1);
    EXPECT_EQ(set_mode(c.id, 1, ids).result, ua::StatusCode::BadTooManyOperations);
}

// open62541: server/check_services_subscriptions.c Server_deleteMonitoredItems
// open62541: server/check_services_subscriptions.c Server_deleteMonitoredItems_invalidSubscription
// open62541: server/check_services_subscriptions.c Server_deleteMonitoredItems_partial_keepsTreeConsistent
TEST_F(SubscriptionTest, Server_deleteMonitoredItems) {
    const Created c = create();
    const auto items = create_items(c.id, {{.node = kAnswer, .handle = 1}, {.node = kTemperature, .handle = 2},
                                           {.node = kReadonly, .handle = 3}});
    ASSERT_EQ(items.size(), 3u);
    Response resp = delete_items(c.id, {items[1].id, 4711});
    EXPECT_EQ(statuses(resp),
              (std::vector<ua::StatusCode>{ua::StatusCode::Good, ua::StatusCode::BadMonitoredItemIdInvalid}));
    EXPECT_EQ(delete_items(99999, {items[0].id}).result, ua::StatusCode::BadSubscriptionIdInvalid);
    const auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 2u) << "the others still report";
    EXPECT_EQ(p->data[0].handle, 1u);
    EXPECT_EQ(p->data[1].handle, 3u);
}

// open62541: server/check_services_subscriptions.c Server_monitoredItems_sameNode_list
// open62541: server/check_services_subscriptions_modify.c multipleMonitoredItems
TEST_F(SubscriptionTest, Server_monitoredItems_sameNode_list) {
    const Created c = create();
    ASSERT_EQ(create_items(c.id, {{.node = kAnswer, .handle = 1}, {.node = kAnswer, .handle = 2}}).size(), 2u);
    const auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 2u);
    EXPECT_EQ(p->data[0].handle, 1u);
    EXPECT_EQ(p->data[1].handle, 2u);
}

// open62541: server/check_services_subscriptions_modify.c multipleSubscriptions
TEST_F(SubscriptionTest, multipleSubscriptions) {
    const Created a = create(100), b = create(100);
    create_item(a.id, {.node = kAnswer, .handle = 1});
    create_item(b.id, {.node = kTemperature, .handle = 2});
    EXPECT_EQ(publish(), ua::StatusCode::Good);
    EXPECT_EQ(publish(), ua::StatusCode::Good);
    advance(100);
    const auto first = take(), second = take();
    ASSERT_TRUE(first && second);
    EXPECT_NE(first->subscription, second->subscription);
    EXPECT_EQ(first->data.size(), 1u);
    EXPECT_EQ(second->data.size(), 1u);
}

// open62541: server/check_services_subscriptions_modify.c timestampsToReturn_source
// open62541: server/check_services_subscriptions_modify.c timestampsToReturn_server
// open62541: server/check_services_subscriptions_modify.c timestampsToReturn_neither
TEST_F(SubscriptionTest, timestampsToReturn) {
    struct Case {
        ua::Int32 ttr;
        bool source, server;
    };
    for (const Case k : {Case{0, true, false}, Case{1, false, true}, Case{3, false, false}}) {
        subs.Clear();
        const Created c = create();
        create_item(c.id, {.node = kAnswer}, k.ttr);
        const auto p = publish_and_take(100);
        ASSERT_TRUE(p && p->data.size() == 1) << k.ttr;
        EXPECT_EQ(p->data[0].value.hasSource(), k.source) << k.ttr;
        EXPECT_EQ(p->data[0].value.hasServer(), k.server) << k.ttr;
    }
}

// A value change is reported in the next publishing cycle; an unchanged one is not.
// open62541: server/check_services_subscriptions_modify.c triggerDataChange
TEST_F(SubscriptionTest, triggerDataChange) {
    const Created c = create(100, 30, 10);
    create_item(c.id, {.node = kAnswer, .sampling = 50});
    auto p = publish_and_take(100);
    ASSERT_TRUE(p && p->data.size() == 1);
    EXPECT_EQ(publish(), ua::StatusCode::Good);
    advance(300);
    EXPECT_FALSE(take()) << "no change, no keep-alive yet";
    g_answer = 43;
    advance(100);
    p = take();
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u);
    EXPECT_EQ(int32(p->data[0]), 43);
}

// The first publishing cycle publishes at once: the initial values (or a keep-alive).
// open62541: server/check_services_subscriptions.c Server_firstPublishingCycleNotBatched
TEST_F(SubscriptionTest, Server_firstPublishingCycleNotBatched) {
    const Created c = create(1000, 30, 10);
    create_item(c.id, {.node = kAnswer, .sampling = 50});
    EXPECT_EQ(publish(), ua::StatusCode::Good);
    advance(990);
    EXPECT_FALSE(take());
    advance(10);
    const auto p = take();
    ASSERT_TRUE(p);
    EXPECT_EQ(p->data.size(), 1u) << "after one interval, not after the keep-alive count";
}

// ---- triggering ---------------------------------------------------------------------------------

// open62541: server/check_services_subscriptions.c Server_setTriggering_nothingToDo
// open62541: server/check_services_subscriptions.c Server_setTriggering_invalidSubscription
// open62541: server/check_services_subscriptions.c Server_setTriggering_invalidMonitoredItem
// open62541: server/check_services_subscriptions.c Server_setTriggering_maxMonitoredItemsPerCall
TEST_F(SubscriptionTest, Server_setTriggering_errors) {
    const Created c = create();
    const ItemResult item = create_item(c.id, {.node = kAnswer});
    EXPECT_EQ(set_triggering(c.id, item.id, {}, {}).result, ua::StatusCode::BadNothingToDo);
    EXPECT_EQ(set_triggering(99999, 1, {item.id}, {}).result, ua::StatusCode::BadSubscriptionIdInvalid);
    EXPECT_EQ(set_triggering(c.id, 99999, {item.id}, {}).result, ua::StatusCode::BadMonitoredItemIdInvalid);
    const std::vector<ua::UInt32> many(Subs::MAX_ITEMS_PER_CALL + 1, item.id);
    EXPECT_EQ(set_triggering(c.id, item.id, many, {}).result, ua::StatusCode::BadTooManyOperations);
}

// A linked item in Sampling mode reports when its triggering item reports.
// open62541: server/check_services_subscriptions.c Server_setTriggering_addAndRemoveLinks
TEST_F(SubscriptionTest, Server_setTriggering_addAndRemoveLinks) {
    const Created c = create(100);
    const ItemResult trigger = create_item(c.id, {.node = kAnswer, .handle = 1, .sampling = 50});
    const ItemResult linked = create_item(c.id, {.node = kTemperature, .mode = 1, .handle = 2, .sampling = 50});
    Response resp = set_triggering(c.id, trigger.id, {linked.id}, {});
    ASSERT_EQ(resp.result, ua::StatusCode::Good);
    auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 2u) << "the trigger's initial value triggers the linked item";
    g_temperature = 30;  // the sampled item changes, the trigger does not
    EXPECT_EQ(publish(), ua::StatusCode::Good);
    advance(200);
    EXPECT_FALSE(take()) << "a Sampling item alone does not report";
    g_answer = 1;
    advance(100);
    p = take();
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 2u);
    EXPECT_EQ(p->data[1].handle, 2u);
    resp = set_triggering(c.id, trigger.id, {}, {linked.id, 4711});
    ASSERT_EQ(resp.result, ua::StatusCode::Good);
    ua::ByteReader r = resp.reader();
    ua::Int32 adds = 0, diag = 0, removes = 0;
    ua::StatusCode a{}, b{};
    ASSERT_TRUE(r.ReadInt32(adds) && adds == 0 && r.ReadInt32(diag) && r.ReadInt32(removes) && removes == 2 &&
                r.ReadStatusCode(a) && r.ReadStatusCode(b));
    EXPECT_EQ(a, ua::StatusCode::Good);
    EXPECT_EQ(b, ua::StatusCode::BadMonitoredItemIdInvalid);
}

// A deleted item leaves the links of its triggering item.
// open62541: server/check_services_subscriptions.c Server_setTriggering_pruneDeletedLinks
TEST_F(SubscriptionTest, Server_setTriggering_pruneDeletedLinks) {
    const Created c = create();
    const ItemResult trigger = create_item(c.id, {.node = kAnswer});
    const ItemResult kept = create_item(c.id, {.node = kTemperature, .mode = 1});
    const ItemResult dropped = create_item(c.id, {.node = kReadonly, .mode = 1});
    ASSERT_EQ(set_triggering(c.id, trigger.id, {kept.id, dropped.id}, {}).result, ua::StatusCode::Good);
    ASSERT_EQ(delete_items(c.id, {dropped.id}).result, ua::StatusCode::Good);
    Response resp = set_triggering(c.id, trigger.id, {}, {dropped.id, kept.id});
    ua::ByteReader r = resp.reader();
    ua::Int32 adds = 0, diag = 0, removes = 0;
    ua::StatusCode a{}, b{};
    ASSERT_TRUE(r.ReadInt32(adds) && r.ReadInt32(diag) && r.ReadInt32(removes) && r.ReadStatusCode(a) &&
                r.ReadStatusCode(b));
    EXPECT_EQ(a, ua::StatusCode::BadMonitoredItemIdInvalid) << "pruned with the item";
    EXPECT_EQ(b, ua::StatusCode::Good);
}

// ---- filters ---------------------------------------------------------------------------------

// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsAbsoluteFilterSetOnCreate
// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsAbsoluteFilterSetLater
// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsAbsoluteFilterSetOnCreateRemoveLater
TEST_F(SubscriptionTest, Server_MonitoredItemsAbsoluteFilter) {
    const Created c = create(100);
    const ItemResult item = create_item(c.id, {.node = kAnswer, .sampling = 50, .queue = 4,
                                               .filter = Filter{.deadband = 1, .value = 2.0}});
    ASSERT_EQ(item.status, ua::StatusCode::Good);
    auto p = publish_and_take(100);
    ASSERT_TRUE(p && p->data.size() == 1);
    for (ua::Int32 v : {43, 44, 45}) {  // 45: more than 2 away from the last reported 42
        g_answer = v;
        advance(50);
    }
    p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u);
    EXPECT_EQ(int32(p->data[0]), 45);
    // filter removed: every change counts
    ASSERT_EQ(modify_item(c.id, item.id, {.sampling = 50, .queue = 4, .filter = Filter{}}).status, ua::StatusCode::Good);
    g_answer = 46;
    p = publish_and_take(100);
    ASSERT_TRUE(p);
    ASSERT_EQ(p->data.size(), 1u);
    EXPECT_EQ(int32(p->data[0]), 46);
}

// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsNoFilter
// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsSetEmpty
TEST_F(SubscriptionTest, Server_MonitoredItemsNoFilter) {
    const Created c = create(100);
    ASSERT_EQ(create_item(c.id, {.node = kAnswer, .sampling = 50, .queue = 4}).status, ua::StatusCode::Good);
    auto p = publish_and_take(100);
    ASSERT_TRUE(p);
    for (ua::Int32 v : {43, 44}) {
        g_answer = v;
        advance(50);
    }
    p = publish_and_take(100);
    ASSERT_TRUE(p);
    EXPECT_EQ(p->data.size(), 2u) << "every change";
}

// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsAbsoluteFilterOnBool
TEST_F(SubscriptionTest, Server_MonitoredItemsAbsoluteFilterOnBool) {
    const Created c = create();
    EXPECT_EQ(create_item(c.id, {.node = kBool, .filter = Filter{.deadband = 1, .value = 0.5}}).status,
              ua::StatusCode::BadFilterNotAllowed);
}

// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsPercentFilterSetOnCreateMissingEURange
// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsPercentFilterSetLaterMissingEURange
// open62541: server/check_monitoreditem_filter.c Server_MonitoredItemsPercentFilterSetOnCreateDeadBandValueOutOfRange
TEST_F(SubscriptionTest, Server_MonitoredItemsPercentFilterMissingEURange) {
    const Created c = create();
    EXPECT_EQ(create_item(c.id, {.node = kAnswer, .filter = Filter{.deadband = 2, .value = 2.0}}).status,
              ua::StatusCode::BadMonitoredItemFilterUnsupported);
    EXPECT_EQ(create_item(c.id, {.node = kAnswer, .filter = Filter{.deadband = 2, .value = 101}}).status,
              ua::StatusCode::BadMonitoredItemFilterUnsupported);
    const ItemResult item = create_item(c.id, {.node = kAnswer});
    EXPECT_EQ(modify_item(c.id, item.id, {.filter = Filter{.deadband = 2, .value = 2.0}}).status,
              ua::StatusCode::BadMonitoredItemFilterUnsupported);
}

// A filter that does not fit is rejected; the item stays as it was.
TEST_F(SubscriptionTest, OtherFiltersAreRejected) {
    const Created c = create();
    EXPECT_EQ(create_item(c.id, {.node = kAnswer, .filterType = ua::NodeId(0, ua::ns0::EventFilter)}).status,
              ua::StatusCode::BadFilterNotAllowed);
    EXPECT_EQ(create_item(c.id, {.node = kAnswer, .filterType = ua::NodeId(0, ua::ns0::AggregateFilter)}).status,
              ua::StatusCode::BadMonitoredItemFilterUnsupported);
    EXPECT_EQ(create_item(c.id, {.node = kAnswer, .attribute = 4, .filter = Filter{}}).status,
              ua::StatusCode::BadFilterNotAllowed)
        << "a DataChangeFilter on DisplayName";
    EXPECT_EQ(create_item(c.id, {.node = kAnswer, .filter = Filter{.deadband = 1, .value = -1}}).status,
              ua::StatusCode::BadDeadbandFilterInvalid);
}

// Deadband on the numeric types: changes within the deadband are not reported.
// open62541: server/check_datachange_deadband_typekinds.c Deadband_sbyte
// open62541: server/check_datachange_deadband_typekinds.c Deadband_uint16
// open62541: server/check_datachange_deadband_typekinds.c Deadband_float
TEST_F(SubscriptionTest, DeadbandTypeKinds) {
    struct Case {
        ua::NodeId node;
        ua::Variant* storage;
        ua::Variant small, big;
    };
    for (const Case& k : {Case{kSByte, &g_sbyte, ua::Variant::Of(ua::SByte{1}), ua::Variant::Of(ua::SByte{-5})},
                          Case{kUInt16, &g_uint16, ua::Variant::Of(ua::UInt16{1}), ua::Variant::Of(ua::UInt16{5})},
                          Case{kFloat, &g_float, ua::Variant::Of(ua::Float{0.5f}), ua::Variant::Of(ua::Float{2.5f})}}) {
        subs.Clear();
        const Created c = create(100);
        ASSERT_EQ(create_item(c.id, {.node = k.node, .sampling = 50, .queue = 4,
                                     .filter = Filter{.deadband = 1, .value = 2.0}})
                      .status,
                  ua::StatusCode::Good);
        auto p = publish_and_take(100);
        ASSERT_TRUE(p && p->data.size() == 1) << "initial value";
        *k.storage = k.small;
        EXPECT_EQ(publish(), ua::StatusCode::Good);
        advance(100);
        EXPECT_FALSE(take()) << "within the deadband: nothing to publish";
        *k.storage = k.big;
        advance(100);
        p = take();
        ASSERT_TRUE(p);
        ASSERT_EQ(p->data.size(), 1u) << "beyond the deadband";
        EXPECT_EQ(p->data[0].value.value, k.big);
    }
}

// ---- rtoos: Publish queue ----------------------------------------------------------------------

TEST_F(SubscriptionTest, PublishQueueLimitsAndTimeouts) {
    EXPECT_EQ(publish(), ua::StatusCode::BadNoSubscription);
    create(10'000, 30, 1000);
    for (std::size_t i = 0; i < Subs::MAX_PUBLISH_REQUESTS; i++) EXPECT_EQ(publish({}, 600 + i), ua::StatusCode::Good);
    EXPECT_EQ(publish(), ua::StatusCode::BadTooManyPublishRequests);
    // the first cycle answers the oldest request
    advance(10'000);
    auto p = take();
    ASSERT_TRUE(p);
    EXPECT_EQ(p->requestId, 600u);
    // a request whose TimeoutHint passed is answered with BadTimeout
    subs.Clear();
    create(10'000, 30, 1000);
    EXPECT_EQ(publish({}, 700, 500), ua::StatusCode::Good);
    advance(600);
    p = take();
    ASSERT_TRUE(p);
    EXPECT_TRUE(p->fault);
    EXPECT_EQ(p->status, ua::StatusCode::BadTimeout);
    EXPECT_EQ(p->requestId, 700u);
}

}  // namespace
