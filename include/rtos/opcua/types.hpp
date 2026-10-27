#pragma once
// Fundamental OPC UA Binary (Part 6) built-in types -- deliberately only what this server
// actually needs today (see the project's own effort-estimation discussion: the "full type
// canon" is added later purely by extending the Variant alternative list + TypeTable entries
// in variant.hpp/.cpp, no rework of the codec itself). Header-only where trivial; encode/decode
// logic lives in codec.hpp/.cpp (needs ByteReader/ByteWriter, which would create an include
// cycle if pulled in here).
//
// No exceptions/RTTI on this target (-fno-exceptions -fno-rtti, see cmake/gcc-arm-none-eabi.cmake)
// -- every fallible operation in this whole server returns bool/StatusCode, never throws.
//
// String/QualifiedName/LocalizedText/NodeId all hold std::string_view, NOT std::string --
// deliberately non-owning (project requirement: the address space must be a compile-time-fixed
// constexpr/Flash-resident data structure, not built by runtime function calls, see
// address_space.hpp). This is safe on both ends: constant node data (BrowseName/DisplayName/
// static Values) points at literal/static storage that outlives the program, and decoded wire
// strings only ever need to stay valid for the duration of one synchronous
// Connection::HandleMessage() call (nothing in this server stores a decoded string past that
// call -- see connection.cpp/net_transport.cpp) -- the underlying bytes live in the session's
// receive buffer, which isn't reused until the current message has been fully processed.
#include <algorithm>
#include <cstdint>
#include <cstddef>
#include <string_view>
#include <variant>

namespace rtos::opcua {

using Boolean = bool;
using SByte = int8_t;
using Byte = uint8_t;
using Int16 = int16_t;
using UInt16 = uint16_t;
using Int32 = int32_t;
using UInt32 = uint32_t;
using Int64 = int64_t;
using UInt64 = uint64_t;
using Float = float;
using Double = double;

// 100ns ticks since 1601-01-01 (the OPC UA / Windows FILETIME epoch) -- same convention used
// project-wide for any future RTC/SNTP integration, kept consistent here even though this
// board has neither (see opcua::Now() in clock.cpp, tx_time_get()-based like the open62541
// port's arch layer was).
//
// TODO(later): this board has no RTC today, so Now() is boot-relative, not calendar-accurate
// (see clock.cpp). Once a real RTC exists on this project, clock.cpp's Now() needs to read it
// instead -- flagged explicitly by the user (2026-08-19) as a follow-up, not in scope now.
using DateTime = int64_t;
constexpr int64_t DATETIME_UNIX_EPOCH = 11644473600LL * 10000000LL;
constexpr int64_t DATETIME_SEC = 10000000LL;

// OPC UA Part 6 6.2.5: length -1 (0xFFFFFFFF on the wire) is the "null string", distinct from
// a present-but-empty string. std::string_view can't represent that distinction on its own, so
// it carries alongside a bool -- default-constructed String() is null (matches a not-yet-set
// optional attribute), String("") is the empty-but-present string.
struct String {
    std::string_view value;
    bool isNull = true;

    constexpr String() = default;
    constexpr String(std::string_view v) : value(v), isNull(false) {}
    static constexpr String Null() { return String{}; }
    friend constexpr bool operator==(const String &, const String &) = default;
};

// Part 6 5.1.3: StatusCode is a plain UInt32, top 16 bits are the severity+code, low 16 bits
// are (mostly-unused-by-us) sub-flags. Only the handful of codes this server actually returns
// are named here -- add more as needed, exactly like the Variant type table.
enum class StatusCode : UInt32 {
    Good = 0x00000000,
    BadInternalError = 0x80020000,
    BadOutOfMemory = 0x80030000,
    BadNotImplemented = 0x80400000,
    BadNotSupported = 0x803D0000,
    BadInvalidArgument = 0x80AB0000,
    BadOutOfRange = 0x803C0000,
    BadNodeIdInvalid = 0x80330000,
    BadNodeIdUnknown = 0x80340000,
    BadAttributeIdInvalid = 0x80350000,
    BadNotReadable = 0x803A0000,
    BadNotWritable = 0x803B0000,
    BadTypeMismatch = 0x80740000,
    BadUserAccessDenied = 0x801F0000,
    BadSecurityChecksFailed = 0x80130000,
    BadSecurityPolicyRejected = 0x80550000,
    BadSecurityModeRejected = 0x80540000,
    // A service other than GetEndpoints was requested on the SecurityPolicy#None
    // discovery-only channel (Part 4 5.4.4 -- GetEndpoints is the one service usable without
    // security; everything else needs a real, appropriately-secured channel).
    BadSecurityModeInsufficient = 0x80E60000,
    BadSecureChannelIdInvalid = 0x80220000,
    BadSecureChannelClosed = 0x80860000,
    BadRequestTypeInvalid = 0x80530000,
    BadRequestHeaderInvalid = 0x802A0000,
    BadTimestampsToReturnInvalid = 0x802B0000,
    BadSessionIdInvalid = 0x80250000,
    BadSessionClosed = 0x80260000,
    BadSessionNotActivated = 0x80270000,
    BadIdentityTokenInvalid = 0x80200000,
    BadIdentityTokenRejected = 0x80210000,
    BadTooManySessions = 0x80560000,
    BadServiceUnsupported = 0x800B0000,
    BadDecodingError = 0x80070000,
    BadEncodingError = 0x80060000,
    BadEncodingLimitsExceeded = 0x80080000,
    BadNothingToDo = 0x800F0000,
    BadNoMatch = 0x806F0000,
    BadTcpMessageTypeInvalid = 0x807E0000,
    BadTcpMessageTooLarge = 0x80800000,
    BadConnectionClosed = 0x80AE0000,
    BadUnexpectedError = 0x80010000,
    UncertainInitialValue = 0x40920000,
    BadNonceInvalid = 0x80240000,
    BadCertificateInvalid = 0x80120000,
    BadApplicationSignatureInvalid = 0x80580000,
    BadCertificateUriInvalid = 0x80170000,
    BadMaxAgeInvalid = 0x80700000,
    BadTooManyOperations = 0x80100000,
    BadIndexRangeInvalid = 0x80360000,
    BadIndexRangeNoData = 0x80370000,
    BadDataEncodingInvalid = 0x80380000,
    BadDataEncodingUnsupported = 0x80390000,
    BadWriteNotSupported = 0x80730000,
    BadBrowseDirectionInvalid = 0x804D0000,
    BadViewIdUnknown = 0x806B0000,
    BadReferenceTypeIdInvalid = 0x804C0000,
    BadContinuationPointInvalid = 0x804A0000,
    BadNoContinuationPoints = 0x804B0000,
    BadBrowseNameInvalid = 0x80600000,
    BadSubscriptionIdInvalid = 0x80280000,
    BadMonitoredItemIdInvalid = 0x80420000,
    BadMonitoringModeInvalid = 0x80410000,
    BadMonitoredItemFilterInvalid = 0x80430000,
    BadMonitoredItemFilterUnsupported = 0x80440000,
    BadFilterNotAllowed = 0x80450000,
    BadTooManySubscriptions = 0x80770000,
    BadTooManyMonitoredItems = 0x80DB0000,
    BadTooManyPublishRequests = 0x80780000,
    BadNoSubscription = 0x80790000,
    BadSequenceNumberUnknown = 0x807A0000,
    BadMessageNotAvailable = 0x807B0000,
    BadDeadbandFilterInvalid = 0x808E0000,
    BadTimeout = 0x800A0000,
    BadRequestTooLarge = 0x80B80000,
    BadResponseTooLarge = 0x80B90000,
    BadSequenceNumberInvalid = 0x80880000,
    BadSecureChannelTokenUnknown = 0x80870000,
    BadNodeClassInvalid = 0x805F0000,
    BadNodeAttributesInvalid = 0x80620000,
    BadTcpEndpointUrlInvalid = 0x80830000,
    GoodSubscriptionTransferred = 0x002D0000,
    BadTooManyArguments = 0x80E50000,
    GoodCompletesAsynchronously = 0x002E0000,
    BadNotFound = 0x803E0000,
    BadInvalidState = 0x80AF0000,
    BadTimestampNotSupported = 0x80A10000,
    BadWaitingForInitialData = 0x80320000,
    BadServerNotConnected = 0x800D0000,
    BadCertificateTimeInvalid = 0x80140000,
    BadCertificateUntrusted = 0x801A0000,
};

constexpr bool IsGood(StatusCode s) { return (static_cast<UInt32>(s) & 0x80000000u) == 0; }
constexpr bool IsBad(StatusCode s) { return !IsGood(s); }

// Part 6 5.2.2: three encodings actually used on the wire by this server -- Numeric (all our
// own NodeIds, spec-optimized into Two-Byte/Four-Byte/Numeric sub-forms on ENCODE) plus String/
// Guid/Opaque on DECODE only (a client could in principle echo one back, even though we never
// hand one out -- see codec.cpp DecodeNodeId()). NamespaceIndex 0 is reserved for the standard
// OPC UA namespace (NS0); this server's own address space lives in namespace 1.
enum class NodeIdType : Byte { Numeric, String, Guid, Opaque };

struct Guid {
    UInt32 data1 = 0;
    UInt16 data2 = 0;
    UInt16 data3 = 0;
    Byte data4[8] = {};
    friend constexpr bool operator==(const Guid &, const Guid &) = default;
};

struct NodeId {
    UInt16 namespaceIndex = 0;
    NodeIdType type = NodeIdType::Numeric;
    UInt32 numeric = 0;
    std::string_view string; // used for String and (raw bytes) Opaque -- see codec.cpp DecodeNodeId()
    Guid guid;

    constexpr NodeId() = default;
    constexpr NodeId(UInt16 ns, UInt32 id) : namespaceIndex(ns), type(NodeIdType::Numeric), numeric(id) {}
    /// A String NodeId (e.g. ns=1;s=the.answer); the text must outlive the NodeId.
    static constexpr NodeId Str(UInt16 ns, std::string_view text) {
        NodeId id;
        id.namespaceIndex = ns;
        id.type = NodeIdType::String;
        id.string = text;
        return id;
    }

    friend constexpr bool operator==(const NodeId &a, const NodeId &b) {
        if(a.namespaceIndex != b.namespaceIndex || a.type != b.type)
            return false;
        switch(a.type) {
        case NodeIdType::Numeric: return a.numeric == b.numeric;
        case NodeIdType::String:  return a.string == b.string;
        case NodeIdType::Opaque:  return a.string == b.string;
        case NodeIdType::Guid:
            return a.guid.data1 == b.guid.data1 && a.guid.data2 == b.guid.data2 &&
                   a.guid.data3 == b.guid.data3 &&
                   std::equal(std::begin(a.guid.data4), std::end(a.guid.data4), std::begin(b.guid.data4));
        }
        return false;
    }
    friend constexpr bool operator!=(const NodeId &a, const NodeId &b) { return !(a == b); }

    constexpr bool IsNull() const { return namespaceIndex == 0 && type == NodeIdType::Numeric && numeric == 0; }
};

// ExpandedNodeId (Part 6 5.2.3): NodeId + optional namespaceUri/serverIndex. This server never
// references another server or a namespace-by-URI, so it's kept to "just a NodeId" -- the
// encoder always writes the two optional fields absent (matches how open62541 and most minimal
// servers behave for local-only address spaces).
using ExpandedNodeId = NodeId;

struct QualifiedName {
    UInt16 namespaceIndex = 0;
    std::string_view name;

    constexpr QualifiedName() = default;
    constexpr QualifiedName(UInt16 ns, std::string_view n) : namespaceIndex(ns), name(n) {}
    friend constexpr bool operator==(const QualifiedName &, const QualifiedName &) = default;
};

struct LocalizedText {
    std::string_view locale;
    std::string_view text;

    constexpr LocalizedText() = default;
    constexpr LocalizedText(std::string_view loc, std::string_view t) : locale(loc), text(t) {}
    friend constexpr bool operator==(const LocalizedText &, const LocalizedText &) = default;
};

// NodeClass (Part 3 5.2.2) -- only the classes this server's address space actually uses.
enum class NodeClass : Int32 {
    Unspecified = 0,
    Object = 1,
    Variable = 2,
    ObjectType = 8,
    VariableType = 16,
    ReferenceType = 32,
    DataType = 64,
    View = 128,
    Method = 4,
};

}  // namespace rtos::opcua
