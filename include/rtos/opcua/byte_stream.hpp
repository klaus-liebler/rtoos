#pragma once
// Flat, non-owning, bounds-checked binary reader/writer -- no heap allocation, no exceptions
// (matches -fno-exceptions, see cmake/gcc-arm-none-eabi.cmake). Every OPC UA Binary value on
// this server lives in a single contiguous per-session/per-message buffer (never a chained
// NX_PACKET scatter/gather list -- the transport layer, message_chunker.cpp, is responsible
// for linearizing before handing bytes to the codec), so this deliberately doesn't support
// non-contiguous sources.
//
// Every Read*/Write* returns bool. Once any operation fails, the stream is "poisoned" (ok_
// becomes false) and every subsequent operation becomes a cheap no-op returning false --
// callers can therefore chain a whole message's worth of Read calls and check Ok() once at the
// end, instead of checking every individual return value (mirrors how open62541's own
// UA_StatusCode-threading decode functions behave, just expressed as a stream object instead
// of a threaded status code).
#include <cstring>
#include <span>
#include <type_traits>

#include "rtos/opcua/types.hpp"

namespace rtos::opcua {

class ByteReader {
public:
    explicit ByteReader(std::span<const Byte> data) : data_(data) {}

    bool Ok() const { return ok_; }
    size_t Position() const { return pos_; }
    size_t Remaining() const { return ok_ ? (data_.size() - pos_) : 0; }

    bool ReadByte(Byte &out) { return ReadRaw(&out, 1); }
    bool ReadSByte(SByte &out) { return ReadRaw(&out, 1); }
    bool ReadBoolean(Boolean &out) {
        Byte b = 0;
        if(!ReadByte(b)) return false;
        out = (b != 0);
        return true;
    }
    bool ReadInt16(Int16 &out) { return ReadLE(out); }
    bool ReadUInt16(UInt16 &out) { return ReadLE(out); }
    bool ReadInt32(Int32 &out) { return ReadLE(out); }
    bool ReadUInt32(UInt32 &out) { return ReadLE(out); }
    bool ReadInt64(Int64 &out) { return ReadLE(out); }
    bool ReadUInt64(UInt64 &out) { return ReadLE(out); }
    bool ReadFloat(Float &out) { return ReadLE(out); }
    bool ReadDouble(Double &out) { return ReadLE(out); }
    bool ReadDateTime(DateTime &out) { return ReadLE(out); }
    bool ReadStatusCode(StatusCode &out) {
        UInt32 raw = 0;
        if(!ReadUInt32(raw)) return false;
        out = static_cast<StatusCode>(raw);
        return true;
    }

    // Part 6 5.2.1.2: length-prefixed (Int32), -1 (0xFFFFFFFF) means "null". Caller
    // distinguishes null vs. empty via the returned String::isNull.
    bool ReadString(String &out) {
        Int32 length = 0;
        if(!ReadInt32(length)) return false;
        if(length < 0) {
            out = String::Null();
            return true;
        }
        if(!CheckRemaining(static_cast<size_t>(length))) return Fail();
        out = String(std::string_view(reinterpret_cast<const char*>(data_.data() + pos_),
                                      static_cast<size_t>(length)));
        pos_ += static_cast<size_t>(length);
        return true;
    }
    // ByteString shares the exact same wire format as String (Part 6 5.2.1.2/5.2.1.3) -- only
    // the C++-side interpretation differs (raw bytes vs. text). Exposed separately so callers
    // don't have to reach through String::value for binary payloads. Non-owning, like
    // ReadString() -- valid only until the underlying buffer is reused (see types.hpp).
    bool ReadByteString(std::string_view &out, bool &isNull) {
        Int32 length = 0;
        if(!ReadInt32(length)) return false;
        if(length < 0) {
            out = {};
            isNull = true;
            return true;
        }
        if(!CheckRemaining(static_cast<size_t>(length))) return Fail();
        out = std::string_view(reinterpret_cast<const char*>(data_.data() + pos_), static_cast<size_t>(length));
        isNull = false;
        pos_ += static_cast<size_t>(length);
        return true;
    }

    bool ReadRaw(void *dst, size_t n) {
        if(!CheckRemaining(n)) return Fail();
        std::memcpy(dst, data_.data() + pos_, n);
        pos_ += n;
        return true;
    }

    // Advances without copying -- e.g. to skip a not-yet-supported field.
    bool Skip(size_t n) {
        if(!CheckRemaining(n)) return Fail();
        pos_ += n;
        return true;
    }

    std::span<const Byte> RemainingSpan() const { return data_.subspan(pos_); }

private:
    bool CheckRemaining(size_t n) const { return ok_ && n <= (data_.size() - pos_); }
    bool Fail() { ok_ = false; return false; }

    template <typename T>
    bool ReadLE(T &out) {
        static_assert(std::is_trivially_copyable_v<T>);
        if(!CheckRemaining(sizeof(T))) return Fail();
        // Cortex-M33 is little-endian and OPC UA Binary is little-endian on the wire (Part 6
        // 5.2.1) -- a plain memcpy is correct without any byte-swapping.
        std::memcpy(&out, data_.data() + pos_, sizeof(T));
        pos_ += sizeof(T);
        return true;
    }

    std::span<const Byte> data_;
    size_t pos_ = 0;
    bool ok_ = true;
};

class ByteWriter {
public:
    explicit ByteWriter(std::span<Byte> data) : data_(data) {}

    bool Ok() const { return ok_; }
    size_t Position() const { return pos_; }
    size_t Remaining() const { return ok_ ? (data_.size() - pos_) : 0; }
    std::span<const Byte> Written() const { return data_.subspan(0, pos_); }

    bool WriteByte(Byte v) { return WriteRaw(&v, 1); }
    bool WriteSByte(SByte v) { return WriteRaw(&v, 1); }
    bool WriteBoolean(Boolean v) { Byte b = v ? 1 : 0; return WriteByte(b); }
    bool WriteInt16(Int16 v) { return WriteLE(v); }
    bool WriteUInt16(UInt16 v) { return WriteLE(v); }
    bool WriteInt32(Int32 v) { return WriteLE(v); }
    bool WriteUInt32(UInt32 v) { return WriteLE(v); }
    bool WriteInt64(Int64 v) { return WriteLE(v); }
    bool WriteUInt64(UInt64 v) { return WriteLE(v); }
    bool WriteFloat(Float v) { return WriteLE(v); }
    bool WriteDouble(Double v) { return WriteLE(v); }
    bool WriteDateTime(DateTime v) { return WriteLE(v); }
    bool WriteStatusCode(StatusCode v) { return WriteUInt32(static_cast<UInt32>(v)); }

    bool WriteString(const String &v) {
        if(v.isNull) return WriteInt32(-1);
        if(!WriteInt32(static_cast<Int32>(v.value.size()))) return false;
        return WriteRaw(v.value.data(), v.value.size());
    }
    bool WriteString(std::string_view v) { return WriteString(String(v)); }
    bool WriteByteString(std::string_view v, bool isNull) {
        if(isNull) return WriteInt32(-1);
        if(!WriteInt32(static_cast<Int32>(v.size()))) return false;
        return WriteRaw(v.data(), v.size());
    }

    bool WriteRaw(const void *src, size_t n) {
        if(!CheckRemaining(n)) return Fail();
        std::memcpy(data_.data() + pos_, src, n);
        pos_ += n;
        return true;
    }

    // Reserves "n" bytes and returns a span to write into directly (e.g. for a length field
    // that's only known after encoding the body that follows it) -- caller is responsible for
    // filling every byte.
    std::span<Byte> Reserve(size_t n) {
        if(!CheckRemaining(n)) { Fail(); return {}; }
        std::span<Byte> out = data_.subspan(pos_, n);
        pos_ += n;
        return out;
    }

    // Overwrites 4 already-written bytes at "offset" (< Position()) with a little-endian
    // UInt32 -- the standard "backfill a length field once the body's size is known" pattern
    // used by every UA TCP message (see tcp_message.cpp/secure_channel.cpp). Does nothing (and
    // returns false) if offset+4 wasn't already written or the stream has failed.
    bool PatchUInt32(size_t offset, UInt32 value) {
        if(!ok_ || offset + sizeof(UInt32) > pos_) return false;
        std::memcpy(data_.data() + offset, &value, sizeof(UInt32));
        return true;
    }
    bool PatchByte(size_t offset, Byte value) {
        if(!ok_ || offset >= pos_) return false;
        data_[offset] = value;
        return true;
    }
    /// The bytes written from `offset` on, writable (in-place encryption of a finished message).
    std::span<Byte> WrittenFrom(size_t offset) { return data_.subspan(offset, pos_ - offset); }

private:
    bool CheckRemaining(size_t n) const { return ok_ && n <= (data_.size() - pos_); }
    bool Fail() { ok_ = false; return false; }

    template <typename T>
    bool WriteLE(T v) {
        static_assert(std::is_trivially_copyable_v<T>);
        if(!CheckRemaining(sizeof(T))) return Fail();
        std::memcpy(data_.data() + pos_, &v, sizeof(T));
        pos_ += sizeof(T);
        return true;
    }

    std::span<Byte> data_;
    size_t pos_ = 0;
    bool ok_ = true;
};

}  // namespace rtos::opcua
