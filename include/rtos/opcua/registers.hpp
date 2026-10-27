#pragma once
// Variables backed by Modbus registers (rtos::modbus::RegisterMap): OPC UA and Modbus see the
// same values, as in the FCU firmware's generated address space. A node's userContext is a
// constexpr Register (table, address); Bind() sets the map once at startup -- the node tables
// themselves stay constexpr (Flash).
//
//   constexpr opcua::registers::Register kLed{modbus::Table::Holding, 0};
//   Node{..., .readCallback = &registers::ReadUInt16, .writeCallback = &registers::WriteUInt16,
//        .userContext = registers::Context(kLed)}
//
// 32-bit values occupy two registers, high word first (the Modbus convention of the FCU).
// Every read or write is one RegisterMap block access: consistent under its guard.
#include "rtos/modbus/modbus.hpp"
#include "rtos/opcua/variant.hpp"

namespace rtos::opcua::registers {

struct Register {
    modbus::Table table;
    UInt16 address;
};

inline modbus::RegisterMap *g_map = nullptr;
inline void Bind(modbus::RegisterMap &map) { g_map = &map; }

constexpr void *Context(const Register &r) { return const_cast<void *>(static_cast<const void *>(&r)); }

namespace detail {
inline bool Read(void *context, std::span<UInt16> out) {
    const auto &r = *static_cast<const Register *>(context);
    return g_map != nullptr && r.address + out.size() <= g_map->size(r.table) &&
           g_map->read(r.table, r.address, out) == modbus::Exception::None;
}
inline bool Write(void *context, std::span<const UInt16> values) {
    const auto &r = *static_cast<const Register *>(context);
    return g_map != nullptr && r.table == modbus::Table::Holding && r.address + values.size() <= g_map->size(r.table) &&
           g_map->write(r.address, values) == modbus::Exception::None;
}
template <class T>
bool Value(const Variant &v, T &out) {
    if(const T *p = std::get_if<T>(&v.storage)) {
        out = *p;
        return true;
    }
    return false;
}
} // namespace detail

inline Variant ReadUInt16(void *context) {
    UInt16 v = 0;
    return detail::Read(context, {&v, 1}) ? Variant::Of(v) : Variant{};
}
inline bool WriteUInt16(void *context, const Variant &value) {
    UInt16 v = 0;
    return detail::Value(value, v) && detail::Write(context, {&v, 1});
}
inline Variant ReadInt16(void *context) {
    UInt16 v = 0;
    return detail::Read(context, {&v, 1}) ? Variant::Of(static_cast<Int16>(v)) : Variant{};
}
inline bool WriteInt16(void *context, const Variant &value) {
    Int16 v = 0;
    if(!detail::Value(value, v)) return false;
    const auto u = static_cast<UInt16>(v);
    return detail::Write(context, std::span<const UInt16>(&u, 1));
}
inline Variant ReadUInt32(void *context) {
    UInt16 w[2] = {};
    return detail::Read(context, w) ? Variant::Of(static_cast<UInt32>((UInt32{w[0]} << 16) | w[1])) : Variant{};
}
inline bool WriteUInt32(void *context, const Variant &value) {
    UInt32 v = 0;
    if(!detail::Value(value, v)) return false;
    const UInt16 w[2] = {static_cast<UInt16>(v >> 16), static_cast<UInt16>(v)};
    return detail::Write(context, w);
}
inline Variant ReadInt32(void *context) {
    UInt16 w[2] = {};
    return detail::Read(context, w) ? Variant::Of(static_cast<Int32>((UInt32{w[0]} << 16) | w[1])) : Variant{};
}

} // namespace rtos::opcua::registers
