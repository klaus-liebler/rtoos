// SPDX-License-Identifier: MIT
//
// CANopen over EtherCAT (CoE, ETG.1000.6 5.6): the SDO server and the SDO information service over an
// object dictionary (object_dictionary.hpp). Pure: a request in (the CoE part of one mailbox, after the
// 6-byte mailbox header), the response out - the mailbox transport is the Slave's.
//
//   SDO upload / download   expedited (up to 4 bytes) and normal (one mailbox); segmented transfer and
//                           complete access are answered with an abort (the dictionary's largest
//                           value fits one mailbox)
//   SDO information         OD list, object description, entry description; a response longer than
//                           one mailbox goes out in fragments (next_fragment())
//   PDO assignment          1C12h/1C13h, writable in PRE-OP: subindex 0 to 0, the PDO indexes, then
//                           subindex 0 to their count (ETG.1000.6 5.6.7.4)
//   process data            the TxPDO image packed from the assigned PDOs' entries (pack_inputs()), an
//                           RxPDO image applied entry by entry - only what changed since the last
//                           image (apply_outputs()), zero_outputs() for the safe state
//
// Register values go through the application's rtos::modbus::RegisterMap: reads and writes exactly as
// Modbus and OPC UA do them, the write filter (e.g. a PLC that owns the outputs) and write hook included.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "rtos/ethercat/object_dictionary.hpp"
#include "rtos/modbus/modbus.hpp"

namespace rtos::ethercat {

/// CoE services (CoE header bits 15:12).
namespace coe_service {
inline constexpr std::uint8_t emergency = 1, sdo_request = 2, sdo_response = 3, sdo_info = 8;
}

/// SDO abort codes (ETG.1000.6 table 41).
namespace sdo_abort {
inline constexpr std::uint32_t toggle = 0x0503'0000, command = 0x0504'0001, unsupported_access = 0x0601'0000,
                               write_only = 0x0601'0001, read_only = 0x0601'0002, no_object = 0x0602'0000,
                               length_mismatch = 0x0607'0010, too_long = 0x0607'0012, too_short = 0x0607'0013,
                               no_subindex = 0x0609'0011, value_range = 0x0609'0030, general = 0x0800'0000,
                               not_stored = 0x0800'0020, local_control = 0x0800'0021, device_state = 0x0800'0022;
}

/// SDO information opcodes (ETG.1000.6 5.6.3).
namespace sdo_info {
inline constexpr std::uint8_t od_list_request = 1, od_list_response = 2, object_request = 3, object_response = 4,
                              entry_request = 5, entry_response = 6, error = 7, incomplete = 0x80;
}

struct CoeCounters {
    std::uint32_t uploads = 0, downloads = 0, aborts = 0, info_requests = 0;
    std::uint32_t output_writes = 0, output_refused = 0;  ///< RxPDO entries written into the map / refused by it
    std::uint32_t last_abort = 0;
    std::uint16_t last_abort_index = 0;
    std::uint8_t last_abort_subindex = 0;
};

/// The read/write access bit of `access` for an AL state (2 PRE-OP, 4 SAFE-OP, 8 OP).
[[nodiscard]] constexpr bool may_read(std::uint16_t a, std::uint8_t state) noexcept {
    return state == 2 ? (a & access::read_preop) != 0 : state == 4 ? (a & access::read_safeop) != 0 : state == 8 ? (a & access::read_op) != 0 : false;
}
[[nodiscard]] constexpr bool may_write(std::uint16_t a, std::uint8_t state) noexcept {
    return state == 2 ? (a & access::write_preop) != 0 : state == 4 ? (a & access::write_safeop) != 0 : state == 8 ? (a & access::write_op) != 0 : false;
}

template <std::size_t MaxPdos = 32, std::size_t InfoBytes = 1024>
class CoeServer {
public:
    CoeServer(std::span<const OdObject> od, modbus::RegisterMap& map) noexcept : od_(od), map_(map) { reset_assignment(); }

    /// Every PDO of the dictionary assigned, in index order (the default; also after INIT).
    void reset_assignment() noexcept {
        rx_count_ = tx_count_ = 0;
        for (const OdObject& o : od_) {
            if (is_rx_pdo(o.index) && rx_count_ < MaxPdos) rx_[rx_count_++] = o.index;
            if (is_tx_pdo(o.index) && tx_count_ < MaxPdos) tx_[tx_count_++] = o.index;
        }
        info_left_ = 0;
    }

    [[nodiscard]] std::span<const std::uint16_t> rx_assigned() const noexcept { return {rx_.data(), rx_count_}; }
    [[nodiscard]] std::span<const std::uint16_t> tx_assigned() const noexcept { return {tx_.data(), tx_count_}; }
    [[nodiscard]] const CoeCounters& counters() const noexcept { return counters_; }
    [[nodiscard]] std::span<const OdObject> dictionary() const noexcept { return od_; }

    [[nodiscard]] const OdObject* find(std::uint16_t index) const noexcept {
        const auto it = std::ranges::lower_bound(od_, index, {}, &OdObject::index);
        return it != od_.end() && it->index == index ? &*it : nullptr;
    }

    /// A CoE request; the response into `out` (the mailbox's data area). Returns its length, 0: none.
    std::size_t request(std::span<const std::byte> in, std::span<std::byte> out, std::uint8_t state) noexcept {
        if (in.size() < 2 || out.size() < 16) return 0;
        const std::uint8_t service = static_cast<std::uint8_t>(u8(in, 1) >> 4);
        if (service == coe_service::sdo_request) return sdo(in, out, state);
        if (service == coe_service::sdo_info) return info(in, out);
        return 0;  // emergency, PDO services: nothing to answer
    }

    /// The next fragment of an SDO information response, 0: none pending.
    std::size_t next_fragment(std::span<std::byte> out) noexcept { return info_left_ > 0 ? info_fragment(out) : 0; }
    [[nodiscard]] bool fragments_pending() const noexcept { return info_left_ > 0; }

    /// Bytes of the assigned PDOs (RxPDO: `tx` false): the mapping entries' bit lengths.
    [[nodiscard]] std::uint16_t process_data_bytes(bool tx) const noexcept {
        std::uint32_t bits = 0;
        for (const std::uint16_t index : tx ? tx_assigned() : rx_assigned()) {
            if (const OdObject* o = find(index)) {
                for (const OdEntry& e : o->entries) {
                    if (e.subindex != 0) bits += e.value & 0xFF;
                }
            }
        }
        return static_cast<std::uint16_t>((bits + 7) / 8);
    }

    // ---- process data --------------------------------------------------------------------------------
    /// The TxPDO image (the assigned PDOs' entries in order, little endian as over SDO) into `out`;
    /// its length.
    std::size_t pack_inputs(std::span<std::byte> out) noexcept {
        std::size_t pos = 0;
        for_each_mapped(tx_assigned(), [&](const OdEntry& e, std::size_t bytes) {
            std::array<std::byte, 64> v{};
            const std::size_t n = read_value(e, v);
            if (pos + bytes <= out.size()) std::copy_n(v.begin(), std::min(n, bytes), out.begin() + static_cast<std::ptrdiff_t>(pos));
            pos += bytes;
        });
        return std::min(pos, out.size());
    }

    /// An RxPDO image: every entry whose bytes differ from `last` (or all, `all`) written into the map
    /// - outputs that Modbus/OPC UA/web wrote meanwhile stay until the master changes its value.
    /// `last` becomes `in`.
    void apply_outputs(std::span<const std::byte> in, std::span<std::byte> last, bool all) noexcept {
        std::size_t pos = 0;
        for_each_mapped(rx_assigned(), [&](const OdEntry& e, std::size_t bytes) {
            if (pos + bytes > in.size() || pos + bytes > last.size()) return;
            const auto now = in.subspan(pos, bytes);
            if (all || !std::ranges::equal(now, last.subspan(pos, bytes))) {
                ++(write_value(e, now) == 0 ? counters_.output_writes : counters_.output_refused);
            }
            pos += bytes;
        });
        std::copy_n(in.begin(), std::min(in.size(), last.size()), last.begin());
    }

    /// The safe state: every entry of the assigned RxPDOs to 0.
    void zero_outputs() noexcept {
        const std::array<std::byte, 64> zero{};
        for_each_mapped(rx_assigned(), [&](const OdEntry& e, std::size_t bytes) {
            if (bytes <= zero.size()) (void)write_value(e, std::span(zero).first(bytes));
        });
    }

private:
    /// Every mapping entry of `pdos` with its object entry and byte length (byte-aligned mappings only:
    /// the dictionary's entries are whole registers).
    template <class F>
    void for_each_mapped(std::span<const std::uint16_t> pdos, F&& f) noexcept {
        for (const std::uint16_t index : pdos) {
            const OdObject* pdo = find(index);
            if (pdo == nullptr) continue;
            for (const OdEntry& m : pdo->entries) {
                if (m.subindex == 0) continue;
                const OdObject* o = find(static_cast<std::uint16_t>(m.value >> 16));
                const OdEntry* e = o != nullptr ? o->entry(static_cast<std::uint8_t>(m.value >> 8)) : nullptr;
                const std::size_t bytes = (m.value & 0xFF) / 8;
                if (e != nullptr) f(*e, bytes);
            }
        }
    }

    static constexpr bool is_rx_pdo(std::uint16_t i) noexcept { return i >= 0x1600 && i < 0x1800; }
    static constexpr bool is_tx_pdo(std::uint16_t i) noexcept { return i >= 0x1A00 && i < 0x1C00; }

    // ---- bytes ---------------------------------------------------------------------------------
    static std::uint8_t u8(std::span<const std::byte> b, std::size_t i) noexcept { return std::to_integer<std::uint8_t>(b[i]); }
    static std::uint16_t u16(std::span<const std::byte> b, std::size_t i) noexcept {
        return static_cast<std::uint16_t>(u8(b, i) | (u8(b, i + 1) << 8));
    }
    static std::uint32_t u32(std::span<const std::byte> b, std::size_t i) noexcept {
        return u16(b, i) | (std::uint32_t{u16(b, i + 2)} << 16);
    }
    static void put8(std::span<std::byte> b, std::size_t i, std::uint32_t v) noexcept { b[i] = std::byte(v & 0xFF); }
    static void put16(std::span<std::byte> b, std::size_t i, std::uint32_t v) noexcept { put8(b, i, v), put8(b, i + 1, v >> 8); }
    static void put32(std::span<std::byte> b, std::size_t i, std::uint32_t v) noexcept { put16(b, i, v), put16(b, i + 2, v >> 16); }
    static void coe_header(std::span<std::byte> out, std::uint8_t service) noexcept { put16(out, 0, std::uint32_t{service} << 12); }

    // ---- SDO --------------------------------------------------------------------------------------
    std::size_t sdo(std::span<const std::byte> in, std::span<std::byte> out, std::uint8_t state) noexcept {
        if (in.size() < 10) return 0;  // shorter than an SDO header: no answer possible (no index)
        const std::uint8_t cmd = u8(in, 2);
        const std::uint16_t index = u16(in, 3);
        const std::uint8_t sub = u8(in, 5);
        switch (cmd >> 5) {
            case 1: return download(in, out, state, cmd, index, sub);
            case 2: return upload(out, state, cmd, index, sub);
            case 4: return 0;  // the master aborts: nothing to answer
            default: return abort(out, index, sub, sdo_abort::command);  // segments: not offered
        }
    }

    std::size_t abort(std::span<std::byte> out, std::uint16_t index, std::uint8_t sub, std::uint32_t code) noexcept {
        ++counters_.aborts;
        counters_.last_abort = code, counters_.last_abort_index = index, counters_.last_abort_subindex = sub;
        coe_header(out, coe_service::sdo_request);  // abort transfer: SDO request service (ETG.1000.6 5.6.2.7)
        put8(out, 2, 0x80);
        put16(out, 3, index);
        put8(out, 5, sub);
        put32(out, 6, code);
        return 10;
    }

    /// The assignment objects' entries 1..n (the dictionary holds only subindex 0).
    struct AssignRef {
        std::array<std::uint16_t, MaxPdos>* list;
        std::size_t* count;
        bool tx;
    };
    std::optional<AssignRef> assignment(std::uint16_t index) noexcept {
        if (index == 0x1C12) return AssignRef{&rx_, &rx_count_, false};
        if (index == 0x1C13) return AssignRef{&tx_, &tx_count_, true};
        return std::nullopt;
    }

    std::size_t upload(std::span<std::byte> out, std::uint8_t state, std::uint8_t cmd, std::uint16_t index, std::uint8_t sub) noexcept {
        if ((cmd & 0x10) != 0) return abort(out, index, sub, sdo_abort::unsupported_access);  // complete access
        const OdObject* o = find(index);
        if (o == nullptr) return abort(out, index, sub, sdo_abort::no_object);
        std::array<std::byte, 128> value{};
        std::size_t n = 0;
        if (const auto a = assignment(index); a && sub > 0) {
            if (sub > MaxPdos) return abort(out, index, sub, sdo_abort::no_subindex);
            put16(value, 0, sub <= *a->count ? (*a->list)[sub - 1] : 0);
            n = 2;
        } else {
            const OdEntry* e = o->entry(sub);
            if (e == nullptr) return abort(out, index, sub, sdo_abort::no_subindex);
            if (!may_read(e->access, state)) return abort(out, index, sub, (e->access & access::read) == 0 ? sdo_abort::write_only : sdo_abort::device_state);
            n = read_value(*e, value);
            if (n == 0) return abort(out, index, sub, sdo_abort::general);
        }
        ++counters_.uploads;
        coe_header(out, coe_service::sdo_response);
        put16(out, 3, index);
        put8(out, 5, sub);
        if (n <= 4) {
            put8(out, 2, 0x43 | ((4 - n) << 2));  // expedited, size indicated
            put32(out, 6, 0);
            std::copy_n(value.begin(), n, out.begin() + 6);
            return 10;
        }
        if (10 + n > out.size()) return abort(out, index, sub, sdo_abort::general);  // segmented: not offered
        put8(out, 2, 0x41);  // normal, size indicated
        put32(out, 6, static_cast<std::uint32_t>(n));
        std::copy_n(value.begin(), n, out.begin() + 10);
        return 10 + n;
    }

    /// The entry's value, little endian; its length (0: could not be read).
    std::size_t read_value(const OdEntry& e, std::span<std::byte> v) noexcept {
        switch (e.source) {
            case Source::constant: {
                const std::size_t n = std::max<std::size_t>(1, e.bits / 8);
                for (std::size_t i = 0; i < n && i < 4; ++i) put8(v, i, e.value >> (8 * i));
                return std::min<std::size_t>(n, 4);
            }
            case Source::string: {
                const std::size_t n = std::min(e.text.size(), v.size());
                for (std::size_t i = 0; i < n; ++i) v[i] = std::byte(static_cast<unsigned char>(e.text[i]));
                return n;
            }
            case Source::input:
            case Source::holding: {
                const std::size_t words = e.bits / 16;
                std::array<std::uint16_t, 32> w{};
                if (words == 0 || words > w.size() || 2 * words > v.size()) return 0;
                const modbus::Table table = e.source == Source::input ? modbus::Table::Input : modbus::Table::Holding;
                if (std::uint32_t{e.address} + words > map_.size(table)) return 0;
                if (map_.read(table, e.address, std::span(w).first(words)) != modbus::Exception::None) return 0;
                if (words == 2 && e.data_type != dtype::octet_string) {  // one 32-bit value: high word first in the map
                    put32(v, 0, (std::uint32_t{w[0]} << 16) | w[1]);
                } else {
                    for (std::size_t i = 0; i < words; ++i) put16(v, 2 * i, w[i]);
                }
                return 2 * words;
            }
            case Source::rx_assign: put8(v, 0, rx_count_); return 1;
            case Source::tx_assign: put8(v, 0, tx_count_); return 1;
        }
        return 0;
    }

    std::size_t download(std::span<const std::byte> in, std::span<std::byte> out, std::uint8_t state, std::uint8_t cmd,
                         std::uint16_t index, std::uint8_t sub) noexcept {
        if ((cmd & 0x10) != 0) return abort(out, index, sub, sdo_abort::unsupported_access);  // complete access
        std::span<const std::byte> data;
        std::size_t size = 0;
        bool size_known = (cmd & 0x01) != 0;
        if ((cmd & 0x02) != 0) {  // expedited
            size = size_known ? 4 - ((cmd >> 2) & 3) : 4;
            data = in.subspan(6, 4);
        } else {  // normal: the complete size, then the data in this mailbox
            if (!size_known) return abort(out, index, sub, sdo_abort::command);
            size = u32(in, 6);
            if (size > in.size() - 10) return abort(out, index, sub, sdo_abort::unsupported_access);  // segments follow: not offered
            data = in.subspan(10, size);
            size_known = true;
        }
        const OdObject* o = find(index);
        if (o == nullptr) return abort(out, index, sub, sdo_abort::no_object);
        if (const auto a = assignment(index)) {
            if (const std::uint32_t code = write_assignment(*a, sub, data, size, state); code != 0) return abort(out, index, sub, code);
        } else {
            const OdEntry* e = o->entry(sub);
            if (e == nullptr) return abort(out, index, sub, sdo_abort::no_subindex);
            if (!may_write(e->access, state)) {
                return abort(out, index, sub, (e->access & access::write) == 0 ? sdo_abort::read_only : sdo_abort::device_state);
            }
            const std::size_t need = e->bits / 8;
            if (size_known && size > need) return abort(out, index, sub, sdo_abort::too_long);
            if (size_known && size < need) return abort(out, index, sub, sdo_abort::too_short);
            if (const std::uint32_t code = write_value(*e, data.first(std::min(need, data.size()))); code != 0) {
                return abort(out, index, sub, code);
            }
        }
        ++counters_.downloads;
        coe_header(out, coe_service::sdo_response);
        put8(out, 2, 0x60);
        put16(out, 3, index);
        put8(out, 5, sub);
        put32(out, 6, 0);
        return 10;
    }

    std::uint32_t write_value(const OdEntry& e, std::span<const std::byte> d) noexcept {
        if (e.source != Source::holding) return sdo_abort::read_only;
        const std::size_t words = e.bits / 16;
        std::array<std::uint16_t, 32> w{};
        if (words == 0 || words > w.size() || d.size() < 2 * words) return sdo_abort::too_short;
        if (words == 2 && e.data_type != dtype::octet_string) {
            const std::uint32_t v = u32(d, 0);
            w[0] = static_cast<std::uint16_t>(v >> 16), w[1] = static_cast<std::uint16_t>(v);
        } else {
            for (std::size_t i = 0; i < words; ++i) w[i] = u16(d, 2 * i);
        }
        if (std::uint32_t{e.address} + words > map_.size(modbus::Table::Holding)) return sdo_abort::general;
        switch (map_.write(e.address, std::span<const std::uint16_t>(w.data(), words))) {
            case modbus::Exception::None: return 0;
            case modbus::Exception::IllegalDataAddress: return sdo_abort::local_control;  // the write filter (a PLC owns it)
            case modbus::Exception::IllegalDataValue: return sdo_abort::value_range;
            default: return sdo_abort::not_stored;
        }
    }

    std::uint32_t write_assignment(const AssignRef& a, std::uint8_t sub, std::span<const std::byte> d, std::size_t size,
                                   std::uint8_t state) noexcept {
        if (state != 2) return sdo_abort::device_state;  // PRE-OP only
        if (sub == 0) {
            if (size < 1) return sdo_abort::too_short;
            const std::uint8_t n = u8(d, 0);
            if (n > MaxPdos) return sdo_abort::value_range;
            for (std::size_t i = 0; i < n; ++i) {
                if ((*a.list)[i] == 0) return sdo_abort::value_range;  // every entry set before the count
            }
            // a staged entry beyond n is dropped: the assignment is exactly 1..n
            for (std::size_t i = n; i < MaxPdos; ++i) (*a.list)[i] = 0;
            *a.count = n;
            return 0;
        }
        if (sub > MaxPdos) return sdo_abort::no_subindex;
        if (*a.count != 0) return sdo_abort::device_state;  // subindex 0 must be 0 while entries change
        if (size < 2) return sdo_abort::too_short;
        const std::uint16_t pdo = u16(d, 0);
        const bool right_kind = a.tx ? is_tx_pdo(pdo) : is_rx_pdo(pdo);
        if (!right_kind || find(pdo) == nullptr) return sdo_abort::value_range;
        (*a.list)[sub - 1] = pdo;
        return 0;
    }

    // ---- SDO information -----------------------------------------------------------------------------
    std::size_t info(std::span<const std::byte> in, std::span<std::byte> out) noexcept {
        if (in.size() < 6) return 0;
        ++counters_.info_requests;
        info_left_ = 0;
        const std::uint8_t opcode = u8(in, 2) & 0x7F;
        std::size_t n = 0;  // bytes of the response's data (after the info header)
        std::span<std::byte> d(info_buf_);
        if (opcode == sdo_info::od_list_request && in.size() >= 8) {
            const std::uint16_t list = u16(in, 6);
            put16(d, 0, list);
            n = 2;
            if (list == 0) {  // the number of objects per list: all, RxPDO-, TxPDO-mappable, backup, settings
                put16(d, 2, static_cast<std::uint32_t>(od_.size()));
                put16(d, 4, count_mappable(access::rxpdo));
                put16(d, 6, count_mappable(access::txpdo));
                put16(d, 8, 0), put16(d, 10, 0);
                n = 12;
            } else {
                for (const OdObject& o : od_) {
                    const bool listed = list == 1 || (list == 2 && mappable(o, access::rxpdo)) || (list == 3 && mappable(o, access::txpdo));
                    if (listed && n + 2 <= d.size()) put16(d, n, o.index), n += 2;
                }
            }
            info_opcode_ = sdo_info::od_list_response;
        } else if (opcode == sdo_info::object_request && in.size() >= 8) {
            const std::uint16_t index = u16(in, 6);
            const OdObject* o = find(index);
            if (o == nullptr) return info_error(out, sdo_abort::no_object);
            put16(d, 0, index);
            put16(d, 2, o->data_type());
            put8(d, 4, std::max<std::size_t>(o->max_subindex(), assignment_size(index)));
            put8(d, 5, o->code);
            n = 6 + copy_text(d.subspan(6), o->name);
            info_opcode_ = sdo_info::object_response;
        } else if (opcode == sdo_info::entry_request && in.size() >= 10) {
            const std::uint16_t index = u16(in, 6);
            const std::uint8_t sub = u8(in, 8);
            const OdObject* o = find(index);
            if (o == nullptr) return info_error(out, sdo_abort::no_object);
            OdEntry e{};
            if (const OdEntry* p = o->entry(sub)) {
                e = *p;
            } else if (sub > 0 && sub <= assignment_size(index)) {
                e = {.subindex = sub, .data_type = dtype::uint16, .bits = 16, .access = access::read | access::write_preop, .name = "SubIndex"};
            } else {
                return info_error(out, sdo_abort::no_subindex);
            }
            put16(d, 0, index);
            put8(d, 2, sub);
            put8(d, 3, 0);  // value info: no unit, default, minimum, maximum
            put16(d, 4, e.data_type);
            put16(d, 6, e.bits);
            put16(d, 8, e.access);
            n = 10 + copy_text(d.subspan(10), e.name);
            info_opcode_ = sdo_info::entry_response;
        } else {
            return info_error(out, sdo_abort::command);
        }
        info_size_ = n, info_sent_ = 0;
        info_left_ = 1;  // set for info_fragment(), which counts the rest
        return info_fragment(out);
    }

    std::size_t assignment_size(std::uint16_t index) const noexcept { return index == 0x1C12 || index == 0x1C13 ? MaxPdos : 0; }

    std::size_t copy_text(std::span<std::byte> d, std::string_view s) const noexcept {
        const std::size_t n = std::min(s.size(), d.size());
        for (std::size_t i = 0; i < n; ++i) d[i] = std::byte(static_cast<unsigned char>(s[i]));
        return n;
    }

    static bool mappable(const OdObject& o, std::uint16_t bit) noexcept {
        return std::ranges::any_of(o.entries, [bit](const OdEntry& e) { return (e.access & bit) != 0; });
    }
    std::uint16_t count_mappable(std::uint16_t bit) const noexcept {
        return static_cast<std::uint16_t>(std::ranges::count_if(od_, [bit](const OdObject& o) { return mappable(o, bit); }));
    }

    std::size_t info_error(std::span<std::byte> out, std::uint32_t code) noexcept {
        coe_header(out, coe_service::sdo_info);
        put8(out, 2, sdo_info::error);
        put8(out, 3, 0);
        put16(out, 4, 0);
        put32(out, 6, code);
        info_left_ = 0;
        return 10;
    }

    /// One fragment: CoE header, info header (opcode, the incomplete flag, fragments left), data.
    std::size_t info_fragment(std::span<std::byte> out) noexcept {
        const std::size_t chunk = out.size() - 6;
        const std::size_t rest = info_size_ - info_sent_;
        const std::size_t n = std::min(chunk, rest);
        const std::size_t after = rest - n;
        const std::size_t fragments_after = (after + chunk - 1) / chunk;
        coe_header(out, coe_service::sdo_info);
        put8(out, 2, info_opcode_ | (fragments_after > 0 ? sdo_info::incomplete : 0));
        put8(out, 3, 0);
        put16(out, 4, static_cast<std::uint32_t>(fragments_after));
        std::copy_n(info_buf_.begin() + static_cast<std::ptrdiff_t>(info_sent_), n, out.begin() + 6);
        info_sent_ += n;
        info_left_ = fragments_after;
        return 6 + n;
    }

    std::span<const OdObject> od_;
    modbus::RegisterMap& map_;
    std::array<std::uint16_t, MaxPdos> rx_{}, tx_{};
    std::size_t rx_count_ = 0, tx_count_ = 0;
    CoeCounters counters_{};
    std::array<std::byte, InfoBytes> info_buf_{};
    std::size_t info_size_ = 0, info_sent_ = 0, info_left_ = 0;
    std::uint8_t info_opcode_ = 0;
};

}  // namespace rtos::ethercat
