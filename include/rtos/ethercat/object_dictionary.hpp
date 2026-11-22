// SPDX-License-Identifier: MIT
//
// The CoE object dictionary (ETG.1000.6 5.6, CiA 301 layout) as constexpr tables: objects with their
// entries, each entry a constant, a string, words of the application's register map
// (rtos::modbus::RegisterMap, the same values as Modbus and OPC UA) or one of the slave's own
// variables (the PDO assignment). No heap, no virtual objects: the tables are generated from the
// register schema (universal_register_access) and joined with standard_objects().
//
// Index ranges (ETG.1000.6 table 69 / ETG.5001 modular device profile layout):
//   1000h-1FFFh communication: device type, name, versions, identity, PDO mapping 1600h/1A00h,
//               SyncManager types 1C00h, PDO assignment 1C12h (RxPDO) / 1C13h (TxPDO)
//   2000h-5FFFh manufacturer specific (commands: SDO only)
//   6000h-6FFFh inputs, 7000h-7FFFh outputs, 8000h-8FFFh configuration
//
// Values on the wire are little endian; a 32-bit register value (two registers, high word first in
// the map) is one UDINT/DINT; an octet string of n registers is the registers in order, each word
// little endian.
#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace rtos::ethercat {

/// CoE basic data types (ETG.1000.6 table 64).
namespace dtype {
inline constexpr std::uint16_t boolean = 0x0001, int8 = 0x0002, int16 = 0x0003, int32 = 0x0004, uint8 = 0x0005,
                               uint16 = 0x0006, uint32 = 0x0007, visible_string = 0x0009, octet_string = 0x000A;
}

/// Object access (SDO info entry description, ETG.1000.6 table 47).
namespace access {
inline constexpr std::uint16_t read_preop = 0x01, read_safeop = 0x02, read_op = 0x04, write_preop = 0x08,
                               write_safeop = 0x10, write_op = 0x20, rxpdo = 0x40, txpdo = 0x80;
inline constexpr std::uint16_t read = read_preop | read_safeop | read_op, write = write_preop | write_safeop | write_op;
inline constexpr std::uint16_t read_write = read | write;
}  // namespace access

/// Object codes.
inline constexpr std::uint8_t object_var = 7, object_array = 8, object_record = 9;

enum class Source : std::uint8_t {
    constant,    ///< `value` (up to 32 bits)
    string,      ///< `text` (VISIBLE_STRING)
    input,       ///< input registers from `address` (`bits` / 16 of them)
    holding,     ///< holding registers from `address`
    rx_assign,   ///< 1C12h: the slave's RxPDO assignment (subindex 0: the count)
    tx_assign,   ///< 1C13h: the TxPDO assignment
};

struct OdEntry {
    std::uint8_t subindex = 0;
    std::uint16_t data_type = dtype::uint16;
    std::uint16_t bits = 16;
    std::uint16_t access = access::read;
    Source source = Source::constant;
    std::uint16_t address = 0;  ///< register (input/holding)
    std::uint32_t value = 0;    ///< constant
    std::string_view text{};    ///< string
    std::string_view name{};
    bool is_signed = false;     ///< a register value (INT16/INT32)
};

struct OdObject {
    std::uint16_t index = 0;
    std::uint8_t code = object_var;
    std::string_view name{};
    std::span<const OdEntry> entries{};  ///< by subindex, subindex 0 first (a VAR: the one entry, subindex 0)

    [[nodiscard]] constexpr const OdEntry* entry(std::uint8_t subindex) const noexcept {
        for (const OdEntry& e : entries) {
            if (e.subindex == subindex) return &e;
        }
        return nullptr;
    }
    /// The object's data type for the object description: a VAR's entry type, else the first
    /// value entry's (a RECORD of mixed types reports 0, "DEFTYPE unknown").
    [[nodiscard]] constexpr std::uint16_t data_type() const noexcept {
        if (entries.empty()) return 0;
        if (code == object_var) return entries[0].data_type;
        return entries.size() > 1 ? entries[1].data_type : 0;
    }
    [[nodiscard]] constexpr std::uint8_t max_subindex() const noexcept {
        std::uint8_t m = 0;
        for (const OdEntry& e : entries) m = std::max(m, e.subindex);
        return m;
    }
};

/// A PDO mapping value: index, subindex, bit length (CiA 301).
[[nodiscard]] constexpr std::uint32_t pdo_mapping(std::uint16_t index, std::uint8_t subindex, std::uint8_t bits) noexcept {
    return (std::uint32_t{index} << 16) | (std::uint32_t{subindex} << 8) | bits;
}

/// Subindex 0 of a RECORD/ARRAY: the highest subindex, UNSIGNED8, read-only.
[[nodiscard]] constexpr OdEntry count_entry(std::uint8_t n) noexcept {
    return {.subindex = 0, .data_type = dtype::uint8, .bits = 8, .access = access::read, .value = n, .name = "SubIndex 000"};
}

/// The device's identity and descriptive strings (1000h-1018h).
struct DeviceInfo {
    std::uint32_t device_type = 0;  ///< 1000h: 0 = no CiA profile
    std::string_view name, hardware_version, software_version;
    std::uint32_t vendor_id = 0, product_code = 0, revision = 0, serial = 0;
};

/// The standard objects' entries (storage for standard_objects(); one per slave).
struct StandardEntries {
    std::array<OdEntry, 1> device_type{}, name{}, hardware_version{}, software_version{};
    std::array<OdEntry, 5> identity{};
    std::array<OdEntry, 5> sm_types{};
    std::array<OdEntry, 1> rx_assign{}, tx_assign{};  ///< subindex 0 only: the slave serves 1..n itself
};

[[nodiscard]] constexpr StandardEntries standard_entries(const DeviceInfo& d) noexcept {
    StandardEntries s{};
    s.device_type[0] = {.data_type = dtype::uint32, .bits = 32, .value = d.device_type, .name = "Device type"};
    s.name[0] = {.data_type = dtype::visible_string, .bits = static_cast<std::uint16_t>(8 * d.name.size()),
                 .source = Source::string, .text = d.name, .name = "Device name"};
    s.hardware_version[0] = {.data_type = dtype::visible_string, .bits = static_cast<std::uint16_t>(8 * d.hardware_version.size()),
                             .source = Source::string, .text = d.hardware_version, .name = "Hardware version"};
    s.software_version[0] = {.data_type = dtype::visible_string, .bits = static_cast<std::uint16_t>(8 * d.software_version.size()),
                             .source = Source::string, .text = d.software_version, .name = "Software version"};
    s.identity = {count_entry(4),
                  OdEntry{.subindex = 1, .data_type = dtype::uint32, .bits = 32, .value = d.vendor_id, .name = "Vendor ID"},
                  OdEntry{.subindex = 2, .data_type = dtype::uint32, .bits = 32, .value = d.product_code, .name = "Product code"},
                  OdEntry{.subindex = 3, .data_type = dtype::uint32, .bits = 32, .value = d.revision, .name = "Revision"},
                  OdEntry{.subindex = 4, .data_type = dtype::uint32, .bits = 32, .value = d.serial, .name = "Serial number"}};
    // SyncManager communication types: mailbox out, mailbox in, outputs, inputs
    s.sm_types = {count_entry(4),
                  OdEntry{.subindex = 1, .data_type = dtype::uint8, .bits = 8, .value = 1, .name = "SubIndex 001"},
                  OdEntry{.subindex = 2, .data_type = dtype::uint8, .bits = 8, .value = 2, .name = "SubIndex 002"},
                  OdEntry{.subindex = 3, .data_type = dtype::uint8, .bits = 8, .value = 3, .name = "SubIndex 003"},
                  OdEntry{.subindex = 4, .data_type = dtype::uint8, .bits = 8, .value = 4, .name = "SubIndex 004"}};
    s.rx_assign[0] = {.data_type = dtype::uint8, .bits = 8, .access = access::read | access::write_preop,
                      .source = Source::rx_assign, .name = "SubIndex 000"};
    s.tx_assign[0] = {.data_type = dtype::uint8, .bits = 8, .access = access::read | access::write_preop,
                      .source = Source::tx_assign, .name = "SubIndex 000"};
    return s;
}

/// The standard objects over `e` (which must outlive them: a constexpr variable).
[[nodiscard]] constexpr std::array<OdObject, 8> standard_objects(const StandardEntries& e) noexcept {
    return {OdObject{.index = 0x1000, .code = object_var, .name = "Device type", .entries = e.device_type},
            OdObject{.index = 0x1008, .code = object_var, .name = "Device name", .entries = e.name},
            OdObject{.index = 0x1009, .code = object_var, .name = "Hardware version", .entries = e.hardware_version},
            OdObject{.index = 0x100A, .code = object_var, .name = "Software version", .entries = e.software_version},
            OdObject{.index = 0x1018, .code = object_record, .name = "Identity", .entries = e.identity},
            OdObject{.index = 0x1C00, .code = object_array, .name = "Sync manager type", .entries = e.sm_types},
            OdObject{.index = 0x1C12, .code = object_array, .name = "RxPDO assign", .entries = e.rx_assign},
            OdObject{.index = 0x1C13, .code = object_array, .name = "TxPDO assign", .entries = e.tx_assign}};
}

/// Two object lists into one, sorted by index (the OD list and the lookup need the order).
template <std::size_t A, std::size_t B>
[[nodiscard]] constexpr std::array<OdObject, A + B> join_objects(const std::array<OdObject, A>& a, const std::array<OdObject, B>& b) noexcept {
    std::array<OdObject, A + B> r{};
    std::ranges::copy(a, r.begin());
    std::ranges::copy(b, r.begin() + A);
    std::ranges::sort(r, {}, &OdObject::index);
    return r;
}

/// The dictionary's invariants (static_assert at the application): indexes unique and sorted,
/// entries sorted by subindex, a RECORD/ARRAY's subindex 0 the count of the others (the assignment
/// objects excepted: the slave serves their entries).
template <std::size_t N>
[[nodiscard]] constexpr bool valid_dictionary(const std::array<OdObject, N>& od) noexcept {
    for (std::size_t i = 1; i < N; ++i) {
        if (od[i - 1].index >= od[i].index) return false;
    }
    for (const OdObject& o : od) {
        if (o.entries.empty()) return false;
        for (std::size_t i = 1; i < o.entries.size(); ++i) {
            if (o.entries[i - 1].subindex >= o.entries[i].subindex) return false;
        }
        const bool assign = o.entries[0].source == Source::rx_assign || o.entries[0].source == Source::tx_assign;
        if (o.code != object_var && !assign && o.entries[0].value != o.max_subindex()) return false;
        for (const OdEntry& e : o.entries) {
            const bool reg = e.source == Source::input || e.source == Source::holding;
            if (reg && (e.bits % 16 != 0 || e.bits == 0)) return false;
            if (e.source == Source::input && (e.access & access::write) != 0) return false;
        }
    }
    return true;
}

}  // namespace rtos::ethercat
