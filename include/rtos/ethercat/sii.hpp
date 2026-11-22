// SPDX-License-Identifier: MIT
//
// The slave information interface (SII, ETG.2010): the EEPROM content a master reads first - the
// ESC configuration area, identity, mailbox layout and the categories (strings, general, FMMU,
// SyncManager). Built at compile time from an SiiConfig; a board without EEPROM answers the ESC's
// EEPROM commands from this image (EEPROM emulation, Slave).
//
//   constexpr auto image = rtos::ethercat::make_sii<256>({.pdi_control = 0x82, .vendor_id = ..., ...});
//
// Process data is described over CoE (PDO mapping and assignment objects, Slave), not in SII
// categories: a master with CoE reads it from the object dictionary (ETG.1000.6, ETG.2010 says the
// TXPDO/RXPDO categories are optional then).
#pragma once

#include <array>
#include <contracts>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>

namespace rtos::ethercat {

/// The ESC configuration area (words 0..7) and the mailbox protocols (word 1Ch).
namespace sii {
inline constexpr std::uint16_t mailbox_aoe = 0x01, mailbox_eoe = 0x02, mailbox_coe = 0x04, mailbox_foe = 0x08;
/// General category, CoE details: SDO, SDO info, PDO assignment, PDO configuration, upload at
/// start, SDO complete access.
inline constexpr std::uint8_t coe_sdo = 0x01, coe_sdo_info = 0x02, coe_pdo_assign = 0x04, coe_pdo_config = 0x08,
                              coe_upload_at_start = 0x10, coe_complete_access = 0x20;
/// SyncManager category types.
inline constexpr std::uint8_t sm_unused = 0, sm_mailbox_out = 1, sm_mailbox_in = 2, sm_outputs = 3, sm_inputs = 4;
/// FMMU category usage.
inline constexpr std::uint8_t fmmu_unused = 0, fmmu_outputs = 1, fmmu_inputs = 2, fmmu_sm_status = 3;
/// Category types.
inline constexpr std::uint16_t cat_strings = 10, cat_general = 30, cat_fmmu = 40, cat_syncm = 41, cat_end = 0xFFFF;
}  // namespace sii

struct SiiSyncManager {
    std::uint16_t start = 0, length = 0;
    std::uint8_t control = 0;
    bool enable = false;
    std::uint8_t type = sii::sm_unused;
};

struct SiiConfig {
    std::uint8_t pdi_control = 0;  ///< ESC register 0140h (the PDI the straps select: it must not change)
    std::uint8_t esc_config = 0;   ///< 0141h: 0 = the PDI acknowledges the AL states
    std::uint32_t vendor_id = 0, product_code = 0, revision = 0, serial = 0;
    /// Standard mailbox (SM0 master -> slave, SM1 slave -> master); size 0: no mailbox.
    std::uint16_t mailbox_out_start = 0, mailbox_out_size = 0, mailbox_in_start = 0, mailbox_in_size = 0;
    std::uint16_t mailbox_protocols = 0;
    std::uint8_t coe_details = 0;
    std::string_view name, group;  ///< strings 1 and 2
    std::array<std::uint8_t, 3> fmmus{};
    std::array<SiiSyncManager, 4> sync_managers{};
    std::uint16_t physical_ports = 0x0011;  ///< nibble per port: 1 = MII (ports 0 and 1)
};

/// CRC-8 of the ESC configuration area (words 0..6) for word 7: polynomial x^8+x^2+x+1, initial FFh.
template <std::size_t N>
[[nodiscard]] constexpr std::uint8_t sii_config_crc(const std::array<std::uint16_t, N>& w) noexcept {
    std::uint8_t crc = 0xFF;
    for (std::size_t i = 0; i < 14; ++i) {
        crc ^= static_cast<std::uint8_t>(w[i / 2] >> (8 * (i % 2)));
        for (int b = 0; b < 8; ++b) crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
    return crc;
}

/// The image of `Words` 16-bit words (a power of two of at least 64: 1 Kbit; word 3Eh says the size).
template <std::size_t Words>
[[nodiscard]] constexpr std::array<std::uint16_t, Words> make_sii(const SiiConfig& c) noexcept {
    static_assert(Words >= 64 && (Words & (Words - 1)) == 0, "SII size: a power of two, at least 1 Kbit");
    contract_assert(c.name.size() + c.group.size() + 3 <= 128 && c.name.size() < 256 && c.group.size() < 256);
    std::array<std::uint16_t, Words> w{};
    for (auto& x : w) x = 0;
    const auto lo = [](std::uint32_t v) { return static_cast<std::uint16_t>(v); };
    const auto hi = [](std::uint32_t v) { return static_cast<std::uint16_t>(v >> 16); };
    // configuration area: PDI control and ESC configuration; PDI configuration, SYNC impulse length,
    // extended PDI configuration, station alias: 0 (the defaults)
    w[0] = static_cast<std::uint16_t>(c.pdi_control | (c.esc_config << 8));
    w[7] = sii_config_crc(w);
    w[0x08] = lo(c.vendor_id), w[0x09] = hi(c.vendor_id);
    w[0x0A] = lo(c.product_code), w[0x0B] = hi(c.product_code);
    w[0x0C] = lo(c.revision), w[0x0D] = hi(c.revision);
    w[0x0E] = lo(c.serial), w[0x0F] = hi(c.serial);
    w[0x18] = c.mailbox_out_start, w[0x19] = c.mailbox_out_size;
    w[0x1A] = c.mailbox_in_start, w[0x1B] = c.mailbox_in_size;
    w[0x1C] = c.mailbox_protocols;
    w[0x3E] = static_cast<std::uint16_t>(Words * 16 / 1024 - 1);  // size in Kbit - 1
    w[0x3F] = 1;                                                  // SII version

    std::size_t p = 0x40;
    // strings: count, then each as length + characters, padded to whole words
    std::array<std::uint8_t, 2 * 64> bytes{};
    std::size_t n = 0;
    const std::array<std::string_view, 2> strings{c.name, c.group};
    bytes[n++] = static_cast<std::uint8_t>(strings.size());
    for (const std::string_view s : strings) {
        bytes[n++] = static_cast<std::uint8_t>(s.size());
        for (const char ch : s) bytes[n++] = static_cast<std::uint8_t>(ch);
    }
    const std::size_t string_words = (n + 1) / 2;
    w[p++] = sii::cat_strings;
    w[p++] = static_cast<std::uint16_t>(string_words);
    for (std::size_t i = 0; i < string_words; ++i) w[p++] = static_cast<std::uint16_t>(bytes[2 * i] | (bytes[2 * i + 1] << 8));
    // general (32 bytes): group string 2, image -, order string 1, name string 1, CoE details,
    // physical ports
    w[p++] = sii::cat_general;
    w[p++] = 16;
    w[p + 0] = static_cast<std::uint16_t>(2 | (0 << 8));                        // group index, image index
    w[p + 1] = static_cast<std::uint16_t>(1 | (1 << 8));                        // order index, name index
    w[p + 2] = static_cast<std::uint16_t>(0 | (c.coe_details << 8));            // reserved, CoE details
    w[p + 8] = c.physical_ports;                                                // byte 16: physical port
    p += 16;
    // FMMUs: one byte each, padded
    w[p++] = sii::cat_fmmu;
    w[p++] = 2;
    w[p++] = static_cast<std::uint16_t>(c.fmmus[0] | (c.fmmus[1] << 8));
    w[p++] = static_cast<std::uint16_t>(c.fmmus[2] | (0 << 8));
    // SyncManagers: 8 bytes each (start, length, control, status, enable, type)
    w[p++] = sii::cat_syncm;
    w[p++] = static_cast<std::uint16_t>(4 * c.sync_managers.size());
    for (const SiiSyncManager& s : c.sync_managers) {
        w[p++] = s.start;
        w[p++] = s.length;
        w[p++] = s.control;  // status byte 0
        w[p++] = static_cast<std::uint16_t>((s.enable ? 1 : 0) | (s.type << 8));
    }
    contract_assert(p < Words);  // the categories fit the image
    w[p] = sii::cat_end;
    return w;
}

}  // namespace rtos::ethercat
