// SPDX-License-Identifier: MIT
//
// The EtherCAT test slave: SII, layout, a small object dictionary over 8 + 8 registers and an
// application that records the AL events. ethercat_test.cpp and the AFL++ harness
// tests/fuzz/ethercat.cpp run ec::Slave<SimEsc, App, 256> over it.
#pragma once

#include <array>
#include <cstdint>
#include <utility>
#include <vector>

#include "ethercat_sim.hpp"
#include "rtos/ethercat/coe.hpp"
#include "rtos/ethercat/object_dictionary.hpp"
#include "rtos/ethercat/sii.hpp"
#include "rtos/ethercat/slave.hpp"
#include "rtos/modbus/register_bank.hpp"

namespace ecat_fixture {

namespace ec = rtos::ethercat;

constexpr ec::SiiConfig sii_config{
    .pdi_control = 0x82,
    .vendor_id = 0x0000'0000,
    .product_code = 0x0000'9253,
    .revision = 2,
    .serial = 0x1234'5678,
    .mailbox_out_start = 0x1000,
    .mailbox_out_size = 128,
    .mailbox_in_start = 0x1080,
    .mailbox_in_size = 128,
    .mailbox_protocols = ec::sii::mailbox_coe,
    .coe_details = ec::sii::coe_sdo | ec::sii::coe_sdo_info,
    .name = "Test slave",
    .group = "Tests",
    .fmmus = {ec::sii::fmmu_outputs, ec::sii::fmmu_inputs, ec::sii::fmmu_sm_status},
    .sync_managers = {{{.start = 0x1000, .length = 128, .control = 0x26, .enable = true, .type = ec::sii::sm_mailbox_out},
                       {.start = 0x1080, .length = 128, .control = 0x22, .enable = true, .type = ec::sii::sm_mailbox_in},
                       {.start = 0x1100, .length = 0, .control = 0x64, .enable = false, .type = ec::sii::sm_outputs},
                       {.start = 0x1400, .length = 0, .control = 0x20, .enable = false, .type = ec::sii::sm_inputs}}},
};
constexpr auto image = ec::make_sii<256>(sii_config);
constexpr ec::SlaveLayout layout{.mailbox_out_start = 0x1000, .mailbox_out_size = 128, .mailbox_in_start = 0x1080,
                                 .mailbox_in_size = 128, .outputs_start = 0x1100, .inputs_start = 0x1400};

inline std::uint8_t byte_of(const std::array<std::uint16_t, 256>& w, std::size_t i) { return static_cast<std::uint8_t>(w[i / 2] >> (8 * (i % 2))); }

struct App {
    std::vector<std::pair<int, int>> changes;
    std::vector<std::uint16_t> refused, errors;
    std::size_t eeprom = 0;
    void al_changed(std::uint8_t from, std::uint8_t to) noexcept { changes.emplace_back(from, to); }
    void al_refused(std::uint8_t, std::uint8_t, std::uint16_t code) noexcept { refused.push_back(code); }
    void al_error(std::uint8_t, std::uint8_t, std::uint16_t code) noexcept { errors.push_back(code); }
    void eeprom_command(unsigned, std::uint32_t) noexcept { ++eeprom; }
};
static_assert(ec::SlaveApp<App>);

// ---- a small object dictionary over 8 + 8 registers -----------------------------------------------
// inputs 6000h: 1 = input 0 (UINT16), 2 = inputs 1-2 (INT32); outputs 7000h: 1 = holding 0;
// configuration 8000h: 1 = holdings 1-2 (UDINT); RxPDO 1600h -> 7000h:01, TxPDO 1A00h -> 6000h:01, :02
constexpr ec::DeviceInfo device{.name = "Test slave", .hardware_version = "1", .software_version = "0.1.0",
                                .vendor_id = 0, .product_code = 0x9253, .revision = 2, .serial = 0x1234'5678};
constexpr ec::StandardEntries standard = ec::standard_entries(device);
constexpr std::array inputs_entries{
    ec::count_entry(2),
    ec::OdEntry{.subindex = 1, .data_type = ec::dtype::uint16, .bits = 16, .access = ec::access::read | ec::access::txpdo,
                .source = ec::Source::input, .address = 0, .name = "Level"},
    ec::OdEntry{.subindex = 2, .data_type = ec::dtype::int32, .bits = 32, .access = ec::access::read | ec::access::txpdo,
                .source = ec::Source::input, .address = 1, .name = "Position", .is_signed = true}};
constexpr std::array outputs_entries{
    ec::count_entry(1),
    ec::OdEntry{.subindex = 1, .data_type = ec::dtype::uint16, .bits = 16, .access = ec::access::read_write | ec::access::rxpdo,
                .source = ec::Source::holding, .address = 0, .name = "Valve"}};
constexpr std::array config_entries{
    ec::count_entry(1),
    ec::OdEntry{.subindex = 1, .data_type = ec::dtype::uint32, .bits = 32, .access = ec::access::read_write,
                .source = ec::Source::holding, .address = 1, .name = "Speed"}};
constexpr std::array rxpdo_entries{ec::count_entry(1),
                                   ec::OdEntry{.subindex = 1, .data_type = ec::dtype::uint32, .bits = 32,
                                               .value = ec::pdo_mapping(0x7000, 1, 16), .name = "SubIndex 001"}};
constexpr std::array txpdo_entries{ec::count_entry(2),
                                   ec::OdEntry{.subindex = 1, .data_type = ec::dtype::uint32, .bits = 32,
                                               .value = ec::pdo_mapping(0x6000, 1, 16), .name = "SubIndex 001"},
                                   ec::OdEntry{.subindex = 2, .data_type = ec::dtype::uint32, .bits = 32,
                                               .value = ec::pdo_mapping(0x6000, 2, 32), .name = "SubIndex 002"}};
constexpr std::array<ec::OdObject, 5> application{
    ec::OdObject{.index = 0x1600, .code = ec::object_record, .name = "RxPDO outputs", .entries = rxpdo_entries},
    ec::OdObject{.index = 0x1A00, .code = ec::object_record, .name = "TxPDO inputs", .entries = txpdo_entries},
    ec::OdObject{.index = 0x6000, .code = ec::object_record, .name = "Inputs", .entries = inputs_entries},
    ec::OdObject{.index = 0x7000, .code = ec::object_record, .name = "Outputs", .entries = outputs_entries},
    ec::OdObject{.index = 0x8000, .code = ec::object_record, .name = "Configuration", .entries = config_entries}};
constexpr auto od = ec::join_objects(ec::standard_objects(standard), application);
static_assert(ec::valid_dictionary(od));

using Bank = rtos::modbus::RegisterBank<8, 8>;

}  // namespace ecat_fixture
