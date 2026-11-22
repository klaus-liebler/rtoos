// SPDX-License-Identifier: MIT
//
// The EtherCAT slave stack (include/rtos/ethercat) against a simulated ESC (ethercat_sim.hpp), and
// the LAN9253 driver's SPI framing against a recording bus.
#include <gtest/gtest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "ethercat_sim.hpp"
#include "rtos/devices/lan9253.hpp"
#include "rtos/ethercat/coe.hpp"
#include "rtos/ethercat/object_dictionary.hpp"
#include "rtos/ethercat/sii.hpp"
#include "rtos/ethercat/slave.hpp"
#include "rtos/modbus/register_bank.hpp"

namespace {

namespace ec = rtos::ethercat;
using ecat_sim::SimEsc;

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

std::uint8_t byte_of(const std::array<std::uint16_t, 256>& w, std::size_t i) { return static_cast<std::uint8_t>(w[i / 2] >> (8 * (i % 2))); }

struct App {
    std::vector<std::pair<int, int>> changes;
    std::vector<std::uint16_t> refused;
    std::size_t eeprom = 0;
    void al_changed(std::uint8_t from, std::uint8_t to) noexcept { changes.emplace_back(from, to); }
    void al_refused(std::uint8_t, std::uint8_t, std::uint16_t code) noexcept { refused.push_back(code); }
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

struct EthercatSlave : ::testing::Test {
    SimEsc esc;
    App app;
    Bank bank;
    ec::Slave<SimEsc, App, 256> slave{esc, image, layout, od, bank, app};

    void mailbox_sms() {
        esc.configure_sm(0, 0x1000, 128, 0x26, true);
        esc.configure_sm(1, 0x1080, 128, 0x22, true);
    }
    /// SM2/SM3 for the default assignment: 2 bytes out, 6 bytes in.
    void process_sms() {
        esc.configure_sm(2, 0x1100, 2, 0x64, true);
        esc.configure_sm(3, 0x1400, 6, 0x20, true);
    }
};

// ---- SII ------------------------------------------------------------------------------------------
TEST(EthercatSii, ConfigurationAreaChecksumAndIdentity) {
    std::uint8_t crc = 0xFF;  // over words 0..6 and the CRC byte the remainder is 0
    for (std::size_t i = 0; i < 15; ++i) {
        crc ^= byte_of(image, i);
        for (int b = 0; b < 8; ++b) crc = static_cast<std::uint8_t>((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
    }
    EXPECT_EQ(crc, 0);
    EXPECT_EQ(image[0] & 0xFF, 0x82);
    EXPECT_EQ(image[0x0A], 0x9253);
    EXPECT_EQ(image[0x0C], 2);
    EXPECT_EQ(image[0x0E] | (image[0x0F] << 16), 0x1234'5678u);
    EXPECT_EQ(image[0x18], 0x1000);
    EXPECT_EQ(image[0x1B], 128);
    EXPECT_EQ(image[0x1C], ec::sii::mailbox_coe);
    EXPECT_EQ(image[0x3E], 3);  // 4 Kbit
}

TEST(EthercatSii, CategoriesWalkToTheEnd) {
    std::size_t p = 0x40;
    std::vector<std::uint16_t> types;
    while (image[p] != ec::sii::cat_end && p < image.size()) {
        types.push_back(image[p]);
        if (image[p] == ec::sii::cat_strings) {
            const std::size_t b = 2 * (p + 2);
            EXPECT_EQ(byte_of(image, b), 2);  // two strings
            const std::size_t len = byte_of(image, b + 1);
            std::string name;
            for (std::size_t i = 0; i < len; ++i) name += static_cast<char>(byte_of(image, b + 2 + i));
            EXPECT_EQ(name, "Test slave");
        }
        if (image[p] == ec::sii::cat_general) {
            EXPECT_EQ(image[p + 2 + 2] >> 8, ec::sii::coe_sdo | ec::sii::coe_sdo_info);
        }
        if (image[p] == ec::sii::cat_syncm) {
            EXPECT_EQ(image[p + 2], 0x1000);
            EXPECT_EQ(image[p + 2 + 3] >> 8, ec::sii::sm_mailbox_out);
            EXPECT_EQ(image[p + 2 + 4 + 2] & 0xFF, 0x22);
        }
        p += 2 + image[p + 1];
    }
    EXPECT_EQ(types, (std::vector<std::uint16_t>{ec::sii::cat_strings, ec::sii::cat_general, ec::sii::cat_fmmu, ec::sii::cat_syncm}));
}

// ---- AL transitions ---------------------------------------------------------------------------------
TEST(EthercatAl, TransitionTable) {
    EXPECT_EQ(ec::transition_code(1, 2), ec::al_code::none);
    EXPECT_EQ(ec::transition_code(2, 4), ec::al_code::none);
    EXPECT_EQ(ec::transition_code(4, 8), ec::al_code::none);
    EXPECT_EQ(ec::transition_code(8, 1), ec::al_code::none);
    EXPECT_EQ(ec::transition_code(4, 2), ec::al_code::none);
    EXPECT_EQ(ec::transition_code(1, 4), ec::al_code::invalid_state_change);
    EXPECT_EQ(ec::transition_code(1, 8), ec::al_code::invalid_state_change);
    EXPECT_EQ(ec::transition_code(2, 8), ec::al_code::invalid_state_change);
    EXPECT_EQ(ec::transition_code(1, 3), ec::al_code::bootstrap_not_supported);
    EXPECT_EQ(ec::transition_code(1, 5), ec::al_code::unknown_state);
    EXPECT_EQ(ec::transition_code(1, 0), ec::al_code::unknown_state);
}

TEST_F(EthercatSlave, UpToOpAndDownToInit) {
    mailbox_sms();
    process_sms();
    for (const std::uint8_t s : {2, 4, 8, 1}) {
        esc.request(s);
        slave.poll(true);
        EXPECT_EQ(esc.al_status(), s);
        EXPECT_EQ(esc.al_code(), 0);
        EXPECT_EQ(slave.state(), s);
    }
    EXPECT_EQ(app.changes, (std::vector<std::pair<int, int>>{{1, 2}, {2, 4}, {4, 8}, {8, 1}}));
    EXPECT_EQ(esc.events() & ec::al_event::al_control, 0u);  // the read of AL control cleared the event
}

TEST_F(EthercatSlave, PreOpRefusedWithoutTheMailboxSyncManagers) {
    esc.request(2);
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 1 | ec::al_error_flag);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_mailbox_config);
    // the wrong size is refused as well
    esc.configure_sm(0, 0x1000, 64, 0x26, true);
    esc.configure_sm(1, 0x1080, 128, 0x22, true);
    esc.request(2, true);
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_mailbox_config);
    EXPECT_EQ(slave.counters().al_refused, 2u);
    // as the SII says: accepted, with the acknowledge
    mailbox_sms();
    esc.request(2, true);
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 2);
    EXPECT_EQ(esc.al_code(), 0);
}

TEST_F(EthercatSlave, AnIndicatedErrorWaitsForTheAcknowledge) {
    mailbox_sms();
    esc.request(8);  // INIT -> OP: not allowed
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 1 | ec::al_error_flag);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_state_change);
    esc.request(2);  // without the acknowledge: ignored
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 1 | ec::al_error_flag);
    esc.request(1, true);  // acknowledged in INIT: the error goes
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 1);
    EXPECT_EQ(esc.al_code(), 0);
}

TEST_F(EthercatSlave, BootIsRefused) {
    esc.request(3);
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::bootstrap_not_supported);
    EXPECT_EQ(app.refused, (std::vector<std::uint16_t>{ec::al_code::bootstrap_not_supported}));
}

TEST_F(EthercatSlave, SafeOpChecksTheProcessDataSyncManagers) {
    mailbox_sms();
    esc.request(2);
    slave.poll(true);
    esc.request(4);  // the assigned PDOs have 2 bytes of outputs: SM2 must carry them
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_output_config);
    esc.configure_sm(2, 0x1100, 2, 0x64, true);
    esc.request(4, true);  // ... and 6 bytes of inputs without SM3
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_input_config);
    esc.configure_sm(3, 0x1400, 4, 0x20, true);  // the wrong size
    esc.request(4, true);
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_input_config);
    process_sms();
    esc.request(4, true);
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 4);
}

TEST_F(EthercatSlave, NotReadyOnlyTheEeprom) {
    esc.request(2);
    const std::size_t writes = esc.pdi_writes;
    slave.poll(false);  // not ready: the AL control stays untouched
    EXPECT_EQ(esc.pdi_writes, writes);
    EXPECT_EQ(esc.al_status(), 1);
    EXPECT_NE(esc.events() & ec::al_event::al_control, 0u);
}

// ---- EEPROM emulation -------------------------------------------------------------------------------
TEST_F(EthercatSlave, EepromReadsTheImage) {
    esc.eeprom_command(ec::eeprom::cmd_read, 0x0A);
    slave.poll(false);  // also before the ESC is ready
    EXPECT_FALSE(esc.eeprom_busy());
    EXPECT_EQ(esc.eeprom_data(), image[0x0A] | (std::uint32_t{image[0x0B]} << 16));
    esc.eeprom_command(ec::eeprom::cmd_read, 0x08, true);  // 8 bytes
    slave.poll(true);
    EXPECT_EQ(esc.eeprom_data(4), image[0x0A] | (std::uint32_t{image[0x0B]} << 16));
    esc.eeprom_command(ec::eeprom::cmd_read, 0xFFFF);  // beyond the image: erased
    slave.poll(true);
    EXPECT_EQ(esc.eeprom_data(), 0xFFFF'FFFFu);
    EXPECT_EQ(slave.counters().eeprom_reads, 3u);
    EXPECT_EQ(esc.events() & ec::al_event::eeprom, 0u);
}

TEST_F(EthercatSlave, EepromWriteIsReadBack) {
    esc.set_eeprom_write_data(0x0042);  // the station alias, word 4
    esc.eeprom_command(ec::eeprom::cmd_write, 4);
    slave.poll(true);
    esc.eeprom_command(ec::eeprom::cmd_read, 4);
    slave.poll(true);
    EXPECT_EQ(esc.eeprom_data() & 0xFFFF, 0x0042u);
    EXPECT_EQ(slave.counters().eeprom_writes, 1u);
}

// ---- diagnostics ---------------------------------------------------------------------------------
TEST_F(EthercatSlave, DiagnosticsReadTheRegisters) {
    esc.set16(ec::reg::station_address, 0x1001);
    esc.set16(ec::reg::dl_status, 0x0215);  // PDI operational, link port 0, communication port 0
    esc.set8(0x0301, 7), esc.set8(0x0302, 2), esc.set8(0x0309, 3), esc.set8(0x0311, 1), esc.set8(0x030D, 9);
    mailbox_sms();
    esc.set32(ec::reg::system_time, 0x89AB'CDEF), esc.set32(ec::reg::system_time + 4, 0x0123'4567);
    const ec::Diagnostics d = slave.read_diagnostics();
    EXPECT_EQ(d.esc_type, 0xC0);
    EXPECT_EQ(d.sync_managers, 8);
    EXPECT_EQ(d.station_address, 0x1001);
    EXPECT_TRUE(ec::dl_status::link(d.dl_status, 0));
    EXPECT_FALSE(ec::dl_status::link(d.dl_status, 1));
    EXPECT_TRUE(ec::dl_status::communication(d.dl_status, 0));
    EXPECT_EQ(d.ports[0].rx_errors, 7);
    EXPECT_EQ(d.ports[1].invalid_frames, 2);
    EXPECT_EQ(d.ports[1].forwarded_errors, 3);
    EXPECT_EQ(d.ports[1].lost_links, 1);
    EXPECT_EQ(d.pdi_errors, 9);
    EXPECT_EQ(d.sm[1].start, 0x1080);
    EXPECT_TRUE(d.sm[1].enabled());
    EXPECT_EQ(d.system_time_ns, 0x0123'4567'89AB'CDEFull);
}

// ---- LAN9253 SPI framing ----------------------------------------------------------------------------
struct RecordingBus {
    static inline std::vector<int> out;  ///< bytes sent; -1 select, -2 deselect
    static inline std::vector<std::uint8_t> answers;
    static inline std::uint32_t waited_ns = 0;
    static void select(bool on) noexcept { out.push_back(on ? -1 : -2); }
    static std::uint8_t transfer(std::uint8_t b) noexcept {
        out.push_back(b);
        if (answers.empty()) return 0;
        const std::uint8_t a = answers.front();
        answers.erase(answers.begin());
        return a;
    }
    static void wait_ns(std::uint32_t ns) noexcept { waited_ns += ns; }
};

TEST(Lan9253, CoreReadIncrementsAndEndsWithSiHigh) {
    RecordingBus::out.clear();
    RecordingBus::answers = {0, 0, 0, 0x34, 0x12};  // command and address bytes, then the data
    rtos::devices::Lan9253<RecordingBus> lan;
    EXPECT_EQ(ec::read16(lan, 0x0130), 0x1234);
    EXPECT_EQ(RecordingBus::out, (std::vector<int>{-1, 0x03, 0x41, 0x30, 0x00, 0xFF, -2}));
}

TEST(Lan9253, CoreWriteWaitsPerByte) {
    RecordingBus::out.clear();
    RecordingBus::waited_ns = 0;
    rtos::devices::Lan9253<RecordingBus> lan;
    ec::write16(lan, 0x0130, 0x0208);
    EXPECT_EQ(RecordingBus::out, (std::vector<int>{-1, 0x02, 0x41, 0x30, 0x08, 0x02, -2}));
    EXPECT_GE(RecordingBus::waited_ns, 2 * 250u + 250 + 2 * 80);
}

TEST(Lan9253, ProbeFindsTheDirectMappedMap) {
    RecordingBus::out.clear();
    RecordingBus::answers = {0, 0, 0, 0x21, 0x43, 0x65, 0x87};  // BYTE_TEST at 3064h
    rtos::devices::Lan9253<RecordingBus> lan;
    EXPECT_EQ(lan.probe(), rtos::devices::Lan9253Mode::direct_mapped);
    EXPECT_EQ(RecordingBus::out[2], 0x30);
    EXPECT_EQ(RecordingBus::out[3], 0x64);
}

// ---- CoE: SDO (the server alone) -----------------------------------------------------------------------
using Bytes = std::vector<std::byte>;

Bytes bytes(std::initializer_list<int> v) {
    Bytes b;
    for (int x : v) b.push_back(std::byte(x & 0xFF));
    return b;
}
/// An SDO request: CoE header (service 2), command, index, subindex, 4 data bytes, then `more`.
Bytes sdo(std::uint8_t cmd, std::uint16_t index, std::uint8_t sub, std::uint32_t data = 0, const Bytes& more = {}) {
    Bytes b = bytes({0x00, 0x20, cmd, index & 0xFF, index >> 8, sub, static_cast<int>(data & 0xFF), static_cast<int>((data >> 8) & 0xFF),
                     static_cast<int>((data >> 16) & 0xFF), static_cast<int>(data >> 24)});
    b.insert(b.end(), more.begin(), more.end());
    return b;
}
std::uint8_t at(const Bytes& b, std::size_t i) { return std::to_integer<std::uint8_t>(b[i]); }
std::uint16_t at16(const Bytes& b, std::size_t i) { return static_cast<std::uint16_t>(at(b, i) | (at(b, i + 1) << 8)); }
std::uint32_t at32(const Bytes& b, std::size_t i) { return at16(b, i) | (std::uint32_t{at16(b, i + 2)} << 16); }

struct EthercatCoe : ::testing::Test {
    Bank bank;
    ec::CoeServer<> coe{od, bank};
    Bytes out = Bytes(122);

    Bytes call(const Bytes& in, std::uint8_t state = 2) {
        const std::size_t n = coe.request(in, out, state);
        return Bytes(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(n));
    }
    std::uint32_t abort_code(const Bytes& r) { return r.size() == 10 && at(r, 2) == 0x80 ? at32(r, 6) : 0; }
};

TEST_F(EthercatCoe, UploadExpeditedFromTheRegisters) {
    bank.set_input(0, 0x1234);
    bank.set_input(1, 0xFFFF), bank.set_input(2, 0xFFFE);  // -2, high word first in the map
    Bytes r = call(sdo(0x40, 0x6000, 1));
    ASSERT_EQ(r.size(), 10u);
    EXPECT_EQ(at16(r, 0) >> 12, ec::coe_service::sdo_response);
    EXPECT_EQ(at(r, 2), 0x4B);  // expedited, size indicated, 2 bytes
    EXPECT_EQ(at16(r, 6), 0x1234);
    r = call(sdo(0x40, 0x6000, 2));
    EXPECT_EQ(at(r, 2), 0x43);  // 4 bytes
    EXPECT_EQ(static_cast<std::int32_t>(at32(r, 6)), -2);
    r = call(sdo(0x40, 0x6000, 0));
    EXPECT_EQ(at(r, 2), 0x4F);  // 1 byte: the count
    EXPECT_EQ(at(r, 6), 2);
    r = call(sdo(0x40, 0x1018, 4));
    EXPECT_EQ(at32(r, 6), 0x1234'5678u);
}

TEST_F(EthercatCoe, UploadNormalString) {
    const Bytes r = call(sdo(0x40, 0x1008, 0));
    ASSERT_EQ(r.size(), 10u + 10);
    EXPECT_EQ(at(r, 2), 0x41);
    EXPECT_EQ(at32(r, 6), 10u);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(r.data()) + 10, 10), "Test slave");
}

TEST_F(EthercatCoe, UploadAborts) {
    EXPECT_EQ(abort_code(call(sdo(0x40, 0x5555, 0))), ec::sdo_abort::no_object);
    EXPECT_EQ(abort_code(call(sdo(0x40, 0x6000, 9))), ec::sdo_abort::no_subindex);
    EXPECT_EQ(abort_code(call(sdo(0x50, 0x6000, 0))), ec::sdo_abort::unsupported_access);  // complete access
    EXPECT_EQ(abort_code(call(sdo(0x60, 0x6000, 0))), ec::sdo_abort::command);             // upload segment
    EXPECT_EQ(call(sdo(0x80, 0x6000, 0)).size(), 0u);                                     // the master aborts
    EXPECT_EQ(coe.counters().last_abort, ec::sdo_abort::command);
    // the abort goes out as an SDO request (ETG.1000.6 5.6.2.7)
    EXPECT_EQ(at16(call(sdo(0x40, 0x5555, 0)), 0) >> 12, ec::coe_service::sdo_request);
}

TEST_F(EthercatCoe, DownloadIntoTheRegistersThroughTheMap) {
    Bytes r = call(sdo(0x2B, 0x7000, 1, 0x0001));  // expedited, size indicated, 2 bytes
    ASSERT_EQ(r.size(), 10u);
    EXPECT_EQ(at(r, 2), 0x60);
    EXPECT_EQ(bank.holding(0), 1);
    r = call(sdo(0x23, 0x8000, 1, 0x0001'0002));  // 4 bytes into two registers, high word first
    EXPECT_EQ(at(r, 2), 0x60);
    EXPECT_EQ(bank.holding(1), 0x0001);
    EXPECT_EQ(bank.holding(2), 0x0002);
    // normal (not expedited) with the size in the header
    r = call(sdo(0x21, 0x7000, 1, 2, bytes({0x34, 0x12})));
    EXPECT_EQ(at(r, 2), 0x60);
    EXPECT_EQ(bank.holding(0), 0x1234);
}

TEST_F(EthercatCoe, DownloadAborts) {
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x6000, 1, 5))), ec::sdo_abort::read_only);
    EXPECT_EQ(abort_code(call(sdo(0x23, 0x7000, 1, 5))), ec::sdo_abort::too_long);   // 4 bytes into a UINT16
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x8000, 1, 5))), ec::sdo_abort::too_short);  // 2 bytes into a UDINT
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x1018, 1, 5))), ec::sdo_abort::read_only);
    EXPECT_EQ(abort_code(call(sdo(0x21, 0x7000, 1, 200))), ec::sdo_abort::unsupported_access);  // segments would follow
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x7000, 1, 5), 1)), ec::sdo_abort::device_state);    // INIT: no access
    EXPECT_EQ(bank.holding(0), 0);
}

TEST_F(EthercatCoe, TheWriteFilterRefusesWithLocalControl) {
    bank.set_write_filter({nullptr, [](void*, std::uint16_t, std::uint16_t) noexcept { return false; }});
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x7000, 1, 1))), ec::sdo_abort::local_control);
    EXPECT_EQ(bank.holding(0), 0);
}

TEST_F(EthercatCoe, PdoAssignmentDefaultAndChangeInPreOp) {
    EXPECT_EQ(coe.process_data_bytes(false), 2);
    EXPECT_EQ(coe.process_data_bytes(true), 6);
    EXPECT_EQ(at(call(sdo(0x40, 0x1C13, 0)), 6), 1);
    EXPECT_EQ(at16(call(sdo(0x40, 0x1C13, 1)), 6), 0x1A00);
    // SAFE-OP: not writable
    EXPECT_EQ(abort_code(call(sdo(0x2F, 0x1C13, 0, 0), 4)), ec::sdo_abort::device_state);
    // an entry while the count is not 0
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x1C13, 1, 0x1A00))), ec::sdo_abort::device_state);
    // the sequence: count 0, the entries, the count
    EXPECT_EQ(at(call(sdo(0x2F, 0x1C13, 0, 0)), 2), 0x60);
    EXPECT_EQ(coe.process_data_bytes(true), 0);
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x1C13, 1, 0x1600))), ec::sdo_abort::value_range);  // an RxPDO
    EXPECT_EQ(abort_code(call(sdo(0x2B, 0x1C13, 1, 0x1A07))), ec::sdo_abort::value_range);  // no such PDO
    EXPECT_EQ(abort_code(call(sdo(0x2F, 0x1C13, 0, 1))), ec::sdo_abort::value_range);       // entry 1 not set
    EXPECT_EQ(at(call(sdo(0x2B, 0x1C13, 1, 0x1A00)), 2), 0x60);
    EXPECT_EQ(at(call(sdo(0x2F, 0x1C13, 0, 1)), 2), 0x60);
    EXPECT_EQ(coe.process_data_bytes(true), 6);
    // and none at all
    EXPECT_EQ(at(call(sdo(0x2F, 0x1C12, 0, 0)), 2), 0x60);
    EXPECT_EQ(coe.process_data_bytes(false), 0);
}

// ---- CoE: SDO information ----------------------------------------------------------------------------
Bytes info(std::uint8_t opcode, const Bytes& data) {
    Bytes b = bytes({0x00, 0x80, opcode, 0, 0, 0});
    b.insert(b.end(), data.begin(), data.end());
    return b;
}

TEST_F(EthercatCoe, OdListCountsAndAllIndexesInFragments) {
    Bytes r = call(info(1, bytes({0, 0})));
    ASSERT_EQ(r.size(), 6u + 12);
    EXPECT_EQ(at16(r, 0) >> 12, ec::coe_service::sdo_info);
    EXPECT_EQ(at(r, 2), ec::sdo_info::od_list_response);
    EXPECT_EQ(at16(r, 8), od.size());  // all
    EXPECT_EQ(at16(r, 10), 1);         // RxPDO-mappable: 7000h
    EXPECT_EQ(at16(r, 12), 1);         // TxPDO-mappable: 6000h
    // all indexes through a small mailbox: fragments with the incomplete flag and the fragments left
    out = Bytes(16);
    r = call(info(1, bytes({1, 0})));
    Bytes data(r.begin() + 6, r.end());
    std::size_t fragments = 1;
    EXPECT_NE(at(r, 2) & ec::sdo_info::incomplete, 0);
    const std::size_t announced = at16(r, 4);
    while (coe.fragments_pending()) {
        const std::size_t n = coe.next_fragment(out);
        ASSERT_GT(n, 6u);
        data.insert(data.end(), out.begin() + 6, out.begin() + static_cast<std::ptrdiff_t>(n));
        ++fragments;
        if (!coe.fragments_pending()) {
            EXPECT_EQ(at(out, 2), ec::sdo_info::od_list_response);  // the last one: complete
        }
    }
    EXPECT_EQ(fragments, announced + 1);
    ASSERT_EQ(data.size(), 2 + 2 * od.size());
    EXPECT_EQ(at16(data, 0), 1);
    for (std::size_t i = 0; i < od.size(); ++i) EXPECT_EQ(at16(data, 2 + 2 * i), od[i].index);
}

TEST_F(EthercatCoe, ObjectAndEntryDescriptions) {
    Bytes r = call(info(3, bytes({0x00, 0x60})));
    EXPECT_EQ(at(r, 2), ec::sdo_info::object_response);
    EXPECT_EQ(at16(r, 6), 0x6000);
    EXPECT_EQ(at16(r, 8), ec::dtype::uint16);  // the first entry type of the record
    EXPECT_EQ(at(r, 10), 2);
    EXPECT_EQ(at(r, 11), ec::object_record);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(r.data()) + 12, r.size() - 12), "Inputs");
    r = call(info(5, bytes({0x00, 0x60, 2, 0})));
    EXPECT_EQ(at(r, 2), ec::sdo_info::entry_response);
    EXPECT_EQ(at16(r, 10), ec::dtype::int32);
    EXPECT_EQ(at16(r, 12), 32);
    EXPECT_EQ(at16(r, 14), ec::access::read | ec::access::txpdo);
    EXPECT_EQ(std::string(reinterpret_cast<const char*>(r.data()) + 16, r.size() - 16), "Position");
    r = call(info(3, bytes({0x55, 0x55})));
    EXPECT_EQ(at(r, 2), ec::sdo_info::error);
    EXPECT_EQ(at32(r, 6), ec::sdo_abort::no_object);
}

// ---- the mailbox (slave over the simulated ESC) ----------------------------------------------------------
struct EthercatMailbox : EthercatSlave {
    void SetUp() override {
        mailbox_sms();
        esc.request(2);
        slave.poll(true);
        ASSERT_EQ(esc.al_status(), 2);
    }
};

TEST_F(EthercatMailbox, CoeRequestAndResponse) {
    bank.set_input(0, 0x4242);
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 1, sdo(0x40, 0x6000, 1)));
    slave.poll(true);
    const Bytes r = esc.mailbox_read();
    ASSERT_EQ(r.size(), 128u);
    EXPECT_EQ(at16(r, 0), 10);                        // length
    EXPECT_EQ(at(r, 5) & 0x0F, ec::mailbox::type_coe);
    EXPECT_EQ(at(r, 5) >> 4, 1);                      // the slave's counter starts at 1
    EXPECT_EQ(at(r, 6 + 2), 0x4B);
    EXPECT_EQ(at16(r, 6 + 6), 0x4242);
    EXPECT_EQ(slave.counters().mailbox_requests, 1u);
    EXPECT_EQ(slave.counters().mailbox_responses, 1u);
}

TEST_F(EthercatMailbox, DuplicateCounterIsDropped) {
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 3, sdo(0x2B, 0x7000, 1, 7)));
    slave.poll(true);
    (void)esc.mailbox_read();
    bank.set_holding(0, 0);
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 3, sdo(0x2B, 0x7000, 1, 7)));  // the same counter: a retry
    slave.poll(true);
    EXPECT_TRUE(esc.mailbox_read().empty());
    EXPECT_EQ(bank.holding(0), 0);
    EXPECT_EQ(slave.counters().mailbox_duplicates, 1u);
}

TEST_F(EthercatMailbox, UnknownProtocolGetsAMailboxError) {
    ASSERT_TRUE(esc.mailbox_write(4 /* FoE */, 1, bytes({1, 2, 3, 4})));
    slave.poll(true);
    const Bytes r = esc.mailbox_read();
    EXPECT_EQ(at(r, 5) & 0x0F, ec::mailbox::type_error);
    EXPECT_EQ(at16(r, 6), 0x0001);
    EXPECT_EQ(at16(r, 8), ec::mailbox::error_unsupported_protocol);
}

TEST_F(EthercatMailbox, ALengthBeyondTheBufferIsAnError) {
    Bytes too_long(130, std::byte{0});
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 1, std::span(too_long).first(122)));
    esc.set16(0x1000, 200);  // the header claims more than the buffer holds
    slave.poll(true);
    const Bytes r = esc.mailbox_read();
    EXPECT_EQ(at(r, 5) & 0x0F, ec::mailbox::type_error);
    EXPECT_EQ(at16(r, 8), ec::mailbox::error_invalid_size);
}

TEST_F(EthercatMailbox, ResponseWaitsForAnEmptySm1AndRepeatRequestResendsIt) {
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 1, sdo(0x40, 0x1018, 2)));
    slave.poll(true);
    const Bytes first = esc.mailbox_read();
    ASSERT_FALSE(first.empty());
    // SM1 full (the master has not read): the next response waits
    esc.set8(ec::reg::sm_base + ec::reg::sm_size + 5, ec::sm_status::mailbox_full);
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 2, sdo(0x40, 0x1018, 3)));
    const std::size_t writes = esc.sm1_writes;
    slave.poll(true);
    EXPECT_EQ(esc.sm1_writes, writes);
    esc.set8(ec::reg::sm_base + ec::reg::sm_size + 5, 0);
    slave.poll(true);
    const Bytes second = esc.mailbox_read();
    EXPECT_EQ(at32(second, 6 + 6), 2u);  // revision
    // the master lost it: repeat request -> the same response again, acknowledged
    esc.toggle_repeat_request();
    slave.poll(true);
    EXPECT_TRUE(esc.repeat_acknowledged());
    EXPECT_EQ(esc.mailbox_read(), second);
}

TEST_F(EthercatMailbox, BackToInitResetsTheAssignment) {
    ASSERT_TRUE(esc.mailbox_write(ec::mailbox::type_coe, 1, sdo(0x2F, 0x1C13, 0, 0)));
    slave.poll(true);
    (void)esc.mailbox_read();
    EXPECT_EQ(slave.coe().process_data_bytes(true), 0);
    esc.request(1);
    slave.poll(true);
    EXPECT_EQ(slave.coe().process_data_bytes(true), 6);
}

}  // namespace
