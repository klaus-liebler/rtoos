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
#include "rtos/ethercat/sii.hpp"
#include "rtos/ethercat/slave.hpp"

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
    std::uint16_t outputs = 0, inputs = 0;
    void al_changed(std::uint8_t from, std::uint8_t to) noexcept { changes.emplace_back(from, to); }
    void al_refused(std::uint8_t, std::uint8_t, std::uint16_t code) noexcept { refused.push_back(code); }
    void eeprom_command(unsigned, std::uint32_t) noexcept { ++eeprom; }
    std::uint16_t outputs_size() const noexcept { return outputs; }
    std::uint16_t inputs_size() const noexcept { return inputs; }
};
static_assert(ec::SlaveApp<App>);

struct EthercatSlave : ::testing::Test {
    SimEsc esc;
    App app;
    ec::Slave<SimEsc, App, 256> slave{esc, image, layout, app};

    void mailbox_sms() {
        esc.configure_sm(0, 0x1000, 128, 0x26, true);
        esc.configure_sm(1, 0x1080, 128, 0x22, true);
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
    app.outputs = 4;  // the application has outputs: SM2 must carry them
    esc.request(4);
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_output_config);
    esc.configure_sm(2, 0x1100, 4, 0x64, true);
    esc.request(4, true);
    slave.poll(true);
    EXPECT_EQ(esc.al_status(), 4);
    app.inputs = 2;  // ... and inputs without SM3
    esc.request(2);
    slave.poll(true);
    esc.request(4);
    slave.poll(true);
    EXPECT_EQ(esc.al_code(), ec::al_code::invalid_input_config);
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

}  // namespace
