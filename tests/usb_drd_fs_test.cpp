// SPDX-License-Identifier: MIT
//
// The STM32H5 USB_DRD_FS device driver against a simulated controller with the hardware's
// register semantics (RM0481 / ST's stm32h5xx_ll_usb.h):
//   CHEPnR  EA, UTYPE, KIND plain; STATRX/TX and DTOGRX/TX toggle where 1 is written; VTRX/VTTX
//           cleared where 0 is written, unchanged where 1 is written; SETUP read-only
//   a completed transaction sets the direction's STAT to NAK and flips its DTOG; a SETUP sets
//   both toggles to DATA1; ISTR.CTR/IDN follow the endpoints with VTRX/VTTX, the other flags
//   are cleared by writing 0
//   PMA: 2 KB, 32-bit words, buffer descriptors at 8 n (TX count 31:16, RX count 25:16)
// A simulated host sends SETUP, OUT and IN tokens. Neither emulator models this controller,
// so these tests stand in for it until the driver runs on the board.
#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <optional>
#include <vector>

#include "rtos/port/stm32h5/usb_device.hpp"
#include "rtos/usb/descriptors.hpp"
#include "rtos/usb/device.hpp"

namespace {

namespace h5 = rtos::port::stm32h5;
namespace usb = rtos::usb;
using Bytes = std::vector<std::uint8_t>;

constexpr std::uint32_t stat_disabled = 0, stat_stall = 1, stat_nak = 2, stat_valid = 3;

struct SimulatedUsb {
    std::array<std::uint32_t, 8> chep{};
    std::uint32_t cntr = 3, flags = 0, daddr = 0, bcdr = 0;  // flags: ISTR without CTR/IDN
    std::array<std::uint32_t, 512> pma{};
    bool setup_needs_valid = false;  ///< variant: SETUP only accepted with STAT_RX = VALID

    static constexpr std::uint32_t toggles = (3u << 4) | (1u << 6) | (3u << 12) | (1u << 14);
    static constexpr std::uint32_t vtrx = 1u << 15, vttx = 1u << 7, setup = 1u << 11;

    void write_chep(unsigned n, std::uint32_t v) {
        std::uint32_t r = chep[n];
        r = (r & ~(0xFu | (1u << 8) | (3u << 9))) | (v & (0xFu | (1u << 8) | (3u << 9)));  // plain
        r ^= v & toggles;                                                                // toggle
        if ((v & vtrx) == 0) {
            r &= ~(vtrx | setup);
        }
        if ((v & vttx) == 0) {
            r &= ~vttx;
        }
        chep[n] = r;
    }
    std::uint32_t istr() const {
        std::uint32_t i = flags;
        for (unsigned n = 0; n < 8; ++n) {
            if ((chep[n] & (vtrx | vttx)) != 0) {
                i |= (1u << 15) | n | ((chep[n] & vtrx) != 0 ? 1u << 4 : 0u);
                break;
            }
        }
        return i;
    }
    std::uint32_t stat_tx(unsigned n) const { return (chep[n] >> 4) & 3u; }
    std::uint32_t stat_rx(unsigned n) const { return (chep[n] >> 12) & 3u; }
    bool dtog_tx(unsigned n) const { return (chep[n] & (1u << 6)) != 0; }
    bool dtog_rx(unsigned n) const { return (chep[n] & (1u << 14)) != 0; }
    void set_field(unsigned n, unsigned shift, std::uint32_t value) {
        chep[n] = (chep[n] & ~(3u << shift)) | (value << shift);
    }
    std::uint16_t rx_buffer(unsigned n) const { return static_cast<std::uint16_t>(pma[(8 * n + 4) / 4] & 0xFFFFu); }
    std::uint16_t rx_capacity(unsigned n) const {
        const std::uint32_t bd = pma[(8 * n + 4) / 4];
        const std::uint32_t blocks = (bd >> 26) & 0x1Fu;
        return static_cast<std::uint16_t>((bd & (1u << 31)) != 0 ? (blocks + 1) * 32 : blocks * 2);
    }
    void write_bytes(std::uint16_t at, const Bytes& data) {
        for (std::size_t i = 0; i < data.size(); ++i) {
            std::uint32_t& w = pma[(at + i) / 4];
            const unsigned shift = 8 * ((at + i) % 4);
            w = (w & ~(0xFFu << shift)) | (std::uint32_t{data[i]} << shift);
        }
    }
    Bytes read_bytes(std::uint16_t at, std::size_t n) const {
        Bytes out(n);
        for (std::size_t i = 0; i < n; ++i) {
            out[i] = static_cast<std::uint8_t>(pma[(at + i) / 4] >> (8 * ((at + i) % 4)));
        }
        return out;
    }
};

/// The Regs policy over the simulation.
struct SimRegs {
    SimulatedUsb* s = nullptr;
    struct Masked {};
    std::uint32_t chep(unsigned n) noexcept { return s->chep[n]; }
    void set_chep(unsigned n, std::uint32_t v) noexcept { s->write_chep(n, v); }
    std::uint32_t cntr() noexcept { return s->cntr; }
    void set_cntr(std::uint32_t v) noexcept { s->cntr = v; }
    std::uint32_t istr() noexcept { return s->istr(); }
    void set_istr(std::uint32_t v) noexcept { s->flags &= v; }
    void set_daddr(std::uint32_t v) noexcept { s->daddr = v; }
    std::uint32_t bcdr() noexcept { return s->bcdr; }
    void set_bcdr(std::uint32_t v) noexcept { s->bcdr = v; }
    std::uint32_t pma_read(std::size_t offset) noexcept { return s->pma[offset / 4]; }
    void pma_write(std::size_t offset, std::uint32_t v) noexcept { s->pma[offset / 4] = v; }
};

using Driver = h5::UsbDrdFs<SimRegs>;
static_assert(usb::DeviceController<Driver>);

enum class Handshake { Ack, Nak, Stall, None };

/// The host side: tokens as the controller sees them.
struct Host {
    SimulatedUsb& s;
    Driver& d;

    void bus_reset() {
        s.flags |= 1u << 10;
        d.on_irq();
    }
    Handshake setup(const Bytes& packet) {
        if (s.setup_needs_valid && s.stat_rx(0) != stat_valid) {
            return Handshake::Nak;
        }
        s.write_bytes(s.rx_buffer(0), packet);
        s.pma[1] = (s.pma[1] & 0xFC00'FFFFu) | (8u << 16);
        s.chep[0] |= SimulatedUsb::setup | SimulatedUsb::vtrx | (1u << 14) | (1u << 6);  // both toggles DATA1
        s.set_field(0, 12, stat_nak);
        d.on_irq();
        return Handshake::Ack;
    }
    Handshake out(unsigned ep, const Bytes& data) {
        switch (s.stat_rx(ep)) {
            case stat_disabled: return Handshake::None;
            case stat_stall: return Handshake::Stall;
            case stat_nak: return Handshake::Nak;
            default: break;
        }
        EXPECT_LE(data.size(), s.rx_capacity(ep));
        s.write_bytes(s.rx_buffer(ep), data);
        s.pma[(8 * ep + 4) / 4] = (s.pma[(8 * ep + 4) / 4] & 0xFC00'FFFFu) | (static_cast<std::uint32_t>(data.size()) << 16);
        s.chep[ep] |= SimulatedUsb::vtrx;
        s.chep[ep] ^= 1u << 14;
        s.set_field(ep, 12, stat_nak);
        d.on_irq();
        return Handshake::Ack;
    }
    /// IN token: the data packet, or the handshake that came instead.
    std::optional<Bytes> in(unsigned ep, Handshake* hs = nullptr) {
        const auto set = [&](Handshake h) {
            if (hs != nullptr) {
                *hs = h;
            }
        };
        switch (s.stat_tx(ep)) {
            case stat_disabled: set(Handshake::None); return std::nullopt;
            case stat_stall: set(Handshake::Stall); return std::nullopt;
            case stat_nak: set(Handshake::Nak); return std::nullopt;
            default: break;
        }
        const std::uint32_t bd = s.pma[(8 * ep) / 4];
        Bytes packet = s.read_bytes(static_cast<std::uint16_t>(bd & 0xFFFFu), bd >> 16);
        s.chep[ep] |= SimulatedUsb::vttx;
        s.chep[ep] ^= 1u << 6;
        s.set_field(ep, 4, stat_nak);
        d.on_irq();
        set(Handshake::Ack);
        return packet;
    }
};

std::vector<usb::Event> drain(Driver& d) {
    std::vector<usb::Event> events;
    while (auto e = d.poll()) {
        events.push_back(*e);
    }
    return events;
}

std::span<std::byte> as_span(std::vector<std::byte>& v) { return {v.data(), v.size()}; }

class UsbDrdFsTest : public ::testing::Test {
protected:
    SimulatedUsb sim;
    Driver drv{SimRegs{&sim}};
    Host host{sim, drv};
    void SetUp() override {
        drv.start();
        host.bus_reset();
        const auto ev = drain(drv);
        ASSERT_EQ(ev.size(), 1u);
        ASSERT_EQ(ev[0].type, usb::EventType::BusReset);
    }
};

TEST_F(UsbDrdFsTest, BusResetPreparesControlEndpoint) {
    EXPECT_EQ(sim.daddr, 0x80u) << "EF, address 0";
    EXPECT_EQ((sim.chep[0] >> 9) & 3u, 1u) << "control";
    EXPECT_EQ(sim.stat_rx(0), stat_valid);
    EXPECT_EQ(sim.stat_tx(0), stat_nak);
    EXPECT_EQ(sim.rx_capacity(0), 64u);
    EXPECT_EQ(sim.cntr & 0x8000u, 0x8000u) << "CTR interrupt enabled";
}

TEST_F(UsbDrdFsTest, SetupThenInDataStageWithAZeroLengthPacket) {
    const Bytes get_descriptor{0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00};
    ASSERT_EQ(host.setup(get_descriptor), Handshake::Ack);
    auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    ASSERT_EQ(ev[0].type, usb::EventType::Setup);
    EXPECT_EQ(std::to_integer<int>(ev[0].setup[1]), 0x06);
    EXPECT_EQ(sim.stat_tx(0), stat_nak) << "no data until the stack answers";
    std::vector<std::byte> data(128);
    for (std::size_t i = 0; i < data.size(); ++i) {
        data[i] = static_cast<std::byte>(i);
    }
    ASSERT_TRUE(drv.transfer(0x80, as_span(data), true));
    EXPECT_EQ(host.in(0)->size(), 64u);
    EXPECT_TRUE(drain(drv).empty()) << "not complete after the first packet";
    EXPECT_EQ(host.in(0)->size(), 64u);
    EXPECT_EQ(host.in(0)->size(), 0u) << "a full last packet is followed by a zero-length one";
    ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].type, usb::EventType::TransferComplete);
    EXPECT_EQ(ev[0].endpoint, 0x80);
    EXPECT_EQ(ev[0].length, 128);
    Handshake hs{};
    EXPECT_FALSE(host.in(0, &hs).has_value());
    EXPECT_EQ(hs, Handshake::Nak);
}

TEST_F(UsbDrdFsTest, ShortInTransferNeedsNoZeroLengthPacket) {
    host.setup({0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00});
    drain(drv);
    std::vector<std::byte> data(18, std::byte{0x12});
    ASSERT_TRUE(drv.transfer(0x80, as_span(data), true));
    const auto p = host.in(0);
    ASSERT_TRUE(p.has_value());
    EXPECT_EQ(p->size(), 18u);
    EXPECT_EQ((*p)[17], 0x12);
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].length, 18);
}

TEST_F(UsbDrdFsTest, BulkOutEndsWithAShortPacketAndNaksWhileIdle) {
    ASSERT_TRUE(drv.open({0x01, usb::TransferType::Bulk, 64}));
    EXPECT_EQ(host.out(1, Bytes(64, 1)), Handshake::Nak) << "no transfer armed";
    std::vector<std::byte> buffer(200);
    ASSERT_TRUE(drv.transfer(0x01, as_span(buffer), false));
    EXPECT_EQ(host.out(1, Bytes(64, 1)), Handshake::Ack);
    EXPECT_EQ(host.out(1, Bytes(64, 2)), Handshake::Ack);
    EXPECT_TRUE(drain(drv).empty());
    EXPECT_EQ(host.out(1, Bytes(30, 3)), Handshake::Ack);
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].endpoint, 0x01);
    EXPECT_EQ(ev[0].length, 158);
    EXPECT_EQ(buffer[0], std::byte{1});
    EXPECT_EQ(buffer[64], std::byte{2});
    EXPECT_EQ(buffer[157], std::byte{3});
    EXPECT_EQ(host.out(1, Bytes(10, 4)), Handshake::Nak) << "idle again";
    EXPECT_FALSE(drv.transfer(0x81, as_span(buffer), false)) << "IN 1 is not open";
}

TEST_F(UsbDrdFsTest, OutTransferEndsWhenTheBufferIsFull) {
    ASSERT_TRUE(drv.open({0x02, usb::TransferType::Bulk, 64}));
    std::vector<std::byte> buffer(128);
    ASSERT_TRUE(drv.transfer(0x02, as_span(buffer), false));
    host.out(2, Bytes(64, 7));
    host.out(2, Bytes(64, 8));
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].length, 128);
}

TEST_F(UsbDrdFsTest, DataTogglesAlternateAndClearStallResetsThem) {
    ASSERT_TRUE(drv.open({0x81, usb::TransferType::Bulk, 64}));
    EXPECT_FALSE(sim.dtog_tx(1)) << "DATA0 after opening";
    std::vector<std::byte> data(10);
    for (int i = 0; i < 3; ++i) {
        ASSERT_TRUE(drv.transfer(0x81, as_span(data), false));
        host.in(1);
    }
    EXPECT_TRUE(sim.dtog_tx(1));
    drv.stall(0x81);
    Handshake hs{};
    EXPECT_FALSE(host.in(1, &hs).has_value());
    EXPECT_EQ(hs, Handshake::Stall);
    drv.clear_stall(0x81);
    EXPECT_FALSE(sim.dtog_tx(1)) << "CLEAR_FEATURE(ENDPOINT_HALT) resets the toggle to DATA0";
    EXPECT_EQ(sim.stat_tx(1), stat_nak);
}

TEST_F(UsbDrdFsTest, ControlStallEndsWithTheNextSetup) {
    host.setup({0x80, 0x06, 0x00, 0x09, 0x00, 0x00, 0x40, 0x00});
    drain(drv);
    drv.stall(0x00);
    Handshake hs{};
    EXPECT_FALSE(host.in(0, &hs).has_value());
    EXPECT_EQ(hs, Handshake::Stall);
    EXPECT_EQ(host.setup({0x80, 0x00, 0, 0, 0, 0, 2, 0}), Handshake::Ack);
    EXPECT_EQ(drain(drv).size(), 1u);
    EXPECT_EQ(sim.stat_tx(0), stat_nak);
}

TEST_F(UsbDrdFsTest, Ep0OutDataBeforeTheStackAsksIsKept) {
    host.setup({0x21, 0x20, 0, 0, 0, 0, 7, 0});  // CDC SET_LINE_CODING, 7 bytes OUT
    drain(drv);
    EXPECT_EQ(host.out(0, {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08}), Handshake::Ack) << "EP0 stays ready";
    EXPECT_TRUE(drain(drv).empty());
    std::vector<std::byte> buffer(7);
    ASSERT_TRUE(drv.transfer(0x00, as_span(buffer), false));
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].length, 7);
    EXPECT_EQ(buffer[1], std::byte{0xC2});
    EXPECT_EQ(drv.stats().ep0_kept, 1u);
}

TEST_F(UsbDrdFsTest, OpenAllocatesPacketMemoryAndCloseAllFreesIt) {
    for (std::uint8_t n = 1; n < 8; ++n) {
        ASSERT_TRUE(drv.open({n, usb::TransferType::Bulk, 64}));
        ASSERT_TRUE(drv.open({static_cast<std::uint8_t>(0x80 | n), usb::TransferType::Bulk, 64}));
    }
    EXPECT_FALSE(drv.open({0x08, usb::TransferType::Bulk, 64})) << "8 endpoint registers";
    drv.close_all();
    EXPECT_EQ(sim.stat_rx(1), stat_disabled);
    ASSERT_TRUE(drv.open({0x01, usb::TransferType::Bulk, 64}));
    EXPECT_EQ(sim.rx_buffer(1), 64u + 128u) << "first buffer after the BD table and EP0";
}

// ---- the real stack over the driver ------------------------------------------------------------

struct Loopback final : usb::Function {
    std::array<std::byte, 64> rx{};
    std::uint8_t first_interface() const noexcept override { return 0; }
    std::uint8_t interface_count() const noexcept override { return 1; }
    bool owns_endpoint(std::uint8_t ep) const noexcept override { return ep == 0x01 || ep == 0x81; }
    void configured(usb::Endpoints& eps) noexcept override {
        eps.open({0x01, usb::TransferType::Bulk, 64});
        eps.open({0x81, usb::TransferType::Bulk, 64});
        eps.transfer(0x01, rx);
    }
    void reset() noexcept override {}
    void transfer_complete(usb::Endpoints& eps, std::uint8_t ep, std::size_t length) noexcept override {
        if (ep == 0x01) {
            eps.transfer(0x81, std::span(rx).first(length));  // echo
        } else {
            eps.transfer(0x01, rx);
        }
    }
};

constexpr auto device_desc = usb::device_descriptor({.vendor = 0x1209, .product = 0x0001});
constexpr auto config_desc = usb::configuration_descriptor(
    {.max_power_ma = 100}, usb::detail::join(usb::detail::interface(0, 0, 2, 0xFF, 0, 0, 0),
                                             usb::detail::endpoint(0x01, usb::TransferType::Bulk, 64, 0),
                                             usb::detail::endpoint(0x81, usb::TransferType::Bulk, 64, 0)));
constexpr std::string_view strings[] = {"rtoos", "FCU", "0001"};

TEST(UsbDrdFsStack, EnumeratesAndEchoesOverTheRegisterModel) {
    for (const bool needs_valid : {false, true}) {
        SimulatedUsb sim;
        sim.setup_needs_valid = needs_valid;  // both controller behaviours
        Driver drv{SimRegs{&sim}};
        Loopback fn;
        std::array<usb::Function*, 1> functions{&fn};
        usb::Device<Driver> dev{drv, {device_desc, config_desc, strings}, functions};
        Host host{sim, drv};
        drv.start();
        dev.connect();
        EXPECT_EQ(sim.bcdr & 0x8000u, 0x8000u) << "pull-up";
        host.bus_reset();
        dev.service();
        const auto control_in = [&](const Bytes& setup) {
            EXPECT_EQ(host.setup(setup), Handshake::Ack);
            dev.service();
            Bytes all;
            while (auto p = host.in(0)) {
                all.insert(all.end(), p->begin(), p->end());
                dev.service();
                if (p->size() < 64) {
                    break;
                }
            }
            EXPECT_EQ(host.out(0, {}), Handshake::Ack) << "status stage";
            dev.service();
            return all;
        };
        const auto control_out = [&](const Bytes& setup) {
            EXPECT_EQ(host.setup(setup), Handshake::Ack);
            dev.service();
            const auto status = host.in(0);
            EXPECT_TRUE(status.has_value() && status->empty()) << "zero-length status packet";
            dev.service();
        };
        const Bytes dd = control_in({0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00});
        ASSERT_EQ(dd.size(), 18u);
        EXPECT_EQ(dd[0], 18);
        control_out({0x00, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00});  // SET_ADDRESS 7
        EXPECT_EQ(sim.daddr, 0x80u | 7u) << "after the status stage";
        const Bytes cd = control_in({0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0xFF, 0x00});
        EXPECT_EQ(cd.size(), config_desc.size());
        control_out({0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00});  // SET_CONFIGURATION 1
        ASSERT_TRUE(dev.configured());
        // bulk echo
        EXPECT_EQ(host.out(1, {1, 2, 3, 4, 5}), Handshake::Ack);
        dev.service();
        const auto echo = host.in(1);
        ASSERT_TRUE(echo.has_value());
        EXPECT_EQ(*echo, (Bytes{1, 2, 3, 4, 5}));
        dev.service();
        EXPECT_EQ(host.out(1, {9}), Handshake::Ack) << "re-armed after the echo";
    }
}

}  // namespace
