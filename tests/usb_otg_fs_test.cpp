// SPDX-License-Identifier: MIT
//
// The STM32H743 USB2_OTG_FS device driver (stm32/usb_otg_device.hpp) against a simulated DWC2 core
// in device mode with the register semantics the driver relies on (ST's stm32h743xx.h, the
// sequences of stm32h7xx_hal_pcd.c / stm32h7xx_ll_usb.c):
//   GINTSTS     rc_w1; RXFLVL while the RX status queue holds an entry; OEPINT / IEPINT while an
//               enabled endpoint interrupt is pending; CMOD = 0 (device) once FDMOD is set
//   GRSTCTL     AHBIDL reads 1; CSRST and the FIFO flushes clear themselves
//   GRXSTSP     pops the next status entry (EPNUM 3:0, BCNT 14:4, PKTSTS 20:17); popping
//               "transfer done" (3) sets DOEPINTn.XFRC and disables the endpoint, "SETUP done" (4)
//               sets DOEPINT0.STUP
//   DxEPCTLn    CNAK/SNAK/SD0PID/EPDIS write-only actions, EPENA set-only (the core clears it
//               when the transaction is done), STALL read/write; a SETUP clears EP0's STALLs
//   DxEPINTn    rc_w1
//   DOEPTSIZn   a received OUT packet takes one from PKTCNT and its bytes from XFRSIZ; a SETUP takes
//               one from EP0's STUPCNT
// A simulated host sends SETUP, OUT and IN tokens. Neither emulator models this core, so these
// tests stand in for it until the driver runs on the board.
#include <gtest/gtest.h>

#include <array>
#include <deque>
#include <map>
#include <optional>
#include <vector>

#include "rtos/port/stm32h7/usb_device.hpp"
#include "rtos/usb/descriptors.hpp"
#include "rtos/usb/device.hpp"

namespace {

namespace h7 = rtos::port::stm32h7;
namespace otg = rtos::port::stm32::otg;
namespace usb = rtos::usb;
using Bytes = std::vector<std::uint8_t>;

constexpr std::uint32_t epena = 1u << 31, epdis = 1u << 30, sd0pid = 1u << 28, snak = 1u << 27, cnak = 1u << 26,
                        stall_bit = 1u << 21;

struct SimulatedOtg {
    std::map<std::uint32_t, std::uint32_t> r;
    struct Status {
        std::uint32_t word;
        std::vector<std::uint32_t> data;
    };
    std::deque<Status> rx;               // status entries with their data words
    std::deque<std::uint32_t> rx_data;  // the words of the entry popped last
    std::array<std::deque<std::uint32_t>, 8> tx;
    std::array<bool, 8> in_nak{}, out_nak{};
    std::array<unsigned, 8> in_pid{}, out_pid{};

    static bool is_ctl(std::uint32_t o, bool& in, unsigned& n) {
        if (o >= 0x900 && o < 0xB00 && (o - 0x900) % 0x20 == 0) {
            in = true, n = (o - 0x900) / 0x20;
            return true;
        }
        if (o >= 0xB00 && o < 0xD00 && (o - 0xB00) % 0x20 == 0) {
            in = false, n = (o - 0xB00) / 0x20;
            return true;
        }
        return false;
    }
    static bool is_int(std::uint32_t o) {
        return (o >= 0x900 && o < 0xD00 && (o - 0x900) % 0x20 == 8);
    }
    std::uint32_t daint() const {
        std::uint32_t v = 0;
        for (unsigned n = 0; n < 8; ++n) {
            if (get(otg::diepint(n)) != 0) v |= 1u << n;
            if (get(otg::doepint(n)) != 0) v |= 1u << (16 + n);
        }
        return v;
    }
    std::uint32_t get(std::uint32_t o) const {
        const auto it = r.find(o);
        return it == r.end() ? 0u : it->second;
    }
    std::uint32_t read(std::uint32_t o) {
        switch (o) {
            case otg::gintsts: {
                std::uint32_t v = get(o) & ~(otg::gint_rxflvl | otg::gint_oepint | otg::gint_iepint | otg::gint_cmod);
                if (!rx.empty()) v |= otg::gint_rxflvl;
                const std::uint32_t d = daint() & get(otg::daintmsk);
                if ((d & 0xFFFF'0000u) != 0) v |= otg::gint_oepint;
                if ((d & 0x0000'FFFFu) != 0) v |= otg::gint_iepint;
                if ((get(otg::gusbcfg) & otg::gusbcfg_fdmod) == 0) v |= otg::gint_cmod;
                return v;
            }
            case otg::grstctl: return get(o) | otg::grstctl_ahbidl;
            case otg::daint: return daint();
            case otg::grxstsp: {
                if (rx.empty()) {
                    ADD_FAILURE() << "GRXSTSP popped while empty";
                    return 0;
                }
                Status s = rx.front();
                rx.pop_front();
                rx_data.assign(s.data.begin(), s.data.end());
                const unsigned n = s.word & 0xF, pktsts = (s.word >> 17) & 0xF;
                if (pktsts == 3) {  // OUT transfer done
                    r[otg::doepctl(n)] &= ~epena;
                    out_nak[n] = true;
                    r[otg::doepint(n)] |= otg::epint_xfrc;
                } else if (pktsts == 4) {  // SETUP stage done
                    r[otg::doepint(0)] |= otg::epint_toc_stup;
                }
                return s.word;
            }
            default: return get(o);
        }
    }
    void write(std::uint32_t o, std::uint32_t v) {
        bool in = false;
        unsigned n = 0;
        if (o == otg::gintsts) {
            r[o] = get(o) & ~v;
        } else if (is_int(o)) {
            r[o] = get(o) & ~v;
        } else if (o == otg::grstctl) {
            if ((v & otg::grstctl_txfflsh) != 0) {
                const unsigned f = (v >> otg::grstctl_txfnum_pos) & 0x1F;
                for (unsigned i = 0; i < 8; ++i) {
                    if (f == 0x10 || f == i) tx[i].clear();
                }
            }
            if ((v & otg::grstctl_rxfflsh) != 0) rx.clear();
            r[o] = v & ~(otg::grstctl_csrst | otg::grstctl_txfflsh | otg::grstctl_rxfflsh);
        } else if (is_ctl(o, in, n)) {
            const std::uint32_t old = get(o);
            std::uint32_t nv = (v & ~(cnak | snak | sd0pid | epdis | epena)) | (old & epena) | (v & epena);
            if ((v & cnak) != 0) (in ? in_nak : out_nak)[n] = false;
            if ((v & snak) != 0) (in ? in_nak : out_nak)[n] = true;
            if ((v & sd0pid) != 0) (in ? in_pid : out_pid)[n] = 0;
            if ((v & epdis) != 0 && (old & epena) != 0) {
                nv &= ~epena;
                r[in ? otg::diepint(n) : otg::doepint(n)] |= otg::epint_epdisd;
            }
            r[o] = nv;
        } else {
            r[o] = v;
        }
    }
    std::uint32_t rx_pop() {
        if (rx_data.empty()) {
            ADD_FAILURE() << "RX FIFO read beyond the packet";
            return 0;
        }
        const std::uint32_t w = rx_data.front();
        rx_data.pop_front();
        return w;
    }
    static std::vector<std::uint32_t> words(const Bytes& b) {
        std::vector<std::uint32_t> w((b.size() + 3) / 4);
        for (std::size_t i = 0; i < b.size(); ++i) w[i / 4] |= std::uint32_t{b[i]} << (8 * (i % 4));
        return w;
    }
    unsigned dad() const { return (get(otg::dcfg) >> 4) & 0x7F; }
};

struct SimRegs {
    static constexpr std::uint8_t irq = 101;
    static constexpr std::uint32_t trdt = 6;
    static constexpr std::uint32_t tx_fifo_words = 16;
    SimulatedOtg* s = nullptr;
    struct Masked {};
    std::uint32_t read(std::uint32_t o) noexcept { return s->read(o); }
    void write(std::uint32_t o, std::uint32_t v) noexcept { s->write(o, v); }
    std::uint32_t rx_pop() noexcept { return s->rx_pop(); }
    void tx_push(unsigned n, std::uint32_t w) noexcept { s->tx[n].push_back(w); }
    void delay_ms(std::uint32_t) noexcept {}
};

using Driver = h7::UsbOtgFs<200'000'000, 400'000'000, SimRegs>;
static_assert(usb::DeviceController<Driver>);
static_assert(h7::UsbOtgHardware<200'000'000, 400'000'000>::irq == 101);

enum class Handshake { Ack, Nak, Stall };

struct Host {
    SimulatedOtg& s;
    Driver& d;

    void irq() {
        for (int i = 0; i < 20 && (s.read(otg::gintsts) & s.read(otg::gintmsk)) != 0; ++i) {
            d.on_irq();
        }
        EXPECT_EQ(s.read(otg::gintsts) & s.read(otg::gintmsk), 0u) << "interrupt left pending";
    }
    void bus_reset() {
        s.r[otg::gintsts] |= otg::gint_usbrst | otg::gint_enumdne;
        irq();
    }
    Handshake setup(const Bytes& packet) {
        EXPECT_EQ(packet.size(), 8u);
        s.r[otg::diepctl(0)] &= ~stall_bit;  // the core clears EP0's stall with a SETUP
        s.r[otg::doepctl(0)] &= ~stall_bit;
        s.in_pid[0] = 1;
        s.out_pid[0] = 1;
        std::uint32_t& tsiz = s.r[otg::doeptsiz(0)];
        const std::uint32_t stupcnt = tsiz >> 29;
        tsiz = (tsiz & ~(3u << 29)) | ((stupcnt > 0 ? stupcnt - 1 : 0u) << 29);
        s.rx.push_back({(6u << 17) | (8u << 4) | 0u, SimulatedOtg::words(packet)});
        s.rx.push_back({(4u << 17) | 0u, {}});
        irq();
        return Handshake::Ack;
    }
    Handshake out(unsigned ep, const Bytes& data) {
        const std::uint32_t ctl = s.get(otg::doepctl(ep));
        if ((ctl & stall_bit) != 0) return Handshake::Stall;
        if ((ctl & epena) == 0 || s.out_nak[ep]) return Handshake::Nak;
        EXPECT_LE(data.size(), s.get(otg::doeptsiz(ep)) & 0x7FFFFu) << "more than the endpoint was armed for";
        std::uint32_t& tsiz = s.r[otg::doeptsiz(ep)];
        const std::uint32_t pktcnt = (tsiz >> 19) & 0x3FF, xfrsiz = tsiz & 0x7FFFF;
        EXPECT_GE(pktcnt, 1u) << "OUT packet without a packet count";
        tsiz = (tsiz & ~((0x3FFu << 19) | 0x7FFFFu)) | ((pktcnt - 1) << 19) |
               (xfrsiz - static_cast<std::uint32_t>(data.size()));
        s.rx.push_back({(2u << 17) | (static_cast<std::uint32_t>(data.size()) << 4) | ep, SimulatedOtg::words(data)});
        s.rx.push_back({(3u << 17) | ep, {}});
        s.out_pid[ep] ^= 1;
        irq();
        return Handshake::Ack;
    }
    std::optional<Bytes> in(unsigned ep, Handshake* hs = nullptr, unsigned* pid = nullptr) {
        const auto set = [&](Handshake h) {
            if (hs != nullptr) *hs = h;
        };
        const std::uint32_t ctl = s.get(otg::diepctl(ep));
        if ((ctl & stall_bit) != 0) {
            set(Handshake::Stall);
            return std::nullopt;
        }
        if ((ctl & epena) == 0 || s.in_nak[ep]) {
            set(Handshake::Nak);
            return std::nullopt;
        }
        const std::uint32_t len = s.get(otg::dieptsiz(ep)) & 0x7FFFFu;
        EXPECT_EQ(s.get(otg::dieptsiz(ep)) >> 19 & 0x3FF, 1u) << "one packet";
        EXPECT_EQ(s.tx[ep].size(), (len + 3) / 4) << "FIFO holds exactly the packet";
        Bytes packet(len);
        for (std::size_t i = 0; i < len; i += 4) {
            const std::uint32_t w = s.tx[ep].front();
            s.tx[ep].pop_front();
            for (std::size_t b = 0; b < 4 && i + b < len; ++b) packet[i + b] = static_cast<std::uint8_t>(w >> (8 * b));
        }
        if (pid != nullptr) *pid = s.in_pid[ep];
        s.in_pid[ep] ^= 1;
        s.r[otg::diepctl(ep)] &= ~epena;
        s.in_nak[ep] = true;
        s.r[otg::diepint(ep)] |= otg::epint_xfrc;
        irq();
        set(Handshake::Ack);
        return packet;
    }
};

std::vector<usb::Event> drain(Driver& d) {
    std::vector<usb::Event> events;
    while (auto e = d.poll()) events.push_back(*e);
    return events;
}

class UsbOtgFsTest : public ::testing::Test {
protected:
    SimulatedOtg sim;
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

TEST(UsbOtgFs, StartConfiguresTheCoreAsAFullSpeedDevice) {
    SimulatedOtg sim;
    Driver drv{SimRegs{&sim}};
    drv.start();
    const std::uint32_t gusbcfg = sim.get(otg::gusbcfg);
    EXPECT_NE(gusbcfg & otg::gusbcfg_fdmod, 0u);
    EXPECT_EQ(gusbcfg & otg::gusbcfg_fhmod, 0u);
    EXPECT_NE(gusbcfg & otg::gusbcfg_physel, 0u);
    EXPECT_EQ((gusbcfg >> 10) & 0xF, 6u) << "TRDT for an AHB clock >= 32 MHz";
    EXPECT_NE(sim.get(otg::gccfg) & otg::gccfg_pwrdwn, 0u) << "PHY powered";
    EXPECT_EQ(sim.get(otg::gccfg) & otg::gccfg_vbden, 0u) << "no VBUS sensing";
    EXPECT_EQ(sim.get(otg::gotgctl) & 0xC0u, 0xC0u) << "B-session valid forced";
    EXPECT_EQ(sim.get(otg::dcfg) & 3u, 3u) << "full speed, embedded PHY";
    EXPECT_NE(sim.get(otg::dctl) & otg::dctl_sdis, 0u) << "soft-disconnected until connect()";
    EXPECT_NE(sim.get(otg::gahbcfg) & otg::gahbcfg_gint, 0u);
    const std::uint32_t mask = sim.get(otg::gintmsk);
    for (const std::uint32_t bit : {otg::gint_rxflvl, otg::gint_usbrst, otg::gint_enumdne, otg::gint_iepint,
                                    otg::gint_oepint, otg::gint_usbsusp, otg::gint_wkuint}) {
        EXPECT_NE(mask & bit, 0u) << bit;
    }
    // FIFO RAM: RX 128 words, then 16 words per TX FIFO - within the FS core's 320 words (1.25 KB)
    EXPECT_EQ(sim.get(otg::grxfsiz), 128u);
    EXPECT_EQ(sim.get(otg::dieptxf0), (16u << 16) | 128u);
    std::uint32_t end = 128 + 16;
    for (unsigned n = 1; n < otg::endpoints; ++n) {
        const std::uint32_t f = sim.get(otg::dieptxf(n));
        EXPECT_EQ(f & 0xFFFF, end) << n;
        end = (f & 0xFFFF) + (f >> 16);
    }
    EXPECT_LE(end, 320u);
    drv.connect();
    EXPECT_EQ(sim.get(otg::dctl) & otg::dctl_sdis, 0u);
    drv.disconnect();
    EXPECT_NE(sim.get(otg::dctl) & otg::dctl_sdis, 0u);
}

TEST_F(UsbOtgFsTest, BusResetPreparesEp0ForSetupPackets) {
    EXPECT_EQ(sim.get(otg::doeptsiz(0)), (3u << 29) | (1u << 19) | 24u) << "STUPCNT 3, PKTCNT 1, 3 x 8 bytes";
    EXPECT_EQ(sim.get(otg::daintmsk), 0x0001'0001u);
    EXPECT_EQ(sim.dad(), 0u);
    EXPECT_EQ(sim.get(otg::diepctl(0)) & 0x7FFu, 0u) << "EP0 MPSIZ 64 after the enumeration";
}

TEST_F(UsbOtgFsTest, SetupThenInDataStageWithAZeroLengthPacket) {
    const Bytes get_desc{0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00};
    ASSERT_EQ(host.setup(get_desc), Handshake::Ack);
    auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    ASSERT_EQ(ev[0].type, usb::EventType::Setup);
    EXPECT_EQ(std::to_integer<int>(ev[0].setup[1]), 0x06);
    EXPECT_EQ(std::to_integer<int>(ev[0].setup[6]), 0x40);
    std::vector<std::byte> data(64);
    for (std::size_t i = 0; i < data.size(); ++i) data[i] = std::byte(i);
    ASSERT_TRUE(drv.transfer(0x80, data, true));
    EXPECT_FALSE(drv.transfer(0x80, data, true)) << "busy";
    const auto p1 = host.in(0);
    ASSERT_TRUE(p1.has_value());
    ASSERT_EQ(p1->size(), 64u);
    EXPECT_EQ((*p1)[63], 63);
    EXPECT_TRUE(drain(drv).empty()) << "the zero-length packet is still to come";
    const auto p2 = host.in(0);
    ASSERT_TRUE(p2.has_value());
    EXPECT_TRUE(p2->empty());
    ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].type, usb::EventType::TransferComplete);
    EXPECT_EQ(ev[0].endpoint, 0x80);
    EXPECT_EQ(ev[0].length, 64);
    // status stage: the stack asks for the host's zero-length OUT
    Handshake hs{};
    EXPECT_EQ(host.out(0, {}), Handshake::Nak) << "not armed before the stack asks";
    ASSERT_TRUE(drv.transfer(0x00, {}, false));
    EXPECT_EQ(host.out(0, {}), Handshake::Ack);
    ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].endpoint, 0x00);
    EXPECT_EQ(ev[0].length, 0);
    EXPECT_EQ(sim.get(otg::doeptsiz(0)), (3u << 29) | (1u << 19) | 24u) << "prepared for the next SETUP (HAL's EP0_OutStart)";
    (void)hs;
}

TEST_F(UsbOtgFsTest, ShortInTransferNeedsNoZeroLengthPacket) {
    ASSERT_TRUE(drv.open({0x81, usb::TransferType::Bulk, 64}));
    std::vector<std::byte> data(100, std::byte{7});
    ASSERT_TRUE(drv.transfer(0x81, data, true));
    EXPECT_EQ(host.in(1)->size(), 64u);
    EXPECT_EQ(host.in(1)->size(), 36u);
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].endpoint, 0x81);
    EXPECT_EQ(ev[0].length, 100);
    Handshake hs{};
    EXPECT_FALSE(host.in(1, &hs).has_value());
    EXPECT_EQ(hs, Handshake::Nak) << "idle IN endpoint NAKs";
}

TEST_F(UsbOtgFsTest, BulkOutEndsWithAShortPacketAndNaksWhileIdle) {
    ASSERT_TRUE(drv.open({0x02, usb::TransferType::Bulk, 64}));
    EXPECT_EQ(host.out(2, {1}), Handshake::Nak);
    std::vector<std::byte> buffer(256);
    ASSERT_TRUE(drv.transfer(0x02, buffer, false));
    Bytes full(64);
    for (std::size_t i = 0; i < full.size(); ++i) full[i] = static_cast<std::uint8_t>(i + 1);
    EXPECT_EQ(host.out(2, full), Handshake::Ack);
    EXPECT_TRUE(drain(drv).empty());
    EXPECT_EQ(host.out(2, {0xAA, 0xBB, 0xCC}), Handshake::Ack);
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].endpoint, 0x02);
    EXPECT_EQ(ev[0].length, 67);
    EXPECT_EQ(std::to_integer<int>(buffer[0]), 1);
    EXPECT_EQ(std::to_integer<int>(buffer[63]), 64);
    EXPECT_EQ(std::to_integer<int>(buffer[66]), 0xCC);
    EXPECT_EQ(host.out(2, {1}), Handshake::Nak) << "idle again";
}

TEST_F(UsbOtgFsTest, OutTransferEndsWhenTheBufferIsFull) {
    ASSERT_TRUE(drv.open({0x02, usb::TransferType::Bulk, 64}));
    std::vector<std::byte> buffer(128);
    ASSERT_TRUE(drv.transfer(0x02, buffer, false));
    EXPECT_EQ(host.out(2, Bytes(64, 1)), Handshake::Ack);
    EXPECT_EQ(host.out(2, Bytes(64, 2)), Handshake::Ack);
    const auto ev = drain(drv);
    ASSERT_EQ(ev.size(), 1u);
    EXPECT_EQ(ev[0].length, 128);
}

TEST_F(UsbOtgFsTest, DataTogglesAlternateAndClearStallResetsThem) {
    ASSERT_TRUE(drv.open({0x81, usb::TransferType::Bulk, 64}));
    std::vector<std::byte> one(1);
    unsigned pid = 9;
    ASSERT_TRUE(drv.transfer(0x81, one, false));
    ASSERT_TRUE(host.in(1, nullptr, &pid).has_value());
    EXPECT_EQ(pid, 0u) << "DATA0 after open";
    (void)drain(drv);
    ASSERT_TRUE(drv.transfer(0x81, one, false));
    ASSERT_TRUE(host.in(1, nullptr, &pid).has_value());
    EXPECT_EQ(pid, 1u);
    (void)drain(drv);
    drv.stall(0x81);
    Handshake hs{};
    EXPECT_FALSE(host.in(1, &hs).has_value());
    EXPECT_EQ(hs, Handshake::Stall);
    drv.clear_stall(0x81);
    ASSERT_TRUE(drv.transfer(0x81, one, false));
    ASSERT_TRUE(host.in(1, nullptr, &pid).has_value());
    EXPECT_EQ(pid, 0u) << "DATA0 again after clear_stall";
}

TEST_F(UsbOtgFsTest, ControlStallEndsWithTheNextSetup) {
    ASSERT_EQ(host.setup({0x80, 0x06, 0x00, 0x09, 0x00, 0x00, 0x40, 0x00}), Handshake::Ack);
    (void)drain(drv);
    drv.stall(0x00);
    Handshake hs{};
    EXPECT_FALSE(host.in(0, &hs).has_value());
    EXPECT_EQ(hs, Handshake::Stall);
    ASSERT_EQ(host.setup({0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00}), Handshake::Ack);
    ASSERT_EQ(drain(drv).size(), 1u);
    std::vector<std::byte> d(18);
    ASSERT_TRUE(drv.transfer(0x80, d, true));
    EXPECT_EQ(host.in(0)->size(), 18u);
}

TEST_F(UsbOtgFsTest, SetAddressIsTakenBeforeTheStatusStage) {
    ASSERT_EQ(host.setup({0x00, 0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00}), Handshake::Ack);
    EXPECT_EQ(sim.dad(), 0x2Au) << "DCFG.DAD before the status IN (RM0433 device programming model)";
    ASSERT_EQ(drain(drv).size(), 1u);
    ASSERT_TRUE(drv.transfer(0x80, {}, false));
    const auto status = host.in(0);
    ASSERT_TRUE(status.has_value());
    EXPECT_TRUE(status->empty());
    drv.set_address(0x2A);  // the stack, after the status stage: no change
    EXPECT_EQ(sim.dad(), 0x2Au);
}

TEST_F(UsbOtgFsTest, OpenRefusesEp0AndTooLargePacketsAndCloseAllDisables) {
    EXPECT_FALSE(drv.open({0x80, usb::TransferType::Bulk, 64}));
    EXPECT_FALSE(drv.open({0x81, usb::TransferType::Bulk, 65}));
    EXPECT_FALSE(drv.open({0x88, usb::TransferType::Bulk, 64}));
    ASSERT_TRUE(drv.open({0x83, usb::TransferType::Interrupt, 16}));
    const std::uint32_t ctl = sim.get(otg::diepctl(3));
    EXPECT_EQ(ctl & 0x7FFu, 16u);
    EXPECT_EQ((ctl >> 18) & 3u, 3u) << "EPTYP interrupt";
    EXPECT_EQ((ctl >> 22) & 0xFu, 3u) << "TX FIFO 3";
    EXPECT_NE(ctl & (1u << 15), 0u) << "USBAEP";
    EXPECT_NE(sim.get(otg::daintmsk) & (1u << 3), 0u);
    std::vector<std::byte> d(4);
    ASSERT_TRUE(drv.transfer(0x83, d, false));
    drv.close_all();
    EXPECT_EQ(sim.get(otg::diepctl(3)) & epena, 0u);
    EXPECT_EQ(sim.get(otg::daintmsk), 0x0001'0001u);
    EXPECT_FALSE(drv.transfer(0x83, d, false)) << "closed";
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

TEST(UsbOtgFsStack, EnumeratesAndEchoesOverTheRegisterModel) {
    SimulatedOtg sim;
    Driver drv{SimRegs{&sim}};
    Loopback fn;
    std::array<usb::Function*, 1> functions{&fn};
    usb::Device<Driver> dev{drv, {device_desc, config_desc, strings}, functions};
    Host host{sim, drv};
    drv.start();
    dev.connect();
    EXPECT_EQ(sim.get(otg::dctl) & otg::dctl_sdis, 0u) << "connected";
    host.bus_reset();
    dev.service();
    // the host retries NAKed tokens; the stack runs in between
    const auto in_retry = [&](unsigned ep) -> std::optional<Bytes> {
        for (int i = 0; i < 5; ++i) {
            Handshake hs{};
            auto p = host.in(ep, &hs);
            if (hs != Handshake::Nak) return p;
            dev.service();
        }
        return std::nullopt;
    };
    const auto out_retry = [&](unsigned ep, const Bytes& data) {
        for (int i = 0; i < 5; ++i) {
            const Handshake hs = host.out(ep, data);
            if (hs != Handshake::Nak) return hs;
            dev.service();
        }
        return Handshake::Nak;
    };
    const auto control_in = [&](const Bytes& setup) {
        EXPECT_EQ(host.setup(setup), Handshake::Ack);
        dev.service();
        Bytes all;
        while (auto p = in_retry(0)) {
            all.insert(all.end(), p->begin(), p->end());
            dev.service();
            if (p->size() < 64) break;
        }
        EXPECT_EQ(out_retry(0, {}), Handshake::Ack) << "status stage";
        dev.service();
        return all;
    };
    const auto control_out = [&](const Bytes& setup) {
        EXPECT_EQ(host.setup(setup), Handshake::Ack);
        dev.service();
        const auto status = in_retry(0);
        EXPECT_TRUE(status.has_value() && status->empty()) << "zero-length status packet";
        dev.service();
    };
    const Bytes dd = control_in({0x80, 0x06, 0x00, 0x01, 0x00, 0x00, 0x40, 0x00});
    ASSERT_EQ(dd.size(), 18u);
    EXPECT_EQ(dd[0], 18);
    control_out({0x00, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00});  // SET_ADDRESS 7
    EXPECT_EQ(sim.dad(), 7u);
    const Bytes cd = control_in({0x80, 0x06, 0x00, 0x02, 0x00, 0x00, 0xFF, 0x00});
    EXPECT_EQ(cd.size(), config_desc.size());
    control_out({0x00, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00});  // SET_CONFIGURATION 1
    ASSERT_TRUE(dev.configured());
    EXPECT_EQ(out_retry(1, {1, 2, 3, 4, 5}), Handshake::Ack);
    dev.service();
    const auto echo = in_retry(1);
    ASSERT_TRUE(echo.has_value());
    EXPECT_EQ(*echo, (Bytes{1, 2, 3, 4, 5}));
    dev.service();
    EXPECT_EQ(out_retry(1, {9}), Handshake::Ack) << "re-armed after the echo";
    EXPECT_EQ(drv.stats().events_dropped, 0u);
}

}  // namespace
