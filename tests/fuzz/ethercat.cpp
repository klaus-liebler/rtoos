// SPDX-License-Identifier: MIT
// AFL++ harness: the EtherCAT slave (rtos/ethercat: AL state machine and its checks, SII EEPROM
// emulation, the mailbox, CoE SDO + SDO information, PDO assignment, process data, watchdog) against
// the simulated ESC of the host tests, driven by a hostile master. The master can write anything
// into the ESC (SyncManager configurations with any start and length included); the slave must
// never read or write past its own buffers.
//
// Input: a sequence of master operations; after each one the slave polls (an ESC interrupt).
//   0 a16 n8 data[n]                  write ESC memory
//   1 s8                              AL control (bit 4: error acknowledge)
//   2 sm8 start16 len16 ctrl8 en8     configure a SyncManager (sm 0..7)
//   3 type8 cnt8 n8 data[n]           mailbox write into SM0 (header: length n)
//   4                                 mailbox read (SM1)
//   5 n8 data[n]                      process data outputs (SM2)
//   6                                 process data inputs read (SM3)
//   7                                 the watchdog expires
//   8 cmd8 addr32                     EEPROM command
//   9                                 mailbox repeat request
// all multi-byte values little endian; an operation cut off by the end of the input ends it.
#include <cstddef>

#include "ethercat_fixture.hpp"
#include "fuzz_main.hpp"

namespace {

namespace ec = rtos::ethercat;
using namespace ecat_fixture;

struct Reader {
    std::span<const std::uint8_t> in;
    std::size_t pos = 0;
    bool ok = true;
    std::uint8_t u8() {
        if (pos >= in.size()) {
            ok = false;
            return 0;
        }
        return in[pos++];
    }
    std::uint16_t u16() { return static_cast<std::uint16_t>(u8() | (u8() << 8)); }
    std::uint32_t u32() { return u16() | (std::uint32_t{u16()} << 16); }
    std::span<const std::byte> bytes(std::size_t n) {
        n = std::min(n, in.size() - pos);
        const auto s = std::as_bytes(in.subspan(pos, n));
        pos += n;
        return s;
    }
};

}  // namespace

void fuzz_one(std::span<const std::uint8_t> input) {
    ecat_sim::SimEsc esc;
    App app;
    Bank bank;
    ec::Slave<ecat_sim::SimEsc, App, 256> slave{esc, image, layout, od, bank, app};
    slave.poll(true);
    Reader r{input};
    for (int ops = 0; ops < 512 && r.ok && r.pos < input.size(); ++ops) {
        switch (r.u8() % 10) {
            case 0: {
                const std::uint16_t a = r.u16();
                const auto d = r.bytes(r.u8());
                if (r.ok) esc.master_write(a, d);
                break;
            }
            case 1: {
                const std::uint8_t s = r.u8();
                if (r.ok) esc.master_write16(ec::reg::al_control, s);
                break;
            }
            case 2: {
                const unsigned n = r.u8() % 8;
                const std::uint16_t start = r.u16(), len = r.u16();
                const std::uint8_t ctrl = r.u8(), en = r.u8();
                if (r.ok) esc.configure_sm(n, start, len, ctrl, (en & 1) != 0);
                break;
            }
            case 3: {
                const std::uint8_t type = r.u8(), cnt = r.u8();
                const auto d = r.bytes(r.u8());
                // the simulated master writes header + data into SM0's buffer: only what fits (the
                // slave sees any length anyway - op 0 writes the header's length field freely)
                const std::size_t sm0_length = esc.get16(ec::reg::sm_base + 2);
                if (r.ok && sm0_length >= 6 + d.size()) (void)esc.mailbox_write(type & 0x0F, cnt & 0x07, d);
                break;
            }
            case 4: (void)esc.mailbox_read(); break;
            case 5: {
                const auto d = r.bytes(r.u8());
                if (r.ok) esc.write_outputs(d);
                break;
            }
            case 6: (void)esc.read_inputs(); break;
            case 7: esc.expire_watchdog(); break;
            case 8: {
                const std::uint8_t cmd = r.u8();
                const std::uint32_t a = r.u32();
                if (r.ok) esc.eeprom_command(cmd & 0x07, a, (cmd & 0x80) != 0);
                break;
            }
            default: esc.toggle_repeat_request(); break;
        }
        slave.poll(true);
    }
}
