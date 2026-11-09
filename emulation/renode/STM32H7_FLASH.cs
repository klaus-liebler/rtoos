// SPDX-License-Identifier: MIT
//
// STM32H743 flash interface (RM0433 chapter "Embedded flash memory"), the parts rtoos uses, for
// Renode. Compiled at run time by stm32h743.resc / fcu_rev3.resc:
//   flashInterface: MTD.STM32H7_FLASH @ sysbus 0x52002000
//       flashBase: 0x08000000
//       flashSize: 0x200000
//
//   ACR          latency and WRHIGHFREQ (reset 0x37), a new latency reads back at once
//   KEYR1/2      the key sequence 0x45670123, 0xCDEF89AB clears CRx.LOCK of that bank; a wrong
//                key keeps it locked
//   CR1/2        LOCK (set again by writing 1), PG, SER + SNB + START: the bank's sector is erased
//                (128 KB of 0xFF in the flash memory), START clears itself
//   SR1/2        never busy (BSY, WBNE, QW stay 0); EOP after an erase; PGSERR when SER/START/PG
//                is set while locked
//   CCR1/2       clears the flags
//   OPTSR_CUR    0: no bank swap
// Bank 1's registers are at 0x000.., bank 2's at +0x100. The CPU programs the flash by writing to
// it; Renode's memory accepts the writes whether PG is set or not and does not enforce "program
// a flash word once after an erase" (a model gap: the driver's sequence and the erase are what
// this model checks).
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.MTD
{
    public class STM32H7_FLASH : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H7_FLASH(IMachine machine, ulong flashBase = 0x08000000, ulong flashSize = 0x200000)
        {
            this.machine = machine;
            this.flashBase = flashBase;
            this.flashSize = flashSize;
            Reset();
        }

        public long Size => 0x400;
        public int Erases { get; private set; }

        public void Reset()
        {
            acr = 0x37;
            for(var b = 0; b < 2; b++)
            {
                cr[b] = Lock;
                sr[b] = 0;
                keyStep[b] = 0;
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == 0x00)
            {
                return acr;
            }
            if(offset == 0x1C)
            {
                return 0;  // OPTSR_CUR: no SWAP_BANK
            }
            var bank = offset >= 0x100 ? 1 : 0;
            switch(offset & 0xFF)
            {
                case 0x0C: return cr[bank];
                case 0x10: return sr[bank];
                default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0x00)
            {
                acr = value & 0x3F;
                return;
            }
            var bank = offset >= 0x100 ? 1 : 0;
            switch(offset & 0xFF)
            {
                case 0x04: WriteKey(bank, value); break;
                case 0x0C: WriteControl(bank, value); break;
                case 0x14: sr[bank] &= ~(value & 0x1FEF0000u); break;  // CCR
                default:
                    this.Log(LogLevel.Debug, "write 0x{0:X} to offset 0x{1:X} ignored", value, offset);
                    break;
            }
        }

        private void WriteKey(int bank, uint value)
        {
            if(keyStep[bank] == 0 && value == Key1)
            {
                keyStep[bank] = 1;
            }
            else if(keyStep[bank] == 1 && value == Key2)
            {
                cr[bank] &= ~Lock;
                keyStep[bank] = 0;
            }
            else
            {
                keyStep[bank] = 0;
                this.Log(LogLevel.Warning, "bank {0}: wrong flash key 0x{1:X}", bank + 1, value);
            }
        }

        private void WriteControl(int bank, uint value)
        {
            if((cr[bank] & Lock) != 0)
            {
                if((value & Lock) == 0 && (value & (Ser | Start | Pg)) != 0)
                {
                    sr[bank] |= PgsErr;
                    this.Log(LogLevel.Warning, "bank {0}: CR written while locked (0x{1:X})", bank + 1, value);
                }
                return;
            }
            cr[bank] = value & ~Start;
            if((value & Ser) != 0 && (value & Start) != 0)
            {
                var sector = (value >> 8) & 0x7;
                var address = flashBase + (ulong)bank * (flashSize / 2) + sector * SectorSize;
                var erased = new byte[SectorSize];
                for(var i = 0; i < erased.Length; i++)
                {
                    erased[i] = 0xFF;
                }
                machine.SystemBus.WriteBytes(erased, address);
                Erases++;
                sr[bank] |= Eop;
                this.Log(LogLevel.Debug, "erased bank {0} sector {1} at 0x{2:X}", bank + 1, sector, address);
            }
        }

        private const uint Key1 = 0x45670123, Key2 = 0xCDEF89AB;
        private const uint Lock = 1u << 0, Pg = 1u << 1, Ser = 1u << 2, Start = 1u << 7;
        private const uint Eop = 1u << 16, PgsErr = 1u << 18;
        private const uint SectorSize = 0x20000;

        private readonly IMachine machine;
        private readonly ulong flashBase, flashSize;
        private uint acr;
        private readonly uint[] cr = new uint[2];
        private readonly uint[] sr = new uint[2];
        private readonly int[] keyStep = new int[2];
    }
}
