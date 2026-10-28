// SPDX-License-Identifier: MIT
//
// STM32H5 flash interface (RM0481 chapter FLASH), the parts rtoos uses, for Renode. Compiled
// at run time by fcu.resc:
//   flashController: MTD.STM32H5_FLASH @ sysbus 0x40022000
//       flashBase: 0x08000000
//       flashSize: 0x200000
//
//   ACR          latency and WRHIGHFREQ (reset 0x13), a new latency applies at once
//   NSKEYR       the key sequence 0x45670123, 0xCDEF89AB clears NSCR.LOCK; a wrong key keeps it
//   NSCR         LOCK (set again by writing 1), PG, SER + SNB + BKSEL + STRT: the sector is
//                erased (8 KB of 0xFF in the flash memory), STRT clears itself
//   NSSR         never busy; PGSERR when SER/STRT is set while locked
//   NSCCR        clears the error flags
//   OPTSR_CUR    0: no bank swap
// The CPU programs the flash by writing to it; Renode's memory accepts the writes whether PG is
// set or not and does not enforce "program once after an erase" (a model gap: the driver's
// sequence and the erase are what this model checks).
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.MTD
{
    public class STM32H5_FLASH : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H5_FLASH(IMachine machine, ulong flashBase = 0x08000000, ulong flashSize = 0x200000)
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
            acr = 0x13;
            nscr = Lock;
            nssr = 0;
            keyStep = 0;
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x00: return acr;
                case 0x20: return nssr;
                case 0x28: return nscr;
                case 0x50: return 0;  // OPTSR_CUR: no SWAP_BANK
                default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case 0x00: acr = value & 0x3F; break;
                case 0x04:  // NSKEYR
                    if(keyStep == 0 && value == Key1)
                    {
                        keyStep = 1;
                    }
                    else if(keyStep == 1 && value == Key2)
                    {
                        nscr &= ~Lock;
                        keyStep = 0;
                    }
                    else
                    {
                        keyStep = 0;
                        this.Log(LogLevel.Warning, "wrong flash key 0x{0:X}", value);
                    }
                    break;
                case 0x28: WriteControl(value); break;
                case 0x30: nssr &= ~(value & 0xFF0000); break;  // NSCCR
                default:
                    this.Log(LogLevel.Debug, "write 0x{0:X} to offset 0x{1:X} ignored", value, offset);
                    break;
            }
        }

        private void WriteControl(uint value)
        {
            if((nscr & Lock) != 0)
            {
                if((value & Lock) == 0 && (value & (Ser | Strt | Pg)) != 0)
                {
                    nssr |= PgsErr;
                    this.Log(LogLevel.Warning, "NSCR written while locked (0x{0:X})", value);
                }
                return;
            }
            nscr = value & ~Strt;
            if((value & Ser) != 0 && (value & Strt) != 0)
            {
                var sector = (value >> 6) & 0x7F;
                var bank = (value & BkSel) != 0 ? 1UL : 0UL;
                var address = flashBase + bank * (flashSize / 2) + sector * SectorSize;
                var erased = new byte[SectorSize];
                for(var i = 0; i < erased.Length; i++)
                {
                    erased[i] = 0xFF;
                }
                machine.SystemBus.WriteBytes(erased, address);
                Erases++;
                nssr |= Eop;
                this.Log(LogLevel.Debug, "erased bank {0} sector {1} at 0x{2:X}", bank + 1, sector, address);
            }
        }

        private const uint Key1 = 0x45670123, Key2 = 0xCDEF89AB;
        private const uint Lock = 1u << 0, Pg = 1u << 1, Ser = 1u << 2, Strt = 1u << 5, BkSel = 1u << 31;
        private const uint Eop = 1u << 16, PgsErr = 1u << 18;
        private const uint SectorSize = 0x2000;

        private readonly IMachine machine;
        private readonly ulong flashBase, flashSize;
        private uint acr, nscr, nssr;
        private int keyStep;
    }
}
