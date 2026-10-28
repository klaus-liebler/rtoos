// SPDX-License-Identifier: MIT
//
// STM32H5 SDMMC (SDMMC v2, RM0481) with an SD memory card, for Renode. Compiled at run time:
//   include @emulation/renode/STM32H5_SDMMC.cs
//   sdmmc1: SD.STM32H5_SDMMC @ sysbus 0x46008000
//   sysbus.sdmmc1 InsertCard @card.img true       # image file, high capacity (SDHC) or not
//
// Why an own model: Renode's STM32 SDMMC model moves data only through the FIFO (IDMA is not
// modelled) and its SD card answers CMD8 with the card status instead of R7 (no echo of the
// check pattern). This model implements what the rtoos driver uses, as the hardware and the
// SD Physical Layer Simplified Specification describe it:
//   * registers POWER, CLKCR, ARGR, CMDR, RESPCMDR, RESP1-4, DTIMER, DLENR, DCTRL, DCNTR, STAR,
//     ICR, MASKR, IDMACTRLR, IDMABASER; a command executes when CMDR is written with CPSMEN,
//     its data (CMDTRANS) moves at once by IDMA between memory and the card image
//   * the card: states idle, ready, ident, stby, tran, data, rcv, prg; CMD0, CMD2, CMD3, CMD7,
//     CMD8 (R7 echoes the check pattern), CMD9 (CSD 2.0 for SDHC, 1.0 for SDSC), CMD12, CMD13,
//     CMD16, CMD17/18, CMD24/25, CMD55, ACMD41 (busy for two calls, then ready with CCS);
//     block (SDHC) or byte (SDSC) addresses; after a write the card stays in "prg" for one
//     CMD13; an unknown or out-of-state command gets no response (command timeout)
//   * D0 is never held busy (STAR.BUSYD0 = 0); writes go straight to the image file
using System;
using System.IO;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.SD
{
    public class STM32H5_SDMMC : IDoubleWordPeripheral, IKnownSize, IDisposable
    {
        public STM32H5_SDMMC(IMachine machine)
        {
            this.machine = machine;
            IRQ = new GPIO();
            Reset();
        }

        public long Size => 0x400;
        public GPIO IRQ { get; }

        /// Insert a card backed by `imagePath` (its size is the capacity, a multiple of 512).
        public void InsertCard(string imagePath, bool highCapacity)
        {
            image?.Dispose();
            image = new FileStream(imagePath, FileMode.Open, FileAccess.ReadWrite);
            this.highCapacity = highCapacity;
            blocks = image.Length / BlockSize;
            cardState = State.Idle;
            this.Log(LogLevel.Info, "SD card {0}: {1} blocks, {2}", imagePath, blocks, highCapacity ? "SDHC" : "SDSC");
        }

        public void Dispose()
        {
            image?.Dispose();
        }

        public void Reset()
        {
            power = clkcr = argr = cmdr = respcmd = dtimer = dlen = dctrl = dcount = star = mask = idmactrl = idmabase = 0;
            resp = new uint[4];
            cardState = State.Idle;
            appCommand = false;
            acmd41Calls = 0;
            UpdateIrq();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x00: return power;
                case 0x04: return clkcr;
                case 0x08: return argr;
                case 0x0C: return cmdr;
                case 0x10: return respcmd;
                case 0x14: return resp[0];
                case 0x18: return resp[1];
                case 0x1C: return resp[2];
                case 0x20: return resp[3];
                case 0x24: return dtimer;
                case 0x28: return dlen;
                case 0x2C: return dctrl;
                case 0x30: return dcount;
                case 0x34: return star;  // CPSMACT, DPSMACT, BUSYD0: never active
                case 0x3C: return mask;
                case 0x50: return idmactrl;
                case 0x58: return idmabase;
                default:
                    this.Log(LogLevel.Warning, "read from unmodelled offset 0x{0:X}", offset);
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case 0x00: power = value & 0x1F; break;
                case 0x04: clkcr = value; break;
                case 0x08: argr = value; break;
                case 0x0C:
                    cmdr = value;
                    if((value & (1u << 12)) != 0)  // CPSMEN
                    {
                        Execute();
                    }
                    break;
                case 0x24: dtimer = value; break;
                case 0x28: dlen = value & 0x1FFFFFF; break;
                case 0x2C: dctrl = value & 0x3FFF; break;
                case 0x38: star &= ~(value & IcrMask); UpdateIrq(); break;
                case 0x3C: mask = value; UpdateIrq(); break;
                case 0x50: idmactrl = value & 0x7; break;
                case 0x58: idmabase = value & ~3u; break;
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X} to unmodelled offset 0x{1:X}", value, offset);
                    break;
            }
        }

        // ---- command path ------------------------------------------------------------------

        private void Execute()
        {
            var index = cmdr & 0x3F;
            var waitresp = (cmdr >> 8) & 3;
            var transfer = (cmdr & (1u << 6)) != 0;
            var app = appCommand;
            appCommand = false;
            var response = new uint[4];
            var answered = image != null && (power & 3) == 3 && Respond(index, argr, app, out response);
            if(!answered)
            {
                response = new uint[4];
            }
            respcmd = index;
            resp = response;
            if(waitresp == 0)
            {
                star |= CmdSent;
            }
            else if(!answered)
            {
                star |= CTimeout;
            }
            else
            {
                star |= CmdREnd;
                if(transfer)
                {
                    Transfer(index);
                }
            }
            UpdateIrq();
        }

        private bool Respond(uint index, uint arg, bool app, out uint[] response)
        {
            response = new uint[4];
            var status = Status(app);
            switch(app ? 100 + index : index)
            {
                case 0:
                    cardState = State.Idle;
                    acmd41Calls = 0;
                    return true;
                case 8:  // SEND_IF_COND: R7 = voltage accepted (11:8) + check pattern (7:0)
                    if(cardState != State.Idle || ((arg >> 8) & 0xF) != 1)
                    {
                        return false;
                    }
                    response[0] = arg & 0xFFF;
                    return true;
                case 55:
                    appCommand = true;
                    response[0] = Status(true);
                    return true;
                case 141:  // ACMD41: busy for two calls, then ready
                    if(cardState != State.Idle)
                    {
                        return false;
                    }
                    if(++acmd41Calls < 3)
                    {
                        response[0] = Ocr;
                        return true;
                    }
                    cardState = State.Ready;
                    response[0] = Ocr | (1u << 31) | (highCapacity && (arg & (1u << 30)) != 0 ? 1u << 30 : 0);
                    return true;
                case 2:
                    if(cardState != State.Ready)
                    {
                        return false;
                    }
                    cardState = State.Ident;
                    response = new uint[] { 0x03534454, 0x52544F53, 0x01000000, 0x00012300 };  // "SD" "TRTOS"
                    return true;
                case 3:
                    if(cardState != State.Ident && cardState != State.Stby)
                    {
                        return false;
                    }
                    cardState = State.Stby;
                    response[0] = (Rca << 16) | ((uint)State.Stby << 9);
                    return true;
                case 9:
                    if(cardState != State.Stby || arg >> 16 != Rca)
                    {
                        return false;
                    }
                    response = Csd();
                    return true;
                case 7:
                    if(arg >> 16 == Rca && cardState == State.Stby)
                    {
                        cardState = State.Tran;
                    }
                    else if(arg >> 16 != Rca && cardState == State.Tran)
                    {
                        cardState = State.Stby;
                    }
                    response[0] = status;
                    return true;
                case 16:
                    if(cardState != State.Tran)
                    {
                        return false;
                    }
                    response[0] = status | (arg != BlockSize ? BlockLenError : 0);
                    return true;
                case 13:
                    if(arg >> 16 != Rca)
                    {
                        return false;
                    }
                    response[0] = status;
                    if(cardState == State.Prg)
                    {
                        cardState = State.Tran;  // programming takes one status poll
                    }
                    return true;
                case 17:
                case 18:
                case 24:
                case 25:
                    if(cardState != State.Tran)
                    {
                        return false;
                    }
                    response[0] = status;
                    return true;
                case 12:
                    response[0] = status;
                    cardState = cardState == State.Rcv ? State.Prg : State.Tran;
                    return true;
                default:
                    this.Log(LogLevel.Warning, "unsupported command {0}{1}", app ? "ACMD" : "CMD", index);
                    return false;
            }
        }

        // ---- data path (IDMA) ---------------------------------------------------------------

        private void Transfer(uint index)
        {
            var toHost = (dctrl & 2) != 0;  // DTDIR
            var multi = index == 18 || index == 25;
            var reading = index == 17 || index == 18;
            if(reading != toHost || (idmactrl & 1) == 0 || dlen == 0 || dlen % BlockSize != 0)
            {
                this.Log(LogLevel.Warning, "CMD{0}: unsupported data setup (DTDIR {1}, IDMA {2}, length {3})", index, toHost, idmactrl & 1, dlen);
                star |= DTimeout;
                return;
            }
            if(!multi && dlen != BlockSize)
            {
                star |= DTimeout;
                return;
            }
            var offset = highCapacity ? (long)argr * BlockSize : argr;
            if(!highCapacity && argr % BlockSize != 0 || offset + dlen > image.Length)
            {
                star |= DTimeout;  // address error: the card sends no data
                return;
            }
            var data = new byte[dlen];
            image.Seek(offset, SeekOrigin.Begin);
            if(reading)
            {
                image.Read(data, 0, data.Length);
                machine.SystemBus.WriteBytes(data, idmabase);
                cardState = multi ? State.Data : State.Tran;
            }
            else
            {
                machine.SystemBus.ReadBytes(idmabase, data.Length, data, 0);
                image.Write(data, 0, data.Length);
                image.Flush();
                cardState = multi ? State.Rcv : State.Prg;
            }
            dcount = 0;
            star |= DataEnd | DBckEnd;
        }

        // ---- card registers ----------------------------------------------------------------

        private uint Status(bool app)
        {
            var ready = cardState == State.Tran ? ReadyForData : 0;
            return ((uint)cardState << 9) | ready | (app ? AppCmd : 0);
        }

        private uint[] Csd()
        {
            var r = new uint[4];  // [0] = bits 127:96
            if(highCapacity)
            {
                Set(r, 127, 126, 1);
                Set(r, 83, 80, 9);
                Set(r, 69, 48, (uint)(blocks / 1024 - 1));
            }
            else
            {
                Set(r, 83, 80, 9);   // READ_BL_LEN 512
                Set(r, 49, 47, 7);   // C_SIZE_MULT: x 512
                Set(r, 73, 62, (uint)(blocks / 512 - 1));
            }
            Set(r, 0, 0, 1);  // stop bit
            return r;
        }

        private static void Set(uint[] r, int hi, int lo, uint value)
        {
            for(var b = lo; b <= hi; b++)
            {
                var bit = 1u << (b % 32);
                var word = 3 - b / 32;
                r[word] = ((value >> (b - lo)) & 1) != 0 ? r[word] | bit : r[word] & ~bit;
            }
        }

        private void UpdateIrq()
        {
            IRQ.Set((star & mask) != 0);
        }

        private enum State : uint { Idle = 0, Ready = 1, Ident = 2, Stby = 3, Tran = 4, Data = 5, Rcv = 6, Prg = 7 }

        private const uint Ocr = 0x00FF8000;  // 2.7 - 3.6 V
        private const uint Rca = 0xAAAA;
        private const int BlockSize = 512;
        private const uint ReadyForData = 1u << 8, AppCmd = 1u << 5, BlockLenError = 1u << 29;
        private const uint CTimeout = 1u << 2, DTimeout = 1u << 3, CmdREnd = 1u << 6, CmdSent = 1u << 7,
                           DataEnd = 1u << 8, DBckEnd = 1u << 10;
        private const uint IcrMask = 0x1FE00FFF;

        private readonly IMachine machine;
        private FileStream image;
        private bool highCapacity;
        private long blocks;
        private State cardState;
        private bool appCommand;
        private int acmd41Calls;
        private uint power, clkcr, argr, cmdr, respcmd, dtimer, dlen, dctrl, dcount, star, mask, idmactrl, idmabase;
        private uint[] resp;
    }
}
