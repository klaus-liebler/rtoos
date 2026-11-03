// SPDX-License-Identifier: MIT
//
// STM32H5 GPDMA1 (RM0481 chapter GPDMA; registers and bits: SVD STM32H563 v1.8), linear
// single-block transfers, for Renode. Compiled at run time by fcu.resc:
//   gpdma1: DMA.STM32H5_GPDMA @ sysbus 0x40020000
//       [0-7] -> nvic@[27-34]
//   tim3:
//       DmaUpdate -> gpdma1@81        // hardware request lines: input = GPDMA1_REQUEST_* number
//
// Renode has no GPDMA model. This one does what the rtoos driver (gpdma.hpp) uses:
//   * per channel CR (EN, RESET, TCIE, HTIE, DTEIE, ULEIE, USEIE, PRIO), SR (IDLEF, TCF, HTF,
//     ...), FCR, TR1 (SDW/DDW, SINC/DINC), TR2 (REQSEL, SWREQ), BR1 (BNDT), SAR, DAR, LLR
//   * a rising edge on request input n moves one data item for every enabled channel whose
//     REQSEL is n: read the source width at SAR, write it at DAR in the destination width
//     (PAM = 0: zero-extended or truncated), advance the incrementing addresses, BNDT minus the
//     source width; BNDT = half: HTF; BNDT = 0: TCF, the channel is idle and disabled
//   * SWREQ (memory to memory): the whole block moves when the channel is enabled
//   * a bus access to nothing (unmapped) sets DTEF and stops the channel
//   * channel interrupt n = SR flags & CR enables (bits 8..14); MISR shows them
// Not modelled: linked lists (LLR != 0 is logged), bursts, 2D addressing, triggers, suspend,
// security/privilege attributes, FIFO levels.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
    public class STM32H5_GPDMA : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput
    {
        public STM32H5_GPDMA(IMachine machine)
        {
            this.machine = machine;
            var irqs = new Dictionary<int, IGPIO>();
            for(var i = 0; i < ChannelCount; i++)
            {
                channel[i] = new Channel();
                irqs[i] = new GPIO();
            }
            Connections = irqs;
            Reset();
        }

        public long Size => 0x1000;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        /// Data items moved since reset, per channel (for tests and the monitor).
        public ulong Transfers(int ch) => channel[ch].Transfers;

        public void Reset()
        {
            foreach(var c in channel)
            {
                c.Reset();
            }
            requestLevel.Clear();
            UpdateIrqs();
        }

        public void OnGPIO(int number, bool value)
        {
            requestLevel.TryGetValue(number, out var was);
            requestLevel[number] = value;
            if(!value || was)
            {
                return;
            }
            for(var i = 0; i < ChannelCount; i++)
            {
                var c = channel[i];
                if(c.Enabled && !c.SoftwareRequest && c.Request == number)
                {
                    MoveOne(i);
                }
            }
            UpdateIrqs();
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset < 0x50)
            {
                switch(offset)
                {
                    case 0x0C: case 0x10:  // MISR, SMISR
                        uint misr = 0;
                        for(var i = 0; i < ChannelCount; i++)
                        {
                            if(channel[i].Interrupt) misr |= 1u << i;
                        }
                        return misr;
                    default: return 0;
                }
            }
            var c = ChannelOf(offset, out var reg);
            if(c == null)
            {
                return 0;
            }
            switch(reg)
            {
                case 0x50: return c.Lbar;
                case 0x5C: return 0;
                case 0x60: return c.Sr;
                case 0x64: return c.Cr;
                case 0x90: return c.Tr1;
                case 0x94: return c.Tr2;
                case 0x98: return c.Bndt;
                case 0x9C: return c.Sar;
                case 0xA0: return c.Dar;
                case 0xCC: return c.Llr;
                default:
                    this.Log(LogLevel.Warning, "read from unmodelled offset 0x{0:X}", offset);
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset < 0x50)
            {
                return;  // SECCFGR, PRIVCFGR, RCFGLOCKR: attributes not modelled
            }
            var c = ChannelOf(offset, out var reg);
            if(c == null)
            {
                return;
            }
            var index = (int)((offset - 0x50) / 0x80);
            switch(reg)
            {
                case 0x50: c.Lbar = value & 0xFFFF0000; break;
                case 0x5C: c.Sr &= ~(value & 0x7F00); break;  // FCR
                case 0x64: WriteCr(index, value); break;
                case 0x90: c.Tr1 = value; break;
                case 0x94: c.Tr2 = value; break;
                case 0x98: c.Bndt = value & 0xFFFF; c.Block = c.Bndt; break;
                case 0x9C: c.Sar = value; break;
                case 0xA0: c.Dar = value; break;
                case 0xCC:
                    c.Llr = value;
                    if((value & 0xFFFC) != 0) this.Log(LogLevel.Warning, "channel {0}: linked lists not modelled", index);
                    break;
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X} to unmodelled offset 0x{1:X}", value, offset);
                    break;
            }
            UpdateIrqs();
        }

        private void WriteCr(int index, uint value)
        {
            var c = channel[index];
            if((value & CrReset) != 0)
            {
                c.Cr = 0;
                c.Sr = SrIdle;
                return;
            }
            var enable = (value & CrEn) != 0 && (c.Cr & CrEn) == 0;
            c.Cr = value & ~CrReset;
            if(!enable)
            {
                return;
            }
            if(c.Bndt == 0)
            {
                c.Sr |= SrUse;  // user setting error: nothing to move
                c.Cr &= ~CrEn;
                return;
            }
            c.Sr &= ~SrIdle;
            if(c.SoftwareRequest)
            {
                while(c.Enabled)
                {
                    MoveOne(index);
                }
            }
        }

        private void MoveOne(int index)
        {
            var c = channel[index];
            var sdw = 1u << (int)(c.Tr1 & 3);
            var ddw = 1u << (int)((c.Tr1 >> 16) & 3);
            var bus = machine.GetSystemBus(this);
            if(bus.WhatIsAt(c.Sar) == null || bus.WhatIsAt(c.Dar) == null)
            {
                c.Sr |= SrDte | SrIdle;
                c.Cr &= ~CrEn;
                this.Log(LogLevel.Warning, "channel {0}: transfer error at 0x{1:X} -> 0x{2:X}", index, c.Sar, c.Dar);
                return;
            }
            ulong data;
            switch(sdw)
            {
                case 1: data = bus.ReadByte(c.Sar); break;
                case 2: data = bus.ReadWord(c.Sar); break;
                default: data = bus.ReadDoubleWord(c.Sar); break;
            }
            switch(ddw)
            {
                case 1: bus.WriteByte(c.Dar, (byte)data); break;
                case 2: bus.WriteWord(c.Dar, (ushort)data); break;
                default: bus.WriteDoubleWord(c.Dar, (uint)data); break;
            }
            ++c.Transfers;
            if((c.Tr1 & Sinc) != 0) c.Sar += sdw;
            if((c.Tr1 & Dinc) != 0) c.Dar += ddw;
            c.Bndt = c.Bndt > sdw ? c.Bndt - sdw : 0;
            if(c.Block >= 2 && c.Bndt == c.Block / 2)
            {
                c.Sr |= SrHt;
            }
            if(c.Bndt == 0)
            {
                c.Sr |= SrTc | SrIdle;
                c.Cr &= ~CrEn;
            }
        }

        private Channel ChannelOf(long offset, out long reg)
        {
            var index = (offset - 0x50) / 0x80;
            reg = (offset - 0x50) % 0x80 + 0x50;
            if(index < 0 || index >= ChannelCount)
            {
                this.Log(LogLevel.Warning, "access to unmodelled offset 0x{0:X}", offset);
                return null;
            }
            return channel[index];
        }

        private void UpdateIrqs()
        {
            for(var i = 0; i < ChannelCount; i++)
            {
                Connections[i].Set(channel[i].Interrupt);
            }
        }

        private class Channel
        {
            public uint Lbar, Sr, Cr, Tr1, Tr2, Bndt, Block, Sar, Dar, Llr;
            public ulong Transfers;
            public bool Enabled => (Cr & CrEn) != 0;
            public bool SoftwareRequest => (Tr2 & (1u << 9)) != 0;
            public int Request => (int)(Tr2 & 0x7F);
            public bool Interrupt => (Sr & Cr & 0x7F00) != 0;

            public void Reset()
            {
                Lbar = Cr = Tr1 = Tr2 = Bndt = Block = Sar = Dar = Llr = 0;
                Sr = SrIdle;
                Transfers = 0;
            }
        }

        private const int ChannelCount = 8;
        private const uint CrEn = 1u << 0, CrReset = 1u << 1;
        private const uint SrIdle = 1u << 0, SrTc = 1u << 8, SrHt = 1u << 9, SrDte = 1u << 10, SrUse = 1u << 12;
        private const uint Sinc = 1u << 3, Dinc = 1u << 19;

        private readonly IMachine machine;
        private readonly Channel[] channel = new Channel[ChannelCount];
        private readonly Dictionary<int, bool> requestLevel = new Dictionary<int, bool>();
    }
}
