// SPDX-License-Identifier: MIT
//
// STM32F1 DMA1 (RM0008 rev 21 chapter 13), seven channels, for Renode. Compiled at run time by
// stm32f103.resc:
//   dma1: DMA.STM32F1_DMA @ sysbus 0x40020000
//       [0-6] -> nvic@[11-17]
//   tim1:
//       DmaUpdate -> dma1@5            // request input = channel number (RM0008 table 78)
//
// Why an own model: Renode's STM32G0DMA (the F1 register layout) moves the whole block as soon as
// a channel is enabled instead of one data item per peripheral request, and counts the block in
// bytes instead of data items (a block of 121 half words throws). rtoos feeds timers item by item
// (WS2812 bits into CCR1 per update event), so the model does what the hardware does:
//   * ISR (GIF, TCIF, HTIF, TEIF per channel), IFCR (write 1 to clear; CGIFx clears all four),
//     per channel CCR (EN, TCIE, HTIE, TEIE, DIR, CIRC, PINC, MINC, PSIZE, MSIZE, PL, MEM2MEM),
//     CNDTR, CPAR, CMAR at 0x08 + 0x14 (channel - 1)
//   * a rising edge on request input n moves one data item for channel n if it is enabled (and not
//     memory to memory): read the source width, write the destination width (zero-extended or
//     truncated, RM0008 table 79), advance the incrementing addresses, CNDTR - 1; CNDTR = half:
//     HTIF; CNDTR = 0: TCIF and - CIRC - the reload of CNDTR and the start addresses, else the
//     channel stays enabled and idle (the hardware's behaviour: EN is not cleared)
//   * MEM2MEM: the whole block moves when the channel is enabled
//   * a bus access to nothing (unmapped) sets TEIF and clears EN
//   * channel interrupt n = flags & enables
// Not modelled: priorities between simultaneous requests (requests are handled as they come).
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.DMA
{
    public class STM32F1_DMA : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput
    {
        public STM32F1_DMA(IMachine machine, int channels = 7)
        {
            this.machine = machine;
            channel = new Channel[channels];
            var irqs = new Dictionary<int, IGPIO>();
            for(var i = 0; i < channels; i++)
            {
                channel[i] = new Channel();
                irqs[i] = new GPIO();
            }
            Connections = irqs;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        /// Data items moved by channel n (1..7) since reset.
        public ulong Transfers(int n) => channel[n - 1].Transfers;

        public void Reset()
        {
            foreach(var c in channel)
            {
                c.Ccr = c.Cndtr = c.Cpar = c.Cmar = c.Flags = 0;
                c.Transfers = 0;
            }
            requestLevel.Clear();
            UpdateIrqs();
        }

        public void OnGPIO(int number, bool value)
        {
            requestLevel.TryGetValue(number, out var was);
            requestLevel[number] = value;
            if(!value || was || number < 1 || number > channel.Length)
            {
                return;
            }
            var c = channel[number - 1];
            if(c.Enabled && (c.Ccr & Mem2Mem) == 0 && c.Remaining != 0)
            {
                MoveOne(c);
                UpdateIrqs();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            if(offset == 0x00)
            {
                uint isr = 0;
                for(var i = 0; i < channel.Length; i++)
                {
                    var f = channel[i].Flags & 0xE;
                    isr |= (f | (f != 0 ? 1u : 0u)) << (4 * i);
                }
                return isr;
            }
            var c = ChannelOf(offset, out var reg);
            if(c == null)
            {
                return 0;
            }
            switch(reg)
            {
                case 0x0: return c.Ccr;
                case 0x4: return c.Remaining;
                case 0x8: return c.Cpar;
                case 0xC: return c.Cmar;
            }
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset == 0x04)  // IFCR
            {
                for(var i = 0; i < channel.Length; i++)
                {
                    var bits = (value >> (4 * i)) & 0xF;
                    if((bits & 1) != 0)
                    {
                        bits |= 0xE;  // CGIF clears all flags of the channel
                    }
                    channel[i].Flags &= ~(bits & 0xE);
                }
                UpdateIrqs();
                return;
            }
            var c = ChannelOf(offset, out var reg);
            if(c == null)
            {
                this.Log(LogLevel.Warning, "write 0x{0:X} to 0x{1:X}: no register", value, offset);
                return;
            }
            switch(reg)
            {
                case 0x0:
                    var enable = (value & En) != 0 && !c.Enabled;
                    c.Ccr = value & 0x7FFF;
                    if(enable)
                    {
                        c.Remaining = c.Cndtr;
                        c.Sar = c.Cmar;
                        c.Par = c.Cpar;
                        if((c.Ccr & Mem2Mem) != 0)
                        {
                            while(c.Enabled && c.Remaining != 0)
                            {
                                MoveOne(c);
                            }
                        }
                    }
                    break;
                // CNDTR, CPAR, CMAR are writable while the channel is disabled (RM0008 13.4.4-13.4.6)
                case 0x4: if(!c.Enabled) { c.Cndtr = value & 0xFFFF; c.Remaining = c.Cndtr; } break;
                case 0x8: if(!c.Enabled) c.Cpar = value; break;
                case 0xC: if(!c.Enabled) c.Cmar = value; break;
            }
            UpdateIrqs();
        }

        private void MoveOne(Channel c)
        {
            var psize = 1u << (int)((c.Ccr >> 8) & 3);
            var msize = 1u << (int)((c.Ccr >> 10) & 3);
            var toPeripheral = (c.Ccr & Dir) != 0;
            var src = toPeripheral ? c.Sar : c.Par;
            var dst = toPeripheral ? c.Par : c.Sar;
            var srcSize = toPeripheral ? msize : psize;
            var dstSize = toPeripheral ? psize : msize;
            var bus = machine.GetSystemBus(this);
            if(bus.WhatIsAt(src) == null || bus.WhatIsAt(dst) == null)
            {
                c.Flags |= Teif;
                c.Ccr &= ~En;
                this.Log(LogLevel.Warning, "transfer error at 0x{0:X} -> 0x{1:X}", src, dst);
                return;
            }
            ulong data;
            switch(srcSize)
            {
                case 1: data = bus.ReadByte(src); break;
                case 2: data = bus.ReadWord(src); break;
                default: data = bus.ReadDoubleWord(src); break;
            }
            switch(dstSize)
            {
                case 1: bus.WriteByte(dst, (byte)data); break;
                case 2: bus.WriteWord(dst, (ushort)data); break;
                default: bus.WriteDoubleWord(dst, (uint)data); break;
            }
            ++c.Transfers;
            if((c.Ccr & Minc) != 0) c.Sar += msize;
            if((c.Ccr & Pinc) != 0) c.Par += psize;
            --c.Remaining;
            if(c.Cndtr >= 2 && c.Remaining == c.Cndtr / 2)
            {
                c.Flags |= Htif;
            }
            if(c.Remaining == 0)
            {
                c.Flags |= Tcif;
                if((c.Ccr & Circ) != 0)
                {
                    c.Remaining = c.Cndtr;
                    c.Sar = c.Cmar;
                    c.Par = c.Cpar;
                }
            }
        }

        private Channel ChannelOf(long offset, out long reg)
        {
            var index = (offset - 0x08) / 0x14;
            reg = (offset - 0x08) % 0x14;
            if(offset < 0x08 || index >= channel.Length || reg > 0xC)
            {
                return null;
            }
            return channel[index];
        }

        private void UpdateIrqs()
        {
            for(var i = 0; i < channel.Length; i++)
            {
                var c = channel[i];
                var irq = ((c.Flags & Tcif) != 0 && (c.Ccr & Tcie) != 0) || ((c.Flags & Htif) != 0 && (c.Ccr & Htie) != 0) ||
                          ((c.Flags & Teif) != 0 && (c.Ccr & Teie) != 0);
                Connections[i].Set(irq);
            }
        }

        private class Channel
        {
            public uint Ccr, Cndtr, Cpar, Cmar, Flags, Remaining, Sar, Par;
            public ulong Transfers;
            public bool Enabled => (Ccr & En) != 0;
        }

        private const uint En = 1u << 0, Tcie = 1u << 1, Htie = 1u << 2, Teie = 1u << 3, Dir = 1u << 4, Circ = 1u << 5,
                           Pinc = 1u << 6, Minc = 1u << 7, Mem2Mem = 1u << 14;
        private const uint Tcif = 1u << 1, Htif = 1u << 2, Teif = 1u << 3;

        private readonly IMachine machine;
        private readonly Channel[] channel;
        private readonly Dictionary<int, bool> requestLevel = new Dictionary<int, bool>();
    }
}
