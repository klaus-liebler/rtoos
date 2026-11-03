// SPDX-License-Identifier: MIT
//
// STM32H5 SPI (RM0481 chapter SPI; registers and bits: SVD STM32H563 v1.8), master mode, for
// Renode. Compiled at run time by fcu.resc:
//   spi2: SPI.STM32H5_SPI @ sysbus 0x40003800
//       kernelFrequency: 100000000
//       fifoSize: 16
//       IRQ -> nvic@56
//
// Why an own model: Renode's SPI models move whole frames at once, with no time on the bus.
// Devices that use the data lines as something else need the bit timing - the HX711 takes
// MOSI as its clock PD_SCK and powers down when it stays high for 60 us. This model shifts
// every bit in virtual time and drives the lines as GPIOs:
//   MOSI  output GPIO (property MOSI), changes at the start of each bit
//   SCK   output GPIO (property SCK), CPOL = 0: rises in the middle of each bit, falls at its end
//   MISO  input 0, sampled at the rising SCK edge
// Bit period: kernel clock / 2^(MBR + 1) (BPASS: kernel clock).
//
// Modelled (what the rtoos driver uses, as the hardware does it):
//   * CR1 SPE, CSTART, SSI; CR2 TSIZE; CFG1 DSIZE (4..8 bits), MBR, BPASS; CFG2 MASTER, LSBFRST,
//     AFCNTR; IER, SR (RXP, TXP, DXP, EOT, TXTF, OVR, TXC, RXPLVL, CTSIZE), IFCR; TXDR/RXDR with
//     8-bit accesses
//   * FIFOs of `fifoSize` frames each (16 for SPI1-3, 8 for SPI4-6: ST HAL, IS_SPI_FULL_INSTANCE /
//     SPI_HIGHEND_FIFO_SIZE). The TxFIFO accepts data as soon as SPE = 1; CSTART starts the
//     transfer of TSIZE frames. Data written before CSTART waits in the FIFO.
//   * a master whose TxFIFO runs empty before TSIZE frames are out suspends the clock until
//     the next write; MOSI keeps the level of the last bit (RM0481 documents neither this idle
//     level nor the one after SPE = 0; ST community reports and the FCU's HX711 wiring both
//     rest on "last bit")
//   * after TSIZE frames: EOT and TXC, CSTART clears; RxFIFO full: OVR, the frame is lost
//   * SPE = 0 flushes both FIFOs and aborts a transfer; the lines keep their level
//   * before the first transfer MOSI is high: the reset level is not documented, high is the
//     worst case for a device that clocks on MOSI (the driver's job is to drive it low)
// Not modelled: slave mode, CRC, TI/I2S, half duplex/simplex, CPHA = 1 (logged), frames over
// 8 bits, MIDI/MSSI idle times, DMA requests, SS output.
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.SPI
{
    public class STM32H5_SPI : IDoubleWordPeripheral, IBytePeripheral, IKnownSize, IGPIOReceiver
    {
        public STM32H5_SPI(IMachine machine, long kernelFrequency = 100000000, int fifoSize = 16)
        {
            this.kernelFrequency = kernelFrequency;
            this.fifoSize = fifoSize;
            IRQ = new GPIO();
            MOSI = new GPIO();
            SCK = new GPIO();
            // Half-bit ticks: the timer counts kernel clocks, its limit is half the prescaler.
            halfBit = new LimitTimer(machine.ClockSource, (ulong)kernelFrequency, this, "halfBit", limit: 1,
                                     direction: Direction.Ascending, enabled: false,
                                     workMode: WorkMode.Periodic, eventEnabled: true, autoUpdate: true);
            halfBit.LimitReached += OnHalfBit;
            Reset();
        }

        public long Size => 0x400;
        public GPIO IRQ { get; }
        public GPIO MOSI { get; }
        public GPIO SCK { get; }

        /// Frames that went over the bus since reset (for tests and the monitor).
        public ulong FramesTransferred { get; private set; }
        /// Times the master suspended the clock because the TxFIFO ran empty mid-transfer.
        public ulong Stalls { get; private set; }

        public void Reset()
        {
            halfBit.Enabled = false;
            cr1 = cr2 = ier = 0;
            cfg1 = 0x00070007;  // SVD reset value
            cfg2 = 0;
            eot = txtf = ovr = false;
            tx.Clear();
            rx.Clear();
            written = sent = 0;
            shifting = stalled = false;
            mosiLevel = true;
            MOSI.Set(true);
            SCK.Set(false);
            UpdateIrq();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number == 0)
            {
                miso = value;
            }
        }

        // ---- register interface ------------------------------------------------------------

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x00: return cr1 | (Started ? Cstart : 0u);
                case 0x04: return cr2;
                case 0x08: return cfg1;
                case 0x0C: return cfg2;
                case 0x10: return ier;
                case 0x14: return Status();
                case 0x30: return ReadByte(offset);
                default:
                    this.Log(LogLevel.Warning, "read from unmodelled offset 0x{0:X}", offset);
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case 0x00: WriteCr1(value); break;
                case 0x04:
                    if(Enabled)
                    {
                        this.Log(LogLevel.Warning, "TSIZE written while SPE = 1 (ignored, RM0481: disable first)");
                        break;
                    }
                    cr2 = value & 0xFFFF;
                    break;
                case 0x08:
                case 0x0C:
                    if(Enabled)
                    {
                        this.Log(LogLevel.Warning, "CFG{0} written while SPE = 1 (ignored)", offset == 0x08 ? 1 : 2);
                        break;
                    }
                    if(offset == 0x08)
                    {
                        cfg1 = value;
                    }
                    else
                    {
                        cfg2 = value;
                    }
                    break;
                case 0x10: ier = value & 0x3FF; UpdateIrq(); break;
                case 0x18:
                    if((value & (1u << 3)) != 0) eot = false;
                    if((value & (1u << 4)) != 0) txtf = false;
                    if((value & (1u << 6)) != 0) ovr = false;
                    UpdateIrq();
                    break;
                case 0x20:
                    WriteByte(offset, (byte)value);
                    if(DataBits > 8 || (value >> 8) != 0)
                    {
                        this.Log(LogLevel.Warning, "32-bit TXDR write: data packing is not modelled, only one frame taken");
                    }
                    break;
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X} to unmodelled offset 0x{1:X}", value, offset);
                    break;
            }
        }

        public byte ReadByte(long offset)
        {
            if(offset != 0x30)
            {
                return (byte)(ReadDoubleWord(offset & ~3L) >> (int)(8 * (offset & 3)));
            }
            if(rx.Count == 0)
            {
                this.Log(LogLevel.Warning, "RXDR read with an empty RxFIFO");
                return 0;
            }
            var b = rx.Dequeue();
            UpdateIrq();
            return b;
        }

        public void WriteByte(long offset, byte value)
        {
            if(offset != 0x20)
            {
                this.Log(LogLevel.Warning, "8-bit write to offset 0x{0:X} is not modelled", offset);
                return;
            }
            if(!Enabled)
            {
                this.Log(LogLevel.Warning, "TXDR written while SPE = 0 (ignored)");
                return;
            }
            if(tx.Count >= fifoSize || (Tsize != 0 && written >= Tsize))
            {
                this.Log(LogLevel.Warning, "TXDR written without TXP (TxFIFO full or all TSIZE frames given): lost");
                return;
            }
            tx.Enqueue(value);
            ++written;
            if(Tsize != 0 && written == Tsize)
            {
                txtf = true;
            }
            if(Started && !shifting)
            {
                StartFrame();
            }
            UpdateIrq();
        }

        // ---- transfer engine ---------------------------------------------------------------

        private void WriteCr1(uint value)
        {
            var wasEnabled = Enabled;
            cr1 = value & ~Cstart;
            if(!Enabled)
            {
                if(wasEnabled)
                {
                    halfBit.Enabled = false;
                    tx.Clear();
                    rx.Clear();
                    shifting = stalled = started = false;
                    SCK.Set(false);
                    if((cfg2 & Afcntr) == 0)
                    {
                        this.Log(LogLevel.Debug, "SPE = 0 with AFCNTR = 0: the pins are released");
                    }
                }
                UpdateIrq();
                return;
            }
            if(!wasEnabled)
            {
                // SPE 0 -> 1: a new session (RM0481: EOT, TXTF, OVR cleared when SPI is re-enabled)
                eot = txtf = ovr = false;
                written = sent = 0;
                started = false;
                if((cfg2 & Master) == 0)
                {
                    this.Log(LogLevel.Warning, "slave mode is not modelled");
                }
                if((cfg2 & Cpha) != 0)
                {
                    this.Log(LogLevel.Warning, "CPHA = 1 is not modelled (sampled as CPHA = 0)");
                }
            }
            if((value & Cstart) != 0 && !started)
            {
                started = true;
                if(Tsize == 0)
                {
                    this.Log(LogLevel.Warning, "endless transfer (TSIZE = 0) is not modelled");
                }
                if(tx.Count > 0)
                {
                    StartFrame();
                }
            }
            UpdateIrq();
        }

        private void StartFrame()
        {
            if(stalled)
            {
                stalled = false;
                this.Log(LogLevel.Debug, "TxFIFO refilled: clock resumes");
            }
            frame = tx.Dequeue();
            frameRx = 0;
            bit = 0;
            secondHalf = false;
            shifting = true;
            var prescaler = (cfg1 & Bpass) != 0 ? 1u : 2u << (int)((cfg1 >> 28) & 7);
            halfBit.Limit = Math.Max(1u, prescaler / 2);
            halfBit.Value = 0;
            DriveBit();
            halfBit.Enabled = true;
        }

        private int BitIndex => (cfg2 & Lsbfrst) != 0 ? bit : DataBits - 1 - bit;

        private void DriveBit()
        {
            mosiLevel = ((frame >> BitIndex) & 1) != 0;
            MOSI.Set(mosiLevel);
        }

        private void OnHalfBit()
        {
            if(!shifting)
            {
                halfBit.Enabled = false;
                return;
            }
            if(!secondHalf)
            {
                SCK.Set(true);
                if(miso)
                {
                    frameRx |= (uint)(1 << BitIndex);
                }
                secondHalf = true;
                return;
            }
            SCK.Set(false);
            secondHalf = false;
            if(++bit < DataBits)
            {
                DriveBit();
                return;
            }
            // frame complete
            ++sent;
            ++FramesTransferred;
            if(rx.Count >= fifoSize)
            {
                ovr = true;
            }
            else
            {
                rx.Enqueue((byte)frameRx);
            }
            if(Tsize != 0 && sent >= Tsize)
            {
                shifting = started = false;
                halfBit.Enabled = false;
                eot = true;
            }
            else if(tx.Count > 0)
            {
                frame = tx.Dequeue();
                frameRx = 0;
                bit = 0;
                DriveBit();
            }
            else
            {
                // TxFIFO empty mid-transfer: the master suspends SCK, MOSI keeps the last bit
                shifting = false;
                stalled = true;
                ++Stalls;
                halfBit.Enabled = false;
                this.Log(LogLevel.Debug, "TxFIFO empty after {0} of {1} frames: clock suspended, MOSI {2}",
                         sent, Tsize, mosiLevel ? "high" : "low");
            }
            UpdateIrq();
        }

        private uint Status()
        {
            uint sr = 0;
            var rxp = rx.Count > 0;
            var txp = tx.Count < fifoSize && (Tsize == 0 || written < Tsize);
            if(rxp) sr |= 1u << 0;
            if(txp) sr |= 1u << 1;
            if(rxp && txp) sr |= 1u << 2;
            if(eot) sr |= 1u << 3;
            if(txtf) sr |= 1u << 4;
            if(ovr) sr |= 1u << 6;
            if(tx.Count == 0 && !shifting) sr |= 1u << 12;  // TXC
            sr |= (uint)Math.Min(rx.Count, 3) << 13;        // RXPLVL (frames up to 8 bits)
            sr |= (uint)((Tsize > sent ? Tsize - sent : 0) & 0xFFFF) << 16;  // CTSIZE
            return sr;
        }

        private void UpdateIrq()
        {
            var sr = Status();
            IRQ.Set((sr & ier & 0x7F) != 0);
        }

        private bool Enabled => (cr1 & Spe) != 0;
        private bool Started => started;
        private uint Tsize => cr2 & 0xFFFF;
        private int DataBits
        {
            get
            {
                var bits = (int)(cfg1 & 0x1F) + 1;
                if(bits > 8)
                {
                    this.Log(LogLevel.Warning, "frames of {0} bits are not modelled (8 used)", bits);
                    return 8;
                }
                return bits;
            }
        }

        private const uint Spe = 1u << 0, Cstart = 1u << 9;
        private const uint Master = 1u << 22, Lsbfrst = 1u << 23, Cpha = 1u << 24, Afcntr = 1u << 31;
        private const uint Bpass = 1u << 31;

        private readonly long kernelFrequency;
        private readonly int fifoSize;
        private readonly LimitTimer halfBit;
        private readonly Queue<byte> tx = new Queue<byte>();
        private readonly Queue<byte> rx = new Queue<byte>();
        private uint cr1, cr2, cfg1, cfg2, ier;
        private bool eot, txtf, ovr;
        private uint written, sent;
        private bool started, shifting, stalled, secondHalf, miso, mosiLevel;
        private uint frame, frameRx;
        private int bit;
    }
}
