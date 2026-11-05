// SPDX-License-Identifier: MIT
//
// STM32H5 I2C (RM0481 chapter I2C; registers and bits: SVD STM32H563 v1.8), master mode, for
// Renode. Compiled at run time by fcu.resc:
//   i2c4: I2C.STM32H5_I2C @ sysbus 0x44002C00
//       frequency: 250000000           // kernel clock (PCLK3)
//       EventIRQ -> nvic@125
//       ErrorIRQ -> nvic@126
//   ina226: Sensors.INA226 @ i2c4 0x40
//
// Why an own model: the bus takes time here. Every byte lasts nine SCL periods, the SCL period
// comes from TIMINGR ((SCLL + 1) + (SCLH + 1)) x (PRESC + 1) kernel clocks), so a thread that
// waits for a transfer really waits - the difference between a busy-waiting and an
// interrupt-driven driver shows. What the rtos driver uses, as the hardware does it:
//   * CR2.START with SADD, RD_WRN, NBYTES, AUTOEND: START + address (9 bit times); no device at
//     the address: NACKF, then STOP (the master sends it by itself after a NACK)
//   * write: TXIS asks for a byte; the byte goes out when TXDR is written (the clock is
//     stretched until then); after NBYTES: AUTOEND -> STOP -> STOPF, else TC (repeated START
//     or STOP by software)
//   * read: each byte arrives after 9 bit times -> RXNE; the next one starts when RXDR is read
//   * CR2.STOP: STOP -> STOPF; BUSY from START to STOP; PE = 0 resets the state machine
//   * EventIRQ = TXIS&TXIE | RXNE&RXIE | NACKF&NACKIE | STOPF&STOPIE | TC&TCIE;
//     ErrorIRQ = (BERR | ARLO | OVR) & ERRIE
//   * devices: Renode II2CPeripheral at 7-bit addresses; a write phase is handed over at its
//     end (Write), a read phase asks for its bytes (Read), STOP ends the transaction
// Not modelled: slave mode, 10-bit addresses, RELOAD, SMBus/PEC, timeouts, DMA requests, bus
// errors (no second master).
using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.I2C
{
    public class STM32H5_I2C : SimpleContainer<II2CPeripheral>, IDoubleWordPeripheral, IKnownSize
    {
        public STM32H5_I2C(IMachine machine, long frequency) : base(machine)
        {
            EventIRQ = new GPIO();
            ErrorIRQ = new GPIO();
            bitClock = new LimitTimer(machine.ClockSource, (ulong)frequency, this, "bus", limit: 1,
                                      direction: Direction.Ascending, enabled: false, workMode: WorkMode.OneShot,
                                      eventEnabled: true, autoUpdate: false);
            bitClock.LimitReached += OnBusEvent;
            Reset();
        }

        public long Size => 0x400;
        public GPIO EventIRQ { get; }
        public GPIO ErrorIRQ { get; }

        /// Bytes on the bus (address bytes included) since reset.
        public ulong BusBytes { get; private set; }

        public override void Reset()
        {
            bitClock.Enabled = false;
            cr1 = cr2 = timingr = 0;
            isr = Txe;
            rxdr = 0;
            state = State.Idle;
            writeBuffer.Clear();
            readBuffer.Clear();
            device = null;
            BusBytes = 0;
            UpdateIrqs();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x00: return cr1;
                case 0x04: return cr2;
                case 0x10: return timingr;
                case 0x18: return isr;
                case 0x24:
                {
                    var b = rxdr;
                    if((isr & Rxne) != 0)
                    {
                        isr &= ~Rxne;
                        UpdateIrqs();
                        if(state == State.ReadWaitRxdr)
                        {
                            NextReadByteOrEnd();
                        }
                    }
                    return b;
                }
                case 0x08: case 0x0C: case 0x14: case 0x20: case 0x28:
                    return 0;
                default:
                    this.Log(LogLevel.Warning, "read from unmodelled offset 0x{0:X}", offset);
                    return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case 0x00:
                {
                    var wasOn = (cr1 & Pe) != 0;
                    cr1 = value;
                    if(wasOn && (cr1 & Pe) == 0)
                    {
                        // software reset: lines released, state machine and flags cleared
                        bitClock.Enabled = false;
                        state = State.Idle;
                        isr = Txe;
                        writeBuffer.Clear();
                        device?.FinishTransmission();
                        device = null;
                    }
                    if((value & (TxDmaEn | RxDmaEn)) != 0) this.Log(LogLevel.Warning, "DMA requests not modelled");
                    UpdateIrqs();
                    break;
                }
                case 0x04: WriteCr2(value); break;
                case 0x10: timingr = value; break;
                case 0x18:
                    if((value & Txe) != 0 && state == State.Idle)
                    {
                        isr |= Txe;  // flush TXDR
                    }
                    break;
                case 0x1C:
                    isr &= ~(value & IcrMask);
                    UpdateIrqs();
                    break;
                case 0x28:
                    txdr = (byte)value;
                    isr &= ~(Txe | Txis);
                    UpdateIrqs();
                    if(state == State.WriteWaitTxdr)
                    {
                        Schedule(State.WriteByte, 9);
                    }
                    break;
                case 0x08: case 0x0C: case 0x14:
                    break;  // own addresses, timeout: slave/SMBus not modelled
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X} to unmodelled offset 0x{1:X}", value, offset);
                    break;
            }
        }

        // ---- master state machine ------------------------------------------------------------

        private enum State { Idle, Address, WriteWaitTxdr, WriteByte, ReadByte, ReadWaitRxdr, Stop, Paused }

        private void WriteCr2(uint value)
        {
            cr2 = value & ~(Start | Stop);
            if((cr1 & Pe) == 0)
            {
                return;
            }
            if((value & Start) != 0 && (state == State.Idle || state == State.Paused))
            {
                // (repeated) START: the previous write phase goes to the device first
                FlushWrite();
                isr &= ~Tc;
                isr |= Busy;
                nbytes = (int)((value >> 16) & 0xFF);
                done = 0;
                reading = (value & RdWrn) != 0;
                address = (int)((value >> 1) & 0x7F);
                if((value & (Add10 | Reload)) != 0) this.Log(LogLevel.Warning, "10-bit addresses / RELOAD not modelled");
                cr2 |= Start;
                Schedule(State.Address, 10);  // START + 8 bits + ACK
                return;
            }
            if((value & Stop) != 0 && state == State.Paused)
            {
                Schedule(State.Stop, 1);
            }
        }

        private void Schedule(State next, int bits)
        {
            state = next;
            var t = timingr;
            var presc = ((t >> 28) & 0xF) + 1;
            var period = ((t & 0xFF) + 1 + ((t >> 8) & 0xFF) + 1) * presc;
            bitClock.Enabled = false;
            bitClock.Value = 0;
            bitClock.Limit = Math.Max(1UL, (ulong)period * (ulong)bits);
            bitClock.Enabled = true;
        }

        private void OnBusEvent()
        {
            bitClock.Enabled = false;
            switch(state)
            {
                case State.Address:
                    cr2 &= ~Start;
                    ++BusBytes;
                    if(!TryGetByAddress(address, out device))
                    {
                        device = null;
                        isr |= Nackf;
                        Schedule(State.Stop, 1);  // the master sends STOP after a NACK
                        break;
                    }
                    if(reading)
                    {
                        readBuffer.Clear();
                        readBuffer.AddRange(nbytes > 0 ? device.Read(nbytes) : new byte[0]);
                        NextReadByteOrEnd();
                    }
                    else
                    {
                        NextWriteByteOrEnd();
                    }
                    break;
                case State.WriteByte:
                    ++BusBytes;
                    writeBuffer.Add(txdr);
                    isr |= Txe;
                    ++done;
                    NextWriteByteOrEnd();
                    break;
                case State.ReadByte:
                    ++BusBytes;
                    rxdr = done < readBuffer.Count ? readBuffer[done] : (byte)0xFF;
                    ++done;
                    isr |= Rxne;
                    state = State.ReadWaitRxdr;
                    break;
                case State.Stop:
                    FlushWrite();
                    device?.FinishTransmission();
                    device = null;
                    isr |= Stopf;
                    isr &= ~Busy;
                    cr2 &= ~Stop;
                    state = State.Idle;
                    break;
            }
            UpdateIrqs();
        }

        private void NextWriteByteOrEnd()
        {
            if(done < nbytes)
            {
                isr |= Txis;
                state = State.WriteWaitTxdr;
                return;
            }
            EndOfPhase();
        }

        private void NextReadByteOrEnd()
        {
            if(done < nbytes)
            {
                Schedule(State.ReadByte, 9);
                return;
            }
            EndOfPhase();
        }

        private void EndOfPhase()
        {
            if((cr2 & Autoend) != 0)
            {
                Schedule(State.Stop, 1);
            }
            else
            {
                isr |= Tc;
                state = State.Paused;
            }
        }

        private void FlushWrite()
        {
            if(writeBuffer.Count > 0 && device != null)
            {
                device.Write(writeBuffer.ToArray());
            }
            writeBuffer.Clear();
        }

        private void UpdateIrqs()
        {
            var ev = ((isr & Txis) != 0 && (cr1 & (1u << 1)) != 0) || ((isr & Rxne) != 0 && (cr1 & (1u << 2)) != 0) ||
                     ((isr & Nackf) != 0 && (cr1 & (1u << 4)) != 0) || ((isr & Stopf) != 0 && (cr1 & (1u << 5)) != 0) ||
                     ((isr & Tc) != 0 && (cr1 & (1u << 6)) != 0);
            var er = (isr & ((1u << 8) | (1u << 9) | (1u << 10))) != 0 && (cr1 & (1u << 7)) != 0;
            EventIRQ.Set(ev && (cr1 & Pe) != 0);
            ErrorIRQ.Set(er && (cr1 & Pe) != 0);
        }

        private const uint Pe = 1u << 0, TxDmaEn = 1u << 14, RxDmaEn = 1u << 15;
        private const uint RdWrn = 1u << 10, Add10 = 1u << 11, Start = 1u << 13, Stop = 1u << 14, Reload = 1u << 24,
                           Autoend = 1u << 25;
        private const uint Txe = 1u << 0, Txis = 1u << 1, Rxne = 1u << 2, Nackf = 1u << 4, Stopf = 1u << 5, Tc = 1u << 6,
                           Busy = 1u << 15;
        private const uint IcrMask = (1u << 3) | (1u << 4) | (1u << 5) | (1u << 8) | (1u << 9) | (1u << 10) |
                                     (1u << 11) | (1u << 12) | (1u << 13);

        private readonly LimitTimer bitClock;
        private readonly List<byte> writeBuffer = new List<byte>();
        private readonly List<byte> readBuffer = new List<byte>();
        private uint cr1, cr2, timingr, isr;
        private byte txdr, rxdr;
        private State state;
        private int nbytes, done, address;
        private bool reading;
        private II2CPeripheral device;
    }
}
