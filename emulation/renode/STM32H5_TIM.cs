// SPDX-License-Identifier: MIT
//
// STM32H5 general-purpose timers TIM2-5, TIM15-17 (RM0481 chapters TIM; registers and bits:
// SVD STM32H563 v1.8), up-counting, for Renode. Compiled at run time by fcu.resc:
//   tim3: Timers.STM32H5_TIM @ sysbus 0x40000400
//       frequency: 250000000      // timer kernel clock
//       counterWidth: 16
//       channels: 4
//       IRQ -> nvic@46
//       DmaUpdate -> gpdma1@81    // request lines (GPDMA1_REQUEST_*)
//       Ch1 -> step_probe@0       // compare output 1 (the pin in its AF mode)
//
// Why an own model: Renode's STM32 timer model has no compare outputs, no DMA requests and no
// one-pulse mode, and its period is ARR ticks instead of ARR + 1. rtoos generates step pulses
// and LED bit streams with the timer alone (compare output, ARR fed by DMA), so the model has
// to do what the hardware does, in virtual time and event by event:
//   * CNT counts kernel clock / (PSC + 1) from 0 to ARR, then an update event: CNT = 0; with
//     ARPE (and OCxPE) the preloaded ARR (CCRx) become active, PSC always; UIF, update DMA
//     request (UDE), one-pulse mode (OPM) stops the counter (CEN = 0); UDIS suppresses it
//   * EGR.UG re-initialises the counter (update event; URS: without UIF and DMA request),
//     EGR.CCxG raises the compare flag and DMA request of channel x
//   * compare match CNT == CCRx: CCxIF, CCx DMA request (CCxDE), and the output reference:
//     frozen, active/inactive on match, toggle, forced inactive/active, PWM mode 1/2
//     (OCxM including bit 3 = 0); a match happens when CNT changes to CCRx - when the update
//     makes CNT 0, or a tick does; writing CNT never matches
//   * outputs ChX = reference XOR CCxP while CCxE (and BDTR.MOE on TIM15-17); disabled: low
//   * IRQ = (SR & DIER) for UIF, CC1IF-CC4IF, TIF
// Not modelled: down/center counting, input capture, slave/trigger modes, encoder, RCR, break,
// complementary outputs, DMA burst (DCR/DMAR), dithering, UIFREMAP (all logged when used).
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class STM32H5_TIM : IDoubleWordPeripheral, IKnownSize
    {
        public STM32H5_TIM(IMachine machine, long frequency, int counterWidth = 16, int channels = 4)
        {
            if(counterWidth != 16 && counterWidth != 32)
            {
                throw new ArgumentException("counterWidth is 16 or 32");
            }
            this.channels = channels;
            advanced = channels <= 2;  // TIM15-17: outputs need BDTR.MOE
            max = counterWidth == 32 ? 0xFFFFFFFFUL : 0xFFFFUL;
            IRQ = new GPIO();
            Ch1 = new GPIO();
            Ch2 = new GPIO();
            Ch3 = new GPIO();
            Ch4 = new GPIO();
            DmaUpdate = new GPIO();
            DmaCc1 = new GPIO();
            DmaCc2 = new GPIO();
            DmaCc3 = new GPIO();
            DmaCc4 = new GPIO();
            outputs = new[] { Ch1, Ch2, Ch3, Ch4 };
            ccDma = new[] { DmaCc1, DmaCc2, DmaCc3, DmaCc4 };
            clock = new LimitTimer(machine.ClockSource, (ulong)frequency, this, "counter", limit: 1,
                                   direction: Direction.Ascending, enabled: false, workMode: WorkMode.OneShot,
                                   eventEnabled: true, autoUpdate: false);
            clock.LimitReached += OnEvent;
            Reset();
        }

        public long Size => 0x400;
        public GPIO IRQ { get; }
        public GPIO Ch1 { get; }
        public GPIO Ch2 { get; }
        public GPIO Ch3 { get; }
        public GPIO Ch4 { get; }
        public GPIO DmaUpdate { get; }
        public GPIO DmaCc1 { get; }
        public GPIO DmaCc2 { get; }
        public GPIO DmaCc3 { get; }
        public GPIO DmaCc4 { get; }

        /// Update events since reset (for tests and the monitor).
        public ulong Updates { get; private set; }

        public void Reset()
        {
            clock.Enabled = false;
            cr1 = dier = sr = ccmr1 = ccmr2 = ccer = bdtr = 0;
            psc = pscActive = 0;
            arr = arrActive = max;
            cntBase = 0;
            ccr = new ulong[4];
            ccrActive = new ulong[4];
            reference = new bool[4];
            level = new bool[4];
            Updates = 0;
            clock.Divider = 1UL;
            foreach(var o in outputs)
            {
                o.Set(false);
            }
            UpdateIrq();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x00: return cr1;
                case 0x0C: return dier;
                case 0x10: return sr;
                case 0x14: return 0;
                case 0x18: return ccmr1;
                case 0x1C: return ccmr2;
                case 0x20: return ccer;
                case 0x24: return (uint)Counter;
                case 0x28: return psc;
                case 0x2C: return (uint)arr;
                case 0x34: case 0x38: case 0x3C: case 0x40:
                    return (uint)ccr[(offset - 0x34) / 4];
                case 0x44: return bdtr;
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
                    if(value != 0) this.Log(LogLevel.Warning, "CR2 0x{0:X}: master/slave modes not modelled", value);
                    break;
                case 0x08:
                    if(value != 0) this.Log(LogLevel.Warning, "SMCR 0x{0:X}: slave modes not modelled", value);
                    break;
                case 0x0C: dier = value; UpdateIrq(); break;
                case 0x10: sr &= value; UpdateIrq(); break;  // rc_w0
                case 0x14: WriteEgr(value); break;
                case 0x18: ccmr1 = value; ModesChanged(); break;
                case 0x1C: ccmr2 = value; ModesChanged(); break;
                case 0x20: ccer = value; DriveOutputs(); break;
                case 0x24:
                    Freeze();
                    cntBase = value & max;
                    Schedule();
                    break;
                case 0x28: psc = value & 0xFFFF; break;  // takes effect at the next update event
                case 0x2C:
                    arr = value & max;
                    if((cr1 & Arpe) == 0)
                    {
                        Freeze();
                        arrActive = arr;
                        Schedule();
                    }
                    break;
                case 0x30:
                    if(value != 0) this.Log(LogLevel.Warning, "RCR {0}: repetition counter not modelled", value);
                    break;
                case 0x34: case 0x38: case 0x3C: case 0x40:
                {
                    var ch = (int)((offset - 0x34) / 4);
                    ccr[ch] = value & max;
                    if(!Preloaded(ch))
                    {
                        Freeze();
                        ccrActive[ch] = ccr[ch];
                        PwmLevel(ch, cntBase);
                        DriveOutputs();
                        Schedule();
                    }
                    break;
                }
                case 0x44: bdtr = value; DriveOutputs(); break;
                case 0x3DC: case 0x3E0:
                    this.Log(LogLevel.Warning, "DMA burst (DCR/DMAR) not modelled");
                    break;
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X} to unmodelled offset 0x{1:X}", value, offset);
                    break;
            }
        }

        // ---- counter -----------------------------------------------------------------------

        private bool Running => (cr1 & Cen) != 0;

        private ulong Counter => Running && !inEvent ? (cntBase + clock.Value) & max : cntBase;

        /// Stop the clock and keep the counter in cntBase.
        private void Freeze()
        {
            if(Running && !inEvent)
            {
                cntBase = (cntBase + clock.Value) & max;
            }
            clock.Enabled = false;
        }

        /// Arm the clock for the next event: the update (CNT passes ARR) or a compare match.
        private void Schedule()
        {
            if(inEvent)
            {
                return;  // OnEvent schedules when it is done
            }
            clock.Enabled = false;
            if(!Running)
            {
                return;
            }
            var cnt = cntBase;
            // ticks until CNT becomes 0 again: past ARR, or (CNT above ARR) past the counter's top
            var next = cnt <= arrActive ? arrActive + 1 - cnt : max + 1 - cnt;
            for(var ch = 0; ch < channels; ch++)
            {
                var c = ccrActive[ch];
                if(c > cnt && c <= arrActive && c - cnt < next)
                {
                    next = c - cnt;
                }
            }
            clock.Divider = (ulong)pscActive + 1;
            clock.Value = 0;
            clock.Limit = next;
            clock.Enabled = true;
        }

        private void OnEvent()
        {
            inEvent = true;
            var cnt = cntBase + clock.Limit;
            if(cnt > arrActive || cnt > max)
            {
                cntBase = 0;
                Overflow();
                for(var ch = 0; ch < channels; ch++)
                {
                    if(ccrActive[ch] == 0)
                    {
                        Match(ch);
                    }
                }
            }
            else
            {
                cntBase = cnt;
                for(var ch = 0; ch < channels; ch++)
                {
                    if(ccrActive[ch] == cnt)
                    {
                        Match(ch);
                    }
                }
            }
            DriveOutputs();
            UpdateIrq();
            inEvent = false;
            Schedule();
        }

        private void Overflow()
        {
            if((cr1 & Udis) != 0)
            {
                return;
            }
            UpdateEvent(true);
            if((cr1 & Opm) != 0)
            {
                cr1 &= ~Cen;  // one-pulse mode: stop at the update event
            }
        }

        /// Shadow registers loaded, UIF and DMA request (unless URS and not from the counter).
        private void UpdateEvent(bool fromCounter)
        {
            ++Updates;
            pscActive = psc;
            arrActive = arr;
            for(var ch = 0; ch < 4; ch++)
            {
                if(Preloaded(ch))
                {
                    ccrActive[ch] = ccr[ch];
                }
                PwmLevel(ch, 0);
            }
            if(fromCounter || (cr1 & Urs) == 0)
            {
                sr |= Uif;
                if((dier & Ude) != 0)
                {
                    DmaUpdate.Set(true);  // the DMA may write ARR/CCR right now (preload)
                    DmaUpdate.Set(false);
                }
            }
        }

        private void Match(int ch)
        {
            sr |= 1u << (ch + 1);
            switch(Mode(ch))
            {
                case 1: reference[ch] = true; break;                // active on match
                case 2: reference[ch] = false; break;               // inactive on match
                case 3: reference[ch] = !reference[ch]; break;      // toggle
                case 6: reference[ch] = false; break;               // PWM 1: CNT >= CCR
                case 7: reference[ch] = true; break;                // PWM 2
            }
            if((dier & (1u << (ch + 9))) != 0)
            {
                ccDma[ch].Set(true);
                ccDma[ch].Set(false);
            }
        }

        /// PWM reference for counter value `cnt` (other modes keep their state).
        private void PwmLevel(int ch, ulong cnt)
        {
            var m = Mode(ch);
            if(m == 6) reference[ch] = cnt < ccrActive[ch];
            else if(m == 7) reference[ch] = cnt >= ccrActive[ch];
        }

        private void WriteCr1(uint value)
        {
            if((value & (Cms | Dir)) != 0) this.Log(LogLevel.Warning, "down/center counting not modelled");
            if((value & (1u << 11 | 1u << 12)) != 0) this.Log(LogLevel.Warning, "UIFREMAP/DITHEN not modelled");
            var was = Running;
            if(was)
            {
                Freeze();
            }
            cr1 = value;
            if((cr1 & Arpe) == 0)
            {
                arrActive = arr;
            }
            Schedule();
        }

        private void WriteEgr(uint value)
        {
            if((value & Ug) != 0)
            {
                Freeze();
                cntBase = 0;
                UpdateEvent(false);
            }
            for(var ch = 0; ch < channels; ch++)
            {
                if((value & (1u << (ch + 1))) != 0)
                {
                    sr |= 1u << (ch + 1);
                    if((dier & (1u << (ch + 9))) != 0)
                    {
                        ccDma[ch].Set(true);
                        ccDma[ch].Set(false);
                    }
                }
            }
            DriveOutputs();
            UpdateIrq();
            Schedule();
        }

        private void ModesChanged()
        {
            Freeze();
            for(var ch = 0; ch < channels; ch++)
            {
                var m = Mode(ch);
                if(m == 4) reference[ch] = false;       // forced inactive
                else if(m == 5) reference[ch] = true;   // forced active
                else if(m > 7) this.Log(LogLevel.Warning, "channel {0}: OCxM {1} not modelled", ch + 1, m);
                PwmLevel(ch, cntBase);
                if(((ch < 2 ? ccmr1 : ccmr2) >> (8 * (ch % 2)) & 3) != 0)
                {
                    this.Log(LogLevel.Warning, "channel {0}: input capture not modelled", ch + 1);
                }
            }
            DriveOutputs();
            Schedule();
        }

        /// OCxM including bit 3 (bit 16/24 of CCMRx).
        private int Mode(int ch)
        {
            var reg = ch < 2 ? ccmr1 : ccmr2;
            var shift = 8 * (ch % 2);
            return (int)((reg >> (shift + 4)) & 7) | (int)(((reg >> (16 + shift)) & 1) << 3);
        }

        private bool Preloaded(int ch)
        {
            var reg = ch < 2 ? ccmr1 : ccmr2;
            return ((reg >> (8 * (ch % 2) + 3)) & 1) != 0;
        }

        private void DriveOutputs()
        {
            var moe = !advanced || (bdtr & Moe) != 0;
            for(var ch = 0; ch < channels; ch++)
            {
                var enabled = ((ccer >> (4 * ch)) & 1) != 0 && moe;
                var inverted = ((ccer >> (4 * ch + 1)) & 1) != 0;
                var l = enabled && (reference[ch] ^ inverted);
                if(l != level[ch])
                {
                    level[ch] = l;
                    outputs[ch].Set(l);
                }
            }
        }

        private void UpdateIrq()
        {
            IRQ.Set((sr & dier & 0x5F) != 0);
        }

        private const uint Cen = 1u << 0, Udis = 1u << 1, Urs = 1u << 2, Opm = 1u << 3, Dir = 1u << 4,
                           Cms = 3u << 5, Arpe = 1u << 7;
        private const uint Uif = 1u << 0, Ug = 1u << 0, Ude = 1u << 8, Moe = 1u << 15;

        private readonly int channels;
        private readonly bool advanced;
        private readonly ulong max;
        private readonly LimitTimer clock;
        private readonly GPIO[] outputs;
        private readonly GPIO[] ccDma;
        private uint cr1, dier, sr, ccmr1, ccmr2, ccer, bdtr, psc, pscActive;
        private ulong arr, arrActive, cntBase;
        private ulong[] ccr, ccrActive;
        private bool[] reference, level;
        private bool inEvent;
    }
}
