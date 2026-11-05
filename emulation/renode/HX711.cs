// SPDX-License-Identifier: MIT
//
// Avia Semiconductor HX711, 24-bit ADC for load cells, for Renode (datasheet "HX711 24-Bit
// Analog-to-Digital Converter (ADC) for Weigh Scales"). Compiled at run time by fcu.resc:
//   hx711: Sensors.HX711 @ gpioPortC 2   // registered at its DOUT pin (monitor name)
//       outputDataRate: 10            // RATE pin: low 10 Hz, high 80 Hz
//       channelA: -2775642
//   spi2:
//       MOSI -> hx711@0                // PD_SCK
//   hx711:
//       DOUT -> spi2@0 | gpioPortC@2
//
// The chip knows nothing of SPI: it watches PD_SCK and drives DOUT, as the datasheet says.
//   * conversions at the output data rate; DOUT goes low when one is ready
//   * each rising PD_SCK edge shifts one bit out on DOUT, MSB first (24 bits, two's
//     complement); the 25th pulse pulls DOUT high again, 25/26/27 pulses select channel A
//     gain 128 / channel B gain 32 / channel A gain 64 for the next conversion
//   * PD_SCK high for more than 60 us: power-down (DOUT high, the reading in progress is lost);
//     PD_SCK low again: reset to channel A gain 128, then the first data after the settling
//     time (4 conversion periods, datasheet: 400 ms at 10 Hz)
//   * pulses while no data is ready are ignored; PD_SCK high longer than T3 = 50 us during a
//     readout is logged and counted (TimingViolations)
// The analog front end is a code per channel, settable in the monitor: ChannelA at gain 128
// (gain 64 gives half of it, as the chip does with the same input), ChannelB at gain 32:
//   sysbus.gpioPortC.hx711 ChannelA 123456
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.SPI;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class HX711 : IPeripheral, IGPIOReceiver
    {
        public HX711(IMachine machine, int outputDataRate = 10, int channelA = 0, int channelB = 0)
        {
            this.machine = machine;
            if(outputDataRate != 10 && outputDataRate != 80)
            {
                throw new ArgumentException("outputDataRate is 10 or 80 Hz (RATE pin)");
            }
            ChannelA = channelA;
            ChannelB = channelB;
            DOUT = new GPIO();
            conversion = new LimitTimer(machine.ClockSource, 1000000, this, "conversion",
                                        limit: (ulong)(1000000 / outputDataRate), direction: Direction.Ascending,
                                        enabled: false, workMode: WorkMode.Periodic, eventEnabled: true, autoUpdate: true);
            conversion.LimitReached += OnConversion;
            powerDown = new LimitTimer(machine.ClockSource, 1000000, this, "powerDown", limit: PowerDownMicroseconds,
                                       direction: Direction.Ascending, enabled: false, workMode: WorkMode.OneShot,
                                       eventEnabled: true, autoUpdate: true);
            powerDown.LimitReached += OnPowerDown;
            Reset();
        }

        public GPIO DOUT { get; }

        /// 24-bit codes of the two inputs: A at gain 128, B at gain 32 (clamped to -8388608..8388607).
        public int ChannelA { get; set; }
        public int ChannelB { get; set; }

        /// Readings shifted out completely (24 data bits) since reset.
        public ulong Readings { get; private set; }
        /// Power-downs caused by PD_SCK high for more than 60 us.
        public ulong PowerDowns { get; private set; }
        public ulong TimingViolations { get; private set; }
        public bool PoweredDown => poweredDown;

        public void Reset()
        {
            powerDown.Enabled = false;
            pdSck = false;
            poweredDown = false;
            Readings = PowerDowns = TimingViolations = 0;
            PowerUp();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number != 0 || value == pdSck)
            {
                return;
            }
            pdSck = value;
            var now = Now;
            if(value)
            {
                risingAt = now;
                powerDown.Enabled = false;
                powerDown.Value = 0;
                powerDown.Enabled = true;
                if(!poweredDown)
                {
                    OnPulse();
                }
                return;
            }
            powerDown.Enabled = false;
            if(poweredDown)
            {
                this.Log(LogLevel.Info, "PD_SCK low again: power-up, reset to channel A gain 128");
                poweredDown = false;
                PowerUp();
                return;
            }
            if(pulses > 0 && pulses <= 25 && now - risingAt > T3MaxMicroseconds)
            {
                ++TimingViolations;
                this.Log(LogLevel.Warning, "PD_SCK high for {0:F1} us during the readout (T3 max 50 us)", now - risingAt);
            }
        }

        // ---- conversion and readout --------------------------------------------------------

        private void PowerUp()
        {
            gain = Gain.A128;
            pulses = 0;
            ready = false;
            settling = SettlingConversions;
            DOUT.Set(true);
            conversion.Enabled = false;
            conversion.Value = 0;
            conversion.Enabled = true;
        }

        private void OnConversion()
        {
            if(poweredDown)
            {
                return;
            }
            if(pulses > 0)
            {
                if(pulses < 24)
                {
                    return;  // a readout in progress keeps its data
                }
                gain = pulses >= 27 ? Gain.A64 : pulses == 26 ? Gain.B32 : Gain.A128;
                pulses = 0;
            }
            if(settling > 0)
            {
                --settling;
                return;
            }
            var raw = gain == Gain.B32 ? ChannelB : gain == Gain.A64 ? ChannelA / 2 : ChannelA;
            data = (uint)Math.Max(-8388608, Math.Min(8388607, raw)) & 0xFFFFFF;
            ready = true;
            DOUT.Set(false);
        }

        private void OnPulse()
        {
            if(!ready && pulses == 0)
            {
                this.Log(LogLevel.Debug, "PD_SCK pulse while no data is ready (ignored)");
                return;
            }
            ++pulses;
            if(pulses <= 24)
            {
                DOUT.Set(((data >> (24 - pulses)) & 1) != 0);
                if(pulses == 24)
                {
                    ++Readings;
                }
            }
            else if(pulses == 25)
            {
                ready = false;
                DOUT.Set(true);
            }
        }

        private void OnPowerDown()
        {
            if(!pdSck || poweredDown)
            {
                return;
            }
            poweredDown = true;
            ready = false;
            pulses = 0;
            ++PowerDowns;
            conversion.Enabled = false;
            DOUT.Set(true);
            this.Log(LogLevel.Warning, "PD_SCK high for more than 60 us: power-down");
        }

        private double Now => machine.ElapsedVirtualTime.TimeElapsed.TotalMicroseconds;

        private enum Gain { A128, B32, A64 }

        private const ulong PowerDownMicroseconds = 60;
        private const double T3MaxMicroseconds = 50;
        private const int SettlingConversions = 4;

        private readonly IMachine machine;
        private readonly LimitTimer conversion;
        private readonly LimitTimer powerDown;
        private bool pdSck, poweredDown, ready;
        private int pulses, settling;
        private uint data;
        private double risingAt;
        private Gain gain;
    }
}
