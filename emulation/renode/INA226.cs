// SPDX-License-Identifier: MIT
//
// TI INA226 current/power monitor (datasheet SBOS547A) as an I2C device for Renode. Compiled
// at run time by fcu.resc:
//   ina226: Sensors.INA226 @ i2c4 0x40
//       busMillivolts: 24000
//       shuntMicrovolts: 5000
//       ALERT -> gpioPortC@9                   // open drain, active low (APOL = 0)
//
// The analog side is two values, settable in the monitor (sysbus.i2c4.ina226 BusMillivolts 20000).
// The digital side as the datasheet describes it:
//   * registers CONFIGURATION (reset 0x4127), SHUNT VOLTAGE (2.5 uV), BUS VOLTAGE (1.25 mV),
//     POWER, CURRENT, CALIBRATION, MASK/ENABLE, ALERT LIMIT, MANUFACTURER ID 0x5449, DIE ID 0x2260;
//     a write sets the register pointer (1 byte) and a register (3 bytes), a read returns the
//     register at the pointer, MSB first
//   * continuous conversions (MODE 5..7) every AVG x (VBUSCT + VSHCT) (conversion times 140 us
//     .. 8.244 ms, 1 .. 1024 averages); triggered modes (1..3) convert once per CONFIG write;
//     each conversion updates the result registers: CURRENT = SHUNT x CAL / 2048, POWER =
//     CURRENT x BUS / 20000, and sets CVRF
//   * alert functions (MASK/ENABLE: SOL, SUL, BOL, BUL, POL against ALERT LIMIT after each
//     conversion, CNVR on CVRF), AFF; ALERT asserted (low, or high with APOL) while the function
//     holds - or, with LEN, until MASK/ENABLE is read. Reading MASK/ENABLE clears CVRF.
//   * CONFIGURATION bit 15: reset
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.I2C;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Sensors
{
    public class INA226 : II2CPeripheral
    {
        public INA226(IMachine machine, int busMillivolts = 0, int shuntMicrovolts = 0)
        {
            BusMillivolts = busMillivolts;
            ShuntMicrovolts = shuntMicrovolts;
            ALERT = new GPIO();
            conversion = new LimitTimer(machine.ClockSource, 1000000, this, "conversion", limit: 1000,
                                        direction: Direction.Ascending, enabled: false, workMode: WorkMode.Periodic,
                                        eventEnabled: true, autoUpdate: true);
            conversion.LimitReached += Convert;
            Reset();
        }

        public GPIO ALERT { get; }
        public int BusMillivolts { get; set; }
        public int ShuntMicrovolts { get; set; }
        /// Conversions since reset.
        public ulong Conversions { get; private set; }

        public void Reset()
        {
            config = 0x4127;
            shunt = bus = power = current = calibration = mask = limit = 0;
            pointer = 0;
            latched = false;
            Conversions = 0;
            StartConversions();
            UpdateAlert();
        }

        public void Write(byte[] data)
        {
            if(data.Length == 0)
            {
                return;
            }
            pointer = data[0];
            if(data.Length < 3)
            {
                return;
            }
            var value = (ushort)((data[1] << 8) | data[2]);
            switch(pointer)
            {
                case 0x00:
                    if((value & 0x8000) != 0)
                    {
                        Reset();
                        return;
                    }
                    config = (ushort)(value & 0x0FFF | 0x4000);
                    mask &= unchecked((ushort)~Cvrf);
                    StartConversions();
                    break;
                case 0x05: calibration = (ushort)(value & 0x7FFF); break;
                case 0x06: mask = (ushort)((value & 0xFC03) | (mask & 0x001C)); break;
                case 0x07: limit = value; break;
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X4} to read-only register 0x{1:X2}", value, pointer);
                    break;
            }
            UpdateAlert();
        }

        public byte[] Read(int count = 1)
        {
            ushort value;
            switch(pointer)
            {
                case 0x00: value = config; break;
                case 0x01: value = shunt; break;
                case 0x02: value = bus; break;
                case 0x03: value = power; break;
                case 0x04: value = current; break;
                case 0x05: value = calibration; break;
                case 0x06:
                    value = mask;
                    mask &= unchecked((ushort)~(Cvrf | Aff));  // read clears CVRF and the latched alert
                    latched = false;
                    UpdateAlert();
                    break;
                case 0x07: value = limit; break;
                case 0xFE: value = 0x5449; break;
                case 0xFF: value = 0x2260; break;
                default: value = 0; break;
            }
            var r = new byte[Math.Max(count, 0)];
            if(r.Length > 0) r[0] = (byte)(value >> 8);
            if(r.Length > 1) r[1] = (byte)value;
            return r;
        }

        public void FinishTransmission()
        {
        }

        private void StartConversions()
        {
            conversion.Enabled = false;
            var mode = config & 7;
            if(mode == 0 || mode == 4)
            {
                return;  // power-down
            }
            int[] times = { 140, 204, 332, 588, 1100, 2116, 4156, 8244 };
            int[] averages = { 1, 4, 16, 64, 128, 256, 512, 1024 };
            var vbus = (mode & 2) != 0 ? times[(config >> 6) & 7] : 0;
            var vsh = (mode & 1) != 0 ? times[(config >> 3) & 7] : 0;
            var period = (ulong)averages[(config >> 9) & 7] * (ulong)(vbus + vsh);
            conversion.Value = 0;
            conversion.Limit = Math.Max(1UL, period);
            conversion.Mode = mode >= 5 ? WorkMode.Periodic : WorkMode.OneShot;
            conversion.Enabled = true;
        }

        private void Convert()
        {
            ++Conversions;
            var mode = config & 7;
            if((mode & 1) != 0)
            {
                shunt = (ushort)(short)Math.Max(-32768, Math.Min(32767, (int)Math.Round(ShuntMicrovolts / 2.5)));
            }
            if((mode & 2) != 0)
            {
                bus = (ushort)Math.Max(0, Math.Min(0x7FFF, (int)Math.Round(BusMillivolts / 1.25)));
            }
            var c = (long)(short)shunt * calibration / 2048;
            current = (ushort)(short)Math.Max(-32768, Math.Min(32767, c));
            power = (ushort)Math.Min(0xFFFF, Math.Abs((long)(short)current) * bus / 20000);
            mask |= Cvrf;
            if(AlertFunction())
            {
                latched = true;
                mask |= Aff;
            }
            UpdateAlert();
        }

        /// The enabled alert function holds (datasheet 7.6.7).
        private bool AlertFunction()
        {
            if((mask & 0x8000) != 0) return (short)shunt > (short)limit;                 // SOL
            if((mask & 0x4000) != 0) return (short)shunt < (short)limit;                 // SUL
            if((mask & 0x2000) != 0) return bus > limit;                                  // BOL
            if((mask & 0x1000) != 0) return bus < limit;                                  // BUL
            if((mask & 0x0800) != 0) return power > limit;                                // POL
            if((mask & 0x0400) != 0) return (mask & Cvrf) != 0;                           // CNVR
            return false;
        }

        private void UpdateAlert()
        {
            var len = (mask & 1) != 0;
            var active = len ? latched : AlertFunction();
            if(!active)
            {
                mask &= unchecked((ushort)~Aff);
            }
            var apol = (mask & 2) != 0;
            ALERT.Set(active == apol);  // active low (APOL = 0): asserted = low
        }

        private const ushort Cvrf = 1 << 3, Aff = 1 << 4;

        private readonly LimitTimer conversion;
        private ushort config, shunt, bus, power, current, calibration, mask, limit;
        private byte pointer;
        private bool latched;
    }
}
