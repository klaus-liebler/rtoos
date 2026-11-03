// SPDX-License-Identifier: MIT
//
// A signal source for Renode tests: periodic pulses on a GPIO, e.g. an object passing a light
// barrier. Compiled at run time by fcu.resc:
//   barrier1: Miscellaneous.PulseGenerator @ gpioPortC 7
//       periodMicroseconds: 7000
//       widthMicroseconds: 1000
//       activeLow: true                // NPN light barrier: low while interrupted
//       Output -> gpioPortC@7
//
// The output idles inactive; every period it goes active for the width. Pulses counts the
// pulses since reset, so a test can compare what the firmware saw with what happened.
// periodMicroseconds = 0: no pulses (the output stays inactive). Input 0 (registered at a GPIO
// port, it is a GPIO receiver): Enabled, also settable in the monitor - false stops the pulses.
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class PulseGenerator : IGPIOReceiver
    {
        public PulseGenerator(IMachine machine, int periodMicroseconds = 0, int widthMicroseconds = 0, bool activeLow = false)
        {
            if(periodMicroseconds != 0 && (widthMicroseconds <= 0 || widthMicroseconds >= periodMicroseconds))
            {
                throw new ArgumentException("0 < widthMicroseconds < periodMicroseconds");
            }
            period = (ulong)periodMicroseconds;
            width = (ulong)widthMicroseconds;
            this.activeLow = activeLow;
            Output = new GPIO();
            timer = new LimitTimer(machine.ClockSource, 1000000, this, "pulse", limit: 1, direction: Direction.Ascending,
                                   enabled: false, workMode: WorkMode.OneShot, eventEnabled: true, autoUpdate: false);
            timer.LimitReached += OnEdge;
            Reset();
        }

        public GPIO Output { get; }
        public ulong Pulses { get; private set; }

        public bool Enabled
        {
            get => enabled;
            set
            {
                enabled = value;
                Reset();
            }
        }

        public void OnGPIO(int number, bool value)
        {
            if(number == 0)
            {
                Enabled = value;
            }
        }

        public void Reset()
        {
            Pulses = 0;
            active = false;
            Output.Set(activeLow);
            timer.Enabled = false;
            if(period != 0 && enabled)
            {
                Arm(period - width);
            }
        }

        private void OnEdge()
        {
            active = !active;
            if(active)
            {
                ++Pulses;
            }
            Output.Set(active != activeLow);
            Arm(active ? width : period - width);
        }

        private void Arm(ulong microseconds)
        {
            timer.Enabled = false;
            timer.Value = 0;
            timer.Limit = microseconds;
            timer.Enabled = true;
        }

        private readonly ulong period, width;
        private readonly bool activeLow;
        private readonly LimitTimer timer;
        private bool active;
        private bool enabled = true;
    }
}
