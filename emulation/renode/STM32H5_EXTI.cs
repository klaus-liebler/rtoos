// SPDX-License-Identifier: MIT
//
// STM32H5 EXTI, the GPIO lines 0-15 (RM0481 chapter EXTI; registers and bits: SVD STM32H563
// v1.8), for Renode. Compiled at run time by fcu.resc:
//   exti: IRQControllers.STM32H5_EXTI @ sysbus 0x44022000
//       [0-15] -> nvic@[11-26]
//   gpioPortA:
//       [0-15] -> exti@[0-15]          // input = port x 16 + pin (A = 0 .. I = 8)
//
// Renode's STM32 EXTI models select the port in SYSCFG (F4) or know only one register layout
// (separate rising/falling pending registers are missing). Here, as on the STM32H5:
//   * EXTICR1-4 pick the port of each line (8 bits per line, 0 = PA .. 8 = PI)
//   * RTSR1/FTSR1 select the edges; an edge on the selected pin sets RPR1/FPR1 (also while
//     the line is masked), SWIER1 sets RPR1
//   * RPR1/FPR1: write 1 to clear; IMR1 unmasks the interrupt of a line
//   * interrupt line n = (RPR1 | FPR1) & IMR1, bit n
// Not modelled: lines 16 and up (internal events), event outputs (EMR1), security/privilege.
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.IRQControllers
{
    public class STM32H5_EXTI : IDoubleWordPeripheral, IKnownSize, IGPIOReceiver, INumberedGPIOOutput
    {
        public STM32H5_EXTI()
        {
            var lines = new Dictionary<int, IGPIO>();
            for(var i = 0; i < Lines; i++)
            {
                lines[i] = new GPIO();
            }
            Connections = lines;
            Reset();
        }

        public long Size => 0x400;
        public IReadOnlyDictionary<int, IGPIO> Connections { get; }

        /// Edges on the selected pin of each line since reset (enabled edges or not).
        public ulong Edges(int line) => edges[line];

        public void Reset()
        {
            rtsr = ftsr = rpr = fpr = 0;
            imr = 0;
            exticr = new uint[4];
            level = new bool[9 * Lines];
            edges = new ulong[Lines];
            Update();
        }

        public void OnGPIO(int number, bool value)
        {
            if(number < 0 || number >= level.Length || level[number] == value)
            {
                return;
            }
            level[number] = value;
            var port = number / Lines;
            var line = number % Lines;
            if(Port(line) != port)
            {
                return;
            }
            ++edges[line];
            var bit = 1u << line;
            if(value && (rtsr & bit) != 0)
            {
                rpr |= bit;
            }
            if(!value && (ftsr & bit) != 0)
            {
                fpr |= bit;
            }
            Update();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
                case 0x00: return rtsr;
                case 0x04: return ftsr;
                case 0x08: return 0;
                case 0x0C: return rpr;
                case 0x10: return fpr;
                case 0x60: case 0x64: case 0x68: case 0x6C: return exticr[(offset - 0x60) / 4];
                case 0x80: return imr | 0xFFFE0000;
                default: return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
                case 0x00: rtsr = value & 0xFFFF; break;
                case 0x04: ftsr = value & 0xFFFF; break;
                case 0x08: rpr |= value & 0xFFFF; break;  // SWIER1: software trigger
                case 0x0C: rpr &= ~value; break;
                case 0x10: fpr &= ~value; break;
                case 0x60: case 0x64: case 0x68: case 0x6C: exticr[(offset - 0x60) / 4] = value; break;
                case 0x80: imr = value & 0xFFFF; break;
                case 0x14: case 0x18: case 0x70: case 0x84: break;  // security, privilege, lock, events
                default:
                    this.Log(LogLevel.Warning, "write 0x{0:X} to unmodelled offset 0x{1:X}", value, offset);
                    break;
            }
            Update();
        }

        private int Port(int line) => (int)((exticr[line / 4] >> (8 * (line % 4))) & 0xFF);

        private void Update()
        {
            var pending = (rpr | fpr) & imr;
            for(var i = 0; i < Lines; i++)
            {
                Connections[i].Set((pending & (1u << i)) != 0);
            }
        }

        private const int Lines = 16;
        private uint rtsr, ftsr, rpr, fpr, imr;
        private uint[] exticr;
        private bool[] level;
        private ulong[] edges;
    }
}
