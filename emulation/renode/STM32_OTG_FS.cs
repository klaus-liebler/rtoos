// SPDX-License-Identifier: MIT
//
// STM32 USB OTG core (Synopsys DWC2) in device mode, full speed, slave mode, for Renode - together
// with a scripted USB host on its bus. Compiled at run time:
//   include @emulation/renode/STM32_OTG_FS.cs
//   usb: USB.STM32_OTG_FS @ sysbus 0x40080000     (STM32H743 USB2_OTG_FS)
//       IRQ -> nvic@101
//   sysbus.usb Mode "echo"            # or "modbus"
//   sysbus.usb ResultFile @result.txt
//
// Why an own model: Renode has no model of this core. This one implements what rtoos'
// stm32/usb_otg_device.hpp uses, with the register semantics of ST's stm32h743xx.h and RM0433
// (the same as the host test tests/usb_otg_fs_test.cpp):
//   GINTSTS rc_w1, RXFLVL while the RX status queue holds an entry, OEPINT/IEPINT from the
//   endpoint interrupts, CMOD = 0 once GUSBCFG.FDMOD is set; GRSTCTL.AHBIDL reads 1, CSRST and
//   the flushes clear themselves; GRXSTSP pops (popping "transfer done" sets DOEPINTn.XFRC and
//   disables the endpoint, "SETUP done" sets DOEPINT0.STUP); DxEPCTLn CNAK/SNAK/SD0PID/EPDIS are
//   actions, EPENA is set-only; DxEPINTn rc_w1; RX FIFO reads at 0x1000, TX FIFO n writes at
//   0x1000 (n + 1); the interrupt line is GINTSTS & GINTMSK with GAHBCFG.GINT.
//
// The host starts 10 ms after the device connects (DCTL.SDIS cleared) and runs one token per
// millisecond, retrying NAKed ones (at most 2000 times): bus reset; GET_DESCRIPTOR device;
// SET_ADDRESS 5 (DCFG.DAD must be 5 before the status stage completes); GET_DESCRIPTOR
// configuration (9 bytes, then all); SET_CONFIGURATION 1; then, on the first CDC data
// interface (class 0x0A) with two bulk endpoints in alternate setting 0:
//   echo    send "rtoos usb echo 0123456789 abcdefghijklmnopqrstuvwxyz ABCDEFGHIJKLMNOPQRSTUVWXYZ"
//           (79 bytes: two packets) and read it back
//   modbus  send the Modbus RTU request "unit 1, read holding register 0 (count 1)" and read
//           the answer: unit 1, function 3, 2 data bytes, valid CRC
// The result - "USB RESULT: PASS <details>" or "USB RESULT: FAIL <step>" - goes to the log and
// to ResultFile.
using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Text;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.USB
{
    public class STM32_OTG_FS : IDoubleWordPeripheral, IKnownSize
    {
        public STM32_OTG_FS(IMachine machine)
        {
            IRQ = new GPIO();
            tick = new LimitTimer(machine.ClockSource, 1000000, this, "host", limit: 1000, direction: Direction.Ascending,
                                  enabled: false, workMode: WorkMode.Periodic, eventEnabled: true, autoUpdate: false);
            tick.LimitReached += OnTick;
            Reset();
        }

        public long Size => 0x10000;
        public GPIO IRQ { get; }
        public string Mode { get; set; } = "echo";
        public string ResultFile { get; set; }
        public string Result { get; private set; } = "";

        public void Reset()
        {
            regs.Clear();
            rx.Clear();
            rxData.Clear();
            for(var i = 0; i < Endpoints; ++i)
            {
                tx[i] = new Queue<uint>();
                inNak[i] = outNak[i] = true;
                inPid[i] = outPid[i] = 0;
            }
            host = null;
            hostStarted = false;
            tick.Enabled = false;
            UpdateIrq();
        }

        // ---- registers ---------------------------------------------------------------------------

        public uint ReadDoubleWord(long offset)
        {
            if(offset >= 0x1000)
            {
                if(offset < 0x2000)
                {
                    if(rxData.Count == 0)
                    {
                        this.Log(LogLevel.Warning, "RX FIFO read beyond the packet");
                        return 0;
                    }
                    return rxData.Dequeue();
                }
                return 0;
            }
            switch(offset)
            {
                case GINTSTS:
                {
                    var v = Get(GINTSTS) & ~(RXFLVL | OEPINT | IEPINT | CMOD);
                    if(rx.Count != 0) v |= RXFLVL;
                    var d = Daint() & Get(DAINTMSK);
                    if((d & 0xFFFF0000u) != 0) v |= OEPINT;
                    if((d & 0x0000FFFFu) != 0) v |= IEPINT;
                    if((Get(GUSBCFG) & (1u << 30)) == 0) v |= CMOD;
                    return v;
                }
                case GRSTCTL: return Get(GRSTCTL) | (1u << 31);
                case DAINT: return Daint();
                case GRXSTSP:
                {
                    if(rx.Count == 0)
                    {
                        this.Log(LogLevel.Warning, "GRXSTSP popped while empty");
                        return 0;
                    }
                    var s = rx.Dequeue();
                    rxData.Clear();
                    foreach(var w in s.Data) rxData.Enqueue(w);
                    var n = (int)(s.Word & 0xF);
                    var pktsts = (s.Word >> 17) & 0xF;
                    if(pktsts == 3)
                    {
                        Set(Doepctl(n), Get(Doepctl(n)) & ~EPENA);
                        outNak[n] = true;
                        Set(Doepint(n), Get(Doepint(n)) | 1u);
                    }
                    else if(pktsts == 4)
                    {
                        Set(Doepint(0), Get(Doepint(0)) | (1u << 3));
                    }
                    UpdateIrq();
                    return s.Word;
                }
                default: return Get(offset);
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            if(offset >= 0x1000)
            {
                var n = (int)(offset / 0x1000) - 1;
                if(n >= 0 && n < Endpoints) tx[n].Enqueue(value);
                return;
            }
            if(offset == GINTSTS || IsInt(offset))
            {
                Set(offset, Get(offset) & ~value);
            }
            else if(offset == GRSTCTL)
            {
                if((value & (1u << 5)) != 0)
                {
                    var f = (value >> 6) & 0x1F;
                    for(var i = 0; i < Endpoints; ++i) if(f == 0x10 || f == i) tx[i].Clear();
                }
                if((value & (1u << 4)) != 0) rx.Clear();
                Set(GRSTCTL, value & ~((1u << 0) | (1u << 4) | (1u << 5)));
            }
            else if(IsCtl(offset, out var isIn, out var n))
            {
                var old = Get(offset);
                var nv = (value & ~(CNAK | SNAK | SD0PID | EPDIS | EPENA)) | (old & EPENA) | (value & EPENA);
                if((value & CNAK) != 0) { if(isIn) inNak[n] = false; else outNak[n] = false; }
                if((value & SNAK) != 0) { if(isIn) inNak[n] = true; else outNak[n] = true; }
                if((value & SD0PID) != 0) { if(isIn) inPid[n] = 0; else outPid[n] = 0; }
                if((value & EPDIS) != 0 && (old & EPENA) != 0)
                {
                    nv &= ~EPENA;
                    var ir = isIn ? Diepint(n) : Doepint(n);
                    Set(ir, Get(ir) | 2u);
                }
                Set(offset, nv);
            }
            else
            {
                Set(offset, value);
                if(offset == DCTL && (value & 2u) == 0 && !hostStarted && Get(GINTMSK) != 0)
                {
                    hostStarted = true;
                    startIn = 10;
                    tick.Value = 0;
                    tick.Enabled = true;
                }
            }
            UpdateIrq();
        }

        // ---- the host ----------------------------------------------------------------------------

        private enum Hs { Ack, Nak, Stall }

        private void OnTick()
        {
            if(startIn > 0)
            {
                if(--startIn == 0)
                {
                    host = Script().GetEnumerator();
                }
                return;
            }
            if(host == null) return;
            try
            {
                if(!host.MoveNext())
                {
                    host = null;
                    tick.Enabled = false;
                }
            }
            catch(Exception e)
            {
                Finish("FAIL " + e.Message);
            }
        }

        private void Finish(string outcome)
        {
            Result = "USB RESULT: " + outcome;
            this.Log(LogLevel.Info, Result);
            if(!string.IsNullOrEmpty(ResultFile)) File.WriteAllText(ResultFile, Result + "\n");
            host = null;
            tick.Enabled = false;
        }

        private class Fail : Exception
        {
            public Fail(string m) : base(m) { }
        }

        // token helpers: one token, the handshake; the coroutine yields between tokens
        private Hs Setup(byte[] packet)
        {
            Set(Diepctl(0), Get(Diepctl(0)) & ~STALL);
            Set(Doepctl(0), Get(Doepctl(0)) & ~STALL);
            var t = Get(Doeptsiz(0));
            var stup = t >> 29;
            Set(Doeptsiz(0), (t & ~(3u << 29)) | ((stup > 0 ? stup - 1 : 0) << 29));
            rx.Enqueue(new Entry((6u << 17) | (8u << 4), Words(packet)));
            rx.Enqueue(new Entry(4u << 17, new uint[0]));
            UpdateIrq();
            return Hs.Ack;
        }

        private Hs Out(int ep, byte[] data)
        {
            var ctl = Get(Doepctl(ep));
            if((ctl & STALL) != 0) return Hs.Stall;
            if((ctl & EPENA) == 0 || outNak[ep]) return Hs.Nak;
            var t = Get(Doeptsiz(ep));
            var pkt = (t >> 19) & 0x3FF;
            var size = t & 0x7FFFF;
            if(pkt == 0 || data.Length > size) throw new Fail($"OUT ep{ep}: armed for {pkt} packets / {size} bytes");
            Set(Doeptsiz(ep), (t & ~((0x3FFu << 19) | 0x7FFFFu)) | ((pkt - 1) << 19) | (size - (uint)data.Length));
            rx.Enqueue(new Entry((2u << 17) | ((uint)data.Length << 4) | (uint)ep, Words(data)));
            rx.Enqueue(new Entry((3u << 17) | (uint)ep, new uint[0]));
            outPid[ep] ^= 1;
            UpdateIrq();
            return Hs.Ack;
        }

        private Hs In(int ep, out byte[] packet)
        {
            packet = null;
            var ctl = Get(Diepctl(ep));
            if((ctl & STALL) != 0) return Hs.Stall;
            if((ctl & EPENA) == 0 || inNak[ep]) return Hs.Nak;
            var t = Get(Dieptsiz(ep));
            var len = (int)(t & 0x7FFFF);
            if(((t >> 19) & 0x3FF) != 1) throw new Fail($"IN ep{ep}: PKTCNT {(t >> 19) & 0x3FF}, expected 1");
            if(tx[ep].Count != (len + 3) / 4) throw new Fail($"IN ep{ep}: FIFO holds {tx[ep].Count} words for {len} bytes");
            packet = new byte[len];
            for(var i = 0; i < len; i += 4)
            {
                var w = tx[ep].Dequeue();
                for(var b = 0; b < 4 && i + b < len; ++b) packet[i + b] = (byte)(w >> (8 * b));
            }
            inPid[ep] ^= 1;
            Set(Diepctl(ep), Get(Diepctl(ep)) & ~EPENA);
            inNak[ep] = true;
            Set(Diepint(ep), Get(Diepint(ep)) | 1u);
            UpdateIrq();
            return Hs.Ack;
        }

        private const int Retries = 2000;

        /// A control transfer: SETUP, data stage (IN up to `length` bytes, or OUT `outData`), status.
        private IEnumerable<bool> Control(byte[] setup, List<byte> inData, byte[] outData, string name)
        {
            Setup(setup);
            yield return true;
            var deviceToHost = (setup[0] & 0x80) != 0;
            var length = setup[6] | (setup[7] << 8);
            if(deviceToHost)
            {
                while(true)
                {
                    byte[] p = null;
                    var tries = 0;
                    Hs hs;
                    while((hs = In(0, out p)) == Hs.Nak)
                    {
                        if(++tries > Retries) throw new Fail(name + ": IN data stage NAKed");
                        yield return true;
                    }
                    if(hs == Hs.Stall) throw new Fail(name + ": stalled");
                    inData.AddRange(p);
                    yield return true;
                    if(p.Length < 64 || inData.Count >= length) break;
                }
                var tries2 = 0;
                Hs s2;
                while((s2 = Out(0, new byte[0])) == Hs.Nak)
                {
                    if(++tries2 > Retries) throw new Fail(name + ": status OUT NAKed");
                    yield return true;
                }
                if(s2 == Hs.Stall) throw new Fail(name + ": status stalled");
                yield return true;
            }
            else
            {
                if(outData != null)
                {
                    for(var at = 0; at < outData.Length; at += 64)
                    {
                        var chunk = outData.Skip(at).Take(64).ToArray();
                        var tries = 0;
                        Hs hs;
                        while((hs = Out(0, chunk)) == Hs.Nak)
                        {
                            if(++tries > Retries) throw new Fail(name + ": OUT data stage NAKed");
                            yield return true;
                        }
                        if(hs == Hs.Stall) throw new Fail(name + ": stalled");
                        yield return true;
                    }
                }
                byte[] st = null;
                var tries3 = 0;
                Hs s3;
                while((s3 = In(0, out st)) == Hs.Nak)
                {
                    if(++tries3 > Retries) throw new Fail(name + ": status IN NAKed");
                    yield return true;
                }
                if(s3 == Hs.Stall) throw new Fail(name + ": stalled");
                if(st.Length != 0) throw new Fail(name + ": status packet not empty");
                yield return true;
            }
        }

        private IEnumerable<bool> BulkOut(int ep, byte[] data)
        {
            for(var at = 0; at < data.Length; at += 64)
            {
                var chunk = data.Skip(at).Take(64).ToArray();
                var tries = 0;
                Hs hs;
                while((hs = Out(ep, chunk)) == Hs.Nak)
                {
                    if(++tries > Retries) throw new Fail($"bulk OUT ep{ep} NAKed");
                    yield return true;
                }
                if(hs == Hs.Stall) throw new Fail($"bulk OUT ep{ep} stalled");
                yield return true;
            }
        }

        private IEnumerable<bool> BulkIn(int ep, List<byte> into, int expected)
        {
            var tries = 0;
            while(into.Count < expected)
            {
                var hs = In(ep, out var p);
                if(hs == Hs.Stall) throw new Fail($"bulk IN ep{ep} stalled");
                if(hs == Hs.Nak)
                {
                    if(++tries > Retries) throw new Fail($"bulk IN ep{ep}: {into.Count} of {expected} bytes");
                    yield return true;
                    continue;
                }
                into.AddRange(p);
                yield return true;
            }
        }

        private IEnumerable<bool> Script()
        {
            Set(GINTSTS, Get(GINTSTS) | USBRST | ENUMDNE);  // bus reset, enumeration done
            UpdateIrq();
            yield return true;
            yield return true;
            var dd = new List<byte>();
            foreach(var s in Control(new byte[] {0x80, 6, 0, 1, 0, 0, 64, 0}, dd, null, "GET_DESCRIPTOR device")) yield return s;
            if(dd.Count != 18 || dd[1] != 1) throw new Fail($"device descriptor: {dd.Count} bytes");
            var vid = dd[8] | (dd[9] << 8);
            var pid = dd[10] | (dd[11] << 8);
            foreach(var s in Control(new byte[] {0x00, 5, 5, 0, 0, 0, 0, 0}, null, null, "SET_ADDRESS")) yield return s;
            if(((Get(DCFG) >> 4) & 0x7F) != 5) throw new Fail("SET_ADDRESS: DCFG.DAD not 5");
            var head = new List<byte>();
            foreach(var s in Control(new byte[] {0x80, 6, 0, 2, 0, 0, 9, 0}, head, null, "GET_DESCRIPTOR config")) yield return s;
            if(head.Count != 9) throw new Fail("configuration descriptor header");
            var total = head[2] | (head[3] << 8);
            var cd = new List<byte>();
            foreach(var s in Control(new byte[] {0x80, 6, 0, 2, 0, 0, (byte)total, (byte)(total >> 8)}, cd, null, "GET_DESCRIPTOR config (all)")) yield return s;
            if(cd.Count != total) throw new Fail($"configuration descriptor: {cd.Count} of {total} bytes");
            foreach(var s in Control(new byte[] {0x00, 9, 1, 0, 0, 0, 0, 0}, null, null, "SET_CONFIGURATION")) yield return s;
            // the CDC data interface with two bulk endpoints in alternate setting 0
            int outEp = -1, inEp = -1;
            var inData = false;
            for(var i = 0; i + 1 < cd.Count; i += cd[i])
            {
                if(cd[i] == 0) break;
                var type = cd[i + 1];
                if(type == 4)
                {
                    if(outEp >= 0 && inEp >= 0) break;
                    inData = cd[i + 5] == 0x0A && cd[i + 3] == 0 && cd[i + 4] == 2;
                    outEp = inEp = -1;
                }
                else if(type == 5 && inData && (cd[i + 3] & 3) == 2)
                {
                    if((cd[i + 2] & 0x80) != 0) inEp = cd[i + 2] & 0xF; else outEp = cd[i + 2] & 0xF;
                }
            }
            if(outEp < 0 || inEp < 0) throw new Fail("no CDC data interface with two bulk endpoints");
            if(Mode == "modbus")
            {
                var request = WithCrc(new byte[] {1, 3, 0, 0, 0, 1});
                foreach(var s in BulkOut(outEp, request)) yield return s;
                var answer = new List<byte>();
                foreach(var s in BulkIn(inEp, answer, 7)) yield return s;
                var a = answer.ToArray();
                if(a[0] != 1 || a[1] != 3 || a[2] != 2 || !CrcValid(a, 7)) throw new Fail("modbus answer " + BitConverter.ToString(a));
                Finish($"PASS (VID {vid:X4} PID {pid:X4}, modbus over ep{outEp}/{inEp}: {BitConverter.ToString(a)})");
            }
            else
            {
                var text = Encoding.ASCII.GetBytes("rtoos usb echo 0123456789 abcdefghijklmnopqrstuvwxyz ABCDEFGHIJKLMNOPQRSTUVWXYZ");
                foreach(var s in BulkOut(outEp, text)) yield return s;
                var back = new List<byte>();
                foreach(var s in BulkIn(inEp, back, text.Length)) yield return s;
                if(!back.SequenceEqual(text)) throw new Fail("echo differs: " + Encoding.ASCII.GetString(back.ToArray()));
                Finish($"PASS (VID {vid:X4} PID {pid:X4}, {text.Length} bytes echoed over ep{outEp}/{inEp})");
            }
        }

        private static byte[] WithCrc(byte[] frame)
        {
            var crc = Crc(frame, frame.Length);
            return frame.Concat(new[] {(byte)crc, (byte)(crc >> 8)}).ToArray();
        }

        private static bool CrcValid(byte[] f, int n) => n >= 3 && Crc(f, n - 2) == (f[n - 2] | (f[n - 1] << 8));

        private static int Crc(byte[] f, int n)
        {
            var crc = 0xFFFF;
            for(var i = 0; i < n; ++i)
            {
                crc ^= f[i];
                for(var b = 0; b < 8; ++b) crc = (crc & 1) != 0 ? (crc >> 1) ^ 0xA001 : crc >> 1;
            }
            return crc;
        }

        // ---- the register file -------------------------------------------------------------------

        private struct Entry
        {
            public Entry(uint word, uint[] data) { Word = word; Data = data; }
            public uint Word;
            public uint[] Data;
        }

        private static uint[] Words(byte[] b)
        {
            var w = new uint[(b.Length + 3) / 4];
            for(var i = 0; i < b.Length; ++i) w[i / 4] |= (uint)b[i] << (8 * (i % 4));
            return w;
        }

        private uint Get(long o) => regs.TryGetValue(o, out var v) ? v : 0;
        private void Set(long o, uint v) => regs[o] = v;

        private uint Daint()
        {
            uint v = 0;
            for(var n = 0; n < Endpoints; ++n)
            {
                if(Get(Diepint(n)) != 0) v |= 1u << n;
                if(Get(Doepint(n)) != 0) v |= 1u << (16 + n);
            }
            return v;
        }

        private void UpdateIrq()
        {
            var pending = (ReadDoubleWord0(GINTSTS) & Get(GINTMSK)) != 0;
            IRQ.Set(pending && (Get(GAHBCFG) & 1u) != 0);
        }

        private uint ReadDoubleWord0(long offset)  // GINTSTS without side effects (ReadDoubleWord has none for it)
        {
            var v = Get(GINTSTS) & ~(RXFLVL | OEPINT | IEPINT | CMOD);
            if(rx.Count != 0) v |= RXFLVL;
            var d = Daint() & Get(DAINTMSK);
            if((d & 0xFFFF0000u) != 0) v |= OEPINT;
            if((d & 0x0000FFFFu) != 0) v |= IEPINT;
            return v;
        }

        private static bool IsCtl(long o, out bool isIn, out int n)
        {
            isIn = false;
            n = 0;
            if(o >= 0x900 && o < 0xB00 && (o - 0x900) % 0x20 == 0) { isIn = true; n = (int)((o - 0x900) / 0x20); return true; }
            if(o >= 0xB00 && o < 0xD00 && (o - 0xB00) % 0x20 == 0) { n = (int)((o - 0xB00) / 0x20); return true; }
            return false;
        }
        private static bool IsInt(long o) => o >= 0x900 && o < 0xD00 && (o - 0x900) % 0x20 == 8;
        private static long Diepctl(int n) => 0x900 + 0x20 * n;
        private static long Diepint(int n) => 0x908 + 0x20 * n;
        private static long Dieptsiz(int n) => 0x910 + 0x20 * n;
        private static long Doepctl(int n) => 0xB00 + 0x20 * n;
        private static long Doepint(int n) => 0xB08 + 0x20 * n;
        private static long Doeptsiz(int n) => 0xB10 + 0x20 * n;

        private const long GAHBCFG = 0x008, GUSBCFG = 0x00C, GRSTCTL = 0x010, GINTSTS = 0x014, GINTMSK = 0x018,
                           GRXSTSP = 0x020, DCFG = 0x800, DCTL = 0x804, DAINT = 0x818, DAINTMSK = 0x81C;
        private const uint CMOD = 1u << 0, RXFLVL = 1u << 4, USBRST = 1u << 12, ENUMDNE = 1u << 13, IEPINT = 1u << 18,
                           OEPINT = 1u << 19;
        private const uint STALL = 1u << 21, CNAK = 1u << 26, SNAK = 1u << 27, SD0PID = 1u << 28, EPDIS = 1u << 30,
                           EPENA = 1u << 31;
        private const int Endpoints = 8;

        private readonly Dictionary<long, uint> regs = new Dictionary<long, uint>();
        private readonly Queue<Entry> rx = new Queue<Entry>();
        private readonly Queue<uint> rxData = new Queue<uint>();
        private readonly Queue<uint>[] tx = new Queue<uint>[Endpoints];
        private readonly bool[] inNak = new bool[Endpoints], outNak = new bool[Endpoints];
        private readonly int[] inPid = new int[Endpoints], outPid = new int[Endpoints];
        private readonly LimitTimer tick;
        private IEnumerator<bool> host;
        private bool hostStarted;
        private int startIn;
    }
}
