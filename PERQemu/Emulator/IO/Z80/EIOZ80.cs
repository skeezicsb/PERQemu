//
// EIOZ80.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
// See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using System;
using System.Runtime.CompilerServices;

using PERQemu.Config;
using PERQemu.Debugger;
using PERQemu.IO.GPIB;
using PERQemu.IO.SerialDevices;

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// The EIO board's Z80 subsystem, used in all PERQ-2 models.
    /// </summary>
    public sealed class EIOZ80 : Z80System
    {
        public EIOZ80(PERQSystem system) : base(system)
        {
            // Set up the infrastructure
            _perqToZ80Fifo = new PERQToZ80FIFO(_system);
            _z80ToPerqFifo = new Z80ToPERQFIFO(_system);

            _irqControl = new Am9519(0x60);
            _dmac = new i8237DMA(0x38, 0x30, _memory, _bus);
            _pdma = new PERQDMA(_system);

            // Set up the EIO peripherals
            _tms9914a = new TMS9914A(0, 0x7b, 0x7c);
            _fdc = new NECuPD765A(0x20, _scheduler);
            _z80sioA = new Z80SIO(0x10, this, "A");
            _z80sioB = new Z80SIO(0x40, this, "B");
            _timerA = new i8254PIT(0x50, "A");
            _timerB = new i8254PIT(0x54, "B");
            _rtc = new Oki5832RTC(0x76);

            // Create our serial devices
            _keyboard = new SerialKeyboard();
            // _speech = new Speech(...)

            // Same Z80 code for EIO/NIO
            // Todo: verify for all variants?  8"/5.25", 24-bit?  The real fun
            // begins when we load a ZBoot file and start executing dynamically
            // loaded code from RAM.  Hmmm.
            _z80Debugger = new Z80Debugger("eioz80.lst");

            DeviceInit();
        }

        // Port "A" is public, since it's a DMA-capable device
        public override Z80SIO SIOA => _z80sioA;

        // No hard disk seek circuit on the EIO
        public override Z80CTC CTC => null;


        /// <summary>
        /// Initializes the EIO devices and attaches them to the bus.
        /// </summary>
        void DeviceInit()
        {
            // Attach the configured tablet(s)
            if (_system.Config.Tablet.HasFlag(TabletType.BitPad))
            {
                var tablet = new BitPadOne(_scheduler, _system);
                _tms9914a.Bus.AddDevice(tablet);
            }

            if (_system.Config.Tablet.HasFlag(TabletType.Kriz))
            {
                var tablet = new KrizTablet(_scheduler, _system);
                _z80sioA.AttachDevice(1, tablet);
            }

            // Attach the keyboard
            _z80sioB.AttachDevice(1, _keyboard);

            // If enabled and configured, attach device to RS232 port A
            if (_system.Config.RSAEnable && Settings.RSADevice != string.Empty)
            {
                if (Settings.RSADevice == "RSX:")
                {
                    var rsx = new RSXFilePort(this);
                    _z80sioA.AttachPortDevice(0, rsx);
                    _timerA.AttachDevice(0, rsx);
                }
                else
                {
                    var rsa = new PhysicalPort(this, Settings.RSADevice, Settings.RSASettings, "A");
                    _z80sioA.AttachPortDevice(0, rsa);
                    _timerA.AttachDevice(0, rsa);
                }
            }
            else
            {
                // Otherwise direct it to the bit bucket
                _z80sioA.AttachPortDevice(0, new NullPort(this));
            }

            // Now do RS232 port B
            if (_system.Config.RSBEnable && Settings.RSBDevice != string.Empty)
            {
                if (Settings.RSBDevice == "RSX:")
                {
                    var rsx = new RSXFilePort(this);
                    _z80sioB.AttachPortDevice(0, rsx);
                    _timerB.AttachDevice(0, rsx);
                }
                else
                {
                    var rsb = new PhysicalPort(this, Settings.RSADevice, Settings.RSASettings, "B");
                    _z80sioB.AttachPortDevice(0, rsb);
                    _timerB.AttachDevice(0, rsb);
                }
            }
            else
            {
                // Otherwise direct it to the bit bucket
                _z80sioB.AttachPortDevice(0, new NullPort(this));
            }

            // All aboard the bus
            _bus.RegisterDevice(_z80sioA);
            _bus.RegisterDevice(_z80sioB);
            _bus.RegisterDevice(_irqControl);

            // These guys are NOT direct Z80 interrupt sources
            _bus.RegisterDevice(_fdc, false);
            _bus.RegisterDevice(_rtc, false);
            _bus.RegisterDevice(_dmac, false);
            _bus.RegisterDevice(_pdma, false);
            _bus.RegisterDevice(_timerA, false);
            _bus.RegisterDevice(_timerB, false);
            _bus.RegisterDevice(_tms9914a, false);
            _bus.RegisterDevice(_perqToZ80Fifo, false);
            _bus.RegisterDevice(_z80ToPerqFifo, false);

            // Register clients of the interrupt controller
            _irqControl.RegisterDevice(Am9519.IRQNumber.GPIB, _tms9914a);
            _irqControl.RegisterDevice(Am9519.IRQNumber.Floppy, _fdc);
            _irqControl.RegisterDevice(Am9519.IRQNumber.Z80DMA, _dmac);
            _irqControl.RegisterDevice(Am9519.IRQNumber.Z80toPERQ, _z80ToPerqFifo);
            _irqControl.RegisterDevice(Am9519.IRQNumber.PERQtoZ80, _perqToZ80Fifo);
            _irqControl.RegisterDevice(Am9519.IRQNumber.PERQDMA, _pdma);

            // Assign DMA devices to their channel
            _dmac.AttachChannelDevice(0, _fdc, 0x21);
            _dmac.AttachChannelDevice(1, _tms9914a, 0x07);
            _dmac.AttachChannelDevice(2, _z80sioA, 0x10);   // Todo: speech
            _dmac.AttachChannelDevice(3, _pdma, 0x75);
        }

        protected override void DeviceReset()
        {
            _fdc.Reset();
            _tms9914a.Reset();
            _z80sioA.Reset();
            _z80sioB.Reset();

            _dmac.Reset();
            _pdma.Reset();
            _z80ToPerqFifo.Reset();
            _perqToZ80Fifo.Reset();
        }

        protected override void DeviceShutdown()
        {
            // If serial devices are attached, close them properly
            _z80sioA.DetachDevice(0);   // RS232-A
            _z80sioB.DetachDevice(0);   // RS232-B
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public override void Run()
        {
            // Is this thing on?
            if (!_running)
            {
                if (_system.Mode == ExecutionMode.Asynchronous)
                {
                    // Pause the thread until the Z80 is turned back on
                    _wakeup = long.MaxValue;
                    _sync.Reset();
                }
                return;
            }

            // Is the master CPU clock ahead of us?
            var diff = (long)(_system.Scheduler.CurrentTimeNsec - _scheduler.CurrentTimeNsec);

            if (diff <= 0)
            {
                if (_system.Mode == ExecutionMode.Asynchronous)
                {
                    // If we are less than one full microcycle ahead of the CPU,
                    // just spin; otherwise, block (when we return).  The PERQ
                    // will wake us when it catches up.  The faster 4Mhz EIO Z80
                    // still takes ~13-80 PERQ microcycles to execute a complete
                    // instruction (using the typical 9-55 clocks per inst metric).
                    diff = -diff;

                    if ((ulong)diff > _system.Scheduler.TimeStepNsec)
                    {
                        // Pause the thread
                        _sync.Reset();
                    }
                }
                return;
            }

#if DEBUG
            var brk = false;

            // Breakpoint set at this address?
            if (_z80Debugger.WatchedInstructionAddr.IsWatched(_cpu.Registers.PC))
            {
                brk = _z80Debugger.WatchedInstructionAddr.BreakpointReached(_cpu.Registers.PC);
            }
            // Todo: if we break/return here before execution, we'll loop indefinitely
            // on the next call unless the breakpoint doesn't auto-reset (not good);
            // but if we execute the IRQ select, run the instruction, clock a DMA cycle
            // AND run the scheduler before stopping, then the PC might be well beyond
            // the state we wanted to catch.  The annoying Z80dotNet way of doing things
            // does at least let you specifiy before or after execution... sigh.  Think
            // on this and figure out if "return _break" (or just setting _break and
            // having the caller check it?) works well enough
#endif

            // No WAIT line INIR/OTIR shenanigans on the EIO!  However, slightly
            // more sophisticated interrupt handling is required: we look for the
            // RETI instruction and let the Am9519 acknowledge completion of the
            // current IRQ.  There are improvements to Z80dotNet in v1.0.7 that
            // eliminate this hack, but it's time to consider a stripped down,
            // purpose-built homegrown Z80 that better suits us (like, allowing
            // for the DMA chip to request bus cycles).
            var peek = (ushort)(_memory[_cpu.Registers.PC] << 8 |
                                _memory[_cpu.Registers.PC + 1]);

            // Update the interrupt controller's state.  Do this separately so
            // that debug commands from the CLI don't affect the flags?
            _irqControl.Clock();

            // Run the instruction
            var ticks = _cpu.ExecuteNextInstruction();

            // Did we just do a RETI?
            if (peek == 0xed4d)
            {
                // There's little harm in this if that's not an accurate decode
                // of the last instruction... since we don't have any access to
                // the simulated M1/IORQ lines it's all guesswork.  Could just
                // do "wait at least n ticks to deassert INT" instead and that
                // might be fine (as it seems to be for CIO...)

                // NB: We do this AFTER executing RETI to avoid the problem where
                // a higher priority interrupt sneaks in after the EI and before
                // the RETI, essentially nesting the interrupt and causing havoc.
                _irqControl.Acknowledge();
            }

            // Clock the EIO DMA
            ticks += _dmac.Clock();

            // Advance our wakeup time now so the CPU can chill a bit
            _wakeup = (long)(_scheduler.CurrentTimeNsec + ((ulong)ticks * IOBoard.Z80CycleTime));

            // Run the scheduler
            _scheduler.Clock(ticks);
        }

        /// <summary>
        /// Writes the control register.  This part deals with the bits relevant
        /// to the Z80:
        ///     bit 3 - Disable Ext A address (DMA, not implemented)
        ///     bit 2 - Z80 reset when clear
        ///     bit 1 - Enable write channel; interrupt when set
        ///     bit 0 - Enable read channel; interrupt when set
        /// </summary>
        public override void WriteStatus(int status)
        {
            //
            // Check the Reset bit first
            //
            if (_running && ((status & 0x04) == 0))
            {
                Log.Debug(Category.Z80, "Shut down by write to Status register");
                _running = false;
                _system.MachineStateChange(WhatChanged.Z80RunState, _running);
            }
            else if (!_running && ((status & 0x04) != 0))
            {
                Log.Debug(Category.Z80, "Started by write to Status register");
                Reset(true);
                _system.MachineStateChange(WhatChanged.Z80RunState, _running);
            }

            // Note: if bit 3 set, DMA address generation is disabled for the
            // ExtA channel, which then drives the MADR lines directly.  So far
            // none of the known optional I/O devices use that functionality.
            // Audre?  MLO?  It seems very unlikely that emulating this feature
            // will ever be needed.

            // Set the enable bits
            _perqToZ80Fifo.InterruptEnabled = ((status & 0x02) != 0);
            _z80ToPerqFifo.InterruptEnabled = ((status & 0x01) != 0);

            // debug
            Log.Debug(Category.FIFO, "PERQ wrote FIFO interrupt enables 0x{0:x4}", status);
        }

        /// <summary>
        /// Reads the status register.  Corresponds to IOA 125 (0x55).  The valid
        /// bits reflect the status of the FIFOs:
        ///     bit 15 - Read data ready when set       (i.e., data present)
        ///     bit 7  - Write channel ready when set   (i.e., FIFO is empty)
        /// </summary>
        public override int ReadStatus()
        {
            int status = (_z80ToPerqFifo.IsReady ? 0x8000 : 0);     // Bit 15: read ready
            status |= (_perqToZ80Fifo.IsReady ? 0x0080 : 0);        // Bit 7: write ready

            Log.Debug(Category.FIFO, "PERQ read FIFO status 0x{0:x4}", status);
            return status;
        }

        /// <summary>
        /// Writes a byte to the Z80's input FIFO.
        /// </summary>
        public override void WriteData(int data)
        {
            _perqToZ80Fifo.Enqueue(data & 0xff);
        }

        /// <summary>
        /// Reads a byte from the Z80's output FIFO.
        /// </summary>
        public override int ReadData()
        {
            return _z80ToPerqFifo.Dequeue();
        }

        /// <summary>
        /// Pass keyboard input from the host to the emulated device.
        /// </summary>
        public override void QueueKeyboardInput(byte keyCode)
        {
            _keyboard.QueueInput(keyCode);
        }

        // debug
        public override void DumpFifos()
        {
            _z80ToPerqFifo.DumpFifo();
            _perqToZ80Fifo.DumpFifo();
            _pdma.DumpFIFOs();
        }

        public override void DumpPortAStatus()
        {
            _z80sioA.DumpPortStatus(0);
        }

        public override void DumpPortBStatus()
        {
            _z80sioB.DumpPortStatus(0);
        }

        public override void DumpIRQStatus()
        {
            _bus.DumpInterrupts();
            _irqControl.DumpStatus();
        }

        public override void DumpDMAStatus()
        {
            _dmac.DumpStatus();
        }

        //
        // EIO/NIO boards
        //
        Am9519 _irqControl;
        Z80SIO _z80sioA, _z80sioB;
        i8254PIT _timerA, _timerB;
        i8237DMA _dmac;
        PERQDMA _pdma;
        Oki5832RTC _rtc;
        SerialKeyboard _keyboard;
        PERQToZ80FIFO _perqToZ80Fifo;
        Z80ToPERQFIFO _z80ToPerqFifo;
    }
}

/*
    Notes:

    Full EIO Z80 decode map:
    
    RTC     SEL CLK DATA L      167 (0x77)      Oki5832RTC
            SEL CLK CTRL L      166 (0x76)

    PDMA    SEL DMA START L     163 (0x73)      PERQDMA
            SEL DMA FLUSH L     164 (0x74)
            SEL DMA DATA        165 (0x75)      <fake!>
            UPROC WR H          172 (0x7a)

    FIFO    SEL IOD STAT L      162 (0x72)      Z80ToPERQFIFO
            SEL IOD WR L        161 (0x71)
            IOD OUT RDY         170 (0x78)

            SEL IOD RD L        160 (0x70)      PERQToZ80FIFO

    GPIB    GPIB PE             174 (0x7c)      TMS9914A
            GPIB SC             173 (0x7b)
            SEL GPIB L          0:17 (0x0)      (8 used?)

    SIOA    SEL SIO A L         20:37 (0x10)    Z80SIO (4 used)
    SIOB    SEL SIO B L         100:137 (0x40)  Z80SIO (4 used)

    SIO?    SPEECH SEL L        171 (0x79)      <tbd>

    CTCA    SEL CTC A L         120:127 (0x50)  i8254PIT (4 used?)
    CTCB    SEL CTC B L         130:137 (0x54)  i8254PIT (4 used?)

    FDC     SEL FLOPPY L        40:57 (0x20)    NECuPD765A (2 used)

    IRQ     SEL INT L           140:157 (0x60)  Am9519 (2 used)

    DMAC    SEL DMA L           60:77 (0x30)    i8237DMA (16 used)

 */
