//
// i8254PIT.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Intel i8254 Programmable Interval Timer.
    /// </summary>
    /// <remarks>
    /// This chip provides baud rate clocks for the RS-232 ports and speech device
    /// on the EIO board.  It replaces the Z80 CTC used on the older IOB/CIO board.
    /// 
    /// Two chips are used to provide six timers (baud rate clocks):
    ///     Timer   Chn
    ///       A      0      RS232 A Receive
    ///       A      1      Speech Transmit / Tablet Receive
    ///       A      2      RS232 A Transmit
    ///       B      0      RS232 B Receive
    ///       B      1      Keyboard Transmit/Receive
    ///       B      2      RS232 B Transmit
    /// 
    /// Operating modes 2, 4 and 5 are not used by the PERQ's Z80 firmware, and
    /// there's no way to program the PIT from Pascal, so those aren't simulated
    /// here.  These are used almost exclusively as baud rate generators in mode
    /// 3 (square wave) but are initially programmed in Mode 0.
    /// </remarks>
    public class i8254PIT : IZ80Device
    {
        public i8254PIT(byte baseAddress, string unit)
        {
            _unit = unit;
            _baseAddress = baseAddress;
            _ports = new byte[] {
                                    _baseAddress,
                                    (byte)(_baseAddress + 1),
                                    (byte)(_baseAddress + 2),
                                    (byte)(_baseAddress + 3)
                                };

            // Set up the channels, 3 per device
            _channels = new Channel[] {
                new Channel(0, this),
                new Channel(1, this),
                new Channel(2, this)
            };
        }

        public string Name => $"i8254 PIT {_unit}";
        public string Unit => _unit;
        public byte[] Ports => _ports;

        public bool IntLineIsActive => false;       // Doesn't interrupt on EIO
        public byte? ValueOnDataBus => null;

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        public void Reset()
        {
            Log.Debug(Category.CTC, "{0} reset", Name);
        }

        public void AttachDevice(int channel, ICTCDevice dev)
        {
            _channels[channel].Client = dev;
        }

        public byte Read(byte portAddress)
        {
            // 
            Log.Debug(Category.CTC, "Read from 0x{0:x2}, returning 0 (unimplemented)", portAddress);
            return 0;
        }

        public void Write(byte portAddress, byte value)
        {
            // base addr + 0..2 are the counter regs; baseaddr + 3 is control

            Log.Debug(Category.CTC, "Write 0x{0:x2} to port 0x{1:x2} (unimplemented)", value, portAddress);
        }


        byte _status;
        byte _control;
        ushort _counterLatch;

        Channel[] _channels;

        string _unit;
        byte _baseAddress;
        byte[] _ports;

        internal class Channel
        {
            public Channel(int num, i8254PIT parent)
            {
                Number = num;
                Counter = 0;
                Running = false;

                _parent = parent;
                _trigger = null;
                Log.Debug(Category.CTC, "Timer {0} Channel {1} initialized", _parent.Unit, Number);
            }

            // in mode 3, loading the high byte of the counter starts the clock;
            // not loading it leaves it off/suspended
            public int Number;
            public ushort Counter;
            public bool Running;

            public ICTCDevice Client;
            public i8254PIT _parent;

            SchedulerEvent _trigger;
        }
    }
}

/*
    Notes:

    ;
    ;        COUNTER/TIMER CHIP
    ;
    CTC.ARx    equ 120Q            ; CHANNEL 0 (RS-232 Port A Receive Speed)
    CTC.Speech equ 121Q            ; CHANNEL 1 (RS-232 CH B Speech / Tablet)
    CTC.ATx    equ 122Q            ; CHANNEL 2 (RS-232 Port A Transmit Speed)
    CTCA.CSR   equ 123Q            ; Control Register
    CTC.SpSel  equ 01000000B       ; Select Speech counter
    CTC.ATxSel equ 10000000B       ; Select Port A Transmit counter
    CTC.ARxSel equ 00000000B       ; Select Port A Receive counter

    CTC.BRx    equ 124Q            ; CHANNEL 0 (RS-232 Port B Receive Speed)
    CTC.KB     equ 125Q            ; CHANNEL 1 (Keybaord Receive/Transmit Speed)
    CTC.BTx    equ 126Q            ; CHANNEL 2 (RS-232 Port B Transmit Speed)
    CTCB.CSR   equ 127Q            ; Control Register
    CTC.BTxSel equ 10000000B       ; Select Port B Transmit counter
    CTC.KBSel  equ 01000000B       ; Select Keyboard counter
    CTC.BRxSel equ 00000000B       ; Select Port B Receive counter
    CTC.M0     equ 00H             ; Select Mode 0  (Int on TC)
    CTC.M1     equ 02H             ; Select Mode 1  (One shot)
                                            Mode 2  (Rate generator)
    CTC.M3     equ 06H             ; Select Mode 3  (Square wave)
                                            Mode 4  (Software strobe)
                                            Mode 5  (Hardware strobe)
    CTC.Latch  equ 00H             ; Latch counter
    CTC.LSB    equ 10H             ; Load LSB only
    CTC.MSB    equ 20H             ; Load MSB only
    CTC.Both   equ 30H             ; Load both LSB and MSB

    EIO.doc has PIT B ports 125/126 reversed; the schematic and code above are
    correct (Tx speed is on chan 2 for both RS232 ports).  Not sure why they
    didn't just do one Tx/Rx speed on RS232B and devote separate channels to the
    tablet and speech... sigh.
    
    If 3RCC didn't define bits for Modes 2,4,5 then they probably didn't use
    them?  Each chip devotes two separate channels to the RS232 ports (separate
    Tx/Rx clocks!) and one to the second channel (keyboard, tablet/speech).  No
    interrupts are generated since those CLK pins are all hardwired directly to
    their SIO channels.  All of the 8254's GATE inputs are tied high.

*/
