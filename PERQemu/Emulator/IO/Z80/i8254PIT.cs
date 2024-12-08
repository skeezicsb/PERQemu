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
            for (int i = 0; i < 3; i++)
                _channels[i].Reset();

            Log.Debug(Category.CTC, "{0} reset", Name);
        }

        public void AttachDevice(int channel, ICTCDevice dev)
        {
            _channels[channel].TimerClient = dev;
        }

        public byte Read(byte portAddress)
        {
            // The chip can latch and return a status byte or the latched value
            // of any of its counters but the PERQ never reads back from the PIT
            throw new NotImplementedException($"{Name} read");
        }

        public void Write(byte portAddress, byte value)
        {
            // Select channels 0..2 or the control reg (3)
            int ch = (portAddress - _baseAddress);

            if (ch < 3)
            {
                // Channel count register
                _channels[ch].LoadCount(value);
            }
            else
            {
                // Control register
                Log.Detail(Category.CTC, "Control word: 0x{0:x2}", value);

                var mode = (value & 0x0e) >> 1;
                var rwCommand = (value & 0x30) >> 4;
                var counter = (value & 0xc0) >> 6;
#if DEBUG
                if ((value & 0x01) != 0)
                {
                    Log.Info(Category.CTC, "BCD mode not supported (ignored)");
                }

                if (counter == 3)
                {
                    Log.Warn(Category.CTC, "PIT Read-back command not supported");
                    return;
                }

                if (rwCommand == 0)
                {
                    Log.Warn(Category.CTC, "PIT Counter Latch command not supported");
                    return;
                }
#endif
                // Set the mode and load bits
                _channels[counter].SetMode(mode);
                _channels[counter].SetLoad((rwCommand & 0x1) == 1, (rwCommand & 0x2) != 0);
            }
        }


        Channel[] _channels;

        string _unit;
        byte _baseAddress;
        byte[] _ports;


        /// <summary>
        /// One channel of the PIT chip.
        /// </summary>
        internal class Channel
        {
            public Channel(int num, i8254PIT parent)
            {
                _parent = parent;
                _number = num;

                _ID = $"Timer {_parent.Unit} Channel {_number} ";
                Log.Debug(Category.CTC, _ID + "initialized");
            }

            public void Reset()
            {
                _counter = 0;
                _writeLSB = false;
                _writeMSB = false;

                Log.Debug(Category.CTC, _ID + "reset");
            }

            /// <summary>
            /// Sets the operating mode for the channel.
            /// </summary>
            /// <remarks>
            /// Mode 0 is only used when "external clocking" is set for the
            /// RS232 ports, but we don't support that (since it's almost never
            /// used and may not even be available on any PC hardware this runs
            /// on).  Mode 3 is the repeating square wave used for all baud rate
            /// generation; none of the others are used by the PERQ.
            /// </remarks> 
            public void SetMode(int mode)
            {
                if (mode != _mode)
                {
                    if (mode != 0 && mode != 3)
                    {
                        Log.Debug(Category.CTC, _ID + "mode {0} not supported (ignored)", mode);
                        return;
                    }

                    _mode = mode;
                    Log.Debug(Category.CTC, _ID + "mode now {0}", _mode);
                }
            }

            /// <summary>
            /// Bloody Intel.  Programming each half of the 16-bit counter must
            /// be set up by writing a command to the control register which says
            /// which byte (or "both") can be written on subsequent writes (that
            /// may be separated by an arbitrary length of time or activity on
            /// other channels).  So cache the intent here in the channel and
            /// update the counter on subsequent writes to the channel register.
            /// </summary>
            public void SetLoad(bool low, bool high)
            {
                _writeLSB = low;
                _writeMSB = high;
            }

            /// <summary>
            /// Write a byte to the counter, depending on how the control reg was
            /// set up earlier.
            /// </summary>
            public void LoadCount(byte value)
            {
                if (_writeLSB)
                {
                    _counter = value;       // Zaps the value?
                    _writeLSB = false;
                    Log.Debug(Category.CTC, _ID + "counter now 0x{0:x4} ({1})", _counter, _counter);

                    // In mode 3, if we're reloading JUST the LSB then alert
                    // the client that the baud rate has changed; none of the
                    // other modes are relevant in this minimal implementation
                    if (_mode == 3 && !_writeMSB)
                    {
                        TimerClient?.NotifyRateChange(_number, _counter);
                    }
                    return;
                }

                if (_writeMSB)
                {
                    _counter = (ushort)((value << 8) | (_counter & 0xff));
                    _writeMSB = false;
                    Log.Debug(Category.CTC, _ID + "counter now 0x{0:x4} ({1})", _counter, _counter);

                    if (_mode == 3)
                    {
                        TimerClient?.NotifyRateChange(_number, _counter);
                    }
                    return;
                }

                throw new InvalidOperationException("PIT load counter unexpected");
            }

            int _number;
            int _mode;
            bool _writeLSB;
            bool _writeMSB;
            ushort _counter;
            string _ID;

            public ICTCDevice TimerClient;

            i8254PIT _parent;
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
    
    3RCC didn't define bits for Modes 2,4,5 and the standard Z80 code doesn't use
    them.  Mode 1 is defined but never referenced either.  Each chip devotes two
    separate channels to the RS232 ports (separate Tx/Rx clocks!) and one to the
    second channel (keyboard, tablet/speech).  No interrupts are generated since
    those CLK pins are all hardwired directly to their SIO channels.  All of the
    8254's GATE inputs are tied high.  So all we implement is Mode 3 and only
    inform the client device if the baud rate changes -- no need to schedule any
    timer callbacks or accept external triggers like the IOB/CTC.

*/
