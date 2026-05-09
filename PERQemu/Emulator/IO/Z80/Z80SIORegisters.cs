//
// Z80SIORegisters.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.IO.Ports;

namespace PERQemu.IO.Z80
{
    public partial class Z80SIO : IZ80Device, IDMADevice
    {
        /// <summary>
        /// Manages all of the bit fiddling for the SIO chip's CSRs.  Each channel
        /// has 8 write registers (WR0..WR7) and three read registers (RR0..RR2).
        /// </summary>
        /// <remarks>
        /// This is entirely silly but the sprawling mess was getting more and more
        /// complicated as additional modes and features of the complex SIO chip
        /// were added.  I may live to regret this.
        /// </remarks>
        internal class Registers
        {
            public Registers(int channelNum)
            {
                _channel = channelNum;      // For logging, mostly

                _write = new byte[8];
                _read = new byte[3];
            }

            public void Reset()
            {
                for (var i = 0; i < _write.Length; i++)
                    _write[i] = 0;

                for (var i = 0; i < _read.Length; i++)
                    _read[i] = 0;

                _selected = 0;

                _huntMode = true;
                _syncMode = false;

                Log.Debug(Category.SIO, "Channel {0} registers reset", _channel);
            }

            public void ErrorReset()
            {
                // This specfically sets the bits in RR1:
                //      D0      AllSent is not modified
                //      D3:1    Residual bits cleared (not implemented here)
                //      D7:4    Cleared (but this is only sort of correct)
                _read[1] = SetBit(_read[1], (byte)~RR1.AllSent, false);
            }

            public int Selected
            {
                get { return _selected; }
                set { _selected = value; }
            }

            public bool RxCharAvailable
            {
                get { return (_read[0] & (byte)RR0.RxCharAvail) != 0; }
                set { _read[0] = SetBit(_read[0], (byte)RR0.RxCharAvail, value); }
            }

            public bool TxBufferEmpty
            {
                get { return (_read[0] & (byte)RR0.TxBufferEmpty) != 0; }
                set { _read[0] = SetBit(_read[0], (byte)RR0.TxBufferEmpty, value); }
            }

            public bool IntPending
            {
                get { return _channel == 1 ? false : (_read[0] & (byte)RR0.IntPending) != 0; }
                set { _read[0] = SetBit(_read[0], (byte)RR0.IntPending, value); }
            }

            public bool AllSent
            {
                get { return (_read[1] & (byte)RR1.AllSent) != 0; }
                set { _read[1] = SetBit(_read[1], (byte)RR1.AllSent, value); }
            }

            public bool ParityError
            {
                get { return (_read[1] & (byte)RR1.ParityError) != 0; }
                set { _read[1] = SetBit(_read[1], (byte)RR1.ParityError, value); }
            }

            public bool DCDState
            {
                get { return (_read[0] & (byte)RR0.DCDState) != 0; }
                set { _read[0] = SetBit(_read[0], (byte)RR0.DCDState, value); }
            }

            public bool CTSState
            {
                get { return (_read[0] & (byte)RR0.CTSState) != 0; }
                set { _read[0] = SetBit(_read[0], (byte)RR0.CTSState, value); }
            }

            public bool HuntMode
            {
                get { return _huntMode; }
                set
                {
                    _huntMode = value;
                    _read[0] = SetBit(_read[0], (byte)RR0.SyncHunt, !_huntMode);
                }
            }

            public bool AutoEnables => (_write[3] & (byte)WR3.AutoEnables) != 0;

            public bool RxEnabled => AutoEnables ? DCDState : (_write[3] & (byte)WR3.RxEnable) != 0;
            public bool TxEnabled => AutoEnables ? CTSState : (_write[5] & (byte)WR5.TxEnable) != 0;

            public bool SyncMode => _syncMode;
            public byte RxSyncByte => _write[7];
            public byte TxSyncByte => _write[6];

            public int RxBits => _rxBits;
            public int TxBits => _txBits;
            public Parity Parity => _parity;
            public StopBits StopBits => _stopBits;

            public bool DTRState => (_write[5] & (byte)WR5.DTR) != 0;
            public bool RTSState => (_write[5] & (byte)WR5.RTS) != 0;
            public bool SendBreak => (_write[5] & (byte)WR5.SendBreak) != 0;

            // Interrupt related bits
            public bool IntOnFirstRxChar => _rxInt == RxIntEnables.OnFirstChar;
            public bool IntOnAllRxChars => (_rxInt == RxIntEnables.OnAll) || ParityAffectsVector;
            public bool ParityAffectsVector => _rxInt == RxIntEnables.OnAllParityAffectsVector;

            public bool TxIntEnabled => (_write[1] & (byte)WR1.TxIntEnable) != 0;
            public bool ExtIntEnabled => (_write[1] & (byte)WR1.ExtIntEnable) != 0;
            public bool StatusAffectsVector => (_write[1] & (byte)WR1.StatusAffectsVector) != 0;


            /// <summary>
            /// Read the raw contents of the currently selected register.
            /// </summary>
            public byte Read()
            {
                return Read(_selected);
            }

            public byte Read(int reg)
            {
                // Range check: RR2 only exists on the B channel
                if (reg > _read.Length || (reg == 2 && _channel == 0))
                {
                    Log.Warn(Category.SIO, "Channel {0} read from invalid register {1}",
                                            _channel, reg);
                    return 0;
                }

                byte value = _read[reg];
                Log.Detail(Category.SIO, "Channel {0} read 0x{1:x2} from register {2}",
                                                         _channel, value, reg);
                return value;
            }

            /// <summary>
            /// Write a raw value to the currently selected register.
            /// </summary>
            public void Write(byte value)
            {
                if (_selected > _write.Length)
                {
                    Log.Warn(Category.SIO, "Channel {0} write to invalid register {1}",
                                            _channel, _selected);
                    return;
                }

                Log.Debug(Category.SIO, "Channel {0} write 0x{1:x2} to register {2}",
                        _channel, value, _selected);

                _write[_selected] = value;

                // Handle special cases for bits which trigger actions when set:
                switch (_selected)
                {
                    case 0:
                        // Commands handled by the Channel; no action here
                        break;

                    case 1:
                        // Bits D4..D3 are the many receive interrupt flags
                        _rxInt = (RxIntEnables)(_write[1] & RxIntEnableMask);

                        // The WAIT/READY functions may affect DMA operation?
                        break;

                    case 2:
                        // Copy interrupt vector into RR2 (Channel B only!)
                        if (_channel == 1)
                        {
                            _read[2] = _write[2];
                        }
                        break;

                    case 3:
                        // Re-enter hunt mode?
                        if ((_write[3] & (byte)WR3.EnterHuntPhase) != 0)
                        {
                            HuntMode = true;    // Set status bit in RR0
                        }

                        // Set the receive bits per char
                        _rxBits = BitsPerChar((_write[3] & RxBitsMask) >> 6);
                        break;

                    case 4:
                        // Update parity
                        var enable = (_write[4] & (byte)WR4.ParityEnable) != 0;
                        var polarity = (_write[4] & (byte)WR4.ParityEvenOdd) >> 1;
                        _parity = (!enable ? Parity.None : (polarity == 1) ? Parity.Even : Parity.Odd);

                        // Update stop bits, mapping from the raw SIO bits to the
                        // SerialPort enum (that later gets mapped again), sigh
                        var sb = (RegStopBits)(_write[4] & StopBitsMask);
                        _stopBits = ((sb == RegStopBits.One) ? StopBits.One :
                                     (sb == RegStopBits.Two) ? StopBits.Two :
                                     (sb == RegStopBits.OnePointFive) ? StopBits.OnePointFive : StopBits.None);

                        // Sync mode?
                        var mode = (SyncModes)(_write[4] & SyncModesMask);
                        _syncMode = (sb == RegStopBits.SyncModesEnable);

                        if (_syncMode && mode != SyncModes.EightBit)
                            throw new NotImplementedException($"Sync mode {mode}");

                        break;

                    case 5:
                        // Parse the Tx bits per char
                        _txBits = BitsPerChar((_write[5] & TxBitsMask) >> 5);
                        break;

                    case 6:
                    case 7:
                        // WR6 is the sync character in 8 bit "monosync" mode
                        // WR7 is the high 8 bits of the sync word in "bisync" mode
                        break;
                }
            }

            /// <summary>
            /// Return a byte with a bit/field set or cleared.
            /// </summary>
            byte SetBit(byte reg, byte mask, bool value)
            {
                if (!value)
                    return (byte)(reg & ~(mask));

                return (byte)(reg | (mask & 0xff));
            }

            /// <summary>
            /// Maps the SIO's register bits to the actual bits-per-char.  Or as
            /// the "helpful" AI getting in the way of everything damn thing I type
            /// says, "Bitses the per char."
            /// </summary>
            int BitsPerChar(int bits)
            {
                // Why, Zilog.  Why.
                return ((bits == 3) ? 8 :
                        (bits == 1) ? 7 :
                        (bits == 2) ? 6 : 5);
            }

            // Debugging
            public void DumpStatus()
            {
                var cmd = (Command)(_write[0] & CmdMask);
                var rst = (Resets)(_write[0] & ResetMask);

                Console.WriteLine($"  Channel {_channel} registers:");
                Console.WriteLine($"    WR0: 0x{_write[0]:x2}  Select={_selected} Cmd={cmd} Rst={rst}");
                Console.WriteLine($"    WR1: 0x{_write[1]:x2}  RxInt={_rxInt}, {(WR1)_write[1]}");
                Console.WriteLine($"    WR3: 0x{_write[3]:x2}  RxBits={_rxBits}, {(WR3)_write[3]}");
                Console.WriteLine($"    WR4: 0x{_write[4]:x2}  Parity={_parity}, Stop={_stopBits}");
                Console.WriteLine($"    WR5: 0x{_write[5]:x2}  TxBits={_txBits}, {(WR5)_write[5]}");
                Console.WriteLine($"    WR6: 0x{_write[6]:x2}  (Sync 7:0)  WR7: 0x{_write[7]:x2}  (Sync 15:8)");

                Console.WriteLine($"    RR0: 0x{_read[0]:x2}  {(RR0)_read[0]}");
                Console.WriteLine($"    RR1: 0x{_read[1]:x2}  {(RR1)_read[1]}");

                if (_channel == 1)
                    Console.WriteLine($"    RR2: 0x{_read[2]:x2}  (Vector base)");
            }


            int _channel;
            int _selected;

            bool _huntMode;
            bool _syncMode;

            byte[] _write;
            byte[] _read;

            int _rxBits;
            int _txBits;
            Parity _parity;
            StopBits _stopBits;

            RxIntEnables _rxInt;
        }

        //
        // Read registers
        //

        [Flags]
        enum RR0 : byte
        {
            RxCharAvail = 0x1,
            IntPending = 0x2,
            TxBufferEmpty = 0x4,
            DCDState = 0x8,
            SyncHunt = 0x10,
            CTSState = 0x20,
            TxUnderrun = 0x40,
            BreakAbort = 0x80
        }

        [Flags]
        enum RR1 : byte
        {
            AllSent = 0x1,
            Residue = 0x0e,
            ParityError = 0x10,
            RxOverrun = 0x20,
            CrcFraming = 0x40,
            EndOfFrame = 0x80
        }

        //
        // Write registers
        //

        const byte RegSelMask = 0x7;            // WR0
        const byte CmdMask = 0x38;
        const byte ResetMask = 0xc0;
        const byte RxIntEnableMask = 0x18;      // WR1
        const byte RxBitsMask = 0xc0;           // WR3
        const byte StopBitsMask = 0x0c;         // WR4
        const byte SyncModesMask = 0x30;
        const byte ClockMask = 0xc0;
        const byte TxBitsMask = 0x60;           // WR5

        // WR0
        enum Command
        {
            NullCode = 0,
            SendAbort = 0x08,
            ResetExtStatusInt = 0x10,
            ChannelReset = 0x18,
            EnableIntOnRx = 0x20,
            ResetTxInt = 0x28,
            ErrorReset = 0x30,
            ReturnFromInt = 0x38
        }

        // WR0
        enum Resets
        {
            NullCode = 0,
            ResetRxCRC = 0x40,
            ResetTxCRC = 0x80,
            ResetTxUnderrun = 0xc0
        }

        [Flags]
        enum WR1 : byte
        {
            ExtIntEnable = 0x1,
            TxIntEnable = 0x2,
            StatusAffectsVector = 0x4,
            RxIntBits = 0x18,
            WaitReadyOnRT = 0x20,
            WaitReadyFunction = 0x40,
            WaitReadyEnable = 0x80
        }

        // WR1
        public enum RxIntEnables
        {
            Disable = 0,
            OnFirstChar = 0x08,
            OnAllParityAffectsVector = 0x10,
            OnAll = 0x18
        }

        [Flags]
        enum WR3 : byte
        {
            RxEnable = 0x1,
            SyncCharLoadInhibit = 0x2,
            AddressSearchMode = 0x4,
            RxCRCEnable = 0x8,
            EnterHuntPhase = 0x10,
            AutoEnables = 0x20,
            RxDataBits = 0xc0
        }

        [Flags]
        enum WR4 : byte
        {
            ParityEnable = 0x01,
            ParityEvenOdd = 0x02,
            StopBits = 0x0c,
            SyncBits = 0x30,
            ClkBits = 0xc0
        }

        // WR4
        enum RegStopBits
        {
            SyncModesEnable = 0,
            One = 0x04,
            OnePointFive = 0x08,
            Two = 0x0c
        }

        // WR4
        enum SyncModes
        {
            EightBit = 0,
            SixteenBit = 0x10,
            SDLCMode = 0x20,
            ExtSyncMode = 0x30
        }

        // WR4
        enum ClockMode
        {
            X1 = 0,
            X16 = 0x40,
            X32 = 0x80,
            X64 = 0xc0
        }

        [Flags]
        enum WR5 : byte
        {
            TxCRCEnable = 0x1,
            RTS = 0x2,
            SDLC = 0x4,
            TxEnable = 0x8,
            SendBreak = 0x10,
            TxBits = 0x60,
            DTR = 0x80
        }
    }
}
