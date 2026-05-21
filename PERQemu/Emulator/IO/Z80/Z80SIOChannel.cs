//
// Z80SIOChannel.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using System.Collections.Generic;

using PERQemu.IO.Ports;
using PERQemu.IO.SerialDevices;

namespace PERQemu.IO.Z80
{
    public partial class Z80SIO : IZ80Device, IDMADevice
    {
        /// <summary>
        /// One Z80 SIO Channel.  They're not _quite_ identical: only channel 1
        /// ("SIO B") holds the interrupt vector in its WR2/RR2 registers!
        /// </summary>
        internal class Channel
        {
            public Channel(int channelNumber, Scheduler scheduler)
            {
                _channelNum = channelNumber;
                _scheduler = scheduler;

                _registers = new Registers(_channelNum);

                _rxFifo = new Queue<byte>(4);
                _txFifo = new Queue<byte>(2);

                _device = null;
            }

            /// <summary>
            /// Reset this instance and any attached device (hard reset).
            /// </summary>
            public void Reset()
            {
                _scheduler.Cancel(_pollEvent);
                _pollEvent = null;
                _nextPoll = 0;

                _rxFifo.Clear();
                _txFifo.Clear();
                _nextCharRx = _nextCharTx = 0;

                _registers.Reset();

                _extIntLatched = false;
                _txIntLatched = false;
                _rxIntLatched = false;
                _rxIntOnNextChar = false;

                _breakSent = false;
                _breakDetected = false;

                if (_device != null)
                {
                    if (!_device.IsOpen) _device.Open();
                    _device.Reset();
                }

                UpdateFlags();

                Log.Info(Category.SIO, "Channel {0} ({1}) reset", _channelNum, _device?.Name);
            }

            public SerialDevice Port => _device;

            public bool CanRead => _rxFifo.Count > 0;
            public bool CanWrite => _txFifo.Count < 2;

            public bool InterruptLatched => _rxIntLatched || _txIntLatched || _extIntLatched;
            public bool StatusAffectsVector => _registers.StatusAffectsVector;
            public byte InterruptBase => _registers.Read(2);
            public int InterruptOffset => _interruptOffset;

            bool RxIntEnabled => _registers.IntOnAllRxChars ||
                                (_registers.IntOnFirstRxChar && _rxIntOnNextChar);


            /// <summary>
            /// Attach a serial device.  Defers the Open() call until Reset.
            /// </summary>
            public void AttachDevice(SerialDevice device)
            {
                _device = device;
            }

            /// <summary>
            /// Close and detach the serial device.
            /// </summary>
            public void DetachDevice()
            {
                _device?.Close();
                _device = null;
            }

            /// <summary>
            /// Reopen and reinitialize the register programming when a device is
            /// reattached.  Will also restart the polling event as appropriate.
            /// </summary>
            /// <remarks>
            /// Rewrites the registers in the order recommended by Zilog so the
            /// newly recreated device inherits the current programming state:
            ///     WR2: Interrupt vector (chan B only)
            ///     WR6: Tx sync byte
            ///     WR7: Rx sync byte
            ///     WR4: Async mode, parity, stop bits, clock
            ///     WR3: Rx/Auto enables, Rx char bits
            ///     WR5: RTS, DTR, Tx enable, Tx char bits
            ///     WR1: Interrupt enables and flags
            /// </remarks>
            public void Reinitialize()
            {
                int[] writeOrder = { 6, 7, 4, 3, 5, 1 };
                var regs = _registers.RawWriteRegisters;

                // If this blows up...
                _device.Open();

                if (_channelNum == 1)
                {
                    _registers.Selected = 2;
                    WriteRegister(regs[2]);
                }

                foreach (int r in writeOrder)
                {
                    _registers.Selected = r;
                    WriteRegister(regs[r]);
                }
            }

            /// <summary>
            /// Read the currently selected Read register.
            /// </summary>
            public byte ReadRegister()
            {
                return _registers.Read();
            }

            /// <summary>
            /// Write into the currently selected Write register.  If the command
            /// register is selected, perform any required actions to update the
            /// channel's device and/or status.
            /// </summary>
            public void WriteRegister(byte value)
            {
                // Store the value and update register state
                _registers.Write(value);

                if (_registers.Selected == 0)
                {
                    // The primary command register
                    var cmd = (Command)(value & CmdMask);

                    Log.Debug(Category.SIO, "Channel {0} command is {1}", _channelNum, cmd);

                    switch (cmd)
                    {
                        case Command.NullCode:
                            break;

                        case Command.ResetExtStatusInt:
                            _extIntLatched = false;
                            break;

                        case Command.ResetTxInt:
                            _txIntLatched = false;
                            break;

                        case Command.ChannelReset:
                            // Whack everything...
                            Reset();
                            break;

                        case Command.ErrorReset:
                            // Clear everything but AllSent
                            _registers.ErrorReset();
                            break;

                        case Command.EnableIntOnRx:
                            _rxIntOnNextChar = _registers.IntOnFirstRxChar;
                            break;

                        //case Command.ReturnFromInt:
                        //case Command.SendAbort:
                        default:
                            throw new NotImplementedException($"SIO command {cmd}");
                    }

                    // The CRC reset/Tx underrun/EOM latch isn't implemented
                    // (These are non-fatal; just log if they're being used!)
                    var crc = (Resets)(value & ResetMask);

                    if (crc != Resets.NullCode)
                        Log.Info(Category.SIO, "Channel {0} reset command {1} not implemented",
                                               _channelNum, crc);

                    // Select register pointer
                    _registers.Selected = (value & RegSelMask);

                    Log.Detail(Category.SIO, "Channel {0} register pointer now {1}",
                                             _channelNum, _registers.Selected);
                    return;
                }

                // See if any changes need to be applied to the device
                switch (_registers.Selected)
                {
                    case 3:
                        // See notes about the separate Tx/Rx bits-per-char problem
                        UpdateBitsPerChar(_registers.RxBits);

                        // If the Rx enable bit changed
                        CheckPollEnable();
                        break;

                    case 4:
                        // Apply changes to parity and stop bits
                        UpdateParity();
                        UpdateStopBits();
                        break;

                    case 5:
                        // Update the Tx bits
                        UpdateBitsPerChar(_registers.TxBits);

                        // Update output pins: DTR tracks the register bit
                        _device.DTR = _registers.DTRState;

                        // RTS only tracks the register in sync modes
                        if (_registers.SyncMode) _device.RTS = _registers.RTSState;

                        // Change in break state?
                        if (_registers.SendBreak != _breakSent)
                        {
                            // Docs say that the SendBreak bit sets the line to
                            // spacing "regardless of any data being transmitted"
                            // (and presumably, independent of the TxEnable bit
                            // as well).
                            _breakSent = _registers.SendBreak;
                            _device.TransmitBreak(_breakSent);
                        }

                        // If the Tx enable bit changed
                        CheckPollEnable();
                        break;

                        // Writes to WR1, WR2, WR6 and WR7 don't require action
                }

                // Next access is to reg 0
                _registers.Selected = 0;
            }

            /// <summary>
            /// Read the next byte from the Rx FIFO.
            /// </summary>
            public byte ReadData()
            {
                byte data = 0;

                if (_rxFifo.Count == 0)
                {
                    Log.Debug(Category.SIO, "Channel {0} read from empty FIFO", _channelNum);
                    return data;
                }

                data = _rxFifo.Dequeue();

                // Update interrupt status
                _rxIntLatched = (_rxFifo.Count > 0 && RxIntEnabled);
                _rxIntOnNextChar = false;

                UpdateFlags();

                Log.Debug(Category.SIO, "Channel {0} read data 0x{1:x2}, {2} remaining",
                                        _channelNum, data, _rxFifo.Count);
                return data;
            }

            /// <summary>
            /// Accept a byte from the SIO and schedule it for transmission.
            /// </summary>
            public void WriteData(byte data)
            {
                // The CanWrite flag shouldn't allow this; drop the byte?
                if (_txFifo.Count > 1)
                {
                    Log.Info(Category.SIO, "Channel {0} write to full FIFO ignored", _channelNum);
                    return;
                }

                _txFifo.Enqueue(data);
                _txIntLatched = false;

                // In async mode, RTS is asserted when there's data to send
                if (!_registers.SyncMode) _device.RTS = true;

                UpdateFlags();

                Log.Debug(Category.SIO, "Channel {0} write data 0x{1:x2}, {2} pending",
                                        _channelNum, data, _txFifo.Count);
            }

            /// <summary>
            /// In monosync mode, return true if programmed for "hunt mode" and
            /// we're still waiting to match the sync byte.  Return false otherwise.
            /// The matched sync bytes are not passed through!
            /// </summary>
            /// <remarks>
            /// Only one 8-bit sync byte is matched; other modes aren't implemented.
            /// </remarks>
            bool Hunting(byte data, byte match)
            {
                if (_registers.HuntMode)                // Looking for sync byte(s)
                {
                    if (data == match)                  // 8-bit sync value
                    {
                        _registers.HuntMode = false;    // Exit hunt mode

                        Log.Debug(Category.SIO, "Channel {0} sync word matched", _channelNum);
                    }

                    return true;    // Consume the sync byte
                }

                // Sync byte was matched, so we're in data mode
                return false;
            }

            /// <summary>
            /// Update the read registers and interrupt status bits.
            /// </summary>
            void UpdateFlags()
            {
                const byte pinMask = (byte)(RR0.DCDState | RR0.CTSState);

                // Save the state of the pins before we update 'em
                var oldPinState = _registers.Read(0);

                // Update char status bits
                _registers.RxCharAvailable = _rxFifo.Count > 0;
                _registers.TxBufferEmpty = _txFifo.Count == 0;

                // Update modem control pins
                _registers.DCDState = _device.DCD;
                _registers.CTSState = _device.CTS;

                // TODO: we aren't detecting breaks (yet?); it's not clear termios/mono
                // will even pass them to us?  But they can be detected by watching the
                // line status... can o' worms.  Also, handling them requires interrupts
                // at each transition, so this is high effort for low/no reward...
                //_readRegs[0] |= (byte)(_breakDetected ? RR0.BreakAbort : 0);

                // If they changed, latch the external/status interrupt (if enabled)
                if ((_registers.Read(0) & pinMask) != (oldPinState & pinMask))
                {
                    _extIntLatched = _registers.ExtIntEnabled;
                }

                // If no interrupts pending, the Z80 returns V3..V1 = 011.  So
                // regardless of whichever channel we are, set up the default...
                // ... except I don't think it works; the Z80 ROM doesn't seem to
                // handle that default case.  Let's set it to zero and see if just
                // giving back the base vector works better?  Sigh.
                _interruptOffset = 0;

                // Now update the offset and interrupts bits in lowest-to-highest
                // priority order, and account for the channel # (A > B).  First,
                // did the tx buffer just become empty?
                if (_txFifo.Count == 0 && _txIntLatched)
                {
                    _interruptOffset = 0;
                }

                // If one of the modem control pins changed state (DCD or CTS),
                // bump the offset ("external/status interrupt")
                if (_extIntLatched)
                {
                    _interruptOffset = 1;
                }

                // Receive character available?
                if (_rxIntLatched)
                {
                    _interruptOffset = 2;
                }

                // Errors received will latch the bits in RR1; just update a few
                // status bits if they've changed
                _registers.AllSent = (_registers.SyncMode ||
                                     (!_registers.SyncMode && _txFifo.Count == 0));

                // If an error occurred ("special receive conditions") that we
                // care about, set the offset...
                if (((_registers.Read(1) & 0xf0) != 0) && _rxIntLatched)
                {
                    // Except... the silly mode where Parity Errors aren't "special"
                    // (so the offset should already have been set, above)
                    var parErr = _registers.ParityError;

                    if (!parErr || (parErr && _registers.ParityAffectsVector))
                    {
                        _interruptOffset = 3;
                    }

                    // TODO: There's probably a LOT of finicky error handling detail that
                    // currently is unhandled; framing, parity, CRC, over/underruns...
                }

                // Finally: update the interrupt pending bit in RR0.  The SIO only updates
                // this bit in channel A's RR0 but it takes into account the status from
                // both channels?  The PERQ may or may not even rely on this bit...
                _registers.IntPending = InterruptLatched;

                Log.Detail(Category.SIO, "Channel {0} RR0 = {1}", _channelNum, (RR0)_registers.Read(0));
                Log.Detail(Category.SIO, "Channel {0} RR1 = {1}", _channelNum, (RR1)_registers.Read(1));
                Log.Detail(Category.SIO, "Channel {0} IRQ enable: Rx {1}, Tx {2}, Ext {3}",
                           _channelNum, RxIntEnabled, _registers.TxIntEnabled, _registers.ExtIntEnabled);
                Log.Detail(Category.SIO, "Channel {0} IRQ status: Rx {1}, Tx {2}, Ext {3}, Vec {4}",
                           _channelNum, _rxIntLatched, _txIntLatched, _extIntLatched, _interruptOffset);
            }

            /// <summary>
            /// Detect a change in the programmed number of data bits per char,
            /// and if necessary issue a change to the connected serial device.
            /// </summary>
            /// <remarks>
            /// The Z80 SIO allows _separate_ sizes for Rx and Tx, which is kind
            /// of nuts.  Given the split-brained approach where the Settings for
            /// the host port are separate from the PERQ's view, all we really do
            /// here is update the virtual state and reflect it back in the status
            /// registers.  But... should we actually mask off bits for every byte 
            /// transmitted or received!?  (Yeah, probably?  Maybe?  Hmm.)
            /// </remarks>
            void UpdateBitsPerChar(int bits)
            {
                if (bits != _device.DataBits)
                {
                    _device.DataBits = bits;

                    Log.Debug(Category.SIO, "Channel {0} bits per character now {1}",
                                            _channelNum, bits);
                }
            }

            /// <summary>
            /// Changes the port's parity setting in response to register updates.
            /// </summary>
            void UpdateParity()
            {
                if (_registers.Parity != _device.Parity)
                {
                    _device.Parity = _registers.Parity;

                    Log.Debug(Category.SIO, "Channel {0} parity now {1}",
                                            _channelNum, _registers.Parity);
                }
            }

            /// <summary>
            /// Pull out all the stop bits.
            /// </summary>
            void UpdateStopBits()
            {
                // None is illegal (used to indicate Sync Mode); don't pass thru
                if (_registers.StopBits == StopBits.None) return;

                // Update the device if the setting changed
                if (_registers.StopBits != _device.StopBits)
                {
                    _device.StopBits = _registers.StopBits;

                    Log.Debug(Category.SIO, "Channel {0} stop bits now {1}",
                                            _channelNum, _registers.StopBits);
                }
            }

            /// <summary>
            /// Check for changes in the device settings that aren't programmed
            /// by the PERQ/Z80.  This allows changing the synthetic DCD Option
            /// or Handshaking flags; other changes should be done (for now) by
            /// disabling and re-enabling the port...
            /// </summary>
            public void UpdateSettings(SerialSettings settings)
            {
                // Todo: push other device changes and/or just assign the struct
                // and do a forced Close()/Open() to reinit?
                _device.FlowControl = settings.FlowControl;
                _device.Options = settings.Options;
            }


            /// <summary>
            /// Start the polling event if a port device is attached, the loop is
            /// not already running, and one/both of the Rx/Tx enable bits are set.
            /// </summary>
            public void CheckPollEnable()
            {
                // Have a device?
                if (_device == null) return;

                // Already polling?
                if (_pollEvent != null) return;

                // Let 'er rip
                Log.Detail(Category.SIO, "Channel {0} check: polling={1} rate={2} rx={3}/{4} tx={5}/{6}",
                                         _channelNum, (_pollEvent != null), _device.PollRate,
                                         _registers.RxEnabled, _device.ReceiveRate,
                                         _registers.TxEnabled, _device.TransmitRate);
                PollDevice(0, null);
            }

            /// <summary>
            /// Poll the attached device and reschedule.  Updates flags if the
            /// signals or state of the fifos changes.
            /// </summary>
            /// <remarks>
            /// The goal here is to consolidate polling into one periodic event
            /// per device.  Poll/PollRate is for updating signals and/or moving
            /// data from the SIO to a real host device; Recv/RecvRate and Xmit/
            /// XmitRate allow characters to be paced according to the baud rate
            /// currently in effect (they can be changed by the Z80 at any time)
            /// and/or as the Tx/Rx enable flags change.  This kinda smells bad.
            /// But the performance impact of the polled approach so far seems
            /// minimal, so for now I'll live with it.  Hmm.
            /// </remarks>
            void PollDevice(ulong skewNsec, object context)
            {
                // Sanity check
                if (_device == null)
                {
                    Log.Debug(Category.SIO, "Channel {0} polling stopped (no device!)", _channelNum);
                    _pollEvent = null;
                    return;
                }

                byte data;
                ulong delay = 0;

                var updateNeeded = false;
                var now = _scheduler.CurrentTimeNsec;

                // Poll the device first, if requested
                if ((_device.PollRate > 0) && (_nextPoll <= now))
                {
                    updateNeeded = _device.Poll();
                    _nextPoll = now - skewNsec + _device.PollRate;
                }

                // Compute offset to next event
                if (_nextPoll > now) delay = _nextPoll - now;

                // Check the receiver
                if ((_device.ReceiveRate > 0) && _registers.RxEnabled)
                {
                    // Time to read another char?
                    if (_nextCharRx <= now)
                    {
                        // Apply Auto Enables logic here
                        if (_device.ReadReady &&
                            (!_registers.AutoEnables || (_registers.AutoEnables && _device.DCD)) &&
                            (_rxFifo.Count < 4))
                        {
                            data = _device.Receive();

                            // Still in hunt mode?
                            if (!_registers.SyncMode ||
                                (_registers.SyncMode && !Hunting(data, _registers.RxSyncByte)))
                            {
                                // Async receive, or Sync mode (not in hunt mode)
                                _rxFifo.Enqueue(data);
                                _rxIntLatched = RxIntEnabled;

                                Log.Detail(Category.SIO, "Channel {0} Rx data: 0x{1:x2}, queue depth {2}",
                                                         _channelNum, data, _rxFifo.Count);
                            }

                            updateNeeded = true;
                        }

                        // Delay 'til the next one
                        _nextCharRx = now - skewNsec + _device.ReceiveRate;
                    }

                    if (delay == 0 || ((_nextCharRx > now) && (_nextCharRx - now < delay)))
                        delay = _nextCharRx - now;
                }

                // Check the transmitter
                if ((_device.TransmitRate > 0) && _registers.TxEnabled)
                {
                    // Time to send it?
                    if (_nextCharTx <= now)
                    {
                        // Can we send it?
                        if (_device.WriteReady &&
                           (!_registers.AutoEnables || (_registers.AutoEnables && _device.CTS)) &&
                           ((_txFifo.Count > 0) || _registers.SyncMode))
                        {
                            // What to send?
                            if (_txFifo.Count > 0)
                            {
                                data = _txFifo.Dequeue();
                            }
                            else
                            {
                                // In SyncMode (Speech) if there's no data, inject a sync byte
                                // TODO: Assert the proper Underrun error status bits...
                                data = _registers.TxSyncByte;
                            }

                            // Ship it
                            _device.Transmit(data);

                            Log.Detail(Category.SIO, "Channel {0} Tx data: 0x{1:x2}, queue depth {2}",
                                                     _channelNum, data, _txFifo.Count);

                            // If the buffer just became empty, raise the Tx interrupt (if enabled)
                            if (_txFifo.Count == 0)
                            {
                                _txIntLatched = _registers.TxIntEnabled;
                                if (!_registers.SyncMode) _device.RTS = false;
                            }

                            updateNeeded = true;
                        }

                        // Pace yourself
                        _nextCharTx = now - skewNsec + _device.TransmitRate;
                    }

                    if (delay == 0 || ((_nextCharTx > now) && (_nextCharTx - now < delay)))
                        delay = _nextCharTx - now;
                }

                // Update the registers if anything changed
                if (updateNeeded) UpdateFlags();

                // Reschedule if a positive poll rate
                if (delay > 0)
                {
                    Log.Verbose(Category.SIO, "Channel {0} rescheduled for {1}", _channelNum, delay);
                    _pollEvent = _scheduler.Schedule(delay, PollDevice);
                    return;
                }

                Log.Debug(Category.SIO, "Channel {0} polling stopped (nothing active!)", _channelNum);
                _pollEvent = null;
            }

            // Debugging
            public void DumpRegs()
            {
                if (_device == null)
                {
                    Console.WriteLine($"  No device attached on channel {_channelNum}");
                    return;
                }

                Console.WriteLine("  Channel {0} polling={1} rate={2} next={3}",
                                  _channelNum, (_pollEvent != null), _device.PollRate, _nextPoll);
                Console.WriteLine("    Receive: enabled={0} count={1} rate={2} next={3}",
                                  _registers.RxEnabled, _rxFifo.Count, _device.ReceiveRate, _nextCharRx);
                Console.WriteLine("    Transmit: enabled={0} count={1} rate={2} next={3}",
                                  _registers.TxEnabled, _txFifo.Count, _device.TransmitRate, _nextCharTx);

                _registers.DumpStatus();
            }


            int _channelNum;

            bool _rxIntOnNextChar;
            bool _rxIntLatched;
            bool _txIntLatched;
            bool _extIntLatched;

            int _interruptOffset;

            bool _breakSent;
            bool _breakDetected;

            ulong _nextPoll;
            ulong _nextCharRx;
            ulong _nextCharTx;

            Queue<byte> _rxFifo;
            Queue<byte> _txFifo;

            Registers _registers;

            Scheduler _scheduler;
            SchedulerEvent _pollEvent;

            SerialDevice _device;
        }
    }
}
