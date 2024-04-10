//
// NullEthernet.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
using System.Net.NetworkInformation;

namespace PERQemu.IO.Network
{
    /// <summary>
    /// A temporary fake Ethernet controller.  Implement only enough to let
    /// Accent properly start up its Net/Msg servers.  For now only responds
    /// to the OIO ports.  To be replaced by a proper Ethernet someday!
    /// </summary>
    public class NullEthernet : INetworkController
    {
        public NullEthernet(PERQSystem sys)
        {
            _system = sys;
            _timer = null;
            _response = null;

            // Physical address is configurable, but fixed;
            _physAddr = new MachineAddress(_system.Config);
            _physAddr.Low = _system.Config.EtherAddress;

            // Receive address can be programmed; set to HW initially
            _recvAddr = new MachineAddress(_system.Config);
            _recvAddr.Low = _physAddr.Low;

            _mcastGroups = new byte[6];

            Log.Debug(Category.Ethernet, "Interface created {0}", _physAddr);
        }

        public PhysicalAddress MACAddress => _physAddr.PA;

        // The Multicast Command Byte
        public byte MCB => _mcastGroups[0];

        public bool CanReceive => true;

        public void Reset()
        {
            if (_timer != null)
            {
                _system.Scheduler.Cancel(_timer);
            }
            _timer = null;

            if (_response != null)
            {
                _system.Scheduler.Cancel(_response);
            }
            _response = null;

            if (_clockInterrupt || _netInterrupt)
            {
                _system.CPU.ClearInterrupt(Processor.InterruptSource.X);
            }
            _clockInterrupt = false;
            _netInterrupt = false;

            _bitCount = 0;
            _usecClock = 0;

            _state = State.Idle;
            _status = Status.None;
            _control = Control.None;

            for (var i = 0; i < _mcastGroups.Length; i++)
            {
                _mcastGroups[i] = 0;
            }

            Log.Debug(Category.Ethernet, "Controller reset");
        }

        public void Shutdown()
        {
        }

        public void LoadRegister(byte address, int value)
        {
            var offset = 0;

            switch (address)
            {
                //
                // Microsecond clock setup - used for "exponential backoff" when
                // a collision occurs, can also be programmed as a general purpose
                // timer; fires an interrupt up to 65535 microseconds from enable
                //
                case 0x88:  // OIO Microsecond clock control
                case 0xdc:  // EIO
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to usec clock (control)", value);
                    break;

                case 0x89:  // OIO uSec clock timer high byte
                case 0xdd:  // EIO
                    _usecClock = (ushort)((value << 8) | (_usecClock & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to usec clock (high)", value);
                    break;

                case 0x8a:  // OIO uSec clock timer low byte
                case 0xde:  // EIO
                    _usecClock = (ushort)((_usecClock & 0xff00) | (value & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to usec clock (low)", value);
                    break;

                //
                // Bit counter setup
                //
                case 0x8c:  // OIO Bit counter control
                case 0xd8:  // EIO
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to bit counter (control)", value);
                    break;

                case 0x8d:  // OIO Bit counter high byte
                case 0xd9:  // EIO
                    _bitCount = (ushort)((value << 8) | (_bitCount & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to bit counter (high)", value);
                    break;

                case 0x8e:  // OIO Bit counter low byte
                case 0xda:  // EIO
                    _bitCount = (ushort)((_bitCount & 0xff00) | (value & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to bit counter (low)", value);
                    break;

                case 0x90:  // OIO Low word of MAC address
                    _recvAddr.Low = (ushort)value;
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x4} to low address register 0x{1:x2}", value, address);
                    break;

                case 0xc9:  // EIO byte 5 of MAC address
                    _recvAddr.Low = (ushort)((value << 8) | (_recvAddr.Low & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to MAC address byte 5", value);
                    break;

                case 0xc8:  // EIO byte 6 of MAC address
                    _recvAddr.Low = (ushort)((_recvAddr.Low & 0xff00) | (value & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to MAC address byte 6", value);
                    break;

                //
                // Multicast group bytes setup - On EIO, each byte is written
                // individually; on OIO, three 16-bit values are written and
                // distributed to the MCB and group bytes (command + 5 groups)
                //
                case 0x91:  // Multicast Grp1|Cmd
                case 0x92:  // Multicast Grp3|Grp2
                case 0x93:  // Multicast Grp5|Grp4
                    offset = address - 0x91;
                    _mcastGroups[offset] = (byte)(value & 0xff);
                    _mcastGroups[offset + 1] = (byte)(value >> 8);
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x4} to multicast register 0x{1:x2}", value, address);
                    break;

                case 0xca:  // EIO Multicast Cmd byte
                case 0xcb:  // EIO Multicast Grp1 .. Grp 5 bytes
                case 0xcc:
                case 0xcd:
                case 0xce:
                case 0xcf:
                    offset = address - 0xca;
                    _mcastGroups[offset] = (byte)(value & 0xff);
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to multicast register {1} (0x{2:x2})", value, offset, address);
                    break;

                case 0xc3:
                    Log.Info(Category.Ethernet, "Wrote 0x{0:x2} to net interrupt enable reg", value);
                    break;

                default:
                    throw new InvalidOperationException($"Unhandled write to port 0x{address:x}");
            }
        }

        /// <summary>
        /// Write to the command register to control the action.
        /// </summary>
        public void LoadCommand(int value)
        {
            // Todo: For now, assume OIO (port 0x99) although I think the EIO
            // (port 0xc2) programming model at this level is identical?

            _control = (Control)value;
            Log.Debug(Category.Ethernet, "Wrote 0x{0:x2} to control register ({1})", value, _control);

            // If the NotReset signal is not asserted, then we reset :-)
            if (!_control.HasFlag(Control.NotReset))
            {
                Reset();
                return;
            }

            // If we're busy and the Go bit dropped, abandon what we're doing
            // and return to Idle!?  Todo: find out if this stops the timer or
            // just the network (Miasma source)
            if (_state != State.Idle && !_control.HasFlag(Control.Go))
            {
                Reset();
                return;
            }

            // See if the Go flag is on and start an action.  Note that while we
            // can "see" the StartFlag, the hardware can't...
            if (_control.HasFlag(Control.Go))
            {
                // Timer: enabled, not already running, count set?
                if (_control.HasFlag(Control.ClockEnable) && _usecClock > 0 && _timer == null)
                {
                    // Start it up
                    Log.Debug(Category.Ethernet, "Timer enabled: will fire in {0}usec", _usecClock);
                    _timer = _system.Scheduler.Schedule(_usecClock * Conversion.UsecToNsec, ClockOverflow);
                }

                // Transmit flag?
                if (_control.HasFlag(Control.Transmit))
                {
                    // The bit count is written as a negative value and counts up;
                    // the later hardware automatically stops when it crosses zero?
                    // POS takes the two's complement in the microcode while Accent
                    // does it in the Pascal code that sets up the DCB.
                    _bitCount = (ushort)(0 - _bitCount);

                    // The microcode isn't supposed to start a new transmit if the
                    // receiver is active; let's do some sanity checks anyway...
                    if (_bitCount < 480 || _bitCount > 12144 || _state != State.Idle)
                    {
                        Log.Debug(Category.Ethernet, "Transmit requested while {0} or bad bit count: {1}", _state, _bitCount);
                        // Uh, what to do?  There's no error provision in the spec
                        // For now, assume whatever is running should finish then
                        // let the microcode reset us?
                        //return;
                    }

                    // Todo: Is the PIP bit defined/used on OIO, EIO or both?
                    _state = State.Transmitting;
                    _status |= (Status.CarrierSense | Status.Busy);

                    var delay = (ulong)((_bitCount * .1) + 9.6) * Conversion.UsecToNsec;
                    _response = _system.Scheduler.Schedule(delay, TransmitComplete);

                    Log.Debug(Category.Ethernet, "Transmitting {0} byte packet, callback in {1}usec",
                                                _bitCount / 8, delay / 1000);
                }
                else
                {
                    // Waiting for Godot... the only receive we'll actually handle
                    // is the special one required to get our Ethernet address back
                    // from the hardware

                    _state = State.Receiving;
                    _status |= Status.Busy;

                    if (MCB == 0xfe)
                    {
                        Log.Debug(Category.Ethernet, "Special receive to fetch address!");

                        // The minimum delay is as long as it takes to DMA one
                        // quad word, but the microcode seems to bank on the fact
                        // that there's at least enough extra delay to hold off
                        // programming the DMA registers.  "The amount of time it
                        // takes the hardware to read a preamble" is 96 bit times,
                        // so let's round up to 10usec?  Oy vey.
                        _response = _system.Scheduler.Schedule(10 * Conversion.UsecToNsec, GetAddress);
                    }

                    // Otherwise we just pretend
                }
            }
        }

        public int ReadRegister(byte address)
        {
            var retVal = 0;

            switch (address)
            {
                case 0x06:
                case 0x5a:
                    retVal = (_bitCount & 0xff);
                    Log.Detail(Category.Ethernet, "Read 0x{0:x2} from bit counter (low)", retVal);
                    return retVal;

                case 0x07:
                case 0x5b:
                    retVal = (_bitCount >> 8);
                    Log.Detail(Category.Ethernet, "Read 0x{0:x2} from bit counter (high)", retVal);
                    return retVal;

                default:
                    throw new InvalidOperationException($"Unhandled write to port 0x{address:x}");
            }
        }

        public int ReadStatus()
        {
            // Save the status we'll actually return to the caller
            var retVal = (int)_status;

            // If the clock overflowed, but the net is still busy, ONLY change
            // the overflow flag?
            if (_clockInterrupt)
            {
                _clockInterrupt = false;
                _status &= ~Status.Overflow;    // Turn off the flag for next time
            }

            // If we completed a packet xmit/recv, reset to Idle and clear the
            // "successful transmission" flag.
            if (_netInterrupt)
            {
                _netInterrupt = false;
                _state = State.Idle;

                // Sigh.  RecvComplete should be a separate bit, positively asserted
                // to clearly distinguish it from XmitComplete.  There are free bits!
                _status &= ~Status.Complete;
            }

            // Assume that reading the status register clears the interrupt
            // regardless of whether the net or timer raised it -- or both!?
            _system.CPU.ClearInterrupt(Processor.InterruptSource.X);
            Log.Debug(Category.Ethernet, "Read status: X interrupt cleared, returning {0}", retVal);
            return retVal;
        }

        //
        // Callbacks for timed events
        //

        void ClockOverflow(ulong nSkew, object context)
        {
            _clockInterrupt = true;

            if (_control.HasFlag(Control.ClockIntrEnable))
            {
                // Update our status and raise the interrupt
                _status |= Status.Overflow;
                _system.CPU.RaiseInterrupt(Processor.InterruptSource.X);
                _timer = null;
            }
        }

        void TransmitComplete(ulong nSkew, object context)
        {
            // Complete our "successful" transmission
            _status &= ~(Status.CarrierSense) | Status.Complete;

            FinishCommand();
        }

        void GetAddress(ulong nSkew, object context)
        {
            var addr = _system.IOB.DMARegisters.GetHeaderAddress(ChannelName.ExtA);

            Log.Debug(Category.Ethernet, "Writing machine address to 0x{0:x6}", addr);

            // DMA the address bytes into the header buffer
            _system.Memory.StoreWord(addr++, _physAddr.High);
            _system.Memory.StoreWord(addr++, _physAddr.Mid);

            // The low word's four nibbles are spread out like this:
            _system.Memory.StoreWord(addr++, (ushort)((_physAddr.Hn << 12) | (_physAddr.MHn << 4)));
            _system.Memory.StoreWord(addr, (ushort)((_physAddr.MLn << 12) | (_physAddr.Ln << 4)));

            FinishCommand();
        }

        void FinishCommand()
        {
            _response = null;
            _state = State.Complete;
            _status &= ~Status.Busy;
            _netInterrupt = true;
            _system.CPU.RaiseInterrupt(Processor.InterruptSource.X);
        }

        public bool WantReceive(PhysicalAddress dest)
        {
            return false;
        }

        public void DoReceive(byte[] packet)
        {
        }

        // Debugging
        public void DumpEther()
        {
            var header = _system.IOB.DMARegisters.GetHeaderAddress(ChannelName.ExtA);
            var buffer = _system.IOB.DMARegisters.GetDataAddress(ChannelName.ExtA);

            Console.WriteLine("Fake Ethernet status:");
            Console.WriteLine($"  My MAC address:    {_physAddr} ({_physAddr.High},{_physAddr.Mid},{_physAddr.Low})");
            Console.WriteLine($"  Receive address:   {_recvAddr} ({_recvAddr.High},{_recvAddr.Mid},{_recvAddr.Low})");
            Console.WriteLine($"  Control register:  {(int)_control:x} ({_control})");
            Console.WriteLine($"  Status register:   {(int)_status:x} ({_status})");
            Console.WriteLine("  Controller state:  {0}, scheduler callback {1} pending", _state,
                              (_response != null ? "IS" : "is NOT"));
            Console.WriteLine($"  DMA addresses:     Header: 0x{header:x6}  Buffer: 0x{buffer:x6}");

            Console.WriteLine("\n  Microsecond clock: {0} enabled, interrupt {1} enabled, {2} ticks",
                              (_control.HasFlag(Control.ClockEnable) ? "IS" : "Is NOT"),
                              (_control.HasFlag(Control.ClockIntrEnable) ? "IS" : "is NOT"),
                              _usecClock);
            if (_timer != null) Console.WriteLine("  Timer is running!");

            Console.WriteLine("\n  Bit counter:       {0} enabled, {1} count",
                              (_control.HasFlag(Control.CounterEnable) ? "IS" : "Is NOT"),
                              _bitCount);

            Console.WriteLine("\n  Multicast bytes:   {0}", string.Join(", ", _mcastGroups));
        }

        enum State
        {
            Idle = 0,
            Reset,
            Receiving,
            Transmitting,
            Complete
        }

        [Flags]
        /// <summary>
        /// OIO Ethernet control register bits.  NB: Reset is assert LOW.  Bits 7 and
        /// 9..15 are undefined in the hardware but may be used by the microcode.
        /// </summary>
        enum Control
        {
            None = 0x0,
            NetIntrEnable = 0x1,
            ClockIntrEnable = 0x2,
            ClockEnable = 0x4,
            CounterEnable = 0x8,
            Transmit = 0x10,
            NotReset = 0x20,
            Promiscuous = 0x40,
            SleepFlag = 0x80,
            Go = 0x100,
            StartFlag = 0x200
        }

        [Flags]
        enum Status
        {
            None = 0x0,
            CRCError = 0x1,
            Collision = 0x2,
            Complete = 0x4,
            Busy = 0x8,
            Unused = 0x10,
            Overflow = 0x20,
            PacketInProgress = 0x40,
            CarrierSense = 0x80
        }

        State _state;
        Control _control;
        Status _status;

        MachineAddress _physAddr;
        MachineAddress _recvAddr;

        byte[] _mcastGroups;

        bool _netInterrupt;
        bool _clockInterrupt;

        ushort _bitCount;
        ushort _usecClock;

        SchedulerEvent _response;
        SchedulerEvent _timer;
        PERQSystem _system;
    }
}
