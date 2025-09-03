//
// NullEthernet.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.Processor;

namespace PERQemu.IO.Network
{
    /// <summary>
    /// A fake Ethernet controller that does not connect to a host adapter.
    /// Implements enough to let Accent properly start up its Net/Msg servers
    /// but acts as if the machine isn't plugged into the network.
    /// </summary>
    /// <remarks>
    /// This baseline implementation can pretend to be an OIO or EIO interface
    /// by handling all of the IOB registers.  The differences are so slight I
    /// just don't feel like breaking this down into OIO / EIO variants. :-P
    /// </remarks>
    public class NullEthernet : INetworkController
    {
        public NullEthernet(PERQSystem sys)
        {
            _system = sys;
            _timer = null;
            _response = null;

            // Physical address is configurable, but fixed.  If not set, generate
            // a random one (to avoid conflicts by having all the PERQs on your
            // local net come up with the same default! :-)
            _physAddr = new MachineAddress(_system.Config);
            _physAddr.Low = _system.Config.EtherAddress;

            // Set a random one if not set.  See Docs\Hardware References\serial.memo
            // for info about the range of PERQ serial numbers (and MAC addresses)
            if (_physAddr.Low == 0)
            {
                _physAddr.Low = (ushort)(new Random().Next(5800, 32766));
            }

            // Receive address can be programmed; set to HW initially
            _recvAddr = new MachineAddress(_system.Config);
            _recvAddr.Low = _physAddr.Low;

            _mcastGroups = new byte[6];

            // Set interrupt vector, DMA channel based on board type
            if (_system.Config.IOBoard == Config.IOBoardType.EIO)
            {
                _irq = InterruptSource.Network;
                _dmaTx = ChannelName.NetXmit;
                _dmaRx = ChannelName.NetRecv;

                // Due to DMA implementation differences, on EIO Accent sets the
                // max packet size to 1524 bytes...
                _maxBits = 1524 * 8;
            }
            else
            {
                // OIO interface
                _irq = InterruptSource.X;
                _dmaTx = ChannelName.ExtA;
                _dmaRx = ChannelName.ExtA;

                // ...but on OIO, the usual non-VLAN-aware old Ethernet 1518 is used
                _maxBits = 1518 * 8;
            }

            Log.Debug(Category.Ethernet, "Interface created {0}", _physAddr);
        }

        // Give back the hardware MAC address
        public PhysicalAddress MACAddress => _physAddr.PA;

        // The Multicast Command Byte
        public byte MCB => _mcastGroups[0];

        /// <summary>
        /// Reset this instance.
        /// </summary>
        public virtual void Reset()
        {
            _system.Scheduler.Cancel(_timer);
            _timer = null;

            _system.Scheduler.Cancel(_response);
            _response = null;

            _clockInterrupt = false;
            _netInterrupt = false;
            _netIntEnable = false;
            SetInterrupt();

            _bitCount = 0;
            _usecClock = 0;

            _state = State.Idle;
            _status = Status.None;
            _control = Control.None;

            Log.Debug(Category.Ethernet, "Controller reset");
        }

        /// <summary>
        /// Shutdown this instance.
        /// </summary>
        public virtual void Shutdown()
        {
            // Nothing to do for the fake interface
        }

        #region IO Registers

        /// <summary>
        /// Ethernet register loads.  Handles both OIO and EIO variants.
        /// </summary>
        /// <remarks>
        /// The microsecond clock is used for "exponential backoff" when a collision
        /// occurs, but Pcap insulates us from that.  It can also be programmed as a
        /// general purpose timer; fires an interrupt up to 65535usec from enable.
        /// 
        /// The bit counter is used by the hardware to know how many bytes to send,
        /// and by the receiver to count incoming bits (which must end as a multiple
        /// of 8 to know if the final byte count is valid).  Here we basically ignore
        /// the counter control register that the microcode uses to manage the counter
        /// and just assume it's active when needed.
        /// </remarks>
        public void LoadRegister(byte address, int value)
        {
            var offset = 0;

            switch (address)
            {
                //
                // Microsecond clock
                //

                // uSec clock control
                case 0x88:      // OIO
                case 0xdc:      // EIO
                    // This register is set to '3' since it's used to drive a PAL that
                    // writes directly to an Am2942 counter chip to set up the uSec Clock.
                    // We just log it and assume that the microcode does the right thing :-)
                    Log.Debug(Category.Ethernet, "Wrote 0x{0:x2} to usec clock (control)", value);
                    break;

                // uSec clock counter high byte
                case 0x89:      // OIO
                case 0xdd:      // EIO
                    _usecClock = (ushort)((value << 8) | (_usecClock & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to usec clock (high)", value);
                    break;

                // uSec clock counter low byte
                case 0x8a:      // OIO
                case 0xde:      // EIO
                    _usecClock = (ushort)((_usecClock & 0xff00) | (value & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to usec clock (low)", value);
                    break;

                //
                // Bit counter
                //

                // Bit counter control
                case 0x8c:      // OIO
                case 0xd8:      // EIO
                    // The bit counter is another Am2942; see above.
                    Log.Debug(Category.Ethernet, "Wrote 0x{0:x2} to bit counter (control)", value);
                    break;

                // Bit counter high byte
                case 0x8d:      // OIO
                case 0xd9:      // EIO
                    _bitCount = (ushort)((value << 8) | (_bitCount & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to bit counter (high)", value);
                    break;

                // Bit counter low byte
                case 0x8e:      // OIO 
                case 0xda:      // EIO
                    _bitCount = (ushort)((_bitCount & 0xff00) | (value & 0xff));
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to bit counter (low)", value);
                    break;

                //
                // Receive Address setup
                //
                // Note: the microcode writes these byte swapped but expects them
                // back in the correct order!  The OIO provides a swapped word,
                // while the EIO programs each byte individually (inverted).  Sigh.
                //

                case 0x90:      // OIO Low word of MAC address - swap the bytes
                    _recvAddr.LowFifth = (byte)(value & 0xff);
                    _recvAddr.LowSixth = (byte)(value >> 8);
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x4} to low address register 0x{1:x2}", value, address);
                    break;

                case 0xc9:      // EIO Low word (byte 5) of MAC address - swap with 6th
                    _recvAddr.LowSixth = (byte)(~value & 0xff);
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to MAC address byte 5", value);
                    break;

                case 0xc8:      // EIO Low word (byte 6) of MAC address - swap with 5th
                    _recvAddr.LowFifth = (byte)(~value & 0xff);
                    Log.Detail(Category.Ethernet, "Wrote 0x{0:x2} to MAC address byte 6", value);
                    break;

                //
                // Multicast setup
                //
                // Minor hardware difference: On OIO, the microcode writes three
                // 16-bit words and the hardware provides byte access; on EIO, each
                // byte is written individually.  The Null device as a stand-in
                // handles either approach.
                //

                // OIO
                case 0x91:      // Grp1|Cmd
                case 0x92:      // Grp3|Grp2
                case 0x93:      // Grp5|Grp4
                    offset = address - 0x91;
                    _mcastGroups[offset] = (byte)(value & 0xff);
                    _mcastGroups[offset + 1] = (byte)(value >> 8);
                    Log.Debug(Category.Ethernet, "Wrote 0x{0:x4} to multicast register 0x{1:x2}", value, address);
                    break;

                // EIO
                case 0xca:      // Cmd byte
                case 0xcb:      // Grp1
                case 0xcc:      // Grp2
                case 0xcd:      // Grp3
                case 0xce:      // Grp4
                case 0xcf:      // Grp5
                    offset = address - 0xca;
                    _mcastGroups[offset] = (byte)(~value & 0xff);
                    Log.Debug(Category.Ethernet, "Wrote 0x{0:x2} to multicast register {1} (0x{2:x2})", value, offset, address);
                    break;

                //
                // Interrupt enable register
                //
                case 0xc3:      // EIO only
                    _netIntEnable = (value & 0x1) != 0;
                    Log.Info(Category.Ethernet, "Wrote 0x{0:x2} to net interrupt enable reg", value);
                    SetInterrupt();
                    break;

                default:
                    throw new UnhandledIORequestException(address, value);
            }
        }

        /// <summary>
        /// Write to the command register to control the action.  It appears that
        /// the OIO (port 0x99) and EIO (port 0xc2) are programmed in almost
        /// exactly the same way, so we handle both here.
        /// </summary>
        public void LoadCommand(int value)
        {
            _control = (Control)value;
            Log.Info(Category.Ethernet, "Wrote 0x{0:x2} to control register ({1})", value, _control);

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

            // If OIO, the interrupt enable bit is significant; EIO uses a separate
            // reg but SOME OSes set the bit on EIO anyway!  Track either approach
            // using a local flag
            if (!_system.IOB.IsEIO)
            {
                _netIntEnable = _control.HasFlag(Control.NetIntrEnable);
            }

            // See if the Go flag is on and start an action
            if (_control.HasFlag(Control.Go))
            {
                // Timer: enabled, not already running, count set?
                if (_control.HasFlag(Control.ClockEnable))
                {
                    // Not already running?
                    if (_usecClock != 0 && _timer == null)
                    {
                        // Start it up
                        Log.Debug(Category.Ethernet, "Timer enabled: will fire in {0}usec", _usecClock);
                        _timer = _system.Scheduler.Schedule(_usecClock * Conversion.UsecToNsec, ClockOverflow);
                    }
                    // Otherwise writes are ignored, per eio.doc
                }
                else
                {
                    // Running?
                    if (_timer != null)
                    {
                        Log.Debug(Category.Ethernet, "Timer disabled: was to fire in {0}usec",
                                 (_system.Scheduler.CurrentTimeNsec - _timer.TimestampNsec) * 0.001);

                        _system.Scheduler.Cancel(_timer);
                        _timer = null;
                    }
                }

                // Transmit flag?
                if (_control.HasFlag(Control.Transmit))
                {
                    StartTransmit();
                }
                else
                {
                    StartReceive();
                }
            }
        }

        /// <summary>
        /// Reads the bit counter registers.
        /// </summary>
        public int ReadRegister(byte address)
        {
            var retVal = 0;

            switch (address)
            {
                case 0x06:      // OIO
                case 0x5a:      // EIO
                    retVal = (_bitCount & 0xff);
                    Log.Detail(Category.Ethernet, "Read 0x{0:x2} from bit counter (low)", retVal);
                    return retVal;

                case 0x07:      // OIO
                case 0x5b:      // EIO
                    retVal = (_bitCount >> 8);
                    Log.Detail(Category.Ethernet, "Read 0x{0:x2} from bit counter (high)", retVal);
                    return retVal;

                default:
                    throw new UnhandledIORequestException(address);
            }
        }

        /// <summary>
        /// Reads the status register.  OIO port 0017 (0x0f); EIO port 0122 (0x52)
        /// </summary>
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

            // Reading the status register clears the interrupt regardless of
            // whether the net or timer raised it
            SetInterrupt();

            Log.Debug(Category.Ethernet, "Read status: 0x{0:x} ({1})", (int)retVal, retVal);
            return retVal;
        }

        #endregion IO Registers

        #region Transmit

        /// <summary>
        /// Set up to transmit a packet.  Does sanity checks and updates state,
        /// but calls the DoTransmit() to start the send.
        /// </summary>
        void StartTransmit()
        {
            // The bit count is written as a negative value and counts up;  the
            // hardware automatically stops when it crosses zero.  POS takes the
            // two's complement in the microcode while Accent does it in Pascal
            // code that sets up the DCB.  To compute transmission delay, take
            // the absolute value...
            if ((short)_bitCount < 0) _bitCount = (ushort)(0 - _bitCount);

            // Sanity checks:  the microcode isn't supposed to start a new send
            // if the receiver is active, so a Reset should have been done first
            // to cancel the receive.  And we check the bit count to make sure
            // the value represents a legal packet length.  (The PERQ should do
            // these itself, so this might be removed after more testing.)
            if (_bitCount < 480 || _bitCount > _maxBits || _state != State.Idle)
            {
                Log.Write(Category.Ethernet, "Transmit requested while {0} or bad bit count: {1}",
                                             _state, (short)_bitCount);

                // Uh, what to do?  There's no error provision in the spec as it
                // must just be assumed that the state machine and Pascal/ucode
                // is setting up a valid packet.  For now, set _bitCount to zero
                // so that TransmitComplete() won't attempt to send a bad packet,
                // finish processing the send normally, and let the ucode reset
                // the interface as usual
                _bitCount = 0;
            }

            // Set up for sending!
            _state = State.Transmitting;
            _status |= (Status.CarrierSense | Status.Busy);

            // Let 'er go
            DoTransmit();
        }

        /// <summary>
        /// Transmit a packet.  For the null interface, just compute how long it
        /// would take based on the bit count, but don't actually send anything.
        /// </summary>
        protected virtual void DoTransmit()
        {
            // Delay includes IPG and 32 bits of FCS
            var delay = (ulong)((_bitCount + 32) * .1 + 9.6) * Conversion.UsecToNsec;
            _response = _system.Scheduler.Schedule(delay, TransmitComplete);

            Log.Info(Category.Ethernet, "Transmitted {0} bytes ({1} bits), callback in {2}usec",
                                         _bitCount / 8, (short)_bitCount, delay / 1000);
        }

        #endregion Transmit

        #region Receive

        /// <summary>
        /// Set up for a receive.  Sets state and status, and handles the special
        /// receive used to get our MAC address from the hardware.  For the null
        /// interface, no packets ever arrive.
        /// </summary>
        void StartReceive()
        {
            _state = State.ReceiveWait;
            _status |= Status.Busy;

            if (MCB == 0xfe)
            {
                Log.Info(Category.Ethernet, "Special receive to fetch address!");
                _state = State.Receiving;

                // The minimum delay is as long as it takes to DMA one
                // quad word, but the microcode seems to bank on the fact
                // that there's at least enough extra delay to hold off
                // programming the DMA registers.  "The amount of time it
                // takes the hardware to read a preamble" is 96 bit times,
                // so let's round up to 10usec?  Oy vey.
                _response = _system.Scheduler.Schedule(10 * Conversion.UsecToNsec, GetAddress);
                return;
            }

            // Go see if any actual packets have arrived
            DoReceive();
        }

        /// <summary>
        /// Initiate or enable packet reception on the interface, if present.
        /// </summary>
        protected virtual void DoReceive()
        {
            // Nothing to do for the null interface
        }

        /// <summary>
        /// Return true if the interface is in a state to receive suitors.
        /// </summary>
        public virtual bool CanReceive => true;

        /// <summary>
        /// Check if an incoming packet is of interest to us.
        /// </summary>
        public virtual bool WantReceive(PhysicalAddress dest)
        {
            return true;
        }

        /// <summary>
        /// Receive the specified packet from the NIC.
        /// </summary>
        public virtual void Receive(byte[] packet)
        {
            // Should never be called on the null interface
        }

        #endregion Receive

        #region Callbacks

        /// <summary>
        /// Fetch the hardware's MAC address from the DMA header in response to
        /// the "special receive".  Store it in memory where the microcode will
        /// transform it into the canonical 48-bit format we know and love.
        /// </summary>
        void GetAddress(ulong nSkew, object context)
        {
            var addr = _system.IOB.DMARegisters.GetHeaderAddress(_dmaRx);
            var words = _physAddr.Unscrambled(_system.IOB.IsEIO);

            Log.Debug(Category.Ethernet, "Writing machine address to 0x{0:x6}", addr);

            // DMA the unscrambled address bytes into the header buffer
            for (var i = 0; i < words.Length; i++)
            {
                _system.Memory.StoreWord(addr++, words[i]);
            }

            FinishCommand();
        }

        /// <summary>
        /// Called when the microsecond clock overflows, which it never will.
        /// </summary>
        /// <remarks>
        /// Unfortunately, I don't think anything but the Ethernet driver ever
        /// used this, since the emulator can't see or pass through collisions
        /// to the microcode.  Would be neat to have a generic event timer with
        /// microsecond accuracy for doing stuff like animations; POS might be
        /// able to use it, but Accent and PNX probably take over the hardware
        /// exclusively and there's no high-level API to access the Am2942.
        /// </remarks>
        protected virtual void ClockOverflow(ulong nSkew, object context)
        {
            // Update our status and raise the interrupt
            _timer = null;
            _status |= Status.Overflow;
            _clockInterrupt = true;
            SetInterrupt();
        }

        /// <summary>
        /// Complete a packet transmissions.
        /// </summary>
        protected virtual void TransmitComplete(ulong nSkew, object context)
        {
            // Assume a successful transmission, since we don't actually do
            // our own collision detect + backoff + retry processing :-)
            _status &= ~(Status.CarrierSense);
            _status |= Status.Complete;

            // Set the bit counter to zero to indicate success
            _bitCount = 0;

            FinishCommand();
        }

        /// <summary>
        /// Finish receive processing.
        /// </summary>
        protected virtual void ReceiveComplete(ulong nSkew, object context)
        {
            // Reception complete!  Turn OFF these bits:
            _status &= ~(Status.CarrierSense | Status.PacketInProgress | Status.Complete);

            FinishCommand();
        }

        #endregion Callbacks

        /// <summary>
        /// Finish a command by resetting state & status as appropriate and
        /// updating the interrupt line.
        /// </summary>
        void FinishCommand()
        {
            _response = null;
            _state = State.Complete;
            _status &= ~Status.Busy;
            _netInterrupt = true;
            SetInterrupt();
        }

        /// <summary>
        /// Manage the shared interrupt line (network and uSec clock).
        /// </summary>
        void SetInterrupt()
        {
            // EIO uses a 74S51 AND-OR-INVERT to directly set the IRQ line
            var raise = (_netInterrupt && _netIntEnable) ||
                        (_clockInterrupt && _control.HasFlag(Control.ClockIntrEnable));

            // Any change?
            if (raise == _irqActive) return;

            // Do it and save state
            if (raise && !_irqActive)
            {
                _system.CPU.RaiseInterrupt(_irq);
                _irqActive = true;
            }
            else if (!raise && _irqActive)
            {
                _system.CPU.ClearInterrupt(_irq);
                _irqActive = false;
            }
        }

        // Debugging
        public virtual void DumpEther()
        {
            var header = _system.IOB.DMARegisters.GetHeaderAddress(_dmaRx);
            var buffer = _system.IOB.DMARegisters.GetDataAddress(_dmaRx);

            Console.WriteLine("Null Ethernet status:");
            Console.WriteLine($"  My MAC address:    {_physAddr} ({_physAddr.High},{_physAddr.Mid},{_physAddr.Low})");
            Console.WriteLine($"  Receive address:   {_recvAddr} ({_recvAddr.High},{_recvAddr.Mid},{_recvAddr.Low})");
            Console.WriteLine($"  DMA addresses:     Header: 0x{header:x6}  Buffer: 0x{buffer:x6} ({_dmaRx})");
            if (_dmaRx != _dmaTx)
            {
                header = _system.IOB.DMARegisters.GetHeaderAddress(_dmaTx);
                buffer = _system.IOB.DMARegisters.GetDataAddress(_dmaTx);

                Console.WriteLine($"  DMA addresses:     Header: 0x{header:x6}  Buffer: 0x{buffer:x6} ({_dmaTx})");
            }
            Console.WriteLine("  Multicast bytes:   {0}", string.Join(", ", _mcastGroups));

            Console.WriteLine($"  Control register:  {(int)_control:x} ({_control})");
            Console.WriteLine($"  Status register:   {(int)_status:x} ({_status})");
            Console.WriteLine("  Controller state:  {0}  Callback pending: {1}", _state, _response != null);
            Console.WriteLine("  Interrupt state:   {0}  Active: {1} | {2}  Enabled: {3} | {4}",
                              _irqActive ? $"{_irq} raised" : "None",
                              _netInterrupt ? "NET" : "net",
                              _clockInterrupt ? "CLK" : "clk",
                              _netIntEnable ? "NET" : "net",
                              _control.HasFlag(Control.ClockIntrEnable) ? "CLK" : "clk");

            Console.WriteLine("  Microsecond clock: Enabled: {0}  Running: {1} ({2} ticks)",
                              _control.HasFlag(Control.ClockEnable),
                              _timer != null,
                              _usecClock);

            Console.WriteLine("  Bit counter:       Enabled: {0}  Count: {1}",
                              _control.HasFlag(Control.CounterEnable),
                              _bitCount);
        }

        /// <summary>
        /// Controller states.
        /// </summary>
        protected enum State
        {
            Idle = 0,
            Reset,
            ReceiveWait,
            Receiving,
            Transmitting,
            Complete
        }

        /// <summary>
        /// Ethernet control register bits.  NB: Reset is assert LOW.  Bits 7 and
        /// 9..15 are undefined in the hardware but may be used by the microcode.
        /// EIO may not use NetIntrEnable (separate register) but OIO does, and it
        /// looks like Accent may set it anyway?
        /// </summary>
        [Flags]
        protected enum Control
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
        protected enum Status
        {
            None = 0x0,
            CRCError = 0x1,
            Collision = 0x2,
            Complete = 0x4,
            Busy = 0x8,
            Unused = 0x10,          // In hardware, the NET INT bit
            Overflow = 0x20,
            PacketInProgress = 0x40,
            CarrierSense = 0x80,
            RetryMask = 0xf00,      // NOT in the hardware; used by ucode
            LargePacket = 0x1000,   // POS (and to some extent Accent and
            Unused13 = 0x2000,      // PNX) also seem to use these extra
            SendError = 0x4000,     // bits in a similar fashion
            CmdInProgress = 0x8000
        }

        protected State _state;
        protected Control _control;
        protected Status _status;

        protected MachineAddress _physAddr;
        protected MachineAddress _recvAddr;

        protected byte[] _mcastGroups;

        protected bool _netInterrupt;
        protected bool _netIntEnable;
        protected bool _clockInterrupt;
        protected bool _irqActive;

        protected InterruptSource _irq;
        protected ChannelName _dmaTx;
        protected ChannelName _dmaRx;

        protected ushort _bitCount;
        protected ushort _maxBits;
        protected ushort _usecClock;

        protected SchedulerEvent _response;
        protected SchedulerEvent _timer;
        protected PERQSystem _system;
    }
}
