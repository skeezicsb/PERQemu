//
// i8237DMA.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// AMD Am9517 (aka Intel i8237) DMA chip.
    /// </summary>
    /// <remarks>
    /// This four-channel DMA controller is used on the EIO board.  This chip
    /// uses two blocks of addresses: control/status registers in one block,
    /// and four address/word count registers in a second.
    /// 
    /// DREQ channel assignments are fixed on the EIO:
    ///     Chn 0 - Floppy      Chn 2 - SIO
    ///     Chn 1 - GPIB        Chn 3 - PERQ
    /// </remarks>
    public class i8237DMA : IZ80Device
    {
        public i8237DMA(byte csrBase, byte chnBase, Z80MemoryBus memoryBus, Z80IOBus ioBus)
        {
            _memory = memoryBus;
            _iobus = ioBus;

            _controlBase = csrBase;
            _channelBase = chnBase;
            _ports = new byte[16];

            // Register both blocks with the Z80
            for (int i = 0; i < 8; i++)
            {
                _ports[i] = (byte)(_controlBase + i);
                _ports[i + 8] = (byte)(_channelBase + i);
            }

            // Allocate our channels
            _channels = new ChannelRegister[4];
        }

        public string Name => "i8237 DMA";
        public byte[] Ports => _ports;

        public bool IntLineIsActive => _interruptEnabled;
        public byte? ValueOnDataBus => null;        // Supplied by the Am9519

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        public void Reset()
        {
            // Hardware reset or Master Clear command
            for (var c = 0; c < _channels.Length; c++)
            {
                _channels[c].Terminated = false;
                _channels[c].Masked = true;
                _channels[c].LowByte = true;
            }

            _command = 0;
            _temporary = 0;

            _active = 0;
            _lastServed = 0;

            _state = DMAState.Idle;
            _interruptEnabled = false;

            Log.Info(Category.Z80DMA, "i8237 reset");
        }

        public void AttachChannelDevice(int chan, IDMADevice dev, byte port)
        {
#if DEBUG
            if (chan < 0 || chan > _channels.Length - 1)
                throw new ArgumentOutOfRangeException(nameof(chan));

            if (_channels[chan].Device != null)
                throw new InvalidOperationException($"Channel {chan} already assigned to {dev}");
#endif
            _channels[chan].Device = dev;
            _channels[chan].DataPort = port;
            Log.Info(Category.Z80DMA, "Channel {0} assigned to {1} (port 0x{2:x2})", chan, dev, port);
        }

        /// <summary>
        /// If a DMA operation is in progress, complete it; if idle, look for the
        /// next device that's requesting service and start it.  Return the cycle
        /// count if any work is done.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public int Clock()
        {
            DMAState nextState = _state;
            int cycles = 0;

            if (_state == DMAState.Idle)
            {
                var chan = NextRequester();
                if (chan < 0) return 0;         // Twiddle thumbs

                // Got one!
                _active = chan;
                _state = DMAState.SourceRead;
            }

            // Transfer mode = direction: Write TO mem, Read FROM mem
            // (Verify is not supported since PERQ doesn't use it?)
            var devToMem = (_channels[_active].Transfer == TransferMode.Write);

            switch (_state)
            {
                case DMAState.SourceRead:
                    //
                    // On reads we just stash the byte in the _temporary reg (so
                    // it can be read out if desired, though that's only for mem-
                    // to-mem xfers which PERQ doesn't use?).  Address and byte
                    // counts are always adjusted in the second half.
                    //
                    if (devToMem)
                    {
                        // Is the source ready? (It better be, since right now
                        // we don't have any timeout or reset if things get out
                        // of sync!
                        if (!_channels[_active].Device.ReadDataReady)
                        {
                            // Bug out, _state unchanged
                            Log.Info(Category.Z80DMA, "Device {0} not ready on read!", _active);
                            return 0;
                        }

                        // Grab the byte; logging done on bus read
                        _temporary = _iobus[_channels[_active].DataPort];
                    }
                    else
                    {
                        // Memory is always ready
                        _temporary = _memory[_channels[_active].CurrentAddress];
                    }

                    nextState = DMAState.DestWrite;
                    cycles = 4;
                    break;

                case DMAState.DestWrite:
                    //
                    // Writes handle the counters and do EOP/TC and autoinit if
                    // so programmed.  As with reads, memory is always ready but
                    // a destination device may not have buffer space or be in a
                    // state to receive the byte; in that case we just loop until
                    // it does.  We'll have to see if this is a problem!
                    // 
                    if (!devToMem)
                    {
                        if (!_channels[_active].Device.WriteDataReady)
                        {
                            Log.Info(Category.Z80DMA, "Device {0} not ready on write!", _active);
                            return 0;
                        }

                        // Write it!
                        _iobus[_channels[_active].DataPort] = _temporary;
                    }
                    else
                    {
                        // Save the data
                        _memory[_channels[_active].CurrentAddress] = _temporary;
                    }

                    // The usual EIO Z80 code doesn't use Block mode... but maybe
                    // PNX does?  Would love some source code to find out...
                    nextState = (_channels[_active].Mode == ChannelMode.Block) ? DMAState.SourceRead : DMAState.Idle;

                    // Bump the counters and test for TC
                    if (_channels[_active].CountComplete())
                    {
                        Log.Info(Category.Z80DMA, "Channel {0} transfer complete!", _active);

                        // Whack the device
                        _channels[_active].Device.DMATerminate();

                        // Reinit?  Will reset or mask as appropriate
                        _channels[_active].Reinit();

                        // Ping the Am9519
                        _interruptEnabled = true;

                        // Regardless of channel mode
                        nextState = DMAState.Idle;
                    }

                    _lastServed = _active;
                    cycles = 3;
                    break;
            }

            _state = nextState;
            return cycles;
        }

        /// <summary>
        /// Scan for the next device to service, in priority order.  Also does
        /// the AutoInitialization for ports that are at TC and are configured
        /// to re-init.  Returns -1 if nobody's active.
        /// </summary>
        int NextRequester()
        {
            // Anybody want service?  Update flags
            for (var c = 0; c < _channels.Length; c++)
            {
                // Update the Requested flag
                _channels[c].CheckReady();
            }

            // Scan by priority
            var start = _command.HasFlag(CommandBits.RotatingPriority) ? _lastServed : 3;
            var next = (start + 1) % 4;

            while (!_channels[next].Requested)
            {
                if (next == start) return -1;   // Nothin' to do
                next = (next + 1) % 4;          // Wrap that rascal
            }

            Log.Info(Category.Z80DMA, "Channel {0} wants service!", next);
            return next;
        }

        /// <summary>
        /// Recompute the status register with current request and terminated
        /// status of all four channels.
        /// </summary>
        void UpdateStatus()
        {
            var ostatus = _status;

            // The EOP (Terminated) bits are supposed to be "sticky" even after
            // an auto-reinit -- so these may not have to get updated here? They
            // are cleared only on a read of the status register.
            _status = (byte)(_channels[0].Terminated ? 0x01 : 0);
            _status |= (byte)(_channels[1].Terminated ? 0x02 : 0);
            _status |= (byte)(_channels[2].Terminated ? 0x04 : 0);
            _status |= (byte)(_channels[3].Terminated ? 0x08 : 0);
            _status |= (byte)(_channels[0].Requested ? 0x10 : 0);
            _status |= (byte)(_channels[1].Requested ? 0x20 : 0);
            _status |= (byte)(_channels[2].Requested ? 0x40 : 0);
            _status |= (byte)(_channels[3].Requested ? 0x80 : 0);

            if (ostatus != _status)
            {
                Log.Info(Category.Z80DMA, "Status change: 0x{0:x2}", _status);
            }
        }

        public byte Read(byte portAddress)
        {
            if (portAddress == _controlBase)
            {
                UpdateStatus();

                byte save = _status;
                Log.Info(Category.Z80DMA, "Read 0x{0:x2} from status register", save);

                // Reading status clears the EOP bits (S3..S0)!  AND the TC bits!?
                _status &= 0xf0;
                _channels[0].Terminated = false;
                _channels[1].Terminated = false;
                _channels[2].Terminated = false;
                _channels[3].Terminated = false;
                _interruptEnabled = false;

                return save;
            }

            if (portAddress == _controlBase + 5)
            {
                Log.Info(Category.Z80DMA, "Read 0x{0:x2} from temporary reg", _temporary);
                return _temporary;
            }

            if (portAddress >= _channelBase && portAddress <= _channelBase + 7)
            {
                // Compute channel #
                var chan = (portAddress - _channelBase) / 2;
                var addr = (portAddress & 0x1) == 0;
                ushort val = 0;
                byte half = 0;

                // Even port is current address, odd port is current word count
                if (addr)
                {
                    val = _channels[chan].CurrentAddress;
                    half = (_channels[chan].LowByte ? (byte)val : (byte)(val >> 8));
                }
                else
                {
                    val = _channels[chan].CurrentCount;
                    half = (_channels[chan].LowByte ? (byte)val : (byte)(val >> 8));
                }

                Log.Info(Category.Z80DMA, "Read 0x{0:x2} from channel {1} current {2} ({3} byte)",
                                            half, chan,
                                           (addr ? "address" : "word count"),
                                           (_channels[chan].LowByte ? "low" : "high"));

                // Toggle the flip flop for the next read
                _channels[chan].LowByte = !_channels[chan].LowByte;
                return half;
            }

            Log.Warn(Category.Z80DMA, "Bad read from port 0x{0:x2} (illegal address)", portAddress);
            return 0;
        }

        public void Write(byte portAddress, byte value)
        {
            var chan = 0;

            // Channel data?
            if (portAddress >= _channelBase && portAddress <= _channelBase + 7)
            {
                chan = (portAddress - _channelBase) / 2;
                var addr = (portAddress & 0x1) == 0;

                // Address or word count?
                if (addr)
                {
                    // Low byte assigns/clears high, high byte adds
                    if (_channels[chan].LowByte)
                        _channels[chan].BaseAddress = value;
                    else
                        _channels[chan].BaseAddress += (ushort)(value << 8);

                    // The Current counter is loaded in parallel!
                    _channels[chan].CurrentAddress = _channels[chan].BaseAddress;
                }
                else
                {
                    // As above, for the word counter
                    if (_channels[chan].LowByte)
                        _channels[chan].WordCount = value;
                    else
                        _channels[chan].WordCount += (ushort)(value << 8);

                    _channels[chan].CurrentCount = _channels[chan].WordCount;
                    _channels[chan].Terminated = false;
                }

                Log.Info(Category.Z80DMA, "Write 0x{0:x2} to channel {1} {2} ({3} byte)",
                                            value, chan,
                                           (addr ? "base address" : "word count"),
                                           (_channels[chan].LowByte ? "low" : "high"));

                // Toggle the flip flop for the next write
                _channels[chan].LowByte = !_channels[chan].LowByte;
                return;
            }

            // Command write
            var cmd = (portAddress - _controlBase);
            chan = (value & 0x03);      // channel select for most commands

            switch (cmd)
            {
                case 0x0:   // Write command register
                    _command = (CommandBits)value;
                    Log.Info(Category.Z80DMA, "Write 0x{0:x2} ({1}) to command register", value, _command);
                    break;

                case 0x1:   // Write request bit
                    if (_channels[chan].Mode != ChannelMode.Block)
                    {
                        Log.Info(Category.Z80DMA, "Channel {0} Request write ignored (not in Block mode)", chan);
                        break;
                    }
                    _channels[chan].Requested = (value & 0x04) > 0;
                    Log.Info(Category.Z80DMA, "Channel {0} Requested is {1}", chan, _channels[chan].Requested);
                    break;

                case 0x2:   // Write single mask bit
                    _channels[chan].Masked = (value & 0x04) > 0;
                    Log.Info(Category.Z80DMA, "Channel {0} Masked is {1}", chan, _channels[chan].Masked);
                    break;

                case 0x3:   // Write mode register
                    _channels[chan].Transfer = (TransferMode)((value & 0x0c) >> 2);
                    _channels[chan].AutoInit = ((value & 0x10) != 0);
                    _channels[chan].AddrDecrement = ((value & 0x20) != 0);
                    _channels[chan].Mode = (ChannelMode)((value & 0xc0) >> 6);
                    Log.Info(Category.Z80DMA, "Write 0x{0:x2} to channel {1} mode reg", value, chan);
                    break;

                case 0x4:   // Clear byte pointer flip flop
                    _channels[chan].LowByte = true;
                    break;

                case 0x5:   // Master clear
                    Reset();
                    break;

                case 0x7:   // Write all mask bits
                    _channels[0].Masked = (value & 0x01) > 0;
                    _channels[1].Masked = (value & 0x02) > 0;
                    _channels[2].Masked = (value & 0x04) > 0;
                    _channels[3].Masked = (value & 0x08) > 0;
                    break;

                default:
                    Log.Warn(Category.Z80DMA, "Bad write to port 0x{0:x2} (illegal address), ignored", portAddress);
                    break;
            }
        }


        // Debugging
        public void DumpStatus()
        {
            Console.WriteLine("i8237 DMAC status:");
            Console.WriteLine($"  Command: {_command}  Status: 0x{_status:x2}  IRQ: {_interruptEnabled}");
            Console.WriteLine($"  State: {_state}  Active: {_active}  Last: {_lastServed}");
            Console.WriteLine();
            Console.Write("Channels:");

            for (var c = 0; c < _channels.Length; c++)
            {
                if (_channels[c].Device == null) continue;

                Console.WriteLine($"\n  {c}  {_channels[c].ToString()}");
            }
        }


        [Flags]
        enum CommandBits
        {
            MemToMemEnable = 0x01,
            Chan0AddrChange = 0x02,
            MasterDisable = 0x04,
            CompressedTiming = 0x08,
            RotatingPriority = 0x10,
            ExtendedWrite = 0x20,
            DREQLow = 0x40,
            DACKHigh = 0x80
        }

        protected enum TransferMode
        {
            Verify = 0,
            Write = 1,
            Read = 2,
            Illegal = 3
        }

        protected enum ChannelMode
        {
            Demand = 0,
            Single = 1,
            Block = 2,
            Cascade = 3
        }

        enum DMAState
        {
            Idle = 0,
            SourceRead,
            DestWrite
        }

        /// <summary>
        /// Contains the internal byte count and vector response memory for a
        /// single channel (plus some extra data for convenient grouping).
        /// </summary>
        protected struct ChannelRegister
        {
            public IDMADevice Device;
            public byte DataPort;

            public bool Requested;          // DREQ active
            public bool Masked;             // Enable bit
            public TransferMode Transfer;   // Mode bits 2,3
            public bool AutoInit;           // Mode bit 4
            public bool AddrDecrement;      // Mode bit 5
            public ChannelMode Mode;        // Mode bits 6,7
            public bool Terminated;         // Completed or cancelled
            public bool LowByte;            // Flip flop for LSB/MSB for R/W

            public ushort BaseAddress;
            public ushort CurrentAddress;
            public ushort WordCount;
            public ushort CurrentCount;

            public bool CountComplete()
            {
                // Bump the address
                CurrentAddress += (ushort)(AddrDecrement ? -1 : 1);

                // Count is weird: it's always 1 more than programmed.  EOP
                // is triggered on wrap around from zero!  And, YES, wraparound
                // of a 16-bit ushort is exactly as the hardware does it. :-P
                CurrentCount--;

                if (CurrentCount == 0xffff)
                {
                    Terminated = true;
                }
                return Terminated;
            }

            public void Reinit()
            {
                if (AutoInit)
                {
                    CurrentAddress = BaseAddress;
                    CurrentCount = WordCount;
                    Terminated = false;
                    Log.Info(Category.Z80DMA, "{0} channel autoinitialized", Device);
                }
                else Masked = true;
            }

            public void CheckReady()
            {
                // Set the Requested flag if the channel is ready to go
                // Note: Use |= if software requests/block mode allowed...
                Requested = (!Masked && !Terminated &&
                             (((Transfer == TransferMode.Read) && Device.WriteDataReady) ||
                              ((Transfer == TransferMode.Write) && Device.ReadDataReady)));
            }

            public override string ToString()
            {
                if (Device == null) return "<No device attached>";

                return $"{Device} (RW port 0x{DataPort:x2})  Select={Mode}\n" +
                    $"  DREQ={Requested}  Masked={Masked}  TC={Terminated}  Auto={AutoInit}  Low={LowByte}\n" +
                    $"  Base=0x{BaseAddress:x4} ({WordCount})  Cur=0x{CurrentAddress:x4} ({CurrentCount})  Mode={Transfer}  AddrDec={AddrDecrement}";
            }
        }

        byte _status;                       // State of all channel req/eop bits
        byte _temporary;                    // Current "in flight" data byte

        int _active;
        int _lastServed;

        CommandBits _command;
        DMAState _state;
        bool _interruptEnabled;

        ChannelRegister[] _channels;
        Z80MemoryBus _memory;
        Z80IOBus _iobus;

        byte _controlBase;
        byte _channelBase;
        byte[] _ports;
    }
}
