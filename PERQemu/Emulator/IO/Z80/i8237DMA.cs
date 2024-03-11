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

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// AMD Am9517 (aka Intel i8237) DMA chip.
    /// </summary>
    /// <remarks>
    /// This four-channel DMA controller is used on the EIO board, replacing
    /// the single-channel Z80 DMA chip from the older IO boards.  This chip
    /// uses two blocks of addresses: control/status registers in one block,
    /// and four address/word count registers in a second.
    /// 
    /// DREQ channel assignments are fixed on the EIO:
    ///     Chn 0 - Floppy      Chn 2 - SIO
    ///     Chn 1 - GPIB        Chn 3 - PERQ
    /// </remarks>
    public class i8237DMA : IZ80Device
    {
        public i8237DMA(byte csrBase, byte chnBase)
        {
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

        // Fixme: placeholders until implemented so we can start debugging
        public bool IntLineIsActive => false;
        public byte? ValueOnDataBus => null;        // Supplied by the Am9519

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        public void Reset()
        {
            // Hardware reset or Master Clear command
            _channels[0].Masked = true;
            _channels[0].LowByte = true;
            _channels[1].Masked = true;
            _channels[1].LowByte = true;
            _channels[2].Masked = true;
            _channels[2].LowByte = true;
            _channels[3].Masked = true;
            _channels[3].LowByte = true;
            _command = 0;
            _temporary = 0;
            Log.Debug(Category.Z80DMA, "i8237 reset");
        }

        public void Clock()
        {
            // Todo: Run the state machine!
        }

        /// <summary>
        /// Recompute the status register with current request and terminated
        /// status of all four channels.
        /// </summary>
        void UpdateStatus()
        {
            var ostatus = _status;

            // Just inline this for simplicity
            _status = (byte)(_channels[0].Requested ? 0x01 : 0);
            _status |= (byte)(_channels[0].Terminated ? 0x10 : 0);
            _status |= (byte)(_channels[1].Requested ? 0x02 : 0);
            _status |= (byte)(_channels[1].Terminated ? 0x20 : 0);
            _status |= (byte)(_channels[2].Requested ? 0x04 : 0);
            _status |= (byte)(_channels[2].Terminated ? 0x40 : 0);
            _status |= (byte)(_channels[3].Requested ? 0x08 : 0);
            _status |= (byte)(_channels[3].Terminated ? 0x80 : 0);

            if (ostatus != _status)
            {
                Log.Info(Category.Z80DMA, "Status change: 0x{0:x2}", _status);  // todo: a toBinary() might be nice here
            }
        }


        public byte Read(byte portAddress)
        {
            if (portAddress == _controlBase)
            {
                UpdateStatus();

                byte save = _status;
                Log.Debug(Category.Z80DMA, "Read 0x{0:x2} from status register", save);

                // Reading status clears the EOP bits (S3..S0)!
                _status &= 0xf0;

                return save;
            }

            if (portAddress == _controlBase + 5)
            {
                Log.Debug(Category.Z80DMA, "Read 0x{0:x2} from temporary reg", _temporary);
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

                // Log it
                Log.Info(Category.Z80DMA, "Read 0x{0:x2} from channel {1} current {2} ({3} byte)",
                                          half, chan,
                                          (addr ? "address" : "word count"),
                                          (_channels[chan].LowByte ? "low" : "high"));

                // Toggle the flip flop for the next read
                _channels[chan].LowByte = !_channels[chan].LowByte;
                return half;
            }

            Log.Warn(Category.Z80DMA, "Bad read from port 0x{0:x2} (illegal address)", portAddress);
            // throw ...
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
                    Log.Debug(Category.Z80DMA, "Write 0x{0:x2} ({1}) to command register",
                                                value, _command);
                    break;

                case 0x1:   // Write request bit
                    _channels[chan].Requested = (value & 0x04) > 0;
                    Log.Debug(Category.Z80DMA, "Channel {0} Requested is {1}", chan, _channels[chan].Requested);
                    break;

                case 0x2:   // Write single mask bit
                    _channels[chan].Masked = (value & 0x04) > 0;
                    Log.Debug(Category.Z80DMA, "Channel {0} Masked is {1}", chan, _channels[chan].Masked);
                    break;

                case 0x3:   // Write mode register
                    _channels[chan].Mode = (TransferMode)((value & 0x0c) >> 2);
                    _channels[chan].AutoInit = ((value & 0x10) != 0);
                    _channels[chan].AddrDecrement = ((value & 0x20) != 0);
                    _channels[chan].Select = (ChannelMode)((value & 0xc0) >> 6);
                    Log.Debug(Category.Z80DMA, "Write 0x{0:x2} to channel {1} mode reg", value, chan);
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
                    // throw
                    break;
            }
        }

        [Flags]
        enum CommandBits
        {
            MemToMemEnable = 0x1,
            Chan0AddrChange = 0x2,
            MasterEnable = 0x4,
            CompressedTiming = 0x8,
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

        /// <summary>
        /// Contains the internal byte count and vector response memory for a
        /// single channel (plus some extra data for convenient grouping).
        /// </summary>
        protected struct ChannelRegister
        {
            public bool Requested;          // DREQ active
            public bool Masked;             // Enable bit
            public TransferMode Mode;       // Mode bits 2,3
            public bool AutoInit;           // Mode bit 4
            public bool AddrDecrement;      // Mode bit 5
            public ChannelMode Select;      // Mode bits 6,7
            public bool Terminated;         // Completed or cancelled
            public bool LowByte;            // Flip flop for LSB/MSB for R/W
            public ushort BaseAddress;
            public ushort CurrentAddress;
            public ushort WordCount;
            public ushort CurrentCount;
        }

        byte _status;                       // State of all channel req/eop bits
        byte _temporary;                    // Current "in flight" data byte
        CommandBits _command;

        ChannelRegister[] _channels;

        byte _controlBase;
        byte _channelBase;
        byte[] _ports;
    }
}

/*
    Notes:
    
    ;
    ;        DMA
    ;
    DMACSR      equ 070Q    ; DMA control and status
    DMAREQ      equ 071Q    ; DMA request register
    DMAMASK     equ 072Q    ; DMA Mask register
    D.Floppy    equ 00H     ; Channel 0 select
    D.GPIB      equ 01H     ; Channel 1 select
    D.SIO       equ 02H     ; Channel 2 select
    D.PERQ      equ 03H     ; Channel 3 select
    D.Set       equ 04H     ; Set mask register bit
    D.Clear     equ 00H     ; Clear mask register bit
    DMAMODE     equ 073Q    ; DMA Mode register
    D.Read      equ 008H    ; Read transfer
    D.Write     equ 004H    ; Write transfer
    D.AutoInit  equ 010H    ; Autoinitialize
    D.Incr      equ 000H    ; Increment mode selected
    D.Decr      equ 020H    ; Decrement mode selected
    D.Demand    equ 000H    ; Demand mode transfer
    D.Single    equ 040H    ; Single transfer
    D.Block     equ 080H    ; Block mode transfer
    DMAPOINT    equ 074Q    ; DMA clear pointer register
    DMATEMP     equ 075Q    ; DMA Temporary register(Read Only)
    DMAMCLR     equ 075Q    ; DMA Master Clear(Write Only)
    DMACLRMASK  equ 076Q    ; DMA Clear mask register
    DMASETMASK  equ 077Q    ; DMA Set/Clear Mask bits

    DMAADR0     equ 060Q    ; DMA Channel 0 address
    DMAWC0      equ 061Q    ; DMA Channel 0 word count
    DMAADR1     equ 062Q    ; DMA Channel 1 address
    DMAWC1      equ 063Q    ; DMA Channel 1 word count
    DMAADR2     equ 064Q    ; DMA Channel 2 address
    DMAWC2      equ 065Q    ; DMA Channel 2 word count
    DMAADR3     equ 066Q    ; DMA Channel 3 address
    DMAWC3      equ 067Q    ; DMA Channel 3 word count


    PDMAStart   equ 163Q    ; Force PERQ DMA cycle to fill/empty FiFos
    PDMAFlush   equ 164Q    ; Flush DMA FiFo to PERQ
    PDMADirect  equ 172Q    ; 0 = DMA from PERQ, 1 = DMA to PERQ

 */
