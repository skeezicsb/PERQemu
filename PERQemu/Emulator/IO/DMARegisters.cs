//
// DMARegisters.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.Processor;

namespace PERQemu.IO
{
    /// <summary>
    /// A superset of the DMA channel descriptions for IOB/CIO and EIO.
    /// </summary>
    public enum ChannelName
    {
        Unused = 0,
        uProc = 1,
        HardDisk = 2,
        Network = 3,
        NetXmit = 3,
        ExtA = 4,
        ExtB = 5,
        NetRecv = 6,
        Idle = 7
    }

    public delegate void LoadRegisterDelegate(ChannelName chan, int value);

    /// <summary>
    /// A common class to handle the DMA registers on the IO bus.
    /// </summary>
    /// <remarks>
    /// The PERQ's DMA strategy is distributed across the CPU, MEM and IO slots,
    /// but for emulation purposes we simply provide a common way to access the
    /// registers used to program the transfers -- the device implementations
    /// that use DMA (hard disk, Ethernet, Canon, etc) don't have to manage any
    /// of that themselves.  At some point the DMA could/should actually request
    /// cycles from the CPU, implementing the actual RQST/GRANT protocol and
    /// executing memory cycles, pre-empting the processor, respecting the
    /// microcode "H" bit, etc.  It'd make for a much more accurate experience,
    /// but that little touch of extra realism is likely a massive can of worms
    /// to try to debug. :-)  For now we fake it (pretty convincingly).
    /// </remarks> 
    public class DMARegisterFile
    {
        public DMARegisterFile()
        {
            _portToChannelAction = new Dictionary<byte, LoadRegisterDelegate>();

            _channels = new DMAChannel[] {
                new DMAChannel(ChannelName.Unused),
                new DMAChannel(ChannelName.uProc),
                new DMAChannel(ChannelName.HardDisk),
                new DMAChannel(ChannelName.NetXmit),
                new DMAChannel(ChannelName.ExtA),
                new DMAChannel(ChannelName.ExtB),
                new DMAChannel(ChannelName.NetRecv),
                new DMAChannel(ChannelName.Idle)
            };
        }

        public void Clear()
        {
            for (var i = 0; i < _channels.Length; i++)
            {
                _channels[i].DataAddr.Value = 0;
                _channels[i].HeaderAddr.Value = 0;
            }
        }

        public void Assign(byte dataHi, byte dataLo, byte hdrHi, byte hdrLo)
        {
            // Four actions for every address pair
            _portToChannelAction.Add(dataHi, LoadDataHigh);
            _portToChannelAction.Add(dataLo, LoadDataLow);
            _portToChannelAction.Add(hdrHi, LoadHeaderHigh);
            _portToChannelAction.Add(hdrLo, LoadHeaderLow);

            Log.Info(Category.DMA, "DMA mapping assigned for ports {0:x}, {1:x}, {2:x}, {3:x}",
                                    dataHi, dataLo, hdrHi, hdrLo);
        }

        public void LoadRegister(ChannelName chan, byte port, int value)
        {
            LoadRegisterDelegate doit;

            if (_portToChannelAction.TryGetValue(port, out doit))
            {
                doit(chan, value);
                return;
            }

            Log.Warn(Category.DMA, "Could not load DMA register 0x{0:2x}, delegate not found!", port);
        }

        public void LoadDataHigh(ChannelName chan, int value)
        {
            _channels[(int)chan].DataAddr.Hi = ~value;
            Log.Debug(Category.DMA, "{0} data buffer addr (high) set to {1:x}", chan, value);
        }

        public void LoadDataLow(ChannelName chan, int value)
        {
            _channels[(int)chan].DataAddr.Lo = (ushort)value;
            Log.Debug(Category.DMA, "{0} data buffer addr (low) set to {1:x4}", chan, value);
        }

        public void LoadHeaderHigh(ChannelName chan, int value)
        {
            _channels[(int)chan].HeaderAddr.Hi = ~value;
            Log.Debug(Category.DMA, "{0} header buffer addr (high) set to {1:x}", chan, value);

            // If EIO header count is bits <7:4> of this word (irrelevant for IOB)
            _channels[(int)chan].HeaderCount = (byte)(~(value >> 4) & 0x0f);
        }

        public void LoadHeaderLow(ChannelName chan, int value)
        {
            _channels[(int)chan].HeaderAddr.Lo = (ushort)value;
            Log.Debug(Category.DMA, "{0} header buffer addr (low) set to {1:x4}", chan, value);
        }

        public int GetDataAddress(ChannelName chan)
        {
            return Unfrob(_channels[(int)chan].DataAddr);
        }

        public int GetHeaderAddress(ChannelName chan)
        {
            return Unfrob(_channels[(int)chan].HeaderAddr);
        }

        public byte GetHeaderCount(ChannelName chan)
        {
            return _channels[(int)chan].HeaderCount;
        }

        /// <summary>
        /// Unfrob a DMA address like the hardware do.  This is common to IOB
        /// and CIO, and any OIO device that uses DMA.  For EIO, the hardware
        /// doesn't invert the high bits, but it does use one nibble to encode
        /// the (inverted) word count for header transfers.
        /// </summary>
        /// <remarks>
        ///                                 ! Explained:
        /// tmp := 176000;                  ! Need to compliment upper 6 bits
        ///                                 ! of buffer address. The hardware
        ///                                 ! has an inversion for the upper
        ///                                 ! 10 bits of the 20 bit address.
        /// Buffer xor tmp, IOB(LHeadAdrL); ! Send lower 16 bits of logical
        ///                                 ! header address to channel ctrl.
        /// not 0, IOB(LHeadAdrH);          ! Send higher 4 bits of logical
        ///                                 ! header address to channel ctrl.
        ///                                 ! Remember, these bits are inverted.
        /// </remarks>
        public virtual int Unfrob(ExtendedRegister addr)
        {
            int unfrobbed;

            if (PERQemu.Sys.IOB.IsEIO)
            {
                // For EIO, only the upper bits are inverted
                unfrobbed = (~addr.Hi & 0x0f0000) | addr.Lo;
            }
            else
            {
                // Hi returns the upper 4 or 8 bits shifted; Lo needs to be unfrobbed
                unfrobbed = addr.Hi | (~(0x3ff ^ addr.Lo) & 0xffff);
            }
            Log.Detail(Category.DMA, "Unfrobbed {0:x} -> {1:x}", addr.Value, unfrobbed);

            return unfrobbed;
        }

        // Debugging
        public void DumpDMARegisters()
        {
            Console.WriteLine("DMA registers:");

            for (var i = ChannelName.uProc; i < ChannelName.Idle; i++)
            {
                Console.WriteLine(_channels[(int)i]);
                Console.WriteLine("Unfrobbed:          {0:x6}        {1:x6}",
                                  GetHeaderAddress(i), GetDataAddress(i));
            }
        }

        internal struct DMAChannel
        {
            public DMAChannel(ChannelName chan)
            {
                Name = chan;

                // Should be 20 bits for IOB/CIO, 20 or 24 bits for EIO!
                HeaderAddr = new ExtendedRegister(CPUBoard.CPUBits - 16, 16);
                DataAddr = new ExtendedRegister(CPUBoard.CPUBits - 16, 16);

                // For EIO, bits <7:4> of the high header address are the complement
                // of the (quad) word count.  Provided for convenience :-)
                HeaderCount = 0;
            }

            public override string ToString()
            {
                return string.Format("{0}  Header: {1:x6}  Data: {2:x6}  Count: {3}",
                                     $"{Name}".PadRight(10), HeaderAddr.Value, DataAddr.Value, HeaderCount);
            }

            public ChannelName Name;
            public ExtendedRegister HeaderAddr;
            public ExtendedRegister DataAddr;

            public byte HeaderCount;
        }

        DMAChannel[] _channels;
        Dictionary<byte, LoadRegisterDelegate> _portToChannelAction;
    }
}
