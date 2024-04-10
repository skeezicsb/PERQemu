//
// PERQDMA.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Simulates the EIO's special PERQ-Z80 DMA channel.
    /// </summary>
    /// <remarks>
    /// The custom EIO PERQ-Z80 DMA channel is a pair of 16-word deep FIFOs
    /// *in addition to* and separate from the pair used for regular protocol
    /// exchange between the two processors.  Here the PERQ side is read and
    /// written as 16 bit words (four at a time) on the main memory buses by
    /// the PERQ's DMA controller.  The Z80 side is accessed a byte-at-a-time
    /// by the Z80's DMA controller.  This arrangement allows memory-to-memory
    /// transfers with almost no software intervention.
    /// </remarks>
    public class PERQDMA : IZ80Device, IDMADevice
    {
        public PERQDMA(PERQSystem sys)
        {
            _system = sys;
            _toZ80 = new Queue<byte>();
            _toPerq = new Queue<ushort>();
        }

        public string Name => "PERQ-Z80 DMA";
        public byte[] Ports => _ports;

        public bool IntLineIsActive => false; // _z80IntRaised;   // Hmm.  Problematic?
        public byte? ValueOnDataBus => null; // AckNull;         // Supplied by the Am9519

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        //
        // Z80 DMA Interface
        //

        public bool ReadDataReady => _readReady;
        public bool WriteDataReady => _writeReady;

        public byte? AckNull
        {
            get { _z80IntRaised = false; return null; }
        }

        public void DMATerminate()
        {
            _readReady = false;
            _writeReady = false;
            _z80IntRaised = true;

            Log.Info(Category.DMA, "PDMA operation terminated");
        }

        //
        // Z80 Bus Interface
        //

        public void Reset()
        {
            _z80IntRaised = false;
            _highByte = true;
            _start = false;
            _holding = 0;
            _address = 0;

            _toZ80.Clear();
            _toPerq.Clear();

            _readReady = false;
            _writeReady = false;

            Log.Info(Category.DMA, "{0} reset", Name);
        }

        /// <summary>
        /// Z80 bus port for reading PDMA data from the PERQ since the Z80dotNet
        /// CPU doesn't simulate bus requests from the DMAC.  This "fake" read
        /// port gives the i8237 a device to read from when the PERQ is sending
        /// data.  Port 165Q (0x75) is not used by the real hardware but may be
        /// reassigned arbitrarily if any conflict is found with a real device.
        /// </summary>
        public byte Read(byte portAddress)
        {
            if (portAddress == 0x75)
            {
                // Fetch another quad?
                if (_toZ80.Count % 8 == 0) Enqueue();

                // If there's data it must be good :-)
                if (_toZ80.Count > 1)
                {
                    byte val = _toZ80.Dequeue();
                    Log.Info(Category.DMA, "Read 0x{0:x2} from PDMA FIFO", val);
                    return val;
                }
#if DEBUG
                // This can't happen... shut it down... 
                if (_toZ80.Count == 0)
                {
                    Console.WriteLine("Read from empty FIFO!?");
                    _readReady = false;
                    return 0;
                }
#endif
            }

            Log.Warn(Category.DMA, "Unhandled read from port 0x{0:x2}", portAddress);
            return 0;
        }

        /// <summary>
        /// Z80 bus ports for managing the PERQ-Z80 DMA FIFOs:
        ///     PDMAStart  equ 163Q    ; Force PERQ DMA cycle to fill/empty FiFos
        ///     PDMAFlush  equ 164Q    ; Flush DMA FiFo to PERQ
        ///     PDMADirect equ 172Q    ; 0 = DMA from PERQ, 1 = DMA to PERQ
        /// 
        /// Additionally, the fake port (0x75) can be written here too; see above.
        /// </summary>
        public void Write(byte portAddress, byte value)
        {
            switch (portAddress)
            {
                // Set direction bit
                case 0x7a:
                    //
                    // This is called first when setting up a new operation.  Set
                    // the start flag here so our first PERQ memory access (either
                    // way) will load the starting address from the DMA registers.
                    //
                    _dmaToPerq = (value != 0);
                    _start = true;
                    _z80IntRaised = false;

                    Log.Info(Category.DMA, "PDMA direction set to {0} ({1})", _dmaToPerq, value);
                    break;

                // Flush both FIFOs
                case 0x74:
                    //
                    // Second step in the EIO firmware's standard setup, after the
                    // direction is set.  If sending TO the PERQ we have to set an
                    // initial _writeReady condition or the DMAC won't start things
                    // rolling when the channel mask is reset.
                    //
                    _toZ80.Clear();
                    _toPerq.Clear();

                    _highByte = true;
                    _writeReady = _dmaToPerq;
                    _z80IntRaised = false;

                    Log.Info(Category.DMA, "PDMA FIFOs flushed");
                    break;

                // Start (or finish!) a DMA transaction
                case 0x73:
                    if (_dmaToPerq)
                    {
                        //
                        // This is actually called at the END of a transaction
                        // (AFTER DMATerminate) to flush any residual bytes to
                        // the PERQ.  At most there should only be one quad left
                        // (or less) but loop until clear to be safe.
                        //
                        while (Dequeue(true)) { }

                        _z80IntRaised = true;
                        Log.Write("Z80 to PERQ operation complete");
                    }
                    else
                    {
                        Log.Write("PERQ to Z80 operation starting");

                        // Prime the _toZ80 FIFO with the first quad word
                        Enqueue();

                        // Tell the i8237 that we're ready to go
                        _readReady = true;
                    }
                    break;

                // "Fake" write data port
                case 0x75:
                    //
                    // Reassemble two bytes from the Z80 into 16-bit words and
                    // queue them up for the PERQ.
                    //
                    if (_highByte)
                    {
                        _holding = value;
                    }
                    else
                    {
                        var word = (ushort)((value << 8) | _holding);
                        _toPerq.Enqueue(word);

                        Log.Debug(Category.DMA, "Enqueued word 0x{0:x4} to PDMA FIFO (count {1})",
                                                word, _toPerq.Count);
                    }

                    // Toggle our flipflop
                    _highByte = !_highByte;

                    // Tell the PERQ to dequeue if we have at least one quad's
                    // worth (safe to call more frequently, but wasted overhead)
                    if (_toPerq.Count >= 4) Dequeue();
                    break;

                default:
                    Log.Warn(Category.DMA, "Unhandled write 0x{0:x2} to port 0x{1:x2}", value, portAddress);
                    break;
            }
        }

        //
        // PERQ Interface
        //

        /// <summary>
        /// Simulates the PERQ side of the DMA interface writing TO the Z80.
        /// If there's room in the queue, loads another quad word into the Z80's
        /// FIFO as 8 bytes and advances the address.
        /// </summary>
        void Enqueue()
        {
            // If there's no room at the inn, bail now
            if (_toZ80.Count > 24) return;

            // New transaction?
            if (_start)
            {
                _address = _system.IOB.DMARegisters.GetDataAddress(ChannelName.uProc);
                _start = false;
            }

            // Just log the start (and assume it's properly aligned)
            Log.Debug(Category.DMA, "Write 4 words to Z80 FIFO from addr 0x{0:x6}", _address);

            // Queue up four more words for the Z80
            for (int i = 0; i < 4; i++)
            {
                var value = _system.Memory.FetchWord(_address++);

                _toZ80.Enqueue((byte)((value >> 8) & 0xff));
                _toZ80.Enqueue((byte)value);
            }
        }

        /// <summary>
        /// Reads (up to) four words from the Z80 and writes them to main memory,
        /// advancing the address appropriately.  No-op if not enough words in
        /// the queue, unless told to flush.  Returns true if there are unread
        /// words in the FIFO.
        /// </summary>
        /// <remarks>
        /// The PERQ DMA controller always writes quads; if the Z80 is a few words
        /// short of a happy meal, I can only assume that the write to the "start"
        /// register (per DMA.EIO source) forces the last good data to be padded
        /// with whatever random bytes end up on the bus after the FIFO empties.
        /// Here we just stop short rather than risk an overrun, although I assume
        /// they always sized the buffers appropriately and that in practice short
        /// transfers are from errors or deliberately cancelling an operation.
        /// </remarks>
        bool Dequeue(bool flush = false)
        {
            if (_start)
            {
                _address = _system.IOB.DMARegisters.GetDataAddress(ChannelName.uProc);
                _start = false;
            }

            // Take up to one quad's worth
            var count = _toPerq.Count > 4 ? 4 : _toPerq.Count;

            // Bail out if we don't have enough and aren't flushing
            if (count == 0) return false;
            if (count < 4 && !flush) return true;

            Log.Debug(Category.DMA, "Read {0} words from PERQ FIFO to addr 0x{1:x6}", count, _address);

            // Read and store a quad's worth from the Z80-to-PERQ queue
            for (int i = 0; i < count; i++)
            {
                ushort value = _toPerq.Dequeue();
                _system.Memory.StoreWord(_address++, value);
            }

            return _toPerq.Count > 0;
        }


        // Debugging
        public void DumpFIFOs()
        {
            Console.WriteLine("PDMA Status:");
            Console.WriteLine($"  Dir={_dmaToPerq}  Addr=0x{_address:x6}  Holding={_holding} (hi={_highByte})");
            Console.WriteLine($"  ReadRdy={_readReady}  WriteRdy={_writeReady}  IRQ={_z80IntRaised}");
            Console.WriteLine();

            Console.Write("DMA to PERQ: ");

            if (_toPerq.Count > 0)
            {
                var words = _toPerq.ToArray();
                for (var i = 0; i < words.Length; i++)
                    Console.Write($" 0x{words[i]:x4}");

                Console.WriteLine();
            }
            else
            {
                Console.WriteLine("is empty.");
            }

            Console.Write("DMA to Z80:  ");

            if (_toZ80.Count > 0)
            {
                var bytes = _toZ80.ToArray();
                for (var i = 0; i < bytes.Length; i++)
                    Console.Write($" 0x{bytes[i]:x2}");

                Console.WriteLine();
            }
            else
            {
                Console.WriteLine("is empty.");
            }
        }


        // Interrupt/DMA status
        bool _z80IntRaised;

        bool _dmaToPerq;
        bool _highByte;
        byte _holding;

        bool _start;
        int _address;

        bool _readReady;
        bool _writeReady;

        Queue<byte> _toZ80;
        Queue<ushort> _toPerq;

        byte[] _ports = { 0x73, 0x74, 0x75, 0x7a };

        PERQSystem _system;
    }
}

/*
    Notes:

    This class fakes up the PERQ-Z80 DMA channel, consisting of a separate set
    of 16-deep FIFOs that are wired into both DMA controllers.  This allows for
    memory-to-memory transfers between the processors with virtually no CPU
    intervention, and it allows the PERQ to dynamically load code into Z80 RAM
    (as the ZBoot process in POS G does).  This means patches or new functions
    can be added to the system without burning PROMs.  That's going to require
    huge changes to how the Z80 Debugger works here... 
    
    Two interesting side effects of this arrangement as it pertains to emulation:

        If it can run entirely on the Z80 thread that avoids concurrency issues;
        because the PERQ side is "fake" (we don't actually implement the DMA
        machinery faithfully) it means we assume that when the PERQ and Z80
        are coordinating a transaction they DO NOT touch the memory locations
        (in either memory space) until the DMA hardware's completion interrupts
        fire.  This means no locking necessary!  Handwave!  Fingers crossed!
        Wishful thinking!  LA LA LA LA LA I can't hear you neener neener

        Secondly, because the PERQ's memory bus is so much faster than the
        Z80's, I don't think any special timing or delays are needed; when
        the Z80 initiates a read or write, we can literally just do the PERQ
        memory accesses (always in quad words, so 8 bytes at a time) and 
        push the data into the FIFOs; the regular Z80 DMA processing then
        makes it seem like the hardware is doing it at the Z80's normal
        speed, interleaved with instruction execution like any other DMAC
        transaction.  All we miss is a handful of 680ns memory cycles that
        might cause CPU stalls, but so far that's been acceptable for pretty
        realistic emulation.  We can always revisit and refactor this once
        the rest of the PERQ-2/EIO emulation is completed.  Functionality
        first, performance and accuracy second. :-)

    There are tons of unanswered or unexplored questions, but thankfully there
    is enough source code around (and even a few helpful docs) to shed light on
    how this works.  Because there are two sets of FIFOs, it would SEEM that
    there could be data queued up in both directions at once, but I'm pretty
    sure that's not how they're used in practice.  Will find out...
 */
