//
// PERQToZ80FIFO.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
using System.Collections.Concurrent;

using PERQemu.Processor;

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Input TO the Z80, FROM the PERQ.
    /// </summary>
    /// <remarks>
    /// The EIO provides a 16 x 8 FIFO for short data transfers from the CPU to
    /// the Z80.  This allows command packets to be sent efficiently, as the
    /// hardware automatically interrupts the Z80 when data is available to read,
    /// and the main processor can query the status register to see when the FIFO
    /// has been emptied and the Z80 is ready for more input.
    /// </remarks>
    public class PERQToZ80FIFO : IZ80Device
    {
        public PERQToZ80FIFO(PERQSystem system)
        {
            _system = system;
            _fifo = new ConcurrentQueue<byte>();
            _interruptEnabled = false;
        }

        public void Reset()
        {
            byte punt;

            // Clear the decks
            while (!_fifo.IsEmpty) _fifo.TryDequeue(out punt);

            // Assume this resets as well
            _interruptEnabled = false;
            _perqIntRaised = false;
            _z80IntRaised = false;

            Log.Debug(Category.FIFO, "PERQ->Z80 FIFO reset");
        }

        public string Name => "PERQ->Z80 FIFO";
        public byte[] Ports => _ports;

        public byte? ValueOnDataBus => null;        // Supplied by the Am9519
        public bool IntLineIsActive => _z80IntRaised;

        public bool IsReady => _fifo.IsEmpty;

        /// <summary>
        /// PERQ access to the enabled flag from the IO bus control register.
        /// </summary>
        public bool InterruptEnabled
        {
            get { return _interruptEnabled; }

            set
            {
                _interruptEnabled = value;

                // Disabling dismisses the interrupt regardless of FIFO state?
                if (!_interruptEnabled)
                {
                    _system.CPU.ClearInterrupt(InterruptSource.Z80DataIn);
                    _perqIntRaised = false;
                }
            }
        }

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        //
        // From the PERQ
        //

        /// <summary>
        /// Writes a byte from the PERQ to the Z80 input FIFO.  This clears the
        /// PERQ interrupt and signals the Z80 interrupt controller that we have
        /// data to read.
        /// </summary>
        public void Enqueue(int value)
        {
            // Clear on every write?
            if (_perqIntRaised)
            {
                _system.CPU.ClearInterrupt(InterruptSource.Z80DataIn);
                _perqIntRaised = false;
            }

            if (_fifo.Count > 16)
            {
                Log.Warn(Category.FIFO, "PERQ overran FIFO, byte 0x{0:x2} will be lost", value);
                return;
            }

            // Queue the byte
            _fifo.Enqueue((byte)value);
            _z80IntRaised = true;

            Log.Detail(Category.FIFO, "PERQ wrote byte 0x{0:x2} to FIFO ({1} bytes)", value, _fifo.Count);
        }

        //
        // Z80 Bus
        //

        /// <summary>
        /// Handle reads from the Z80:
        ///     port 160Q (0x70) - data byte from the input FIFO from the PERQ
        /// </summary>
        public byte Read(byte portAddress)
        {
            byte value = 0;

            if (!_fifo.TryDequeue(out value))
            {
                _z80IntRaised = false;
                Log.Warn(Category.FIFO, "Z80 read from empty FIFO, returning 0");
                return 0;
            }

            Log.Detail(Category.FIFO, "Z80 read byte 0x{0:x2} from FIFO ({1} bytes left)", value, _fifo.Count);

            // Is the FIFO empty?  Then interrupt if the PERQ has asked us to
            // and deassert the Z80 IRQ (no more data to read)
            if (_fifo.IsEmpty)
            {
                _z80IntRaised = false;

                if (_interruptEnabled)
                {
                    _system.CPU.RaiseInterrupt(InterruptSource.Z80DataIn);
                    _perqIntRaised = true;
                }
            }

            return value;
        }

        public void Write(byte portAddress, byte value)
        {
            // Should never get called, this FIFO is read-only from the Z80 side.
            // If it does, we should yell about it.
            throw new NotImplementedException("Z80 write to read-only FIFO");
        }

        // debug
        public void DumpFifo()
        {
            Console.WriteLine($"PERQ->Z80 FIFO: IRQ enabled={_interruptEnabled}");

            Console.Write("PERQ->Z80 FIFO: ");

            if (_fifo.IsEmpty)
            {
                Console.WriteLine("is empty.");
                return;
            }

            var contents = _fifo.ToArray();

            foreach (var b in contents) Console.Write($" 0x{b:x2}");
            Console.WriteLine();
        }

        PERQSystem _system;
        ConcurrentQueue<byte> _fifo;

        bool _interruptEnabled;
        bool _perqIntRaised;
        bool _z80IntRaised;

        byte[] _ports = { 0x70 };  // PERQ.IN
    }
}

/*
    Notes:
    WRITE == PERQ TO EIO, READ == EIO to CPU (from EIO.doc)

    On the 225, IR means the memory is FULL; OR means it is NOT empty

    (PERQ)IOB -> (Z80)D produces PERQ INT when the FIFO is NOT EMPTY (OR),
    meaning the PERQ has sent a byte.  IR is not used.

    If the PtoZ FIFO is NOT empty:
        PERQ INT is true H; this is the Am9519 IREQ1 line 
        This reasserts the Z80 interrupt until all the bytes are read
        UPROC RDY is false L; this tells the PERQ the Z80 hasn't drained the data

    If the PtoZ FIFO IS empty:
        PERQ INT is false; no Z80 IRQ
        UPROC RDY is true, reflected in the and UPROC RDY ENB are both true, assert UPROC RDY INT
        (otherwise de-assert UPROC RDY INT)

    UPROC RDY is bit 6 in the Z80's status register, AND bit 7 in the PERQ's

    When the Z80 reads a byte, PERQ INT and UPROC RDY INT are dismissed, then
    reasserted if the FIFO isn't yet empty.  For efficiency, the PERQ interrupt
    doesn't need to be cleared/reasserted; because the PERQ INT is the highest
    priority on the Z80 side it'll immediately be retriggered in the next Z80
    instruction cycle.

    THUS:   IsReady tells the PERQ that the FIFO is EMPTY and can receive up to
            16 bytes (a quad word DMA transfer) or it can start a new message

            IsReady tells the Z80 that there are NO more bytes to read, which 
            is counterintuitive and why this is so fucking confusing

 */