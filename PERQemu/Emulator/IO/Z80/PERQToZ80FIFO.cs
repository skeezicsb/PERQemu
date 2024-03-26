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
using System.Collections.Generic;

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
            _lock = new object();
            _fifo = new Queue<byte>();
            _interruptEnabled = false;
        }

        public void Reset()
        {
            // Clear the decks
            _fifo.Clear();

            // Assume this resets as well
            _interruptEnabled = false;
            _perqIntRaised = false;
            _z80IntRaised = false;

            Log.Debug(Category.FIFO, "{0} reset", Name);
        }

        public string Name => "PERQ->Z80 FIFO";
        public byte[] Ports => _ports;

        public byte? ValueOnDataBus => null;        // Supplied by the Am9519
        public bool IntLineIsActive => _z80IntRaised;

        public bool IsReady
        {
            get { lock (_lock) { return _fifo.Count == 0; } }
        }

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

            lock (_lock)
            {
                if (_fifo.Count < 16)
                {
                    // Queue the byte
                    _fifo.Enqueue((byte)value);
                    _z80IntRaised = true;

                    Log.Detail(Category.FIFO, "PERQ wrote byte 0x{0:x2} to FIFO ({1} bytes)", value, _fifo.Count);
                }
                else
                {
                    Log.Warn(Category.FIFO, "PERQ overran FIFO, byte 0x{0:x2} will be lost", value);
                }
            }
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

            lock (_lock)
            {
                if (_fifo.Count > 0)
                {
                    value = _fifo.Dequeue();
                    Log.Detail(Category.FIFO, "Z80 read byte 0x{0:x2} from FIFO ({1} bytes left)", value, _fifo.Count);
                }
                else
                {
                    Log.Warn(Category.FIFO, "Z80 read from empty FIFO, returning 0");
                }

                // Is the FIFO empty?  Then interrupt if the PERQ has asked us to
                // and deassert the Z80 IRQ (no more data to read)
                if (_fifo.Count == 0)
                {
                    _z80IntRaised = false;

                    if (_interruptEnabled)
                    {
                        _system.CPU.RaiseInterrupt(InterruptSource.Z80DataIn);
                        _perqIntRaised = true;
                    }
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


        // Debugging
        public void DumpFifo()
        {
            Console.WriteLine($"PERQ->Z80 FIFO: IRQ enabled={_interruptEnabled}");

            Console.Write("PERQ->Z80 FIFO: ");

            if (_fifo.Count == 0)
            {
                Console.WriteLine("is empty.");
                return;
            }

            var contents = _fifo.ToArray();

            foreach (var b in contents) Console.Write($" 0x{b:x2}");
            Console.WriteLine();
        }


        Queue<byte> _fifo;
        readonly object _lock;

        bool _interruptEnabled;
        bool _perqIntRaised;
        bool _z80IntRaised;

        byte[] _ports = { 0x70 };  // PERQ.IN

        PERQSystem _system;
    }
}
