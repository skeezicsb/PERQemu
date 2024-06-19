//
// Z80ToPERQFIFO.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
    /// Output TO the PERQ, FROM the Z80.
    /// </summary>
    /// <remarks>
    /// For the EIO board this is an actual 16 x 8 FIFO, much like the original
    /// implementation from a much earlier version of PERQemu.  It also includes
    /// the control register, a one bit "output ready" flip flop that the Z80
    /// uses to trigger the PERQ's "uPROC INT" interrupt line (Z80DataOut).
    /// </remarks>
    public class Z80ToPERQFIFO : IZ80Device
    {
        public Z80ToPERQFIFO(PERQSystem system)
        {
            _system = system;
            _lock = new object();
            _fifo = new Queue<byte>();
            _interruptEnabled = false;
            _perqIntRaised = false;
            _outputReady = false;
        }

        public void Reset()
        {
            // Clear the queue
            _fifo.Clear();
            _outputReady = false;

            // Dismiss the interrupts
            InterruptEnabled = false;
            _z80IntRaised = false;

            Log.Debug(Category.FIFO, "{0} reset", Name);
        }

        public string Name => "Z80->PERQ FIFO";
        public byte[] Ports => _ports;

        public byte? ValueOnDataBus => null;        // Supplied by the Am9519
        public bool IntLineIsActive => _z80IntRaised;

        public bool IsReady => _outputReady;

        /// <summary>
        /// PERQ access to the enabled flag from the IO bus control register.
        /// </summary>
        public bool InterruptEnabled
        {
            get { return _interruptEnabled; }

            set
            {
                _interruptEnabled = value;

                // The PERQ can dismiss the interrupt even if there are still
                // bytes in the output queue!
                if (!_interruptEnabled && _perqIntRaised)
                {
                    _system.CPU.ClearInterrupt(InterruptSource.Z80DataOut);
                    _perqIntRaised = false;
                }
            }
        }

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        //
        // To the PERQ
        //

        /// <summary>
        /// Read a byte from the Z80 output FIFO to the PERQ.  Also asserts the
        /// Z80 "RD UPROC DATA L" interrupt (which may be masked by the 9519).
        /// </summary>
        public byte Dequeue()
        {
            byte value = 0;

            lock (_lock)
            {
                if (_fifo.Count > 0)
                {
                    value = _fifo.Dequeue();
                    Log.Detail(Category.FIFO, "PERQ read byte 0x{0:x2} from FIFO ({1} bytes left)", value, _fifo.Count);
                    _z80IntRaised = true;
                }
                else
                {
                    Log.Warn(Category.FIFO, "PERQ read from empty FIFO, returning 0");
                }

                // If the FIFO is empty, drop the interrupts
                if (_fifo.Count == 0)
                {
                    _z80IntRaised = false;

                    if (_perqIntRaised)
                    {
                        _system.CPU.ClearInterrupt(InterruptSource.Z80DataOut);
                        _perqIntRaised = false;
                    }
                }
            }

            return value;
        }

        //
        // Z80 Bus
        //

        /// <summary>
        /// Handle reads from the Z80:
        ///     port 162Q (0x72) - FIFO status register
        /// </summary>
        /// <remarks>
        /// Read the Z80's copy of the two bit FIFO status register.  The format
        /// is slightly different and the bits aren't quite the same as the PERQ's
        /// status (returned by the EIO board):
        ///     bit 7 - set if the Z80->PERQ FIFO is not full
        ///     bit 6 - set if PERQ->Z80 FIFO is not empty
        /// To access the other FIFO, we just read the status word from the public
        /// interface and copy the appropriate bit.  And they say cheaters never win.
        /// </remarks>
        public byte Read(byte portAddress)
        {
            // Status register?
            if (portAddress == 0x72)
            {
                lock (_lock)
                {
                    // Set our local status
                    var result = (byte)(_fifo.Count < 16 ? 0x80 : 0);

                    // Check the other FIFO
                    var status = _system.IOB.Z80System.ReadStatus();
                    result |= (byte)((status & 0x0080) != 0 ? 0x40 : 0);

                    // Turn off our interrupt, which is (weirdly) driven directly by
                    // the IOA decoder into the Z80's interrupt controller... :-(
                    // Either the schematics are wrong, or the ONLY way the Z80 can
                    // dismiss this is by masking off the IRQ in the Am9519, OR the
                    // signal is toggled off again when a different IOA address on
                    // the EIO is accessed.  This is a little bizarre and requires
                    // more study.
                    _z80IntRaised = _outputReady;

                    Log.Debug(Category.FIFO, "Z80 read FIFO status 0x{0:x}", result);
                    return result;
                }
            }

            // This FIFO is write-only from the Z80 side; this shouldn't happen
            throw new InvalidOperationException("Z80 read from write-only FIFO");
        }

        /// <summary>
        /// Handle writes from the Z80:
        ///     port 170Q (0x78) - control register to set the output ready bit
        ///     port 161Q (0x71) - data bytes into the output FIFO
        /// 
        /// (This avoids setting up another IZ80Device for a 1-bit flip flop...)
        /// </summary>
        public void Write(byte portAddress, byte value)
        {
            lock (_lock)
            {
                if (portAddress == 0x78)
                {
                    // Control port: set or clear the output ready bit
                    _outputReady = ((value & 0x1) != 0);
                    Log.Debug(Category.FIFO, "Z80 output ready now {0}", _outputReady);

                    // After a reset?
                    _z80IntRaised = _outputReady;

                    // Fall through to fire the CPU interrupt if necessary
                }
                else
                {
                    // Data port: queue the byte if there's room
                    if (_fifo.Count < 16)
                    {
                        _fifo.Enqueue(value);
                        Log.Detail(Category.FIFO, "Z80 wrote byte 0x{0:x2} to FIFO ({1} bytes)", value, _fifo.Count);
                    }
                    else
                    {
                        Log.Warn(Category.FIFO, "Z80 overran FIFO, byte 0x{0:x2} will be lost", value);
                    }
                }

                // Let the PERQ know there's data available
                if (_interruptEnabled && _outputReady && _fifo.Count > 0)
                {
                    _system.CPU.RaiseInterrupt(InterruptSource.Z80DataOut);
                    _perqIntRaised = true;
                }
            }
        }


        // Debugging
        public void DumpFifo()
        {
            Console.WriteLine($"Z80->PERQ FIFO: Enabled={_interruptEnabled} Ready={_outputReady}");

            Console.Write("Z80->PERQ FIFO: ");

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

        bool _outputReady;
        bool _interruptEnabled;
        bool _perqIntRaised;
        bool _z80IntRaised;

        // PERQ.OUT, status register, output ready flip flop
        byte[] _ports = { 0x71, 0x72, 0x78 };

        PERQSystem _system;
    }
}
