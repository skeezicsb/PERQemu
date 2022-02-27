//
// PERQFifo.cs - Copyright (c) 2006-2022 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
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
using System.Threading;

using PERQemu.Processor;

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Input TO the Z80, FROM the PERQ.
    /// </summary>
    /// <remarks>
    /// For the original I/O board (IOB, CIO) this is a single 8-bit latch.  If
    /// requested by the PERQ, raises the Z80DataIn interrupt when the "fifo" is
    /// free and can accept new data.
    /// </remarks>
    public class PERQToZ80FIFO : IZ80Device
    {
        public PERQToZ80FIFO(PERQSystem system)
        {
            _system = system;
            _lock = new object();
        }

        public void Reset()
        {
            _fifo = 0;
            _valid = false;

            _interruptsEnabled = false;
            _interruptActive = false;

            // Assume these reset as well
            _dataReadyInterruptRequested = false;
            _system.CPU.ClearInterrupt(InterruptSource.Z80DataIn);

            Log.Debug(Category.FIFO, "PERQ->Z80 FIFO reset");
        }

        public string Name => "PERQ->Z80 FIFO";
        public byte[] Ports => _ports;
        public byte? ValueOnDataBus => 0x20;    // PRQVEC

        public bool IntLineIsActive
        {
            get { lock (_lock) { return (_interruptActive && _interruptsEnabled && _valid); } }
        }

        public bool InterruptsEnabled
        {
            get { return _interruptsEnabled; }
            set { _interruptsEnabled = value; }
        }

        public event EventHandler NmiInterruptPulse;

        /// <summary>
        /// Writes a byte from the PERQ to the Z80.
        /// </summary>
        /// <remarks>
        /// The IOB schematic (p. 49 of the PDF, "IOA DECODE") sheds some light
        /// on the Z80 "input" interrupt.  This is actually labeled as "Z80 READY
        /// INT L" (meaning it's active Low) and seems to be enabled by the PERQ
        /// sending data with bit 8 high, and can be dismissed by the PERQ sending
        /// data with bit 8 low.
        ///
        /// IOD 8 is latched on a WRITE from the Z80 and gated with the PERQ->Z80
        /// REQ L signal at a NAND gate.  So -- if the PERQ sets IOD 8 high with a
        /// write, and there is no pending PERQ->Z80 request, Z80 READY INT L will
        /// be low (triggered).
        /// </remarks>
        public void Enqueue(int value)
        {
            // Clear the DataInReady CPU interrupt
            _system.CPU.ClearInterrupt(InterruptSource.Z80DataIn);

            lock (_lock)
            {
                if (_valid)
                {
                    Log.Warn(Category.FIFO, "cycle {0}: PERQ overran latch, byte 0x{1:x2} will be lost", _system.IOB.Z80System.Clocks, _fifo);
                }

                // Latch the 8th bit of the incoming word
                _dataReadyInterruptRequested = ((value & 0x100) != 0);

                // Queue the byte
                _fifo = (byte)value;
                _valid = true;

                // Interrupt the Z80 to signal we have data to read
                _interruptActive = true;
            }

            // Logs are s l o w
            Log.Detail(Category.FIFO, "cycle {0}: PERQ wrote byte 0x{1:x2} to latch", _system.IOB.Z80System.Clocks, value);
        }

        /// <summary>
        /// Reads a byte from the PERQ.  This should only be called in response
        /// to an interrupt, and yet, the Z80 seems to just decide to call it
        /// whenever it damn well pleases.
        /// </summary>
        public byte Read(byte portAddress)
        {
            byte value = 0;

            lock (_lock)
            {
                // Clear the Z80 interrupt
                _interruptActive = false;

                if (_valid)
                {
                    value = _fifo;
                    _valid = false;

                    Log.Detail(Category.FIFO, "cycle {0}: Z80 read byte 0x{1:x2} from latch", _system.IOB.Z80System.Clocks, value);
                }
                else
                {
                    Log.Warn(Category.FIFO, "cycle {0}: Z80 read from empty latch, returning 0", _system.IOB.Z80System.Clocks);
                }

                // FIFO is empty; interrupt if the PERQ has asked us to
                if (_dataReadyInterruptRequested)
                {
                    _system.CPU.RaiseInterrupt(InterruptSource.Z80DataIn);
                }
            }

            return value;
        }

        public void Write(byte portAddress, byte value)
        {
            // Should never get called, this FIFO is read-only from the Z80 side.
            // If it does, we should yell about it.
            throw new NotImplementedException("Z80 write to read-only latch");
        }


        private bool _interruptActive;
        private bool _interruptsEnabled;
        private bool _dataReadyInterruptRequested;

        private byte _fifo;
        private volatile bool _valid;
        private object _lock;

        private byte[] _ports = { 0xa0 };   // PERQR

        private PERQSystem _system;
    }
}