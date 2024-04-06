//
// Am9519.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
    /// The AMD Am9519 interrupt priority encoder, used by the EIO Z80.
    /// </summary>
    /// <remarks>
    /// On EIO the Z80 interrupt chain is a hybrid of two approaches:  both SIO
    /// chips are directly wired to the Z80 INT line, with the other IRQ sources
    /// managed by the Am9519 which can mask, prioritize and assert/acknowledge
    /// interrupts on their behalf.  This keeps the EI->EO chain to just three
    /// chips.  This implementation just uses the IZ80InterruptSource methods as
    /// the Z80dotNet CPU does so controllers like the FDC and GPIB chips don't
    /// have to "know" which board they're attached to!
    /// </remarks>
    public class Am9519 : IZ80Device
    {
        public Am9519(byte baseAddress)
        {
            _baseAddress = baseAddress;
            _ports = new byte[] { _baseAddress, (byte)(_baseAddress + 1) };

            _registers = new byte[4];

            // Do initial allocation of the IRQ vector memory
            _response = new ResponseRegister[8];

            for (var i = 0; i < 8; i++)
            {
                _response[i].Device = null;
                _response[i].ByteCount = 0;
                _response[i].Counter = 0;
                _response[i].Vector = 0;
            }
        }

        public string Name => "Am9519";
        public byte[] Ports => _ports;

        public bool IntLineIsActive => _interruptEnabled;
        public byte? ValueOnDataBus => GetActiveVector();

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        public void Reset()
        {
            // At power up OR software reset
            _mode = 0;
            _status = 0;
            _registers[IRR] = 0;
            _registers[ISR] = 0;
            _registers[ACR] = 0;
            _registers[IMR] = 0xff;

            _regSelect = 0;
            _active = -1;
            _busy = false;
            _loadVector = false;
            _interruptEnabled = false;

            // The Byte Count and Vector memory registers are explicitly NOT
            // affected by reset and are unpredictable, must be initialized
            // by the processor!

            Log.Debug(Category.Z80IRQ, "Am9519 interrupt controller reset");
        }

        /// <summary>
        /// Receive handles from the devices that are hardwired to our IREQ/IACK
        /// lines.  For EIO, these are the FDC, GPIB, FIFOs and DMA chip.
        /// </summary>
        public void RegisterDevice(IRQNumber source, IZ80Device dev)
        {
            _response[(int)source].Device = dev;
            Log.Debug(Category.Z80IRQ, "Registered device {0} at IRQ {1} ({2})",
                                       dev.Name, (int)source, source);
        }

        /// <summary>
        /// Check the status of the interrupt request lines and return true if a
        /// device wants attention.  If so, sets the _active IRQ to the highest
        /// priority requester.
        /// <remarks>
        /// </remarks>
        /// To make this slightly less painful, we quickly bug out if any relevant
        /// condition allows it.  If the _busy flag is set we don't scan for the
        /// next device until the current interrupt handler returns (EIO Z80 does
        /// a check for RETI explicitly).  This should cut down on extraneous polls
        /// of every device's IntLineIsActive.  At least this runs on the Z80
        /// thread so it shouldn't bog down the main CPU?  Can look for further
        /// optimizations once it's working properly...
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clock()
        {
            // If "master arm" is not set or we're in polled mode, bug out
            if (!_mode.HasFlag(ModeBits.ChipArmed) || _mode.HasFlag(ModeBits.PolledMode))
            {
                _interruptEnabled = false;
                return;
            }

            // Keep it simple, for now, and don't allow nested interrupts;
            // if the busy flag is set we don't rescan until that clears
            if (_busy)
            {
                _interruptEnabled = false;
                return;
            }

            // Not busy!  Scan the devices to see if anything requires attention
            IRQMask scan = IRQMask.None;

            if (_response[(int)IRQNumber.PERQDMA].Device.IntLineIsActive) scan |= IRQMask.EndDMAInt;
            if (_response[(int)IRQNumber.PERQtoZ80].Device.IntLineIsActive) scan |= IRQMask.PERQInt;
            if (_response[(int)IRQNumber.Z80toPERQ].Device.IntLineIsActive) scan |= IRQMask.RduProcData;
            if (_response[(int)IRQNumber.Floppy].Device.IntLineIsActive) scan |= IRQMask.FlopInt;
            if (_response[(int)IRQNumber.GPIB].Device.IntLineIsActive) scan |= IRQMask.GPIBInt;
            if (_response[(int)IRQNumber.Z80DMA].Device.IntLineIsActive) scan |= IRQMask.EOP;

            // Any change?
            if (_registers[IRR] != (byte)scan)
            {
                Log.Debug(Category.Z80IRQ, "Interrupt state change: {0}", scan);

                // Save the new state
                _registers[IRR] = (byte)scan;
            }

            // Update status (in case mask or mode bits changed)
            UpdateStatus();

            // Check status bit S7 to see if it's valid before checking priorities.
            // (This should be the only place we rely on this flag bit)
            if (_status.HasFlag(StatusBits.NoGroupInt))
            {
                // We're waiting to complete the only outstanding request (IRR
                // bit clear but ISR set... but we can't rely on that since ACR
                // might have cleared it :-) so wait for RETI to finish normally
                if (_interruptEnabled && _active >= 0)
                {
                    return;
                }

                // Otherwise punt; nothing active, no valid requests pending
                // (or something is out of sync!?)
                _interruptEnabled = false;
                _active = -1;
                return;
            }

            // Here if NOT busy, AND there's at least one unmasked IRR set (pre-
            // computed in status bits S2..S0).  If not raised, assign the active
            // IRQ and raise the interrupt; if already set but not yet acknowledged
            // we can bump the priority if a higher (_lower_ numbered!) request has
            // come in.
            _active = (int)(_status & StatusBits.PriorityMask);
            _interruptEnabled = true;

            Log.Debug(Category.Z80IRQ, "Highest active interrupt now {0}", _active);
        }

        /// <summary>
        /// Handles a request from the Z80 for an interrupt vector.  This simulates
        /// the IACK pulse that lets the Am9519 know that the requesting device with
        /// the highest interrupt priority is being serviced.
        /// </summary>
        public byte? GetActiveVector()
        {
#if DEBUG
            // If disabled or there are no outstanding requests, punt
            if (!_interruptEnabled)
            {
                Log.Info(Category.Z80IRQ, "IACK with no outstanding request?");
                return null;
            }

            // Sanity check
            if (_active < 0 || _active > 5)
            {
                Log.Info(Category.Z80IRQ, "IACK invalid request {0}?", _active);
                return null;
            }
#endif
            // Eh, why not.  Assume if the Z80 is asking for our vector our
            // (virtual) EI pin is active.  I don't think anything cares.
            _status |= StatusBits.ChipEnabled;

            // Clear the IRR bit for the active device, and set its ISR bit,
            // unless the corresponding auto clear bit is on.  Got it?
            SetBit(ISR, (byte)_active);
            ClearBit(IRR, (byte)_active);

            Log.Debug(Category.Z80IRQ, "IACK for device {0}, IRR={1:x2} ISR={2:x2}", _active,
                                       _registers[IRR], _registers[ISR]);

            // Deassert the interrupt line and set Busy (so we don't reevaluate
            // until RETI and avoid complicating things with nested interrupts)
            _interruptEnabled = false;
            _busy = true;

            // The PERQ uses individual vectors, but check the mode bit just in case
            if (_mode.HasFlag(ModeBits.CommonVector))
            {
                Log.Debug(Category.Z80IRQ, "Returning common vector 0x{0:x2}", _response[0].Vector);
                return _response[0].Vector;
            }

            // Uh, I don't think this can happen
            if (_response[_active].Device == null)
            {
                Log.Warn(Category.Z80IRQ, "Unknown vector response for device {0}", _active);
                return null;
            }

            Log.Debug(Category.Z80IRQ, "Sending device {0} vector 0x{1:x2}",
                                      _response[_active].Device.Name,
                                      _response[_active].Vector);

            // Touch the active device with our noodly appendage
            var ignore = _response[_active].Device.ValueOnDataBus;

            // Return the programmed vector
            return _response[_active].Vector;
        }

        /// <summary>
        /// Called by the processor when a RETI is processed to clear our busy
        /// flag.  In Z80dotNet 1.0.7 there are new events we can catch that do
        /// exactly this (eliminating the guesswork).
        /// </summary>
        public void Acknowledge()
        {
            // One of the SIOs might be active, but we aren't :-)
            if (!_busy) return;

            // If the active ISR and ACR bits are set, do the autoclear
            var bit = 1 << _active;
            if ((_registers[ISR] & bit) > 0 && (_registers[ACR] & bit) > 0)
            {
                ClearBit(ISR, _active);
                Log.Debug(Category.Z80IRQ, "Autoclear active IRQ {0}", _active);
            }

            _busy = false;
            _active = -1;
            Log.Debug(Category.Z80IRQ, "RETI Acknowledge");
        }


        public byte Read(byte portAddress)
        {
            // Read from the control register address returns the status register
            if (portAddress == _baseAddress + 1)
            {
                Log.Debug(Category.Z80IRQ, "Read 0x{0:x2} from status register ({1})",
                                           (byte)_status, _status);
                return (byte)_status;
            }

            // Read the register (ISR, IMR, IRR or ACR) selected by mode[6:5]
            if (portAddress == _baseAddress)
            {
                var reg = (((byte)_mode & 0x60) >> 5);

                Log.Debug(Category.Z80IRQ, "Read 0x{0:x2} from {1} register", _registers[reg], R[reg]);

                return _registers[reg];
            }

            // Shouldn't get here...
            Log.Warn(Category.Z80IRQ, "Unhandled read from 0x{0:x2}, returning 0", portAddress);
            return 0;
        }


        public void Write(byte portAddress, byte value)
        {
            if (portAddress == _baseAddress)
            {
                do
                {
                    if (_loadVector)
                    {
                        // Vector out of range, or byte count exceeded: fall through and blow up
                        if (_regSelect < 0 || _regSelect > 7) break;
                        if (_response[_regSelect].Counter >= _response[_regSelect].ByteCount) break;

                        Log.Debug(Category.Z80IRQ, "Load vector {0} byte {1} = 0x{2:x2}",
                                                    _regSelect, _response[_regSelect].Counter, value);

                        // Save the byte and bump
                        _response[_regSelect].Vector = value;   // Just the one...
                        _response[_regSelect].Counter++;

                        // More to load?
                        _loadVector = (_response[_regSelect].Counter < _response[_regSelect].ByteCount);
                        return;
                    }

                    // Shouldn't/can't happen, but check anyway
                    if (_regSelect < ISR || _regSelect > ACR) break;

                    Log.Debug(Category.Z80IRQ, "Write 0x{0:x2} to {1} register", value, R[_regSelect]);

                    _registers[_regSelect] = value;
                    return;

                } while (true);     // I am not proud of this
            }

            if (portAddress == _baseAddress + 1)
            {
                // Command bytes have multiple formats, but the high nibble is
                // unique for decoding purposes.  This is kinda goofy, AMD.
                var cmd = (CommandPrefix)(value & 0xf0);
                var all = (value & 0x08) == 0;
                var bit = (value & 0x07);

                Log.Detail(Category.Z80IRQ, "Write 0x{0:x2} to command port: {1}", value, cmd);

                // Command byte
                switch (cmd)
                {
                    case CommandPrefix.Reset:
#if DEBUG
                        if (value != 0)
                        {
                            Log.Warn(Category.Z80IRQ, "Illegal Am9519 reset command 0x{0:x2}", value);
                        }
#endif
                        Reset();
                        break;

                    case CommandPrefix.ClearIRR_IMR:
                        ClearBit(IRR, bit, all);
                        ClearBit(IMR, bit, all);
                        break;

                    case CommandPrefix.ClearIRR:
                        ClearBit(IRR, bit, all);
                        break;

                    case CommandPrefix.ClearIMR:
                        ClearBit(IMR, bit, all);
                        break;

                    case CommandPrefix.ClearISR:
                        ClearBit(ISR, bit, all);
                        break;

                    case CommandPrefix.ClearHighISR:
                        if (_registers[ISR] > 0)
                        {
                            for (byte b = 0; b < 8; b++)
                            {
                                if ((_registers[ISR] & (1 << b)) != 0)
                                {
                                    ClearBit(ISR, b);
                                    break;
                                }
                            }
                        }
                        break;

                    case CommandPrefix.SetIRR:
                        SetBit(IRR, bit, all);
                        break;

                    case CommandPrefix.SetIMR:
                        SetBit(IMR, bit, all);
                        break;

                    case CommandPrefix.LoadMode40:
                    case CommandPrefix.LoadMode41:
                        SetModeBits(0xe0, (byte)(value & 0x1f));
#if DEBUG
                        if (_mode.HasFlag(ModeBits.RotatePriority))
                            Log.Debug(Category.Z80IRQ, "Rotating priority not implemented");
#endif                  
                        Log.Debug(Category.Z80IRQ, "Loaded mode register (lo): {0}", _mode);
                        break;

                    case CommandPrefix.LoadMode65:
                        var setClear = (byte)(value & 0x03);

                        SetModeBits(0x9f, (byte)((value & 0xc0) << 3));
                        if (setClear == 1) _mode |= ModeBits.ChipArmed;
                        if (setClear == 2) _mode &= ~(ModeBits.ChipArmed);
#if DEBUG
                        if (setClear == 3) Log.Warn(Category.Z80IRQ, "Illegal value in Load Mode command");
#endif
                        Log.Debug(Category.Z80IRQ, "Loaded mode register (hi): {0}", _mode);
                        break;

                    case CommandPrefix.SelectIMR:
                        _regSelect = IMR;
                        _loadVector = false;
                        Log.Debug(Category.Z80IRQ, "Preselected IMR");
                        break;

                    case CommandPrefix.SelectAutoClr:
                        _regSelect = ACR;
                        _loadVector = false;
                        Log.Debug(Category.Z80IRQ, "Preselected ACR");
                        break;

                    case CommandPrefix.LoadByteCnt0:
                    case CommandPrefix.LoadByteCnt1:
                        var count = (byte)(((value & 0x18) >> 3) + 1);  // count
                        var regno = (byte)(value & 0x07);               // reg #
                        _response[regno].ByteCount = count;             // save count
                        _response[regno].Counter = 0;                   // reset
                        _regSelect = regno;                             // set flag
                        _loadVector = true;
                        Log.Debug(Category.Z80IRQ, "Preselected vector {0} ({1} bytes)", regno, count);
                        break;

                    default:
                        throw new InvalidOperationException($"Bad command byte 0x{value:x2}");
                }

                return;
            }

            throw new InvalidOperationException($"Bad register write 0x{portAddress:x2}");
        }

        //
        // Set or clear a bit or all bits based on the low nibble of a command byte.
        //
        void ClearBit(int reg, int bit, bool all = false)
        {
            if (all)
                _registers[reg] = 0;
            else
                _registers[reg] &= (byte)~(1 << bit);

            Log.Detail(Category.Z80IRQ, "Clear {0} reg {1} now 0x{2:x2}",
                       (all ? "all" : $"bit {bit}"), R[reg], _registers[reg]);
        }

        void SetBit(int reg, int bit, bool all = false)
        {
            if (all)
                _registers[reg] = 0xff;
            else
                _registers[reg] |= (byte)(1 << bit);

            Log.Detail(Category.Z80IRQ, "Set {0} reg {1} now 0x{2:x2}",
                       (all ? "all" : $"bit {bit}"), R[reg], _registers[reg]);
        }

        void SetModeBits(byte mask, byte val)
        {
            var tmp = (byte)_mode & mask;
            _mode = (ModeBits)(tmp | val);
        }

        /// <summary>
        /// Update the status register, including the "highest priority IRR bit"
        /// and the NoGroupInt flag.  This has the effect of doing a fixed IRQ
        /// priority sweep (rotating priority mode isn't used by the PERQ so isn't
        /// supported here).
        /// <remarks>
        /// </remarks>
        /// This is kind of expensive and slightly horrible, but is only called
        /// when the next active IRQ is needed or when the status register is read.
        /// As it happens, the standard EIO Z80 firmware doesn't appear to ever
        /// actually read it, apparently... 
        /// </summary>
        void UpdateStatus()
        {
            // Save current for logging
            var ostatus = _status;

            // Get the unmasked active interrupts
            var highest = _registers[IRR] & ~_registers[IMR];

            // Find the first set bit, starting from 0 (highest priority)
            if (highest > 0)
            {
                _status = 0;

                while ((highest & 0x1) == 0)
                {
                    _status++;
                    highest >>= 1;
                }
            }
            else
            {
                // No unmasked bits, so set bit 7 (bits 2:0 are irrelevant)
                _status = StatusBits.NoGroupInt;
            }

            // Bits 3-5 copied from the mode register, from different bit positions.  Oy vey, AMD.
            _status |= _mode.HasFlag(ModeBits.ChipArmed) ? StatusBits.ChipArmed : 0;
            _status |= _mode.HasFlag(ModeBits.PolledMode) ? StatusBits.PolledMode : 0;
            _status |= _mode.HasFlag(ModeBits.RotatePriority) ? StatusBits.RotatePriority : 0;

            // Bit 6, ChipEnabled, is the status of the chip's EI pin which, in
            // our case, isn't available (we don't simulate the actual hardware
            // interrupt chain).  For now, the _busy flag means we've IACK'ed an
            // interrupt and are waiting for RETI, so that's close enough. :-)
            _status |= _busy ? StatusBits.ChipEnabled : 0;

            if (ostatus != _status)
            {
                Log.Debug(Category.Z80IRQ, "Status change: {0} (prio {1})",
                                          _status & ~StatusBits.PriorityMask,
                                          _status & StatusBits.PriorityMask);
            }
        }

        // Debugging
        public void DumpStatus()
        {
            Console.WriteLine("Am9519 status:");
            Console.WriteLine($"  Mode reg:   {_mode}");
            Console.WriteLine($"  Status reg: {(_status & ~StatusBits.PriorityMask)} ({(_status & StatusBits.PriorityMask)})");
            Console.WriteLine($"  Interrupt:  {_interruptEnabled}  Busy: {_busy}  Active IRQ: {_active}");
            Console.WriteLine("  IRR={0:x2} IMR={1:x2} ISR={2:x2} ACR={3:x2}",
                              _registers[IRR], _registers[IMR], _registers[ISR], _registers[ACR]);
            Console.WriteLine();
            Console.WriteLine("Vectors:");

            for (var i = 0; i < 8; i++)
            {
                if (_response[i].Device != null)
                    Console.Write($"  {_response[i].Device.Name}={_response[i].Vector:x2}");
            }
            Console.WriteLine();
        }


        [Flags]
        enum StatusBits : byte
        {
            // Bits 2:0 are a vector #
            PriorityMask = 0x07,
            ChipArmed = 0x08,
            PolledMode = 0x10,
            RotatePriority = 0x20,
            ChipEnabled = 0x40,
            NoGroupInt = 0x80
        }

        [Flags]
        enum ModeBits : byte
        {
            RotatePriority = 0x01,
            CommonVector = 0x02,
            PolledMode = 0x04,
            GINTPolarityHi = 0x08,
            IREQPolarityHi = 0x10,
            ISRSelect = 0x00,
            IMRSelect = 0x20,
            IRRSelect = 0x40,
            AutoClear = 0x60,
            ChipArmed = 0x80
        }

        /// <summary>
        /// Interrupt sources for the Am9519 in priority order (high to low).
        /// Corresponds to the IREQ input pins on the chip.
        /// </summary>
        public enum IRQNumber
        {
            PERQDMA = 0,
            PERQtoZ80 = 1,
            Z80toPERQ = 2,
            Floppy = 3,
            GPIB = 4,
            Z80DMA = 5
        }

        [Flags]
        enum IRQMask : byte
        {
            None = 0x0,
            EndDMAInt = 0x01,
            PERQInt = 0x02,
            RduProcData = 0x04,
            FlopInt = 0x08,
            GPIBInt = 0x10,
            EOP = 0x20
        }

        /// <summary>
        /// Upper nibble of a byte written to the command register.
        /// </summary>
        enum CommandPrefix : byte
        {
            Reset = 0x00,
            ClearIRR_IMR = 0x10,
            ClearIMR = 0x20,
            SetIMR = 0x30,
            ClearIRR = 0x40,
            SetIRR = 0x50,
            ClearHighISR = 0x60,
            ClearISR = 0x70,
            LoadMode40 = 0x80,
            LoadMode41 = 0x90,
            LoadMode65 = 0xa0,
            SelectIMR = 0xb0,
            SelectAutoClr = 0xc0,
            LoadByteCnt0 = 0xe0,
            LoadByteCnt1 = 0xf0
        }

        /// <summary>
        /// Contains the internal byte count and vector response memory for a
        /// single channel (plus some extra data for convenient grouping).
        /// </summary>
        /// <remarks>
        /// While the chip can store and return up to four bytes of data for each
        /// channel, the PERQ's Z80 only ever uses one byte responses.  So I've
        /// simplified the code to just store and return a single byte.
        /// </remarks>
        protected struct ResponseRegister
        {
            public IZ80Device Device;       // Handle to the requesting device
            public byte ByteCount;          // As programmed
            public byte Counter;            // Counts cycles during a load/read
            public byte Vector;             // One byte for EIO Z80
        }

        //
        // Internal registers
        //
        const int ISR = 0;
        const int IMR = 1;
        const int IRR = 2;
        const int ACR = 3;
        string[] R = { "ISR", "IMR", "IRR", "ACR" };    // Debugging

        byte[] _registers;
        ModeBits _mode;
        StatusBits _status;
        ResponseRegister[] _response;

        byte _regSelect;        // Next register to load from data bus
        bool _loadVector;       // Load to response memory?

        bool _interruptEnabled;
        bool _busy;
        int _active;

        byte _baseAddress;
        byte[] _ports;
    }
}
