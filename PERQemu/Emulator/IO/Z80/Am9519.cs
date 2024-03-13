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
                _response[i].Vector = new byte[4];
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

            _preselect = 0;
            _loadVector = false;

            // The Byte Count and Vector memory registers are explicitly NOT
            // affected by reset and are unpredictable, must be initialized
            // by the processor!

            Log.Info(Category.Z80IRQ, "Am9519 interrupt controller reset");
        }

        /// <summary>
        /// Receive handles from the devices that are hardwired to our IREQ/IACK
        /// lines.  For EIO, these are the FDC, GPIB, FIFOs and DMA chip.
        /// </summary>
        public void RegisterDevice(IRQNumber source, IZ80Device dev)
        {
            _response[(int)source].Device = dev;
            Log.Info(Category.Z80IRQ, "Registered device {0} at IRQ {1} ({2})",
                                       dev.Name, (int)source, source);
        }

        /// <summary>
        /// Check the status of the interrupt request lines.  This is going to be
        /// painfully slow, polling them every cycle (which is what the Z80dotNet
        /// CPU does for its list of sources?) but let's just see if it works,
        /// before worrying about making it go fast.  At least this runs on the Z80
        /// thread so it shouldn't bog down the main CPU?
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clock()
        {
            // Scan the devices to see if anything requires attention.  This is
            // magnificently ugly.  Profoundly grotesque.  Sublimely absurd.
            IRQMask scan = 0;

            if (_response[(int)IRQNumber.PERQDMA].Device?.IntLineIsActive ?? false) scan |= IRQMask.EndDMAInt;
            if (_response[(int)IRQNumber.PERQInput].Device?.IntLineIsActive ?? false) scan |= IRQMask.PERQInt;
            if (_response[(int)IRQNumber.PERQOutput].Device?.IntLineIsActive ?? false) scan |= IRQMask.RduProcData;
            if (_response[(int)IRQNumber.Floppy].Device?.IntLineIsActive ?? false) scan |= IRQMask.FlopInt;
            if (_response[(int)IRQNumber.GPIB].Device?.IntLineIsActive ?? false) scan |= IRQMask.GPIBInt;
            if (_response[(int)IRQNumber.Z80DMA].Device?.IntLineIsActive ?? false) scan |= IRQMask.EOP;

            if ((byte)scan != _registers[IRR])
            {
                Log.Info(Category.Z80IRQ, "Interrupt state change: {0}", scan);

                // Save the new state
                _registers[IRR] = (byte)scan;

                // Update status so we can check flags
                UpdateStatus();
            }

            // If "master arm" is not set or we're in polled mode, bug out
            if (!_mode.HasFlag(ModeRegister.ChipArmed) || !_mode.HasFlag(ModeRegister.PolledMode))
            {
                _interruptEnabled = false;
                return;
            }

            // Status register will contain the highest priority unmasked IRQ;
            // check bit S7 to see if it's valid before checking priorities
            if (_status.HasFlag(StatusRegister.NoGroupInt))
            {
                if (_interruptEnabled)
                    Log.Info(Category.Z80IRQ, "No (further?) interrupts requested, disabling");

                _interruptEnabled = false;
                return;
            }

            if (_mode.HasFlag(ModeRegister.RotatingPriority))
            {
                Log.Info(Category.Z80IRQ, "Rotating priority not implemented");
            }

            // Set the active IRQ and raise the interrupt line
            _active = (int)(_status & StatusRegister.PriorityMask);
            _interruptEnabled = true;

            Log.Info(Category.Z80IRQ, "Highest active interrupt now {0}", _active);
        }

        /// <summary>
        /// Handles a request from the Z80 for an interrupt vector.  This simulates
        /// the IACK pulse that lets the Am9519 know that the requesting device with
        /// the highest interrupt priority is being serviced.
        /// </summary>
        public byte? GetActiveVector()
        {
            // If disabled or there are no outstanding requests, punt
            if (!_interruptEnabled)
            {
                Log.Info(Category.Z80IRQ, "IACK with no outstanding request?");
                return null;
            }

            // Clear the IRR bit for the active device, and set its ISR bit,
            // unless the corresponding auto clear bit is on.  Got it?
            ClearBit(IRR, (byte)_active);

            if ((_registers[ACR] & (1 << _active)) != 0) SetBit(ISR, (byte)_active);

            // The PERQ uses individual vectors, but check the mode bit just in case
            if (_mode.HasFlag(ModeRegister.CommonVector))
            {
                Log.Info(Category.Z80IRQ, "Returning common vector 0x{0:x2}", _response[0].Vector[0]);
                return _response[0].Vector[0];
            }

            // Touch the active device with our noodly appendage
            if (_response[_active].Device != null)
            {
                Log.Info(Category.Z80IRQ, "IACK for device {0}, vector 0x{1:x2}",
                                          _response[_active].Device.Name,
                                          _response[_active].Vector[0]);

                var ignore = _response[_active].Device.ValueOnDataBus;

                // IN THEORY we can return up to four bytes of data, and we'd use
                // our ByteCount and Counter fields to do that.  In practice, I'm
                // just gonna cheat and rely on the knowledge that the PERQ/Z80
                // only ever uses one byte responses.  Yes, this is bad and lazy.

                // Return the programmed vector
                return _response[_active].Vector[0];
            }

            // Whoops.  If we fell through, something went terribly wrong
            Log.Warn(Category.Z80IRQ, "Unknown vector response for device {0}", _active);
            return null;
        }


        public byte Read(byte portAddress)
        {
            // Read from the control register address returns the status register
            if (portAddress == _baseAddress + 1)
            {
                UpdateStatus();
                Log.Debug(Category.Z80IRQ, "Read 0x{0:x2} from status register ({1})",
                                           (byte)_status, _status);
                return (byte)_status;
            }

            // Read the preselected register (ISR, IMR, IRR or ACR) which should
            // be reflected by the mode bits 5..6
            if (portAddress == _baseAddress)
            {
                if (_preselect >= ISR && _preselect <= ACR)
                {
                    Log.Debug(Category.Z80IRQ, "Read 0x{0:x2} from {1} register",
                                              _registers[_preselect],
                                              (_preselect > 2) ? "ACR" :
                                              (_preselect > 1) ? "IRR" :
                                              (_preselect > 0) ? "IMR" : "ISR");

                    return _registers[_preselect];
                }

                Log.Warn(Category.Z80IRQ, "Bad preselect value {0}, returning 0!", _preselect);
                return 0;
            }

            // Shouldn't get here...
            Log.Debug(Category.Z80IRQ, "Read from 0x{0:x2}, returning 0 (unimplemented)", portAddress);
            return 0;
        }


        public void Write(byte portAddress, byte value)
        {
            Log.Detail(Category.Z80IRQ, "Write 0x{0:x2} to port 0x{1:x2}", value, portAddress);

            if (portAddress == _baseAddress)
            {
                do
                {
                    if (_loadVector)
                    {
                        // Vector out of range, or byte count exceeded: fall through and blow up
                        if (_preselect < 0 || _preselect > 7) break;
                        if (_response[_preselect].Counter >= _response[_preselect].ByteCount) break;

                        Log.Debug(Category.Z80IRQ, "Load vector {0} byte {1} = 0x{2:x2}",
                                                    _preselect, _response[_preselect].Counter, value);

                        // Save the byte and bump
                        _response[_preselect].Vector[_response[_preselect].Counter++] = value;

                        // Last one?
                        if (_response[_preselect].Counter == _response[_preselect].ByteCount) _loadVector = false;
                        return;
                    }

                    // Shouldn't/can't happen, but check anyway
                    if (_preselect < ISR || _preselect > ACR) break;

                    Log.Debug(Category.Z80IRQ, "Write 0x{0:x2} to {1} register", value,
                                              (_preselect > 2) ? "ACR" :
                                              (_preselect > 1) ? "IRR" :
                                              (_preselect > 0) ? "IMR" : "ISR");

                    _registers[_preselect] = value;
                    return;

                } while (true);     // I am not proud of this
            }

            if (portAddress == _baseAddress + 1)
            {
                // Command bytes have multiple formats, but the high nibble is
                // unique for decoding purposes.  This is kinda goofy, AMD.
                var cmd = (CommandPrefix)(value & 0xf0);

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
                        ClearBit(IRR, value);
                        ClearBit(IMR, value);
                        break;

                    case CommandPrefix.ClearIRR:
                        ClearBit(IRR, value);
                        break;

                    case CommandPrefix.ClearIMR:
                        ClearBit(IMR, value);
                        break;

                    case CommandPrefix.ClearISR:
                        ClearBit(ISR, value);
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
                        SetBit(IRR, value);
                        break;

                    case CommandPrefix.SetIMR:
                        SetBit(IMR, value);
                        break;

                    case CommandPrefix.LoadMode40:
                    case CommandPrefix.LoadMode41:
                        SetBits(IMR, 0xe0, (byte)(value & 0x1f));
                        break;

                    case CommandPrefix.LoadMode65:
                        var setClear = (byte)(value & 0x03);

                        SetBits(IMR, 0x9f, (byte)((value & 0x0c) << 2));
                        if (setClear == 1) SetBit(IMR, 7);
                        if (setClear == 2) ClearBit(IMR, 7);
#if DEBUG
                        if (setClear == 3) Log.Warn(Category.Z80IRQ, "Illegal value in Load Mode command");
#endif
                        break;

                    case CommandPrefix.SelectIMR:
                        SetBits(IMR, 0x9f, (byte)ModeRegister.IMRSelect);   // DOES this set bits 6:5?
                        _preselect = IMR;
                        _loadVector = false;
                        Log.Debug(Category.Z80IRQ, "Preselected IMR");
                        break;

                    case CommandPrefix.SelectAutoClr:
                        SetBits(IMR, 0x9f, (byte)ModeRegister.AutoClear);   // check the docs carefully
                        _preselect = ACR;
                        _loadVector = false;
                        Log.Debug(Category.Z80IRQ, "Preselected ACR");
                        break;

                    case CommandPrefix.LoadByteCnt0:
                    case CommandPrefix.LoadByteCnt1:
                        var count = (byte)(((value & 0x18) >> 3) + 1);  // count
                        var regno = (byte)(value & 0x07);               // reg #
                        _response[regno].ByteCount = count;             // save count
                        _response[regno].Counter = 0;                   // reset
                        _preselect = regno;                             // set flag
                        _loadVector = true;
                        Log.Debug(Category.Z80IRQ, "Preselected vector {0} ({1} bytes)", regno, count);
                        break;

                    default:
                        throw new InvalidOperationException($"Bad command byte 0x{value:x2}");
                }

                UpdateStatus(); // debug?  Clock() and Read() will call us, right?
                return;
            }

            throw new InvalidOperationException($"Bad register write 0x{portAddress:x2}");
        }

        //
        // Set or clear a bit or all bits based on the low nibble of a command byte.
        //
        void ClearBit(int reg, byte cmd)
        {
            var bit = (cmd & 0x7);

            if ((cmd & 0x08) == 0)
                _registers[reg] = 0;
            else
                _registers[reg] &= (byte)~(1 << bit);
        }

        void SetBit(int reg, byte cmd)
        {
            var bit = (cmd & 0x07);

            if ((cmd & 0x08) == 0)
                _registers[reg] = 0xff;
            else
                _registers[reg] |= (byte)(1 << bit);
        }

        void SetBits(int reg, byte mask, byte val)
        {
            _registers[reg] = (byte)((_registers[reg] & mask) | val);
        }

        /// <summary>
        /// Update the status register.  Kind of expensive, but only needs updating
        /// when read, not on every signal change?
        /// </summary>
        void UpdateStatus()
        {
            // Save current for logging
            var ostatus = _status;

            // Initialize with the highest unmasked bit that's set (regardless
            // of our polling mode or current status?).
            var highest = (byte)(_registers[IRR] & _registers[IMR]);

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
                _status = StatusRegister.NoGroupInt;
            }

            // Bits 3-5 copied from the mode register, from different bit positions.  Oy vey, AMD.
            _status |= _mode.HasFlag(ModeRegister.ChipArmed) ? StatusRegister.ChipArmed : 0;
            _status |= _mode.HasFlag(ModeRegister.PolledMode) ? StatusRegister.PolledMode : 0;
            _status |= _mode.HasFlag(ModeRegister.RotatingPriority) ? StatusRegister.RotatingPriority : 0;

            // Bit 6, ChipEnabled, is that actual status of the hardware EI pin
            // which, in our case, isn't available; we'd have to chain together
            // the actual interrupt chain to determine which chip is active (and
            // the Z80dotNet CPU's interrupt code isn't that precise).  For now,
            // let's just assume that if we have any interrupts pending that we
            // are enabled.  This is not ideal.
            _status |= StatusRegister.ChipEnabled;      // "if (active)" or something

            if (ostatus != _status)
            {
                Log.Info(Category.Z80IRQ, "Status change: {0} (prio {1})",
                                          _status & ~StatusRegister.PriorityMask,
                                          _status & StatusRegister.PriorityMask);
            }
        }

        // Debugging
        public void DumpStatus()
        {
            Console.WriteLine("Am9519 status:");
            Console.WriteLine($"  Mode reg:   {_mode}");
            Console.WriteLine($"  Status reg: {_status}");
            Console.WriteLine($"  Interrupt:  {_interruptEnabled}  Active: {_active}");
            Console.WriteLine("  IRR={0:x2} IMR={1:x2} ISR={2:x2} ACR={3:x2}",
                              _registers[IRR], _registers[IMR], _registers[ISR], _registers[ACR]);
            Console.WriteLine();
            Console.WriteLine("Vectors:");

            for (var i = 0; i < 8; i++)
            {
                if (_response[i].Device != null)
                    Console.Write($"  {_response[i].Device.Name}={_response[i].Vector[0]:x2}");
            }
            Console.WriteLine();
        }

        [Flags]
        enum StatusRegister : byte
        {
            // Bits 2:0 are a vector #
            PriorityMask = 0x07,
            ChipArmed = 0x08,
            PolledMode = 0x10,
            RotatingPriority = 0x20,
            ChipEnabled = 0x40,
            NoGroupInt = 0x80
        }

        [Flags]
        enum ModeRegister : byte
        {
            RotatingPriority = 0x01,
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
            PERQInput = 1,
            PERQOutput = 2,
            Floppy = 3,
            GPIB = 4,
            Z80DMA = 5
        }

        [Flags]
        enum IRQMask : byte
        {
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
        protected struct ResponseRegister
        {
            public IZ80Device Device;       // Handle to the requesting device
            public byte ByteCount;          // As programmed
            public byte Counter;            // Counts cycles during a load/read
            public byte[] Vector;           // Holds up to 4 response bytes
        }

        //
        // Internal registers
        //
        const int ISR = 0;
        const int IMR = 1;
        const int IRR = 2;
        const int ACR = 3;

        byte[] _registers;
        ModeRegister _mode;
        StatusRegister _status;
        ResponseRegister[] _response;

        byte _preselect;    // Next register to load from data bus
        bool _loadVector;   // Load to response memory?

        bool _interruptEnabled;
        int _active;

        byte _baseAddress;
        byte[] _ports;
    }
}

/*
    Notes:
    
    ;
    ; Interrupt chip definitions
    ;
    INTCSR      equ 141Q    ; Interrupt controller control/status register
    I.PDMA      equ 00H     ; DMA to/from PERQ done interrupt
    I.PERQI     equ 01H     ; Data available in From PERQ FIFO
    I.PERQO     equ 02H     ; Data just read from To PERQ FIFO
    I.Floppy    equ 03H     ; Floppy interrupt
    I.GPIB      equ 04H     ; GPIB interrupt
    I.ZDMA      equ 05H     ; End of Process by DMA controller

    I.Single    equ 08H     ; Select single level
    I.CIMRIRR   equ 010H    ; Clear IMR and IRR
    I.CIMR      equ 020H    ; Clear IMR
    I.SIMR      equ 030H    ; Set IMR
    I.CIRR      equ 040H    ; Clear IRR
    I.SIRR      equ 050H    ; Set IRR
    I.CHISR     equ 060H    ; Clear highest ISR bit
    I.CISR      equ 070H    ; Clear ISR
    I.M01234    equ 080H    ; Load mode bits 0..4
    I.Fixed     equ 000H    ; Set for fixed priority
    I.Rotate    equ 001H    ; Set for rotating priority
    I.Indiv     equ 000H    ; Individual vectors
    I.Common    equ 002H    ; Common vector for group
    I.Interrupt equ 000H    ; Operate in interrupt move
    I.Polled    equ 004H    ; Operate in polled mode
    I.GrpLow    equ 000H    ; Group interrupt is active low
    I.GrpHigh   equ 008H    ; Group interrupt is active low
    I.ReqLow    equ 000H    ; Interrupt request is active low
    I.ReqHigh   equ 010H    ; Interrupt request is active high
    I.M567      equ 0A0H    ; Load Mode bits 5,6,7
    I.Arm       equ 001H    ; Arm interrupt chip
    I.DisArm    equ 002H    ; Disarm interrupt chip
    I.RdISR     equ 000H    ; Preselect to read ISR
    I.RdIMR     equ 004H    ; Preselect to read IMR
    I.RdIRR     equ 008H    ; Preselect to read IRR
    I.RdACR     equ 00CH    ; Preselect to read ACR
    I.WrIMR     equ 0B0H    ; Select IMR
    I.WrACR     equ 0C0H    ; Select ACR
    I.WrRES     equ 0E0H    ; Select response memory
    I.BY1       equ 00H     ; readback 1 byte
    I.BY2       equ 08H     ; readback 2 bytes
    I.BY3       equ 10H     ; readback 3 bytes
    I.BY4       equ 18H     ; readback 4 bytes

    INTDATA     equ 140Q    ; Interrupt controller data

startup sequence:
Z80IRQ: Write 0x00 to port 0x61 (unimplemented)     reset
Z80IRQ: Write 0x90 to port 0x61 (unimplemented)     load mode 4-0 with 00000
Z80IRQ: Write 0xc0 to port 0x61 (unimplemented)     select autoclear
Z80IRQ: Write 0xff to port 0x60 (unimplemented)     write all 1s to autoclear reg
Z80IRQ: Write 0xe1 to port 0x61 (unimplemented)     select byte count (1) reg 1
Z80IRQ: Write 0x14 to port 0x60 (unimplemented)     write vector byte 0x14 (perq input vec!)
Z80IRQ: Write 0x19 to port 0x61 (unimplemented)     clear irr, imr bit 1 (perq int)
Z80IRQ: Write 0xe2 to port 0x61 (unimplemented)     select byte count (1) reg 2
Z80IRQ: Write 0x12 to port 0x60 (unimplemented)     write vector byte 0x12 (perq output vec)
Z80IRQ: Write 0x1a to port 0x61 (unimplemented)     clear irr, imr bit 2 (rd uproc data)

Z80DMA: Write 0x00 to port 0x3d (unimplemented)     dmac initialization?
Z80DMA: Read from 0x38, returning 0 (unimplemented)
Z80DMA: Write 0x60 to port 0x38 (unimplemented)

Z80IRQ: Write 0xe5 to port 0x61 (unimplemented)     select byte count (1) reg 5
Z80IRQ: Write 0x1a to port 0x60 (unimplemented)     write vector byte 0x1a (z80 dma eop)
Z80IRQ: Write 0xe0 to port 0x61 (unimplemented)     select byte count (1) reg 0
Z80IRQ: Write 0x10 to port 0x60 (unimplemented)     write vector byte 0x10 (perq dma)
Z80IRQ: Write 0x10 to port 0x61 (unimplemented)     clear irr, imr bit 0 (end dma int)
Z80IRQ: Write 0xe3 to port 0x61 (unimplemented)     select byte count (1) reg 3
Z80IRQ: Write 0x16 to port 0x60 (unimplemented)     write vector byte 0x16 (floppy)
Z80IRQ: Write 0xa1 to port 0x61 (unimplemented)     load mode 6-5 with 00, set 7 (master arm)

[repeats first block - chip gets reset]

*/
