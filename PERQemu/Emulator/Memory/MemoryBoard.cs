//
// MemoryBoard.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Runtime.CompilerServices;

using PERQemu.IO;
using PERQemu.Config;
using PERQemu.Processor;


namespace PERQemu.Memory
{
    /// <summary>
    /// Memory cycle types (from the microinstruction SF encoding).
    /// </summary>
    public enum MemoryCycle
    {
        None = 0x0,
        Fetch4R = 0x8,
        Store4R = 0x9,
        Fetch4 = 0xa,
        Store4 = 0xb,
        Fetch2 = 0xc,
        Store2 = 0xd,
        Fetch = 0xe,
        Store = 0xf
    }

    /// <summary>
    /// Enumerate the subtle differences between board types, which only really
    /// matter when reporting parity errors.
    /// </summary>
    public enum BoardType
    {
        Unknown = 0,
        PQSLayout,
        ICLLayout,
        Old1Meg
    }

    /// <summary>
    /// Main memory array.  Provides word addressing for normal CPU and I/O
    /// 16-bit accesses, or quad words for 64-bit video fetches.
    /// </summary>
    /// <remarks>
    /// C# purists may howl, but testing under 32-bit Mono showed a 30% speed
    /// increase when shifting video data out as longs, and now that we've 
    /// pretty much been freed/forced to run on 64-bits (on later MacOS, at
    /// any rate) we might as well take advantage.  Booyah.
    /// </remarks>
    [StructLayout(LayoutKind.Explicit)]
    struct Core
    {
        [FieldOffset(0)]
        public ulong[] Quads;

        [FieldOffset(0)]
        public ushort[] Words;
    }


    /// <summary>
    /// Implements the PERQ's Memory board and memory state machine, and
    /// attaches the video controller (which is connected to the IO bus).
    /// </summary>
    /// <remarks>
    /// All references here to MDI (Memory Data IN) and MDO (Memory Data OUT) are
    /// from the CPU's point of view - the opposite of the Memory Board's (and the
    /// hardware schematics)!
    /// 
    /// Currently the IO / DMA subsystem cheats and performs its memory accesses
    /// directly, but it could at some point be integrated -- allowing us to more
    /// accurately emulate the real PERQ which must give up processor cycles for
    /// DMA. Thus, the "Hold" field of the microinstruction is basicalgnored.
    /// 
    /// The VideoController class handles most of the IO bus interaction for the
    /// MemoryBoard directly, since that's more efficient.  The only non-video-
    /// related IO registers are the parity error read/clear, which (as of 0.9.x)
    /// are only used by diagnostics; we don't actually compute and check the
    /// parity on every memory transaction, which would be nutso bananacakes.
    /// </remarks>
    public sealed class MemoryBoard : IIODevice
    {
        public MemoryBoard(PERQSystem system)
        {
            _system = system;
            _memSize = _system.Config.MemorySizeInBytes / 2;  // KB -> KW
            _memSizeMask = _memSize - 1;
            _quadSizeMask = _memSizeMask >> 2;

            _memory = new Core { Words = new ushort[_memSize] };

            // Set up the board type (for decoding parity error addresses)
            if (_system.Config.Chassis == ChassisType.PERQ1)
            {
                if (_memSize == Configurator.ONE_MEG)
                {
                    // 2MB & Portrait? ICL type; a 2MB Landscape wasn't actually
                    // a supported or shipped config, so assume a modern board
                    _boardType = (_system.Config.Display == DisplayType.Portrait ?
                                  BoardType.ICLLayout : BoardType.PQSLayout);
                }
                else
                {
                    _boardType = BoardType.Old1Meg;
                }
            }
            else
            {
                // For all other types, use the modern standard
                _boardType = BoardType.PQSLayout;
            }

            _controller = new MemoryController(this);
        }

        public void Reset()
        {
            _Tstate = 0;

            _lastFetchAddress = 0;
            _parityErrorAddress = NoError;
            _parityErrorOnNextWrite = false;

            SetInterrupt(false);

            _controller.Reset();

            Log.Info(Category.Memory, "Board reset");
        }

        public void Shutdown()
        {
            // Nothing to do
        }

        public BoardType Layout => _boardType;

        public int MemSize => _memSize;
        public int MemSizeMask => _memSizeMask;

        // Expose this for the Debugger
        public ushort[] Memory => _memory.Words;

        public int TState => _Tstate;

        public bool Wait => _controller.Wait;
        public bool MDIValid => _controller.MDI.Valid;
        public bool MDONeeded => _controller.MDO.Valid;

        public ushort MDIWord => _controller.MDI.Data;

        // For convenience, useful for RasterOp/debugging
        public int MDIIndex => _controller.MDI.Index;
        public int MDIAddress => _controller.MDI.Address;
        public int MDOAddress => _controller.MDO.Address;

        // Even if parity checking compiled out, the "CRT signals" needs this
        public int LastAddress => _lastFetchAddress;

        public bool RopEnabled => _system.CPU.RasterOp.Enabled;


        /// <summary>
        /// First half of the memory cycle: clocks the state counter, clocks the
        /// MDI and MDO queues, then sets up executes the current Fetch (if any).
        /// Asserts the Wait signal if the CPU should abort this cycle.
        /// </summary>
        /// <remarks>
        /// The hardware clocks the parity error address register every time Cpar
        /// is asserted (same cycle as Cout).  Even if parity is compiled out (it's
        /// just overhead 99.9% of the time) we have to provide the high address
        /// bits for certain older boards through the video status register so the
        /// microcode can determine the type.  The 2-4MB "PQS layout" boards use
        /// the dedicated high parity address register.  It's a small bit of extra
        /// overhead that hopefully doesn't impact performance much.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Tick(MemoryCycle cycleType)
        {
            // Bump cycle counter
            _Tstate = (_Tstate + 1) & 0x3;

            Log.Detail(Category.Memory, "T{0} Tick! {1}", _Tstate,
                       (cycleType != MemoryCycle.None ? $"Cycle={cycleType}" : ""));

            _controller.Clock(cycleType);

            if (MDIValid)
            {
#if PARITY
                // Save most recent fetched address; see remarks above
                if (_controller.MDI.Flags.Load)
                {
                    _lastFetchAddress = MDIAddress;
                }

                // Check if any word in this quad was flagged
                if (_lastFetchAddress == _parityErrorAddress)
                {
                    Log.Info(Category.Memory, "Parity error @ 0x{0:x6}!", _parityErrorAddress);
                    SetInterrupt(true);
                }
#endif
                Log.Detail(Category.Memory, "Fetch addr {0:x6} (idx {1}) --> {2:x4}",
                                             MDIAddress, MDIIndex, MDIWord);
            }
        }

        /// <summary>
        /// Second half of the memory cycle: if a store is pending, writes the
        /// output (from the ALU or current RasterOp result) to memory.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Tock(ushort data)
        {
            Log.Detail(Category.Memory, "T{0} Tock! Data={1:x4}", _Tstate, data);

            // Store to the MDO buffers
            _controller.MDO.Data = data;

#if PARITY
            if (_parityErrorOnNextWrite)
            {
                _parityErrorAddress = MDOAddress;
                Log.Info(Category.Memory, "Write bad parity @ 0x{0:x6}!", _parityErrorAddress);
            }
#endif
        }

        /// <summary>
        /// Let the CPU know if we're in the correct cycle to initiate a LoadOp
        /// function to refill the Op cache.  (Op file itself moved to the CPU.)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool LoadOpFile()
        {
            // If currently executing a Fetch4/4R, start the refill on the next
            // T2 state (i.e., next cycle).  Note: I've never seen any production
            // ucode that uses a LoadOp with Fetch4R, but OF COURSE PNX 5 does it
            return (_Tstate == 1 && (_controller.MDI.CycleType == MemoryCycle.Fetch4 ||
                                     _controller.MDI.CycleType == MemoryCycle.Fetch4R));
        }

        /// <summary>
        /// Submit a memory cycle request to the controller.                
        /// </summary>
        /// <remarks>
        /// We're in no-man's land at the bottom of the CPU cycle, but the controller
        /// will have stalled the processor until the correct time.  So this request
        /// should be recognized at the next Tick().
        /// </remarks>
        public void RequestMemoryCycle(MemoryCycle cycleType, int address)
        {
            _controller.Request(cycleType, address);
        }

        /// <summary>
        /// Fetch one quadword from memory (immediate).
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ulong FetchQuad(int address)
        {
            return _memory.Quads[address & _quadSizeMask];
        }

        /// <summary>
        /// Fetch one quadword from memory, byte swapped.  For fast video and
        /// Canon printer access to the frame buffer.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ulong FetchQuadSwapped(int address)
        {
            ulong data = _memory.Quads[address & _quadSizeMask];

            // To be useful for video access, the data has to be byte swapped.
            // Welp, there goes our salvage, guys.
            return ((data << 8) & 0xff00ff00ff00ff00) | ((data >> 8) & 0x00ff00ff00ff00ff);
        }

        /// <summary>
        /// Fetch one word from memory (immediate).  For DMA access.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public ushort FetchWord(int address)
        {
            ushort data = _memory.Words[address & _memSizeMask];

            Log.Detail(Category.Memory, "Fetch addr {0:x6} --> {1:x4}",
                                         address & _memSizeMask, data);
            return data;
        }

        /// <summary>
        /// Store one word into memory (immediate).  For DMA access.
        /// </summary>
        /// <remarks>
        /// In the Accent kernel init microcode, there is a note in the memory
        /// sizing routine that says memory boards >2MB do _not_ wrap around, but
        /// no mention of what happens to fetches/stores to addresses that are
        /// out of bounds.  After extensive experimentation, it seems the 24-bit
        /// kernel will only properly size and use the 4MB board if the usual
        /// mask/wrap behavior is restored.  Affects DMA too.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void StoreWord(int address, ushort data)
        {
            Log.Detail(Category.Memory, "Store addr {0:x6} <-- {1:x4}",
                                         address & _memSizeMask, data);

            _memory.Words[address & _memSizeMask] = data;
        }

        /// <summary>
        /// Set or clear the Parity interrupt.
        /// </summary>
        void SetInterrupt(bool raise)
        {
            raise &= _parityIntEnabled;

            if (raise && !_parityIntRaised)
            {
                _system.CPU.RaiseInterrupt(InterruptSource.Parity);
                _parityIntRaised = true;
            }
            else if (!raise && _parityIntRaised)
            {
                _system.CPU.ClearInterrupt(InterruptSource.Parity);
                _parityIntRaised = false;
            }
        }

        /// <summary>
        /// Handle the parity error address registers.
        /// </summary>
        public int IORead(byte ioPort)
        {
            int addr;

            switch (ioPort)
            {
                // Read parity error address (high bits)
                case 0x66:
                    //
                    // This register doesn't exist on the older boards; they get the
                    // high address bit(s) from the video status register!
                    if (_boardType != BoardType.PQSLayout) return 0;

                    //
                    // PQSLayout boards return address bits <21:18> as IOD<3:0>
                    //
                    addr = (_parityErrorAddress >> 18) & 0x000f;
                    Log.Debug(Category.Memory, "Read parity error address (Hi): {0:x2}", addr);
                    return addr;

                // Read parity error address (low word)
                case 0x67:
                    //
                    // All supported board layouts return address<17:2> as IOD<15:0>
                    //
                    addr = (_parityErrorAddress >> 2) & 0xffff;
                    Log.Debug(Category.Memory, "Read parity error address (Lo): {0:x4}", addr);

                    // "Clear" the address register
                    _parityErrorAddress = NoError;

                    // Also clears the interrupt!
                    SetInterrupt(false);
                    return addr;

                default:
                    throw new UnhandledIORequestException(ioPort);
            }
        }

        /// <summary>
        /// Handle the "Force bad parity" bit to signal that the next memory write
        /// should trigger a parity error interrupt.
        /// </summary>
        /// <remarks>
        /// VideoController still handles the register directly for efficiency.
        /// It calls us directly only when the bit is set.
        /// </remarks>
        public void IOWrite(byte ioPort, int value)
        {
            if (ioPort != 0xe3)
                throw new UnhandledIORequestException(ioPort, value);

            _parityErrorOnNextWrite = ((value & 0x1000) != 0);
            _parityIntEnabled = ((value & 0x0800) == 0);

            Log.Debug(Category.Memory, "Force bad parity enabled={0}, IRQ enabled={1}",
                                       _parityErrorOnNextWrite, _parityIntEnabled);

            if (!_parityIntEnabled) SetInterrupt(false);
        }

        public bool HandlesPort(byte ioPort)
        {
            // Lazy slow routine to indicate whether this device handles the given port
            for (int i = 0; i < _handledPorts.Length; i++)
            {
                if (ioPort == _handledPorts[i]) { return true; }
            }

            return false;
        }

        //[Conditional("DEBUG")]
        public void DumpMemState()
        {
            _controller.DumpState();

#if PARITY
            Console.WriteLine("Parity check:");
            Console.WriteLine("\tType={0} Force next={1} IRQ enabled={2} raised={3}",
                              _boardType, _parityErrorOnNextWrite, _parityIntEnabled, _parityIntRaised);
            Console.WriteLine("\tLast Fetch addr=0x{0:x6} Error addr=0x{1:x6}",
                              _lastFetchAddress, _parityErrorAddress);
#endif
        }


        const int NoError = 0x01000000;         // Reads back as 0 if no parity error

        Core _memory;

        int _memSize;
        int _memSizeMask;
        int _quadSizeMask;

        int _Tstate;

        int _lastFetchAddress;
        int _parityErrorAddress;

        bool _parityErrorOnNextWrite;
        bool _parityIntEnabled;
        bool _parityIntRaised;

        byte[] _handledPorts = { 0x66, 0x67 };  // Parity error regs

        MemoryController _controller;

        BoardType _boardType;
        PERQSystem _system;
    }
}
