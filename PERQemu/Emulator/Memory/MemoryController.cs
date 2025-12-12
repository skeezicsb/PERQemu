//
// MemoryController.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.IO;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace PERQemu.Memory
{
    /// <summary>
    /// Represents a request to the memory subsystem.
    /// </summary>
    public class MemoryRequest
    {
        public MemoryRequest()
        {
            Clear();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            StartAddress = -1;
            CycleType = MemoryCycle.None;
            Bookmark = 0;
            Active = false;
        }

        public override string ToString()
        {
            return string.Format("Addr={0:x6} cycle={1} bookmark={2:x} active={3}",
                                StartAddress, CycleType, Bookmark, Active);
        }

        public int StartAddress;
        public MemoryCycle CycleType;
        public int Bookmark;
        public bool Active;
    }


    public enum MemoryState
    {
        Idle = 0,
        WaitForT3,
        WaitForT2,
        Running
    }


    /// <summary>
    /// A bookmark comprises a set of flags, a word index, and next state value.
    /// These are read from the BKM16 ROM and are consulted each cycle to drive
    /// the memory state machine.
    /// </summary>
    public class BookmarkEntry
    {
        public BookmarkEntry(byte result)
        {
            Abort = ((result & 0x80) != 0);
            Recognize = ((result & 0x40) != 0);
            Complete = ((result & 0x20) != 0);
            Valid = ((result & 0x10) != 0);
            Index = (result & 0x0c) >> 2;
            NextState = (MemoryState)(result & 0x03);
        }

        public override string ToString()
        {
            return string.Format("PA={0} ST={1} CO={2} VA={3} idx={4} next={5}",
                                Abort, Recognize, Complete, Valid, Index, NextState);
        }

        public bool Abort;
        public bool Recognize;
        public bool Complete;
        public bool Valid;
        public int Index;
        public MemoryState NextState;
    }


    /// <summary>
    /// Object to keep track of the Memory board's input and output queues.
    /// Instantiating one each for Stores and Fetches dramatically simplifies
    /// the overlapping quad-word read/write cycles used by RasterOp.  This
    /// class handles all of the arcane timing requirements, generates CPU Wait
    /// (and IO Hold eventually?) states when needed, and calculates the address
    /// for the appropriate word in the quad as each request executes.
    /// </summary>
    public sealed class MemoryController
    {
        static MemoryController()
        {
            _bkmTable = new BookmarkEntry[256];
            LoadBookmarkROM();
        }

        public MemoryController(MemoryBoard mem, string name)
        {
            _mem = mem;
            _name = name;
            _current = new MemoryRequest();
            _pending = new MemoryRequest();

            _quadWordMask = _mem.MemSizeMask & 0xfffffc;     // nip two LSBs
            _doubleWordMask = _mem.MemSizeMask & 0xfffffe;   // nip LSB
        }

        public void Reset()
        {
            _state = _nextState = MemoryState.Idle;
            _bookmark = _nextBookmark = 0;
            _address = -1;
            _index = 0;
            _wait = false;
            _valid = false;
            _current.Clear();
            _pending.Clear();

            Log.Debug(Category.MemCycle, "{0} queue reset", _name);
        }

        public bool Wait => _wait;
        public bool Valid => _valid;
        public int Address => _address;
        public int WordIndex => _index;
        public MemoryCycle Cycle => _current.CycleType;


        /// <summary>
        /// Clocks this memory queue's state machine, setting flags appropriately for
        /// the current running request, or setting up for the next one.  Called from
        /// Memory.Tick(), this executes at the top of the microcycle, so it may abort 
        /// the current instruction if a new request is issued at the wrong time.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clock(MemoryCycle nextCycle)
        {
#if DEBUG
            if (nextCycle != MemoryCycle.None || _current.CycleType != MemoryCycle.None)
                Log.Detail(Category.MemCycle,
                           "{0} queue  IN: Clock T{1} cycle={2} bkm={3:x} next={4} state={5} next={6}",
                           _name, _mem.TState, _current.CycleType, _bookmark, nextCycle, _state, _nextState);
#endif

            // Update the current op
            Recognize();

            // Update state and set flags for this cycle
            RunStateMachine();

            // Update bookmarks for the next cycle
            UpdateBookmarks(nextCycle);

#if DEBUG
            if (nextCycle != MemoryCycle.None || _current.CycleType != MemoryCycle.None)
                Log.Detail(Category.MemCycle,
                           "{0} queue OUT: Clock T{1} cycle={2} bkm={3:x} next={4} state={5} next={6}",
                           _name, _mem.TState, _current.CycleType, _bookmark, nextCycle, _state, _nextState);
#endif
        }

        /// <summary>
        /// Accept a new memory request (at the bottom of the CPU cycle, after
        /// R is computed).  Because the CPU now aborts until the correct cycle
        /// when issuing new memory operations, we simply latch the new request
        /// and let the state machine do all the right magic at the next Clock().
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Request(int startAddr, MemoryCycle cycleType)
        {
            // fixme: whoops, we aren't actually checking the bookmark Start bit!
            // if pending is *already* active, we've screwed up!?
            if (_pending.Active)
                Log.Write(Category.MemCycle, "Request {0} while {1} already pending!?",
                                              cycleType, _pending.CycleType);

            _pending.StartAddress = startAddr;
            _pending.CycleType = cycleType;

            _pending.Active = true;
            _pending.Bookmark = _nextBookmark;
        }

        /// <summary>
        /// If the current op is complete and a pending op is ready, promote it.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Recognize()
        {
            if (_pending.Active && !_current.Active)
            {
                // Copy in the relevant bits
                _current.CycleType = _pending.CycleType;
                _current.StartAddress = _pending.StartAddress;
                _current.Bookmark = _pending.Bookmark;
                _current.Active = true;

                // Clear the request
                _pending.Clear();

                // Set the new bookmark
                _bookmark = _current.Bookmark;

                Log.Debug(Category.MemCycle, "{0} queue: Recognized {1}", _name, _current);
            }
        }

        /// <summary>
        /// Update the current state of the controller, and computes the address and
        /// index of the current op if applicable.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void RunStateMachine()
        {
            // Bump the state
            _state = _nextState;

            // Get the flags for the current bookmark
            var flags = GetBookmarkEntry(_bookmark, _state);

            // Set the wait and next state based on the current flags
            _wait = flags.Abort;
            _nextState = flags.NextState;

            // If reading or writing this cycle, compute the address for this word
            _valid = flags.Valid;

            if (_valid)
            {
                _index = flags.Index;

                switch (_current.CycleType)
                {
                    case MemoryCycle.Fetch4R:
                    case MemoryCycle.Fetch4:
                    case MemoryCycle.Store4R:
                    case MemoryCycle.Store4:
                        _address = (_current.StartAddress & _quadWordMask) + _index;
                        break;

                    case MemoryCycle.Fetch2:
                    case MemoryCycle.Store2:
                        _address = (_current.StartAddress & _doubleWordMask) + _index;
                        break;

                    default:
                        _address = _current.StartAddress;
                        break;
                }
            }

            // If this is the last word in a cycle, retire the current op
            if (flags.Complete)
            {
                Log.Debug(Category.MemCycle, "{0} queue: Retired {1}", _name, _current);

                _current.Clear();
                _bookmark = 0;
            }
        }

        /// <summary>
        /// Sets bookmarks for the next cycle, and modifies the current one if necessary.
        /// WARNING: THIS IS WHERE THE SAUSAGE IS MADE.
        /// </summary>
        /// <remarks>
        /// I used to think this was crazy and bad, a terribly improvised series of
        /// hacks and assumptions to work around the complexity of the hardware.  Then
        /// sources were found to the PALs and PROMs that make up the MST01/MST10 and
        /// GMV02/BKM16.2 memory state machines and... well, it's eerie how I managed
        /// to come closer with these wild-ass-guesses to the way the hardware actually
        /// operates than I ever imagined.  It still cries out for refactoring, though.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void UpdateBookmarks(MemoryCycle nextCycle)
        {
            if (nextCycle == MemoryCycle.None)
            {
                // If no active or pending op, reset our bookmark
                if (!_current.Active && !_pending.Active)
                {
                    _bookmark = 0;
                }
                return;
            }

            // This microinstruction specifies a new memory request: initialize
            // the next bookmark value based on the request type
            _nextBookmark = (int)nextCycle;
            var book = _nextBookmark;

            // 
            // Special cases for RasterOp
            //
            if (_mem.RopEnabled)
            {
                if (_mem.TState == 0)
                {
                    // First: we're allowed to issue Store4/4R in T0, ahead
                    // of the usual T3.  So we tweak the cycle type to index
                    // the bookmark ROM with the modified timings.
                    if (nextCycle == MemoryCycle.Store4R)
                    {
                        book = 0x2;         // "RopStore4R"
                    }
                    else if (nextCycle == MemoryCycle.Store4)
                    {
                        book = 0x4;         // "RopStore4"
                    }
                }
                else if (_mem.TState == 3)
                {
                    //
                    // Second: Fetch4/4Rs are issued back-to-back (in the
                    // correct t3) but must NOT introduce the possible CPU
                    // abort of a WaitT2 state;  MDI must remain valid AND
                    // the index values must count down correctly for the
                    // operation in progress, so after the t0,t1 complete
                    // the next op's four words arrive in the four subsequent
                    // Tstates.  This introduces two additional fake cycle
                    // types, as with the case above.
                    //
                    if (_current.CycleType == MemoryCycle.Fetch4R &&
                                 nextCycle == MemoryCycle.Fetch4R)
                    {
                        _bookmark = book = 0x1;     // "RopFetch4R"
                    }
                    else if (_current.CycleType == MemoryCycle.Fetch4 &&
                                      nextCycle == MemoryCycle.Fetch4)
                    {
                        _bookmark = book = 0x3;     // "RopFetch4"
                    }
                }

                // For RasterOp special cases, use modified bookmark for entire cycle
                _nextBookmark = book;
            }

            //
            // Special cases for indirect or overlapped Fetches (non-RasterOp)
            //
            if (_mem.IsFetch(nextCycle) && _current.Active)
            {
                //
                // Back-to-back Fetch type operations requests present unique
                // timing challenges.  To accommodate this with as little
                // embarrassment as possible, we use a transitional bookmark
                // to cover the overlap.  Gory details in Docs/MemoryRules.txt.
                //

                // Never found a case where Fetch4Rs overlap, actually
                if (_current.CycleType == MemoryCycle.Fetch || _current.CycleType == MemoryCycle.Fetch2 ||
                   ((_current.CycleType == MemoryCycle.Fetch4 || _current.CycleType == MemoryCycle.Fetch4R) && !_mem.RopEnabled))
                {
                    book = (int)_current.CycleType / 2;     // Compute indirect fetch to cover overlap

                    Log.Debug(Category.MemCycle, "Overlap in T{0} @ PC 0x{1:x}: active {2} pending {3} book {4:x} new book {5:x} next book {6:x}",
                                                 _mem.TState, PERQemu.Sys.CPU.PC, _current.CycleType, nextCycle, _bookmark, book, _nextBookmark);

                    _bookmark = book;                       // Force immediate switch for the (t2,t3)
                }
            }

            // Get a new set of flags -- these may modify the current cycle!
            var flags = GetBookmarkEntry(book, _nextState);

            // Set the wait and next state based on the new flags
            _wait = flags.Abort;
            _nextState = flags.NextState;

            // If the done flag is set, retire the current op
            if (flags.Complete)
            {
                Log.Debug(Category.MemCycle, "{0} queue: Terminated {1}", _name, _current);
                _current.Clear();
            }
        }

        /// <summary>
        /// Gets the bookmark for a particular cycle type.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        BookmarkEntry GetBookmarkEntry(int book, MemoryState state)
        {
            //
            // Index into the "bookmark" table:
            //		bits	value
            //		7:4		bookmark (cycle type)
            //		3:2		current state
            //		1:0		current Tstate
            //
            int lookup = (book & 0x0f) << 4 | ((int)state << 2) | _mem.TState;

            Log.Detail(Category.MemCycle, "{0} Bookmark[{1:x3}]: {2}", _name, lookup, _bkmTable[lookup]);

            return _bkmTable[lookup];
        }

        /// <summary>
        /// Load the BKM16 ROM image from disk.
        /// </summary>
        static void LoadBookmarkROM()
        {
            // BKM is a lookup table with an 8-bit index, returning an 8-bit value
            using (var fs = new FileStream(Paths.BuildPROMPath("bkm16emu.rom"), FileMode.Open))
            {
                for (int i = 0; i < 256; i++)
                {
                    // Split result byte into fields once, rather than on lookup
                    _bkmTable[i] = new BookmarkEntry((byte)fs.ReadByte());
                }
                fs.Close();
            }
            Log.Info(Category.Emulator, "Initialized BKM ROM lookup table");
        }

        /// <summary>
        /// Dumps the current controller state and request slots. Quick and dirty debugging aid.
        /// </summary>
        //[Conditional("DEBUG")]
        public void DumpQueue()
        {
            Console.WriteLine("{0} queue:\tstate: wait={1} valid={2} index={3} addr={4:x6}",
                              _name, _wait, _valid, _index, _address);
            Console.WriteLine("\t\tcurrent: {0}", _current);
            Console.WriteLine("\t\tpending: {0}", _pending);
        }


        MemoryBoard _mem;

        string _name;
        MemoryState _state;
        MemoryState _nextState;

        MemoryRequest _current;
        MemoryRequest _pending;

        int _quadWordMask;
        int _doubleWordMask;

        int _address;
        int _index;
        bool _wait;
        bool _valid;

        int _bookmark;
        int _nextBookmark;

        static BookmarkEntry[] _bkmTable;
    }
}
