//
// Bookmarks.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace PERQemu.Memory
{
    public enum BookmarkType
    {
        None = 0x0,
        Unused = 0x1,

        // Modified timings overlapped RasterOp fetch/store cycles
        RopStore4R = 0x2,
        RopStore4 = 0x3,
        RopFetch4R = 0x4,
        RopFetch4 = 0x5,

        // Overlapped single- and double-word fetches
        IndFetch2 = 0x6,
        IndFetch = 0x7,

        // Real ones used by the hardware
        Fetch4R = MemoryCycle.Fetch4R,
        Store4R = MemoryCycle.Store4R,
        Fetch4 = MemoryCycle.Fetch4,
        Store4 = MemoryCycle.Store4,
        Fetch2 = MemoryCycle.Fetch2,
        Store2 = MemoryCycle.Store2,
        Fetch = MemoryCycle.Fetch,
        Store = MemoryCycle.Store
    }

    /// <summary>
    /// A bookmark comprises a set of flags, a word index, and next state value.
    /// These are consulted each microcycle to drive the memory state machine.
    /// </summary>
    public class BookmarkEntry
    {
        public bool Load;
        public bool Complete;
        public bool Valid;
        public int Index;
        public RequestState Next;

        public override string ToString()
        {
            return string.Format("LD={0} CO={1} VA={2} idx={3} next={4}",
                                Load, Complete, Valid, Index, Next);
        }
    }

    /// <summary>
    /// Consolidates the state machine data from the hardware's BKM16.2, GMV02,
    /// MST10 and MST01 PROMs, essentially.
    /// </summary>
    public static class BookmarkTable
    {
        static BookmarkTable()
        {
            _table = new BookmarkEntry[256];
            InitTable();
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public static BookmarkEntry Lookup(BookmarkType book, RequestState state, int time)
        {
            //
            // Index:
            //		bits	value
            //		7:4		bookmark (cycle type)
            //		3:2		current state
            //		1:0		current Tstate
            //
            int lookup = ((int)book & 0x0f) << 4 | ((int)state << 2) | time;

            Log.Detail(Category.MemCycle, "Bookmark[{0:x3}]: {1}", lookup, _table[lookup]);

            return _table[lookup];
        }

        public static BookmarkEntry Clear => _table[0];

        /// <summary>
        /// Build the bookmarks table which helps drive the memory state machine.
        /// Generated here once, rather than loaded from an external file, eliminating
        /// the external dependency on memstate.txt table and Perl script used to build it.
        /// </summary>
        static void InitTable()
        {
            BookmarkEntry b;

            //
            // This condensed version of the table only cares about the two run
            // states, which is where the action is.  I could squeeze it down by
            // a bit and halve the size, but for now let's just go with it...
            //
            for (var book = BookmarkType.None; book <= BookmarkType.Store; book++)
                for (var state = RequestState.Idle; state <= RequestState.Illegal; state++)
                    for (var time = 0; time < 4; time++)
                    {
                        b = new BookmarkEntry();

                        // Set up the VA flag, IDX and Next
                        switch (book)
                        {
                            case BookmarkType.Fetch:
                            case BookmarkType.Fetch2:
                            case BookmarkType.Fetch4:
                            case BookmarkType.Fetch4R:

                                // Fetches all share common timing
                                if (state == RequestState.RunCycle1)
                                {
                                    if (time == 0 || time == 3)
                                        b.Next = RequestState.RunCycle1;

                                    if (time == 1)
                                        b.Next = RequestState.RunCycle2;
                                }

                                // All the Valid, Index action in the second half
                                if (state == RequestState.RunCycle2)
                                {
                                    b.Valid = true;
                                    b.Load = (time == 2);
                                    b.Complete = (time == 1);

                                    if ((book == BookmarkType.Fetch2 && (time == 1 || time == 3)) ||
                                        (book == BookmarkType.Fetch4 && time == 3) ||
                                        (book == BookmarkType.Fetch4R && time == 0))
                                        b.Index = 1;

                                    if ((book == BookmarkType.Fetch4 && time == 0) ||
                                        (book == BookmarkType.Fetch4R && time == 3))
                                        b.Index = 2;

                                    if ((book == BookmarkType.Fetch4 && time == 1) ||
                                        (book == BookmarkType.Fetch4R && time == 2))
                                        b.Index = 3;

                                    if (time != 1)
                                        b.Next = RequestState.RunCycle2;
                                }
                                break;

                            case BookmarkType.IndFetch:
                            case BookmarkType.IndFetch2:

                                // Entered in RC2, t3: must invalidate the current
                                // op so the next can Recognize in the next Tick

                                // Most transitions here aren't valid
                                b.Next = RequestState.Illegal;

                                // Override the one that is
                                if (state == RequestState.RunCycle2 && time == 3)
                                {
                                    if (book == BookmarkType.IndFetch2)
                                        b.Index = 1;

                                    b.Valid = true;
                                    b.Complete = true;
                                    b.Next = RequestState.Idle;
                                }
                                break;

                            case BookmarkType.RopFetch4:
                            case BookmarkType.RopFetch4R:

                                // Entered in RC1, t2: the special RasterOp fetch
                                // skips the normal wait in t0,t1 and immediately
                                // pumps out the next quad word                               
                                if (state == RequestState.RunCycle1)
                                {
                                    b.Valid = (time > 1);

                                    if (time == 2)
                                    {
                                        b.Load = true;
                                        b.Index = (book == BookmarkType.RopFetch4 ? 0 : 3);
                                        b.Next = RequestState.RunCycle1;
                                    }
                                    else if (time == 3)
                                    {
                                        b.Index = (book == BookmarkType.RopFetch4 ? 1 : 2);
                                        b.Next = RequestState.RunCycle2;
                                    }
                                    else
                                    {
                                        b.Next = RequestState.Illegal;
                                    }
                                }
                                else if (state == RequestState.RunCycle2)
                                {
                                    b.Valid = (time < 2);

                                    if (time == 0)
                                    {
                                        b.Index = (book == BookmarkType.RopFetch4 ? 2 : 1);
                                        b.Next = RequestState.RunCycle2;
                                    }
                                    else if (time == 1)
                                    {
                                        b.Complete = true;
                                        b.Index = (book == BookmarkType.RopFetch4 ? 3 : 0);
                                        b.Next = RequestState.Idle;
                                    }
                                    else
                                    {
                                        b.Next = RequestState.Illegal;
                                    }
                                }
                                break;

                            case BookmarkType.Store:

                                // Store has odd timing; it starts in t2 and
                                // finishes "short" in t3 (just half a cycle!)
                                if (state == RequestState.RunCycle1)
                                {
                                    if (time == 2) b.Next = state;

                                    if (time == 3)
                                    {
                                        b.Valid = true;
                                        b.Complete = true;
                                    }
                                }
                                break;

                            case BookmarkType.Store2:
                            case BookmarkType.Store4:
                            case BookmarkType.Store4R:

                                // Other stores run in the first half
                                if (state == RequestState.RunCycle1)
                                {
                                    b.Valid = ((book == BookmarkType.Store2 && time < 2) ||
                                                book == BookmarkType.Store4 ||
                                                book == BookmarkType.Store4R);

                                    // All stores complete in t3
                                    b.Complete = (time == 3);

                                    if (book == BookmarkType.Store2 && time == 1)
                                        b.Index = 1;

                                    if (book == BookmarkType.Store4)
                                        b.Index = time;

                                    if (book == BookmarkType.Store4R)
                                        b.Index = ~time & 0x3;
                                }

                                // Runs in RC1, transitions back to Idle in t3
                                if (state == RequestState.RunCycle1 && time < 3)
                                    b.Next = state;

                                break;

                            case BookmarkType.RopStore4:
                            case BookmarkType.RopStore4R:

                                if (state == RequestState.RunCycle1)
                                {
                                    b.Valid = (time > 0);

                                    // Shifted by a cycle
                                    if (book == BookmarkType.RopStore4 && time > 0)
                                        b.Index = (time + 3) & 0x3;

                                    // ...and inverted
                                    if (book == BookmarkType.RopStore4R)
                                        b.Index = ~(time + 3) & 0x3;

                                    b.Next = (time < 3 ? RequestState.RunCycle1 : RequestState.RunCycle2);
                                }

                                if (state == RequestState.RunCycle2)
                                {
                                    if (time == 0)
                                    {
                                        b.Valid = b.Complete = true;

                                        if (book == BookmarkType.RopStore4)
                                            b.Index = 3;
                                    }
                                    else
                                    {
                                        b.Next = RequestState.Illegal;
                                    }
                                }
                                break;
                        }

                        // Save it!
                        int lookup = ((int)book & 0x0f) << 4 | ((int)state << 2) | time;
                        _table[lookup] = b;
                    }
        }


        [Conditional("DEBUG")]
        public static void DumpTable()
        {
            Console.WriteLine("Bookmarks (new):");
            Console.WriteLine("\tAddr  Cycle       State      T  |  LD     CO     VA     IDX  Next");
            //                   ----  ----------  ---------  -  |  -----  -----  -----  ---  ---------

            for (var i = 0; i < _table.Length; i++)
            {
                var c = (BookmarkType)(i >> 4);
                var s = (RequestState)((i >> 2) & 0x3);
                var t = i & 0x3;

                // Skip extraneous stuff
                if (c <= BookmarkType.Unused) continue;
                if (s < RequestState.RunCycle1 || s > RequestState.RunCycle2) continue;

                var b = _table[i];

                Console.WriteLine("\t{0:x4}  {1,-10}  {2,-9}  {3}  |  {4,-5}  {5,-5}  {6,-5}   {7}   {8}",
                                   i, c, s, t, b.Load, b.Complete, b.Valid, b.Index, b.Next);
            }
        }

        static BookmarkEntry[] _table;
    }
}
