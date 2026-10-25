//
// MemoryController.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using System.Runtime.CompilerServices;

namespace PERQemu.Memory
{
    public class MemoryRuleViolation : Exception
    {
        public MemoryRuleViolation(int time, string msg) : base($"Memory violation in T{time}: {msg}") { }
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
        public MemoryController(MemoryBoard mem)
        {
            _mem = mem;

            // Can have two requests "in flight"
            _fetch = new MemoryRequest(_mem);
            _store = new MemoryRequest(_mem);
        }

        public void Reset()
        {
            _fetch.Reset();
            _store.Reset();

            _wait = false;

            Log.Info(Category.Memory, "Controller reset");
        }

        public MemoryRequest MDI => _fetch;
        public MemoryRequest MDO => _store;

        public bool Wait => _wait;


        /// <summary>
        /// Advance the state machine and set flags for the current microcycle.
        /// May abort the current instruction if a new request is issued at the
        /// wrong time.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clock(MemoryCycle nextCycle)
        {
            Retire();
            Recognize();
            RunStateMachine(nextCycle);
        }

        /// <summary>
        /// Accept a new request into the fetch or store pipeline.  Pass in the
        /// next bookmark value.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Request(MemoryCycle cycleType, int startAddr)
        {
            if (IsFetch(cycleType))
                _fetch.Request(cycleType, startAddr, _nextBookmark);
            else
                _store.Request(cycleType, startAddr, _nextBookmark);
        }

        /// <summary>
        /// If the current op is complete and a pending op is ready, promote it.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        void Recognize()
        {
            // Separating requests sort of mimics the "OldRead" SM loop
            // in the hardware's MST PROMs
            if (_fetch.Ready)
            {
                // Don't clobber the active op
                if (_fetch.Active)
                {
                    Log.Debug(Category.MemCycle, "Recognize delayed, {0} still active", _fetch.CycleType);
                }
                else
                {
                    _fetch.Start();
                    Log.Debug(Category.MemCycle, "Recognized {0}", _fetch.CycleType);
                }
            }

            if (_store.Ready)
            {
                // Never clobber a running store
                if (_store.Active)
                {
                    Log.Debug(Category.MemCycle, "Recognize delayed, {0} still active", _store.CycleType);
                }
                else
                {
                    _store.Start();
                    Log.Debug(Category.MemCycle, "Recognized {0}", _store.CycleType);
                }
            }
        }

        /// <summary>
        /// Clear completed requests.  For stores, commit the appropriate word(s)
        /// in the store buffer to memory.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Retire()
        {
            if (_fetch.Complete)
            {
                Log.Debug(Category.MemCycle, "Retired {0}", _fetch.CycleType);
                _fetch.Clear();
            }

            if (_store.Complete)
            {
                // Do the writeback
                _store.Commit();

                // Log it and clear it
                Log.Debug(Category.MemCycle, "Retired {0}", _store.CycleType);
                _store.Clear();
            }
        }


        /// <summary>
        /// Run "the" memory state machine.  Here we clock both of the separate
        /// requests, which sort of mashes up the hardware's CPU state (BKM16 and
        /// GMV02 PROMs) and the MEM board state machine (MST10/MST01).  Sets the
        /// flags for data into and out of the buffers, and next states for all
        /// active requests.  Much sausage making is happening here.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void RunStateMachine(MemoryCycle nextCycle)
        {
            // Attached to next op at Request() in bottom of cycle
            _nextBookmark = (BookmarkType)nextCycle;

            // Always positively assert this
            _wait = false;

            // Something wicked this way comes
            if (nextCycle != MemoryCycle.None)
            {
                switch (_mem.TState)
                {
                    case 0:
                        // Special RasterOp overlapped fetch/store case allows for a
                        // store in t0; note that the hardware doesn't actually enforce
                        // a check that RasterOp is active!?
                        //if (//_mem.RopEnabled &&
                        if (((nextCycle == MemoryCycle.Store4 && _fetch.CycleType == MemoryCycle.Fetch4) ||
                             (nextCycle == MemoryCycle.Store4R && _fetch.CycleType == MemoryCycle.Fetch4R)))
                        {

                            // Update the current request to patch the transition
                            _nextBookmark = (_fetch.CycleType == MemoryCycle.Fetch4 ? BookmarkType.RopStore4 :
                                                                                      BookmarkType.RopStore4R);

                            Log.Debug(Category.MemCycle,
                                     "OVERLAPPED {0}: active={1} state={2} rop={3} switch={4}",
                                      nextCycle, _store.Bookmark, _store.State, _mem.RopEnabled, _nextBookmark);

                            // Recognize immediately, don't stall the running fetch
                            _store.Ready = true;
                        }
                        else
                        {
                            _wait = true;
                        }
                        break;

                    case 1:
                        // All new requests stall in t1
                        _wait = true;
                        break;

                    case 2:
                        // Single word Store runs in t2
                        if (nextCycle == MemoryCycle.Store)
                        {
                            _store.Ready = true;
                        }
                        else
                        {
                            _wait = true;
                        }
                        break;

                    case 3:
                        // All others in t3
                        switch (nextCycle)
                        {
                            case MemoryCycle.Fetch:
                            case MemoryCycle.Fetch2:
                            case MemoryCycle.Fetch4:
                            case MemoryCycle.Fetch4R:

                                // Handle overlapping fetch cases
                                if (_fetch.Active)
                                {
                                    // Oh yes, I have done this.  Deal with it. ;-P
                                    var book = (int)_fetch.Bookmark / 2;

                                    Log.Debug(Category.MemCycle,
                                             "OVERLAPPED {0}: active={1} state={2} rop={3} switch={4}",
                                              nextCycle, _fetch.Bookmark, _fetch.State, _mem.RopEnabled, book);

                                    // If RasterOp is NOT enabled and a quad fetch is active,
                                    // we must let it complete (blow a rev and wait); otherwise
                                    // we switch to the indirect states to handle the transition
                                    if (_fetch.CycleType == MemoryCycle.Fetch4 || _fetch.CycleType == MemoryCycle.Fetch4R)
                                    {
                                        if (!_mem.RopEnabled)
                                        {
                                            _wait = true;
                                        }
                                        else
                                        {
                                            // For RasterOp, the current Fetch4/R is allowed
                                            // to complete normally; the _next_ op is switched
                                            // to a RopFetch4/R and will stall in Recognize()
                                            // until t2, where it reloads and continues without
                                            // the t0, t1 gap (so two quad words are delivered
                                            // in succession).  It's a slightly ugly tradeoff
                                            // but I'll live with it
                                            _nextBookmark = (BookmarkType)book;
                                            _fetch.Ready = true;
                                        }
                                    }
                                    else
                                    {
                                        // Bump the current bookmark, enable the next op
                                        _fetch.Bookmark = (BookmarkType)book;
                                        _fetch.Ready = true;
                                    }
                                }
                                else
                                {
                                    // Clear to launch
                                    _fetch.Ready = true;
                                }
                                break;

                            case MemoryCycle.Store:

                                // Single word stores stall in t3
                                _wait = true;
                                break;

                            case MemoryCycle.Store2:
                            case MemoryCycle.Store4:
                            case MemoryCycle.Store4R:

                                // Another special case: a Store2 can be pipelined in t3 since
                                // we aren't waiting on the last word to arrive and commit.
                                // Otherwise, we have to blow a rev for the final Tock()
                                if (_store.Active && _store.CycleType != MemoryCycle.Store2)
                                {
                                    _wait = true;
                                }
                                else
                                {
                                    // Start the next Store2/4/4R
                                    _store.Ready = true;
                                }
                                break;
                        }
                        break;
                }
            }

            // Even if the CPU stalls, update the in-flight requests
            if (_fetch.Active) _fetch.Update();
            if (_store.Active) _store.Update();

            // Remove this, eventually, once confidence is restored :-)
            if (_fetch.State == RequestState.Illegal || _store.State == RequestState.Illegal)
                throw new MemoryRuleViolation(_mem.TState, "Illegal state entered");
        }

        /// <summary>
        /// Return true if the memory request type is a Fetch.
        /// </summary>
        /// <remarks>
        /// The SF encodings are very specific: the hardware/firmware uses the
        /// fact that all Fetch types are even-numbered, Stores types are odd.
        /// In case you were wondering.
        /// </remarks>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        bool IsFetch(MemoryCycle c)
        {
            return c > MemoryCycle.None && ((int)c & 0x1) == 0;
        }

        /// <summary>
        /// Dumps the current and pending requests. Quick and dirty debugging aid.
        /// </summary>
        //[Conditional("DEBUG")]
        public void DumpState()
        {
            Console.WriteLine($"Controller: T{_mem.TState} wait={_wait}");
            Console.WriteLine($"Fetch:   {_fetch}");
            Console.WriteLine($"Flags:   {_fetch.Flags}");
            if (_fetch.Ready) Console.WriteLine($"Pending: {_fetch.Pending()}");

            Console.WriteLine($"Store:   {_store}");
            Console.WriteLine($"Flags:   {_store.Flags}");
            if (_store.Ready) Console.WriteLine($"Pending: {_store.Pending()}");
        }


        MemoryBoard _mem;

        MemoryRequest _store;
        MemoryRequest _fetch;

        BookmarkType _nextBookmark;

        bool _wait;
    }
}
