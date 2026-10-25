//
// MemoryRequest.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

using System.Runtime.CompilerServices;

namespace PERQemu.Memory
{
    public enum RequestState
    {
        Idle = 0,
        RunCycle1,
        RunCycle2,
        Illegal
    }

    /// <summary>
    /// Represents a request to the memory subsystem.
    /// </summary>
    public class MemoryRequest
    {
        public MemoryRequest(MemoryBoard mem)
        {
            // Direct access to core
            _mem = mem;

            // Local quad word buffer
            _buf = new Core();
            _buf.Words = new ushort[4];

            _flags = new BookmarkEntry();
        }

        public bool Active;
        public bool Ready;

        public MemoryCycle CycleType;
        public BookmarkType Bookmark;

        // For debugging
        public BookmarkEntry Flags => _flags;

        public RequestState State;
        public RequestState NextState;

        public int StartAddress;
        public int Address => _baseAddress + _bufIndex;
        public int Index => _flags.Index;

        public ushort Data
        {
            get { return _buf.Words[_bufIndex]; }
            set { _buf.Words[_bufIndex] = value; }
        }

        public bool Valid => _flags.Valid;
        public bool Complete => _flags.Complete;


        public override string ToString()
        {
            return string.Format("Type={0} Start={1:x6} Book={2} Active={3} Ready={4} State={5}",
                                  CycleType, StartAddress, Bookmark, Active, Ready, State);
        }

        public string Pending()
        {
            return string.Format("Type={0} Start={1:x6} Book={2}",
                                 _nextCycle, _nextAddress, _nextBookmark);
        }

        public void Reset()
        {
            Clear();

            Ready = false;
            _nextAddress = -1;
            _nextCycle = MemoryCycle.None;
            _nextBookmark = BookmarkType.None;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Clear()
        {
            Active = false;

            State = NextState = RequestState.Idle;

            CycleType = MemoryCycle.None;
            Bookmark = BookmarkType.None;
            _flags = BookmarkTable.Clear;

            StartAddress = -1;
            _baseAddress = 0;
            _buf.Quads[0] = 0ul;
        }

        /// <summary>
        /// Accept a new memory request (at the bottom of the CPU cycle, after R
        /// is computed and latched as MADR).  Because the CPU aborts until the
        /// correct cycle when issuing new memory operations, we simply latch the
        /// new request and let the state machine do all the right magic at the
        /// next Clock().
        /// </summary>
        public void Request(MemoryCycle cycleType, int startAddr, BookmarkType book)
        {
            _nextAddress = startAddr;
            _nextCycle = cycleType;
            _nextBookmark = book;

            Log.Debug(Category.MemCycle, "Requested {0}", cycleType);
        }

        /// <summary>
        /// Update the fields relevant to starting OR overlapping a new request.
        /// </summary>
        public void Start()
        {
            StartAddress = _nextAddress;
            CycleType = _nextCycle;
            Bookmark = _nextBookmark;
            NextState = RequestState.RunCycle1;

            _bufIndex = StartAddress & 0x3;
            _baseAddress = StartAddress & 0xfffffc;

            // Small optimization? Nothing accesses flags between Start and Update
            // so it's a wasted lookup... 
            //_flags = BookmarkTable.Lookup(Bookmark, NextState, _mem.TState);

            Active = true;
            Ready = false;
        }

        /// <summary>
        /// Load a quad word from memory into the fetch buffer.
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void LoadBuffer()
        {
            _buf.Quads[0] = _mem.FetchQuad(_baseAddress >> 2);

            Log.Debug(Category.MemCycle, "Load Fetch buffer: addr {0:x6} (base {1:x6}) --> {2:x16}",
                                          StartAddress, _baseAddress, _buf.Quads[0]);
        }

        /// <summary>
        /// Flush the store buffer to memory.
        /// </summary>
        public void Commit()
        {
            // Commit the data from the buffers now!?
            switch (CycleType)
            {
                case MemoryCycle.Store4R:
                case MemoryCycle.Store4:
                    // Write out all four words
                    for (var i = 0; i < 4; i++)
                    {
                        _mem.StoreWord(_baseAddress + i, _buf.Words[i]);
                    }
                    break;

                case MemoryCycle.Store2:
                    // Offset to write out the low or high half of the quad
                    var offset = StartAddress & 0x2;

                    for (var i = offset; i < offset + 2; i++)
                    {
                        _mem.StoreWord(_baseAddress + i, _buf.Words[i]);
                    }
                    break;

                case MemoryCycle.Store:
                    // Index is precomputed at Recognize() since it doesn't change
                    _mem.StoreWord(StartAddress, _buf.Words[_bufIndex]);
                    break;
            }

            // Dubious... the individual stores get reported
            Log.Debug(Category.MemCycle, "Commit Store buffer: addr {0:x6} (base {1:x6}) <-- {2:x16}",
                                          StartAddress, _baseAddress, _buf.Quads[0]);
        }

        /// <summary>
        /// Update the index of the current word based on the cycle type and Tstate.
        /// Assumes the caller is updating in a relevant (i.e., Valid) cycle, or it's
        /// wasted effort. :-)
        /// </summary>
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Update()
        {
            Log.Debug(Category.MemCycle, "Update  IN: {0}", this);

            // Bump the state
            State = NextState;

            // Update the current flags
            _flags = BookmarkTable.Lookup(Bookmark, State, _mem.TState);

            // Update the buffer index; for single-word ops, precomputed value doesn't change
            if (Bookmark != BookmarkType.Fetch && Bookmark != BookmarkType.Store)
            {
                // Look it up
                _bufIndex = _flags.Index;

                // Adjust for high/low half of the quad
                if (Bookmark == BookmarkType.Fetch2 || 
                    Bookmark == BookmarkType.IndFetch2 ||
                    Bookmark == BookmarkType.Store2)
                {
                    _bufIndex = _flags.Index + (StartAddress & 0x2);
                }
            }

            NextState = _flags.Next;

            // Load the buffer if it's time
            if (_flags.Load) LoadBuffer();

            Log.Debug(Category.MemCycle, "Update OUT: {0}", this);
        }


        MemoryBoard _mem;

        Core _buf;
        int _bufIndex;
        int _baseAddress;

        BookmarkEntry _flags;

        int _nextAddress;
        MemoryCycle _nextCycle;
        BookmarkType _nextBookmark;
    }
}
