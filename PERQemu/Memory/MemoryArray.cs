// memoryarray.cs - Copyright 2019 S. Boondoggle (skeezicsb@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using System;
using System.Runtime.InteropServices;

namespace PERQemu.Memory
{
    /// <summary>
    /// C# purists, look away now to maintain plausible deniability.
    /// </summary>
    [StructLayout(LayoutKind.Explicit)]
    internal struct Core
    {
        [FieldOffset(0)]
        public ulong[] Quads;

        [FieldOffset(0)]
        public ushort[] Words;

        [FieldOffset(0)]
        public byte[] Bytes;
    }

    /// <summary>
    /// Access the core by byte (cursor line updating), word (normal access)
    /// or quad (faster video updates).  For speed, no explicit bounds checking
    /// is done here; the Fetch/Store routines at least clip the address to 20
    /// (or someday 24) bits at least...
    /// </summary>
    public class MemoryArray
    {
        public MemoryArray(int sizeInBytes)
        {
            _size = sizeInBytes;

            // The unified array
            _memory = new Core();
            _memory.Bytes = new byte[_size];

#if TRACING_ENABLED
            if (Trace.TraceOn)
            Trace.Log(LogType.MemoryState, "MemoryCore: allocated {0} bytes.", _size);
#endif
        }

        public int SizeInQuads
        {
            get { return _size / 8; }
        }

        public int SizeInWords
        {
            get { return _size / 2; }
        }

        public int SizeInBytes
        {
            get { return _size; }
        }

        public void WriteQuad(int a, ulong q)
        {
            _memory.Quads[a] = q;
        }

        public ulong ReadQuad(int a)
        {
            return _memory.Quads[a];
        }

        public void WriteWord(int a, ushort w)
        {
            _memory.Words[a] = w;
        }

        public ushort ReadWord(int a)
        {
            return _memory.Words[a];
        }

        public void WriteByte(int a, byte b)
        {
            _memory.Bytes[a] = b;
        }

        public byte ReadByte(int a)
        {
            return _memory.Bytes[a];
        }

        private int _size;
        private Core _memory;
    }
}
