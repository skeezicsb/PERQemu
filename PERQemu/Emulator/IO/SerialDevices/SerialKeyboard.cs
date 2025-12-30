//
// SerialKeyboard.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.IO.Z80;

namespace PERQemu.IO.SerialDevices
{
    /// <summary>
    /// The PERQ-2 "VT100-style" serial keyboard.
    /// </summary>
    public class SerialKeyboard : ISIODevice
    {
        public SerialKeyboard()
        {
            // This keyboard doesn't latch data or interrupt the Z80 the way the
            // PERQ-1 parallel unit does; it lets the SIO interrupt as keystrokes
            // are received.  The hardware is set to a fixed 300 baud, 8/N/1, and
            // normal SIO and circular buffer processing by the Z80 code deals
            // with possible overruns.  So we don't sweat any of that here. :-)
        }

        public void RegisterReceiveDelegate(ReceiveDelegate rxDelegate)
        {
            _rxDelegate = rxDelegate;
        }

        public void Reset()
        {
            Log.Debug(Category.Keyboard, "Reset");
        }

        public void QueueInput(byte key)
        {
            // Key bytes transmitted inverted; for logging, flip it back
            Log.Detail(Category.Keyboard, "Queuing key '{0}' (0x{1:x2})", (char)(~key & 0x7f), key);
            _rxDelegate(key);
        }

        public void Transmit(byte value)
        {
            throw new NotImplementedException();
        }

        public void TransmitAbort()
        {
            throw new NotImplementedException();
        }

        public void TransmitBreak()
        {
            Log.Detail(Category.Keyboard, "VT100 Keyboard received a Break?");
        }


        ReceiveDelegate _rxDelegate;
    }
}
