//
// SerialKeyboard.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.IO.Z80;

namespace PERQemu.IO.SerialDevices
{
    /// <summary>
    /// The PERQ-2 "VT100-style" serial keyboard.
    /// </summary>
    public class SerialKeyboard : SerialDevice
    {
        public SerialKeyboard(Z80System sys) : base(sys)
        {
            // This keyboard doesn't latch data or interrupt the Z80 the way the
            // PERQ-1 parallel unit does; it lets the SIO interrupt as keystrokes
            // are received.  The hardware is set to a fixed 300 baud, 8/N/1, and
            // normal SIO and circular buffer processing by the Z80 code deals
            // with possible overruns.  So we don't sweat any of that here. :-)

            // Like the PERQ-1 keyboard, this is written on the main thread and
            // read on the Z80 thread.  I shouldn't be so cavalier with the lack
            // of proper locking but it's just not worth fussing over...

            _name = "VT100-style serial keyboard";
            _rxRate = Conversion.BaudRateToNsec(300);
        }

        public override bool ReadReady => _keyReady;

        public override bool CTS => true;
        public override bool DCD => true;

        public override void Reset()
        {
            _key = 0;
            _keyReady = false;

            Log.Debug(Category.Keyboard, "Reset");
        }

        public override byte Receive()
        {
            _keyReady = false;
            return _key;
        }

        public void QueueInput(byte key)
        {
            _key = key;
            _keyReady = true;

            // Key bytes are transmitted inverted; for logging, flip it back
            Log.Debug(Category.Keyboard, "Queuing key '{0}' (0x{1:x2})", (char)(~key & 0x7f), key);
        }

        byte _key;
        bool _keyReady;
    }
}
