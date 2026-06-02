//
// SerialTypes.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu.IO.Ports
{
    public enum Parity
    {
        None = 0,
        Odd,
        Even        // Note: The Z80 SIO doesn't support Mark or Space
    }

    public enum StopBits
    {
        None = 0,
        One,
        Two,
        OnePointFive
    }

    public enum Handshake
    {
        None = 0,
        XOnXOff,
        RTSCTS,
        Both
    }

    public enum SerialOptions
    {
        None = 0,
        DCDForceOn,
        DCDFollowDSR
    }

    [Flags]
    public enum SerialSignal
    {
        None = 0,   // No signals active
        DCD = 1,    // Carrier detect 
        CTS = 2,    // Clear to send
        DSR = 4,    // Data set ready
        DTR = 8,    // Data terminal ready
        RTS = 16    // Request to send
    }

    public struct SerialSettings
    {
        public SerialSettings(int baud, int data, Parity parity, StopBits stop,
                              Handshake hs = Handshake.None,
                              SerialOptions opt = SerialOptions.None)
        {
            BaudRate = baud;
            DataBits = data;
            Parity = parity;
            StopBits = stop;
            FlowControl = hs;
            Options = opt;
        }

        public override string ToString()
        {
            // Basic port settings
            return $"{BaudRate} {DataBits} {Parity} {StopBits}";
        }

        public string ToStringExt()
        {
            // All fields
            return ToString() +
                (FlowControl != Handshake.None? $" {FlowControl}" : "") +
                (Options != SerialOptions.None? $" ({Options})" : "");
        }

        public int BaudRate;
        public int DataBits;
        public Parity Parity;
        public StopBits StopBits;
        public Handshake FlowControl;
        public SerialOptions Options;

        public readonly static SerialSettings Defaults = new SerialSettings(9600, 8, Parity.None, StopBits.One);
    }
}
