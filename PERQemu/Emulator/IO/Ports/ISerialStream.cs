//
// ISerialStream.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.  It is a rewrite of the Mono implementation:
// 
//      System.IO.Ports.ISerialStream.cs
//
//      Authors:
//          Carlos Alberto Cortez (calberto.cortez@gmail.com)
//
//      (c) Copyright 2006 Novell, Inc. (http://www.novell.com)
// 
// Originally licensed under the MIT X11 and/or GNU GPL.  Please see the file
// COPYING.txt in the top-level of the PERQemu distribution for more information.
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

using System.Runtime.InteropServices;

namespace PERQemu.IO.Ports
{
    public delegate void SerialErrorDelegate(char portID, string errorMessage);

    interface ISerialStream
    {
        int Read([Out] byte[] buffer, int count);
        int Write([In] byte[] buffer, int count);

        void SetAttributes(SerialSettings settings);

        SerialSignal GetSignals();
        void SetSignal(SerialSignal signal, bool value);

        void SetBreakState(bool value);

        int BytesToRead { get; }
        int BytesToWrite { get; }

        int ReadTimeout { get; set; }
        int WriteTimeout { get; set; }

        void RegisterErrorDelegate(SerialErrorDelegate handler, char id = 'A');

        void Flush();
        void Dispose();
    }
}
