﻿//
//  DevicePerformance.cs
//
//  Author:  S. Boondoggle <skeezicsb@gmail.com>
//
//  Copyright (c) 2022, Boondoggle Heavy Industries, Ltd.
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

using System;

namespace PERQmedia
{
    /// <summary>
    /// Device performance characteristics that the emulator cares about.
    /// </summary>
    public struct DevicePerformance
    {
        public DevicePerformance(int rpm, int pulse, int delay, int seekMin, int seekMax, int settle, int xfer)
        {
            RPM = rpm;
            IndexPulse = pulse;
            StartupDelay = delay;
            MinimumSeek = seekMin;
            MaximumSeek = seekMax;
            HeadSettling = settle;
            TransferRate = xfer;
        }

        public override string ToString()
        {
            return string.Format("[RPM {0}, Index {1:n}us, Delay {2}sec, Rate {3:n}KB/sec]\n" +
                                 "[Min seek {4}ms, Max seek {5}ms, Settling {6}ms]", RPM,
                                 (IndexPulse / 1000.0),     // ns -> usec
                                 (StartupDelay / 1000),     // msec -> sec
                                 (TransferRate / 1000.0),   // bytes -> Kbytes
                                 MinimumSeek, MaximumSeek, HeadSettling);
        }

        public readonly int RPM;
        public readonly int IndexPulse;      // in nsec
        public readonly int StartupDelay;    // in msec
        public readonly int MinimumSeek;     // in msec
        public readonly int MaximumSeek;     // in msec
        public readonly int HeadSettling;    // in msec
        public readonly int TransferRate;    // in bytes/sec

        //
        // Shugart SA851 8" floppy drive
        //
        public static DevicePerformance SA851 = new DevicePerformance(360, 400000, 90, 3, 231, 15, 62500);

        //
        // Shugart SA4000 series 14" hard drives
        //
        public static DevicePerformance SA4000 = new DevicePerformance(2964, 1100, 90000, 20, 140, 20, 888375);

        //
        // Specifications for the 8", 5.25" and other drive types are
        // dynamically configured at runtime.
        //
    }
}