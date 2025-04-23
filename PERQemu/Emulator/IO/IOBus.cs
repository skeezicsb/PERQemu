//
// IOBus.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.Collections.Generic;

namespace PERQemu.IO
{
    /// <summary>
    /// Handle read/write errors in a uniform way.
    /// </summary>
    public class UnhandledIORequestException : Exception
    {
        public UnhandledIORequestException(byte addr) : base($"Unhandled IO Read from port {addr:x2}")
        {
        }

        public UnhandledIORequestException(byte addr, byte val) : base($"Unhandled IO Write 0x{val:x2} to port {addr:x2}")
        {
        }

        public UnhandledIORequestException(byte addr, int val) : base($"Unhandled IO Write 0x{val:x4} to port {addr:x2}")
        {
        }
    }

    /// <summary>
    /// IOBus acts as an arbiter between the PERQ CPU and various devices attached
    /// to the system's IOA (address) and IOD (data) buses on the backplane.  Reads
    /// and writes to a specific device are routed to the board/slot they're connected
    /// to.  It does not at this time deal with details like arbitration and timing.
    /// </summary>
    public sealed class IOBus
    {
        public IOBus()
        {
            _deviceDispatch = new IIODevice[256];
            _devices = new List<IIODevice>(16);
        }

        public void Reset()
        {
            foreach (IIODevice device in _devices)
            {
                device.Reset();
            }
        }

        public void AddDevice(IIODevice device)
        {
            if (device == null) return;

            if (_devices.Contains(device))
            {
                throw new InvalidOperationException("This component has already been added to the bus");
            }

            _devices.Add(device);

            UpdateDispatchTable(device);
        }

        public int IORead(byte ioPort)
        {
            int value = 0xffff;

            try
            {
                IIODevice device = _deviceDispatch[ioPort];
                if (device == null) throw new UnhandledIORequestException(ioPort);

                value = device.IORead(ioPort);
#if DEBUG
                // Reads from the video regs cause too much log spewage
                if (!(device is Memory.VideoController))
                    Log.Debug(Category.IO, "Read 0x{0:x4} from port 0x{1:x2} ({2})",
                                            value, ioPort, device.ToString());
#endif
            }
            catch (UnhandledIORequestException e)
            {
                Log.Warn(Category.IO, e.Message);
            }

            return value;
        }

        public void IOWrite(byte ioPort, int value)
        {
            try
            {
                IIODevice device = _deviceDispatch[ioPort];
                if (device == null) throw new UnhandledIORequestException(ioPort, value);

                value &= 0xffff;
                device.IOWrite(ioPort, value);
#if DEBUG
                // Cut down on too much noise; comment out to debug video
                if (!(device is Memory.VideoController))
                    Log.Debug(Category.IO, "Write 0x{0:x4} to port 0x{1:x2} ({2})",
                                            value, ioPort, device.ToString());
#endif
            }
            catch (UnhandledIORequestException e)
            {
                Log.Warn(Category.IO, e.Message);
            }
        }


        public void Shutdown()
        {
            foreach (IIODevice device in _devices)
            {
                device.Shutdown();
            }

            _devices.Clear();
        }

        /// <summary>
        /// Adds a new device to the dispatch table.
        /// Will throw if conflicts are found.
        /// </summary>
        void UpdateDispatchTable(IIODevice device)
        {
            Log.Debug(Category.IO, "Updating dispatch table for device {0}", device);

            for (int i = 0; i < 255; i++)
            {
                if (device.HandlesPort((byte)i))
                {
                    if (_deviceDispatch[i] != null)
                    {
                        throw new InvalidOperationException($"IO Port conflict at 0x{i:x2} between {device} and {_deviceDispatch[i]}");
                    }

                    _deviceDispatch[i] = device;
                }
            }
        }

        /// <summary>
        /// Dispatch table for device IO
        /// </summary>
        IIODevice[] _deviceDispatch;

        /// <summary>
        /// The devices attached to the bus
        /// </summary>
        List<IIODevice> _devices;
    }
}
