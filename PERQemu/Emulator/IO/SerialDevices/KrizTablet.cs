//
// KrizTablet.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.IO.Z80;

namespace PERQemu.IO.SerialDevices
{
    /// <summary>
    /// Implements the Kriz tablet.  This is a custom electromagnetic ranging
    /// tablet designed and patented by PERQ engineer J. Stanley Kriz, used
    /// primarily on the PERQ-2 line of workstations.  Comes in both portrait
    /// and landscape orientations to match the selected display.  Interfaces
    /// with the Z80 via a serial port.
    /// </summary>
    /// <remarks>
    /// See the file Docs/SerialPorts.txt for information about the tablet's
    /// message format and other operational details.
    /// </remarks>
    public class KrizTablet : SerialDevice
    {
        public KrizTablet(Z80System sys, PERQSystem perq) : base(sys)
        {
            _name = "Kriz tablet";
            _perq = perq;
            _sampleEvent = null;

            // 60 samples/sec
            _sampleRate = (ulong)(16.666667 * Conversion.MsecToNsec);

            // This should be 32kHz, but we'll use 16kHz (shared Speech clock)
            // to reduce overhead a little bit.  Sync mode (8 bit chars).
            _dataRate = Conversion.BaudRateToNsec(16000, 8);

            _sample = new byte[8];
            _nextByte = 0;
        }

        public override ulong ReceiveRate => _dataRate;
        public override bool ReadReady => _nextByte > 0;


        public override void Reset()
        {
            _scheduler.Cancel(_sampleEvent);
            _sampleEvent = _scheduler.Schedule(_sampleRate, SampleTablet);

            _nextByte = 0;

            Log.Debug(Category.Tablet, "Kriz reset");
        }

        public override byte Receive()
        {
            var value = _sample[_nextByte];

            if (_nextByte > 0) _nextByte--;

            return value;
        }

        void SampleTablet(ulong skewNsec, object context)
        {
            // Don't clobber the sample buffer if the receiver is active
            if (_nextByte == 0 || _nextByte == 7)
            {
                // SDL provides absolute mouse positions clipped to the PERQ screen
                // dimensions for us, so there's no need to adjust for display width.
                // Apply X/Y "kluge" values based on POS tablet driver's expectations
                int tabX = _perq.HID.MouseX + 64;
                int tabY = _perq.VideoController.DisplayHeight -
                           _perq.Display.TopY -
                           _perq.HID.MouseY + 64;

                // Format 'em
                var tab1 = (byte)(((tabX >> 8) & 0x0f) |
                                   (_perq.HID.MouseOffTablet ? 0x40 : 0) |
                                   (_perq.Config.Display == Config.DisplayType.Landscape ? 0x20 : 0));
                var tab2 = (byte)(tabX & 0xff);
                var tab3 = (byte)(((tabY >> 8) & 0x0f) | (_perq.HID.MouseButton << 5));
                var tab4 = (byte)(tabY & 0xff);

                // Save the sample data - invert (active low data) if NOT EIO
                _sample[7] = (_perq.IOB.IsEIO ? Sync : (byte)~Sync);
                _sample[6] = (_perq.IOB.IsEIO ? tab1 : (byte)~tab1);
                _sample[5] = (_perq.IOB.IsEIO ? tab2 : (byte)~tab2);
                _sample[4] = (_perq.IOB.IsEIO ? tab3 : (byte)~tab3);
                _sample[3] = (_perq.IOB.IsEIO ? tab4 : (byte)~tab4);

                // CIO and EIO explicitly do three extra reads to "clear out any junk
                // left in the chip" (i.e., the padding/CRC bytes the SIO injected)

                // Let the receiver know we have data available
                _nextByte = 7;

                Log.Debug(Category.Tablet, "Kriz sampled: x={0} y={1} button={2}",
                                            tabX, tabY, (tab3 >> 5));
            }

            // Wait a jiffy and do it again
            _sampleEvent = _scheduler.Schedule(_sampleRate, SampleTablet);
        }

        public override void Status()
        {
            Console.WriteLine("Kriz tablet status:");
            Console.Write("  Sample buffer:");
            for (var i = 6; i > 2; i--) Console.Write($"  0x{_sample[i]:x2}");
            Console.WriteLine($"  Next: {_nextByte}");
        }


        readonly byte Sync = 0x7e;      // Standard SDLC flag character
        readonly ulong _sampleRate;     // Mouse position sample rate
        ulong _dataRate;                // Baud rate for serial transmission

        int _nextByte;
        byte[] _sample;

        SchedulerEvent _sampleEvent;
        PERQSystem _perq;
    }
}
