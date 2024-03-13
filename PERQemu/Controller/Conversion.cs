//
// Conversion.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu
{
    public static class Conversion
    {
        /// <summary>
        /// Conversion from millseconds to nanoseconds
        /// </summary>
        public static readonly ulong MsecToNsec = 1000000;

        /// <summary>
        /// Conversion from nanoseconds to milliseconds
        /// </summary>
        public static readonly double NsecToMsec = 0.000001;

        /// <summary>
        /// Conversion from microseconds to nanoseconds
        /// </summary>
        public static readonly ulong UsecToNsec = 1000;

        /// <summary>
        /// Conversion from microseconds to seconds
        /// </summary>
        public static readonly double UsecToSec = 0.000001;

        /// <summary>
        /// Conversion from seconds to milliseconds
        /// </summary>
        public static readonly double MsecToSec = 0.001;

        /// <summary>
        /// Convert disk revolutions to nanoseconds for scheduling index pulses.
        /// </summary>
        public static ulong RPMtoNsec(int rpm)
        {
            // Put the ()s in the right spot to preserve accuracy...
            return (ulong)(1 / (rpm / 60.0) * 1000 * MsecToNsec);
        }

        /// <summary>
        /// Convert a timer count to a standard baud rate.  Returns 0 if TC does
        /// not map to a standard rate (from 110 to 19200).
        /// </summary>
        /// <remarks>
        /// The PERQ-1/IOB Z80's calculations are based on counts from a Zilog
        /// CTC chip and clock rate.  For PERQ-2/EIO, the i8254 PIT chip is used
        /// based on the faster 4Mhz timing, and supports up to 19200.
        /// </remarks>
        public static int TimerCountToBaudRate(int rate, int prescale = 16)
        {
            //
            // The CTC is programmed to fire the RS232A Tx/Rx clock at 16x the
            // chosen baud rate; this corresponds to the TC value written into
            // the timer's count register, and the "x16 mode" selected in the
            // SIO channel setup.  For rate limiting, TC is multiplied by the
            // Z80's clock tick length (2.4576Mhz or 407ns) to get the interval
            // in nanoseconds between byte transmissions!  (10 bit times/byte)
            //
            // For the PIT, set prescale to 1 and look up the rate directly.
            // This lazy hack that works because the PIT doesn't do prescaling
            // and the rate values are distinct.  Cheeeeeeeese!!
            //
            var baseRate = rate / prescale;
            int baud = 0;

            switch (baseRate)
            {
                // Standard CTC rates from 9600 down to 150 baud
                case 1:
                case 2:
                case 4:
                case 8:
                case 16:
                case 32:
                case 64:
                    baud = 9600 / baseRate;
                    break;

                // EIO and the i8254 use this table based on the 4MHz clock
                case 13:
                case 26:
                case 52:
                case 104:
                case 208:
                case 417:
                case 833:
                case 1667:
                    baud = 19200 / (baseRate / 13);
                    break;

                // Check the outlier in case there's an ASR-33 attached!
                case 87:        // CTC
                case 2273:      // PIT
                    baud = 110;
                    break;

                default:
                    Log.Error(Category.RS232, "Could not decode baud rate from timer value {0}", rate);
                    break;
            }

            return baud;
        }

        /// <summary>
        /// Convert a baud rate to the number of nanoseconds between characters
        /// transmitted/received on a serial data stream, suitable for scheduling
        /// Scheduler events.
        /// </summary>
        /// <remarks>
        /// This calculation doesn't account for the granularity of the actual
        /// clock period used by the hardware, but it doesn't have to be that
        /// accurate.  We just want to "pace" data from the virtual machine so
        /// that the Z80/PERQ see realistic interrupt rates for serial transfers.
        /// Assume 10 bits per character (8-N-1).
        /// </remarks>
        public static ulong BaudRateToNsec(int baud)
        {
            return (ulong)(1000000000 / (baud / 10));
        }
    }
}
