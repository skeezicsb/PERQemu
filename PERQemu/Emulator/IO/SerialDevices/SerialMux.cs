//
// SerialMux.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
    /// Provide a single SIO device that can split the traffic on two channels
    /// to two separate devices.  On PERQ this means the "speech" device uses
    /// the transmit half of a channel to stream bytes through the MC3417 for
    /// audio output, while the Kriz tablet uses the receive half to report
    /// mouse updates.  Should handle both the original IOB implementation and
    /// the later EIO as well.  Also handles DMA for "HiVol" operation.
    /// </summary>
    public sealed class SerialMux : ISIODevice
    {
        public SerialMux()
        {
            Log.Info(Category.SIO, "Created {0} mux device", IOBoard.Name);
        }

        public void AttachRxDevice(ISIODevice rxDev)
        {
            _rxDevice = rxDev;
            Log.Info(Category.SIO, "Attached Rx device {0}", rxDev);
        }

        public void AttachTxDevice(ISIODevice txDev)
        {
            _txDevice = txDev;
            Log.Info(Category.SIO, "Attached Tx device {0}", txDev);
        }

        //
        // ISIODevice implementation
        //

        public void Reset()
        {
            _txDevice?.Reset();
            _rxDevice?.Reset();
        }

        /// <summary>
        /// If the Rx device is attached, pass through the receive delegate.
        /// </summary>
        public void RegisterReceiveDelegate(ReceiveDelegate rxDelegate)
        {
            _rxDevice?.RegisterReceiveDelegate(rxDelegate);
        }

        /// <summary>
        /// If the Tx device is attached, pass through the bytes to transmit.
        /// </summary>
        public void Transmit(byte value)
        {
            _txDevice?.Transmit(value);
        }

        // Not used by Speech device
        public void TransmitBreak()
        {
            _txDevice?.TransmitBreak();
        }

        public ulong TransmitRate => _txDevice?.TransmitRate ?? 0;
        public ulong ReceiveRate => _rxDevice?.ReceiveRate ?? 0;

        ISIODevice _txDevice;       // MC3417 for audio output
        ISIODevice _rxDevice;       // KrizTablet for mouse input
    }
}
