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

using System.IO;

using PERQemu.IO.Z80;

namespace PERQemu.IO.SerialDevices
{
    /// <summary>
    /// Provide a single SIO device that can split the traffic on two channels
    /// to two separate devices.  On PERQ this means the "speech" device uses
    /// the transmit half of a channel to stream bytes through the MC3417 for
    /// audio output, while the Kriz tablet uses the receive half to report
    /// mouse updates.  Should handle both the original IOB implementation and
    /// the later EIO as well.
    /// </summary>
    public sealed class SerialMux : SerialDevice
    {
        public SerialMux(Z80System sys) : base(sys)
        {
            _name = "SIO mux device";
            Log.Info(Category.SIO, "Created {0} mux device", IOBoard.Name);
        }

        public override bool ReadReady => _rxDevice?.ReadReady ?? false;
        public override ulong ReceiveRate => _rxDevice?.ReceiveRate ?? 0;

        public override bool WriteReady => _txDevice?.WriteReady ?? false;
        public override ulong TransmitRate => _txDevice?.TransmitRate ?? 0;

        public override ulong PollRate => _txDevice?.PollRate ?? 0;
     
        public void AttachRxDevice(SerialDevice rxDev)
        {
            _rxDevice = rxDev;
            Log.Info(Category.SIO, "Attached Rx device {0}", rxDev);
        }

        public void AttachTxDevice(SerialDevice txDev)
        {
            _txDevice = txDev;
            Log.Info(Category.SIO, "Attached Tx device {0}", txDev);
        }

        public override void Reset()
        {
            _rxDevice?.Reset();
            _txDevice?.Reset();
        }

        public override bool Poll()
        {
            return _txDevice?.Poll() ?? false;
        }

        public override byte Receive()
        {
            return _rxDevice?.Receive() ?? 0;
        }

        public override void Transmit(byte value)
        {
            _txDevice?.Transmit(value);
        }

        public override void Status()
        {
            _rxDevice?.Status();
            _txDevice?.Status();
        }

        public override void Telemetry(bool enable, ref StreamWriter file)
        {
            // Pass through to speech (no need for Kriz?)
            _txDevice?.Telemetry(enable, ref file);
        }

        SerialDevice _rxDevice;       // KrizTablet for mouse input
        SerialDevice _txDevice;       // MC3417 for audio output
    }
}
