//
// SerialDevice.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using PERQemu.IO.Ports;

namespace PERQemu.IO.SerialDevices
{
    /// <summary>
    /// Provide the basis for implementation of serial devices that can exchange
    /// data between a real/host device and the SIO chip, and receive baud rate
    /// updates from the timer chip.  Replaces the ISIODevice interface.
    /// </summary>
    public abstract class SerialDevice : ICTCDevice
    {
        protected SerialDevice(Z80System sys)
        {
            _system = sys;
            _scheduler = _system.Scheduler;

            _name = "Generic serial device";
            _portName = string.Empty;
            _isOpen = false;

            _txRate = 0;
            _rxRate = 0;
            _pollRate = 0;
        }

        protected SerialDevice(Z80System sys, string port) : this(sys)
        {
            _portName = port;
        }

        public virtual string Name
        {
            get { return _name; }
            protected set { _name = value; }
        }

        public virtual string Port
        {
            get { return _portName; }
            set { _portName = value; }
        }

        public virtual int BaudRate
        {
            get { return 9600; }
        }

        public virtual int DataBits
        {
            get { return 8; }
            set { Log.Detail(Category.SIO, "Ignoring data bits set to {0}", value); }
        }

        public virtual Parity Parity
        {
            get { return Parity.None; }
            set { Log.Detail(Category.SIO, "Ignoring parity set to {0}", value); }
        }

        public virtual StopBits StopBits
        {
            get { return StopBits.One; }
            set { Log.Detail(Category.SIO, "Ignoring stop bits set to {0}", value); }
        }

        public virtual Handshake FlowControl
        {
            get { return Handshake.None; }
            set { Log.Detail(Category.SIO, "Ignoring flow control {0}", value); }
        }

        public virtual SerialOptions Options
        {
            get { return SerialOptions.None; }
            set { Log.Detail(Category.SIO, "Ignoring serial option {0}", value); }
        }

        public virtual bool DTR
        {
            get { return false; }
            set { Log.Detail(Category.SIO, "Ignoring DTR pin set to {0}", value); }
        }

        public virtual bool RTS
        {
            get { return false; }
            set { Log.Detail(Category.SIO, "Ignoring RTS pin set to {0}", value); }
        }

        public virtual bool DCD => false;
        public virtual bool CTS => false;
        public virtual bool DSR => false;

        public virtual bool IsOpen => _isOpen;

        public virtual bool WriteReady => false;
        public virtual bool ReadReady => false;

        public virtual ulong TransmitRate => _txRate;
        public virtual ulong ReceiveRate => _rxRate;
        public virtual ulong PollRate => _pollRate;

        //
        // ICTCDevice implementation
        //

        public virtual void NotifyRateChange(int chan, int newRate)
        {
            Log.Detail(Category.SIO, "Clock rate change to {0} ignored for {1}", newRate, Name);
        }

        //
        // SerialDevice
        //

        public virtual void Reset()
        {
            Log.Debug(Category.SIO, "{0} reset", _name);
        }

        public virtual void Open()
        {
            _isOpen = true;
        }

        public virtual bool Poll()
        {
            Log.Info(Category.SIO, "Polling on {0} ignored", Name);
            return false;
        }

        public virtual byte Receive()
        {
            throw new NotImplementedException($"Receive on {Name}");
        }

        public virtual void Transmit(byte value)
        {
            throw new NotImplementedException($"Transmit on {Name}");
        }

        public virtual void TransmitBreak(bool enable)
        {
            throw new NotImplementedException($"TransmitBreak on {Name}");
        }

        public virtual void Close()
        {
            _isOpen = false;
        }

        public virtual void NotifySettingsChange(SerialSettings settings)
        {
            Log.Detail(Category.SIO, "Settings change notification ignored for {0}", Name);
        }

        public virtual void Status()
        {
            Console.WriteLine($"No status available for this {Name}.");
        }

        protected Z80System _system;
        protected Scheduler _scheduler;

        protected bool _isOpen;
        protected string _name;
        protected string _portName;

        protected ulong _txRate;
        protected ulong _rxRate;
        protected ulong _pollRate;
    }
}
