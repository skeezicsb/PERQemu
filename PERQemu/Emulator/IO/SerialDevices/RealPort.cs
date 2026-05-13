//
// RealPort.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
    /// "RealPort" encapsulates a PERQemu.IO.Ports.SerialPort to exchange data
    /// between the Z80 SIO operating in "virtual time" and a real serial port
    /// on the host (or an equivalent USB to serial bridge adapter/cable).
    /// </summary>
    /// <remarks>
    /// See the file Docs/SerialPorts.txt for the gruesome implementation details.
    /// </remarks>
    public class RealPort : SerialDevice
    {
        public RealPort(Z80System sys, string id, string portName, SerialSettings portSet) : base(sys, portName)
        {
            _name = id;                                 // Distinguish RS-232 "A" and "B"
            _host = portSet;                            // User's host-side configuration
            _perq = SerialSettings.Defaults;            // PERQ's view (mostly ignored)
            _port = new SerialPort(portName, portSet);  // The host's actual port

            // Tune some things that should only need setting once?  These may not
            // even be exposed as user-settable once debugging and performance is
            // characterized against the full range of baud rates.
            _port.WriteBufferSize = 128;
            _port.ReadBufferSize = 256;
            _port.ReadTimeout = 0;
        }


        /// <summary>
        /// Check that the port is open by comparing our expected state (_isOpen)
        /// with the actual port status (_port.IsOpen) because the bloody device
        /// may suddenly disappear and throw IOExceptions if you unplug it.  Ugh.
        /// </summary>
        public override bool IsOpen
        {
            get
            {
                _isOpen = _port?.IsOpen ?? false;
                return _isOpen;
            }
        }

        public override int BaudRate
        {
            get { return _perq.BaudRate; }
        }

        public override int DataBits
        {
            get { return _perq.DataBits; }
            set { _perq.DataBits = value; }
        }

        public override Parity Parity
        {
            get { return _perq.Parity; }
            set { _perq.Parity = value; }
        }

        public override StopBits StopBits
        {
            get { return _perq.StopBits; }
            set { _perq.StopBits = value; }
        }

        //
        // Virtual I/O pins
        //

        /// <summary>
        /// DTR is set/cleared by register write when the SIO is enabled.  We
        /// pass it straight through to the physical port.
        /// </summary>
        public override bool DTR
        {
            get { return _port.DataTerminalReady; }
            set { _port.DataTerminalReady = value; }
        }

        /// <summary>
        /// The virtual RTS pin is set based on the register bit programming or
        /// the state of the Tx FIFO (depending on Sync Mode/Auto Enables control
        /// bits).  This is decoupled from the physical port, which will assert
        /// RTS/CTS if set (by the user) to do "hardware flow control," but the
        /// virtual machine doesn't touch the hardware directly.
        /// </summary>
        public override bool RTS
        {
            get { return _rts; }
            set { _rts = value; }
        }

        /// <summary>
        /// Virtual CTS is set based on the physical port's read buffer state.
        /// </summary>
        public override bool CTS => _port.ReadPending < _port.ReadBufferSize;

        /// <summary>
        /// Virtual DCD is passed through from the physical port.  But because
        /// the crazy SIO uses it as the Rx enable in AutoEnables mode, we allow
        /// a software override to avoid the chicken & egg problem when talking
        /// to uh, an actual modem.  <facepalm />
        /// </summary>
        public override bool DCD => (_host.Options == SerialOptions.DCDFollowDSR ? DSR :
                                     _host.Options == SerialOptions.DCDForceOn ? true :
                                     _port.CarrierDetect);

        /// <summary>
        /// Virtual DSR is passed through but the Z80 SIO doesn't actually use or
        /// report this pin.  However, to simplify cabling/interfacing to physical
        /// devices, we can use it to enable DCD if configured in Settings.
        /// </summary>
        public override bool DSR => _port.DataSetReady;


        public override bool WriteReady => _port.WritePending < _port.WriteBufferSize;
        public override bool ReadReady => _port.ReadPending > 0;


        public string SignalStatus
        {
            get
            {
                if (!_isOpen) return "[Port closed]";

                return $"DCD: {DCD}  DTR: {DTR}  DSR: {DSR}  CTS: {CTS}  RTS: {_rts}";
            }
        }

        public override void Reset()
        {
            // Flush the local buffers
            _port.DiscardInBuffer();
            _port.DiscardOutBuffer();

            // Reset to PERQ defaults
            _perq = SerialSettings.Defaults;

            // Adjust the pacing rates for scheduling characters to the PERQ
            _txRate = _rxRate = Conversion.BaudRateToNsec(_perq.BaudRate);

            // Adjust the polling rate of the physical interface
            _pollRate = (_rxRate * 16);

            Log.Info(Category.RS232, "{0} physical device reset", _name);
        }

        /// <summary>
        /// Open the host device and apply the user-configured Settings.  This
        /// runs at any rate they configure, while the PERQ side emulates/limits
        /// the data flow to the PERQ's restricted range of speeds.
        /// </summary>
        public override void Open()
        {
            // Most of the port's characteristics only take effect before calling
            // Open(); if the user initiates a change, close and reopen to apply
            // new settings.

            if (IsOpen) Close();

            _port.PortName = _portName;
            _port.BaudRate = _host.BaudRate;
            _port.DataBits = _host.DataBits;
            _port.Parity = _host.Parity;
            _port.StopBits = _host.StopBits;
            _port.FlowControl = _host.FlowControl;

            _port.Open();

            _isOpen = _port.IsOpen;
            Log.Info(Category.RS232, "{0} is {1}", _name, _isOpen ? "now open" : "still closed!");
        }

        /// <summary>
        /// If the port is open, call its Poll routine to move data between the
        /// virtual machine and the host.
        /// </summary>
        public override bool Poll()
        {
            Log.Verbose(Category.RS232, "Polling {0}", _name);
            return _port.Poll();
        }

        /// <summary>
        /// Stop any active polling and close the host port.
        /// </summary>
        public override void Close()
        {
            _port.Close();

            Log.Info(Category.RS232, "{0} is {1}", _name, IsOpen ? "still open!" : "now closed");
        }

        /// <summary>
        /// Compute new baud rate from the timer tick rate provided by the CTC.
        /// </summary>
        public override void NotifyRateChange(int chan, int newRate)
        {
            var prescale = _system.IsEIO ? 1 : 16;
            var checkRate = Conversion.TimerCountToBaudRate(newRate, prescale);

            // This is highly unlikely, but alert if it happens
            if (checkRate == 0)
            {
                Log.Warn(Category.RS232, "{0} bad baud rate {1} from the PERQ!", _name, newRate);
                return;
            }

            _perq.BaudRate = checkRate;

            // On EIO, ports A & B support separate Tx/Rx baud rates
            if (_system.IsEIO)
            {
                if (chan == 0)
                    _rxRate = Conversion.BaudRateToNsec(_perq.BaudRate);
                else if (chan == 2)
                    _txRate = Conversion.BaudRateToNsec(_perq.BaudRate);
                else
                    throw new InvalidOperationException($"RS232 baud rate change from CTC chan {chan}?");

                Log.Info(Category.RS232, "{0} {1} baud rate changed to {2}", _name,
                                         (chan == 0) ? "receive" : "transmit", _perq.BaudRate);
                return;
            }

            // On IOB/CIO, no split rates
            _txRate = _rxRate = Conversion.BaudRateToNsec(_perq.BaudRate);

            Log.Info(Category.RS232, "{0} baud rate changed to {1}", _name, _perq.BaudRate);
        }

        /// <summary>
        /// If a byte is available from the host, send it to the PERQ.
        /// </summary>
        public override byte Receive()
        {
            var result = _port.ReadByte();

            if (result < 0)
            {
                Log.Warn(Category.RS232, "{0} read failed: {1}", _name,
                                         result < 0 ? "device not open!" : "buffer empty");
                return 0;
            }

            Log.Debug(Category.RS232, "Read byte {0:x2} ({1} in input queue)", result, _port.ReadPending);

            return (byte)result;
        }

        /// <summary>
        /// Write a byte from the PERQ to the physical port.
        /// </summary>
        public override void Transmit(byte value)
        {
            var result = _port.WriteByte(value);

            if (result < 1)
            {
                Log.Warn(Category.RS232, "{0} write ({1:x2}) failed: {2}", _name, value,
                                         result < 0 ? "device not open!" : "buffer full");
                return;
            }

            Log.Debug(Category.RS232, "Wrote byte {0:x2} ({1} in output queue)", value, _port.WritePending);
        }

        /// <summary>
        /// Transmit a break.
        /// </summary>
        public override void TransmitBreak(bool enable)
        {
            _port.BreakState = enable;
        }

        /// <summary>
        /// Pass the error delegate down the chain.
        /// </summary>
        public void SetErrorHandler(SerialErrorDelegate handler, char port)
        {
            _port.SetErrorHandler(handler, port);
        }


        // Debugging
        public override void Status()
        {
            Console.WriteLine($"Serial {Name}:  device '{Port}', IsOpen: {IsOpen}");
            Console.WriteLine($"  Host settings: {_host}");
            Console.WriteLine($"  PERQ settings: {_perq}");

            Console.WriteLine("  Pacing: Rx {0}ms  Tx {1}ms  Break state: {2}",
                              _rxRate * Conversion.NsecToMsec,
                              _txRate * Conversion.NsecToMsec,
                              _port.BreakState);
            Console.WriteLine("  " + SignalStatus);
            Console.WriteLine("Physical state:");
            Console.WriteLine("  " + _port.SignalStatus);
            Console.WriteLine("  " + _port.StreamStatus);
        }

        // Virtual pins
        bool _rts;

        // Host side
        SerialPort _port;
        SerialSettings _host;

        // PERQ side
        SerialSettings _perq;
    }
}

