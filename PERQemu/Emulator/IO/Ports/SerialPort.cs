//
// SerialPort.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using System.IO;
using System.Collections.Generic;

using Microsoft.Win32;

namespace PERQemu.IO.Ports
{
    /// <summary>
    /// Provides a stripped-down, non-compatible API for PERQemu to replace the
    /// utterly broken System.IO.Ports.SerialPort.  Uses the underlying Stream
    /// and DLL calls but strictly non-blocking and polled (no events).
    /// </summary>
    public class SerialPort
    {
        public SerialPort(string portName, SerialSettings settings)
        {
            _portName = portName;
            _settings = settings;

            // Assume some defaults
            _readBufSize = DefaultReadBufferSize;
            _writeBufSize = DefaultWriteBufferSize;
            _signals = SerialSignal.None;
        }

        //
        // Properties that can only be changed when the port is closed:
        //

        public string PortName
        {
            get { return _portName; }

            set
            {
                if (value == null)
                    throw new ArgumentNullException();

                if (value.Length == 0 || value.StartsWith("\\\\", StringComparison.Ordinal))
                    throw new ArgumentException("Port name is empty or illegal");

                if (_isOpen)
                    throw new InvalidOperationException("Port name cannot be set while port is open");

                _portName = value;
            }
        }

        public int ReadBufferSize
        {
            get { return _readBufSize; }
            set { if (!_isOpen) _readBufSize = value; }
        }

        public int WriteBufferSize
        {
            get { return _writeBufSize; }
            set { if (!_isOpen) _writeBufSize = value; }
        }

        //
        // Properties that can be set before or after the port is open?
        //

        public int BaudRate
        {
            get { return _settings.BaudRate; }

            set
            {
                if (value <= 0) throw new ArgumentOutOfRangeException();

                if (value != _settings.BaudRate)
                {
                    _settings.BaudRate = value;
                    if (_isOpen) _stream.SetAttributes(_settings);
                }
            }
        }

        public int DataBits
        {
            get { return _settings.DataBits; }

            set
            {
                if (value < 5 || value > 8)
                    throw new ArgumentOutOfRangeException();

                if (value != _settings.DataBits)
                {
                    _settings.DataBits = value;
                    if (_isOpen) _stream.SetAttributes(_settings);
                }
            }
        }

        public Parity Parity
        {
            get { return _settings.Parity; }

            set
            {
                if (value < Parity.None || value > Parity.Even)
                    throw new ArgumentOutOfRangeException();

                if (value != _settings.Parity)
                {
                    _settings.Parity = value;
                    if (_isOpen) _stream.SetAttributes(_settings);
                }
            }
        }

        public StopBits StopBits
        {
            get { return _settings.StopBits; }

            set
            {
                if (value < StopBits.One || value > StopBits.OnePointFive)
                    throw new ArgumentOutOfRangeException();

                if (value != _settings.StopBits)
                {
                    _settings.StopBits = value;
                    if (_isOpen) _stream.SetAttributes(_settings);
                }
            }
        }

        public Handshake FlowControl
        {
            get { return _settings.FlowControl; }

            set
            {
                if (value < Handshake.None || value > Handshake.Both)
                    throw new ArgumentOutOfRangeException();

                if (value != _settings.FlowControl)
                {
                    _settings.FlowControl = value;
                    if (_isOpen) _stream.SetAttributes(_settings);
                }
            }
        }

        //
        // Operations
        //

        public bool IsOpen => _isOpen;

        public int BytesToRead
        {
            get { return _isOpen ? _stream.BytesToRead : 0; }
        }

        public int BytesToWrite
        {
            get { return _isOpen ? _stream.BytesToWrite : 0; }
        }

        public bool CarrierDetect
        {
            get { return _isOpen && ((_signals & SerialSignal.DCD) != 0); }
        }

        public bool ClearToSend
        {
            get { return _isOpen && ((_signals & SerialSignal.CTS) != 0); }
        }

        public bool DataSetReady
        {
            get { return _isOpen && ((_signals & SerialSignal.DSR) != 0); }
        }

        public bool DataTerminalReady
        {
            get { return _isOpen ? (_signals & SerialSignal.DTR) != 0 : _dtrEnable; }

            set
            {
                if (_isOpen && value != _dtrEnable)
                    _stream.SetSignal(SerialSignal.DTR, value);

                _dtrEnable = value;
            }
        }

        public bool RequestToSend
        {
            get { return _isOpen ? (_signals & SerialSignal.RTS) != 0 : _rtsEnable; }

            set
            {
                if (_isOpen && value != _rtsEnable)
                    _stream.SetSignal(SerialSignal.RTS, value);

                _rtsEnable = value;
            }
        }

        public bool BreakState
        {
            get { return _breakState; }

            set
            {
                if (value == _breakState) return;  // No change

                _breakState = value;
                if (_isOpen) _stream.SetBreakState(_breakState);
            }
        }

        public int ReadTimeout
        {
            get { return _isOpen ? _stream.ReadTimeout : InfiniteTimeout; }
            set { if (_isOpen) _stream.ReadTimeout = value; }
        }

        public int WriteTimeout
        {
            get { return _isOpen ? _stream.WriteTimeout : InfiniteTimeout; }
            set { if (_isOpen) _stream.WriteTimeout = value; }
        }

        public int ReadPending
        {
            get { return _isOpen ? _rxBuffer.Count : 0; }
        }

        public int WritePending
        {
            get { return _isOpen ? _txBuffer.Count : 0; }
        }

        //
        // Debugging and status
        //

        public string SignalStatus
        {
            get
            {
                if (!_isOpen) return "[Port closed]";

                return string.Format("DCD: {0}  DTR: {1}  DSR: {2}  CTS: {3}  RTS: {4}",
                                     CarrierDetect, DataTerminalReady, DataSetReady,
                                     ClearToSend, RequestToSend);
            }
        }

        public string StreamStatus
        {
            get
            {
                return string.Format("Rx: {0}/{1} of {2}  Tx: {3}/{4} of {5}",
                                     BytesToRead, ReadPending, ReadBufferSize,
                                     BytesToWrite, WritePending, WriteBufferSize);
            }
        }

        /// <summary>
        /// Open the stream (which opens the physical port).  No-op if already open.
        /// Attempts to set the requested port parameters; catches many different
        /// exceptions if anything fails and sets _isOpen accordingly.
        /// </summary>
        public void Open()
        {
            if (_isOpen) return;

            try
            {
                _rxBuffer = new Queue<byte>(_readBufSize);
                _txBuffer = new Queue<byte>(_writeBufSize);
                _devBuffer = new byte[16];

                if (PERQemu.HostIsUnix)
                    _stream = new UnixSerialStream(_portName, _settings, _dtrEnable, _rtsEnable, 0, 0);
                else
                    _stream = new WinSerialStream(_portName, _settings, _dtrEnable, _rtsEnable, 0, 0);

                _stream.RegisterErrorDelegate(_errorHandler, _portID);
                _isOpen = true;
            }
            catch (Exception e)
            {
                Log.Warn(Category.RS232, "Failed to open port: {0}", e.Message);
            }
        }

        /// <summary>
        /// Poll checks to see if there are bytes available to read on the stream,
        /// and if so pulls as many as will fit into the Rx buffer.  If there are
        /// bytes queued for output and the stream has room, the Tx buffer is written
        /// to the stream.  It then caches and checks the errors/status/pin states
        /// and signals the appropriate event(s) if a change is detected??
        /// </summary>
        /// <remarks>
        /// This should be called at a reasonable rate to make sure the OS/kernel/device
        /// buffers aren't overrun, but not so often that the CPU is overly burdened by
        /// an excessive poll rate.
        /// </remarks>
        public bool Poll()
        {
            if (!_isOpen) return false;

            var count = 0;

            if (_rxBuffer.Count < _readBufSize)
            {
                // Count = min of data pending, buffer space, buffer size
                var bytesAvail = _stream.BytesToRead;
                var spaceAvail = _readBufSize - _rxBuffer.Count;
                count = Math.Min(bytesAvail, spaceAvail);
                count = Conversion.Clamp(count, 0, 16);

                if (count > 0)
                {
                    var bytesRead = _stream.Read(_devBuffer, count);

                    if (bytesRead < count)
                        Log.Info(Category.RS232, "Read {0} bytes, expected {1}!", bytesRead, count);

                    // Queue up what we did get
                    for (var i = 0; i < bytesRead; i++)
                        _rxBuffer.Enqueue(_devBuffer[i]);
                }
            }

            while (_txBuffer.Count > 0)
            {
                _devBuffer[0] = _txBuffer.Peek();
                if (_stream.Write(_devBuffer, 1) != 1)
                {
                    Log.Info(Category.RS232, "Failed to write, device buffer full?");
                    break;
                }

                _txBuffer.Dequeue();
            }

            // Cache the latest signals/errors
            _signals = _stream.GetSignals();

            return true;
        }

        /// <summary>
        /// Return the next byte from the receive buffer, or 0 if none available.
        /// </summary>
        public int ReadByte()
        {
            if (!_isOpen) return -1;

            if (_rxBuffer.Count == 0) return 0;

            return _rxBuffer.Dequeue();
        }

        /// <summary>
        /// Write a byte to the transmit buffer.  Returns 0 if no room available.
        /// </summary>
        public int WriteByte(byte val)
        {
            if (!_isOpen) return -1;

            if (_txBuffer.Count > _writeBufSize) return 0;

            _txBuffer.Enqueue(val);
            return 1;
        }

        /// <summary>
        /// Close the stream.  Flushes any pending unread or unwritten data!
        /// </summary>
        public void Close()
        {
            if (!_isOpen) return;

            _rxBuffer.Clear();
            _txBuffer.Clear();

            _stream.Dispose();
            _stream = null;

            _isOpen = false;
        }

        // These may not be needed in the emulator...
        public void DiscardInBuffer()
        {
            if (_isOpen) _rxBuffer.Clear();
        }

        public void DiscardOutBuffer()
        {
            if (_isOpen) _txBuffer.Clear();
        }

        /// <summary>
        /// Save the error delegate info.  Due to lazy initialization, this gets
        /// applied only when the device is opened and the stream instantiated.
        /// </summary>
        public void SetErrorHandler(SerialErrorDelegate handler, char port)
        {
            _errorHandler = handler;
            _portID = port;
        }

        /// <summary>
        /// Gets the names of serial port devices for this host.  For PERQemu,
        /// will probably leave this in the Configurator/Settings, not here.
        /// </summary>
        public static string[] GetPortNames()
        {
            List<string> ports = new List<string>();

            // Are we on "Unix?"
            if (PERQemu.HostIsUnix)
            {
                // For now, just add the Linux-styled names.  I'd LOVE to add a
                // Solaris port someday if I could ever afford a high-end SPARC
                // that can run PERQemu at reasonable rates!
                ports.AddRange(Directory.GetFiles("/dev", "ttyS*"));
                ports.AddRange(Directory.GetFiles("/dev", "ttyUSB*"));
                ports.AddRange(Directory.GetFiles("/dev", "ttyACM*"));
            }
            else
            {
                // From the Mono implementation; assume it works on any version
                // of Windows that can run PERQemu...
                using (RegistryKey subkey = Registry.LocalMachine.OpenSubKey("HARDWARE\\DEVICEMAP\\SERIALCOMM"))
                {
                    if (subkey != null)
                    {
                        string[] names = subkey.GetValueNames();
                        foreach (string value in names)
                        {
                            string port = subkey.GetValue(value, "").ToString();
                            if (port != "")
                                ports.Add(port);
                        }
                    }
                }
            }

            return ports.ToArray();
        }


        //
        // Defaults
        //
        public const int InfiniteTimeout = -1;

        const int DefaultReadBufferSize = 1024;
        const int DefaultWriteBufferSize = 512;

        int _readBufSize = DefaultReadBufferSize;
        int _writeBufSize = DefaultWriteBufferSize;

        Queue<byte> _rxBuffer;
        Queue<byte> _txBuffer;

        byte[] _devBuffer;

        SerialSignal _signals;
        ISerialStream _stream;

        SerialSettings _settings;
        SerialErrorDelegate _errorHandler;

        string _portName;
        char _portID;

        bool _isOpen;
        bool _breakState;
        bool _dtrEnable;
        bool _rtsEnable;
    }
}
