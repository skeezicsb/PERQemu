//
// UnixSerialStream.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.  It is a rewrite of the Mono implementation:
//
//      System.IO.Ports.SerialPortStream.cs
//
//      Authors:
//          Chris Toshok (toshok@ximian.com)
//          Carlos Alberto Cortez (calberto.cortez@gmail.com)
//
//      (c) Copyright 2006 Novell, Inc. (http://www.novell.com)
//
//      Slightly modified by Konrad M. Kruczynski (added baud rate value checking)
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

using System;
using System.IO;
using System.Runtime.InteropServices;

namespace PERQemu.IO.Ports
{
    public class UnixSerialStream : Stream, ISerialStream, IDisposable
    {
        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int open_serial(string portName);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern bool poll_serial(int fd, out int error, int timeout);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int read_serial(int fd, byte[] buffer, int offset, int count);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int write_serial(int fd, byte[] buffer, int offset, int count, int timeout);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern bool set_attributes(int fd, int baudRate, Parity parity, int dataBits, StopBits stopBits, Handshake handshake);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int set_signal(int fd, SerialSignal signal, bool value);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern SerialSignal get_signals(int fd, out int error);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int get_bytes_in_buffer(int fd, int input);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int discard_buffer(int fd, bool inputBuffer);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int breakprop(int fd);

        [DllImport("MonoPosixHelper")]
        static extern bool is_baud_rate_legal(int baudRate);

        [DllImport("MonoPosixHelper", SetLastError = true)]
        static extern int close_serial(int fd);

        [DllImport("libc")]
        static extern IntPtr strerror(int errnum);


        public UnixSerialStream(string portName, SerialSettings settings,
                                bool dtrEnable, bool rtsEnable,
                                int readTimeout, int writeTimeout)
        {
            // Open the device
            _fd = open_serial(portName);

            if (_fd == -1)
                ThrowIOException();

            // Check, set low level attributes
            if (!is_baud_rate_legal(settings.BaudRate))
            {
                throw new ArgumentOutOfRangeException(nameof(settings.BaudRate),
                              $"Baud rate {settings.BaudRate} is not supported on this platform.");
            }

            if (!set_attributes(_fd, settings.BaudRate, settings.Parity,
                                     settings.DataBits, settings.StopBits,
                                     settings.FlowControl))
                ThrowIOException();

            // Set timeouts
            _readTimeout = readTimeout;
            _writeTimeout = writeTimeout;

            // Set DTR
            SetSignal(SerialSignal.DTR, dtrEnable);

            // If CTS/RTS handshaking is NOT specified, set RTS
            if (settings.FlowControl != Handshake.RTSCTS &&
                settings.FlowControl != Handshake.Both)
                SetSignal(SerialSignal.RTS, rtsEnable);
        }


        ~UnixSerialStream()
        {
            try { Dispose(false); }
            catch (IOException) { }
        }

        //
        // Public properties
        //

        public override bool CanSeek => false;
        public override bool CanRead => true;
        public override bool CanWrite => true;
        public override bool CanTimeout => true;

        public override int ReadTimeout
        {
            get { return _readTimeout; }

            set
            {
                if (value < SerialPort.InfiniteTimeout)
                    throw new ArgumentOutOfRangeException();

                _readTimeout = value;
            }
        }

        public override int WriteTimeout
        {
            get { return _writeTimeout; }

            set
            {
                if (value < SerialPort.InfiniteTimeout)
                    throw new ArgumentOutOfRangeException();

                _writeTimeout = value;
            }
        }

        public int BytesToRead => get_bytes_in_buffer(_fd, 1);
        public int BytesToWrite => get_bytes_in_buffer(_fd, 0);

        //
        // Not supported on a serial stream:
        //

        public override long Length
        {
            get { throw new NotSupportedException(); }
        }

        public override long Position
        {
            get { throw new NotSupportedException(); }
            set { throw new NotSupportedException(); }
        }

        public override void SetLength(long value)
        {
            throw new NotSupportedException();
        }

        public override long Seek(long offset, SeekOrigin origin)
        {
            throw new NotSupportedException();
        }

        //
        // Required, but not supported on THIS implementation:
        //

        public override int Read(byte[] buffer, int offset, int count)
        {
            throw new NotSupportedException();
        }

        public override void Write(byte[] buffer, int offset, int count)
        {
            throw new NotSupportedException();
        }

        //
        // Public methods
        //

        /// <summary>
        /// Read up to 'count' bytes from the port into the specified buffer and
        /// returns the actual number read, or 0 if no data available.  Will throw
        /// an IOException only if an actual error occurs, or a TimeoutException
        /// only if a non-zero read timeout is set and 'count' bytes can't be read.
        /// </summary>
        public int Read(byte[] buffer, int count)
        {
            CheckDisposed();

            if (buffer == null)
                throw new ArgumentNullException(nameof(buffer));

            if (count < 0 || count > buffer.Length)
                throw new ArgumentOutOfRangeException(nameof(count));

            int error;
            bool bytesAvailable = poll_serial(_fd, out error, _readTimeout);

            // An actual error
            if (error == -1) ThrowIOException();

            // Or an actual timeout?
            if (!bytesAvailable && _readTimeout != 0)
                throw new TimeoutException();

            // Otherwise, return the read or 0
            return bytesAvailable ? read_serial(_fd, buffer, 0, count) : 0;
        }

        /// <summary>
        /// Write 'count' bytes to the port from the specified buffer.  Returns the
        /// syscall result, NOT an exception.  Will only throw a TimeoutException
        /// if the write timeout is non-zero and the requested number of bytes
        /// can't be written.
        /// </summary>
        public int Write(byte[] buffer, int count)
        {
            CheckDisposed();

            if (buffer == null)
                throw new ArgumentNullException(nameof(buffer));

            if (count < 0 || count > buffer.Length)
                throw new ArgumentOutOfRangeException(nameof(count));

            // On Unix: write_serial returns 0 if all the bytes written; -1 is
            // either a poll error or a write error (no way to tell which with
            // the current implementation).
            var result = write_serial(_fd, buffer, 0, count, _writeTimeout);

            // So: if -1 and a timeout is set, assume that's the cause?
            if (result < 0 && _writeTimeout != 0)
                throw new TimeoutException();

            // Otherwise: if result is 0 return count (to match the Windows side)
            return result == 0 ? count : result;
        }

        /// <summary>
        /// Sets the operational port attributes: baud rate, parity, data and stop
        /// bits, and handshaking strategy.  Throws an IOException if any of them
        /// are invalid.
        /// </summary>
        public void SetAttributes(SerialSettings settings)
        {
            if (!set_attributes(_fd, settings.BaudRate, settings.Parity,
                                     settings.DataBits, settings.StopBits,
                                     settings.FlowControl))
                ThrowIOException();
        }

        /// <summary>
        /// Sends an RS232 "break" signal.  This doesn't set/clear the state as it
        /// does on Windows; tcsendbreak() sends a fixed (on MacOS X, anyway) .4sec
        /// break down the line and returns.  Synchronously?  Will the emulator hang
        /// for nearly half a second if you use this?  TBD.
        /// </summary>
        /// <remarks>
        /// There really is NO reason why this should throw an exception; for PERQemu
        /// we just fire and forget.  But like anything that touches the file handle,
        /// throw in case the USB cable got yanked and the underlying port disappeared.
        /// Great API, everyone.  Really cool.
        /// </remarks>
        public void SetBreakState(bool value)
        {
            if (value)
                if (breakprop(_fd) == -1)
                    ThrowIOException();
        }

        /// <summary>
        /// Set the state of A signal pin - singular.  The underlying set_signal()
        /// wrapper probably doesn't behave well if you OR together multiple pins.
        /// But there are only TWO pins that can be set anyway: DTR and RTS!  This
        /// is very silly.
        /// </summary>
        public void SetSignal(SerialSignal signal, bool value)
        {
            if (signal != SerialSignal.DTR && signal != SerialSignal.RTS)
                throw new ArgumentException("invalid signal");

            if (set_signal(_fd, signal, value) == -1)
                ThrowIOException();
        }

        /// <summary>
        /// Gets the current state of all the signal pins (input and output).
        /// </summary>
        public SerialSignal GetSignals()
        {
            int error;
            var signals = get_signals(_fd, out error);

            if (error == -1)
                ThrowIOException();

            return signals;
        }

        /// <summary>
        /// Discards the characters in the OS/device receive buffer.  Not clear
        /// what happens if flow control is active.
        /// </summary>
        public void DiscardInBuffer()
        {
            if (discard_buffer(_fd, true) != 0)
                Console.WriteLine("DiscardInBuffer reported failure!");
        }

        /// <summary>
        /// Discards the characters in the OS/device transmit buffer.
        /// </summary>
        public void DiscardOutBuffer()
        {
            if (discard_buffer(_fd, false) != 0)
                Console.WriteLine("DiscardOutBuffer reported failure!");
        }

        /// <summary>
        /// Flush the data in both device buffers.
        /// </summary>
        public override void Flush()
        {
            CheckDisposed();
            DiscardInBuffer();
            DiscardOutBuffer();
        }

        //
        // Private methods
        //

        protected override void Dispose(bool disposing)
        {
            if (_isDisposed) return;

            _isDisposed = true;
            if (close_serial(_fd) != 0)
                ThrowIOException();
        }

        void IDisposable.Dispose()
        {
            Dispose(true);
            GC.SuppressFinalize(this);
        }

        void CheckDisposed()
        {
            if (_isDisposed)
                throw new ObjectDisposedException(GetType().FullName);
        }

        static void ThrowIOException()
        {
            int errnum = Marshal.GetLastWin32Error();
            string error_message = Marshal.PtrToStringAnsi(strerror(errnum));

            throw new IOException(error_message);
        }


        int _fd;
        int _readTimeout;
        int _writeTimeout;
        bool _isDisposed;
    }
}
