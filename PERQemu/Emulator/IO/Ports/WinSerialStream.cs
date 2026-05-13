//
// WinSerialStream.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.  It is a rewrite of the Mono implementation:
//
//      System.IO.Ports.WinSerialStream.cs
//
//      Authors:
//          Carlos Alberto Cortez (calberto.cortez@gmail.com)
//
//      (c) Copyright 2006 Novell, Inc. (http://www.novell.com)
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
using System.Threading;
using System.ComponentModel;

namespace PERQemu.IO.Ports
{
    public class WinSerialStream : Stream, ISerialStream, IDisposable
    {
        [DllImport("kernel32", SetLastError = true)]
        static extern int CreateFile(string port_name, uint desired_access,
                                     uint share_mode, uint security_attrs,
                                     uint creation, uint flags, uint template);

        [DllImport("kernel32", SetLastError = true)]
        static extern unsafe bool ReadFile(int handle, byte* buffer, int bytes_to_read,
                                           out int bytes_read, IntPtr overlapped);

        [DllImport("kernel32", SetLastError = true)]
        static extern unsafe bool WriteFile(int handle, byte* buffer, int bytes_to_write,
                                            out int bytes_written, IntPtr overlapped);

        [DllImport("kernel32", SetLastError = true)]
        static extern unsafe bool GetOverlappedResult(int handle, IntPtr overlapped,
                                                      ref int bytes_transfered, bool wait);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool SetupComm(int handle, int read_buffer_size, int write_buffer_size);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool PurgeComm(int handle, uint flags);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool SetCommTimeouts(int handle, Timeouts timeouts);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool GetCommState(int handle, [Out] DCB dcb);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool SetCommState(int handle, DCB dcb);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool ClearCommError(int handle, out uint errors, out CommStat stat);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool GetCommModemStatus(int handle, out uint flags);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool EscapeCommFunction(int handle, uint flags);

        [DllImport("kernel32", SetLastError = true)]
        static extern bool CloseHandle(int handle);


        public WinSerialStream(string portName, SerialSettings settings,
                               bool dtrEnable, bool rtsEnable,
                               int readTimeout, int writeTimeout)
        {
            // Open the device
            _handle = CreateFile(portName != null &&
                                !portName.StartsWith(@"\\.\", StringComparison.Ordinal)
                                ? @"\\.\" + portName : portName,
                                GenericRead | GenericWrite, 0, 0, OpenExisting,
                                FileFlagOverlapped, 0);

            if (_handle == -1)
                ReportIOError(portName);

            // Only allocate once (used in SetSignal)
            _dcb = new DCB();

            // Set port low level attributes
            SetAttributes(settings);

            // Set timeouts
            _readTimeout = readTimeout;
            _writeTimeout = writeTimeout;
            _timeouts = new Timeouts(readTimeout, writeTimeout);

            if (!SetCommTimeouts(_handle, _timeouts))
                ReportIOError(null);

            // Set DTR
            SetSignal(SerialSignal.DTR, dtrEnable);

            // If CTS/RTS handshaking is NOT specified, set RTS
            if (settings.FlowControl != Handshake.RTSCTS &&
                settings.FlowControl != Handshake.Both)
                SetSignal(SerialSignal.RTS, rtsEnable);

#pragma warning disable 618
            // Init overlapped structures
            NativeOverlapped wo = new NativeOverlapped();
            _writeEvent = new ManualResetEvent(false);
            wo.EventHandle = _writeEvent.Handle;
            _writeOverlapped = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(NativeOverlapped)));
            Marshal.StructureToPtr(wo, _writeOverlapped, true);

            NativeOverlapped ro = new NativeOverlapped();
            _readEvent = new ManualResetEvent(false);
            ro.EventHandle = _readEvent.Handle;
            _readOverlapped = Marshal.AllocHGlobal(Marshal.SizeOf(typeof(NativeOverlapped)));
            Marshal.StructureToPtr(ro, _readOverlapped, true);
#pragma warning restore 618

        }

        ~WinSerialStream()
        {
            Dispose(false);
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

                _timeouts.SetValues(value, _writeTimeout);

                if (!SetCommTimeouts(_handle, _timeouts))
                    ReportIOError(null);

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

                _timeouts.SetValues(_readTimeout, value);

                if (!SetCommTimeouts(_handle, _timeouts))
                    ReportIOError(null);

                _writeTimeout = value;
            }
        }

        public int BytesToRead
        {
            get
            {
                var stat = GetPortStatus();
                return (int)stat.BytesIn;
            }
        }

        public int BytesToWrite
        {
            get
            {
                var stat = GetPortStatus();
                return (int)stat.BytesOut;
            }
        }

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

        public override long Seek(long offset, SeekOrigin origin)
        {
            throw new NotSupportedException();
        }

        public override void SetLength(long value)
        {
            throw new NotSupportedException();
        }

        //
        // Required, but not supported on THIS wacky implementation:
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

        public int Read(byte[] buffer, int count)
        {
            CheckDisposed();

            if (buffer == null)
                throw new ArgumentNullException(nameof(buffer));

            if (count < 0 || count > buffer.Length)
                throw new ArgumentOutOfRangeException(nameof(count));

            int bytesRead;

            unsafe
            {
                fixed (byte* ptr = buffer)
                {
                    if (ReadFile(_handle, ptr, count, out bytesRead, _readOverlapped))
                        return bytesRead;

                    // Test for overlapped behavior
                    if (Marshal.GetLastWin32Error() != FileIOPending)
                        ReportIOError(null);

                    if (!GetOverlappedResult(_handle, _readOverlapped, ref bytesRead, true))
                        ReportIOError(null);
                }
            }

            // If we're not polling and didn't get any bytes, timeout
            if (bytesRead == 0 && _readTimeout != 0)
                throw new TimeoutException();

            return bytesRead;
        }

        public int Write(byte[] buffer, int count)
        {
            CheckDisposed();

            if (buffer == null)
                throw new ArgumentNullException(nameof(buffer));

            if (count < 0 || count > buffer.Length)
                throw new ArgumentOutOfRangeException(nameof(count));

            int bytesWritten = 0;

            unsafe
            {
                fixed (byte* ptr = buffer)
                {
                    if (WriteFile(_handle, ptr, count, out bytesWritten, _writeOverlapped))
                        return bytesWritten;

                    if (Marshal.GetLastWin32Error() != FileIOPending)
                        ReportIOError(null);

                    if (!GetOverlappedResult(_handle, _writeOverlapped, ref bytesWritten, true))
                        ReportIOError(null);
                }
            }

            // If the operation timed out, then we transfered fewer bytes than
            // requested.  But the original API (throwing an exception) gives us
            // NO useful information about how many, if any, actually did get out
            // in the time alloted.  What a mess.
            if (bytesWritten < count && _writeTimeout != 0)
                throw new TimeoutException();

            return bytesWritten;
        }

        public void SetAttributes(SerialSettings settings)
        {
            if (!GetCommState(_handle, _dcb))
                ReportIOError(null);

            _dcb.SetValues(settings.BaudRate, settings.Parity,
                          settings.DataBits, settings.StopBits,
                          settings.FlowControl);

            if (!SetCommState(_handle, _dcb))
                ReportIOError(null);
        }

        public void SetBreakState(bool value)
        {
            if (!EscapeCommFunction(_handle, value ? SetBreak : ClearBreak))
                ReportIOError(null);
        }

        public void SetSignal(SerialSignal signal, bool value)
        {
            uint flag;

            switch (signal)
            {
                case SerialSignal.RTS:
                    flag = value ? SetRts : ClearRts;

                    if (value)
                        _signals |= SerialSignal.RTS;
                    else
                        _signals &= ~(SerialSignal.RTS);
                    break;

                case SerialSignal.DTR:
                    flag = value ? SetDtr : ClearDtr;

                    if (value)
                        _signals |= SerialSignal.DTR;
                    else
                        _signals &= ~(SerialSignal.DTR);
                    break;

                default:
                    throw new ArgumentException("invalid signal");
            }

            if (!EscapeCommFunction(_handle, flag))
                ReportIOError(null);
        }

        public SerialSignal GetSignals()
        {
            const SerialSignal inputPins = SerialSignal.DCD | SerialSignal.CTS | SerialSignal.DSR;
            uint flags;

            if (!GetCommModemStatus(_handle, out flags))
                ReportIOError(null);

            // Mask off the input pins and update their states
            _signals &= ~(inputPins);

            if ((flags & RsldOn) != 0)
                _signals |= SerialSignal.DCD;

            if ((flags & CtsOn) != 0)
                _signals |= SerialSignal.CTS;

            if ((flags & DsrOn) != 0)
                _signals |= SerialSignal.DSR;

            return _signals;
        }

        public void DiscardInBuffer()
        {
            if (!PurgeComm(_handle, PurgeRxClear))
                Console.WriteLine("DiscardInBuffer reported failure!");
        }

        public void DiscardOutBuffer()
        {
            if (!PurgeComm(_handle, PurgeTxClear))
                Console.WriteLine("DiscardOutBuffer reported failure!");
        }

        public override void Flush()
        {
            CheckDisposed();
            SetBreakState(false);
            DiscardInBuffer();
            DiscardOutBuffer();
        }

        /// <summary>
        /// Register the error delegate to handle fatal exceptions.
        /// </summary>
        public void RegisterErrorDelegate(SerialErrorDelegate handler, char id)
        {
            _errorHandler = handler;
            _portID = id;

            Log.Info(Category.RS232, "Port {0} error handler set", id);
        }

        //
        // Private methods
        //

        CommStat GetPortStatus()
        {
            uint errors;
            CommStat stat;

            if (!ClearCommError(_handle, out errors, out stat))
                ReportIOError(null);

            if (errors != _errors)
            {
                // Log it?  Send an event?  Or simply implode?
                Console.WriteLine($"Error status changed: 0x{errors:x}");
                _errors = errors;
            }

            return stat;
        }

        protected override void Dispose(bool disposing)
        {
            if (_isDisposed) return;

            _isDisposed = true;
            CloseHandle(_handle);
            Marshal.FreeHGlobal(_writeOverlapped);
            Marshal.FreeHGlobal(_readOverlapped);
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

        void ReportIOError(string optional_arg)
        {
            int error = Marshal.GetLastWin32Error();
            string message;

            switch (error)
            {
                case 2:
                case 3:
                    message = "The port `" + optional_arg + "' does not exist.";
                    break;
                case 87:
                    message = "Parameter is incorrect.";
                    break;
                default:
                    // As fallback, we show the win32 error
                    message = new Win32Exception().Message;
                    break;
            }

            var handler = _errorHandler;

            if (handler == null)
                throw new IOException(message);

            handler.Invoke(_portID, message);
        }


        // Windows API Constants
        const uint GenericRead = 0x80000000;
        const uint GenericWrite = 0x40000000;
        const uint OpenExisting = 3;
        const uint FileFlagOverlapped = 0x40000000;
        const uint PurgeRxClear = 0x0008;
        const uint PurgeTxClear = 0x0004;
        const uint FileIOPending = 997;

        // Signal constants
        const uint SetRts = 3;
        const uint ClearRts = 4;
        const uint SetDtr = 5;
        const uint ClearDtr = 6;
        const uint SetBreak = 8;
        const uint ClearBreak = 9;
        const uint CtsOn = 0x0010;
        const uint DsrOn = 0x0020;
        const uint RsldOn = 0x0080;


        int _handle;
        int _readTimeout;
        int _writeTimeout;
        bool _isDisposed;
        uint _errors;

        DCB _dcb;
        SerialSignal _signals;
        IntPtr _readOverlapped;
        IntPtr _writeOverlapped;
        ManualResetEvent _readEvent;
        ManualResetEvent _writeEvent;
        Timeouts _timeouts;

        static char _portID;
        static SerialErrorDelegate _errorHandler;
    }


    [StructLayout(LayoutKind.Sequential)]
    class DCB
    {
        public int dcb_length;
        public int baud_rate;
        public int flags;
        public short w_reserved;
        public short xon_lim;
        public short xoff_lim;
        public byte byte_size;
        public byte byte_parity;
        public byte stop_bits;
        public byte xon_char;
        public byte xoff_char;
        public byte error_char;
        public byte eof_char;
        public byte evt_char;
        public short w_reserved1;

        // flags:
        const int fBinary = 0x0001;                 // Supposed to always be true?
        const int fParity = 0x0002;                 // Set if Parity != None?
        const int fOutxCtsFlow = 0x0004;
        //const int fOutxDsrFlow = 0x0008;          // If the PERQ software wants to
        //const int fDtrControl1 = 0x0010;          // use DSR/DTR, we'll leave these
        //const int fDtrControl2 = 0x0020;          // unset so Windows won't get in
        //const int fDsrSensitivity = 0x0040;       // the way...
        //const int fTXContinueOnXoff = 0x0080;
        const int fOutX = 0x0100;
        const int fInX = 0x0200;
        //const int fErrorChar = 0x0400;
        //const int fNull = 0x0800;
        const int fRtsControl1 = 0x1000;            // If BOTH are set, we could track
        const int fRtsControl2 = 0x2000;            // RTS like the SIO does if AutoEnabled?
        const int fAbortOnError = 0x4000;           // Persists; set to FALSE? (Once on open?)

        public void SetValues(int baud, Parity parity, int bits, StopBits sb, Handshake hs)
        {
            if ((flags & fBinary) == 0)
                Console.WriteLine("fBinary not set!?");

            if ((flags & fAbortOnError) != 0)
                Console.WriteLine("fAbortOnError is TRUE");

            switch (sb)
            {
                case StopBits.One:
                    stop_bits = 0;
                    break;

                case StopBits.OnePointFive:
                    stop_bits = 1;
                    break;

                case StopBits.Two:
                    stop_bits = 2;
                    break;
            }

            baud_rate = baud;
            if (baud_rate > 38400)
                Console.WriteLine($"baud rate {baud_rate} is kind of ridiculous");

            byte_size = (byte)bits;
            byte_parity = (byte)parity;

            // This is probably a mistake. We don't need this much accuracy :-)
            if (parity != Parity.None) flags |= fParity;

            // Clear Handshake flags
            flags &= ~(fOutxCtsFlow | fOutX | fInX | fRtsControl2);

            // Set Handshake flags
            switch (hs)
            {
                case Handshake.None:
                    break;

                case Handshake.XOnXOff:
                    flags |= fOutX | fInX;
                    break;

                case Handshake.RTSCTS:
                    flags |= fOutxCtsFlow | fRtsControl2;
                    break;

                case Handshake.Both:
                    flags |= fOutxCtsFlow | fOutX | fInX | fRtsControl2;
                    break;
            }
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    class Timeouts
    {
        public uint ReadIntervalTimeout;
        public uint ReadTotalTimeoutMultiplier;
        public uint ReadTotalTimeoutConstant;

        public uint WriteTotalTimeoutMultiplier;
        public uint WriteTotalTimeoutConstant;

        public const uint MaxDWord = 0xFFFFFFFF;

        public Timeouts(int read_timeout, int write_timeout)
        {
            SetValues(read_timeout, write_timeout);
        }

        /// <summary>
        /// Sets the values.  In this simplified implementation, read intervals
        /// and read/write multipliers are NOT used -- left as zero.  This means
        /// that the Windows and Unix calls behave the same; zero means polled
        /// operation with NO waits or timeouts, -1 is "infinite", > 0 is total
        /// timeout (in milliseconds).
        /// </summary>
        public void SetValues(int read_timeout, int write_timeout)
        {
            ReadTotalTimeoutConstant = (read_timeout == SerialPort.InfiniteTimeout) ? MaxDWord : (uint)read_timeout;
            WriteTotalTimeoutConstant = (write_timeout == SerialPort.InfiniteTimeout) ? MaxDWord : (uint)write_timeout;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    struct CommStat
    {
        public uint flags;
        public uint BytesIn;
        public uint BytesOut;
    }
}
