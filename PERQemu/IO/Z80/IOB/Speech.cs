// speech.cs - Copyright (c) 2006-2019 Josh Dersch (derschjo@gmail.com)
//
// This file is part of PERQemu.
//
// PERQemu is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PERQemu is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with PERQemu.  If not, see <http://www.gnu.org/licenses/>.
//

using System;
using System.Collections.Generic;

namespace PERQemu.IO.Z80.IOB
{
    /// <summary>
    /// Represents the PERQ's "Speech" device (digital sound output).  On the
    /// PERQ, this is the MC3417 CVSD chip, driven by a stream of bytes fed in
    /// by the Z80 SIO "B".
    /// 
    /// In Z80 ROM V8.7, the speech data is received from the PERQ in 32-byte
    /// chunks.  The Z80 sets aside two 128-byte buffers in memory and switches
    /// between them in a simple double-buffering scheme.  As one 128-byte buffer
    /// is filled its contents are DMA'ed to the audio output while the other
    /// buffer stores incoming data from the PERQ.
    /// 
    /// The audio device shares the Z80 DMA chip with the floppy controller, but
    /// we may just cheat and allow both devices unfettered data transfer; the
    /// status messages returned will just lie about "ownership" of the DMA.  We
    /// may also have to keep track of how many buffers are active and report
    /// that back to the PERQ in a GetStatus message as well (though our Z80 can
    /// have as much local memory as it wants).  For now the goal is to let the
    /// SIGGRAPH '82 talking demo to run (even if it's not hooked up yet to a
    /// host audio device).
    /// </summary>
    public sealed class Speech : IZ80Device
    {
        public Speech()
        {
            Reset();
        }

        public void Reset()
        {
            _messageData = new byte[64];
            _messageIndex = 0;
            _busyClocks = 0;
        }

        public ReadyFlags BusyBit
        {
            get { return ReadyFlags.Speech; }
        }

        public int BusyClocks
        {
            get { return _busyClocks; }
            set { _busyClocks = value; }
        }

        public bool RunStateMachine(PERQtoZ80Message message, byte value)
        {
            bool retVal = false;

            _messageData[_messageIndex] = value;
            _messageIndex++;

            switch (message)
            {
                case PERQtoZ80Message.SetSpeechStatus:
                    // Set speech status:
                    //   byte 0 = length (may be 1 or 2 bytes)
                    //   byte 1 = baud rate change (so we have to keep track of this?)
                    //   byte 2 = if present, reset the speech device
                    if (_messageIndex > 1 + _messageData[0])
                    {
                        SetStatus();
                        _messageIndex = 0;
                        retVal = true;
                    }
                    break;

                case PERQtoZ80Message.Speech:
                    // Variable bytes for Speech data:
                    //  byte 0 = length (no greater than 32)
                    //  byte 1-N = data
                    if (_messageIndex > 1 + _messageData[0])
                    {
                        // This is "wrong" (see notes above) but it'll do for now
                        MakeSound();
                        _messageIndex = 0;
                        retVal = true;

                        // At 8KHz, figure ~ 90 Z80 clocks/byte... but set our
                        // "busy" time to something a little more reasonable
                        _busyClocks = 10 * _messageData[0];
                    }
                    break;

                default:
#if TRACING_ENABLED
                    if (Trace.TraceOn)
                        Trace.Log(LogType.Warnings, "Unhandled Speech message {0}", message);
#endif
                    break;
            }

            return retVal;
        }

        public void Poll(ref Queue<byte> fifo)
        {
            // Nothing.  Speech is output-only.
        }

        public bool Enabled
        {
            get { return _enabled; }    // never used...
        }

        public void GetStatus(ref Queue<byte> fifo)
        {
            // This is supposed to send back some actual information... which
            // device (speech or floppy) owns the DMA channel?  How many memory
            // buffers are available?  For now it seems that nothing actually
            // requests the status...
#if TRACING_ENABLED
            if (Trace.TraceOn)
                Trace.Log(LogType.Speech, "GetStatus requested for Speech!  (Not Implemented)");
#endif
        }

        private void SetStatus()
        {
            //
            // Sets the status (baud rate) of the speech output device, or for
            // debugging allows a forced reset, per Z80 V8.7.
            //
            if (_messageData[0] == 1)
            {
#if TRACING_ENABLED
                if (Trace.TraceOn)
                    Trace.Log(LogType.Speech, "SetSpeechStatus: baud rate change is {0:x2}", _messageData[1]);
#endif
                _busyClocks = 5;        // time to reset the sio chip and ctc :-)
            }
            else if (_messageData[0] == 2)
            {
                // Did they send us the super sekrit code?
                if (_messageData[2] == 0x99)
                {
#if TRACING_ENABLED
                    if (Trace.TraceOn)
                        Trace.Log(LogType.Speech, "SetSpeechStatus: resetting speech device.");
#endif
                    _busyClocks = 5;    // hack just to get it working
                }
                else
                {
#if TRACING_ENABLED
                    if (Trace.TraceOn)
                        Trace.Log(LogType.Speech, "SetSpeechStatus: bad reset code! {0:2x}", _messageData[2]);
#endif
                    _busyClocks = 1;    // to force a status change?  this isn't supposed to be used normally anyway
                }
            }
            else
            {
#if TRACING_ENABLED
                if (Trace.TraceOn)
                    Trace.Log(LogType.Speech, "SetSpeechStatus:  bad message length!");
#endif
            }
        }

        private void MakeSound()
        {
            int dataLength = _messageData[0];

            if (dataLength != 32)
            {
#if TRACING_ENABLED
                if (Trace.TraceOn)
                    Trace.Log(LogType.Speech, "MakeSound: wrong byte count; expecting 32, got " + dataLength);
                // Just let it go, man, let it go
#endif
            }
            else
            {
#if TRACING_ENABLED
                if (Trace.TraceOn)
                    Trace.Log(LogType.Speech, "MakeSound: {0} bytes", dataLength);
#endif
                // Someday we'll actually make some noise...
                // Does ^G (ASCII BEL) work on Windows? :-)
            }
        }

        private byte[] _messageData;
        private int _messageIndex;
        private int _busyClocks;
        private bool _enabled = false;
    }
}
