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

using System;

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
        /// <remarks>
        /// Every PERQ has the CVSD chip, but we can't always assume that the
        /// virtual machine is configured to use it.  If the user elects not to
        /// enable, quietly drop the bytes in the bitbucket.
        /// </remarks>
        public void Transmit(byte value)
        {
            _txDevice?.Transmit(value);
        }

        // Not used by Speech device?
        public void TransmitAbort()
        {
            _txDevice?.TransmitAbort();
        }

        // Not used by Speech device?
        public void TransmitBreak()
        {
            _txDevice?.TransmitBreak();
        }

        ISIODevice _txDevice;       // MC3417 for audio output
        ISIODevice _rxDevice;       // KrizTablet for mouse input
    }
}

/*
    Notes:

    This class acts as a mux that allows two devices to share one SIO channel.
    On the PERQ-1 IOB, the RS232 port is "A", while speech out is the transmit
    half of "B".  I _guessed_ that if you attached a Kriz tablet to a PERQ-1
    it'd work like the EIO - using the receive half of "B" - and it seems that
    is in fact the case.  Several OSes running the CIO Z80 code work with it in
    that arrangement. :-)

    Need to get back to testing EIO Serial stuff to see if it actually works on
    a real port, anyway.  (And HiVol GPIB, for that matter, though I'm not sure
    how if I don't have any other simulated GPIB devices yet).

    Time for a Docs/Speech.txt to collect up the notes!

    Re: the SDL2 interface... Should I create
        UI/SDL/OutputDevices.cs     -- audio... and?
            or
        just add it to Display.cs?  -- the speaker is in the display cabinet... :-)

    and:
        UI/SDL/CustomEvents.cs      -- pull the two user events from Display and
                                       EventLoop into a small separate class to
                                       add new Audio events
                                       
    Do I actually _need_ to create some custom events?  Using Pause()/Queue()
    just seems to "work" without any fuss, and I think they're both thread safe?
    But I have the plumbing in place to register delegates already, and it'd be
    nice to be able to fill the buffer then signal "hey, go grab it and play it"
    like the Display renderer does, kind of.

    Hmm.

 */
