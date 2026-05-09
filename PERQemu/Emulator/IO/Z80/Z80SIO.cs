//
// Z80SIO.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

using System;

using PERQemu.IO.SerialDevices;

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Implements the Z80 SIO serial controller, with some PERQ peculiarities
    /// built-in.  It provides the operational modes that the PERQ I/O boards
    /// make use of for talking to RS-232 ports, the Speech output device, and
    /// the Kriz tablet.  On EIO, the serial keyboard is connected to an SIO.
    /// Handles async and (simple) sync modes.
    /// </summary>
    /// <remarks>
    /// See Docs/SerialPorts.txt for way more information.
    /// </remarks>
    public partial class Z80SIO : IZ80Device, IDMADevice
    {
        public Z80SIO(byte baseAddress, Scheduler scheduler, char unit, byte selAddress)
            : this(baseAddress, scheduler, 5)
        {
            _unit = unit;
            _isEIO = true;
            _ports[4] = _dmaSelPortAddress = selAddress;

            Log.Info(Category.SIO, "EIO unit A initialized.");
        }

        public Z80SIO(byte baseAddress, Scheduler scheduler, char unit = 'A')
            : this(baseAddress, scheduler, 4)
        {
            _unit = unit;
            _isEIO = (_unit == 'B');

            Log.Info(Category.SIO, "{0} unit {1} initialized.", _isEIO ? "EIO" : "IOB/CIO", _unit);
        }

        protected Z80SIO(byte baseAddress, Scheduler scheduler, int portCount)
        {
            _baseAddress = baseAddress;

            _ports = new byte[portCount];

            for (var i = 0; i < portCount; i++)
            {
                _ports[i] = (byte)(baseAddress + i);
            }

            _channels = new Channel[2];
            _channels[0] = new Channel(0, scheduler);
            _channels[1] = new Channel(1, scheduler);
        }

        /// <summary>
        /// Fully reset this instance - both channels and DMA.
        /// </summary>
        public void Reset()
        {
            Reset(0);
            Reset(1);

            _dmaChanSelect = SpeechSel;
            _dmaAcknowledged = false;

            Log.Debug(Category.SIO, "Unit {0} reset", _unit);
        }

        /// <summary>
        /// Reset only the specified channel; used to re-open the serial device(s).
        /// </summary>
        public void Reset(int chan)
        {
            _channels[chan].Reset();
        }

        public char Unit => _unit;
        public string Name => $"Z80 SIO {_unit}";
        public byte[] Ports => _ports;

        public bool IntLineIsActive
        {
            get { return _channels[0].InterruptLatched || _channels[1].InterruptLatched; }
        }

        public byte? ValueOnDataBus
        {
            get { return ComputeVector(); }
        }

        /// <summary>
        /// Compute the SIO's interrupt vector based on the chip's arcane rules.
        /// </summary>
        /// <remarks>
        /// The SIO's interrupt vector is only programmed into channel B's WR2
        /// register.  When read back from RR2 (or put on the bus when the IRQ
        /// is serviced) it is supposed to return the current vector based on
        /// the highest priority from _both_ channels.  Because channel A has
        /// higher priority, we have to get both offsets and compute the V3..V1
        /// bits from A or B but using the base vector from B, and only if the
        /// "Status Affects Vector" bit is set (only in B's WR1).  Oof.
        /// </remarks>
        public byte ComputeVector()
        {
            byte vector = _channels[1].InterruptBase;

            if (_channels[1].StatusAffectsVector)
            {
                // If an interrupt is pending on A, use its offset; otherwise
                // assume that B is interrupting...
                var priority = (_channels[0].InterruptLatched ? _channels[0].InterruptOffset + 4 :
                                                                _channels[1].InterruptOffset);

                // Now hack the offset into vector<3:1>
                vector = (byte)((vector & 0xf1) | ((priority & 0x07) << 1));
                Log.Detail(Category.SIO, "Status Affected Vector is 0x{0:x2} (prio={1})", vector, priority);
            }

            return vector;
        }

        public event EventHandler NmiInterruptPulse { add { } remove { } }

        #region IDMADevice implementation

        public bool DMAReadReady
        {
            get
            {
                if (_unit == 'B') return false;                         // Not wired for DMA

                if (_isEIO && _dmaChanSelect == SpeechSel) return false;// No reads from Speech

                return _channels[0].CanRead;                            // Check RSA
            }
        }

        public bool DMAWriteReady
        {
            get
            {
                if (_unit == 'B') return false;                         // Not wired for DMA

                if (_isEIO) return _channels[_dmaChanSelect].CanWrite;  // EIO: Check selected

                return _channels[0].CanWrite || _channels[1].CanWrite;  // Not EIO: check both
            }
        }

        public AcknowledgeDelegate DMAAcknowledge => DMAReadWriteAck;

        public void DMATerminate()
        {
            Log.Detail(Category.SIO, "DMATerminate called (chan {0}, ack {1})",
                                     _dmaChanSelect, _dmaAcknowledged);

            _dmaAcknowledged = false;
        }

        public void DMAReadWriteAck(byte portAddress)
        {
            // Set the SIO_ACK_L equivalent flag to discern "normal" low-volume
            // reads/writes from HiVol DMA ops; tells the unswizzler to apply
            // the _dmaChanSelect remapping, or not.  This is just kinda gross.
            _dmaAcknowledged = true;

            Log.Detail(Category.SIO, "DMA ACK on port 0x{0:x2}", portAddress);
        }

        #endregion

        public void AttachDevice(int channel, SerialDevice device)
        {
            if (channel < 0 || channel > 1)
            {
                throw new ArgumentOutOfRangeException(nameof(channel));
            }

            _channels[channel].AttachDevice(device);
        }

        public void DetachDevice(int channel)
        {
            _channels[channel].DetachDevice();
        }

        /// <summary>
        /// Read from a chip register.  For EIO, they scrambled the bits when
        /// the second SIO was added; the decoder puts data on 0+1, ctrl on 2+3.
        /// This means ports base+1 and base+2 are swapped.  SIGH.
        /// </summary>
        public byte Read(byte portAddress)
        {
            switch (UnSwizzle(portAddress))
            {
                case 0:
                    _dmaAcknowledged = false;
                    return _channels[0].ReadData();

                case 1:
                    return _channels[0].ReadRegister();

                case 2:
                    _dmaAcknowledged = false;
                    return _channels[1].ReadData();

                case 3:
                    return _channels[1].ReadRegister();

                default:
                    throw new InvalidOperationException("Invalid SIO port address");
            }
        }

        /// <summary>
        /// Write to a register.  Same decoder screwiness as above for EIO.
        /// </summary>
        public void Write(byte portAddress, byte value)
        {
            // Check for the extra EIO address first
            if (portAddress == _dmaSelPortAddress)
            {
                // This is programmed as 0 = speech, 1 = RS-232, but they
                // assigned the A & B halves of the device in the opposite way:
                // RS-232 (A is active LOW and Speech (B) is active HIGH. <smdh>
                _dmaChanSelect = ~value & 0x1;

                Log.Debug(Category.SIO, "EIO Speech Select now {0} (0x{1:x})", _dmaChanSelect, value);
                return;
            }

            switch (UnSwizzle(portAddress))
            {
                case 0:
                    _dmaAcknowledged = false;
                    _channels[0].WriteData(value);
                    break;

                case 1:
                    _channels[0].WriteRegister(value);
                    break;

                case 2:
                    _dmaAcknowledged = false;
                    _channels[1].WriteData(value);
                    break;

                case 3:
                    _channels[1].WriteRegister(value);
                    break;

                default:
                    throw new InvalidOperationException("Invalid SIO port address");
            }
        }

        /// <summary>
        /// Rewrite the port address to deal with the EIO hardware swap of the
        /// two low address bits AND account for the SPEECH_SEL_L latch.  Oy vey.
        /// </summary>
        /// <remarks>
        ///     IOB/CIO             EIO (SIOA)      (SIOB)
        /// RSAData 260     -->     RSAData 020     RSBData 100
        /// RSACtrl 261     -->     RSACtrl 022     RSBCtrl 102
        /// SPData  262     -->     SPData  021     KBData  101
        /// SPCtrl  263     -->     SPCtrl  023     KBCtrl  103
        /// 
        /// The SPEECH_SEL_L signal forces the B/A select based on A<0> and the
        /// state of the SIO_ACK_L from the DMAC -- which we now half-assedly
        /// simulate using the DMAAcknowledged delegate to warn of an incoming
        /// DMA read/write.  (It also prioritizes the DMA RQST line based on
        /// whether the RSA or SP port(s) are both requesting service at the
        /// same time, but that's not implemented.)  It's a complicated mess.
        /// </remarks>
        int UnSwizzle(byte portAddress)
        {
            var offset = portAddress - _baseAddress;

            if (!_isEIO) return offset;     // on IOB/CIO, no change

            switch (offset)
            {
                case 0:
                    // RSBData?  As you were
                    if (_unit != 'A') return 0;

                    // RSAData, unless DMA active and Speech selected
                    return (_dmaAcknowledged && _dmaChanSelect == SpeechSel) ? 2 : 0;

                case 1:
                    // KBData?  Right-o, on your way
                    if (_unit != 'A') return 2;

                    // SPData, unless DMA active and RS232 selected
                    return (_dmaAcknowledged && _dmaChanSelect == RSASel) ? 0 : 2;

                case 2:
                    return 1;   // RSACtrl or RSBCtrl

                case 3:
                    return 3;   // SPCtrl or KBCtrl
            }

            return -1;          // Fail
        }

        // Debugging
        public void DumpRegisters()
        {
            Console.WriteLine($"SIO {_unit} status:");

            if (_channels[0].Port != null) _channels[0].DumpRegs();
            if (_channels[1].Port != null) _channels[1].DumpRegs();
        }

        public void DumpPortStatus(int chan)
        {
            _channels[chan].Port?.Status();
        }

        public void Telemetry(int chan, bool enable)
        {
            if (enable)
                _channels[chan].StartLog();
            else
                _channels[chan].StopLog();
        }

        // Extra EIO bits
        const int RSASel = 0;
        const int SpeechSel = 1;

        int _dmaChanSelect;
        byte _dmaSelPortAddress;
        bool _dmaAcknowledged;

        byte _baseAddress;
        byte[] _ports;
        bool _isEIO;
        char _unit;

        Channel[] _channels;
    }
}
