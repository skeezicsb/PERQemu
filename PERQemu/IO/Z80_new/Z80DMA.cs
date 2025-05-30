﻿using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace PERQemu.IO.Z80_new
{
    /// <summary>
    /// Implements most of the Z80 DMA controller.
    /// </summary>
    public class Z80DMA : IZ80Device
    {
        public Z80DMA(byte baseAddress, IOBMemoryBus memoryBus, IOBIOBus ioBus)
        {
            _memoryBus = memoryBus;
            _ioBus = ioBus;
            _baseAddress = baseAddress;

            Reset();
        }

        public void Reset()
        {
            _writeBaseRegister = true;
            _baseRegister = 0;

            _portAddressAInit = 0;
            _portAddressBInit = 0;
            _blockLength = 0;
            _byteCounter = 0;
            _maskByte = 0;
            _matchByte = 0;
            _interruptControl = 0;
            _pulseControl = 0;
            _interruptVector = 0;
            _interruptActive = false;
            _enableDMA = false;

            _wr = new byte[7];
        }

        public string Name => "Z80 DMA";

        public byte[] Ports => new byte[] { _baseAddress };

        public bool IntLineIsActive => _interruptEnabled && _interruptActive;

        public byte? ValueOnDataBus => _interruptVector;    // TODO: implement dynamic vector based on type

        public event EventHandler NmiInterruptPulse;

        public void AttachDeviceA(IDMADevice device)
        {
            _deviceA = device;
        }

        public void AttachDeviceB(IDMADevice device)
        {
            _deviceB = device;
        }

        public void Execute()
        {
            // TODO: handle "Interrupt on RDY" option

            // If DMA is in progress, make it happen.
            if (_enableDMA)
            {
                IDMADevice source;
                IDMADevice dest;

                ushort sourceAddress;
                bool sourceIsIO;
                ushort destAddress;
                bool destIsIO;

                // What direction is this going in
                WR0 wr0 = (WR0)_wr[0];
                WR1 wr1 = (WR1)_wr[1];
                WR2 wr2 = (WR2)_wr[2];

                if ((wr0 & WR0.DirectionAtoB) != 0)
                {
                    // Source is A, dest is B
                    source = _deviceA;
                    sourceAddress = _portAddressA;
                    sourceIsIO = (wr1 & WR1.MemoryOrIO) != 0;
                    dest = _deviceB;
                    destAddress = _portAddressB;
                    destIsIO = (wr2 & WR2.MemoryOrIO) != 0;
                }
                else
                {
                    source = _deviceB;
                    sourceAddress = _portAddressB;
                    sourceIsIO = (wr2 & WR2.MemoryOrIO) != 0;
                    dest = _deviceA;
                    destAddress = _portAddressA;
                    destIsIO = (wr1 & WR1.MemoryOrIO) != 0;
                }

                // If the source has a byte ready for us and the destination is also ready, 
                // do the transfer:
                if (source.ReadDataReady && dest.WriteDataReady)
                {
                    byte data = 0;
                    if (sourceIsIO)
                    {
                        data = _ioBus[sourceAddress];
                    }
                    else
                    {
                        data = _memoryBus[sourceAddress];
                    }

                    if (destIsIO)
                    {
                        _ioBus[destAddress] = data;
                    }
                    else
                    {
                        _memoryBus[destAddress] = data;
                    }

                    // Update addresses & counters
                    _byteCounter--;

#if TRACING_ENABLED
                    if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "DMA transfer of 0x{0:x2} from {1} (0x{2:x4}) to {3} (0x{4:x4}), {5} bytes left.", data, source, sourceAddress, dest, destAddress, _byteCounter);
#endif


                    if ((wr1 & WR1.PortAAddressFixed) == 0)
                    {
                        if ((wr1 & WR1.PortAIncrements) == 0)
                        {
                            _portAddressA--;
                        }
                        else
                        {
                            _portAddressA++;
                        }
                    }

                    if ((wr2 & WR2.PortBAddressFixed) == 0)
                    {
                        if ((wr2 & WR2.PortBIncrements) == 0)
                        {
                            _portAddressB--;
                        }
                        else
                        {
                            _portAddressB++;
                        }
                    }
                }

                // Take action at the end of the block --
                // interrupt, restart, etc.
                if (_byteCounter == 0)
                {
#if TRACING_ENABLED
                    if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "DMA transfer complete.");
#endif
                    if ((_interruptControl & 0x2) != 0)
                    {
                        _interruptActive = true;
                    }

                    WR5 wr5 = (WR5)_wr[5];

                    if ((wr5 & WR5.AutoRepeat) != 0)
                    {
                        _byteCounter = _blockLength;
                        _portAddressA = _portAddressAInit;
                        _portAddressB = _portAddressBInit;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "DMA transfer auto-restarting.");
#endif
                    }
                    else
                    {
                        source.DMATerminate();
                        dest.DMATerminate();
                        _enableDMA = false;
                    }

                }
            }
        }

        public byte Read(byte portAddress)
        {
            throw new NotImplementedException();
        }

        public void Write(byte portAddress, byte value)
        {
            // DMA is disabled when a control byte is written
            _enableDMA = false;

            if (_writeBaseRegister)
            {
                _baseRegister = 0;

                // Note: we start with the decode for reg 1; if no matches are found
                // we assume a match on reg 0.
                for (int i = 1; i < _decodes.Length; i++)
                {
                    if ((value & _decodes[i].Mask) == _decodes[i].Value)
                    {
                        _baseRegister = i;
                        break;
                    }
                }

#if TRACING_ENABLED
                if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Write of 0x{0:x2} to Z80 DMA base register {1}", value, _baseRegister);
#endif
                WriteBaseRegister(value);
            }
            else
            {
                // Write sub-registers
                WriteSubRegister(value);
            }
        }

        private void WriteBaseRegister(byte value)
        {
            _wr[_baseRegister] = value;

            switch(_baseRegister)
            {
                case 0:
                    // bits D3-D6 indicate sub-registers to be written to, if none are set,
                    // return to base state.
                    _writeBaseRegister = (value & 0x78) == 0;
                    break;

                case 1:
                case 2:
                    // bit D6 indicates a sub-register is to be written
                    _writeBaseRegister = (value & 0x40) == 0;
                    break;

                case 3:
                    // bits D3 and D4:
                    _writeBaseRegister = (value & 0x18) == 0;
                    break;

                case 4:
                    // bits D2-D4:
                    _writeBaseRegister = (value & 0x1c) == 0;
                    break;

                case 6:
                    // handle command
                    ExecuteCommand(value);
                    _writeBaseRegister = true;
                    break;
            }
        }

        private void WriteSubRegister(byte value)
        {
            byte regVal = _wr[_baseRegister];
            switch(_baseRegister)
            {
                case 0:
                    if ((regVal & 0x08) != 0)
                    {
                        _portAddressAInit = (ushort)((_portAddressAInit & 0xff00) | value);
                        _wr[_baseRegister] &= 0xf7;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Port A address now 0x{0:x4}", _portAddressAInit);
#endif
                    }
                    else if ((regVal & 0x10) != 0)
                    {
                        _portAddressAInit = (ushort)((_portAddressAInit & 0x00ff) | (value << 8));
                        _wr[_baseRegister] &= 0xef;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Port A address now 0x{0:x4}", _portAddressAInit);
#endif
                    }
                    else if ((regVal & 0x20) != 0)
                    {
                        _blockLength = (ushort)((_blockLength & 0xff00) | value);
                        _wr[_baseRegister] &= 0xdf;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Block length now 0x{0:x4}", _blockLength);
#endif
                    }
                    else if ((regVal & 0x20) != 0)
                    {
                        _blockLength = (ushort)((_blockLength & 0x00ff) | (value << 8));
                        _wr[_baseRegister] &= 0xbf;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Block length now 0x{0:x4}", _blockLength);
#endif
                    }
                    _writeBaseRegister = (value & 0x78) == 0;
                    break;

                case 1:
                case 2:
                    // Right now I'm not implementing variable timing, so just ignore this.
                    _writeBaseRegister = true;
                    break;

                case 3:
                    if ((regVal & 0x08) != 0)
                    {
                        _maskByte = value;
                        _wr[_baseRegister] &= 0xf7;
                    }
                    else if ((regVal & 0x10) != 0)
                    {
                        _matchByte = value;
                        _wr[_baseRegister] &= 0xef;
                    }
                    _writeBaseRegister = (value & 0x18) == 0;
                    break;

                case 4:
                    if ((regVal & 0x04) != 0)
                    {
                        _portAddressBInit = (ushort)((_portAddressBInit & 0xff00) | value);
                        _wr[_baseRegister] &= 0xfb;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Port Address B now 0x{0:x4}", _portAddressBInit);
#endif
                    }
                    else if ((regVal & 0x08) != 0)
                    {
                        _portAddressBInit = (ushort)((_portAddressBInit & 0x00ff) | (value << 8));
                        _wr[_baseRegister] &= 0xf7;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Port Address B now 0x{0:x4}", _portAddressBInit);
#endif
                    }
                    else if ((regVal & 0x10) != 0)
                    {
                        _interruptControl = value;
                        _wr[_baseRegister] &= 0xef;
                    }
                    else if ((_interruptControl & 0x08) != 0)
                    {
                        _pulseControl = value;
                        _interruptControl &= 0xf7;
                    }
                    else if ((_interruptControl & 0x10) != 0)
                    {
                        _interruptVector = value;
                        _interruptControl &= 0xef;

#if TRACING_ENABLED
                        if (Trace.TraceOn) Trace.Log(LogType.Z80DMA, "Interrupt vector now 0x{0:x4}", _interruptVector);
#endif
                    }
                    _writeBaseRegister = ((regVal & 0x1c) == 0) && ((_interruptControl & 0x18) == 0);

                    break;

                default:
                    // Shouldn't happen
                    throw new InvalidOperationException("Unexpected subregister write.");
            }
        }
        
        private void ExecuteCommand(byte command)
        {
            switch(command)
            {
                case 0xc3: // Reset
                case 0xa3: // TODO: this is a slightly different reset
                    Reset();
                    break;

                case 0xc7:
                case 0xc8:
                    // Reset timings, ignored for now.
                    break;

                case 0xcf:  // Load starting addresses, reset byte counter
                    _byteCounter = 0;
                    _portAddressA = _portAddressAInit;
                    _portAddressB = _portAddressBInit;
                    break;

                case 0xd3:  // Continue from current locations, clear byte counter
                    _byteCounter = 0;
                    break;

                case 0xab:
                    _interruptEnabled = true;
                    break;

                case 0xaf:
                    _interruptEnabled = false;
                    break;

                case 0x87:
                    _enableDMA = true;
                    _byteCounter = (ushort)(_blockLength + 1);
                    break;

                case 0x83:
                    _enableDMA = false;
                    break;

                case 0xbb:
                    //_readMask = 

                case 0xb3:
                case 0x88:
                case 0xb7:
                    // Unimplemented.
                    throw new NotImplementedException();
            }
        }

        private IOBIOBus _ioBus;
        private IOBMemoryBus _memoryBus;
        private IDMADevice _deviceA;
        private IDMADevice _deviceB;
        private byte _baseAddress;

        private bool _writeBaseRegister;
        private int _baseRegister;

        private byte[] _wr;

        private ushort _portAddressAInit;
        private ushort _portAddressBInit;
        private ushort _blockLength;
        private byte _maskByte;
        private byte _matchByte;
        private byte _interruptControl;
        private byte _pulseControl;
        private byte _interruptVector;

        private bool _enableDMA;
        private bool _interruptActive;
        private bool _interruptEnabled;

        // Current port addresses
        private ushort _portAddressA;
        private ushort _portAddressB;
        private ushort _byteCounter;

        [Flags]
        private enum WR0
        {
            Transfer = 0x1,
            Search = 0x2,
            SearchTransfer = 0x3,
            DirectionAtoB = 0x4,
        }

        [Flags]
        private enum WR1
        {
            MemoryOrIO = 0x8,
            PortAIncrements = 0x10,
            PortAAddressFixed = 0x20,
        }

        [Flags]
        private enum WR2
        {
            MemoryOrIO = 0x8,
            PortBIncrements = 0x10,
            PortBAddressFixed = 0x20,
        }

        [Flags]
        private enum WR3
        {
            StopOnMatch = 0x4,
            InterruptEnable = 0x20,
            DMAEnable = 0x40,
        }

        [Flags]
        private enum WR5
        {
            ReadyActiveHigh = 0x8,
            CEWAITMultiplexed = 0x10,
            AutoRepeat = 0x20,
        }

        private struct RegisterDecodes
        {
            public RegisterDecodes(byte mask, byte value)
            {
                Mask = mask;
                Value = value;
            }

            public byte Mask;
            public byte Value;
        }

        private RegisterDecodes[] _decodes = new RegisterDecodes[]
        {
            new RegisterDecodes(0x80, 0x00),    // WR0, not actually decoded (assumed if no matches are found)
            new RegisterDecodes(0x87, 0x04),
            new RegisterDecodes(0x87, 0x00),
            new RegisterDecodes(0x83, 0x80),
            new RegisterDecodes(0x83, 0x81),
            new RegisterDecodes(0xc7, 0x82),
            new RegisterDecodes(0x83, 0x83),    // WR6
        };

    }
}
