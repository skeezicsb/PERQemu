//
// DMARouter.cs - Copyright (c) 2006-20265 Josh Dersch (derschjo@gmail.com)
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

namespace PERQemu.IO.Z80
{
    public enum SelectedDMADevice
    {
        None = 0,
        Floppy = 1,
        PERQReadFIFO = 2,
        PERQWriteFIFO = 3,
        SIOA = 4,
        SIOB = 5,
        GPIB = 6,
    }

    /// <summary>
    /// Routes DMA requests to PERQ IOB's Z80 DMA-capable devices as controlled
    /// by IOReg3 and enumerated above.
    /// </summary>
    public class DMARouter : IDMADevice
    {
        public DMARouter(Z80System system)
        {
            _system = system;
        }

        public void SelectDMADevice(SelectedDMADevice device)
        {
            switch (device)
            {
                case SelectedDMADevice.None:
                    // No change
                    break;

                case SelectedDMADevice.Floppy:
                    _selectedDevice = _system.FDC;
                    break;

                case SelectedDMADevice.SIOA:
                    _selectedDevice = _system.SIOA;     // RS232, through SIO chan A
                    break;

                case SelectedDMADevice.SIOB:
                    _selectedDevice = _system.SIOA;     // Speech, through SIO chan B
                    break;

                default:
                    throw new NotImplementedException($"DMA not implemented for device {device}");
            }

            if (device != SelectedDMADevice.None)
            {
                Log.Debug(Category.Z80DMA, "Selected DMA device {0}", device);
            }
        }

        public bool DMAReadReady => _selectedDevice.DMAReadReady;
        public bool DMAWriteReady => _selectedDevice.DMAWriteReady;

        public AcknowledgeDelegate DMAAcknowledge => _selectedDevice.DMAAcknowledge;

        public void DMATerminate()
        {
            _selectedDevice.DMATerminate();
        }

        Z80System _system;
        IDMADevice _selectedDevice;
    }
}
