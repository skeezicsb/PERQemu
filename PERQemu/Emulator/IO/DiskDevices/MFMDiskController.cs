//
// MFMDiskController.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
using PERQmedia;

namespace PERQemu.IO.DiskDevices
{
    /// <summary>
    /// Represents the 5.25" hard drive controller on the EIO board which manages
    /// disk drives in the Disk5Inch class.  This is implemented in the PERQ as an
    /// adapter from the SA4000 interface to a variety of MFM/ST-506 style drives.
    /// This version emulates both types of DIB (one which supports write pre-
    /// compensation, and the other that doesn't).  See Docs/HardDisk.txt for more.
    /// </summary>
    public sealed class MFMDiskController : IStorageController
    {
        public MFMDiskController()
        {
            _dib = new DiskInterfaceBoard(this);
        }

        public void Reset()
        {
            throw new NotImplementedException();
        }

        public void AttachDrive(uint unit, StorageDevice dev)
        {
            throw new NotImplementedException();
        }

        public void LoadRegister(byte address, int value)
        {
            throw new NotImplementedException();
        }

        public void DoSingleSeek()
        {
            throw new NotImplementedException();
        }

        public int ReadStatus()
        {
            throw new NotImplementedException();
        }

        public void DumpStatus()
        {
            throw new NotImplementedException();
        }

        /// <summary>
        /// The 5.25" Disk Interface Board.  Uses a six-bit bus, similar to but
        /// very distinct from the 8" DIB used in the Micropolis driver.
        /// </summary>
        /// <remarks>
        /// In theory the EIO board could be jumpered to talk to all three types
        /// of drives, but that's way too messy and confusing. 
        /// </remarks>
        internal class DiskInterfaceBoard
        {
            public DiskInterfaceBoard(MFMDiskController control)
            {
                _drives = new HardDisk[2];  // Four, someday? :-)
            }

            HardDisk[] _drives;
        }

        DiskInterfaceBoard _dib;
    }
}
