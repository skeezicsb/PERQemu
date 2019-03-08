// rawfloppydisk.cs - Copyright (c) 2006-2019 Josh Dersch (derschjo@gmail.com)
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
using System.Text;

namespace PERQemu.PhysicalDisk
{
    // Represents a single or double-sided 8" floppy disk in RAW format,
    // with or without the 9-byte magic cookie ("PFD" header).
    public sealed class RawFloppyDisk : PhysicalDisk
    {
        public RawFloppyDisk(DiskGeometry geometry) : base()
        {
            _diskType = geometry;
            CreateSectors();
        }

        public override void Load(System.IO.FileStream fs)
        {
            base.Load(fs);
        }

        //
        // The cheap & cheerful "header" format for PERQemu floppy disk images is:
        //
        // 7 bytes - "PERQflp"
        // 1 byte  - sector size + # of sides
        // 1 byte  - filesystem type hint (not used by PERQemu)
        //
        // This is a silly hack; we should consider reading and writing the "IMD"
        // format directly, at least for floppies (if not hard disks too?).
        public override void ReadHeader(System.IO.FileStream fs)
        {
            // No header info on raw floppy disk images.  But since
            // none of the known PERQ floppy formats (including PNX)
            // use sector (0,0,0), we write our "PFD" cookie there,
            // and remain compatible.

            byte[] pfd = new byte[9];

            fs.Read(pfd, 0, 9);
            fs.Position = 0;        // Rewind

            for (int i = 0; i < 7; i++)
            {
                if (pfd[i] != _cookie[i])
                {
                    return;         // No cookie for you
                }
            }

            byte geom = pfd[7];

            Console.WriteLine("** Floppy header geometry hint is {0:x2}", geom);

            if ((geom & 0xfc) * 2 != _diskType.SectorSize)
            {
                Console.WriteLine("** Floppy header byte says sector size is {0}, was set to {1}",
                                  (geom & 0xfc) * 2, _diskType.SectorSize);
            }

            if ((geom & 0x03) != _diskType.Tracks)
            {
                Console.WriteLine("** Floppy header says disk has {0} tracks, was set to {1}",
                                  (geom & 0x03), _diskType.Tracks);
            }

            byte fsHint = pfd[8];

            Console.WriteLine("** Floppy header filesystem hint is {0:x2}", fsHint);
        }

        public override void WriteHeader(System.IO.FileStream fs)
        {
            // Here we tuck our pseudo-header into the first 9 bytes of the
            // first sector, rather than write it out to the file stream directly.
        }

        private static byte[] _cookie = { (byte)'P', (byte)'E', (byte)'R', (byte)'Q',
                                          (byte)'f', (byte)'l', (byte)'p' };

    }
}
