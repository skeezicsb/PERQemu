﻿//
//  FileUtilities.cs
//
//  Author:  S. Boondoggle <skeezicsb@gmail.com>
//
//  Copyright (c) 2022, Boondoggle Heavy Industries, Ltd.
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

using System.IO;
using System.Collections.Generic;

namespace PERQmedia
{
    /// <summary>
    /// Utilities for working with PERQmedia files.
    /// </summary>
    public static class FileUtilities
    {

        /// <summary>
        /// Determine a device type by querying a file's header.  (Does not
        /// load any data or determine validity of the contents).
        /// </summary>
        public static DeviceType GetDeviceTypeFromFile(string file)
        {
            var dev = new StorageDevice();

            if (dev.CanLoad(file))
            {
                return dev.Info.Type;
            }

            return DeviceType.Unused;
        }

        /// <summary>
        /// Return a list of the formatters to try based on the given pathname.
        /// This is kind cheesy.  In time it shouldn't even be necessary as the
        /// library of PERQ media images is all converted to PERQmedia format!
        /// </summary>
        public static List<IMediaFormatter> GetFormattersForFile(string path)
        {
            var formatters = new List<IMediaFormatter>();

            var ext = Path.GetExtension(path).ToLower();

            switch (ext)
            {
                case ".prqm":
                    formatters.Add(new PRQFormatter());
                    break;

                case ".phd":
                    formatters.Add(new PHDFormatter());
                    break;

                case ".imd":
                    formatters.Add(new IMDFormatter());
                    break;

                case ".pfd":
                case ".raw":
                    formatters.Add(new RawFormatter());
                    break;

                //  tape - prqm, tap - none yet implemented

                default:
                    // Hell, try 'em all?
                    formatters.Add(new PRQFormatter());
                    formatters.Add(new PHDFormatter());
                    formatters.Add(new IMDFormatter());
                    formatters.Add(new RawFormatter());
                    break;
            }

            return formatters;
        }
    }
}