//
// Settings.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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
using System.IO;
using System.Net.NetworkInformation;
using System.Collections.Generic;

using SDL2;

using PERQemu.IO;
using PERQemu.IO.Ports;
using PERQemu.IO.Network;
using PERQemu.IO.Z80;

namespace PERQemu
{
    public enum Ask
    {
        Yes = 0,
        No,
        Maybe
    }

    public enum Radix
    {
        Binary = 2,
        Octal = 8,
        Decimal = 10,
        Hexadecimal = 16
    }

    public enum Cursor
    {
        DefaultArrow = 0,
        Crosshairs,
        Hidden
    }

    public enum ImageFormat
    {
        None = 0,
        Jpeg,
        Png,
        Tiff,
        Raw
    }

    [Flags]
    public enum RateLimit
    {
        None = 0,                       // 110% on the reactor!
        CPUSpeed = 0x01,                // Strive for accuracy
        DiskSpeed = 0x02,               // Feel the pain
        TapeSpeed = 0x04,               // Is this a trick question?
        PrinterSpeed = 0x08,            // Realistic Canon printer delays?
        StartupDelay = 0x10,            // For the truly hardcore
        SpeechDelay = 0x20              // Do NOT adjust speech rate
    }

    public static class Settings
    {
        static Settings()
        {
            Reset();
        }

        public static void Reset()
        {
            // Set to defaults
            SaveDiskOnShutdown = Ask.Maybe;
            SaveFloppyOnEject = Ask.Maybe;
            SaveTapeOnUnload = Ask.Maybe;
            PauseOnReset = true;
            PauseWhenMinimized = true;
            CursorPreference = Cursor.DefaultArrow;

            Performance = RateLimit.CPUSpeed | RateLimit.DiskSpeed | RateLimit.TapeSpeed;
            RunMode = ExecutionMode.Asynchronous;

            DebugRadix = Radix.Decimal;
            Z80Radix = Radix.Decimal;

            OutputDirectory = Paths.OutputDir;
            ScreenshotFormat = ImageFormat.Png;         // More compact
            ScreenshotTemplate = "Screenshot_{0}.{1}";  // Default: Screenshot_{date}.png

            CanonFormat = ImageFormat.Tiff;             // Multi-page capable
            CanonTemplate = "{0}_{1:000}.{2}";          // Default: {canon,cx}_pg_000.png
            CanonPaperSize = PaperCode.USLetter;        // Set to A4 based on locale? :-)
            CanonResolution = 300;                      // CX is probably more popular

            EtherDevice = string.Empty;

            RSADevice = string.Empty;
            RSASettings = SerialSettings.Defaults;

            RSBDevice = string.Empty;
            RSBSettings = SerialSettings.Defaults;

            AudioDevice = PERQemu.HostIsUnix ? string.Empty : "dsound";
            AudioSettings = SpeechSettings.Defaults;

            RTCYearOffset = 1980;
            Keymap = string.Empty;

            Reason = "Settings reset to defaults.";
            Changed = false;
        }

        // General
        public static Ask SaveDiskOnShutdown;
        public static Ask SaveFloppyOnEject;
        public static Ask SaveTapeOnUnload;
        public static bool PauseOnReset;

        public static bool PauseWhenMinimized;
        public static Cursor CursorPreference;

        // Performance
        public static RateLimit Performance;
        public static ExecutionMode RunMode;

        // Debugger
        public static Radix DebugRadix;
        public static Radix Z80Radix;

        // Output
        public static string OutputDirectory;
        public static ImageFormat ScreenshotFormat;
        public static string ScreenshotTemplate { get; private set; }
        public static ImageFormat CanonFormat;
        public static PaperCode CanonPaperSize;
        public static uint CanonResolution;
        public static string CanonTemplate { get; private set; }

        // Logging - see Log.cs
        //public static string LogDir;
        //public static string LogTemplate;
        //public static int LogSize;
        //public static int LogLimit;

        // Host Devices
        public static string RSADevice;
        public static string RSBDevice;

        public static SerialSettings RSASettings;
        public static SerialSettings RSBSettings;

        public static string AudioDevice;
        public static SpeechSettings AudioSettings;

        public static string EtherDevice;

        // Customizations
        public static int RTCYearOffset;
        public static string Keymap;

        // Housekeeping
        public static string Reason;
        public static bool Changed;


        /// <summary>
        /// Load the user preferences file, or fall back to the defaults
        /// if it doesn't exist.
        /// </summary>
        public static void Load()
        {
            // Set the list of host ports for the "assign x device" commands
            PERQemu.CLI.UpdateKeywordMatchHelpers("ComPorts", GetHostSerialPorts());
            PERQemu.CLI.UpdateKeywordMatchHelpers("NICs", GetHostEthernetNICs());
            PERQemu.CLI.UpdateKeywordMatchHelpers("AudioDrivers", GetAudioDrivers());
            PERQemu.CLI.UpdateKeywordMatchHelpers("SerialFlags", GetSerialOptionFlags());

            try
            {
                if (!File.Exists(Paths.SettingsPath))
                {
                    // First time?  Missing file?  Set to defaults and save
                    Reset();
                    Changed = true;
                    Save();
                }
                else
                {
                    PERQemu.CLI.ReadScript(Paths.SettingsPath);
                    Reason = "Settings loaded.";
                    Changed = false;
                }
            }
            catch (Exception e)
            {
                Reason = $"Failed to load settings: {e.Message}";
                Reset();
            }
        }

        /// <summary>
        /// Write out a new user preferences file with the current settings.
        /// </summary>
        public static bool Save()
        {
            if (!Changed)
            {
                // Lie.  Nothing changed, so nothing to save...
                Reason = "Settings unchanged.";
                return true;
            }

            try
            {
                using (StreamWriter sw = new StreamWriter(Paths.SettingsPath, false))
                {
                    //
                    // Write a small header, then enter configuration mode and
                    // write the basic things first.
                    //
                    sw.WriteLine($"# PERQemu settings file, written {DateTime.Now}");
                    sw.WriteLine($"# {PERQemu.Version}");
                    sw.WriteLine("#\n# * Please take care if hand-editing this file! *");
                    sw.WriteLine("# *  Errors may cause PERQemu to fail to load.  *\n#");
                    sw.WriteLine("settings");
                    sw.WriteLine("default");
                    sw.WriteLine($"autosave harddisk {SaveDiskOnShutdown}");
                    sw.WriteLine($"autosave floppy {SaveFloppyOnEject}");
                    sw.WriteLine($"autosave tape {SaveTapeOnUnload}");
                    sw.WriteLine($"display cursor {CursorPreference}");
                    sw.WriteLine($"pause on reset {PauseOnReset}");
                    sw.WriteLine($"pause when minimized {PauseWhenMinimized}");

                    // Enumerate any rate limit options
                    if (Performance == RateLimit.None)
                    {
                        sw.WriteLine("rate limit none");
                    }
                    else
                    {
                        foreach (RateLimit opt in Enum.GetValues(typeof(RateLimit)))
                        {
                            if (Performance.HasFlag(opt))
                                sw.WriteLine($"rate limit {opt}");
                        }
                    }

                    // Device mappings
                    if (!string.IsNullOrEmpty(AudioDevice))
                    {
                        sw.WriteLine($"assign audio device {AudioDevice}");
                    }

                    // Todo: write out tunings -- either update CLI to understand float/double
                    // or will have to store scaled integer values (would make for easier comparisons)
                    // for now, just save the channels setting, since that's the only one likely to change
                    if (AudioSettings.Channels != 1)
                    {
                        sw.WriteLine($"assign audio option channels {AudioSettings.Channels}");
                    }

                    // Syntax change in v0.9.0
                    if (!string.IsNullOrEmpty(RSADevice))
                    {
                        sw.Write("assign rs232a device ");
                        sw.WriteLine(RSADevice == "RSX:" ? "RSX:" : $"{RSADevice} {RSASettings}");

                        // Write out handshaking, option flags separately
                        if (RSASettings.FlowControl != Handshake.None)
                        {
                            if (RSASettings.FlowControl == Handshake.XOnXOff || RSASettings.FlowControl == Handshake.Both)
                                sw.WriteLine($"assign rs232a option {Handshake.XOnXOff}");

                            if (RSASettings.FlowControl == Handshake.RTSCTS || RSASettings.FlowControl == Handshake.Both)
                                sw.WriteLine($"assign rs232a option {Handshake.RTSCTS}");
                        }

                        if (RSASettings.Options != SerialOptions.None)
                        {
                            sw.WriteLine($"assign rs232a option {RSASettings.Options}");
                        }
                    }

                    // New in v0.9.0; RSX: on port B never worked; quietly drop it if set
                    if (!string.IsNullOrEmpty(RSBDevice) && RSBDevice != "RSX:")
                    {
                        sw.WriteLine($"assign rs232b device {RSBDevice} {RSBSettings}");

                        if (RSBSettings.FlowControl != Handshake.None)
                        {
                            if (RSBSettings.FlowControl == Handshake.XOnXOff || RSBSettings.FlowControl == Handshake.Both)
                                sw.WriteLine($"assign rs232b option {Handshake.XOnXOff}");

                            if (RSBSettings.FlowControl == Handshake.RTSCTS || RSBSettings.FlowControl == Handshake.Both)
                                sw.WriteLine($"assign rs232b option {Handshake.RTSCTS}");
                        }

                        if (RSBSettings.Options != SerialOptions.None)
                        {
                            sw.WriteLine($"assign rs232b option {RSBSettings.Options}");
                        }
                    }

                    if (!string.IsNullOrEmpty(EtherDevice))
                    {
                        sw.WriteLine($"assign ethernet device {EtherDevice}");
                    }

                    if (!string.IsNullOrEmpty(OutputDirectory))
                        sw.WriteLine($"output directory \"{OutputDirectory}\"");

                    sw.WriteLine($"canon output format {CanonFormat}");
                    sw.WriteLine($"canon paper size {CanonPaperSize}");
                    sw.WriteLine($"canon resolution {CanonResolution}");

                    // New in v0.7.8; for limited backward compatibility don't
                    // write the option if it hasn't been changed
                    if (RTCYearOffset != 1980)
                        sw.WriteLine($"rtc offset {RTCYearOffset}");

                    // New in v0.8.4; save custom keymap, if set
                    if (!string.IsNullOrEmpty(Keymap))
                        sw.WriteLine($"keymap {Keymap}");

                    sw.WriteLine("#");
                    sw.WriteLine("# These options are not yet implemented:");
                    sw.WriteLine($"# debug radix {DebugRadix}");
                    sw.WriteLine($"# z80 debug radix {Z80Radix}");
                    sw.WriteLine($"# screenshot output format {ScreenshotFormat}");
                    sw.WriteLine("#");

                    sw.WriteLine("done");

                    // If the user has selected a valid PERQ, save it so it's
                    // the default machine configuration upon restart
                    if (PERQemu.Config.Current.IsValid &&
                        PERQemu.Config.Current.Name != "default")
                    {
                        sw.WriteLine("#\n# Most recent loaded configuration:");
                        sw.WriteLine($"configure load {PERQemu.Config.Current.Name}");
                    }

                    sw.Close();
                }

                Reason = "Settings saved.";
                Changed = false;
                return true;
            }
            catch (Exception e)
            {
                Changed = true;
                Reason = $"Could not save settings: {e.Message}";
                return false;
            }
        }

        /// <summary>
        /// Find the serial ports on a host according to slightly better heuristics
        /// (on Unix) than the built-in SerialPort.GetPortNames() call.
        /// </summary>
        public static string[] GetHostSerialPorts()
        {
            var ports = new List<string>();

            // Always available, as a fallback...
            ports.Add("RSX:");

            if (PERQemu.HostIsUnix)
            {
                // Ha ha ha, this doesn't actually work.  I'm just now finding this out.
                // if (Environment.OSVersion.Platform == PlatformID.MacOSX)

                // Okay, try looking for the MacOS X-style names first...
                ports.AddRange(Directory.GetFiles("/dev", "cu.*usb*"));
                ports.AddRange(Directory.GetFiles("/dev", "tty.*usb*"));

                if (ports.Count == 1)
                {
                    // Nothin'?  Well, just add whatever Mono gives us
                    ports.AddRange(SerialPort.GetPortNames());
                }
            }
            else
            {
                // Assume Windows finds the appropriate COMn-style names
                ports.AddRange(SerialPort.GetPortNames());
            }

            return ports.ToArray();
        }

        /// <summary>
        /// Update the list of available Ethernet adapters (for the CLI).
        /// </summary>
        public static string[] GetHostEthernetNICs()
        {
            var nics = new List<string>();
            var interfaces = NetworkInterface.GetAllNetworkInterfaces();

            // Add "null" for the NullEthernet device (to boot Accent without Ethernet)
            nics.Add("null");

            foreach (NetworkInterface adapter in interfaces)
            {
                if (HostAdapter.IsEthernet(adapter.NetworkInterfaceType))
                {
                    nics.Add(adapter.Name);
                }
            }

            return nics.ToArray();
        }

        /// <summary>
        /// Gets a list of available audio drivers.
        /// </summary>
        public static string[] GetAudioDrivers()
        {
            var numDrivers = SDL.SDL_GetNumAudioDrivers();
            var drivers = new string[numDrivers + 1];

            drivers[0] = "default";

            for (var i = 0; i < numDrivers; i++)
            {
                drivers[i + 1] = SDL.SDL_GetAudioDriver(i);
            }

            return drivers;
        }

        /// <summary>
        /// Helper strings for parsing serial port option flags.
        /// </summary>
        public static string[] GetSerialOptionFlags()
        {
            return new string[] { "none", "xonxoff", "rtscts", "dcdforceon", "dcdfollowdsr" };
        }
    }
}
