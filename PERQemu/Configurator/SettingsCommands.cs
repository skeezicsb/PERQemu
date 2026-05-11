//
// SettingsCommands.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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

using PERQemu.IO.Z80;
using PERQemu.IO.Ports;

namespace PERQemu.UI
{
    /// <summary>
    /// CLI interface to the Option setting commands.  Program preferences are
    /// automatically saved and loaded in a platform-neutral format in a fixed
    /// location in the user's home directory.  Hint: the "format" is just the
    /// series of CLI commands required to set the desired options, read in and
    /// run automatically at startup. ;-)
    /// </summary>
    public class SettingsCommands
    {
        [Command("settings", "Enter the settings subsystem", Prefix = true)]
        public void SetSettingsPrefix()
        {
            PERQemu.CLI.SetPrefix("settings", SettingsChanged);
        }

        [Command("settings commands", "Show settings commands and their descriptions")]
        public void ShowSettingsCommands()
        {
            PERQemu.CLI.ShowCommands("settings");
        }

        [Command("settings done", "Exit settings mode, return to top-level")]
        public void SettingsDone()
        {
            CheckSerialPorts();
            PERQemu.CLI.ResetPrefix();
        }

        // Delegate for the CommandPrompt check
        public bool SettingsChanged()
        {
            return Settings.Changed;
        }

        [Command("settings show", "Show all program settings")]
        public void ShowSettings()
        {
            Console.WriteLine("Current settings:");
            Console.WriteLine("-----------------");
            Console.WriteLine($"Autosave harddisks on shutdown: {Settings.SaveDiskOnShutdown}");
            Console.WriteLine($"Autosave floppies on eject:     {Settings.SaveFloppyOnEject}");
            Console.WriteLine($"Autosave tapes on unload:       {Settings.SaveTapeOnUnload}");
            Console.WriteLine($"Pause execution after reset:    {Settings.PauseOnReset}");
            Console.WriteLine($"Pause when window minimized:    {Settings.PauseWhenMinimized}");
            Console.WriteLine($"Cursor in PERQ display window:  {Settings.CursorPreference}");
            Console.WriteLine();
            Console.WriteLine($"Rate limiting options:  {Settings.Performance}");
            Console.WriteLine();

            // Format/output options
            //Console.WriteLine($"Default radix for CPU debugger: {Settings.DebugRadix}");
            //Console.WriteLine($"Default radix for Z80 debugger: {Settings.Z80Radix}");
            Console.WriteLine($"Default output directory:   {Settings.OutputDirectory}");
            Console.WriteLine($"Screenshot file format:     {Settings.ScreenshotFormat}");
            Console.WriteLine($"Canon output file format:   {Settings.CanonFormat}");
            Console.WriteLine($"Canon default paper type:   {Settings.CanonPaperSize}");
            Console.WriteLine($"Canon default resolution:   {Settings.CanonResolution}dpi");
            Console.WriteLine();

            // Customizations
            if (Settings.RTCYearOffset != 1980)
                Console.WriteLine($"EIO RTC chip year offset:   {Settings.RTCYearOffset}");

            if (!string.IsNullOrEmpty(Settings.Keymap))
                Console.WriteLine($"Custom keyboard map:        {Settings.Keymap}");

            // Devices
            Console.WriteLine();
            Console.Write("Host audio device:          ");
            Console.WriteLine(Settings.AudioDevice == string.Empty ? "<default>" :
                              $"{Settings.AudioDevice}");
            Console.Write("Host serial port A device:  ");
            Console.WriteLine(Settings.RSADevice == string.Empty ? "<unassigned>" :
                              Settings.RSADevice == "RSX:" ? "RSX:" :
                              $"{Settings.RSADevice} {Settings.RSASettings.ToStringExt()}");
            Console.Write("Host serial port B device:  ");
            Console.WriteLine(Settings.RSBDevice == string.Empty ? "<unassigned>" :
                              $"{Settings.RSBDevice} {Settings.RSBSettings.ToStringExt()}");
            Console.Write("Host Ethernet device:       ");
            Console.WriteLine(Settings.EtherDevice == string.Empty ? "<unassigned>" :
                              $"{Settings.EtherDevice}");

            if (Settings.Changed)
            {
                Console.WriteLine("\nModified settings have not been saved.");
            }
        }

        [Command("settings default", "Reset all program settings to defaults")]
        public void SetDefaults()
        {
            Settings.Reset();
            QuietWrite(Settings.Reason);
        }

        [Command("settings load", "Reload saved settings")]
        public void LoadSettings()
        {
            Settings.Load();
            Console.WriteLine(Settings.Reason);
        }

        [Command("settings save", "Save current settings")]
        public void SaveSettings()
        {
            Settings.Save();
            Console.WriteLine(Settings.Reason);
        }

        #region General settings

        [Command("settings autosave harddisk", "Save harddisks on shutdown")]
        public void SetAutosaveHard(Ask doit)
        {
            if (doit != Settings.SaveDiskOnShutdown)
            {
                Settings.SaveDiskOnShutdown = doit;
                Settings.Changed = true;

                QuietWrite($"Autosave of hard disks is now {doit}.");
            }
        }

        [Command("settings autosave floppy", "Save modified floppy disks on eject")]
        public void SetAutosaveFloppy(Ask doit)
        {
            if (doit != Settings.SaveFloppyOnEject)
            {
                Settings.SaveFloppyOnEject = doit;
                Settings.Changed = true;

                QuietWrite($"Autosave of floppy disks is now {doit}.");
            }
        }

        [Command("settings autosave tape", "Save modified streamer tapes on unload")]
        public void SetAutosaveTape(Ask doit)
        {
            if (doit != Settings.SaveTapeOnUnload)
            {
                Settings.SaveTapeOnUnload = doit;
                Settings.Changed = true;

                QuietWrite($"Autosave of streamer tapes is now {doit}.");
            }
        }

        [Command("settings pause on reset", "Pause the emulator after a reset")]
        public void SetPauseOnReset(bool doit)
        {
            if (doit != Settings.PauseOnReset)
            {
                Settings.PauseOnReset = doit;
                Settings.Changed = true;

                QuietWrite($"Pause on reset is now {doit}.");
            }
        }

        [Command("settings pause when minimized", "Pause the emulator when the display window is minimized")]
        public void SetPauseWhenMinimized(bool doit)
        {
            if (doit != Settings.PauseWhenMinimized)
            {
                Settings.PauseWhenMinimized = doit;
                Settings.Changed = true;

                QuietWrite($"Pause when minimized is now {doit}.");
            }
        }

        [Command("settings display cursor", "Change the system cursor when in the display window")]
        public void SetCursorPref(Cursor curs)
        {
            if (curs != Settings.CursorPreference)
            {
                Settings.CursorPreference = curs;
                Settings.Changed = true;

                QuietWrite($"Cursor preference changed to {curs}.");
            }
        }

        [Command("settings rate limit default", "Set default rate limits")]
        public void SetPerfDefaults()
        {
            Settings.Performance = RateLimit.CPUSpeed | RateLimit.DiskSpeed | RateLimit.TapeSpeed;

            QuietWrite("Rate limit options set to defaults.");
        }

        [Command("settings rate limit", "Set rate limit option flags")]
        public void SetPerformance(RateLimit opt)
        {
            // "None" clears all the options; otherwise, they toggle
            if (opt == RateLimit.None)
            {
                Settings.Changed = (Settings.Performance != RateLimit.None);
                Settings.Performance = opt;
            }
            else
            {
                if (Settings.Performance.HasFlag(opt))
                {
                    Settings.Performance &= ~opt;
                }
                else
                {
                    Settings.Performance |= opt;
                }
                Settings.Changed = true;
            }

            QuietWrite($"Rate limit options set to {Settings.Performance}.");
        }

        #endregion

        #region Output settings

        [Command("settings output directory", "Set directory for saving printer output and screenshots")]
        public void SetOutputDir(string dir)
        {
            if (dir == string.Empty)
            {
                dir = Paths.OutputDir;      // Reset to default?  Hmm.
            }

            dir = Paths.Canonicalize(dir);

            if (dir != Settings.OutputDirectory)
            {
                Settings.OutputDirectory = dir;
                Settings.Changed = true;

                QuietWrite($"Default output directory is now '{dir}'.");
            }
        }

        [Command("settings canon resolution", "Set resolution (model) of Canon laser printer to simulate")]
        public void SetCanonResolution(int dpi)
        {
            if (dpi != 240 && dpi != 300)
            {
                QuietWrite($"{dpi}dots per inch is not valid.  Please choose 240 (LBP-10) or 300 (LBP-CX).");
                dpi = 300;
            }

            if (Settings.CanonResolution != dpi)
            {
                Settings.CanonResolution = (uint)dpi;
                Settings.Changed = true;

                QuietWrite($"Canon printer resolution set to {dpi}dpi.");
            }
        }

        [Command("settings canon paper size", "Set default paper size for the Canon laser printer")]
        public void SetCanonPaperType(IO.PaperCode size)
        {
            if (size != Settings.CanonPaperSize)
            {
                Settings.CanonPaperSize = size;
                Settings.Changed = true;
                QuietWrite($"Canon default paper size set to {size}.");
            }
        }

        [Command("settings canon output format", "Set image file format for Canon laser printer output")]
        public void SetCanonOutputFormat(ImageFormat format)
        {
            if (format != Settings.CanonFormat)
            {
                Settings.CanonFormat = format;
                Settings.Changed = true;
                QuietWrite($"Canon default output format set to {format}.");
            }
        }

        #endregion

        #region Serial device settings


        [Command("settings assign rs232a device", "Map a host serial device to port A")]
        public void SetRSADevice([KeywordMatch("ComPorts")] string hostDevice,
                     int baud = 9600, int data = 8,
                     Parity parity = Parity.None, StopBits stop = StopBits.One)
        {
            SetRS232Device('A', hostDevice, baud, data, parity, stop);
        }

        [Command("settings assign rs232b device", "Map a host serial device to port B")]
        public void SetRSBDevice([KeywordMatch("ComPorts")] string hostDevice,
                             int baud = 9600, int data = 8,
                             Parity parity = Parity.None, StopBits stop = StopBits.One)
        {
            SetRS232Device('B', hostDevice, baud, data, parity, stop);
        }

        [Command("settings assign rs232 device", Discreet = true)]  // Deprecated in v0.9.0
        public void SetRS232Device(char port, string hostDevice, int baud = 9600, int data = 8,
                                   Parity par = Parity.None, StopBits stop = StopBits.One)
        {
            // Sanity check the baud rate and character length values
            if (baud < 110 || baud > 38400)
            {
                baud = 9600;
                QuietWrite($"Baud rate {baud} out of range, reset to default.");
            }

            if (data < 5 || data > 8)
            {
                data = 8;
                QuietWrite($"Bits-per-character {data} out of range (5..8), reset to default.");
            }

            var dev = hostDevice;
            var devSettings = new SerialSettings(baud, data, par, stop);
            var curDev = string.Empty;
            SerialSettings curSettings;

            switch (port)
            {
                case 'a':
                case 'A':
                    curDev = Settings.RSADevice;
                    curSettings = Settings.RSASettings;
                    port = 'A';
                    break;

                case 'b':
                case 'B':
                    curDev = Settings.RSBDevice;
                    curSettings = Settings.RSBSettings;
                    port = 'B';
                    break;

                default:
                    Console.WriteLine($"Port {port} is invalid; please choose 'A' or 'B'.");
                    return;
            }

            if (CheckDevice(ref dev))
            {
                if (dev != curDev)
                {
                    if (port == 'A')
                    {
                        Settings.RSADevice = dev;
                        Settings.RSASettings = devSettings;
                    }
                    else
                    {
                        if (curDev == "RSX:")
                        {
                            QuietWrite("RSX: only works on port A.");
                            return;
                        }

                        Settings.RSBDevice = dev;
                        Settings.RSBSettings = devSettings;
                    }

                    Settings.Changed = true;
                    QuietWrite($"Device '{dev}' assigned to serial port {port}.");
                    return;
                }

                if (!devSettings.Equals(curSettings))
                {
                    if (port == 'A')
                    {
                        Settings.RSASettings = devSettings;
                    }
                    else
                    {
                        Settings.RSBSettings = devSettings;
                    }

                    Settings.Changed = true;
                    QuietWrite($"Serial port {port} settings changed.");
                }
                return;
            }

            Console.WriteLine($"Device '{dev}' invalid or not found; port {port} unchanged.");
        }

        [Command("settings unassign rs232a device", "Unmap the host serial device from port A")]
        public void UnSetRSADevice()
        {
            if (!string.IsNullOrEmpty(Settings.RSADevice))
            {
                Settings.RSADevice = string.Empty;
                Settings.Changed = true;
            }

            Console.WriteLine("Serial port A unassigned.");
        }

        [Command("settings unassign rs232b device", "Unmap the host serial device from port B")]
        public void UnSetRSBDevice()
        {
            if (!string.IsNullOrEmpty(Settings.RSBDevice))
            {
                Settings.RSBDevice = string.Empty;
                Settings.Changed = true;
            }

            Console.WriteLine("Serial port B unassigned.");
        }

        [Command("settings unassign rs232 device", Discreet = true)]    // Deprecated in v0.9.0
        public void UnSetRS232Device(char port = 'A')
        {
            switch (port)
            {
                case 'a':
                case 'A':
                    UnSetRSADevice();
                    return;

                case 'b':
                case 'B':
                    UnSetRSBDevice();
                    return;

                default:
                    Console.WriteLine($"Port {port} is invalid; please choose 'A' or 'B'.");
                    return;
            }
        }

        [Command("settings assign rs232a option", "Set or clear options for serial port A")]
        public void SetRSAOptions([KeywordMatch("SerialFlags")] string opt)
        {
            SetRS232Options('A', opt, ref Settings.RSASettings);
        }

        [Command("settings assign rs232b option", "Set or clear options for serial port B")]
        public void SetRSBOptions([KeywordMatch("SerialFlags")] string opt)
        {
            SetRS232Options('B', opt, ref Settings.RSBSettings);
        }

        /// <summary>
        /// Sets the RS-232 port flow control or option flags.
        /// </summary>
        void SetRS232Options(char port, string opt, ref SerialSettings settings)
        {
            int hs;

            switch (opt.ToLower())
            {
                case "none":
                    if ((settings.FlowControl != Handshake.None) ||
                        (settings.Options != SerialOptions.None))
                    {
                        settings.FlowControl = Handshake.None;
                        settings.Options = SerialOptions.None;

                        Settings.Changed = true;
                        QuietWrite($"Serial port {port} options reset.");
                    }
                    break;

                case "xonxoff":
                    // This is so bad it's good!  Well, no, it's really just bad.
                    hs = (int)settings.FlowControl ^ 1;
                    settings.FlowControl = (Handshake)hs;

                    Settings.Changed = true;
                    QuietWrite($"Serial port {port} flow control option now {settings.FlowControl}.");
                    break;

                case "rtscts":
                    hs = (int)settings.FlowControl ^ 2;
                    settings.FlowControl = (Handshake)hs;

                    Settings.Changed = true;
                    QuietWrite($"Serial port {port} flow control option now {settings.FlowControl}.");
                    break;

                case "dcdforceon":
                    if (settings.Options == SerialOptions.DCDForceOn)
                        settings.Options = SerialOptions.None;
                    else
                        settings.Options = SerialOptions.DCDForceOn;

                    Settings.Changed = true;
                    QuietWrite($"Serial port {port} carrier detect flag now {settings.Options}.");
                    break;

                case "dcdfollowdsr":
                    if (settings.Options == SerialOptions.DCDFollowDSR)
                        settings.Options = SerialOptions.None;
                    else
                        settings.Options = SerialOptions.DCDFollowDSR;

                    Settings.Changed = true;
                    QuietWrite($"Serial port {port} carrier detect flag now {settings.Options}.");
                    break;

                default:
                    Console.WriteLine($"Unknown RS-232 option '{opt}', ignored.");
                    return;
            }
        }

        /// <summary>
        /// Checks a host serial device specification.
        /// </summary>
        bool CheckDevice(ref string dev)
        {
            // Any host:  allow "rsx" or "rsx:", upcase it
            if (dev.ToUpper() == "RSX" || dev.ToUpper() == "RSX:")
            {
                dev = "RSX:";
                return true;
            }

            // If Unix, allow "devN" or /dev/devN" form; prepend /dev if not supplied
            if (PERQemu.HostIsUnix)
            {
                if (!dev.StartsWith("/dev", StringComparison.InvariantCulture))
                    dev = "/dev/" + dev;

                return File.Exists(dev);
            }

            // If Windows, expect 'COMn' form; the names should have been pre-
            // qualified by the KeywordMatch setter. ;-)
            if (dev.StartsWith("com", StringComparison.InvariantCultureIgnoreCase))
            {
                dev = dev.ToUpper();

                return true;    // Would have to do a serial .Open() to verify...
            }

            return false;
        }

        /// <summary>
        /// Checks the serial port assignments to make sure the same device isn't
        /// assigned to both channels.
        /// </summary>
        /// <remarks>
        /// For now (?) this is a warning only, since there's no harm in assigning
        /// port B to the same device as port A when initializing a PERQ-1 -- since
        /// there is no port B so it can't conflict.  But with PERQ-2/EIO configs,
        /// opening the same physical port twice will either fail outright, or act
        /// very strangely...
        /// </remarks>
        void CheckSerialPorts()
        {
            // If either (or both) is empty, or they don't match, no conflict!
            if (string.IsNullOrEmpty(Settings.RSADevice) ||
                string.IsNullOrEmpty(Settings.RSBDevice) ||
               Settings.RSADevice != Settings.RSBDevice)
                return;

            // If they do match issue a warning
            Log.Warn(Category.All,
                     "Note: Both RS-232 ports assigned to the same device; some configurations\n" +
                     "might not load properly.  Please check your settings to reassign ports.");
        }

        #endregion

        #region Audio/speech settings

        [Command("settings assign audio device", "Set the host audio driver to use")]
        public void SetAudioDev([KeywordMatch("AudioDrivers")] string hostDevice)
        {
            if (hostDevice == "default")
            {
                UnSetAudioDev();
                return;
            }

            if (Settings.AudioDevice != hostDevice)
            {
                Settings.AudioDevice = hostDevice;
                Settings.Changed = true;
            }

            QuietWrite($"Audio device set to '{hostDevice}'.");
        }

        [Command("settings unassign audio device", "Use the host's default audio driver")]
        public void UnSetAudioDev()
        {
            if (!string.IsNullOrEmpty(Settings.AudioDevice))
            {
                Settings.AudioDevice = string.Empty;
                Settings.Changed = true;
            }

            QuietWrite("Audio device reset to default.");
        }

        [Command("settings assign audio option", "Fine tune the CVSD audio output parameters")]
        public void TuneAudio(AudioKnobs knob, int val)
        {
            // CLI doesn't handle floating pt inputs, yet?
            var scaled = (val * .0001);
            var changed = $"{knob} = {scaled:N4}";

            switch (knob)
            {
                case AudioKnobs.Channels:
                    if (val < 1 || val > 2)
                    {
                        Console.WriteLine($"Audio channels {val} out of range (1..2), ignored.");
                        return;
                    }

                    if (Settings.AudioSettings.Channels != (byte)val)
                    {
                        Settings.AudioSettings.Channels = (byte)val;
                        changed = $"Channels = {val} " + (val == 2 ? "(stereo)" : "(mono)");
                    }
                    break;

                case AudioKnobs.Min:
                    Settings.AudioSettings.FilterMin = scaled;
                    break;

                case AudioKnobs.Max:
                    Settings.AudioSettings.FilterMax = scaled;
                    break;

                case AudioKnobs.Decay:
                    Settings.AudioSettings.FilterDecayTC = scaled;
                    break;

                case AudioKnobs.Charge:
                    Settings.AudioSettings.FilterChargeTC = scaled;
                    break;

                case AudioKnobs.Leak:
                    Settings.AudioSettings.IntegratorLeakTC = scaled;
                    break;

                case AudioKnobs.Gain:
                    Settings.AudioSettings.SampleGain = val / 32768.0;
                    break;

                default:
                    Console.WriteLine($"Unknown audio parameter '{knob}', ignored.");
                    return;
            }

            // Rather than fuss over floating point precision, assume the value
            // has changed.  Other than channels, nobody will likely mess with these
            Settings.Changed = true;
            QuietWrite($"Audio settings changed:  {changed}");

            // Note: If the machine is running, changes will be picked up when the
            // next sample starts playing (at filter reset)
        }

        [Command("settings assign audio option default", "Reset audio output parameters to defaults")]
        public void ResetAudio()
        {
            if (!Settings.AudioSettings.Equals(SpeechSettings.Defaults))
            {
                Settings.AudioSettings = SpeechSettings.Defaults;
                Settings.Changed = true;
                Console.WriteLine("Audio settings reset to defaults.");
            }
        }

        #endregion

        #region Network settings

        [Command("settings show ethernet devices", "List available host Ethernet interfaces")]
        public void ShowEtherDevices()
        {
            IO.Network.HostAdapter.ShowInterfaceSummary();
        }

        [Command("settings assign ethernet device", "Map a host network adapter to the PERQ Ethernet device")]
        public void SetEtherDev([KeywordMatch("NICs")] string hostDevice)
        {
            if (hostDevice != Settings.EtherDevice)
            {
                Settings.EtherDevice = hostDevice;
                Settings.Changed = true;
                QuietWrite($"Host adapter '{hostDevice}' assigned to the PERQ Ethernet device.");
            }
        }

        [Command("settings unassign ethernet device", "Unmap a host network adapter (disable PERQ Ethernet)")]
        public void UnsetEtherDev()
        {
            if (!string.IsNullOrEmpty(Settings.EtherDevice))
            {
                Settings.EtherDevice = string.Empty;
                Settings.Changed = true;
            }

            QuietWrite("Ethernet device unassigned.");
        }

        #endregion

        #region Customizations

        [Command("settings rtc offset", "Set the base year for the EIO RTC chip")]
        public void SetRTCOffset(int year)
        {
            // Chip only stores 2 digits, so range check
            var offset = DateTime.Now.Year - year;

            if (offset < 0 || offset > 99)
            {
                Console.WriteLine($"Year offset invalid; must be between 1980 and {DateTime.Now.Year}.");
                return;
            }

            if (year != Settings.RTCYearOffset)
            {
                Settings.RTCYearOffset = year;
                Settings.Changed = true;
                QuietWrite($"EIO RTC Offset is now {year}.");
            }
        }

        [Command("settings keymap", "Set a default custom keyboard map")]
        public void SetKeymap([KeywordMatch("Keymaps")] string mapName)
        {
            if (mapName.ToLower() == "default" || mapName.ToLower() == "none")
            {
                mapName = string.Empty;
            }

            if (mapName != Settings.Keymap)
            {
                Settings.Keymap = mapName;
                Settings.Changed = true;

                if (string.IsNullOrEmpty(mapName))
                    QuietWrite("No custom keyboard map will be applied.");
                else
                    QuietWrite($"Custom keyboard map is now '{mapName}'.");
            }

        }

        #endregion

        // Pure cheese.  Don't spew messages when reading on startup.
        void QuietWrite(string s)
        {
            if (PERQemu.Initialized) Console.WriteLine(s);
        }
    }
}

/*
	Todo:
	settings::screenshot format [jpg, png, tiff, ?]
	settings::screenshot template [str] -- really?  cmon...
	settings::canon template [str]      -- same 
	settings::logging directory         -- default: Output/
	settings::logging template [str]    -- hmm.
	settings::logging keep [n]          -- how many files
	settings::logging filesize [n]      -- in mb?  kb?

	Host interface to the network, serial and audio output devices
	is globally set for all virtual machines:

	settings::ethernet encapsulation [raw, udp, ???]
*/
