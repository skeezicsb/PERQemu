//
// MC3417.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
using System.IO;

using PERQemu.UI;
using PERQemu.IO.SerialDevices;

namespace PERQemu.IO.Z80
{
    /// <summary>
    /// Audio knobs, for frobbing.
    /// </summary>
    public enum AudioKnobs
    {
        Min,
        Max,
        Decay,
        Charge,
        Leak,
        Gain,
        Channels
    }

    /// <summary>
    /// Customizable speech device settings.  Allows fine tuning of the CVSD to
    /// PCM conversion parameters, output channels, possibly more.
    /// </summary>
    public struct SpeechSettings
    {
        public SpeechSettings(double min, double max, double decay, double charge,
                             double leak, double gain, byte chan = 1)
        {
            FilterMin = min;
            FilterMax = max;
            FilterDecayTC = decay;
            FilterChargeTC = charge;
            IntegratorLeakTC = leak;
            SampleGain = (gain / 32768.0);
            Channels = chan;
        }

        public double FilterMin;
        public double FilterMax;
        public double FilterDecayTC;
        public double FilterChargeTC;
        public double IntegratorLeakTC;
        public double SampleGain;

        public byte Channels;

        public override string ToString()
        {
            return string.Format("Min/Max {0:N4}/{1:N4}, Decay {2:N4}, Charge {3:N4}, Leak {4:N4}, Gain {5:N4}",
                                 FilterMin, FilterMax, FilterDecayTC, FilterChargeTC, IntegratorLeakTC, SampleGain);
        }

        // PERQemu defaults, based on frobbing the knobs
        public readonly static SpeechSettings Defaults = new SpeechSettings(0.020, 1.500, 0.005, 0.003, 0.001, 10000.0);

        // CVSD to PCM baseline settings, based on some old reference code?
        public readonly static SpeechSettings Reference = new SpeechSettings(0.416, 1.0954, 0.004, 0.004, 0.001, 10000.0);
    }

    /// <summary>
    /// Simulate the Motorola MC3417 Continuously Variable Slope Delta Modulator
    /// chip.  Implementation notes are in Docs/SerialPorts.txt.
    /// </summary>
    public class MC3417 : SerialDevice
    {
        public MC3417(Z80System sys) : base(sys)
        {
            _name = "MC3417";
            _channels = 1;
            _frequency = _nextFrequency = 16000;
            _samples = new short[8];
            _speaker = PERQemu.GUI.Audio;
            _settings = SpeechSettings.Defaults;
        }

        public override bool WriteReady => true;

        public override void Reset()
        {
            ResetFilter();

            _speaker.Enabled = PERQemu.Config.Current.SpeechEnabled;
            _speaker.Reset();

            Log.Info(Category.Speech, "MC3417 reset");
        }

        public void ResetFilter()
        {
            // Pick up tuning changes
            _settings = Settings.AudioSettings;

            // Update new audio frequency, channels if changed
            _frequency = _nextFrequency;
            _channels = _settings.Channels;

            // Poke the device
            _speaker.Frequency = _frequency;
            _speaker.Channels = _channels;

            // Recompute
            _charge = Math.Pow(Math.Exp(-1.0), 1.0 / (_settings.FilterChargeTC * _frequency));
            _decay = Math.Pow(Math.Exp(-1.0), 1.0 / (_settings.FilterDecayTC * _frequency));
            _leak = Math.Pow(Math.Exp(-1.0), 1.0 / (_settings.IntegratorLeakTC * _frequency));

            // Size the buffer for mono/stereo
            _samples = new short[_channels * 8];

            // Compute sync mode rate (8 bits per char)
            _txRate = Conversion.BaudRateToNsec(_frequency, 8);

            // Now cheat the transfer rate based on our emulation speed :-)
            // Todo: make the cheat a Settings/RateLimit option
            var offset = PERQemu.Sys.Display.AverageFPS / 60.0;

            // Too slow?  For now, ignore if too fast :-)
            if (offset < 1.0)
            {
                _txRate = (ulong)(_txRate * offset);
                Log.Info(Category.Speech, "Adjusting tx pacing by {0:N4}", offset);
            }

            // Poll every 1/10th of a second (fixed for now)
            _pollRate = 100 * Conversion.MsecToNsec;

            Log.Info(Category.Speech, "Tx pacing at {0:N4}ms/char, polling {1:N4}ms",
                                      _txRate * Conversion.NsecToMsec,
                                      _pollRate * Conversion.NsecToMsec);

            // Debug
            _byteCount = 0;
            _startTime = 0;
            _lastTime = 0;

            // Prime for playback
            _sylFilter = _intFilter = 0.0;
            _shiftReg = 0;
        }

        /// <summary>
        /// Handle baud rate changes, which affects playback sample rate.  This
        /// is usually called just before and after a sample is played.  Inform
        /// the audio device and let it adjust if necessary; reset our filter
        /// parameters for the next sample.  For now we only handle 16K or 32K
        /// sample rates (those are the only example files we have).
        /// </summary>
        public override void NotifyRateChange(int chan, int newRate)
        {
            var prescale = _system.IsEIO ? 1 : 16;
            _nextFrequency = Conversion.TimerCountToBaudRate(newRate, prescale);

            if (_nextFrequency == 0)
            {
                Log.Warn(Category.Speech, "Bad baud rate request ignored: {0}", newRate);
                _nextFrequency = _frequency;    // restore
                return;
            }

            Log.Debug(Category.Speech, "Baud rate change: divider {0}, frequency {1}", newRate, _nextFrequency);
            ResetFilter();
        }

        /// <summary>
        /// "Transmit" a byte from the SIO to the CVSD chip.
        /// </summary>
        public override void Transmit(byte value)
        {
            // If speech output is disabled, don't bother
            if (!_speaker.HaveAudio) return;

            // Silence detect: count sync bytes sent by the SIO; several in a row
            // likely means the sample is finished playing, so we stop sending
            // samples to the speaker.  This should help reduce audible "pops"
            _silence = (value == SyncByte) ? _silence + 1 : 0;

            if (_silence > Threshold) return;

            // Debug
            if (_byteCount == 0)
            {
                _startTime = _scheduler.CurrentTimeNsec;
            }
            _byteCount++;
            _lastTime = _scheduler.CurrentTimeNsec;

            Log.Verbose(Category.Speech, "CVSD input byte 0x{0:x2}", value);
            ConvertCVSDtoPCM(value);
        }

        /// <summary>
        /// Convert a byte of an incoming CVSD bitstream into 8 PCM samples and
        /// queue them up for ouput to the audio device.
        /// </summary>
        void ConvertCVSDtoPCM(byte input)
        {
            int j = 0;
            short s = 0;

            for (int i = 0; i < 8; i++)
            {
                var bit = input & 0x1;
                input >>= 1;

                _shiftReg = (_shiftReg << 1) | bit;

                // Move the estimator up or down a step
                if (bit == 0)
                    _intFilter += _sylFilter;
                else
                    _intFilter -= _sylFilter;

                // Simulate leakage (eww)
                _intFilter *= _leak;

                // If the last n bits are all 0 or 1, bump the step
                if (((_shiftReg & Mask) == 0) || (_shiftReg & Mask) == Mask)
                {
                    // Coincidence is true
                    _sylFilter = _settings.FilterMax - ((_settings.FilterMax - _sylFilter) * _charge);

                    if (_sylFilter > _settings.FilterMax)
                        _sylFilter = _settings.FilterMax;
                }
                else
                {
                    _sylFilter *= _decay;

                    if (_sylFilter < _settings.FilterMin)
                        _sylFilter = _settings.FilterMin;
                }

                // Scale and chop to 16 signed bits
                s = (short)((_intFilter * _settings.SampleGain) * 32768.0);

                _samples[j++] = s;                      // mono channel
                if (_channels == 2) _samples[j++] = s;  // copy to 2nd channel
            }

            // Send the samples to the Speaker device
            _speaker.QueueSamples(ref _samples);
        }

        /// <summary>
        /// Check playback status and resume or pause the output device.
        /// </summary>
        /// <remarks>
        /// Since this is a pseudo device that doesn't need to update the SIO's
        /// read registers, we always return false.  It's just convenient to use
        /// the Poll() call here on the Z80 thread rather than schedule a separate
        /// CheckIdle event (or use the SDL event loop).
        /// </remarks>
        public override bool Poll()
        {
            _speaker.CheckIdle();
            return false;
        }


        // Debugging
        public override void Status()
        {
            // How fast is the Z80 delivering bytes?
            var interval = _byteCount > 0 ? ((_lastTime - _startTime) / _byteCount) : 0.0;

            Console.WriteLine("MC3417/Speech status:");
            Console.WriteLine("  Tx pacing at {0:N4}ms/char, polling {1:N4}ms, avg. byte time {2:N4}ms",
                                 _txRate * Conversion.NsecToMsec,
                                 _pollRate * Conversion.NsecToMsec,
                                 interval * Conversion.NsecToMsec);
            Console.WriteLine("  Filter: " + _settings.ToString());
        }

        // Extended debugging
        public override void Telemetry(bool enable, ref StreamWriter file)
        {
            base.Telemetry(enable, ref file);
            if (_logging) _log.WriteLine($"0,{_scheduler.CurrentTimeNsec},MC3417 telemetry enabled");

            _speaker.Telemetry(enable);
        }

        // Constants
        const short Mask = 0x7;         // 3417 is a 3-bit device (3418 is 4 bits)
        const byte SyncByte = 0xaa;     // Silent byte, holy byte
        const int Threshold = 32;       // Stop encoding after n sync bytes in a row

        // Tunables
        SpeechSettings _settings;

        // Working set
        short[] _samples;

        byte _channels;

        int _shiftReg;
        int _frequency;
        int _nextFrequency;
        int _silence;

        double _leak;
        double _decay;
        double _charge;
        double _sylFilter;
        double _intFilter;

        ulong _byteCount;
        ulong _startTime;
        ulong _lastTime;

        // Save typing
        Speaker _speaker;
    }
}
