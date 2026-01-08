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

using PERQemu;
using PERQemu.UI;

namespace PERQemu.IO.Z80
{
    // for debugging, make these tunable
    public enum AudioKnobs
    {
        Max,
        Min,
        Decay,
        Charge,
        Leak,
        Gain
    }

    /// <summary>
    /// Simulate the Motorola MC3417 Continuously Variable Slope Delta Modulator
    /// chip.  Some implementation notes are at the bottom of the file (for now).
    /// </summary>
    public class MC3417 : ISIODevice, ICTCDevice
    {
        public MC3417()
        {
            _frequency = 16000;
            _samples = new short[8];
            _speaker = PERQemu.GUI.Audio;

            Log.Info(Category.Speech, "Knobs: charge {0:N4}, decay {1:N4}, leak {2:N4}, gain {3:N4}",
                                       FilterChargeTC, FilterDecayTC, IntegratorLeakTC, SampleGain);
        }

        public void Reset()
        {
            ResetFilter(_frequency);

            // Reset the audio device, too
            _speaker.Reset();

            Log.Info(Category.Speech, "MC3417 reset");
        }

        public void ResetFilter(int frequency)
        {
            // Set the new audio frequency
            _frequency = frequency;

            // Recompute
            _charge = Math.Pow(Math.Exp(-1.0), 1.0 / (FilterChargeTC * _frequency));
            _decay = Math.Pow(Math.Exp(-1.0), 1.0 / (FilterDecayTC * _frequency));
            _leak = Math.Pow(Math.Exp(-1.0), 1.0 / (IntegratorLeakTC * _frequency));

            // Now cheat the transfer rate based on our emulation speed :-)
            _txRate = Conversion.BaudRateToNsec(_frequency);

            if (PERQemu.Sys.Display.AverageFPS < 60)
            {
                // Too slow?
                var offset = PERQemu.Sys.Display.AverageFPS / 60;
                _txRate = (ulong)(_txRate * offset);
                Log.Info(Category.Speech, "Adjusting tx pacing by {0:N4}", offset);
            }

            // Prime for playback
            _sylFilter = _intFilter = 0.0;
            _shiftReg = 0;
        }

        //
        // ICTCDevice implementation
        //

        /// <summary>
        /// Handle baud rate changes, which affects playback sample rate.  This
        /// is usually called just before and after a sample is played.  Inform
        /// the audio device and let it adjust if necessary; reset our filter
        /// parameters for the next sample.  For now we only handle 16K or 32K
        /// sample rates (those are the only example files we have).
        /// </summary>
        public void NotifyRateChange(int chan, int newRate)
        {
            var prescale = PERQemu.Sys.IOB.IsEIO ? 1 : 16;
            var frequency = Conversion.TimerCountToBaudRate(newRate, prescale);

            if (frequency == 0)
            {
                Log.Warn(Category.Speech, "Bad baud rate request ignored: {0}", newRate);
                return;
            }

            Log.Info(Category.Speech, "Baud rate change: divider {0}, frequency {1}", newRate, frequency);
            _speaker.RateChange(frequency);
            ResetFilter(frequency);
        }

        //
        // ISIODevice implementation
        //

        public ulong TransmitRate => _txRate;
        public ulong ReceiveRate => 0;

        /// <summary>
        /// "Transmit" a byte from the SIO to the CVSD chip.
        /// </summary>
        public void Transmit(byte value)
        {
            // If speech output is disabled, don't bother
            if (!_speaker.HaveAudio) return;

            Log.Verbose(Category.Speech, "CVSD input byte 0x{0:x2}", value);
            ConvertCVSDtoPCM(value);
        }

        // Mux doesn't pass these through
        public void TransmitBreak()
        {
            throw new NotImplementedException("TransmitBreak on Speech");
        }

        public void RegisterReceiveDelegate(ReceiveDelegate rxDelegate)
        {
            throw new NotImplementedException("RegisterReceive on Speech");
        }


        /// <summary>
        /// Convert a byte of an incoming CVSD bitstream into 8 PCM samples and
        /// queue them up for ouput to the audio device.
        /// </summary>
        void ConvertCVSDtoPCM(byte input)
        {
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
                    _sylFilter = FilterMax - ((FilterMax - _sylFilter) * _charge);

                    if (_sylFilter > FilterMax)
                        _sylFilter = FilterMax;
                }
                else
                {
                    _sylFilter *= _decay;

                    if (_sylFilter < FilterMin)
                        _sylFilter = FilterMin;
                }

                // Scale and chop to 16 signed bits
                _samples[i] = (short)((_intFilter * SampleGain) * 32768.0);
            }

            // Send the samples to the Speaker device
            _speaker.QueueSamples(ref _samples, 8);
        }

        public void SetTunable(AudioKnobs knob, int val)
        {
            var scaled = (double)(val * .0001);

            // CLI doesn't handle floating pt inputs, yet?
            switch (knob)
            {
                case AudioKnobs.Min:
                    FilterMin = scaled;
                    break;

                case AudioKnobs.Max:
                    FilterMax = scaled;
                    break;

                case AudioKnobs.Decay:
                    FilterDecayTC = scaled;
                    break;

                case AudioKnobs.Charge:
                    FilterChargeTC = scaled;
                    break;

                case AudioKnobs.Leak:
                    IntegratorLeakTC = scaled;
                    break;

                case AudioKnobs.Gain:
                    SampleGain = val / 32768.0;
                    break;
            }

            ResetFilter(_frequency);
        }

        // Constants and tweakables
        const short Mask = 0x7;         // 3417 is a 3-bit device (3418 is 4 bits)

        // debug: turn these to private, const when set
        public double FilterMax = 1.500;                 // 1.0954
        public double FilterMin = 0.020;                 // 0.416
        public double FilterDecayTC = 0.005;             // 0.004
        public double FilterChargeTC = 0.003;            // 0.004
        public double IntegratorLeakTC = 0.001;          // 0.001
        public double SampleGain = (10000.0 / 32768.0);  // 10000


        // Working set
        short[] _samples;

        int _shiftReg;
        int _frequency;

        ulong _txRate;

        double _leak;
        double _decay;
        double _charge;
        double _sylFilter;
        double _intFilter;

        // Save typing
        Speaker _speaker;
    }
}
