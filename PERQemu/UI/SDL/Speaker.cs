//
// Speaker.cs - Copyright (c) 2006-2026 Josh Dersch (derschjo@gmail.com)
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

using SDL2;

using System;
using System.IO;

namespace PERQemu.UI
{
    /// <summary>
    /// Provides the interface to the SDL2 Audio subsystem for the PERQ's
    /// "speech" device.  Speaker buffers PCM output from the MC3417 and plays
    /// it on the host's default audio device.
    /// </summary>
    public class Speaker
    {
        public Speaker()
        {
            _devId = 0;
            _devChannels = 1;           // Assume default mono sound
            _channels = 1;              // Can be changed in Settings
            _devFrequency = 32000;      // Assume the initial Kriz baud rate
            _frequency = 16000;         // Assume the default Speech baud rate

            _paused = true;
            _enabled = true;
            _logging = false;

            // Allocate once and set fixed params
            _desired = new SDL.SDL_AudioSpec();
            _desired.format = SDL.AUDIO_S16SYS; // Signed 16-bit PCM samples
            _desired.samples = 1024;            // Audio buffer size in samples
            _desired.callback = null;           // No callback function
            _desired.userdata = IntPtr.Zero;    // No user data
        }

        public bool HaveAudio => _devId > 0;
        public long DevBytes => HaveAudio ? SDL.SDL_GetQueuedAudioSize(_devId) : 0;

        public bool Enabled
        {
            get { return _enabled; }
            set { _enabled = value; }
        }

        /// <summary>
        /// Returns the current/active frequency.  Buffers rate changes from the
        /// CVSD chip so that a current sample will complete without distortion;
        /// the device will be reopened with the new rate at the next pause.
        /// </summary>
        public int Frequency
        {
            get { return _frequency; }
            set { _devFrequency = value; }
        }

        /// <summary>
        /// Return the current/active number of channels.  Buffers changes the
        /// same as Frequency so the new setting takes effect at the next pause.
        /// </summary>
        public byte Channels
        {
            get { return _channels; }
            set { _devChannels = value; }
        }

        /// <summary>
        /// Reset this instance: pause and clear.  Called by the MC3417 when the
        /// PERQ resets SpeechOut or the Z80 powers up.
        /// </summary>
        public void Reset()
        {
            Pause();
        }

        /// <summary>
        /// Open the default audio device.  Initially paused.
        /// </summary>
        public void Initialize()
        {
            if (!_enabled) return;

            if (_devId == 0)
            {
                // Set the tunable audio options
                _desired.freq = _devFrequency;      // PERQ sets this
                _desired.channels = _devChannels;   // User can set

                _devId = SDL.SDL_OpenAudioDevice(IntPtr.Zero, 0, ref _desired, out _spec, 0);

                if (_devId == 0)
                {
                    Log.Warn(Category.UI, "Could not open audio device; no speech output available.");
                    return;
                }

                Log.Info(Category.Speech, "Requested: format {0}, freq {1}, chan {2}, samples {3}",
                                            _desired.format, _desired.freq, _desired.channels, _desired.samples);
                Log.Info(Category.Speech, "Obtained:  format {0}, freq {1}, chan {2}, samples {3}, size {4}",
                                           _spec.format, _spec.freq, _spec.channels, _spec.samples, _spec.size);

                if (_spec.format != _desired.format) Log.Warn(Category.Speech, "Audio device cannot match requested format!");
                // Just continue and hope for the best?

                if (_spec.freq != _devFrequency) Log.Warn(Category.Speech, "Audio device could not match requested frequency");
                _frequency = _spec.freq;

                if (_spec.channels != _devChannels) Log.Warn(Category.Speech, "Audio device could not match requested channels");
                _channels = _spec.channels;

                // Recompute the threshold (in ms) for the idle check
                // (I know, I know, don't think hard about this, just go with it)
                _startDelay = _frequency / (_channels * 2);
                _idleThreshold = 100;
            }

            if (_logging) _log.WriteLine($"0,{HighResolutionTimer.ElapsedHiRes()},Initialize called");
            Log.Info(Category.UI, "Audio device ID {0} open for playback", _devId);

            Reset();
        }

        /// <summary>
        /// Pause playback.  Clears the stream buffer and audio device queue just
        /// to be sure (remove artifacts from previous samples, if any).
        /// </summary>
        public void Pause()
        {
            if (HaveAudio && !_paused)
            {
                SDL.SDL_PauseAudioDevice(_devId, 1);
                _paused = true;

                Log.Info(Category.Speech, "Audio PAUSED");    // debug
            }
        }

        /// <summary>
        /// Resume playback.
        /// </summary>
        public void Resume()
        {
            if (HaveAudio && _paused)
            {
                SDL.SDL_PauseAudioDevice(_devId, 0);
                _paused = false;

                Log.Info(Category.Speech, "Audio RESUMED");   // debug
            }
        }

        /// <summary>
        /// Queue a block of samples from the MC3417 for playback.  We let SDL2
        /// do all of the buffering.
        /// </summary>
        public void QueueSamples(ref short[] samples)
        {
            // Nothing queued and parameters changed?
            if (DevBytes == 0 && ((_frequency != _devFrequency) || (_channels != _devChannels)))
            {
                // Debug: should only do this when paused!
                if (!_paused) Log.Warn(Category.Speech, "Settings change while not idle!");

                // Close and reopen the device with the new settings
                Shutdown();
                Initialize();
            }

            // Silently discard if we haven't got anywhere to send 'em
            if (!HaveAudio) return;

            _lastSampleRcvd = HighResolutionTimer.ElapsedHiRes();

            // Hand off the samples to SDL's audio buffer directly
            unsafe
            {
                fixed (short* p = samples)
                {
                    IntPtr ptr = (IntPtr)p;

                    if (SDL.SDL_QueueAudio(_devId, ptr, (uint)(samples.Length * 2)) < 0)
                    {
                        Log.Warn(Category.Speech, "QueueAudio failed: {0}", SDL.SDL_GetError());
                    }
                }
            }

            if (_logging) _log.WriteLine($"1,{_lastSampleRcvd},{DevBytes}");

            Log.Debug(Category.Speech, "Queued {0} samples @ time {1}", samples.Length, _lastSampleRcvd);
        }

        /// <summary>
        /// Manage the state of the audio device: resume or pause playback as
        /// required based on buffer status.
        /// </summary>
        public void CheckIdle()
        {
            var now = HighResolutionTimer.ElapsedHiRes();
            var delta = now - _lastSampleRcvd;
            var buffered = DevBytes;

            if (_logging) _log.WriteLine($"2,{now},{buffered},{(_paused ? 1 : 0)}");

            // Resume playback?
            if (_paused && (((delta > _idleThreshold) && (buffered > 0)) || (buffered > _startDelay)))
            {
                // Short sample, or enough data buffered up to begin!
                Resume();
                return;
            }

            // Pause playback?
            if (!_paused && (delta > _idleThreshold) && (buffered == 0))
            {
                // Not paused but the data stopped and the queue has played out
                Pause();
            }

            // In all other cases, just continue
        }

        /// <summary>
        /// Sort of a "soft" shutdown.  Closes the device and resets.
        /// </summary>
        public void Shutdown()
        {
            if (_logging) _log.WriteLine($"0,{HighResolutionTimer.ElapsedHiRes()},Shutdown called");

            if (HaveAudio)
            {
                SDL.SDL_ClearQueuedAudio(_devId);
                SDL.SDL_CloseAudioDevice(_devId);
                _devId = 0;

                Log.Info(Category.UI, "Audio device closed");
            }
        }

        // Debugging
        public void Status()
        {
            if (!HaveAudio)
            {
                Console.WriteLine("Audio device is not " + (_enabled ? "available." : "enabled."));
                return;
            }

            var delta = HighResolutionTimer.ElapsedHiRes() - _lastSampleRcvd;
            var stat = SDL.SDL_GetAudioDeviceStatus(_devId);
            var bytes = SDL.SDL_GetQueuedAudioSize(_devId);
            var samples = bytes / (2 * _channels);

            Console.WriteLine("Audio status:");
            Console.WriteLine("  Device: ID: {0}  Frequency: {1:N1}kHz  Channels: {2}  Status: {3}",
                              _devId, _devFrequency / 1000.0, _devChannels, stat);
            Console.WriteLine("   Input: {0:N1}kHz  Last sample: {1}ms  Late threshold: {2:N3}ms",
                              _frequency / 1000.0, delta, _idleThreshold);
            Console.WriteLine("  Output: Queued: {0} bytes ({1} samples)  Paused: {2}",
                              bytes, samples, _paused);
        }

        // Extended debugging
        public void Telemetry(bool enable)
        {
            if (enable)
            {
                if (_logging) return;

                var path = Paths.BuildOutputPath("Speech-telemetry.log");
                _log = File.AppendText(path);
                _log.WriteLine("0,{0},Logging started at {1}",
                               HighResolutionTimer.ElapsedHiRes(), DateTime.Now.ToString());
                _logging = true;

                Console.WriteLine($"Opened {path} for Speech telemetry logging.");
                return;
            }

            // Disable
            if (!_logging) return;

            _logging = false;
            _log.WriteLine("0,{0},Logging stopped at {1}",
                           HighResolutionTimer.ElapsedHiRes(), DateTime.Now.ToString());
            _log.Flush();
            _log.Close();

            Console.WriteLine("Speech telemetry log closed.");
        }

        uint _devId;

        byte _channels;
        byte _devChannels;

        int _frequency;
        int _devFrequency;

        bool _enabled;
        bool _paused;

        int _startDelay;            // in bytes
        uint _idleThreshold;        // in ms
        double _lastSampleRcvd;

        SDL.SDL_AudioSpec _desired;
        SDL.SDL_AudioSpec _spec;

        // Extended debugging
        protected bool _logging;
        protected StreamWriter _log;
    }
}
