//
// Speaker.cs - Copyright (c) 2006-2025 Josh Dersch (derschjo@gmail.com)
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
    /// <remarks>
    /// We manage the actual playback by tracking deltas (in real time) between
    /// updates to the sample buffer.  The upstream driver (MC3417.cs) doesn't
    /// have any way to know when the stream has stopped on purpose or the PERQ
    /// just can't keep up, so we use a window of around 1/2 buffer's worth to
    /// decide to pause playback.  The idea is to not waste time having the SDL
    /// audio thread send silence when PERQ "speech" is so sporadic.  Similarly,
    /// we only unpause after a full buffer, to let the PERQ get a little head
    /// start and cover over any flutter in execution rate.  In theory, anyway.
    /// 
    /// Thus, the buffer length is a tradeoff between lag (on fast hosts) and
    /// avoiding gaps and stutter on slow ones.  May have to make this a tunable
    /// Setting?
    /// </remarks>
    public class Speaker
    {
        public Speaker()
        {
            _devId = 0;
            _frequency = _newFrequency = 32000;

            _count = 0;
            _bufSize = 1024;
            _buffer = new short[_bufSize];

            _paused = true;
        }

        public bool HaveAudio => _devId > 0;
        public bool Buffering => _count > 0;
        public bool Paused => !HaveAudio || _paused;
        public bool Playing => HaveAudio && !Paused;
        public bool Busy => HaveAudio && (!_paused || _count > 0);

        // Big knobs
        public int SampleRate => _frequency;
        public ushort BufferSize => _bufSize;

        //
        // Magic numbers FTW!
        //
        // Buflen / 8 = bytes * .5ms per char @ 16kbaud / 2 for safety margin
        // So, SDL recommends the sample buffer be sized in powers of 2 up to
        // 4096 max; a lot of the PERQ software sends blocks of 512 bytes in
        // one UnitIO call, or 64 bytes for short beeps -- so empirical study
        // and some back of the envelope scratchings arrive at a buffer size
        // of 512-1K samples (64-128 bytes) ought to be the sweet spot.  Time
        // and testing will tell.  
        //
        public ulong NominalRateInMsec => (ulong)(_bufSize / 32);

        /// <summary>
        /// Open the default audio device, pause playback, and clear the sample
        /// buffer.  Uses the fixed PERQ default 16kHz sample rate, mono.
        /// </summary>
        public void Initialize()
        {
            // Of two minds.  Could close and reinit, say, if the user Settings
            // changed and a different device was specified.  Or just keep the
            // existing device and just reset...

            if (_devId == 0)
            {
                SDL.SDL_AudioSpec desired = new SDL.SDL_AudioSpec();

                // Set the desired audio format
                desired.freq = _newFrequency;       // Samples per second
                desired.format = SDL.AUDIO_S16SYS;  // Signed 16-bit PCM samples
                desired.channels = 1;               // Mono channel
                desired.samples = BufferSize;       // Audio buffer size in samples
                desired.callback = null;            // No callback function
                desired.userdata = IntPtr.Zero;     // User data passed to the callback

                _devId = SDL.SDL_OpenAudioDevice(IntPtr.Zero, 0, ref desired, out _spec,
                                                (int)SDL.SDL_AUDIO_ALLOW_ANY_CHANGE);

                // Assume it worked
                _frequency = _newFrequency;
                _freqChangeRequested = false;

                // But make sure :-)
                if (_devId == 0)
                {
                    Log.Warn(Category.UI, "Could not open audio device; no speech output available.");
                    return;
                }

                Log.Debug(Category.Speech, "Requested: format {0}, freq {1}, chan {2}, samples {3}",
                                            desired.format, desired.freq, desired.channels, desired.samples);
                Log.Debug(Category.Speech, "Obtained:  format {0}, freq {1}, chan {2}, samples {3}",
                                           _spec.format, _spec.freq, _spec.channels, _spec.samples);

                // Todo: would be nice to see if we can enumerate and name the host
                // devices, like Ethernet, and actually let the user assign one in
                // settings... ya don't need Dolby 5.1 for a mono, 8Khz quality output!
            }

            Log.Info(Category.UI, "Audio device ID {0} open for playback", _devId);

            Reset();
        }

        /// <summary>
        /// Handle a baud rate change from the CTC/PIT for the channel feeding
        /// the CVSD chip.  Requires that we potentially close and re-open the
        /// audio device!
        /// </summary>
        public void RateChange(int newFreq)
        {
            Log.Write("Rate change request: {0}, current {1}", newFreq, _spec.freq);

            if (newFreq != _spec.freq)
            {
                _newFrequency = newFreq;
                _freqChangeRequested = true;

                // Good time to re-init?
                if (Paused && !Buffering)
                {
                    Shutdown();
                    Initialize();
                }
            }
        }

        /// <summary>
        /// Reset this instance: pause and clear.  Called by the MC3417 when the
        /// PERQ resets SpeechOut or the Z80 powers up.
        /// </summary>
        public void Reset()
        {
            Pause();
            Clear();
        }

        /// <summary>
        /// Pause playback.
        /// </summary>
        public void Pause()
        {
            if (HaveAudio && !_paused)
            {
                SDL.SDL_PauseAudioDevice(_devId, 1);
                _paused = true;

                Log.Info(Category.Speech, "Audio PAUSED");  // debug

                // If playback was running and a change request came in,
                // deal with it now.  This is a terrible hack and will be
                // refactored?
                if (_freqChangeRequested && !Buffering)
                {
                    Shutdown();
                    Initialize();
                }
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

                Log.Info(Category.Speech, "Audio RESUMED");  // debug
            }
        }

        /// <summary>
        /// Queue a block of samples from the MC3417 for playback.  If we've
        /// accumulated a full buffer and are paused, start the playback.
        /// </summary>
        public void QueueSamples(ref short[] samples, int count)
        {
            // Silently discard if we haven't got anywhere to send 'em
            if (!HaveAudio) return;

            _lastSampleRcvd = HighResolutionTimer.ElapsedHiRes();

            // Got room?
            if (_count + count > BufferSize)
            {
                if (Flush() > 0)
                {
                    if (_paused) Resume();
                }
            }

            // Queue up the current batch in one shot
            samples.CopyTo(_buffer, _count);
            _count += count;

            Log.Debug(Category.Speech, "Queued {0} samples, count = {1}", count, _count);
        }

        /// <summary>
        /// Flush the sample buffer to the audio device.  Resets _count.
        /// </summary>
        /// <remarks>
        /// Could SDL2-CS have wrapped things in a way that hides the marshaling
        /// and made interfacing easier?  Probably.  Could I have used a pinned
        /// GCHandle or Marshal.Copy or whatever?  Yeah.  But SDL_QueueAudio says
        /// it's both thread safe AND it copies the data anyway, so "I'm just gonna
        /// give this a go and see what happens," said every Darwin Award winner.
        /// </remarks>
        public int Flush()
        {
            // Anything to do?
            if (!HaveAudio || _count == 0) return 0;

            // This is going to hurt me more than it hurts you
            unsafe
            {
                fixed (short* p = _buffer)
                {
                    IntPtr ptr = (IntPtr)p;

                    SDL.SDL_QueueAudio(_devId, ptr, (uint)(_count * 2));
                }
            }

            var residual = _count;

            Clear();

            // Let the caller know how much we flushed
            Log.Debug(Category.Speech, "Flushed {0} samples", residual);
            return residual;
        }

        /// <summary>
        /// Manage the state of the audio device: if input from the PERQ has
        /// stopped, flush any residual/partial buffer data, then pause playback.
        /// </summary>
        /// <remarks>
        /// Having done the work to make custom SDL events easier to manage, it
        /// turns out just polling this once a frame (at the 16ms timer tick used
        /// for display updates) is probably fine.  It avoids the question of
        /// infinite loops (or checking for SDL_POLLSENTINEL?) if CheckIdle were
        /// to push another event to continue checking.  That might be preferred,
        /// though, as we could inject the event when a new sample arrives, with
        /// zero overhead if audio is disabled or inactive.  Meh.  We'll see.
        /// </remarks>
        public void CheckIdle()
        {
            if (!HaveAudio) return;     // Should never happen, but check?

            var now = HighResolutionTimer.ElapsedHiRes();
            var delta = now - _lastSampleRcvd;

            // Has the data from the MC3417 stopped?
            if (delta > NominalRateInMsec)
            {
                // Do we have data buffered up?
                if (_count > 0)
                {
                    Log.Info(Category.Speech, "Data's late! delta={0:N4}ms", delta);

                    // Yes: whether playing or paused, ship it
                    Flush();

                    // Must be a short sample, so start playback
                    if (_paused) Resume();
                }
                else
                {
                    var remaining = SDL.SDL_GetQueuedAudioSize(_devId) / 2;

                    Log.Info(Category.Speech, "Waiting for playback to complete ({0} samples)", remaining);

                    // Done playing out the remaining bytes?
                    if (remaining == 0) Pause();
                }
            }
            //else
            //{
            //    Log.Info(Category.Speech, "PERQ is actively streaming: delta={0:N4}ms", delta);
            //}

            // todo: calculate the running average bps/latency?
        }

        // todo: provide a big knob for adjusting the buffer size (when not busy)

        /// <summary>
        /// Clear the local buffer to zeros and reset counter.
        /// </summary>
        void Clear()
        {
            Log.Detail(Category.Speech, "Clearing {0} samples", _count);

            while (_count > 0)
            {
                _count--;
                _buffer[_count] = 0;
            }
        }

        /// <summary>
        /// Sort of a "soft" shutdown.  Closes the device and resets.
        /// </summary>
        public void Shutdown()
        {
            if (_devId > 0)
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
                Console.WriteLine("Audio device is not available.");
                return;
            }

            var stat = SDL.SDL_GetAudioDeviceStatus(_devId);
            var bytes = SDL.SDL_GetQueuedAudioSize(_devId);
            Console.WriteLine("Audio device ID: {0}  Status: {1}  Queued: {2} samples",
                              _devId, stat, (bytes / 2));
            Console.WriteLine("Nominal rate: {0:N2}ms ({1}kHz)  Buffer: {2}  Count: {3}",
                              NominalRateInMsec, _frequency / 1000, BufferSize, _count);
            Console.WriteLine("Filter: Min/Max {0:N4}/{1:N4}, Decay {2:N4}, Charge {3:N4}, Leak {4:N4}, Gain {5:N4}",
                              PERQemu.Sys.IOB.Z80System.Speech.FilterMin,
                              PERQemu.Sys.IOB.Z80System.Speech.FilterMax,
                              PERQemu.Sys.IOB.Z80System.Speech.FilterDecayTC,
                              PERQemu.Sys.IOB.Z80System.Speech.FilterChargeTC,
                              PERQemu.Sys.IOB.Z80System.Speech.IntegratorLeakTC,
                              PERQemu.Sys.IOB.Z80System.Speech.SampleGain);
        }


        uint _devId;
        bool _paused;

        int _frequency;
        int _newFrequency;
        bool _freqChangeRequested;

        int _count;
        ushort _bufSize;
        short[] _buffer;

        // For pacing and statistics
        double _lastSampleRcvd;

        SDL.SDL_AudioSpec _spec;
    }
}
