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
using System.Runtime.InteropServices;

namespace PERQemu.UI
{
    /// <summary>
    /// Provides the interface to the SDL2 Audio subsystem for the PERQ's
    /// "speech" device.  Speaker buffers PCM output from the MC3417 and plays
    /// it on the host's default audio device.
    /// </summary>
    /// <remarks>
    /// Like the approach to other host devices, here we open the default audio
    /// device with a very common output profile:  48kHz, mono or stereo, 16-bit
    /// PCM.  We use a conversion stream to do the upsampling from the PERQ's
    /// 16kHz (or 32kHz) mono output and allow SDL to do the buffering for us.
    /// We let callbacks from the driver handle the pacing AND let us determine
    /// when the PERQ has stopped playback without a polling loop.
    /// </remarks>
    public class Speaker
    {
        public Speaker()
        {
            _devId = 0;
            _devFrequency = 44100;      // Tunable; should come from Settings
            _devChannels = 1;           // Tunable; should come from Settings
            _devBufSize = 1024;         // Desired buffer size (in samples)

            _frequency = 32000;         // Assume the initial Kriz baud rate
            _newFrequency = 16000;      // Assume the default Speech baud rate

            _bufSize = 0;

            _stream = IntPtr.Zero;
            _paused = true;
        }

        public bool HaveAudio => _devId > 0;
        public bool HaveStream => _stream != IntPtr.Zero;

        public int BytesAvailable => HaveStream ? SDL.SDL_AudioStreamAvailable(_stream) : 0;

        public bool Paused => !HaveAudio || _paused;
        public bool Playing => HaveAudio && !Paused;
        public bool Busy => Playing || BytesAvailable > 0;

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
        /// Open the default audio device.  Initially paused.
        /// </summary>
        public void Initialize()
        {
            if (_devId == 0)
            {
                SDL.SDL_AudioSpec desired = new SDL.SDL_AudioSpec();

                // Set the desired audio format
                desired.freq = _devFrequency;       // Samples per second
                desired.format = SDL.AUDIO_S16SYS;  // Signed 16-bit PCM samples
                desired.channels = _devChannels;    // Channels (mono or fake stereo)
                desired.samples = _devBufSize;      // Audio buffer size in samples
                desired.callback = null;            // No callback function
                desired.userdata = IntPtr.Zero;     // User data passed to the callback

                _devId = SDL.SDL_OpenAudioDevice(IntPtr.Zero, 0, ref desired, out _spec, 0);
                                               // (int)SDL.SDL_AUDIO_ALLOW_ANY_CHANGE);
                if (_devId == 0)
                {
                    Log.Warn(Category.UI, "Could not open audio device; no speech output available.");
                    return;
                }

                Log.Info(Category.Speech, "Requested: format {0}, freq {1}, chan {2}, samples {3}",
                                            desired.format, desired.freq, desired.channels, desired.samples);
                Log.Info(Category.Speech, "Obtained:  format {0}, freq {1}, chan {2}, samples {3}, size {4}",
                                           _spec.format, _spec.freq, _spec.channels, _spec.samples, _spec.size);

                // If the driver insists on stereo or can't match our requested
                // frequency, update our expectations to match the host
                _devFrequency = _spec.freq;
                _devChannels = _spec.channels;
                _devBufSize = _spec.samples;

                // Set our buffer size in bytes - just match what the device gave us
                _bufSize = _spec.size;
                _buffer = new byte[_bufSize];

                // Recompute the threshold (in ms) for the idle check
                // (I know, I know, don't think hard about this, just go with it)
                _threshold = (ulong)(_devBufSize / (_frequency / 1000.0) / 2.0);
            }

            Log.Info(Category.UI, "Audio device ID {0} open for playback", _devId);

            Reset();
        }

        /// <summary>
        /// Set up a new AudioStream with the current frequency rate.
        /// </summary>
        bool InitStream()
        {
            _frequency = _newFrequency;
            _stream = SDL.SDL_NewAudioStream(SDL.AUDIO_S16SYS, 1, _frequency,
                                             SDL.AUDIO_S16SYS, _devChannels, _devFrequency);

            if (!HaveStream)
            {
                Log.Warn(Category.Speech, "Failed to create audio stream: {0}", SDL.SDL_GetError());
                return false;
            }

            Log.Info(Category.Speech, "Created conversion stream for {0}kHz sample output", _frequency / 1000.0);
            return true;
        }

        /// <summary>
        /// Buffer the latest baud rate change from the CTC/PIT for the channel feeding
        /// the CVSD chip.  If necessary will reopen the converter stream at next pause.
        /// </summary>
        public void RateChange(int newFreq)
        {
            _newFrequency = newFreq;

            Log.Info(Category.Speech, "Rate change request: {0}", _newFrequency);
        }

        /// <summary>
        /// Pause playback.  Clears the stream buffer and audio device queue just
        /// to be sure (remove artifacts from previous samples, if any).
        /// </summary>
        public void Pause()
        {
            if (HaveAudio && !_paused)
            {
                Clear();                                    // Clear stream
                SDL.SDL_PauseAudioDevice(_devId, 1);        // Clear device
                _paused = true;

                Log.Info(Category.Speech, "Audio PAUSED");  // debug
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
        /// Queue a block of samples from the MC3417 for playback.  If we're
        /// paused and have accumulated a buffer's worth, start the playback.
        /// </summary>
        public void QueueSamples(ref short[] samples, int count)
        {
            // Silently discard if we haven't got anywhere to send 'em
            if (!HaveAudio) return;

            // Do we have a conversion stream set up?
            if (!HaveStream)
            {
                // Make one, or else
                if (!InitStream()) return;
            }

            _lastSampleRcvd = HighResolutionTimer.ElapsedHiRes();

            var queued = BytesAvailable;

            // Are we starting up playback?  If output is paused and we haven't
            // queued up any samples yet, check that the conversion stream is 
            // set up for the correct sample frequency.  Close and recreate it
            // if necessary; leave paused and bail out if that fails.
            if (queued == 0 && _paused && _frequency != _newFrequency)
            {
                SDL.SDL_FreeAudioStream(_stream);
                if (!InitStream()) return;
            }

            // This is going to hurt me more than it hurts you
            unsafe
            {
                fixed (short* p = samples)
                {
                    IntPtr ptr = (IntPtr)p;

                    // Queue up the current batch of samples for conversion
                    var res = SDL.SDL_AudioStreamPut(_stream, ptr, count * 2);

                    if (res < 0)
                    {
                        Log.Warn(Category.Speech, "AudioStreamPut failed: {0}", SDL.SDL_GetError());
                    }
                }
            }

            Log.Debug(Category.Speech, "Queued {0} samples", count);

            // If we've got a full buffer, send it
            if (BytesAvailable >= _bufSize)
            {
                Flush();
                if (_paused) Resume();
            }
        }


        /// <summary>
        /// Flush the sample stream buffer to the audio device.
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
            if (!HaveAudio || !HaveStream)
            {
                Log.Info(Category.Speech, "Flush called but no audio dev/stream!");
                return 0;
            }

            var queued = BytesAvailable;

            if (queued == 0) return 0;    // Nothing more to send

            // Clip to one full buffer's worth
            if (queued >= _bufSize)
            {
                queued = (int)_bufSize;
            }

            // Sigh.  Have to double copy to move from the conversion stream to
            // the output queue.  It's too bad the SDL-CS API sucks so hard.
            unsafe
            {
                fixed (byte* p = _buffer)
                {
                    IntPtr ptr = (IntPtr)p;

                    // Queue up the current batch of samples for conversion
                    if (SDL.SDL_AudioStreamGet(_stream, ptr, queued) < 0)
                    {
                        Log.Warn(Category.Speech, "AudioStreamGet failed: {0}", SDL.SDL_GetError());
                    }

                    if (SDL.SDL_QueueAudio(_devId, ptr, (uint)queued) < 0)
                    {
                        Log.Warn(Category.Speech, "QueueAudio failed: {0}", SDL.SDL_GetError());
                    }
                }
            }

            var residual = BytesAvailable;

            // Let the caller know how much we flushed
            Log.Info(Category.Speech, "Flushed {0} bytes, {1} buffered", queued, residual);
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
            var now = HighResolutionTimer.ElapsedHiRes();
            var delta = now - _lastSampleRcvd;

            // Has the data from the MC3417 stopped?
            if (delta > _threshold)
            {
                // Do we have data buffered up?
                if (BytesAvailable > 0)
                {
                    Log.Info(Category.Speech, "Data's late! delta={0:N4}ms", delta);

                    // Yes: whether playing or paused, ship it
                    Flush();

                    // Must be a short sample, so start playback
                    if (_paused) Resume();
                }
                else
                {
                    var remaining = SDL.SDL_GetQueuedAudioSize(_devId) / (2 * _devChannels);

                    Log.Debug(Category.Speech, "Waiting for playback to complete ({0} samples)", remaining);

                    // Done playing out the remaining bytes?
                    if (remaining == 0) Pause();
                }
            }
            else
            {
                Log.Debug(Category.Speech, "PERQ is actively streaming: delta={0:N4}ms", delta);
            }
        }


        /// <summary>
        /// Clear the local buffer to zeros and reset counter.
        /// </summary>
        void Clear()
        {
            var queued = BytesAvailable;

            if (queued > 0)
            {
                SDL.SDL_AudioStreamClear(_stream);

                Log.Info(Category.Speech, "Cleared {0} samples", queued);
            }
        }

        /// <summary>
        /// Sort of a "soft" shutdown.  Closes the device and resets.
        /// </summary>
        public void Shutdown()
        {
            if (HaveStream)
            {
                SDL.SDL_FreeAudioStream(_stream);
                _stream = IntPtr.Zero;

                Log.Info(Category.UI, "Conversion stream closed");
            }

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
                Console.WriteLine("Audio device is not available.");
                return;
            }

            var stat = SDL.SDL_GetAudioDeviceStatus(_devId);
            var bytes = SDL.SDL_GetQueuedAudioSize(_devId);
            var queued = BytesAvailable;

            Console.WriteLine("Audio status:");
            Console.WriteLine("  Device: ID: {0}  Frequency: {1:N1}kHz  Channels: {2}  Status: {3}",
                              _devId, _devFrequency / 1000.0, _devChannels, stat);
            Console.WriteLine("  Stream: Input: {0:N1}kHz  Buffered: [{1} / {2} bytes] ({3} samples)",
                              _frequency / 1000.0, queued, _bufSize, queued / (2 * _devChannels));
            Console.WriteLine("  Output: Late threshold: {0:N3}ms  Queued: {1} bytes ({2} samples)",
                              _threshold, bytes, bytes / (2 * _devChannels));
            Console.WriteLine("  Filter: Min/Max {0:N4}/{1:N4}, Decay {2:N4}, Charge {3:N4}, Leak {4:N4}, Gain {5:N4}",
                              PERQemu.Sys.IOB.Z80System.Speech.FilterMin,
                              PERQemu.Sys.IOB.Z80System.Speech.FilterMax,
                              PERQemu.Sys.IOB.Z80System.Speech.FilterDecayTC,
                              PERQemu.Sys.IOB.Z80System.Speech.FilterChargeTC,
                              PERQemu.Sys.IOB.Z80System.Speech.IntegratorLeakTC,
                              PERQemu.Sys.IOB.Z80System.Speech.SampleGain);
        }

        // Debugging
        public void SetChannels(byte chan)
        {
            // Only execute if not busy -- be careful with this
            if (_paused)
            {
                Console.WriteLine($"Resetting for {chan} output channels");
                _devChannels = chan;
                Shutdown();
                Initialize();
            }
        }


        uint _devId;
        int _devFrequency;
        byte _devChannels;
        ushort _devBufSize;

        bool _paused;

        int _frequency;
        int _newFrequency;

        uint _bufSize;
        byte[] _buffer;

        ulong _threshold;
        double _lastSampleRcvd;

        IntPtr _stream;
        SDL.SDL_AudioSpec _spec;
    }
}

/*
    Notes:

    New strategery:  open the device once with a fixed common frequency to avoid
    Windows 10 stupidity (at least on my fonky Dell 7050?).  Buffer any baud rate
    change requests forwarded from the MC3417 (via the CTC/SIO) until the first
    PCM samples arrive -- at that point, if the conversion stream needs to be
    opened or changed, close/reopen it with the new settings.  Pump the samples
    into the stream object and let SDL2 manage 'em -- no manual buffering in the
    managed code since SDL does it for us anyway.

    We'll hold off un-pausing the output until a "block" of data is received;
    this ought to be a reasonable size chunk of bytes that balances latency and
    overhead.  Use the callback mechanism to simply wait for requests from the
    output driver, then pull bytes from the conversion queue and shove them out
    the host output pipe.  *Hopefully* by choosing a common output rate there
    won't be any need for resampling a second time!  (Have to see how much CPU
    overhead this scheme adds.)

    Using the callback mechanism, however, requires all the thread locking crap
    that using the "QueueAudio" interface avoids (or handles itself), so for now
    we'll still just poll in the main event loop when audio is "busy" to pause
    and unpause the output.  Sigh.

    Data rates:

	16kHz == 2000 chars/sec in CVSD samples; 32kHz == 4000 chars/sec

	Speaker sees 8 samples (or 16 bytes) at a time, every .5 (.25) ms at the
	nominal SIO rate.  We adjust this if the host can't keep up.

	Conversion stream queues up 1024 samples before playback is started: that's
	1K samples @ 16kHz or 512 @ 32kHz, or 2KB on the input side; about .064 (.032)
	seconds' worth of playback in real time.  Thus, 15.625 full buffers/sec at
	16kHz is exactly 1 second of audio output (or 31.25 buffers at 32kHz).

	Output is pulled from the stream and shoved out the device interface in 2KB
	or 4KB chunks, depending on how many channels; the up-sampling should produce
	x3 (or x1.5) the input rate to match the 48kHz output rate of the device;
	these have to be fed quickly enough to prevent gaps, so sending 2K (or 4K)
	bytes every 21ms (very roughly) should assure smooth playback.

	Thus, if the PERQ stops sending data and the conversion stream holds back 
	~ half a buffer's worth (to smooth resampling) then we should figure that an
	11ms gap means "we're done here" and playback can be stopped and reset once
	the output device queue drains.  Checking in the event loop every 15-16ms
	ought to be fine.

	IN THEORY, the Z80's 1KB speech buffer could hold up to 8K samples after the
	PERQ stops sending, so _at most_ the playback should stop no more than 512
	milliseconds later?  But we aren't pacing the DMA or SIO quite right yet, and
	are queueing up 30+ seconds' worth on the host side so more tuning is needed...

 */
