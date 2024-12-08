//
// HardDisk.cs - Copyright (c) 2006-2024 Josh Dersch (derschjo@gmail.com)
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
    // Optional hook fired on index pulses from the drive
    public delegate void IndexPulseCallback(ulong last);

    /// <summary>
    /// Emulates the mechanical operation of a hard disk drive, handling the
    /// timing for seek operations, index pulses, and other basic operations.
    /// Exposes the block API from StorageDevice through Get/SetSector for data
    /// access, and provides status infomation useful to a disk controller.  
    /// </summary>
    /// <remarks>
    /// Status flags are true/false; the associated Controller presents these
    /// to the PERQ in whichever active low/high state is appropriate.
    /// </remarks>
    public class HardDisk : StorageDevice
    {
        public HardDisk(Scheduler sched, string filename) : base(filename)
        {
            _scheduler = sched;

            _ready = false;
            _fault = false;
            _index = false;
            _seekComplete = false;

            _startupEvent = null;
            _seekEvent = null;
            _seekCallback = null;
            _indexEvent = null;
            _indexCallback = null;

            _cyl = 0;
            _lastStep = 0;
            _lastIndexPulse = 0;
            _discRotationTimeNsec = 0;
            _indexPulseDurationNsec = 0;
        }

        // Status bits
        public virtual bool Ready => _ready;
        public virtual bool Fault => _fault;
        public virtual bool Index => _index;
        public virtual bool Track0 => (_cyl == 0);
        public virtual bool SeekComplete => _seekComplete;

        // Debugging/sanity check
        public virtual ushort CurCylinder => _cyl;
        public virtual byte CurHead => _head;

        /// <summary>
        /// Does a hardware reset on the device.
        /// </summary>
        public virtual void Reset()
        {
            if (!IsLoaded)
            {
                // Not really relevant until we support removable pack hard drives
                Log.Warn(Category.HardDisk, "Reset called but no disk loaded!");
                _fault = true;
                return;
            }

            // Clear fault status and reset the seek state
            FaultClear();
            StopSeek();

            // Schedule a motor start to bring the drive up to speed (if it isn't
            // already); will bring the drive on-line and set the ready bit
            MotorStart();

            // Stop the current index event and reset, restart it
            if (_indexEvent != null)
            {
                _scheduler.Cancel(_indexEvent);
            }

            _index = false;
            IndexPulseStart(0, null);
        }

        /// <summary>
        /// Gets a sector from the current cylinder/track.
        /// </summary>
        public virtual Sector GetSector(ushort sector)
        {
            return Read(_cyl, _head, sector);
        }

        /// <summary>
        /// Get a sector of data immediately.
        /// </summary>
        public virtual Sector GetSector(ushort cylinder, byte head, ushort sector)
        {
            return Read(cylinder, head, sector);
        }

        /// <summary>
        /// Writes a sector.  Embrace your inner address!
        /// </summary>
        public virtual void SetSector(Sector sec)
        {
            Write(sec);
        }

        /// <summary>
        /// Register a callback to fire when a seek operation completes.
        /// </summary>
        public virtual void SetSeekCompleteCallback(SchedulerEventCallback cb)
        {
            _seekCallback = cb;
        }

        /// <summary>
        /// Sets the index pulse callback.  Used by the MFM formatter.
        /// </summary>
        public virtual void SetIndexPulseCallback(IndexPulseCallback cb)
        {
            _indexCallback = cb;
        }

        /// <summary>
        /// Set the current head from the Head Select lines.
        /// </summary>
        public virtual void HeadSelect(byte head)
        {
            _head = head;
        }

        /// <summary>
        /// Initiates or continues a Seek by pulsing the Disk Step line.
        /// Direction is >0 for positive steps, or 0 for negative steps.
        /// </summary>
        public virtual void SeekStep(int direction)
        {
            // Compute and check our new cylinder
            if (direction > 0)
            {
                _cyl = (ushort)Math.Min(_cyl + 1, (Geometry.Cylinders - 1));
            }
            else
            {
                _cyl = Math.Min((ushort)(_cyl - 1), _cyl);      // Don't underflow yer ushorts!
            }

            var seekStart = _scheduler.CurrentTimeNsec;
            var seekEnd = (ulong)Specs.MinimumSeek;

            // Schedule the time delay based on drive specifications
            if (_seekEvent == null)
            {
                // Starting a new seek
                _lastStep = seekStart;
                _stepCount = 1;

                Log.Detail(Category.HardDisk, "Initial step to cyl {0}, seek {1}ms", _cyl, seekEnd);

                _seekEvent = _scheduler.Schedule(seekEnd * Conversion.MsecToNsec, SeekCompletion, seekStart);
            }
            else
            {
                // Seek in progress, so buffer the step by extending the delay
                _stepCount++;

                // Get our start time from the event context...
                seekStart = (ulong)_seekEvent.Context;

                // Recompute our total expected seek time based on the number of
                // step pulses received so far.  This is a rough linear approximation
                // rather than a per-drive ramp function but is accurate enough :-)
                var seekTime = (_stepCount * _rampStep) + Specs.MinimumSeek;

                // How long since the last step?  Technically there are tight specs
                // for the duration of and time between pulses but we can only assume
                // that the microcode/controller will adhere to them... (computing
                // this is mostly to see how the microcode/DIB implementation behaves
                // and can eventually be removed as unneeded overhead)
                var interval = (_scheduler.CurrentTimeNsec - _lastStep) * Conversion.NsecToMsec;
                _lastStep = _scheduler.CurrentTimeNsec;

                // How long since the start of this seek (in ms)?
                var elapsed = (_lastStep - seekStart) * Conversion.NsecToMsec;

                seekEnd = (ulong)((seekTime - elapsed) * Conversion.MsecToNsec);

                Log.Detail(Category.HardDisk,
                           "Buffered step to cyl {0} (interval={1:n}ms), total seek now {2:n}ms ({3:n}ms elapsed, {4:n}ms remaining)",
                          _cyl, interval, seekTime, elapsed, seekEnd * Conversion.NsecToMsec);

                // Update the seek event with the new delay time
                _seekEvent = _scheduler.ReSchedule(_seekEvent, seekEnd);
            }
        }

        /// <summary>
        /// Seek to the given cylinder.
        /// </summary>
        /// <remarks>
        /// For 8" Micropolis drives with embedded controllers, computes the timing
        /// for a seek from the current head position to the requested cylinder.
        /// Assumes the controller checks bounds and that it tracks busy status,
        /// not initiating a new seek while one is in progress...
        /// </remarks>
        public virtual void SeekTo(ushort cyl)
        {
            // NB: If the heads are already over the requested cylinder, we DO
            // fire the completion callback, with a minimal (1 usec) delay so
            // that the microcode can potentially catch the state machine's idle
            // to busy transition (which some versions explicitly check for).

            double delay = Specs.MinimumSeek;
            _stepCount = Math.Abs(_cyl - cyl);

            if (Settings.Performance.HasFlag(RateLimit.DiskSpeed) && _stepCount > 0)
            {
                delay += (_stepCount * _rampStep);
            }

            Log.Debug(Category.HardDisk, "Drive seek to cyl {0} from {1}, {2} steps in {3:n}ms",
                                          cyl, _cyl, _stepCount, delay);
            _cyl = cyl;

            // Schedule the callback
            _seekEvent = _scheduler.Schedule((ulong)delay * Conversion.MsecToNsec, SeekCompletion);
        }

        /// <summary>
        /// Finish a seek and inform any registered client.  Here's where we
        /// apply the "head settling" time if the device requires it.
        /// </summary>
        public virtual void SeekCompletion(ulong skewNsec, object context)
        {
            _seekComplete = true;
            _seekEvent = null;

            // Schedule a callback to the registered client, if any
            if (_seekCallback != null)
            {
                ulong settle = 1 * Conversion.UsecToNsec;

                // If faithfully emulating the slow ass disk drives of the mid-
                // 1980s, then add the head settling time to cap off our seek
                // odyssey.  Otherwise a default 1us delay is reasonable.
                if (Settings.Performance.HasFlag(RateLimit.DiskSpeed) && _stepCount > 0 && Specs.HeadSettling > 0)
                {
                    settle = (ulong)Specs.HeadSettling * Conversion.MsecToNsec;

                    Log.Detail(Category.HardDisk, "Seek complete [settling callback in {0:n}ms]",
                                                  settle * Conversion.NsecToMsec);
                }
                else
                {
                    Log.Detail(Category.HardDisk, "Seek complete [callback in 1us]");
                }

                _scheduler.Schedule(settle, _seekCallback);
            }
            else
            {
                Log.Detail(Category.HardDisk, "Seek complete");
            }
        }

        /// <summary>
        /// Stops a seek in progress and resets.  It's not clear that the microcode
        /// would ever do this or how the drive would react (presumably the hardware
        /// completes the seek operation based on how many steps have been buffered;
        /// check the manual...)
        /// </summary>
        public virtual void StopSeek()
        {
            if (_seekEvent != null)
            {
                _scheduler.Cancel(_seekEvent);
                _seekEvent = null;
            }

            _seekComplete = false;
            _stepCount = 0;
        }

        /// <summary>
        /// Computes a rotational delay for the start of a sector from the last
        /// index pulse.  Returns nanoseconds so you don't have to scale it for
        /// scheduling.  Isn't terribly accurate as it doesn't account for sector
        /// interleaving, but adds a little extra realism. :-)
        /// </summary>
        public ulong ComputeRotationalDelay(ulong now, int sector)
        {
            // t = time between pulses (in ns) = rpm / #sectors
            var t = (long)Conversion.RPMtoNsec(Specs.RPM) / Geometry.Sectors;

            // cur = what sector the heads are over now (time now - last pulse) / t
            var cur = (long)(now - _lastIndexPulse) / t;

            // dist = distance from current to desired sector (linear, no account for interleave)
            var dist = sector - cur;
            var delay = (ulong)((dist < 0 ? dist + Geometry.Sectors : dist) * t);

            Log.Detail(Category.HardDisk, "Rotational delay from cur={0} to desired={1} is {2}", cur, sector, delay);
            return delay;
        }

        /// <summary>
        /// Requests that the drive clear its Fault status.
        /// </summary>
        public virtual void FaultClear()
        {
            if (_fault)
            {
                Log.Debug(Category.HardDisk, "Fault cleared!");
                _fault = false;
            }
        }

        /// <summary>
        /// Spin up the virtual drive on a cold start.  This schedules an event to
        /// raise the ready signal (which should cause an interrupt) after hardware
        /// reset.  If already on-line, calls DriveReady without delay.
        /// </summary>
        void MotorStart()
        {
            if (!_ready && _startupEvent == null)
            {
                ulong delay = 100;

                if (Settings.Performance.HasFlag(RateLimit.StartupDelay) && _lastIndexPulse == 0)
                {
                    // This makes sense if we do the whole "watch the screen warm up
                    // and play audio of the fans & disks whirring" for the total
                    // multimedia experience.  If we just rewrote this as a fully
                    // immersive 60fps 3D game, texture mapping the screen output
                    // to the display and simulating actual keyboard presses through
                    // data glove/VR or 3D first person shooter style, then you'd
                    // really appreciate this attention to detail.  Otherwise it's
                    // basically just insane.

                    // Introduce a little variation, from 66-99% of max startup time
                    var rand = new Random();
                    delay = (ulong)(Specs.StartupDelay / 100 * rand.Next(66, 99));
                }

                _startupEvent = _scheduler.Schedule(delay * Conversion.MsecToNsec, DriveReady);

                Log.Info(Category.HardDisk, "Drive {0} motor start (ready in {1:n} seconds)",
                                              Info.Name, delay * Conversion.MsecToSec);
            }
            else
            {
                // On a reset, just assume the drive is online
                DriveReady(0, null);
            }
        }

        /// <summary>
        /// Signal that the drive is online.
        /// </summary>
        void DriveReady(ulong skew, object context)
        {
            _ready = true;
            _startupEvent = null;

            Log.Info(Category.HardDisk, "{0} is online: {1}", Info.Description, Geometry);

            // The change in Ready should trigger an interrupt, but many versions
            // of the early Boot/Vfy/SysB microcode just barf if an unexpected
            // interrupt occurs.  Fire the SeekCompletion callback if registered
            // and let the controller decide to interrupt or not.
            if (_seekCallback != null)
            {
                _seekCallback(0, _ready);
            }
        }

        /// <summary>
        /// Raises the Index signal for the drive's specified duration.  Fires the
        /// IndexPulseCallback if registered.
        /// </summary>
        void IndexPulseStart(ulong skew, object context)
        {
            _index = true;
            _indexEvent = _scheduler.Schedule(_indexPulseDurationNsec, IndexPulseEnd);

            if (_indexCallback != null)
            {
                _indexCallback(_lastIndexPulse);
            }
        }

        /// <summary>
        /// Clears the Index pulse and schedules the next one.
        /// </summary>
        void IndexPulseEnd(ulong skew, object context)
        {
            var next = _discRotationTimeNsec - _indexPulseDurationNsec;
            _lastIndexPulse = _scheduler.CurrentTimeNsec;

            _index = false;
            _indexEvent = _scheduler.Schedule(next, IndexPulseStart);
        }

        /// <summary>
        /// Initialize some tings when we loaded, mon.
        /// </summary>
        public override void OnLoad()
        {
            // Compute the index pulse duration and gap
            _discRotationTimeNsec = Conversion.RPMtoNsec(Specs.RPM);
            _indexPulseDurationNsec = (ulong)Specs.IndexPulse;
            _lastIndexPulse = _scheduler.CurrentTimeNsec;

            Log.Info(Category.HardDisk, "{0} drive loaded!  Index is {1:n}us every {2:n}ms",
                     Info.Name, _indexPulseDurationNsec / 1000.0,
                     _discRotationTimeNsec * Conversion.NsecToMsec);

            // Compute the per-seek-step time (simple linear ramp for now)
            _rampStep = (Specs.MaximumSeek - Specs.MinimumSeek) / (double)Geometry.Cylinders;

            base.OnLoad();
        }

        // Access to a scheduler
        Scheduler _scheduler;

        // Status that the drive keeps track of
        bool _ready;
        bool _fault;
        bool _index;
        bool _seekComplete;

        // Where my heads at
        ushort _cyl;
        byte _head;

        // Index timing
        ulong _discRotationTimeNsec;
        ulong _indexPulseDurationNsec;
        ulong _lastIndexPulse;
        SchedulerEvent _indexEvent;
        IndexPulseCallback _indexCallback;

        // Seek timing
        int _stepCount;
        ulong _lastStep;
        double _rampStep;
        SchedulerEvent _seekEvent;
        SchedulerEventCallback _seekCallback;

        // Startup delay
        SchedulerEvent _startupEvent;
    }
}
