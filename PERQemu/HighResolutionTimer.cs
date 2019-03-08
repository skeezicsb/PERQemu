// highresolutiontimer.cs - Copyright (c) 2019 S. Boondoggle (skeezicsb@gmail.com)
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
using System.Diagnostics;
using System.Threading;
using System.Collections.Generic;

namespace PERQemu
{
    /// <summary>
    /// Custom event args, though this seems unnecessarily clumsy.
    /// </summary>
    public class HRTimerElapsedEventArgs : EventArgs
    {
        public double Delay { get; }

        internal HRTimerElapsedEventArgs(double delay)
        {
            Delay = delay;
        }
    }

    public delegate void HRTimerElapsedCallback(HRTimerElapsedEventArgs a);

    /// <summary>
    /// Internal class to track our timer clients (multiple intervals for
    /// one stopwatch).
    /// </summary>
    internal class TimerThing
    {
        public TimerThing()
        {
            Interval = 0;
            NextTrigger = 0;
            Callback = null;
            Enabled = false;
            Free = true;
        }

        public TimerThing(double interval, HRTimerElapsedCallback handler)
        {
            Interval = interval;
            NextTrigger = interval;
            Callback = handler;
            Enabled = false;
            Free = false;
        }

        public HRTimerElapsedCallback Callback { get; set; }
        public double NextTrigger { get; set; }
        public double Interval { get; set; }
        public bool Enabled { get; set; }
        public bool Free { get; set; }

        public override string ToString()
        {
            return string.Format("NextTrigger={0}, Interval={1}, Enabled={2}, Free={3}", NextTrigger, Interval, Enabled, Free);
        }
    }

    /// <summary>
    /// Shared high precision timer that supports multiple subscribers with
    /// separate intervals.  Runs a single Stopwatch on a background thread
    /// and doesn't rely on platform-specific code.
    /// 
    /// Based initially on an anonymous code snippet found on-line...
    /// </summary>
    /// <remarks>
    /// To register or unregister a timer client, the timer should probably
    /// be stopped since I'm not locking it or using enumerators (too slow).
    /// For now this is more a proof-of-concept than a final implementation...
    /// </remarks>
    public sealed class HighResolutionTimer
    {
        /// <summary>
        /// Creates a timer and an empty list of requesters.
        /// </summary>
        private HighResolutionTimer()
        {
            Console.WriteLine("HRTimer private constructor called.");   // FIXME
            _stopwatch = new Stopwatch();
            _requesters = new List<TimerThing>();
            _thread = null;
            _runThread = false;
            _isRunning = false;
            _throttle = new AutoResetEvent(true);
        }

        public static HighResolutionTimer Instance
        {
            get { return _instance; }
        }

        /// <summary>
        /// Tick time length in milliseconds.
        /// </summary>
        public static readonly double TickLength = 1000f / Stopwatch.Frequency;

        /// <summary>
        /// Tick frequency of the underlying mechanism.
        /// </summary>
        public static readonly double Frequency = Stopwatch.Frequency;

        /// <summary>
        /// True if the system/operating system supports HighResolution timer.
        /// </summary>
        public static bool IsHighResolution = Stopwatch.IsHighResolution;

        /// <summary>
        /// True when our Stopwatch is running.
        /// </summary>
        public bool IsRunning => _isRunning;

        /// <summary>
        /// Register as a timer client to receive events at the specified
        /// interval and event handler.
        /// </summary>
        public int Register(double period, HRTimerElapsedCallback cb)
        {
            int tag = 0;
            double next = period;

            Console.WriteLine("Register called, requesters length = " + _requesters.Count);   // FIXME

            // Loop to see if we have an existing subscriber with the same
            // period; if so, adjust our new request to fire at the same time,
            // in effect coalescing the two and slightly improving efficiency :-)
            for (int i = 0; i < _requesters.Count; i++)
            {
                if (_requesters[i].Interval == period && !_requesters[i].Free)
                {
                    next = _requesters[i].NextTrigger;
                    Console.WriteLine("Coalesced new timer at " + next);   // FIXME
                }
            }

            // Now find an empty slot for the new request and set it
            for (tag = 0; tag < _requesters.Count; tag++)
            {
                if (_requesters[tag].Free)
                {
                    _requesters[tag].Free = false;
                    _requesters[tag].Interval = period;
                    _requesters[tag].NextTrigger = next;
                    _requesters[tag].Callback = cb;
                    Console.WriteLine("Registered timer {0}, interval {1}, next trigger {2}",
                                      tag, period, next);   // FIXME

                    return tag;
                }
            }

            // None free?  Extend...
            _requesters.Add(new TimerThing(period, cb));
            Console.WriteLine("Extended list for new timer {0}, interval {1}, next trigger {2}",
                              tag, period, next);   // FIXME
            return tag;
        }

        /// <summary>
        /// Enable events for a particular subscriber.  If the thread isn't yet
        /// running, start it up.  This might be problematic, if we start firing
        /// events for the CPU before the IO starts back up, or whatever.  We'll
        /// fall off that bridge when we come to it.
        /// </summary>
        public void Enable(int tag, bool doit)
        {
            try
            {
                _requesters[tag].Enabled = doit;

                // Enabling a timer starts the thread if it isn't already running
                if (doit && !_isRunning)
                {
                    Start();
                }
            }
            catch
            {
                Console.WriteLine("Failed to set enable for tag " + tag);   // FIXME
            }
        }

        /// <summary>
        /// Unregister the specified client and mark the slot as free.
        /// </summary>
        public void Unregister(int tag)
        {
            try
            {
                if (_requesters[tag].Free)
                {
                    Console.WriteLine("Request to unregister alread freed timer " + tag);   // FIXME
                }
                else
                {
                    // Be sure to shut it down first
                    Enable(tag, false);
                    _requesters[tag].Free = true;
                }
            }
            catch
            {
                Console.WriteLine("Bad call to unregister a timer!");   // FIXME
                // do proper exception handling here...
            }
        }

        /// <summary>
        /// Starts the timer and begins firing off events.  Launches the
        /// background thread if it isn't already running.
        /// </summary>
        public void Start()
        {
            if (_isRunning) return;

            _isRunning = true;
            _throttle.Set();        // In case we're hanging in WaitOne()

            if (_thread == null)
            {
                _runThread = true;
                _thread = new Thread(ExecuteTimer) { IsBackground = true };
                _thread.Name = "HighResTimer";
                _thread.Start();
            }

            Console.WriteLine("Timer thread started.");   // FIXME
        }

        /// <summary>
        /// Stops the timer, leaves the thread running so it can be resumed.
        /// </summary>
        public void Stop()
        {
            if (!_isRunning) return;

            _isRunning = false;
            _throttle.Reset();
            Console.WriteLine("Timer thread paused.");   // FIXME
        }

        /// <summary>
        /// Shutdown the timer and exit the background thread.
        /// </summary>
        public void Shutdown()
        {
            if (_thread == null) return;

            Stop();
            _runThread = false;     // flag that we're ready to exit
            _throttle.Set();        // if we're in Wait(), release the hold

            if (Thread.CurrentThread != _thread)
            {
                _thread.Join();
            }
            Console.WriteLine("Timer thread shutting down.");   // FIXME
        }

        /// <summary>
        /// Executes the timer.
        /// </summary>
        private void ExecuteTimer()
        {
            const double tolerance = 0.001d;    // some wiggle room
            double now, next, diff, skew;

            // curiosity
            long shortSpin, longSpin, shortSleep, longSleep;

            // Set our processor affinity so the timer thread doesn't bounce
            // around from cpu to cpu.  Should help accuracy and performance?
            Thread.BeginThreadAffinity();

            ResetIntervals();

            _stopwatch.Reset();
            Console.WriteLine("Stopwatch initialized, HRT thread {0} running {1} timers",
                              Thread.CurrentThread.ManagedThreadId, _requesters.Count);   // FIXME

            shortSpin = longSpin = shortSleep = longSleep = 0;

            now = next = 0d;
            _stopwatch.Start();

            while (_runThread)
            {
                // Set our next target
                next = NextInterval(now);

                // Loop until we reach it
                while (_isRunning)
                {
                    diff = next - ElapsedHiRes();
                    if (diff <= tolerance)
                        break;

                    if (diff < 1d)
                    {
                        shortSpin++;
                        Thread.SpinWait(10);
                    }
                    else if (diff < 5d)
                    {
                        longSpin++;
                        Thread.SpinWait(100);
                    }
                    else if (diff < 15d)
                    {
                        shortSleep++;
                        _throttle.WaitOne(1);
                    }
                    else
                    {
                        longSleep++;
                        _throttle.WaitOne(10);
                    }
                }

                // Snapshot the elapsed time
                now = ElapsedHiRes();
                skew = now - next;

                // Time to fire, unless we've been stopped...
                if (_isRunning)
                {
                    // Fire off expired callbacks, then reschedule them
                    for (int i = 0; i < _requesters.Count; i++)
                    {
                        if ((_requesters[i].NextTrigger <= now + tolerance) && _requesters[i].Enabled)
                        {
                            _requesters[i].Callback.Invoke(new HRTimerElapsedEventArgs(skew));
                            _requesters[i].NextTrigger += _requesters[i].Interval;
                        }
                    }
                }
                else
                {
                    // Stop time
                    _stopwatch.Stop();

                    // While we're paused, do nothing until told to restart
                    // or shutdown the thread
                    _throttle.WaitOne();

                    // Restart it to resume from pause
                    _stopwatch.Start();
                }
            }

            // Thread is exiting, stop the ticker
            _stopwatch.Stop();

            // This is probably irrelevant
            Thread.EndThreadAffinity();

            // for posterity
            Console.WriteLine("Stopwatch stopped, HRT thread exiting");   // FIXME
            Console.WriteLine("--> SpinWaits short={0} long={1}", shortSpin, longSpin);
            Console.WriteLine("--> Sleeps    short={0} long={1}", shortSleep, longSleep);
        }

        /// <summary>
        /// Resets the intervals to zero.  Must be called when the Stopwatch
        /// is reset, otherwise the first interval can be a long, long wait. ;-)
        /// </summary>
        private void ResetIntervals()
        {
            for (int i = 0; i < _requesters.Count; i++)
            {
                _requesters[i].NextTrigger = _requesters[i].Interval;
            }
            Console.WriteLine("Interval timers reset.");   // FIXME
        }

        private double NextInterval(double now)
        {
            double next = 0.0;

            if (_requesters.Count > 0)
            {
                next = _requesters[0].NextTrigger;

                for (int i = 1; i < _requesters.Count; i++)
                {
                    if (_requesters[i].Enabled && (_requesters[i].NextTrigger < next))
                        next = _requesters[i].NextTrigger;
                }
            }

            if (next > 0.0)
            {
                return next;
            }
            else
            {
                Console.WriteLine("No active requesters? Pausing HR timer.");   // FIXME
                _isRunning = false;
                return now;
            }
        }

        public static double ElapsedHiRes()
        {
            return _stopwatch.ElapsedTicks * TickLength;
        }

        public void DumpTimers()
        {
            Console.WriteLine("HighResTimer status:");
            for (int i = 0; i < _requesters.Count; i++)
            {
                Console.WriteLine("\t" + _requesters[i]);
            }
        }

        /// <summary>
        /// The timer is running and firing events
        /// </summary>
        private volatile bool _isRunning;

        /// <summary>
        /// If false, signals the thread to exit
        /// </summary>
        private volatile bool _runThread;

        /// <summary>
        /// A wait handle to act as a throttle
        /// </summary>
        private static AutoResetEvent _throttle;

        private static Thread _thread;
        private static Stopwatch _stopwatch;
        private static List<TimerThing> _requesters;

        private static readonly HighResolutionTimer _instance = new HighResolutionTimer();
    }
}