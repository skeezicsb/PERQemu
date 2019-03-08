// systemtimer.cs - Copyright (c) 2018-2019 Josh Dersch (derschjo@gmail.com)
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
using System.Threading;

namespace PERQemu
{
    /// <summary>
    /// Provides a heartbeat timer with a high-resolution interval.
    /// </summary>
    public class SystemTimer
    {
        public SystemTimer(float ival)
        {
            _interval = ival;
            _callback = new HRTimerElapsedCallback(OnElapsed);
            _handle = _hrTimer.Register(_interval, _callback);

            _sync = new ManualResetEventSlim(false);
#if DEBUG
            Console.WriteLine("SystemTimer constructed, HR timer handle is {0}", _handle);  // FIXME
#endif
        }

        ~SystemTimer()
        {
            try
            {
                // Free up our callback in case we reconfigure and reinstantiate
                _hrTimer?.Unregister(_handle);
            }
            catch
            {
                Console.WriteLine("Barfed trying to unregister SystemTimer " + _handle);    // FIXME
            }
        }

        public void Reset()
        {
            _hrTimer.Enable(_handle, false);
            _sync.Reset();
        }

        public void StartTimer(bool enabled)
        {
            _hrTimer.Enable(_handle, enabled);
#if DEBUG
            Console.WriteLine("Heartbeat " + (enabled ? "started" : "stopped"));    // FIXME
#endif
        }

        public float Interval
        {
            get { return _interval; }
            set { _interval = value; }
        }

        /// <summary>
        /// Waits for heartbeat timer to fire.  Called on the main emulator
        /// thread to sync CPU execution (which regulates our display too); can
        /// also be used for the "real" Z80, or other real-time events.
        /// </summary>
        public void WaitForHeartbeat()
        {
            _sync.Wait();
            //Console.WriteLine("Heartbeat on " + Thread.CurrentThread.ManagedThreadId);  // FIXME
            _sync.Reset();
        }

        void OnElapsed(HRTimerElapsedEventArgs e)
        {
            //Console.WriteLine("Elapsed on {0}, delay {1}", Thread.CurrentThread.ManagedThreadId, e.Delay);  // FIXME
            _sync.Set();
        }

        // Save a handle to the shared HR timer
        private HighResolutionTimer _hrTimer = HighResolutionTimer.Instance;

        private int _handle;
        private float _interval;
        private HRTimerElapsedCallback _callback;
        private ManualResetEventSlim _sync;
    }
}
