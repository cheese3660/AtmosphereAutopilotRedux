/*
Atmosphere Autopilot, plugin for Kerbal Space Program.
Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.
 
Atmosphere Autopilot is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
Atmosphere Autopilot is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with Atmosphere Autopilot.  If not, see <http://www.gnu.org/licenses/>. 
*/

/*
Modified by Infinity Irrealis, 2026, as part of AtmosphereAutopilotRedux,
a port of Atmosphere Autopilot to Kerbal Space Program 2.
See the repository history for details of changes.
*/

using System;
using System.Collections.Generic;
using System.Linq;
using System.IO;
using System.Text;
using System.Threading;
//using UnityEngine;

namespace AtmosphereAutopilotRedux.Utils
{

    /// <summary>
    /// Background thread, designed to execute a set of functions with limits on execution freuency
    /// </summary>
    public class BackgroundThread : IDisposable
    {
        private readonly List<Func<bool>> _executionQue = new List<Func<bool>>();

        private readonly Thread _thread;
        private readonly ManualResetEvent _sema = new ManualResetEvent(true);

        public BackgroundThread()
        {
            _thread = new Thread(new ThreadStart(Cycle)) { IsBackground = true, Name = "AtmosphereAutopilot" };
        }

        public void Start()
        {
            if (_stop)
                throw new InvalidOperationException("Thread was aborted");
            if (!_thread.IsAlive)
            {
                _thread.Start();
            }
        }

        public void Stop()
        {
            Resume();
            _stop = true;
        }

        public void Dispose()
        {
            Stop();
        }

        public void Pause()
        {
            if (_stop)
                throw new InvalidOperationException("Thread was stopped");
            _sema.Reset();
        }

        public void Resume()
        {
            if (_stop)
                throw new InvalidOperationException("Thread was stopped");
            _sema.Set();
        }

        public bool IsRunning => _thread.IsAlive;

        private bool _stop = false;

        private void Cycle()
        {
            while (true)
            {
                _sema.WaitOne();
                var success = false;
                foreach (var func in _executionQue)
                {
                    if (_stop)
                        return;
                    try
                    {
                        success |= func();
                    }
                    catch (Exception e)
                    {
                        UnityEngine.Debug.LogError("[AtmosphereAutopilot] Background thread exception - broken function removed from queue\n" + e);
                        remove_func(func);
                    }
                    _sema.WaitOne();
                }
                if (_stop)
                    return;
                if (!success)
                    Thread.Sleep(5);
                if (_queFlag)
                {
                    _queFlag = false;
                    lock (_removalQue)
                    {
                        // Process removal requests
                        while (_removalQue.Count > 0)
                        {
                            var hndl = _removalQue[0];
                            _executionQue.Remove(hndl);
                            _removalQue.RemoveAt(0);
                        }
                    }
                    lock (_additionQue)
                    {
                        // Process addition requests
                        while (_additionQue.Count > 0)
                        {
                            var hndl = _additionQue[0];
                            _executionQue.Add(hndl);
                            _additionQue.RemoveAt(0);
                        }
                    }
                }
            }
        }

        private readonly List<Func<bool>> _removalQue = new List<Func<bool>>();
        private readonly List<Func<bool>> _additionQue = new List<Func<bool>>();
        private volatile bool _queFlag = false;

        /// <summary>
        /// Remove function from thread execution queue.
        /// </summary>
        /// <param name="hndl">Function to remove,</param>
        public void remove_func(Func<bool> hndl)
        {
            lock (_removalQue)
                _removalQue.Add(hndl);
            _queFlag = true;
        }

        /// <summary>
        /// Add function to thread execution queue
        /// </summary>
        /// <param name="hndl">Handle to parameterless function, that returns true if it was fully executed or false if
        /// it didn't execute. You would want to return false if you didn't allow it to run now.</param>
        public void add_func(Func<bool> hndl)
        {
            lock (_additionQue)
                _additionQue.Add(hndl);
            _queFlag = true;
        }
    }
}
