/*
Atmosphere Autopilot Redux, plugin for Kerbal Space Program 2.
Copyright (C) 2026, Infinity Irrealis.
Based on Atmosphere Autopilot, Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

using System;
using KSP.Game;
using KSP.Sim.impl;
using UnityEngine;

using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// Owns the autopilot for the active vessel: creates its context when a vessel takes control, tears it down when
    /// the vessel changes or flight ends, and pauses model training with the game.
    /// Replaces the lifecycle half of KSP1 Atmosphere Autopilot's AtmosphereAutopilot addon.
    /// </summary>
    public sealed class AARuntime : MonoBehaviour
    {
        private static BackgroundThread background_thread;

        /// <summary>
        /// Worker thread for online model regression, shared by every vessel.
        /// </summary>
        public static BackgroundThread BackgroundThread => background_thread ??= new BackgroundThread();

        public static AARuntime Instance { get; private set; }

        /// <summary>
        /// Autopilot of the active vessel, or null outside flight.
        /// </summary>
        public AutopilotContext Context { get; private set; }

        /// <summary>
        /// Raised when <see cref="Context" /> is replaced.
        /// </summary>
        public event Action ContextChanged;

        /// <summary>
        /// Raised just before <see cref="Context" /> is disposed and replaced, while it still knows what it was doing.
        /// </summary>
        public event Action<AutopilotContext> ContextChanging;

        private bool thread_paused;

        private void Awake()
        {
            Instance = this;
        }

        private void Update()
        {
            var game = GameManager.Instance != null ? GameManager.Instance.Game : null;
            VesselComponent sim = null;
            VesselBehavior view = null;
            if (game != null && InFlight(game) &&
                game.ViewController.TryGetActiveSimVessel(out var active) && !active.IsKerbalEVA)
            {
                sim = active;
                view = game.ViewController.GetBehaviorIfLoaded(active);
            }

            // Vessel switch, docking, undocking and view rebuilds all show up as a new component or behavior
            if (Context != null && (Context.Vessel.Sim != sim || Context.Vessel.View != view))
                SetContext(null);
            if (Context == null && sim != null && view != null)
                SetContext(new AutopilotContext(sim, view));

            var should_pause = game == null || Context == null || game.ViewController.IsPaused;
            if (should_pause != thread_paused && background_thread != null && background_thread.IsRunning)
            {
                if (should_pause)
                    background_thread.Pause();
                else
                    background_thread.Resume();
                thread_paused = should_pause;
            }
        }

        private static bool InFlight(GameInstance game)
        {
            if (game.GlobalGameState == null)
                return false;
            var state = game.GlobalGameState.GetState();
            // Map view is its own state, and the vessel keeps flying there
            return state == GameState.FlightView || state == GameState.Map3DView;
        }

        private void SetContext(AutopilotContext context)
        {
            if (Context != null)
                ContextChanging?.Invoke(Context);
            Context?.Dispose();
            Context = context;
            ContextChanged?.Invoke();
        }

        private void OnDestroy()
        {
            SetContext(null);
            background_thread?.Stop();
            background_thread = null;
            if (Instance == this)
                Instance = null;
        }
    }
}
