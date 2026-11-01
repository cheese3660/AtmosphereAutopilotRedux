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
using System.Collections.Generic;
using KSP.Game;
using KSP.Sim.impl;
using KSP.Sim.State;
using UnityEngine;

using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Modules;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// The autopilot modules of one vessel, and their hook into the vessel's per-tick control input.
    /// Replaces KSP1 Atmosphere Autopilot's TopModuleManager.
    /// </summary>
    public sealed class AutopilotContext : IDisposable
    {
        public AAVessel Vessel { get; }

        private readonly Dictionary<Type, AutopilotModule> modules = new Dictionary<Type, AutopilotModule>();

        public FlightModel FlightModel { get; }
        public StandardFlyByWire FlyByWire { get; }
        public CruiseController Cruise { get; }
        public ProgradeThrustController Speed { get; }

        /// <summary>
        /// The engaged high-level autopilot, or null when disengaged.
        /// </summary>
        public StateController ActiveController { get; private set; }

        /// <summary>
        /// Raised when the engaged autopilot changes, including when it disengages itself.
        /// </summary>
        public event Action ActiveControllerChanged;

        private readonly AACtrlState ctrl = new AACtrlState();
        private bool hooked;

        public AutopilotContext(VesselComponent sim, VesselBehavior view)
        {
            Vessel = new AAVessel(sim, view);

            // Every module the fly-by-wire, cruise and speed controls need, wired the same way upstream did
            // through reflection.
            Add(FlightModel = new FlightModel(Vessel));
            Add(new PitchAngularAccController(Vessel));
            Add(new RollAngularAccController(Vessel));
            Add(new YawAngularAccController(Vessel));
            Add(new PitchAngularVelocityController(Vessel));
            Add(new RollAngularVelocityController(Vessel));
            Add(new YawAngularVelocityController(Vessel));
            Add(new PitchAoAController(Vessel));
            Add(new SideslipController(Vessel));
            Add(new DirectorController(Vessel));
            Add(Speed = new ProgradeThrustController(Vessel));
            Add(FlyByWire = new StandardFlyByWire(Vessel));
            Add(Cruise = new CruiseController(Vessel));

            foreach (var module in modules.Values)
                module.InitializeDependencies(modules);

            VelocityControllers = new AngularVelAdaptiveController[]
            {
                (AngularVelAdaptiveController)modules[typeof(PitchAngularVelocityController)],
                (AngularVelAdaptiveController)modules[typeof(RollAngularVelocityController)],
                (AngularVelAdaptiveController)modules[typeof(YawAngularVelocityController)],
            };

            // The recorder logs disengaged flight too, so while it is on the hook stays for the context's lifetime
            recorder = new FlightRecorder(this);
            if (FlightRecorder.Enabled)
                Hook();
        }

        internal AngularVelAdaptiveController[] VelocityControllers { get; }

        internal DirectorController Director => (DirectorController)modules[typeof(DirectorController)];

        // Diagnostics, see FlightRecorder
        private readonly FlightRecorder recorder;

        private void Add(AutopilotModule module) => modules[module.GetType()] = module;

        /// <summary>
        /// Engage a high-level autopilot, switching from the current one if needed.
        /// </summary>
        public void Engage(StateController controller)
        {
            if (controller == ActiveController)
                return;
            var switching = ActiveController != null;
            if (switching)
                ActiveController.Deactivate();
            else
            {
                Vessel.Sim.SetAutopilotEnableDisable(false);     // stock SAS would fight us
                Hook();
            }
            ActiveController = controller;
            controller.Activate();
            if (switching)
                FlightModel.sequential_dt = true;       // keep derivatives continuous across the switch
            ActiveControllerChanged?.Invoke();
        }

        /// <summary>
        /// Disengage the autopilot and hand control back to the pilot.
        /// </summary>
        public void Disengage()
        {
            if (ActiveController == null)
                return;
            foreach (var module in modules.Values)
                module.Deactivate();
            ActiveController = null;
            if (!FlightRecorder.Enabled)
                Unhook();
            ActiveControllerChanged?.Invoke();
        }

        private void Hook()
        {
            if (hooked)
                return;
            Vessel.View.OnAutopilotUpdate += OnAutopilotUpdate;
            hooked = true;
        }

        private void Unhook()
        {
            if (!hooked)
                return;
            if (Vessel.View != null)
                Vessel.View.OnAutopilotUpdate -= OnAutopilotUpdate;
            hooked = false;
        }

        /// <summary>
        /// Runs inside FlightInputHandler.OnFixedUpdate, after the pilot's input is read and before control surfaces,
        /// engines and reaction wheels consume it. KSP1 fired pre, main and post autopilot events; KSP2 only has the
        /// first two, so the flight model's pre and post steps run here around the controller.
        /// </summary>
        private void OnAutopilotUpdate(ref FlightCtrlState st, float deltaTime)
        {
            // Non-active vessels also fire this from Update with a neutral stick
            if (!Time.inFixedTimeStep)
                return;
            recorder.CountCall();
            var recording = FlightRecorder.Enabled;
            if (ActiveController == null && !recording)
                return;
            var view = GameManager.Instance.Game.ViewController;
            if (!view.IsActiveVessel(Vessel.Sim))
                return;
            var warp = view.TimeWarp;
            if (warp.IsWarping && !warp.IsPhysicsTimeWarp)
            {
                FlightModel.sequential_dt = false;
                return;
            }
            // The pilot switched stock SAS back on, so they want control back
            if (ActiveController != null && Vessel.Sim.Autopilot != null && Vessel.Sim.Autopilot.Enabled)
                Disengage();
            if (!Vessel.Refresh())
            {
                FlightModel.sequential_dt = false;
                return;
            }

            var engaged = ActiveController != null;
            var raw_in = st;
            var prev_applied = Vessel.Sim.flightCtrlState;
            ctrl.CopyFrom(st);
            // While recording, the flight model also runs disengaged so open-loop flight is measured. Its trainers
            // only run on the background thread when it is activated, so this does not train the model.
            if (FlightModel.Active || recording)
                FlightModel.OnPreAutopilot(ctrl);
            if (engaged)
            {
                ActiveController.ApplyControl(ctrl);
                offload_trim(raw_in);
            }
            if (FlightModel.Active || recording)
                FlightModel.OnPostAutopilot(ctrl);
            if (engaged)
                ctrl.CopyTo(ref st);

            if (recording)
                recorder.Record(raw_in, prev_applied, ctrl, engaged);
        }

        /// <summary>
        /// Time constant, s, the game's trim follows the autopilot's output with.
        /// </summary>
        public float trim_follow_time = 3.0f;

        // Smallest stick deflection that counts as the pilot flying that axis
        private const float pilot_input_tolerance = 1e-4f;

        // The game adds trim to the pilot's stick, and the autopilot replaces the sum every step, so trim does
        // nothing while engaged. Kept following the output, it is what holds the surfaces where the autopilot had
        // them once it lets go; left alone, they snapped back to the old trim and the nose moved. The low-pass
        // takes the steady deflection, not manoeuvres, and an axis the pilot is flying is left alone. The input
        // handler hands this step's state in by reference and pushes it to the vessel, so the trim written here is
        // the game's own.
        private void offload_trim(in FlightCtrlState raw_in)
        {
            var k = trim_follow_time > 0.0f ? 1.0f - Mathf.Exp(-Time.fixedDeltaTime / trim_follow_time) : 1.0f;
            if (Mathf.Abs(raw_in.pitch - raw_in.pitchTrim) < pilot_input_tolerance)
                ctrl.pitchTrim = Mathf.Clamp(ctrl.pitchTrim + (ctrl.pitch - ctrl.pitchTrim) * k, -1.0f, 1.0f);
            if (Mathf.Abs(raw_in.roll - raw_in.rollTrim) < pilot_input_tolerance)
                ctrl.rollTrim = Mathf.Clamp(ctrl.rollTrim + (ctrl.roll - ctrl.rollTrim) * k, -1.0f, 1.0f);
            if (Mathf.Abs(raw_in.yaw - raw_in.yawTrim) < pilot_input_tolerance)
                ctrl.yawTrim = Mathf.Clamp(ctrl.yawTrim + (ctrl.yaw - ctrl.yawTrim) * k, -1.0f, 1.0f);
        }

        public void Dispose()
        {
            Disengage();
            Unhook();
            recorder.Dispose();
        }
    }
}
