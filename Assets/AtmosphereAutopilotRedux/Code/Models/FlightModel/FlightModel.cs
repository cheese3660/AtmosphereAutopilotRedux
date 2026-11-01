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
using System.Text;
using System.Threading;
using UnityEngine;
using System.IO;

using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Modules;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Models
{

    using Vector = VectorArray.Vector;

    /// <summary>
    /// Short-term motion model. Is responsible for angular velocity, angular acceleration, control signal and 
    /// angle of attack evaluation and storage. Executes analysis of pitch, roll and yaw evolution and control authority.
    /// </summary>
    public sealed partial class FlightModel : AutopilotModule
    {
        internal FlightModel(AAVessel v) :
            base(v, "Flight model")
        {
            for (var i = 0; i < 3; i++)
            {
                input_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                csurf_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                gimbal_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                angular_v_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
                angular_acc_buf[i] = new CircularBufferAA<double>(BUFFER_SIZE, true);
                aoa_buf[i] = new CircularBufferAA<float>(BUFFER_SIZE, true);
            }
            initialize_lin_tainers();
        }

        protected override void OnActivate()
        {
            sequential_dt = false;
            var thread = AARuntime.BackgroundThread;
            thread.add_func(train_pitch_ann);
            thread.add_func(train_roll_ann);
            thread.add_func(train_yaw_ann);
            thread.add_func(train_pitch_lift);
            thread.add_func(train_yaw_lift);
            thread.Start();
        }

        protected override void OnDeactivate()
        {
            var thread = AARuntime.BackgroundThread;
            thread.remove_func(train_pitch_ann);
            thread.remove_func(train_roll_ann);
            thread.remove_func(train_yaw_ann);
            thread.remove_func(train_pitch_lift);
            thread.remove_func(train_yaw_lift);
            sequential_dt = false;
            moments_cycle_counter = 0;
        }

        private static readonly int BUFFER_SIZE = 8;



        #region Buffers

        private CircularBufferAA<float>[] input_buf = new CircularBufferAA<float>[3];

        private CircularBufferAA<float>[] csurf_buf = new CircularBufferAA<float>[3];   // True control surface action values

        private CircularBufferAA<float>[] gimbal_buf = new CircularBufferAA<float>[3];   // True gimbal action values

        private CircularBufferAA<float>[] angular_v_buf = new CircularBufferAA<float>[3];

        private CircularBufferAA<double>[] angular_acc_buf = new CircularBufferAA<double>[3];

        private CircularBufferAA<float>[] aoa_buf = new CircularBufferAA<float>[3];

        #endregion



        #region BufferExports

        /// <summary>
        /// Control signal history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBufferAA<float> ControlInputHistory(int axis) { return input_buf[axis]; }
        /// <summary>
        /// Control signal for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlInput(int axis) { return input_buf[axis].GetLast(); }

        /// <summary>
        /// Lagged control surface position history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBufferAA<float> ControlSurfPosHistory(int axis) { return csurf_buf[axis]; }
        /// <summary>
        /// Lagged control surface position for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float ControlSurfPos(int axis) { return csurf_buf[axis].GetLast(); }

        /// <summary>
        /// Lagged gimbal position history for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public CircularBufferAA<float> GimbalPosHistory(int axis) { return gimbal_buf[axis]; }
        /// <summary>
        /// Lagged gimbal position for pitch, roll or yaw. [-1.0, 1.0].
        /// </summary>
        public float GimbalPos(int axis) { return gimbal_buf[axis].GetLast(); }

        /// <summary>
        /// Angular velocity history for pitch, roll or yaw. Radians per second.
        /// </summary>
        public CircularBufferAA<float> AngularVelHistory(int axis) { return angular_v_buf[axis]; }
        /// <summary>
        /// Angular velocity for pitch, roll or yaw. Radians per second.
        /// </summary>
        public float AngularVel(int axis) { return angular_v_buf[axis].GetLast(); }

        /// <summary>
        /// Angular acceleration hitory for pitch, roll or yaw. Radians per second per second.
        /// </summary>
        public CircularBufferAA<double> AngularAccHistory(int axis) { return angular_acc_buf[axis]; }
        /// <summary>
        /// Angular acceleration for pitch, roll or yaw. Radians per second per second.
        /// </summary>
        public double AngularAcc(int axis) { return angular_acc_buf[axis].GetLast(); }

        /// <summary>
        /// Angle of attack hitory for pitch, roll or yaw. Radians.
        /// </summary>
        public CircularBufferAA<float> AoAHistory(int axis) { return aoa_buf[axis]; }
        /// <summary>
        /// Angle of attack for pitch, roll or yaw. Radians.
        /// </summary>
        public float AoA(int axis) { return aoa_buf[axis].GetLast(); }

        #endregion



        /// <summary>
        /// Called by the autopilot context after the controllers wrote their outputs. KSP2 has no
        /// OnPostAutopilotUpdate event, so the context calls this at the end of its own handler.
        /// </summary>
        internal void OnPostAutopilot(AACtrlState state)     // update control input
        {
            update_control(state);
            if (!vessel.LandedOrSplashed())
                sequential_dt = true;
            postupdate_dynamics();
        }

        internal bool sequential_dt = false;
        private Transform reference_transform_parent = null;

        /// <summary>
        /// Called by the autopilot context every physics step before the controllers run.
        /// </summary>
        internal void OnPreAutopilot(AACtrlState state)      // workhorse function
        {
            // The control transform is reparented to the controlling part, so track the parent.
            var cntrl_parent = vessel.ReferenceTransform.parent;
            if (reference_transform_parent != cntrl_parent)
                sequential_dt = false;
            reference_transform_parent = cntrl_parent;

            update_moments();
            if (!lin_models_seeded)
                seed_lin_models();
            update_velocity_acc();
            update_aoa();
            update_engine_moments();
            get_gimbal_authority();
            update_dynamics();

            if (sequential_dt)
            {
                if (angular_acc_buf[0].Size > 0 && !vessel.LandedOrSplashed())
                {
                    update_model_acc();
                    // Only learn while the autopilot flies. The flight recorder also runs the model disengaged, and
                    // hands-off flight with surfaces parked at zero tells the regression nothing about authority.
                    if (Active)
                    {
                        update_training_inputs();
                        update_cpu();
                    }
                    update_pitch_rot_model();
                    update_roll_rot_model();
                    update_yaw_rot_model();
                }
            }
        }
    }
}
