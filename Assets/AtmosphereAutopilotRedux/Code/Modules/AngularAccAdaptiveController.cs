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
using UnityEngine;
using System.IO;

using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{
    /// <summary>
    /// Controls angular acceleration. Meant to be used by AngularVelAdaptiveController
    /// </summary>
    public abstract class AngularAccAdaptiveController : SISOController
    {
        protected int axis;

        protected FlightModel imodel;

        /// <summary>
        /// Create controller instance.
        /// </summary>
        /// <param name="vessel">Vessel to control</param>
        /// <param name="module_name">Name of controller</param>
        /// <param name="wnd_id">unique for types window id</param>
        /// <param name="axis">Pitch = 0, roll = 1, yaw = 2</param>
        protected AngularAccAdaptiveController(AAVessel vessel, string module_name, int axis)
            : base(vessel, module_name)
        {
            this.axis = axis;
        }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            this.imodel = modules[typeof(FlightModel)] as FlightModel;
        }

        protected override void OnActivate()
        {
            imodel.Activate();
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
        }

        // [AutoGuiAttr("angular acc", false, "G6")]
        protected float acc;

        //[AutoGuiAttr("model acc", false, "G8")]
        //protected float model_acc;

        // [AutoGuiAttr("output", false, "G6")]
        public float output;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired angular acceleration</param>
        public override float ApplyControl(AACtrlState cntrl, float target_value)
        {
            predict_next_step();
            acc = (float)imodel.AngularAcc(axis);
            //model_acc = (float)imodel.model_acc[axis];
            desired_acc = target_value;

            var cur_input_raw = get_required_input(cntrl, desired_acc);
            output = cur_input_raw;
            if (float.IsNaN(output) || float.IsInfinity(output))
                output = 0.0f;

            // fighting numerical precision issues
            //if (axis == ROLL && imodel.dyn_pressure > 1000.0 && Mathf.Abs(output) < 0.006f)
            //    output = 0.0f;

            ControlUtils.set_raw_output(cntrl, axis, output);

            return output;
        }

        protected virtual float get_required_input(AACtrlState cntrl, float target_value)
        {
            return ControlUtils.getControlFromState(cntrl, axis);
        }

        #region Aero delay look-ahead

        // KSP2 computes aero forces from the surface deflection before this step's surface update
        // (FlightModel.aero_delay_steps), so the coming step's acceleration is already decided by the current
        // deflection. We predict where that step leaves angular velocity and AoA, and control the step after it.

        private double lookahead_time = double.NaN;

        /// <summary>
        /// Acceleration the model predicts for the coming physics step, from the current deflection.
        /// </summary>
        public double next_acc { get; private set; }

        /// <summary>
        /// Angular velocity after the coming physics step.
        /// </summary>
        public float lookahead_vel { get; private set; }

        /// <summary>
        /// Angle of attack after the coming physics step.
        /// </summary>
        public float lookahead_aoa { get; private set; }

        /// <summary>
        /// Run the look-ahead once per physics step. The velocity controller calls it before choosing its target,
        /// and ApplyControl calls it again harmlessly.
        /// </summary>
        public void predict_next_step()
        {
            if (Time.fixedTimeAsDouble == lookahead_time)
                return;
            lookahead_time = Time.fixedTimeAsDouble;
            var vel = imodel.AngularVel(axis);
            var aoa = imodel.AoA(axis);
            next_acc = 0.0;
            var aoa_rate = 0.0f;
            if (FlightModel.aero_delay_steps > 0)
            {
                next_acc = eval_next_acc(out aoa_rate);
                if (double.IsNaN(next_acc) || double.IsInfinity(next_acc))
                    next_acc = 0.0;
                if (float.IsNaN(aoa_rate) || float.IsInfinity(aoa_rate))
                    aoa_rate = 0.0f;
            }
            lookahead_vel = vel + (float)next_acc * Time.fixedDeltaTime;
            lookahead_aoa = aoa + aoa_rate * Time.fixedDeltaTime;
        }

        /// <summary>
        /// Acceleration over the coming step, with the surfaces held where they are now.
        /// </summary>
        protected virtual double eval_next_acc(out float aoa_rate)
        {
            aoa_rate = 0.0f;
            return 0.0;
        }

        #endregion

        // Diagnostics for FlightRecorder
        internal virtual double DebugPredictedAcc => double.NaN;
        internal virtual double DebugAuthority => double.NaN;
        // 0 = low authority fallback, 1 = exponential lag model, 2 = undelayed model, 3 = surface speed exceeded
        internal int DebugBranch;

        // [AutoGuiAttr("Csurf output", false, "G6")]
        protected float csurf_output => imodel.ControlSurfPos(axis);

        // [AutoGuiAttr("desired acc", false, "G6")]
        internal float desired_acc { get; private set; }
    }


    //
    // Three realizations
    //

    public abstract class PitchYawAngularAccController : AngularAccAdaptiveController
    {
        protected PitchYawAngularAccController(AAVessel vessel, string module_name, int axis)
            : base(vessel, module_name, axis)
        { }

        protected Matrix cur_state = new Matrix(4, 1);
        protected Matrix input_mat = new Matrix(1, 1);

        // [AutoGuiAttr("model_predicted_acc", false, "G6")]
        protected double model_predicted_acc;

        protected double cur_model_acc, prev_model_acc;

        internal override double DebugPredictedAcc => model_predicted_acc;
        internal override double DebugAuthority => authority;

        // [AutoGuiAttr("authority", false, "G6")]
        protected double authority;

        protected LinearSystemModel lin_model, lin_model_undelayed;

        private Matrix next_state = new Matrix(3, 1);
        private Matrix next_input = new Matrix(1, 1);

        protected override double eval_next_acc(out float aoa_rate)
        {
            // Undelayed model with the current deflection as its input
            next_state[0, 0] = imodel.AoA(axis);
            next_state[1, 0] = imodel.AngularVel(axis);
            next_state[2, 0] = imodel.GimbalPos(axis);
            next_input[0, 0] = imodel.ControlSurfPos(axis);
            aoa_rate = (float)lin_model_undelayed.eval_row(0, next_state, next_input);
            return lin_model_undelayed.eval_row(1, next_state, next_input);
        }

        protected override float get_required_input(AACtrlState cntrl, float target_value)
        {
            var new_input = 0.0f;

            if (imodel.csurf_exponential)
            {
                DebugBranch = 1;
                // Exponential surfaces (the KSP1 FAR branch): the surface is a model state, and this step it covers
                // csurf_lag_k of the way to the new input, which the delayed model's B already accounts for.
                authority = lin_model.B[1, 0];

                // check if we have inadequate model authority
                if (authority < 1e-4)
                {
                    DebugBranch = 0;
                    var user_input = axis == PITCH ? cntrl.pitch : cntrl.yaw;
                    if (user_input != 0.0f)
                        return user_input;
                    else
                        return Common.Clampf(target_value, 1.0f);
                }

                // get model prediction for the step after the coming one
                cur_state[0, 0] = lookahead_aoa;
                cur_state[1, 0] = lookahead_vel;
                cur_state[2, 0] = imodel.ControlSurfPos(axis);
                cur_state[3, 0] = imodel.GimbalPos(axis);
                input_mat[0, 0] = cur_state[2, 0];
                var lag_acc_prediction = lin_model.eval_row(1, cur_state, input_mat);

                var lag_acc_error = target_value - lag_acc_prediction;
                new_input = (float)(cur_state[2, 0] + lag_acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);

                model_predicted_acc = lag_acc_prediction + authority * (new_input - cur_state[2, 0]);
                return new_input;
            }

            DebugBranch = 2;
            authority = lin_model_undelayed.B[1, 0];

            // check if we have inadequate model authority
            if (authority < 1e-4)
            {
                DebugBranch = 0;
                var user_input = axis == PITCH ? cntrl.pitch : cntrl.yaw;
                if (user_input != 0.0f)
                    return user_input;
                else
                    return Common.Clampf(target_value, 1.0f);
            }

            // get model prediction for the step after the coming one
            cur_state[0, 0] = lookahead_aoa;
            cur_state[1, 0] = lookahead_vel;
            cur_state[2, 0] = imodel.GimbalPos(axis);
            input_mat[0, 0] = imodel.ControlSurfPos(axis);
            var cur_acc_prediction = lin_model_undelayed.eval_row(1, cur_state, input_mat);

            var acc_error = target_value - cur_acc_prediction;
            new_input = (float)(input_mat[0, 0] + acc_error / authority);
            new_input = Common.Clampf(new_input, 1.0f);

            if (Math.Abs(new_input - input_mat[0, 0]) / Time.fixedDeltaTime > imodel.csurf_spd)
            {
                // we're exceeding control surface speed
                DebugBranch = 3;
                cur_state[3, 0] = cur_state[2, 0];
                cur_state[2, 0] = input_mat[0, 0];
                if (new_input > input_mat[0, 0])
                    cur_state[2, 0] += Time.fixedDeltaTime * imodel.csurf_spd;
                else
                    cur_state[2, 0] -= Time.fixedDeltaTime * imodel.csurf_spd;
                cur_state[2, 0] = Common.Clamp(cur_state[2, 0], 1.0);
                input_mat[0, 0] = cur_state[2, 0];
                cur_acc_prediction = lin_model.eval_row(1, cur_state, input_mat);
                acc_error = target_value - cur_acc_prediction;
                authority = lin_model.B[1, 0];
                new_input = (float)(cur_state[2, 0] + acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);
            }
            model_predicted_acc = cur_acc_prediction + authority * (new_input - input_mat[0, 0]);

            return new_input;
        }
    }

    public sealed class PitchAngularAccController : PitchYawAngularAccController
    {
        internal PitchAngularAccController(AAVessel vessel)
            : base(vessel, "Pitch ang acc controller", PITCH)
        { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            base.InitializeDependencies(modules);
            lin_model = imodel.pitch_rot_model;
            lin_model_undelayed = imodel.pitch_rot_model_undelayed;
        }
    }

    public sealed class YawAngularAccController : PitchYawAngularAccController
    {
        internal YawAngularAccController(AAVessel vessel)
            : base(vessel, "Yaw ang acc controller", YAW)
        { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            base.InitializeDependencies(modules);
            lin_model = imodel.yaw_rot_model;
            lin_model_undelayed = imodel.yaw_rot_model_undelayed;
        }
    }

    public sealed class RollAngularAccController : AngularAccAdaptiveController
    {
        internal RollAngularAccController(AAVessel vessel)
            : base(vessel, "Roll ang acc controller", ROLL)
        { }

        private AngularAccAdaptiveController yc;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            base.InitializeDependencies(modules);
            yc = modules[typeof(YawAngularAccController)] as AngularAccAdaptiveController;
        }

        private Matrix cur_state = new Matrix(3, 1);
        private Matrix input_mat = new Matrix(3, 1);

        private Matrix next_state = new Matrix(2, 1);
        private Matrix next_input = new Matrix(3, 1);

        protected override double eval_next_acc(out float aoa_rate)
        {
            // Undelayed model with the current roll and yaw deflections as its inputs
            aoa_rate = 0.0f;
            next_state[0, 0] = imodel.AngularVel(ROLL);
            next_state[1, 0] = imodel.GimbalPos(ROLL);
            next_input[0, 0] = imodel.ControlSurfPos(ROLL);
            next_input[1, 0] = imodel.ControlSurfPos(YAW);
            next_input[2, 0] = imodel.AoA(YAW);
            return imodel.roll_rot_model_undelayed.eval_row(0, next_state, next_input);
        }

        // [AutoGuiAttr("model_predicted_acc", false, "G6")]
        private double model_predicted_acc;

        //double cur_model_acc;
        //double prev_model_acc;

        internal override double DebugPredictedAcc => model_predicted_acc;
        internal override double DebugAuthority => authority;

        // [AutoGuiAttr("authority", false, "G6")]
        private double authority;

        protected override float get_required_input(AACtrlState cntrl, float target_value)
        {
            var new_input = 0.0f;

            if (imodel.csurf_exponential)
            {
                DebugBranch = 1;
                // Exponential surfaces (the KSP1 FAR branch), see PitchYawAngularAccController
                authority = imodel.roll_rot_model.B[0, 0];
                // check if we have inadequate model authority
                if (authority < 1e-4)
                {
                    DebugBranch = 0;
                    var user_input = cntrl.roll;
                    if (user_input != 0.0f)
                        return user_input;
                    else
                        return Common.Clampf(target_value, 1.0f);
                }

                // get model prediction for the step after the coming one
                cur_state[0, 0] = lookahead_vel;
                cur_state[1, 0] = imodel.ControlSurfPos(ROLL);
                cur_state[2, 0] = imodel.GimbalPos(ROLL);
                input_mat[0, 0] = cur_state[1, 0];
                input_mat[1, 0] = imodel.stock_actuator_blend(imodel.ControlSurfPos(YAW), yc.output);
                input_mat[2, 0] = imodel.AoA(YAW);
                var lag_acc_prediction = imodel.roll_rot_model.eval_row(0, cur_state, input_mat);

                var lag_acc_error = target_value - lag_acc_prediction;
                new_input = (float)(cur_state[1, 0] + lag_acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);

                model_predicted_acc = lag_acc_prediction + authority * (new_input - cur_state[1, 0]);
                return new_input;
            }

            DebugBranch = 2;
            authority = imodel.roll_rot_model_undelayed.B[0, 0];
            // check if we have inadequate model authority
            if (authority < 1e-4)
            {
                DebugBranch = 0;
                var user_input = cntrl.roll;
                if (user_input != 0.0f)
                    return user_input;
                else
                    return Common.Clampf(target_value, 1.0f);
            }

            // get model prediction for the step after the coming one
            cur_state[0, 0] = lookahead_vel;
            cur_state[1, 0] = imodel.GimbalPos(ROLL);
            input_mat[0, 0] = imodel.ControlSurfPos(ROLL);
            input_mat[1, 0] = imodel.stock_actuator_blend(imodel.ControlSurfPos(YAW), yc.output);
            input_mat[2, 0] = imodel.AoA(YAW);
            var cur_acc_prediction = imodel.roll_rot_model_undelayed.eval_row(0, cur_state, input_mat);

            var acc_error = target_value - cur_acc_prediction;
            new_input = (float)(input_mat[0, 0] + acc_error / authority);
            new_input = Common.Clampf(new_input, 1.0f);

            if (Math.Abs(new_input - input_mat[0, 0]) / Time.fixedDeltaTime > imodel.csurf_spd)
            {
                // we're exceeding control surface speed
                DebugBranch = 3;
                cur_state[2, 0] = cur_state[1, 0];
                cur_state[1, 0] = input_mat[0, 0];
                if (new_input > input_mat[0, 0])
                    cur_state[1, 0] += Time.fixedDeltaTime * imodel.csurf_spd;
                else
                    cur_state[1, 0] -= Time.fixedDeltaTime * imodel.csurf_spd;
                cur_state[1, 0] = Common.Clamp(cur_state[1, 0], 1.0);
                input_mat[0, 0] = cur_state[1, 0];
                cur_acc_prediction = imodel.roll_rot_model.eval_row(0, cur_state, input_mat);
                acc_error = target_value - cur_acc_prediction;
                authority = imodel.roll_rot_model.B[0, 0];
                new_input = (float)(cur_state[1, 0] + acc_error / authority);
                new_input = Common.Clampf(new_input, 1.0f);
            }
            model_predicted_acc = cur_acc_prediction + authority * (new_input - input_mat[0, 0]);

            return new_input;
        }
    }

}
