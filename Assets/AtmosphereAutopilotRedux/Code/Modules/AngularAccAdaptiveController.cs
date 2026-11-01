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
            reset_indi_scale();
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
                update_indi_scale();
                // The incremental estimate needs no aero model beyond authority
                var indi_next = indi_next_acc();
                DebugIndiNext = indi_next;
                if (!double.IsNaN(indi_next))
                    next_acc = indi_next;
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

        #region Incremental control

        // Incremental nonlinear dynamic inversion (INDI), written from the published concept. The measured angular
        // acceleration already holds every aerodynamic and engine effect, so only the change the surfaces make has
        // to be modelled, and that needs nothing but control authority. The learned constant, AoA, damping and
        // sideslip terms drop out, and with them the prediction errors they gave at high dynamic pressure.
        //
        // Timing, as the regression pairs it: the acceleration measured this step answers to the surface position
        // from two commands ago. The one committed last step shows up in the coming step, and the one chosen now in
        // the step after it, which is the step the velocity controller's target is for.

        /// <summary>
        /// Use incremental control when the game's surface authority is available for this axis.
        /// </summary>
        public bool indi_control = true;

        /// <summary>
        /// Share of the remaining acceleration error the incremental law corrects per step.
        /// </summary>
        public float indi_gain = 1.0f;

        // Set by indi_required_input for the derived controllers' diagnostics
        protected double indi_authority = double.NaN;
        protected double indi_predicted_acc = double.NaN;

        /// <summary>
        /// Acceleration the incremental estimate expects for the coming step. NaN when it isn't running.
        /// </summary>
        internal double DebugIndiNext = double.NaN;

        /// <summary>
        /// Time over which old samples fade from the authority estimate, s.
        /// </summary>
        public float indi_scale_forget_time = 1.5f;

        /// <summary>
        /// Range the authority estimate may take, as a multiple of the game's value.
        /// </summary>
        public float indi_scale_min = 0.05f, indi_scale_max = 1.5f;

        /// <summary>
        /// Time constant of the smoothing between the fitted scale and the one incremental control uses while the
        /// fit is above it, s. During fast transients the fit is pulled off by the controller's own reactions for a
        /// second or two, so the used scale follows the fit slowly.
        /// </summary>
        public float indi_scale_smooth_time = 3.0f;

        /// <summary>
        /// The same while the fit is below the used scale, s. Past about Mach 3.5 the real authority can fall to a
        /// quarter within a few seconds, and following that too slowly leaves the controller overestimating it.
        /// </summary>
        public float indi_scale_fall_time = 1.0f;

        // Weight of the starting value, in squared rad/s^2 of predicted acceleration step. Small, so a few real
        // surface steps outweigh it, and it fades with the rest.
        private const double indi_scale_prior = 1e-3;

        private double indi_sxx, indi_sxy;
        private double indi_scale_fit = 1.0;
        private double indi_scale = 1.0;

        /// <summary>
        /// Real surface authority as a multiple of the game's value, as incremental control uses it.
        /// </summary>
        internal double DebugIndiScale => indi_scale;

        /// <summary>
        /// The fitted scale before smoothing.
        /// </summary>
        internal double DebugIndiScaleFit => indi_scale_fit;

        /// <summary>
        /// Surface authority incremental control works with, rad/s^2 of net angular acceleration per full
        /// deflection: the game's value with the measured scale and PhysX damping on it. NaN when incremental
        /// control isn't running.
        /// </summary>
        internal double IndiAuthority => IndiActive ? indi_surface_authority() * imodel.AngularDampingFactor : double.NaN;

        private void reset_indi_scale()
        {
            indi_sxx = indi_scale_prior;
            indi_sxy = indi_scale_prior;
            indi_scale_fit = 1.0;
            indi_scale = 1.0;
        }

        // The game's surface authority is right at low speed but runs ahead of the real one with Mach, by three to
        // four times near Mach 4 in flight tests, and incremental control goes unstable once it overestimates by
        // about three. The acceleration step answers to the surface step two commands back, cleanly enough to fit
        // even in steady flight, so the scale on the game's value is fitted from that by least squares with
        // exponential forgetting. Small steps weigh little, so quiet flight keeps the last estimate.
        //
        // The fit uses second differences, the change of each step. In steady flight the surfaces creep with trim
        // while the acceleration stays at zero, since the trim is what cancels it, and on plain steps that creep
        // pulled the fit to its floor in level flight. A steady creep has no second difference.
        private void update_indi_scale()
        {
            if (!indi_control)
                return;
            if (!indi_off_ground())
            {
                // Start the fit afresh from the current scale once airborne, so nothing from the ground lingers
                indi_sxx = indi_scale_prior;
                indi_sxy = indi_scale_prior * indi_scale;
                indi_scale_fit = indi_scale;
                return;
            }
            var surface_authority = imodel.IndiSurfaceAuthority(axis);
            if (double.IsNaN(surface_authority) || surface_authority <= 0.0)
                return;
            var vel_history = imodel.AngularVelHistory(axis);
            var surface = imodel.ControlSurfPosHistory(axis);
            var input = imodel.ControlInputHistory(axis);
            if (vel_history.Size < 4 || surface.Size < 4 || input.Size < 4)
                return;
            // The fit runs on the torque acceleration, before PhysX damping, so the scale is only what the game's
            // lift estimate misses
            var x = surface_authority * second_difference(surface, 1);
            var y = indi_torque_acc(0) - 2.0 * indi_torque_acc(1) + indi_torque_acc(2) -
                imodel.IndiDirectAuthority(axis) * second_difference(input, 1);
            if (double.IsNaN(x) || double.IsNaN(y) || double.IsInfinity(x) || double.IsInfinity(y))
                return;
            var forget = Math.Exp(-Time.fixedDeltaTime / indi_scale_forget_time);
            indi_sxx = forget * indi_sxx + x * x;
            indi_sxy = forget * indi_sxy + x * y;
            if (indi_sxx > 1e-12)
                indi_scale_fit = Common.Clamp(indi_sxy / indi_sxx, indi_scale_min, indi_scale_max);
            var smooth_time = indi_scale_fit < indi_scale ? indi_scale_fall_time : indi_scale_smooth_time;
            indi_scale += (indi_scale_fit - indi_scale) * (1.0 - Math.Exp(-Time.fixedDeltaTime / smooth_time));
        }

        /// <summary>
        /// Time after the last ground contact, s, before incremental control and its authority fit trust the
        /// measured acceleration. The differences it works from reach a few steps back, and on a bouncing takeoff
        /// one sample spanning a wheel strike pulled the yaw and roll scale to its floor.
        /// </summary>
        public float indi_liftoff_settle_time = 0.5f;

        private double indi_last_ground_time = double.NegativeInfinity;

        private bool indi_off_ground()
        {
            var now = Time.fixedTimeAsDouble;
            if (vessel.LandedOrSplashed())
                indi_last_ground_time = now;
            return now - indi_last_ground_time >= indi_liftoff_settle_time;
        }

        // Change of the step ending shift entries back from the newest
        private static double second_difference(CircularBufferAA<float> buf, int shift)
        {
            return buf.GetFromTail(shift) - 2.0 * buf.GetFromTail(shift + 1) + buf.GetFromTail(shift + 2);
        }

        // The game's surface authority with the measured scale on it
        private double indi_surface_authority()
        {
            return imodel.IndiSurfaceAuthority(axis) * indi_scale;
        }

        /// <summary>
        /// Lowest damping factor incremental control divides by. Past it the vessel barely rotates at all and the
        /// speed limit (ProgradeThrustController.max_damping_dt) is what keeps it out of there, so this sits below
        /// the factor that limit allows.
        /// </summary>
        public float indi_min_damping_factor = FlightModel.MinAngularDampingFactor;

        private float indi_damping_factor()
        {
            return Mathf.Max(indi_min_damping_factor, imodel.AngularDampingFactor);
        }

        // Torque acceleration over the step ending back steps ago: the rate change before PhysX damping took its
        // share, w_new = f * (w_old + torque_acc * dt)
        private double indi_torque_acc(int back)
        {
            var vel = imodel.AngularVelHistory(axis);
            return (vel.GetFromTail(back) / indi_damping_factor() - vel.GetFromTail(back + 1)) / Time.fixedDeltaTime;
        }

        /// <summary>
        /// Whether incremental control runs this step. Valid after predict_next_step.
        /// </summary>
        internal bool IndiActive => !double.IsNaN(DebugIndiNext);

        // The coming step's torque acceleration: the latest measured one plus what the already committed surface
        // position and direct input change. NaN when incremental control can't run.
        protected double indi_next_torque_acc()
        {
            // On the ground the measured acceleration carries wheel and ground forces no surface accounts for
            if (!indi_control || !indi_off_ground())
                return double.NaN;
            var surface_authority = indi_surface_authority();
            if (double.IsNaN(surface_authority) || surface_authority <= 0.0)
                return double.NaN;
            var vel_history = imodel.AngularVelHistory(axis);
            var surface = imodel.ControlSurfPosHistory(axis);
            var input = imodel.ControlInputHistory(axis);
            if (vel_history.Size < 2 || surface.Size < 2 || input.Size < 2)
                return double.NaN;
            var direct_authority = imodel.IndiDirectAuthority(axis);
            return indi_torque_acc(0) +
                surface_authority * (surface.GetLast() - surface.GetFromTail(1)) +
                direct_authority * (input.GetLast() - input.GetFromTail(1));
        }

        // The coming step's net angular acceleration, PhysX damping included
        protected double indi_next_acc()
        {
            var torque_acc = indi_next_torque_acc();
            if (double.IsNaN(torque_acc))
                return double.NaN;
            var vel = imodel.AngularVel(axis);
            return (indi_damping_factor() * (vel + torque_acc * Time.fixedDeltaTime) - vel) / Time.fixedDeltaTime;
        }

        // The input that takes the step after the coming one to the target acceleration, or false to fall back to
        // the model-based law
        protected bool indi_required_input(float target_value, out float new_input)
        {
            new_input = 0.0f;
            var next = indi_next_torque_acc();
            if (double.IsNaN(next))
                return false;
            // The target is a net rate change from the look-ahead rate. PhysX damps the torque's change along with
            // the rate itself, w_after = f * (w_look + torque_acc * dt), so the torque acceleration needed is
            var f = indi_damping_factor();
            var dt = Time.fixedDeltaTime;
            var target_torque_acc = target_value / f + lookahead_vel * (1.0 / f - 1.0) / dt;
            var surface_authority = indi_surface_authority();
            var direct_authority = imodel.IndiDirectAuthority(axis);
            var surface = imodel.ControlSurfPos(axis);
            var prev_input = imodel.ControlInput(axis);
            // The surface covers lag_k of the way to the input this step. Rate-limited surfaces are taken to reach
            // it and the measurement corrects the rest.
            var lag_k = imodel.csurf_exponential ? imodel.csurf_lag_k : 1.0f;
            var authority = surface_authority * lag_k + direct_authority;
            if (authority < 1e-4)
                return false;
            // target = next + surface_authority * lag_k * (u - surface) + direct_authority * (u - prev_input)
            var u = (indi_gain * (target_torque_acc - next) + surface_authority * lag_k * surface +
                direct_authority * prev_input) / authority;
            new_input = Common.Clampf((float)u, 1.0f);
            indi_authority = authority * f;
            var predicted_torque_acc = next + surface_authority * lag_k * (new_input - surface) +
                direct_authority * (new_input - prev_input);
            indi_predicted_acc = (f * (lookahead_vel + predicted_torque_acc * dt) - lookahead_vel) / dt;
            return !float.IsNaN(new_input);
        }

        #endregion

        // Diagnostics for FlightRecorder
        internal virtual double DebugPredictedAcc => double.NaN;
        internal virtual double DebugAuthority => double.NaN;
        // 0 = low authority fallback, 1 = exponential lag model, 2 = undelayed model, 3 = surface speed exceeded,
        // 4 = incremental
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

            if (indi_required_input(target_value, out var indi_input))
            {
                DebugBranch = 4;
                authority = indi_authority;
                model_predicted_acc = indi_predicted_acc;
                return indi_input;
            }

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

            if (indi_required_input(target_value, out var indi_input))
            {
                DebugBranch = 4;
                authority = indi_authority;
                model_predicted_acc = indi_predicted_acc;
                return indi_input;
            }

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
