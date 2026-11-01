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

using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{

    /// <summary>
    /// Controls surface velocity vector
    /// </summary>
    public sealed class DirectorController : AutopilotModule
    {
        internal DirectorController(AAVessel v)
            : base(v, "Director controller")
        { }

        private PitchAoAController aoa_c;
        private SideslipController side_c;
        private PitchAngularVelocityController pitch_c;
        private YawAngularVelocityController yaw_c;
        private RollAngularVelocityController roll_c;
        private ProgradeThrustController thrust_c;
        private FlightModel imodel;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            aoa_c = modules[typeof(PitchAoAController)] as PitchAoAController;
            side_c = modules[typeof(SideslipController)] as SideslipController;
            pitch_c = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            roll_c = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            yaw_c = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        private bool aoa_moderation_saved = false;
        private bool sideslip_moderation_saved = false;

        protected override void OnActivate()
        {
            imodel.Activate();
            aoa_c.Activate();
            side_c.Activate();      // upstream called side_c without activating it
            yaw_c.Activate();
            roll_c.Activate();
            commanded_turn_acc = Vector3d.zero;
            turn_limit_frac = 1.0;
            vert_integral = 0.0;
            turn_error_filter_reset = true;
            held_pitch_rate = double.NaN;
            held_yaw_rate = double.NaN;
            // save moderation states
            aoa_moderation_saved = pitch_c.moderate_aoa;
            sideslip_moderation_saved = yaw_c.moderate_aoa;
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            aoa_c.Deactivate();
            side_c.Deactivate();
            yaw_c.Deactivate();
            roll_c.Deactivate();
            // restore moderation states
            pitch_c.moderate_aoa = aoa_moderation_saved;
            yaw_c.moderate_aoa = sideslip_moderation_saved;
        }

        // [VesselSerializable("strength")]
        // [AutoGuiAttr("strength", true, "G4")]
        public double strength = 0.95;
        public string strength_as_text
        {
            set => this.strength = double.TryParse(value, out var v) ? v : this.strength;
            get => this.strength.ToString("G4");
        }

        // [AutoGuiAttr("angular error", false, "G5")]
        public double angular_error;

        /// <summary>
        /// Time constant, s, of heading tracking: turn acceleration is horizontal velocity error over this.
        /// </summary>
        public double turn_time = 6.0;

        /// <summary>
        /// Time constant, s, of the low-pass on the heading error before it becomes a turn command.
        /// </summary>
        public double turn_error_filter_time = 1.0;

        private Vector3d filtered_turn_error;
        private bool turn_error_filter_reset = true;

        /// <summary>
        /// Time constant, s, of the low-pass on the turn rate that the equilibrium AoA balances PhysX damping for.
        /// </summary>
        public double held_rate_filter_time = 2.0;

        private double held_pitch_rate = double.NaN, held_yaw_rate = double.NaN;

        /// <summary>
        /// Time constant, s, of vertical velocity tracking.
        /// </summary>
        public double vert_time = 1.0;

        /// <summary>
        /// Time for the vertical integral to match a steady vertical speed error, s, on top of vert_time.
        /// </summary>
        public double vert_integral_time = 5.0;

        /// <summary>
        /// Largest acceleration the vertical integral adds, m/s^2.
        /// </summary>
        public double vert_integral_limit = 5.0;

        /// <summary>
        /// Vertical speed error, m/s, beyond which the vertical integral holds still.
        /// </summary>
        public double vert_integral_band = 5.0;

        /// <summary>
        /// Set by the caller each step while its vertical command is still changing, to hold the vertical integral.
        /// </summary>
        public bool hold_vert_integral;

        private double vert_integral;

        /// <summary>
        /// Vertical acceleration the integral adds, m/s^2.
        /// </summary>
        internal double DebugVertIntegral => vert_integral;

        /// <summary>
        /// Hardest turn, in g of horizontal acceleration, ever asked for.
        /// </summary>
        public double max_turn_g = 4.0;

        /// <summary>
        /// Fastest change of the turn acceleration, g per second. Lets the bank build and unwind smoothly.
        /// </summary>
        public double turn_jerk_g = 0.5;

        /// <summary>
        /// Speed deficit, m/s, under the speed hold setpoint over which the turn limit shrinks to its floor.
        /// </summary>
        public double turn_speed_margin = 20.0;

        /// <summary>
        /// Fraction of max_turn_g kept however much speed the turn is bleeding.
        /// </summary>
        public double turn_limit_floor = 0.075;

        /// <summary>
        /// How fast the turn limit fraction may change, per second.
        /// </summary>
        public double turn_limit_rate = 0.2;

        private Vector3d commanded_turn_acc = Vector3d.zero;
        private double turn_limit_frac = 1.0;

        // Diagnostics for FlightRecorder
        internal double debug_turn_limit, debug_turn_acc, debug_turn_error;

        // [VesselSerializable("max_neg_g")]
        // [AutoGuiAttr("max_neg_g", true, "G5")]
        public double max_neg_g = 8.0;

        // [AutoGuiAttr("min_rollover_alt", true, "G5")]
        public double min_rollover_alt = 150.0;

        public double max_lift_acc = 0.0;
        public double max_sideslip_acc = 0.0;

        private bool precision_mode = false;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="desired_vel">Desired velocity direction in surface reference frame.</param>
        /// <param name="desired_acceleration">Desired acceleration.</param>
        public void ApplyControl(AACtrlState state, Vector3d desired_vel, Vector3d desired_acceleration)
        {
            precision_mode = state.precision;
            var planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            var planet2vesNorm = planet2ves.normalized;
            var shift_acc = Vector3d.zero;

            // Following the planet's curve at constant altitude takes a centripetal acceleration of v^2 / r toward
            // its centre, so level flight needs that much less lift than weight. Upstream left it out, which is fine
            // at airliner speeds, but at Mach 4 near Kerbin's surface it is about 3.4 m/s^2, a third of g, and the
            // altitude hold settled 10 to 13 m high asking for a descent it never got.
            var horizontal_v = imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm);
            var level_acc = -planet2vesNorm * horizontal_v.sqrMagnitude / planet2ves.magnitude;

            // Rotation vector
            var desired_turn_acc_dir = Vector3d.Cross(
                Vector3d.Cross(imodel.surface_v, desired_vel).normalized,
                imodel.surface_v).normalized;
            angular_error = Vector3d.Angle(imodel.surface_v.normalized, desired_vel) * dgr2rad;

            max_lift_acc = Math.Max(0.01, max_lift_acceleration(PITCH));
            max_sideslip_acc = Math.Max(0.001, max_lift_acceleration(YAW));

            // let's find this craft's maximum acceleration toward desired_turn_acc_dir without stalling
            // it can be solved from simple quadratic equation
            var max_turn_acc = non_stall_turn_acceleration(desired_turn_acc_dir, max_lift_acc);
            max_turn_acc = strength * max_turn_acc * (precision_mode ? 0.4 : 1.0);

            // Upstream turned at max_turn_acc until the remaining error fit in the roll's stopping time, then eased
            // off. That made the turn gain V / stop_time, and fast-rolling craft stop in under a second, so a 1 degree
            // heading change at 500 m/s asked for over 1 g and snap-rolled into it. Instead the velocity error is
            // tracked with time constants: slow for heading, quicker for climb and descent, whose command is already
            // shaped by the cruise altitude hold.
            var dt = Time.fixedDeltaTime;
            var speed = imodel.surface_v_magnitude;
            var vel_error = desired_vel.normalized * speed - imodel.surface_v;
            vel_error -= Vector3d.Project(vel_error, imodel.surface_v);      // speed is the thrust controller's job
            var vert_error = Vector3d.Project(vel_error, planet2vesNorm);
            var turn_error = vel_error - vert_error;
            // At Mach 4 the velocity follows the nose almost rigidly, so every small yaw wiggle shows up as heading
            // error, and V / turn_time turns it into bank: a 1 mrad yaw asked for about 2 degrees of roll. Roll then
            // stirred the yaw again. The heading loop is slow, so it loses little to a low-pass that the wiggle
            // can't get through.
            if (turn_error_filter_reset || turn_error_filter_time <= 0.0)
                filtered_turn_error = turn_error;
            else
            {
                filtered_turn_error += (turn_error - filtered_turn_error) * (1.0 - Math.Exp(-dt / turn_error_filter_time));
                // keep it horizontal and normal to the velocity as both rotate
                filtered_turn_error -= Vector3d.Project(filtered_turn_error, planet2vesNorm);
                filtered_turn_error -= Vector3d.Project(filtered_turn_error, imodel.surface_v);
            }
            turn_error_filter_reset = false;
            turn_error = filtered_turn_error;

            // The turn limit shrinks while the turn bleeds speed below the speed hold setpoint, so the craft turns as
            // hard as its engines can sustain
            var limit_frac_target = 1.0;
            if (thrust_c.spd_control_enabled)
            {
                var spd_diff = speed - thrust_c.setpoint_mps;
                if (spd_diff < 0.0)
                    limit_frac_target = Math.Max(turn_limit_floor, 1.0 + spd_diff / turn_speed_margin);
            }
            turn_limit_frac += Common.Clamp(limit_frac_target - turn_limit_frac, turn_limit_rate * dt);
            var turn_limit = Math.Min(max_turn_g * 9.81 * turn_limit_frac, max_turn_acc.magnitude);

            var wanted_turn_acc = turn_error / turn_time;
            if (wanted_turn_acc.magnitude > turn_limit)
                wanted_turn_acc *= turn_limit / wanted_turn_acc.magnitude;
            // Jerk limit, so the bank angle builds and unwinds instead of snapping
            var turn_step = wanted_turn_acc - commanded_turn_acc;
            var max_step = turn_jerk_g * 9.81 * dt;
            if (turn_step.magnitude > max_step)
                turn_step *= max_step / turn_step.magnitude;
            commanded_turn_acc += turn_step;
            // keep it horizontal and normal to the velocity as both rotate
            commanded_turn_acc -= Vector3d.Project(commanded_turn_acc, planet2vesNorm);
            commanded_turn_acc -= Vector3d.Project(commanded_turn_acc, imodel.surface_v);

            // A slow integral on the vertical error takes out what the lift model and the curvature term still
            // miss, Coriolis among them. It holds still while the lift limit is in the way.
            shift_acc = commanded_turn_acc + vert_error / vert_time + planet2vesNorm * vert_integral;
            if (shift_acc.magnitude > max_turn_acc.magnitude)
                shift_acc *= max_turn_acc.magnitude / shift_acc.magnitude;
            else
            {
                // Only while the vertical command is settled: during an altitude change the vessel trails or leads
                // the profile by several m/s, and integrating that wound the integral end to end in a descent and
                // bounced the level-off 15 m back up
                var vert_speed_error = Vector3d.Dot(vert_error, planet2vesNorm);
                if (!hold_vert_integral && Math.Abs(vert_speed_error) < vert_integral_band)
                    vert_integral = Common.Clamp(vert_integral + vert_speed_error / (vert_time * vert_integral_time) * dt,
                        vert_integral_limit);
            }

            debug_turn_limit = turn_limit;
            debug_turn_acc = commanded_turn_acc.magnitude;
            debug_turn_error = turn_error.magnitude / Math.Max(1.0, speed) * rad2dgr;

            //if (angular_error > 0.2 || Vector3d.Dot(desired_acceleration, shift_acc) < -0.1)
            //    target_acc = shift_acc;
            //else
            target_acc = desired_acceleration + shift_acc + level_acc;
            //current_acc = imodel.sum_acc;

            // we need aoa moderation for AoA controllers to work
            pitch_c.moderate_aoa = true;
            yaw_c.moderate_aoa = true;

            var neutral_acc = -imodel.gravity_acc - imodel.noninert_acc;
            var target_lift_acc = target_acc + neutral_acc;
            var target_normal_lift_acc = target_lift_acc - Vector3d.Project(target_lift_acc, imodel.surface_v);

            // Prevent rolling on small errors when little lift is wanted: the bank angle is the lateral demand over
            // the lift, so in a pushover a 0.2 m/s^2 heading correction asked for 8 degrees. Upstream dropped the
            // lateral part outright below 0.3 g with the velocity within 0.02 rad of its target, which stepped the
            // bank target by all of that whenever either crossed. It fades out instead, over 0.3 to 0.6 g of lift
            // and 0.02 to 0.04 rad of error, keeping whichever asks for more of it.
            var lift_g = target_normal_lift_acc.magnitude / Math.Max(1e-3, neutral_acc.magnitude);
            var lateral_weight = Math.Max(Common.Clamp((lift_g - 0.3) / 0.3, 0.0, 1.0),
                Common.Clamp((angular_error - 2e-2) / 2e-2, 0.0, 1.0));
            if (lateral_weight < 1.0)
            {
                var vertical_lift_acc = Vector3d.Project(target_lift_acc, neutral_acc);
                target_lift_acc = vertical_lift_acc + (target_lift_acc - vertical_lift_acc) * lateral_weight;
                target_normal_lift_acc = target_lift_acc - Vector3d.Project(target_lift_acc, imodel.surface_v);
            }

            var desired_right_direction = Vector3d.Cross(target_normal_lift_acc, vessel.ReferenceTransform.up).normalized;

            // let's apply roll to maintain desired_right_direction
            var right_vector = imodel.virtualRotation * Vector3.right;
            var new_roll_error = Math.Sign(Vector3d.Dot(right_vector, target_normal_lift_acc)) *
                                 Math.Acos(Math.Min(Math.Max(Vector3d.Dot(desired_right_direction, right_vector), -1.0), 1.0));
            // rolling to pitch up is not always as efficient as pitching down
            var spine_to_zenith = Vector3d.Dot(desired_right_direction, Vector3d.Cross(imodel.surface_v, imodel.gravity_acc));
            if (target_normal_lift_acc.magnitude < max_neg_g * 9.81 || !allow_spine_down || vessel.heightFromTerrain < min_rollover_alt)
                if (Math.Abs(new_roll_error) > 90.0 * dgr2rad && spine_to_zenith < 0.0)
                    new_roll_error = new_roll_error - 180.0 * dgr2rad * Math.Sign(new_roll_error);
            // filter it
            if ((Math.Abs(new_roll_error) < roll_error_filter_margin * dgr2rad) && (Math.Abs(roll_error) < roll_error_filter_margin * dgr2rad))
                roll_error = Common.simple_filter(new_roll_error, roll_error, roll_error_filter_k);
            else
                roll_error = new_roll_error;
            // generate desired roll angular_v
            roll_c.user_controlled = false;
            roll_c.ApplyControl(state, get_desired_roll_v(roll_error));

            // now let's apply pitch and yaw AoA controls

            // pitch AoA

            desired_pitch_lift = 0.0;
            if (Math.Abs(roll_error) < 30.0 * dgr2rad || Math.Abs(roll_error - 180.0) < 30.0 * dgr2rad)
                desired_pitch_lift = Vector3.Dot(imodel.pitch_tangent, target_normal_lift_acc);
            else
                desired_pitch_lift = Vector3.Dot(imodel.pitch_tangent, neutral_acc);
            desired_pitch_acc = desired_pitch_lift + imodel.pitch_gravity_acc + imodel.pitch_noninert_acc;
            desired_pitch_v = desired_pitch_acc / imodel.surface_v_magnitude;
            // let's find equilibrium AoA for desired lift
            held_pitch_rate = filter_held_rate(held_pitch_rate, desired_pitch_v);
            desired_aoa = get_desired_aoa(imodel.pitch_rot_model_gen, desired_pitch_v, 0.0, held_pitch_rate);
            // The balance with the moment equation can go near singular while the learned moment terms move,
            // and then asks for tenths of a radian out of nowhere. Lift alone, with the surfaces at their trim,
            // says roughly where the AoA has to be, so a solution far from that is not trusted. The trim is the
            // surface position averaged over fallback_trim_time: with the instantaneous one the target AoA
            // followed the elevator and the elevator followed the target, an oscillation of its own.
            update_fallback_trim();
            var lift_only_aoa = get_lift_only_aoa(imodel.pitch_rot_model_gen, desired_pitch_v, 0.0,
                fallback_surface_trim, fallback_gimbal_trim);
            if (!float.IsNaN(lift_only_aoa) && !float.IsInfinity(lift_only_aoa) &&
                (float.IsNaN(desired_aoa) || float.IsInfinity(desired_aoa) ||
                 Math.Abs(desired_aoa - lift_only_aoa) > max_aoa_disagreement))
                desired_aoa = lift_only_aoa;
            if (float.IsNaN(desired_aoa) || float.IsInfinity(desired_aoa))
                desired_aoa = 0.0f;
            aoa_c.user_controlled = false;
            aoa_c.ApplyControl(state, desired_aoa, 0.0f);

            // yaw sideslip

            //if (Math.Abs(roll_angle) > 3.0 * dgr2rad)
            //{
            //    desired_yaw_lift = 0.0;
            //    desired_sideslip = 0.0f;
            //}
            //else
            //{
            desired_yaw_lift = 0.0; // Vector3.Dot(imodel.yaw_tangent, normal_lift_acc);
            desired_yaw_acc = desired_yaw_lift + imodel.yaw_gravity_acc + imodel.yaw_noninert_acc;
            desired_yaw_v = desired_yaw_acc / imodel.surface_v_magnitude;
            // let's find equilibrium sideslip for desired lift
            //if (Math.Abs(desired_yaw_lift) < 0.01f)
            held_yaw_rate = filter_held_rate(held_yaw_rate, desired_yaw_v);
            desired_sideslip = (float)Common.simple_filter(get_desired_aoa(imodel.yaw_rot_model_gen, desired_yaw_v, 0.0, held_yaw_rate), desired_sideslip, sideslip_filter_k);
            if (float.IsNaN(desired_sideslip) || float.IsInfinity(desired_sideslip) || Math.Abs(desired_sideslip) < 0.001f)
                desired_sideslip = 0.0f;
            desired_sideslip = 0.0f;
            //}
            //desired_sideslip = 0.0f;
            side_c.user_controlled = false;
            side_c.ApplyControl(state, desired_sideslip, 0.0f);
        }

        //[AutoGuiAttr("target_acc", false, "G3")]
        public Vector3d target_acc;

        //[AutoGuiAttr("current_acc", false, "G3")]
        public Vector3d current_acc;

        //[AutoGuiAttr("desired_yaw_lift", false, "G5")]
        public double desired_yaw_lift;

        //[AutoGuiAttr("desired_yaw_acc", false, "G5")]
        public double desired_yaw_acc;

        //[AutoGuiAttr("desired_yaw_v", false, "G5")]
        public double desired_yaw_v;

        //[AutoGuiAttr("sideslip_filter_k", true, "G4")]
        public double sideslip_filter_k = 10.0;

        // [AutoGuiAttr("desired_pitch_lift", false, "G5")]
        public double desired_pitch_lift;

        // [AutoGuiAttr("desired_pitch_acc", false, "G5")]
        public double desired_pitch_acc;

        // [AutoGuiAttr("desired_pitch_v", false, "G5")]
        public double desired_pitch_v;

        /// <summary>
        /// Allow rotating belly-up to prevent too large negative g-forces
        /// </summary>
        // [AutoGuiAttr("allow_spine_down", true)]
        public bool allow_spine_down = true;

        public Vector3d non_stall_turn_acceleration(Vector3d desired_turn_acc_dir, double max_lift_acc)
        {
            var max_turn_acc = Vector3d.zero;
            var rest_lift = imodel.pitch_tangent * (-imodel.pitch_gravity_acc - imodel.pitch_noninert_acc);
            var b = 2.0 * Vector3d.Dot(rest_lift, desired_turn_acc_dir);
            var a = desired_turn_acc_dir.sqrMagnitude;
            var c = rest_lift.sqrMagnitude - max_lift_acc * max_lift_acc;
            if (Math.Abs(a) > 1e-10)
            {
                var discriminant = b * b - 4.0 * a * c;
                if (discriminant < 0.0)
                {
                    // we'll stall anyways, so let's just do a max-force turn
                    max_turn_acc = max_lift_acc * desired_turn_acc_dir;
                }
                else
                {
                    var disc_root = Math.Sqrt(discriminant);
                    var k = (-b + disc_root) / 2.0 / a;
                    max_turn_acc = desired_turn_acc_dir * k;
                }
            }
            else
            {
                if (Math.Abs(b) > 1e-10)
                {
                    var k = -c / b;
                    max_turn_acc = desired_turn_acc_dir * k;
                }
                else
                    max_turn_acc = max_lift_acc * desired_turn_acc_dir;
            }
            return max_turn_acc;
        }

        # region Roll

        private Matrix roll_state_m = new Matrix(3, 1);
        private Matrix roll_input_m = new Matrix(3, 1);

        /// <summary>
        /// Max roll angular acceleration
        /// </summary>
        // [AutoGuiAttr("roll_acc_factor", false, "G5")]
        public double roll_acc_factor = 0.0;

        // [AutoGuiAttr("roll_acc_filter", true, "G5")]
        public double roll_acc_filter = 4.0;

        // [AutoGuiAttr("roll_cubic_K", true, "G5")]
        public double roll_cubic_K = 0.3;

        // [AutoGuiAttr("roll_cubic_relax_frame", true, "G5")]
        public double roll_relax_frame = 10.0;

        // [AutoGuiAttr("roll_relax_Kp", true, "G5")]
        public double roll_relax_Kp = 0.1;

        // [AutoGuiAttr("roll_error_filter_margin", true, "G5")]
        public double roll_error_filter_margin = 3.0;

        // [AutoGuiAttr("roll_error_filter_k", true, "G5")]
        public double roll_error_filter_k = 0.5;

        /// <summary>
        /// Max roll angular velocity
        /// </summary>
        // [AutoGuiAttr("max_roll_v", false, "G5")]
        public double max_roll_v;

        // [AutoGuiAttr("roll_error", false, "G5")]
        public double roll_error = 0.0;

        // [AutoGuiAttr("roll_cubic", false)]
        public bool cubic;

        // [AutoGuiAttr("roll_snap_boundary", true, "G5")]
        public double snapping_boundary = 3.0;

        /// <summary>
        /// Calculate input for roll velocity controller
        /// </summary>
        /// <param name="angle_error">Roll error</param>
        /// <returns>desired roll angular velocity</returns>
        public float get_desired_roll_v(double angle_error)
        {
            roll_state_m[0, 0] = 0.0;
            roll_state_m[1, 0] = 1.0;
            roll_state_m[2, 0] = 1.0;
            roll_input_m[0, 0] = 1.0;
            var roll_max_acc = imodel.roll_rot_model_gen.eval_row(0, roll_state_m, roll_input_m) * (precision_mode ? 0.4 : 1.0) * strength;
            roll_acc_factor = Common.simple_filter(Math.Abs(roll_max_acc), roll_acc_factor, roll_acc_filter);

            // calculate anti-overshooting perameters
            if (angle_error >= 0.0)
                max_roll_v = Math.Min(roll_c.max_input_v, roll_c.max_v_construction);
            else
                max_roll_v = Math.Abs(Math.Max(roll_c.min_input_v, -roll_c.max_v_construction));
            max_roll_v *= strength;
            var stop_time = Math.Max(max_roll_v / Math.Max(1e-3, roll_acc_factor) + 1.0f / imodel.csurf_response_rate, 0.1);

            // we'll use cubic descend to desired bank angle
            var cubic_k = roll_cubic_K * Math.Min(roll_acc_factor / 6.0, max_roll_v / 3.0 / stop_time / stop_time);
            var stop_angle_frame = Math.Sqrt(max_roll_v / 3.0 / cubic_k);

            if (angle_error > stop_angle_frame)
            {
                cubic = false;
                return (float)max_roll_v * Math.Sign(angle_error);
            }
            else
            {
                // we're in cubic section
                var snap_bound = snapping_boundary * dgr2rad;
                if (Math.Abs(angle_error) < snap_bound)
                {
                    // error is too small, let's use snapping code
                    var new_dyn_max_v = Math.Sqrt(snap_bound * roll_acc_factor);
                    if (!double.IsNaN(new_dyn_max_v))
                    {
                        new_dyn_max_v = Common.Clamp(new_dyn_max_v, max_roll_v);
                        return (float)(roll_c.snapping_Kp * angle_error / snap_bound * new_dyn_max_v);
                    }
                    else
                        return (float)(roll_c.snapping_Kp * angle_error / snap_bound * max_roll_v * 0.05);
                }
                else
                {
                    cubic = true;
                    var t = Math.Pow(Math.Abs(angle_error / cubic_k), 1.0 / 3.0);
                    if (t < roll_relax_frame * Time.fixedDeltaTime)
                    {
                        // relaxation mode
                        return (float)(roll_relax_Kp * angle_error / Time.fixedDeltaTime);
                    }
                    else
                    {
                        var t_next = Math.Max(0.0, t - Time.fixedDeltaTime);
                        return (float)(Math.Sign(angle_error) * cubic_k * (Math.Pow(t, 3) - Math.Pow(t_next, 3)) / Time.fixedDeltaTime);
                    }
                }
            }
        }

        #endregion

        #region AoAs

        private Matrix aoa_A = new Matrix(2, 2);
        private Matrix aoa_B = new Matrix(2, 1);
        private Matrix eq_x;

        // The low-passed turn rate for the damping term in get_desired_aoa. NaN starts it from the current rate.
        private double filter_held_rate(double held, double rate)
        {
            if (double.IsNaN(held) || held_rate_filter_time <= 0.0)
                return rate;
            return held + (rate - held) * (1.0 - Math.Exp(-Time.fixedDeltaTime / held_rate_filter_time));
        }

        public float get_desired_aoa(LinearSystemModel rotation_model, double desired_v, double desired_aoa_deriv,
            double held_v)
        {
            aoa_A[0, 0] = rotation_model.A[0, 0];
            aoa_A[0, 1] = rotation_model.A[0, 2] + rotation_model.A[0, 3] + rotation_model.B[0, 0];
            aoa_A[1, 0] = rotation_model.A[1, 0];
            aoa_A[1, 1] = rotation_model.A[1, 2] + rotation_model.A[1, 3] + rotation_model.B[1, 0];
            aoa_B[0, 0] = desired_aoa_deriv - desired_v - rotation_model.C[0, 0];
            // Turning at desired_v takes torque against PhysX damping, which takes (1 - f) of the rate every step:
            // w = f * (w + torque_acc * dt) holds w only with torque_acc = w * (1 - f) / (f * dt). The moment row is
            // in torque acceleration, so that is what it has to balance to. Following the planet's curve at Mach 4
            // near sea level that is about 1 rad/s^2, and the surface deflection that supplies it adds lift the
            // equilibrium otherwise doesn't expect; the craft held altitude only because the doubled curvature term
            // happened to cancel it at 500 m. The rate is the low-passed one, held_v: the gain (1 - f) / (f * dt) is
            // about 400 near the speed limit, and with the raw rate it carried every vertical speed correction into
            // the AoA and set pitch oscillating there. The steady turn is all that needs the torque.
            var f = Math.Max(FlightModel.MinAngularDampingFactor, imodel.AngularDampingFactor);
            aoa_B[1, 0] = held_v * (1.0 - f) / (f * Time.fixedDeltaTime) - rotation_model.C[1, 0];
            aoa_A.old_lu = true;
            try
            {
                eq_x = aoa_A.SolveWith(aoa_B);
                var desired_aoa = eq_x[0, 0];
                return (float)desired_aoa;
            }
            catch (MSingularException) { return float.NaN; }
        }

        /// <summary>
        /// Largest gap between the full equilibrium AoA and the lift-only one before the lift-only one is used,
        /// radians.
        /// </summary>
        public float max_aoa_disagreement = 0.02f;
        /// <summary>
        /// Time constant of the surface and gimbal trim the lift-only AoA uses, s.
        /// </summary>
        public float fallback_trim_time = 1.0f;

        private float fallback_surface_trim, fallback_gimbal_trim;
        private double fallback_trim_time_stamp = double.NaN;

        private void update_fallback_trim()
        {
            var surface = imodel.ControlSurfPos(PITCH);
            var gimbal = imodel.GimbalPos(PITCH);
            var now = Time.fixedTimeAsDouble;
            if (!(now - fallback_trim_time_stamp <= 2.0 * Time.fixedDeltaTime))
            {
                fallback_surface_trim = surface;
                fallback_gimbal_trim = gimbal;
            }
            else
            {
                var k = 1.0f - Mathf.Exp(-Time.fixedDeltaTime / fallback_trim_time);
                fallback_surface_trim += (surface - fallback_surface_trim) * k;
                fallback_gimbal_trim += (gimbal - fallback_gimbal_trim) * k;
            }
            fallback_trim_time_stamp = now;
        }

        // AoA the lift equation alone asks for, with the surfaces and gimbals at the given positions
        private float get_lift_only_aoa(LinearSystemModel rotation_model, double desired_v, double desired_aoa_deriv,
            float surface, float gimbal)
        {
            var lift_slope = rotation_model.A[0, 0];
            if (Math.Abs(lift_slope) < 1e-9)
                return float.NaN;
            var rhs = desired_aoa_deriv - desired_v - rotation_model.C[0, 0] -
                (rotation_model.A[0, 2] + rotation_model.B[0, 0]) * surface -
                rotation_model.A[0, 3] * gimbal;
            return (float)(rhs / lift_slope);
        }

        // [AutoGuiAttr("desired_aoa", false, "G5")]
        public float desired_aoa;

        // [AutoGuiAttr("desired_sideslip", false, "G5")]
        public float desired_sideslip;

        public float max_lift_acceleration(int axis)
        {
            switch (axis)
            {
                case PITCH:
                    return (float)(pitch_c.res_equilibr_v_upper * imodel.surface_v_magnitude - imodel.prev_pitch_gravity_acc - imodel.prev_pitch_noninert_acc);
                case YAW:
                    return (float)(yaw_c.res_equilibr_v_upper * imodel.surface_v_magnitude - imodel.prev_yaw_gravity_acc - imodel.prev_yaw_noninert_acc);
                default:
                    return 0.0f;
            }
        }

        #endregion

    }
}
