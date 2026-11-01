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
using UnityEngine;

using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{
    /// <summary>
    /// Manages cruise flight modes, like heading and altitude holds
    /// </summary>
    public sealed class CruiseController : StateController
    {
        internal CruiseController(AAVessel v)
            : base(v, "Cruise Flight controller")
        { }

        private FlightModel imodel;
        private DirectorController dir_c;
        private ProgradeThrustController thrust_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            dir_c = modules[typeof(DirectorController)] as DirectorController;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        protected override void OnActivate()
        {
            dir_c.Activate();
            thrust_c.Activate();
            imodel.Activate();

            // let's set new circle axis
            if (vessel.srfSpeed > 5.0)
                circle_axis = Vector3d.Cross(vessel.srf_velocity, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            else
                circle_axis = Vector3d.Cross(vessel.ReferenceTransform.up, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
        }

        protected override void OnDeactivate()
        {
            dir_c.Deactivate();
            thrust_c.Deactivate();
            imodel.Deactivate();
        }

        private Vector3d desired_velocity = Vector3d.zero;
        private Vector3d planet2ves = Vector3d.zero;
        private Vector3d planet2vesNorm = Vector3d.zero;
        private Vector3d desired_vert_acc = Vector3d.zero;

        public override void ApplyControl(AACtrlState cntrl)
        {
            if (vessel.LandedOrSplashed())
                return;

            if (thrust_c.spd_control_enabled)
                thrust_c.ApplyControl(cntrl, thrust_c.setpoint_mps);

            desired_velocity = Vector3d.zero;
            planet2ves = vessel.ReferenceTransform.position - vessel.mainBody.position;
            planet2vesNorm = planet2ves.normalized;
            desired_vert_acc = Vector3d.zero;
            // The centripetal acceleration that keeps altitude along the planet's curve is the director's own term.
            // Upstream added it here too, and with the director's on top it was counted twice: 3.4 m/s^2 short of
            // lift at Mach 4, which the vertical integral had to make up.

            switch (current_mode)
            {
                default:
                case CruiseMode.LevelFlight:
                    // simply select velocity from axis
                    desired_velocity = Vector3d.Cross(planet2vesNorm, circle_axis);
                    handle_wide_turn();
                    if (vertical_control)
                    {
                        switch (height_mode)
                        {
                            case HeightMode.Altitude:
                                desired_velocity = account_for_height(desired_velocity);
                                break;
                            case HeightMode.VerticalSpeed:
                                desired_velocity = account_for_vertical_vel(desired_velocity);
                                break;
                        }
                    }
                    break;

                case CruiseMode.CourseHold:
                    if (Math.Abs(vessel.latitude) > 80.0)
                    {
                        // we're too close to poles, let's switch to level flight
                        LevelFlightMode = true;
                        goto case CruiseMode.LevelFlight;
                    }
                    // get direction vector form course
                    var north_projected = Vector3d.ProjectOnPlane(vessel.mainBody.RotationAxis, planet2vesNorm);
                    var rotation = QuaternionD.AngleAxis(desired_course, planet2vesNorm);
                    desired_velocity = rotation * north_projected;
                    handle_wide_turn();
                    if (vertical_control)
                    {
                        switch (height_mode)
                        {
                            case HeightMode.Altitude:
                                desired_velocity = account_for_height(desired_velocity);
                                break;
                            case HeightMode.VerticalSpeed:
                                desired_velocity = account_for_vertical_vel(desired_velocity);
                                break;
                        }
                    }
                    break;

                case CruiseMode.Waypoint:
                    // set new axis
                    var world_target_pos = vessel.mainBody.GetWorldSurfacePosition(desired_latitude, desired_longitude, vessel.altitude);
                    dist_to_dest = Vector3d.Distance(world_target_pos, vessel.ReferenceTransform.position);
                    if (dist_to_dest > 10000.0)
                    {
                        var radius = vessel.mainBody.Radius;
                        dist_to_dest = Math.Acos(1 - (dist_to_dest * dist_to_dest) / (2 * radius * radius)) * radius;
                    }
                    if (dist_to_dest < 200.0)
                    {
                        // we're too close to target, let's switch to level flight
                        LevelFlightMode = true;
                        WaypointReached?.Invoke();
                        goto case CruiseMode.LevelFlight;
                    }
                    // set new axis according to waypoint
                    circle_axis = Vector3d.Cross(world_target_pos - vessel.mainBody.position, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                    goto case CruiseMode.LevelFlight;
            }

            // Any stick input would hand pitch and yaw back to the pilot inside the AoA controllers
            ControlUtils.neutralize_user_input(cntrl, PITCH);
            ControlUtils.neutralize_user_input(cntrl, YAW);

            var old_str = dir_c.strength;
            dir_c.strength *= strength_mult;
            // The director's vertical integral is for steady error; hold it while the altitude profile is moving
            dir_c.hold_vert_integral = desired_vert_acc.magnitude > vert_integral_settled_acc;
            dir_c.ApplyControl(cntrl, desired_velocity, desired_vert_acc);
            dir_c.hold_vert_integral = false;
            dir_c.strength = old_str;
        }

        private void handle_wide_turn()
        {
            var hor_vel = imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm);
            if (Vector3d.Dot(hor_vel.normalized, desired_velocity.normalized) < Math.Cos(0.5))
            {
                // we're turning for more than 45 degrees, let's force the turn to be horizontal
                var right_turn = Vector3d.Cross(planet2vesNorm, imodel.surface_v);
                double sign = Math.Sign(Vector3d.Dot(right_turn, desired_velocity));
                if (sign == 0.0)
                    sign = 1.0;
                desired_velocity = right_turn.normalized * sign * Math.Tan(0.5) + hor_vel.normalized;
            }
        }

        public enum CruiseMode
        {
            LevelFlight,
            CourseHold,
            Waypoint
        }

        public CruiseMode current_mode = CruiseMode.LevelFlight;

        public enum HeightMode
        {
            Altitude,
            VerticalSpeed
        }

        public HeightMode height_mode = HeightMode.Altitude;

        // axis to rotate around in level flight mode
        public Vector3d circle_axis = Vector3d.zero;

        /// <summary>
        /// Raised when the waypoint is reached and the controller falls back to level flight.
        /// </summary>
        public event Action WaypointReached;

        /// <summary>
        /// Course to hold, degrees from north around the local vertical
        /// </summary>
        public double desired_course = 90.0;

        /// <summary>
        /// Waypoint latitude, degrees
        /// </summary>
        public double desired_latitude = 0.0;

        /// <summary>
        /// Waypoint longitude, degrees
        /// </summary>
        public double desired_longitude = 0.0;

        public bool vertical_control = false;

        /// <summary>
        /// Altitude above sea level to hold, meters
        /// </summary>
        public double desired_altitude = 1000.0;

        /// <summary>
        /// Vertical speed to hold, m/s
        /// </summary>
        public double desired_vertsetpoint = 0.0;

        public double strength_mult = 0.75;

        /// <summary>
        /// Vertical speed per metre of height error close to the target altitude, 1/s.
        /// </summary>
        public double height_relax_Kp = 0.3;

        public double max_climb_angle = 30.0;

        /// <summary>
        /// Shallowest climb or dive angle, degrees, while an altitude change is under way, so the energy balance can
        /// slow an altitude change but never stall it.
        /// </summary>
        public double min_climb_angle = 2.0;

        /// <summary>
        /// How fast the allowed climb or dive angle may change, degrees per second.
        /// </summary>
        public double climb_angle_rate = 5.0;

        /// <summary>
        /// Largest commanded vertical acceleration, in g. Keeps pushovers and flares gentle.
        /// </summary>
        public double max_vert_acc_g = 0.5;

        /// <summary>
        /// Vertical acceleration, in g, for winning back a vertical speed that points away from the profile, such as
        /// a climb still under way when altitude hold engages. At max_vert_acc_g a 190 m/s climb at Mach 4 took
        /// almost 4 km to stop.
        /// </summary>
        public double capture_vert_acc_g = 1.5;

        /// <summary>
        /// Vertical speed error, m/s, over which the capture acceleration blends in from max_vert_acc_g, reaching it
        /// at twice this.
        /// </summary>
        public double capture_band = 20.0;

        /// <summary>
        /// Fastest change of the commanded vertical acceleration, g per second. Without it the command flipped from
        /// full dive to full flare in a step, and the elevator kick that took briefly jolted speed hold.
        /// </summary>
        public double vert_jerk_g = 1.0;

        /// <summary>
        /// Time constant, s, the vertical speed command settles onto the profile with once close to it.
        /// </summary>
        public double vert_settle_time = 0.5;

        /// <summary>
        /// Commanded vertical acceleration, m/s^2, under which the altitude profile counts as settled and the
        /// director's vertical integral may run.
        /// </summary>
        public double vert_integral_settled_acc = 0.5;

        /// <summary>
        /// Smoothing of the director's lift estimate that shapes descents, in physics steps.
        /// </summary>
        public double lift_filter_k = 20.0;

        /// <summary>
        /// Fraction of max_vert_acc_g that climb and descent profiles plan their level-off with, leaving the rest as
        /// margin for the vertical speed command to catch up.
        /// </summary>
        public double flare_margin = 0.8;

        public double dist_to_dest = 0.0;

        private double filtered_drag = 0.0;
        private double filtered_lift = 0.0;
        private double allowed_angle = 0.0;             // rate-limited climb or dive angle limit, degrees
        private double commanded_vert_speed = 0.0;      // jerk-limited vertical speed command, m/s
        private double commanded_vert_acc = 0.0;        // its rate of change, m/s^2
        private double last_height_hold_time = double.NaN;

        // Diagnostics for FlightRecorder: the altitude hold's last decision
        internal double debug_height_error, debug_raw_vert_speed, debug_cmd_vert_speed, debug_max_vert_speed,
            debug_max_climb_angle, debug_relax_k;
        internal bool debug_apply_acc;
        internal double DebugFilteredDrag => filtered_drag;
        internal Vector3d DebugDesiredVertAcc => desired_vert_acc;

        private Vector3d account_for_vertical_vel(Vector3d desired_direction)
        {
            var res = desired_direction.normalized * vessel.horizontalSrfSpeed + planet2vesNorm * desired_vertsetpoint;
            thrust_c.commanded_vertical_speed = desired_vertsetpoint;
            return res.normalized;
        }

        private Vector3d account_for_height(Vector3d desired_direction)
        {
            var dt = Time.fixedDeltaTime;
            var cur_alt = vessel.altitude;
            var height_error = desired_altitude - cur_alt;
            var acc = Vector3d.Dot(imodel.gravity_acc + imodel.noninert_acc, -planet2vesNorm);    // free-fall vertical acceleration
            var cur_vert_speed = Vector3d.Dot(imodel.surface_v, planet2vesNorm);

            // (Re)entering altitude hold: start the smoothed state from how the vessel is flying now
            if (!(Time.fixedTimeAsDouble - last_height_hold_time <= 2.0 * dt))
            {
                commanded_vert_speed = cur_vert_speed;
                commanded_vert_acc = 0.0;
                allowed_angle = Math.Abs(Math.Atan2(cur_vert_speed, Math.Max(1.0, vessel.horizontalSrfSpeed))) * rad2dgr;
                filtered_lift = dir_c.max_lift_acc;
                filtered_drag = thrust_c.drag_estimate;
            }
            last_height_hold_time = Time.fixedTimeAsDouble;
            filtered_drag = Common.simple_filter(thrust_c.drag_estimate, filtered_drag, 5.0);
            filtered_lift = Common.simple_filter(dir_c.max_lift_acc, filtered_lift, lift_filter_k);

            // The vertical speed command can change no faster than max_vert_acc, so the profile must not plan a
            // harder level-off than that, or the command lags it and overshoots (into the ground, on descents)
            var max_vert_acc = max_vert_acc_g * acc;
            var flare_acc = flare_margin * max_vert_acc;
            if (height_error < 0.0)
            {
                // The director's lift estimate swings with AoA and dynamic pressure; filtered, so the descent
                // target doesn't jump around with it
                var lift_flare_acc = Math.Max(5.0, dir_c.strength * strength_mult * filtered_lift * 0.5 - acc);
                flare_acc = Math.Min(flare_acc, lift_flare_acc);
            }

            // Square-root profile: levels off at a constant flare_acc far from the target, then exponentially
            // (vs = Kp * h) inside h_lin, where that needs no more than flare_acc. The two meet with the same value
            // and slope, so the command never has a step to chase.
            var abs_error = Math.Abs(height_error);
            var h_lin = flare_acc / (height_relax_Kp * height_relax_Kp);
            var in_linear_tail = abs_error <= h_lin;
            var des_vert_speed = in_linear_tail
                ? height_relax_Kp * height_error
                : Math.Sign(height_error) * Math.Sqrt(2.0 * flare_acc * (abs_error - 0.5 * h_lin));

            // Steepest climb or dive, rate-limited so the cap eases in. With speed hold on, how much of that the
            // engines can pay for is the energy balance's business below.
            allowed_angle += Common.Clamp(max_climb_angle - allowed_angle, climb_angle_rate * dt);
            var effective_max_climb_angle = allowed_angle;

            var max_vert_speed = vessel.horizontalSrfSpeed * Math.Tan(effective_max_climb_angle * dgr2rad);
            var apply_acc = Math.Abs(des_vert_speed) < max_vert_speed;
            debug_raw_vert_speed = des_vert_speed;
            des_vert_speed = Common.Clamp(des_vert_speed, max_vert_speed);
            if (thrust_c.spd_control_enabled)
                des_vert_speed = Common.Clamp(energy_balance(des_vert_speed, cur_vert_speed, acc),
                    max_vert_speed);

            // Ease into the new vertical speed instead of stepping to it; its rate of change is the feed-forward,
            // so the director is told the same acceleration the command implies. The acceleration itself changes
            // at no more than vert_jerk_g, and eases off as the command nears the profile, sqrt(2 * jerk * error),
            // so it arrives with none left over. Speed pointing away from the profile is won back harder.
            var vert_speed_error = des_vert_speed - commanded_vert_speed;
            var acc_limit = max_vert_acc;
            if (vert_speed_error * commanded_vert_speed < 0.0 && capture_band > 0.0)
            {
                var capture_k = Common.Clamp((Math.Abs(vert_speed_error) - capture_band) / capture_band, 0.0, 1.0);
                acc_limit += capture_k * Math.Max(0.0, capture_vert_acc_g * acc - max_vert_acc);
            }
            var jerk = vert_jerk_g * acc;
            var abs_vert_speed_error = Math.Abs(vert_speed_error);
            var wanted_vert_acc = Math.Min(acc_limit, Math.Sqrt(2.0 * jerk * abs_vert_speed_error));
            if (vert_settle_time > 0.0)
                wanted_vert_acc = Math.Min(wanted_vert_acc, abs_vert_speed_error / vert_settle_time);
            wanted_vert_acc *= Math.Sign(vert_speed_error);
            commanded_vert_acc += Common.Clamp(wanted_vert_acc - commanded_vert_acc, jerk * dt);
            commanded_vert_speed += commanded_vert_acc * dt;
            var res = desired_direction.normalized * vessel.horizontalSrfSpeed + planet2vesNorm * commanded_vert_speed;
            desired_vert_acc = planet2vesNorm * commanded_vert_acc;
            debug_height_error = height_error;
            debug_max_climb_angle = effective_max_climb_angle;
            debug_max_vert_speed = max_vert_speed;
            debug_relax_k = in_linear_tail ? 1.0 : 0.0;
            debug_cmd_vert_speed = commanded_vert_speed;
            debug_apply_acc = apply_acc;
            // The throttle pays for the flight path being commanded
            thrust_c.commanded_vertical_speed = commanded_vert_speed;
            return res.normalized;
        }

        #region Energy balance

        // Total energy control, after Lambregts: the throttle answers for total energy (speed hold plus the flight
        // path's share, ProgradeThrustController), pitch for how it is split between height and speed. Here pitch
        // trades speed_weight of the speed error, worth V * dV / g of height, over energy_balance_time. With the
        // throttle pinned at full while short of speed in a climb, or at idle while fast in a descent, it moves to
        // speed priority: the climb or descent shrinks toward min_climb_angle, never reverses. This replaces the
        // speed rules of the old climb and descent angle limits.

        /// <summary>
        /// How pitch shares the energy: 0 holds the flight path and leaves speed to the throttle (airliner path
        /// modes), 1 splits the error evenly, 2 holds speed with pitch (level change). Throttle saturation moves it
        /// to 2. Altitude and vertical speed hold are path modes, as on airliners.
        /// </summary>
        public double speed_weight = 0.0;

        /// <summary>
        /// Time constant, s, pitch trades the speed share of the energy error over.
        /// </summary>
        public double energy_balance_time = 5.0;

        /// <summary>
        /// Time constant, s, of the move to speed priority while the throttle is saturated, and back.
        /// </summary>
        public double speed_priority_time = 1.0;

        private double speed_priority;

        internal double debug_speed_weight, debug_balance_vs;

        private double energy_balance(double profile_vs, double cur_vs, double g)
        {
            var dt = Time.fixedDeltaTime;
            var speed = imodel.surface_v_magnitude;
            var reference = thrust_c.SpeedReference;
            if (double.IsNaN(reference) || speed < 1.0 || g <= 0.0)
            {
                debug_speed_weight = speed_weight;
                debug_balance_vs = 0.0;
                return profile_vs;
            }
            var target_error = thrust_c.desired_v - speed;
            var saturated = (thrust_c.ThrottleAtFull && target_error > 0.0 && profile_vs > 0.0) ||
                (thrust_c.ThrottleAtIdle && target_error < 0.0 && profile_vs < 0.0);
            speed_priority += ((saturated ? 1.0 : 0.0) - speed_priority) *
                (1.0 - Math.Exp(-dt / Math.Max(dt, speed_priority_time)));
            var weight = Common.Clamp(speed_weight, 0.0, 2.0);
            weight += speed_priority * (2.0 - weight);
            var height_share = Math.Min(1.0, 2.0 - weight);
            var speed_share = Math.Min(1.0, weight);
            var balance_vs = -(speed / g) * speed_share * (reference - speed) / Math.Max(dt, energy_balance_time);
            var vs = height_share * profile_vs + (1.0 - height_share) * cur_vs + balance_vs;
            // Speed priority may shrink an altitude change toward min_climb_angle but not turn it around
            if (height_share < 0.99)
            {
                var min_vs = vessel.horizontalSrfSpeed * Math.Tan(min_climb_angle * dgr2rad);
                if (profile_vs > 0.0)
                    vs = Math.Max(Math.Min(vs, profile_vs), Math.Min(min_vs, profile_vs));
                else if (profile_vs < 0.0)
                    vs = Math.Min(Math.Max(vs, profile_vs), Math.Max(-min_vs, profile_vs));
            }
            debug_speed_weight = weight;
            debug_balance_vs = balance_vs;
            return vs;
        }

        #endregion

        /// <summary>
        /// Course of the current surface velocity, in the same convention as <see cref="desired_course" />.
        /// </summary>
        public double current_course()
        {
            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            var north_projected = Vector3d.ProjectOnPlane(vessel.mainBody.RotationAxis, up);
            var hor_vel = vessel.srfSpeed > 1.0 ?
                Vector3d.ProjectOnPlane(vessel.srf_velocity, up) :
                Vector3d.ProjectOnPlane(vessel.ReferenceTransform.up, up);
            var course = Vector3d.SignedAngle(north_projected, hor_vel, up);
            return course < 0.0 ? course + 360.0 : course;
        }

        public bool LevelFlightMode
        {
            get => current_mode == CruiseMode.LevelFlight;
            set
            {
                if (value)
                {
                    if (current_mode != CruiseMode.LevelFlight)
                    {
                        // let's set new circle axis
                        circle_axis = Vector3d.Cross(vessel.srf_velocity, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                    }
                    current_mode = CruiseMode.LevelFlight;
                }
            }
        }

        public bool CourseHoldMode
        {
            get => current_mode == CruiseMode.CourseHold;
            set
            {
                if (value)
                {
                    if (Math.Abs(vessel.latitude) > 80.0)
                        return;
                    current_mode = CruiseMode.CourseHold;
                }
            }
        }

        public bool WaypointMode
        {
            get => current_mode == CruiseMode.Waypoint;
            set
            {
                if (value)
                {
                    if (current_mode != CruiseMode.Waypoint && Active)
                    {
                        circle_axis = Vector3d.Cross(vessel.srf_velocity, vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                        var world_target_pos = vessel.mainBody.GetWorldSurfacePosition(desired_latitude, desired_longitude, vessel.altitude);
                        dist_to_dest = Vector3d.Distance(world_target_pos, vessel.ReferenceTransform.position);
                    }
                    current_mode = CruiseMode.Waypoint;
                }
            }
        }
    }
}
