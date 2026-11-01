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

        // centrifugal acceleration to stay on desired altitude
        private Vector3d level_acc = Vector3d.zero;

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

            // centrifugal acceleration to stay on desired altitude
            level_acc = -planet2vesNorm * (imodel.surface_v - Vector3d.Project(imodel.surface_v, planet2vesNorm)).sqrMagnitude / planet2ves.magnitude;

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
            dir_c.ApplyControl(cntrl, desired_velocity, level_acc + desired_vert_acc);
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

        public bool pseudo_flc = true;

        public double flc_margin = 20.0;

        public double strength_mult = 0.75;

        /// <summary>
        /// Vertical speed per metre of height error close to the target altitude, 1/s.
        /// </summary>
        public double height_relax_Kp = 0.3;

        public double max_climb_angle = 30.0;

        /// <summary>
        /// Shallowest climb or dive angle, degrees, while an altitude change is under way, so speed hold can slow an
        /// altitude change but never stall it.
        /// </summary>
        public double min_climb_angle = 2.0;

        /// <summary>
        /// Fraction of the sustainable climb (or idle dive) angle kept when far off the speed target.
        /// </summary>
        public double climb_angle_floor = 0.25;

        /// <summary>
        /// How fast the allowed climb or dive angle may change, degrees per second.
        /// </summary>
        public double climb_angle_rate = 5.0;

        /// <summary>
        /// Largest commanded vertical acceleration, in g. Keeps pushovers and flares gentle.
        /// </summary>
        public double max_vert_acc_g = 0.5;

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
        private double commanded_vert_speed = 0.0;      // rate-limited vertical speed command, m/s
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

            // How steep to climb or dive. With speed hold on, that depends on how much thrust (climbing) or drag
            // (diving) there is to spare. Rate-limited so a momentary speed dip can't flatten the climb out.
            var target_angle = max_climb_angle;
            if (thrust_c.spd_control_enabled)
            {
                var spd_diff = imodel.surface_v_magnitude - thrust_c.setpoint_mps;
                target_angle = height_error >= 0.0 ? climb_angle_limit(acc, spd_diff) : descent_angle_limit(acc, spd_diff);
            }
            allowed_angle += Common.Clamp(target_angle - allowed_angle, climb_angle_rate * dt);
            var effective_max_climb_angle = allowed_angle;

            var max_vert_speed = vessel.horizontalSrfSpeed * Math.Tan(effective_max_climb_angle * dgr2rad);
            var apply_acc = Math.Abs(des_vert_speed) < max_vert_speed;
            debug_raw_vert_speed = des_vert_speed;
            des_vert_speed = Common.Clamp(des_vert_speed, max_vert_speed);

            // Ease into the new vertical speed instead of stepping to it; its rate of change is the feed-forward,
            // so the director is told the same acceleration the command implies
            var prev_cmd_vert_speed = commanded_vert_speed;
            commanded_vert_speed += Common.Clamp(des_vert_speed - commanded_vert_speed, max_vert_acc * dt);
            var res = desired_direction.normalized * vessel.horizontalSrfSpeed + planet2vesNorm * commanded_vert_speed;
            desired_vert_acc = planet2vesNorm * ((commanded_vert_speed - prev_cmd_vert_speed) / dt);
            debug_height_error = height_error;
            debug_max_climb_angle = effective_max_climb_angle;
            debug_max_vert_speed = max_vert_speed;
            debug_relax_k = in_linear_tail ? 1.0 : 0.0;
            debug_cmd_vert_speed = commanded_vert_speed;
            debug_apply_acc = apply_acc;
            return res.normalized;
        }

        /// <summary>
        /// Steepest climb angle, degrees, for the current speed error.
        /// </summary>
        /// <remarks>
        /// Replaces upstream's pseudo-FLC, which mixed kN with m/s^2, inverted its own range check and compared radians
        /// with degrees, so only its speed rule worked: 20 m/s under the speed target it forced the climb angle to 0,
        /// which made climbs stall into a plateau and crawl back.
        /// </remarks>
        private double climb_angle_limit(double g, double spd_diff)
        {
            if (!pseudo_flc)
                return max_climb_angle * Common.Clamp(vessel.srfSpeed / Math.Max(1.0, thrust_c.setpoint_mps), 0.0, 1.0);
            // Steepest climb the engines can hold at constant speed: thrust / mass + drag = g * sin(angle)
            var excess = thrust_c.estimated_max_thrust / Math.Max(1e-3, imodel.sum_mass) + filtered_drag;
            var sustainable = Math.Asin(Common.Clamp(excess / Math.Max(1e-3, g), 0.0, 1.0)) * rad2dgr;
            double angle;
            if (spd_diff >= 0.0)
                // faster than wanted: climb steeper and trade the extra speed for height
                angle = sustainable + Math.Min(1.0, spd_diff / flc_margin) * (max_climb_angle - sustainable);
            else
                // slower than wanted: climb shallower so the engines can catch up, but keep climbing
                angle = sustainable * Math.Max(climb_angle_floor, 1.0 + spd_diff / flc_margin);
            return Common.Clamp(angle, min_climb_angle, max_climb_angle);
        }

        /// <summary>
        /// Steepest dive angle, degrees, for the current speed error. Upstream had no speed limit on descents, so a low
        /// speed target overspeeded on the way down.
        /// </summary>
        private double descent_angle_limit(double g, double spd_diff)
        {
            // Dive angle that holds speed with the engines idle: -drag = g * sin(angle)
            var idle = Math.Asin(Common.Clamp(-filtered_drag / Math.Max(1e-3, g), 0.0, 1.0)) * rad2dgr;
            double angle;
            if (spd_diff <= 0.0)
                // slower than wanted: dive steeper to pick speed up
                angle = idle + Math.Min(1.0, -spd_diff / flc_margin) * (max_climb_angle - idle);
            else
                // faster than wanted: dive shallower so drag can bleed the extra speed
                angle = idle * Math.Max(climb_angle_floor, 1.0 - spd_diff / flc_margin);
            return Common.Clamp(angle, min_climb_angle, max_climb_angle);
        }

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
