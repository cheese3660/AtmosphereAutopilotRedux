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

        public double height_relax_time = 6.0;

        public double height_relax_Kp = 0.3;

        public double max_climb_angle = 30.0;

        public double dist_to_dest = 0.0;

        private double filtered_drag = 0.0;

        private Vector3d account_for_vertical_vel(Vector3d desired_direction)
        {
            var res = desired_direction.normalized * vessel.horizontalSrfSpeed + planet2vesNorm * desired_vertsetpoint;
            return res.normalized;
        }

        private Vector3d account_for_height(Vector3d desired_direction)
        {
            var cur_alt = vessel.altitude;
            var height_error = desired_altitude - cur_alt;
            var acc = Vector3d.Dot(imodel.gravity_acc + imodel.noninert_acc, -planet2vesNorm);    // free-fall vertical acceleration
            var height_relax_frame = 0.5 * acc * height_relax_time * height_relax_time;

            var relax_transition_k = 0.0;
            var des_vert_speed = 0.0;
            var relax_vert_speed = 0.0;
            var res = Vector3d.zero;

            var proportional_acc = Vector3d.zero;
            var cur_vert_speed = Vector3d.Dot(imodel.surface_v, planet2vesNorm);
            if (Math.Abs(height_error) < height_relax_frame)
            {
                relax_transition_k = Common.Clamp(2.0 * (height_relax_frame - Math.Abs(height_error)), 0.0, 1.0);
                // we're in relaxation frame
                relax_vert_speed = height_relax_Kp * height_error;
                // exponential descent
                if (cur_vert_speed * height_error > 0.0)
                    proportional_acc = -planet2vesNorm * height_relax_Kp * cur_vert_speed;
            }

            // let's assume parabolic ascent\descend
            var parabolic_acc = Vector3d.zero;
            if (height_error >= 0.0)
            {
                des_vert_speed = Math.Sqrt(acc * height_error);
                if (cur_vert_speed > 0.0)
                    parabolic_acc = -planet2vesNorm * 0.5 * cur_vert_speed * cur_vert_speed / height_error;
            }
            else
            {
                var vert_acc_descent = 2.0 * Math.Min(-5.0, acc - dir_c.strength * strength_mult * dir_c.max_lift_acc * 0.5);
                des_vert_speed = -Math.Sqrt(vert_acc_descent * height_error);
                if (cur_vert_speed < 0.0)
                    parabolic_acc = -planet2vesNorm * 0.5 * cur_vert_speed * cur_vert_speed / height_error;
            }

            // speed control portion for ascend
            var effective_max_climb_angle = max_climb_angle;
            if (thrust_c.spd_control_enabled && (height_error >= 0.0))
            {
                if (pseudo_flc)
                {
                    filtered_drag = Common.simple_filter(thrust_c.drag_estimate, filtered_drag, 5.0);
                    if (thrust_c.estimated_max_thrust > filtered_drag)
                    {
                        var sin = Vector3d.Dot(-planet2vesNorm, imodel.gravity_acc + imodel.noninert_acc) / (thrust_c.estimated_max_thrust - filtered_drag);
                        if (sin < 0.0 || sin >= 1.0)
                            effective_max_climb_angle = Math.Min(Math.Asin(sin), max_climb_angle);
                    }
                    else
                        effective_max_climb_angle = 1.0;

                    var spd_diff = (imodel.surface_v_magnitude - thrust_c.setpoint_mps);
                    if (spd_diff < -flc_margin)
                        effective_max_climb_angle *= 0.0;
                    else if (spd_diff < 0.0)
                        effective_max_climb_angle *= (spd_diff + flc_margin) / flc_margin;
                }
                else
                    effective_max_climb_angle *= Math.Max(0.0, Math.Min(1.0, vessel.srfSpeed / thrust_c.setpoint_mps));
            }

            var max_vert_speed = vessel.horizontalSrfSpeed * Math.Tan(effective_max_climb_angle * dgr2rad);
            var apply_acc = Math.Abs(des_vert_speed) < max_vert_speed;
            des_vert_speed = Common.Clamp(des_vert_speed, max_vert_speed);
            res = desired_direction.normalized * vessel.horizontalSrfSpeed + planet2vesNorm * Common.Lerp(des_vert_speed, relax_vert_speed, relax_transition_k);
            if (apply_acc)
                desired_vert_acc = parabolic_acc * (1.0 - relax_transition_k) + proportional_acc * relax_transition_k;
            return res.normalized;
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
