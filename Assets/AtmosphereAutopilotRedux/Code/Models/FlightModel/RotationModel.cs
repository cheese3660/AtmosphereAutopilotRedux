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
using KSP.Modules;
using KSP.Sim.impl;
using Redux.Ecs.Structural;
using UnityEngine;

using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Modules;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Models
{
    public sealed partial class FlightModel : AutopilotModule
    {
        private Vector3 angular_vel = Vector3.zero;
        private Vector3 angular_vel_prev = Vector3.zero;

        public Vector3 MOI;

        public Vector3 AM;

        public Vector3 CoM;

        public Vector3d surface_v;

        public double surface_v_magnitude;

        public float sum_mass = 0.0f;

        private const int FullMomentFreq = 40;      // with standard 0.025 sec fixedDeltaTime it gives freq around 1 Hz
        private int moments_cycle_counter = 0;

        public Vector3 reaction_torque = Vector3.zero;

        private int prev_part_count = 0;

        private void update_moments()
        {
            var part_count = vessel.View.PartOwner.PartsCount;
            if (part_count != prev_part_count)
                moments_cycle_counter = 0;
            prev_part_count = part_count;
            if (moments_cycle_counter == 0)
            {
                check_csurfaces();
                gather_torque_surfaces();
                get_moments();
                reaction_torque = get_sas_authority();
                get_rcs_characteristics();
                get_engines();
            }
            else
                get_moments();
            moments_cycle_counter = (moments_cycle_counter + 1) % FullMomentFreq;
        }

        public bool HasControlSurfaces { get; private set; }

        /// <summary>
        /// PhysX angular damping on the vessel, 1/s. KSP2 sets it every step to the parts' angularDrag times the
        /// dynamic pressure in atmospheres times AngularDragMultiplier, and PhysX scales the angular velocity by
        /// 1 - damping * dt after applying the step's torques. Near Mach 4 at sea level that removes most of any
        /// rotation within one step. With separate part rigidbodies, the moment-of-inertia-weighted mean.
        /// </summary>
        public float angular_damping;

        /// <summary>
        /// Share of the angular velocity, torque change included, that survives this step's PhysX damping.
        /// </summary>
        // Mathf.Max passes a NaN through, and a damping not read yet would turn every rate difference to NaN
        public float AngularDampingFactor => float.IsNaN(angular_damping) || float.IsInfinity(angular_damping)
            ? 1.0f
            : Mathf.Max(0.0f, 1.0f - angular_damping * Time.fixedDeltaTime);

        /// <summary>
        /// Lowest damping factor the torque acceleration divides by. Below the speed limit's damping, which keeps
        /// the factor above 0.05, so the division stays exact where the vessel flies.
        /// </summary>
        public const float MinAngularDampingFactor = 0.02f;

        /// <summary>
        /// Angular acceleration from torques over the last step, before PhysX damping took its share:
        /// w_new = f * (w_old + torque_acc * dt). Aerodynamic and engine torques answer to this, not to the net
        /// acceleration, which near Mach 4 at sea level is mostly the damping.
        /// </summary>
        public double TorqueAcc(int axis)
        {
            var vel = angular_v_buf[axis];
            if (vel.Size < 2)
                return angular_acc_buf[axis].Size > 0 ? angular_acc_buf[axis].GetLast() : 0.0;
            var f = Mathf.Max(MinAngularDampingFactor, AngularDampingFactor);
            return (vel.GetLast() / f - vel.GetFromTail(1)) / Time.fixedDeltaTime;
        }

        /// <summary>
        /// Normalized control surface actuation speed, in full deflections per second. KSP2 surfaces move at
        /// ActuatorSpeedNormalScale degrees per second towards AuthorityLimiter * input degrees, so the normalized
        /// speed is their ratio. The stock defaults (30 deg/s over 15 deg) give the 2.0 that KSP1 Atmosphere Autopilot
        /// forced on every surface.
        /// </summary>
        public float csurf_spd = 2.0f;

        /// <summary>
        /// Control surfaces ease toward their target with Lerp(deflection, target, ActuatorSpeedNormalScale * dt)
        /// instead of moving at a fixed rate. Most stock KSP2 surfaces work this way. It is the first-order lag that
        /// KSP1 Atmosphere Autopilot modelled for FAR, so those model branches are used for it.
        /// </summary>
        public bool csurf_exponential = false;

        /// <summary>
        /// Physics steps between a control surface deflection and the aerodynamic force it makes. KSP2 computes aero
        /// forces before the part modules move the surfaces each tick, so a deflection set this tick acts one step
        /// later than in KSP1. Measured on a stock jet in flight tests; the regression and controllers account for it.
        /// </summary>
        public const int aero_delay_steps = 1;

        /// <summary>
        /// Fraction of the remaining distance an exponential surface covers each physics step.
        /// </summary>
        public float csurf_lag_k = 1.0f;

        /// <summary>
        /// Continuous rate of the exponential lag, 1/s: d(csurf)/dt = rate * (input - csurf).
        /// </summary>
        public float csurf_lag_rate => csurf_lag_k / Time.fixedDeltaTime;

        /// <summary>
        /// How fast the surfaces respond, 1/s, for whichever actuator model is in use.
        /// </summary>
        public float csurf_response_rate => csurf_exponential ? csurf_lag_rate : csurf_spd;

        private void check_csurfaces()
        {
            HasControlSurfaces = false;
            var min_spd = float.PositiveInfinity;
            var min_lag_k = 1.0f;
            var exponential_count = 0;
            var linear_count = 0;
            foreach (var part in vessel.View.parts)
            {
                if (part == null || part.Model == null)
                    continue;
                if (!part.Model.TryGetModuleData<PartComponentModule_ControlSurface, Data_ControlSurface>(out var cs))
                    continue;
                // These surfaces follow the raw pilot input and ignore the autopilot output
                if (cs.IgnoreSASInputs)
                    continue;
                HasControlSurfaces = true;
                if (cs.UseExponentialSpeed)
                {
                    // Mathf.Lerp clamps its factor to [0, 1]
                    exponential_count++;
                    var lag_k = Mathf.Clamp01(cs.ActuatorSpeedNormalScale * Time.fixedDeltaTime);
                    if (lag_k > 0.0f && lag_k < min_lag_k)
                        min_lag_k = lag_k;
                }
                else
                {
                    linear_count++;
                    var limiter = cs.AuthorityLimiter.GetValue();
                    if (float.IsNaN(limiter) || Math.Abs(limiter) < 1e-3f)
                        limiter = cs.CtrlSurfaceRange;
                    var spd = cs.ActuatorSpeedNormalScale / Math.Max(1e-3f, Math.Abs(limiter));
                    if (spd < min_spd)
                        min_spd = spd;
                }
            }
            // One actuator model per vessel; the slowest surface of the more common kind sets its speed
            csurf_exponential = exponential_count > 0 && exponential_count >= linear_count;
            csurf_lag_k = min_lag_k;
            csurf_spd = linear_count > 0 ? min_spd : 2.0f;
        }

        private Vector3 get_sas_authority()
        {
            var res = Vector3.zero;
            foreach (var part in vessel.View.parts)
            {
                if (part == null || part.Model == null)
                    continue;
                if (!part.Model.TryGetModuleData<PartComponentModule_ReactionWheel, Data_ReactionWheel>(out var rw))
                    continue;
                // ManualOnly wheels follow the raw pilot input and SASOnly wheels need stock SAS on,
                // so neither answers to the autopilot output
                if (rw.ToggleTorque.GetValue() && rw.WheelState == Data_ReactionWheel.ReactionWheelState.Active &&
                    rw.WheelActuatorMode.GetValue() == Data_ReactionWheel.ActuatorModes.All)
                {
                    var authority = rw.WheelAuthority.GetValue();
                    res.x += rw.PitchTorque * authority;
                    res.y += rw.RollTorque * authority;
                    res.z += rw.YawTorque * authority;
                }
            }
            return res;
        }

        // Rotations of the currently controlling part of the vessel
        public Quaternion world_to_cntrl_part;
        public Quaternion cntrl_part_to_world;

        private readonly HashSet<Rigidbody> seen_bodies = new HashSet<Rigidbody>();

        private void get_moments()
        {
            cntrl_part_to_world = vessel.ReferenceTransform.rotation;
            world_to_cntrl_part = Quaternion.Inverse(cntrl_part_to_world);            // from world to control part rotation
            MOI = Vector3.zero;
            AM = Vector3.zero;
            sum_mass = 0.0f;

            var owner = vessel.View.PartOwner;
            var shared_body = StructuralPhysicsService.IsAnalytical(owner) ?
                owner.ViewObject?.Rigidbody?.activeRigidBody : null;

            if (shared_body != null)
            {
                // The whole vessel flies as one rigidbody with an aggregate inertia tensor
                CoM = shared_body.worldCenterOfMass;
                sum_mass = shared_body.mass;
                var principal_to_cntrl = world_to_cntrl_part * shared_body.rotation * shared_body.inertiaTensorRotation;
                MOI = get_rotated_moi(shared_body.inertiaTensor, principal_to_cntrl);
                angular_vel = -(world_to_cntrl_part * shared_body.angularVelocity);     // minus because left-handed Unity
                angular_damping = shared_body.angularDamping;
            }
            else
            {
                // First pass: center of mass and its velocity, from every part rigidbody
                seen_bodies.Clear();
                var com_sum = Vector3.zero;
                var impulse_sum = Vector3.zero;
                foreach (var part in owner)
                {
                    var rb = part != null ? part.Rigidbody?.activeRigidBody : null;
                    if (rb == null || !seen_bodies.Add(rb))
                        continue;
                    sum_mass += rb.mass;
                    com_sum += rb.mass * rb.worldCenterOfMass;
                    impulse_sum += rb.mass * rb.linearVelocity;
                }
                if (sum_mass <= 0.0f)
                    return;
                CoM = com_sum / sum_mass;
                var com_velocity = impulse_sum / sum_mass;

                // Second pass: moment of inertia and angular momentum around the center of mass
                var damping_sum = 0.0f;
                var damping_weight = 0.0f;
                foreach (var rb in seen_bodies)
                {
                    var mass = rb.mass;
                    var pv = world_to_cntrl_part * (rb.worldCenterOfMass - CoM);
                    var impulse = mass * (world_to_cntrl_part * (rb.linearVelocity - com_velocity));
                    // from part.rb principal frame to control from part rotation
                    var principal_to_cntrl = world_to_cntrl_part * rb.rotation * rb.inertiaTensorRotation;
                    // MOI of part as offsetted material point
                    var moi = mass * new Vector3(pv.y * pv.y + pv.z * pv.z, pv.x * pv.x + pv.z * pv.z, pv.x * pv.x + pv.y * pv.y);
                    // MOI of part as rigid body
                    var rotated_moi = get_rotated_moi(rb.inertiaTensor, principal_to_cntrl);
                    moi += rotated_moi;
                    // angular moment of part as offsetted material point
                    var am = Vector3.Cross(pv, impulse);
                    // angular moment of part as rotating rigid body
                    am += Vector3.Scale(rotated_moi, world_to_cntrl_part * rb.angularVelocity);

                    MOI += moi;
                    AM -= am;                   // minus because left-handed Unity
                    var weight = moi.x + moi.y + moi.z;
                    damping_sum += rb.angularDamping * weight;
                    damping_weight += weight;
                }
                angular_vel = Common.DivideVector(AM, MOI);
                angular_damping = damping_weight > 0.0f ? damping_sum / damping_weight : 0.0f;
            }

            surface_v = vessel.srf_velocity;
            surface_v_magnitude = surface_v.magnitude;
        }

        private static Matrix rot_matrix = new Matrix(3, 3);
        private static Matrix rot_matrix_t = new Matrix(3, 3);
        private static Matrix tmp_mat = new Matrix(3, 3);
        private static Matrix new_inert = new Matrix(3, 3);

        public static Vector3 get_rotated_moi(Vector3 inertia_tensor, Quaternion rotation)
        {
            Common.RotationMatrix(rotation, rot_matrix);
            Matrix.TransposeUnsafe(rot_matrix, rot_matrix_t);
            optimized_inert_mult(rot_matrix, inertia_tensor, tmp_mat);
            Matrix.MultiplyUnsafe(tmp_mat, rot_matrix_t, new_inert);
            return new Vector3((float)new_inert[0, 0], (float)new_inert[1, 1], (float)new_inert[2, 2]);
        }

        public static void optimized_inert_mult(Matrix rot_matrix, Vector3 inert_tensor, Matrix res)
        {
            for (var i = 0; i < 3; i++)
                for (var j = 0; j < 3; j++)
                    res[i, j] = rot_matrix[i, j] * inert_tensor[j];
        }

        private void update_velocity_acc()
        {
            for (var i = 0; i < 3; i++)
            {
                angular_v_buf[i].Put(angular_vel[i]);           // update angular velocity
                if (angular_v_buf[i].Size >= 2 && sequential_dt)
                    angular_acc_buf[i].Put(
                        Common.derivative1_short(
                            angular_v_buf[i].GetFromTail(1),
                            angular_v_buf[i].GetLast(),
                            Time.fixedDeltaTime));
            }
        }

        public Vector3 up_srf_v;        // velocity, projected to vessel up direction
        public Vector3 fwd_srf_v;       // velocity, projected to vessel forward direction
        public Vector3 right_srf_v;     // velocity, projected to vessel right direction

        public float aoa_virtual_gain = 0.95f;

        /// <summary>
        /// Virtual rotation of the vessel, filtered from interpart oscillations
        /// </summary>
        public Quaternion virtualRotation = Quaternion.identity;

        private void update_aoa()
        {
            if (!sequential_dt)
                virtualRotation = vessel.ReferenceTransform.rotation;
            else
            {
                var avg_angvel = -0.5f * (angular_vel_prev + angular_vel);
                var ang_delta = rad2dgr * (Time.fixedDeltaTime * avg_angvel.magnitude);
                var delta = Quaternion.AngleAxis(ang_delta, cntrl_part_to_world * avg_angvel);
                virtualRotation = Quaternion.Lerp(vessel.ReferenceTransform.rotation,
                    delta * virtualRotation, aoa_virtual_gain);
                virtualRotation.Normalize();
            }

            angular_vel_prev = angular_vel;

            var upv = virtualRotation * Vector3.up;
            var fwd = virtualRotation * Vector3.forward;
            var right = virtualRotation * Vector3.right;

            up_srf_v = Vector3.Project(surface_v, upv);
            fwd_srf_v = Vector3.Project(surface_v, fwd);
            right_srf_v = Vector3.Project(surface_v, right);

            var projected_vel = up_srf_v + fwd_srf_v;
            if (projected_vel.sqrMagnitude > 1.0f)
            {
                var aoa_p = (float)Math.Asin(Common.Clampf(Vector3.Dot(fwd, projected_vel.normalized), 1.0f));
                if (Vector3.Dot(projected_vel, upv) < 0.0)
                    aoa_p = (float)Math.PI - aoa_p;
                aoa_buf[PITCH].Put(aoa_p);
            }
            else
                aoa_buf[PITCH].Put(0.0f);

            projected_vel = up_srf_v + right_srf_v;
            if (projected_vel.sqrMagnitude > 1.0f)
            {
                var aoa_y = (float)Math.Asin(Common.Clampf(Vector3.Dot(-right, projected_vel.normalized), 1.0f));
                if (Vector3.Dot(projected_vel, upv) < 0.0)
                    aoa_y = (float)Math.PI - aoa_y;
                aoa_buf[YAW].Put(aoa_y);
            }
            else
                aoa_buf[YAW].Put(0.0f);

            projected_vel = right_srf_v + fwd_srf_v;
            if (projected_vel.sqrMagnitude > 1.0f)
            {
                var aoa_r = (float)Math.Asin(Common.Clampf(Vector3.Dot(fwd, projected_vel.normalized), 1.0f));
                if (Vector3.Dot(projected_vel, right) < 0.0)
                    aoa_r = (float)Math.PI - aoa_r;
                aoa_buf[ROLL].Put(aoa_r);
            }
            else
                aoa_buf[ROLL].Put(0.0f);
        }

        private void update_control(AACtrlState state)
        {
            for (var i = 0; i < 3; i++)
            {
                var raw_input = Common.Clampf(ControlUtils.getControlFromState(state, i), 1.0f);
                input_buf[i].Put(raw_input);
                if (HasControlSurfaces)
                {
                    float csurf_input;
                    if (csurf_buf[i].Size >= 1 && sequential_dt)
                        csurf_input = stock_actuator_blend(csurf_buf[i].GetLast(), raw_input);
                    else
                        csurf_input = raw_input;
                    csurf_buf[i].Put(csurf_input);
                }
                else
                    csurf_buf[i].Put(0.0f);
                if (any_gimbals)
                {
                    if (gimbal_buf[i].Size >= 1 && sequential_dt && !float.IsPositiveInfinity(gimbal_spd_norm))
                        gimbal_buf[i].Put(exponential_blend(gimbal_buf[i].GetLast(), raw_input, gimbal_spd_norm, 0.0f));
                    else
                        gimbal_buf[i].Put(raw_input);
                }
                else
                    gimbal_buf[i].Put(0.0f);
            }
        }

        /// <summary>
        /// Control surface position one physics step after commanding <paramref name="desire" />, for this
        /// vessel's actuator model.
        /// </summary>
        public float stock_actuator_blend(float prev, float desire)
        {
            if (csurf_exponential)
                return prev + (desire - prev) * csurf_lag_k;
            return stock_blend_custom(prev, desire, csurf_spd);
        }

        public static float stock_blend_custom(float prev, float desire, float spd)
        {
            var max_delta = Time.fixedDeltaTime * spd;
            return prev + Common.Clampf(desire - prev, max_delta);
        }

        public static float exponential_blend(float prev, float desire, float spd, float snap_margin)
        {
            var error = desire - prev;
            if (Mathf.Abs(error) >= snap_margin)
            {
                return Mathf.Lerp(prev, desire, Time.fixedDeltaTime * spd);
            }
            else
                return desire;
        }

        public Vector3 rcs_authority_pos;

        public Vector3 rcs_authority_neg;

        // RCS torque is not modelled yet. Upstream only counted it with the RCS action group on, which is rare in
        // atmospheric flight, and the regression absorbs what it adds.
        private void get_rcs_characteristics()
        {
            rcs_authority_pos = Vector3.zero;
            rcs_authority_neg = Vector3.zero;
        }

        public double get_rcs_torque(int axis, float input)
        {
            if (input >= 0.0f)
                return input * rcs_authority_pos[axis];
            else
                return input * rcs_authority_neg[axis];
        }
    }
}
