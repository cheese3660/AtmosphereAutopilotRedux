using System;
using System.Collections.Generic;
using System.Threading;
using KSP.Modules;
using KSP.OAB;
using UnityEngine;

using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Models
{
    public sealed partial class FlightModel
    {
        /// <summary>
        /// Take control surface authority from the game's own lift model instead of learning it. The regression then
        /// fits the remaining terms around it. In steady flight the surface and AoA terms move together, and the
        /// learned authority can drift far from the real one, most of all at high dynamic pressure. Per axis: pitch,
        /// roll, yaw.
        /// </summary>
        public readonly bool[] analytic_authority = { true, true, true };

        /// <summary>
        /// Correction factor on the game's authority before the regression uses it.
        /// </summary>
        public double analytic_authority_scale = 1.0;

        private readonly List<Module_ControlSurface> torque_surfaces = new List<Module_ControlSurface>();

        // Body flaps resolve their deflection from torque and report no potential torque, so with one aboard the
        // game's sum would leave it out
        private bool has_body_flaps;

        // Per axis, in the regression's units: torque per full deflection / dyn_pressure * 1e2. NaN when unavailable.
        // Written on the physics thread and read by the trainers' validators.
        private readonly double[] analytic_authority_value = { double.NaN, double.NaN, double.NaN };

        private void gather_torque_surfaces()
        {
            torque_surfaces.Clear();
            lift_providers.Clear();
            has_body_flaps = false;
            foreach (var part in vessel.View.parts)
            {
                if (part == null)
                    continue;
                foreach (var module in part.Modules.Values)
                {
                    if (module is ILiftProvider lift_provider)
                        lift_providers.Add(lift_provider);
                    if (module is Redux.Modules.Module_BodyFlapControlSurface)
                        has_body_flaps = true;
                    else if (module is Module_ControlSurface cs)
                        torque_surfaces.Add(cs);
                }
            }
        }

        private void update_analytic_authority()
        {
            var available = HasControlSurfaces && !has_body_flaps && dyn_pressure >= 60.0;
            var sum = Vector3.zero;
            if (available)
            {
                foreach (var cs in torque_surfaces)
                {
                    if (cs == null)
                        continue;
                    // pos and neg are the surface's torque at full deflection each way, x pitch, y roll, z yaw in
                    // the control frame. Half their difference is the deflection's share, without the part from
                    // the surface's AoA. Mirrored surfaces have opposite signs, so each one adds its magnitude.
                    cs.GetPotentialTorque(out var pos, out var neg);
                    sum.x += Math.Abs(pos.x - neg.x) * 0.5f;
                    sum.y += Math.Abs(pos.y - neg.y) * 0.5f;
                    sum.z += Math.Abs(pos.z - neg.z) * 0.5f;
                }
            }
            for (var axis = 0; axis < 3; axis++)
            {
                var value = available ? sum[axis] / dyn_pressure * 1e2 : double.NaN;
                if (double.IsInfinity(value) || value <= 0.0)
                    value = double.NaN;
                Volatile.Write(ref analytic_authority_value[axis], value);
            }
        }

        /// <summary>
        /// Surface authority from the game's lift model for pitch, roll or yaw, in the regression's units, before
        /// the correction factor. NaN when unavailable.
        /// </summary>
        internal double DebugAnalyticAuthority(int axis) { return Volatile.Read(ref analytic_authority_value[axis]); }

        /// <summary>
        /// Surface authority for incremental control, rad/s^2 per full deflection, from the game's lift model. NaN
        /// when unavailable, which leaves the axis on the model-based law.
        /// </summary>
        public double IndiSurfaceAuthority(int axis)
        {
            var held = held_analytic_authority(axis);
            if (double.IsNaN(held) || MOI[axis] <= 0.0f)
                return double.NaN;
            return held / 1e2 * dyn_pressure / MOI[axis];
        }

        /// <summary>
        /// Authority of the actuators that follow the input with no lag, reaction wheels and RCS, rad/s^2 per unit
        /// input.
        /// </summary>
        public double IndiDirectAuthority(int axis)
        {
            if (MOI[axis] <= 0.0f)
                return 0.0;
            return (reaction_torque[axis] + get_rcs_authority(axis)) / MOI[axis];
        }

        // The authority a validator should hold the regression to, or NaN to leave it learning
        private double held_analytic_authority(int axis)
        {
            if (!analytic_authority[axis])
                return double.NaN;
            return Volatile.Read(ref analytic_authority_value[axis]) * analytic_authority_scale;
        }

        #region Aero stiffness

        /// <summary>
        /// Airflow tilt used to measure the stiffness terms, radians.
        /// </summary>
        public float stiffness_probe_angle = 0.5f * Mathf.Deg2Rad;

        /// <summary>
        /// Physics steps between stiffness measurements. They change slowly, and each one queries every lifting
        /// surface four times.
        /// </summary>
        public int stiffness_period = 5;

        private readonly List<ILiftProvider> lift_providers = new List<ILiftProvider>();
        private readonly ObjectAssemblyCOPressureQuery lift_query = new ObjectAssemblyCOPressureQuery();
        private int stiffness_cycle;

        // The regression's k1 per axis, in its units (torque per radian / dyn_pressure * 1e2): pitch torque per pitch
        // AoA, roll torque per sideslip, yaw torque per sideslip. NaN when unavailable.
        private readonly double[] analytic_stiffness_value = { double.NaN, double.NaN, double.NaN };

        /// <summary>
        /// Hold the regression's k1 to the game's wing lift, per axis: pitch AoA stiffness, roll per sideslip, yaw
        /// sideslip stiffness. Sideslip barely varies in flight, so the learned roll and yaw terms keep whatever
        /// they picked up at takeoff, which can be a hundred times off or of the wrong sign, and a little sideslip
        /// then kicks roll. Pitch stays learned: AoA does vary, and the wing lift leaves out the fuselage, which on
        /// a test plane gave about a quarter of the stiffness that trimmed flight showed.
        /// </summary>
        public readonly bool[] analytic_stiffness = { false, true, true };

        /// <summary>
        /// The regression's k1 for pitch, roll or yaw from the game's wing lift, without fuselage body lift. NaN
        /// when unavailable.
        /// </summary>
        internal double DebugAnalyticStiffness(int axis) { return Volatile.Read(ref analytic_stiffness_value[axis]); }

        // The k1 a validator should hold the regression to, or NaN to leave it learning
        private double held_analytic_stiffness(int axis)
        {
            if (!analytic_stiffness[axis])
                return double.NaN;
            return Volatile.Read(ref analytic_stiffness_value[axis]);
        }

        // The game's wing-only k1, available to start a term that has not been fitted yet
        private double analytic_stiffness_estimate(int axis) { return Volatile.Read(ref analytic_stiffness_value[axis]); }

        /// <summary>
        /// Hold roll damping to the game's wing lift. Steady flight carries almost no roll rate, so the learned
        /// damping is fitted to noise, and the validator pins a positive fit to zero. At high dynamic pressure the
        /// real damping takes most of a roll rate away within one physics step, and a model without it flips the
        /// roll input every tick. Wings provide nearly all of it, which is the part the game's lift query covers.
        /// </summary>
        public bool analytic_roll_damping = true;

        /// <summary>
        /// Arm, m, at which the probe roll rate tilts the airflow by stiffness_probe_angle.
        /// </summary>
        public float damping_probe_arm = 5.0f;

        // In the regression's units: torque per unit of (roll rate / speed * 1e2), / dyn_pressure * 1e2
        private double analytic_roll_damping_value = double.NaN;

        /// <summary>
        /// Roll damping from the game's wing lift in the regression's units, before the sign check. NaN when
        /// unavailable.
        /// </summary>
        internal double DebugAnalyticRollDamping() { return Volatile.Read(ref analytic_roll_damping_value); }

        // The damping a validator should hold the regression to, or NaN to leave it learning. A positive value would
        // feed rotation, so it is not trusted.
        private double held_analytic_roll_damping()
        {
            if (!analytic_roll_damping)
                return double.NaN;
            var value = Volatile.Read(ref analytic_roll_damping_value);
            return value < 0.0 ? value : double.NaN;
        }

        private void update_analytic_stiffness()
        {
            if (++stiffness_cycle < stiffness_period)
                return;
            stiffness_cycle = 0;
            Vector3 v = surface_v;
            if (lift_providers.Count == 0 || dyn_pressure < 60.0 || v.sqrMagnitude < 1.0f)
            {
                for (var i = 0; i < 3; i++)
                    Volatile.Write(ref analytic_stiffness_value[i], double.NaN);
                Volatile.Write(ref analytic_roll_damping_value, double.NaN);
                return;
            }
            var t = vessel.ReferenceTransform;
            var up = t.up;
            var fwd = t.forward;
            var right = t.right;
            var k = Mathf.Tan(stiffness_probe_angle);
            var speed = v.magnitude;
            var dir = v / speed;

            // Tilting the airflow towards fwd raises pitch AoA and towards -right raises sideslip, as update_aoa
            // measures them. The angles the tilts actually give are measured the same way.
            var v_pitch_up = (dir + k * fwd).normalized * speed;
            var v_pitch_dn = (dir - k * fwd).normalized * speed;
            var v_yaw_up = (dir - k * right).normalized * speed;
            var v_yaw_dn = (dir + k * right).normalized * speed;
            var d_aoa = probe_aoa(v_pitch_up, up, fwd) - probe_aoa(v_pitch_dn, up, fwd);
            var d_slip = probe_aoa(v_yaw_up, up, -right) - probe_aoa(v_yaw_dn, up, -right);

            var d_torque_pitch = probe_torque(v_pitch_up, Vector3.zero) - probe_torque(v_pitch_dn, Vector3.zero);
            var d_torque_yaw = probe_torque(v_yaw_up, Vector3.zero) - probe_torque(v_yaw_dn, Vector3.zero);
            publish_stiffness(PITCH, d_torque_pitch[PITCH] / d_aoa / dyn_pressure * 1e2);
            publish_stiffness(ROLL, d_torque_yaw[ROLL] / d_slip / dyn_pressure * 1e2);
            publish_stiffness(YAW, d_torque_yaw[YAW] / d_slip / dyn_pressure * 1e2);

            // Roll damping: each surface sees the airflow of its own point under a probe roll rate. angular_vel is
            // minus the world rate in the control frame, so a model roll rate p is -p about the world roll axis.
            var p = speed * k / damping_probe_arm;
            var omega_world = -(cntrl_part_to_world * Vector3.up) * p;
            var d_torque_roll = probe_torque(v, omega_world) - probe_torque(v, -omega_world);
            var damping = d_torque_roll[ROLL] / (2.0 * p) * speed / dyn_pressure;
            if (double.IsNaN(damping) || double.IsInfinity(damping))
                damping = double.NaN;
            Volatile.Write(ref analytic_roll_damping_value, damping);
        }

        private void publish_stiffness(int axis, double value)
        {
            if (double.IsNaN(value) || double.IsInfinity(value))
                value = double.NaN;
            Volatile.Write(ref analytic_stiffness_value[axis], value);
        }

        // AoA of an airflow in the plane of up and the given axis, as update_aoa measures it
        private static float probe_aoa(Vector3 v, Vector3 up, Vector3 axis)
        {
            var projected = Vector3.Project(v, up) + Vector3.Project(v, axis);
            return (float)Math.Asin(Common.Clampf(Vector3.Dot(axis, projected.normalized), 1.0f));
        }

        // Wing lift torque about the CoM for an airflow, in the control frame with the model's signs. With a
        // rotation rate, in world space, each surface sees the velocity of its centre of pressure.
        private Vector3 probe_torque(Vector3 velocity, Vector3 omega_world)
        {
            var torque = Vector3.zero;
            foreach (var provider in lift_providers)
            {
                if (provider is UnityEngine.Object unity_object && unity_object == null)
                    continue;
                if (!query_lift(provider, velocity))
                    continue;
                if (omega_world != Vector3.zero)
                {
                    // Unity's point velocity is v + Cross(angularVelocity, r)
                    var point_velocity = velocity + Vector3.Cross(omega_world, lift_query.pos - CoM);
                    if (!query_lift(provider, point_velocity))
                        continue;
                }
                var arm = world_to_cntrl_part * (lift_query.pos - CoM);
                var force = world_to_cntrl_part * (lift_query.dir * lift_query.lift);
                torque += -Vector3.Cross(arm, force);       // minus because Unity's left-handed
            }
            return torque;
        }

        private bool query_lift(ILiftProvider provider, Vector3 velocity)
        {
            lift_query.Reset();
            lift_query.refVector = velocity;
            lift_query.refAltitude = vessel.altitude;
            lift_query.refStaticPressure = vessel.Sim.StaticPressure_kPa;
            lift_query.refAirDensity = vessel.atmDensity;
            provider.OnCenterOfLiftQuery(lift_query);
            return lift_query.lift > 0.0f && !float.IsNaN(lift_query.lift);
        }

        #endregion
    }
}
