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

using AtmosphereAutopilotRedux.Controllers;
using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{
    /// <summary>
    /// Naive thrust controller for regular plane flight.
    /// </summary>
    public sealed class ProgradeThrustController : SISOController
    {
        private FlightModel imodel;

        internal ProgradeThrustController(AAVessel v)
            : base(v, "Prograde thrust controller")
        {
            pid.IntegralClamp = double.PositiveInfinity;
            pid.AccumulDerivClamp = double.PositiveInfinity;
            pid.KP = 0.4;
            pid.KI = 0.1;
            pid.KD = 0.5;
        }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
        }

        protected override void OnActivate()
        {
            imodel.Activate();
            ResetSpeedHold();
        }

        /// <summary>
        /// Starts the speed hold afresh from how the vessel flies now, as on engaging: after something else held the
        /// throttle, such as TOGA thrust, its integral and speed reference no longer mean anything.
        /// </summary>
        internal void ResetSpeedHold()
        {
            v_integral = 0.0;
            filtered_acc = double.NaN;
            speed_reference = double.NaN;
            speed_reference_acc = 0.0;
            commanded_vertical_speed = double.NaN;
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
        }

        // [AutoGuiAttr("desired_v", false, "G5")]
        public float desired_v = 0.0f;

        // [AutoGuiAttr("current_v", false, "G5")]
        public double current_v = 0.0f;

        // [VesselSerializable("break_margin")]
        // [AutoGuiAttr("break spd margin %", true, "G5")]
        public double break_margin = 10.0f;

        // [GlobalSerializable("use_breaks")]
        // [AutoGuiAttr("Use breaks", true)]
        public bool use_breaks = true;

        // [AutoGuiAttr("prograde_thrust", false, "G5")]
        public double prograde_thrust;

        //[AutoGuiAttr("drag_estimate", false, "G5")]
        // drag acceleration
        public double drag_estimate;

        // [VesselSerializable("Kp_v_factor")]
        // [AutoGuiAttr("Kp_v", true, "G5")]
        public double Kp_v = 0.5;

        // [AutoGuiAttr("acc_filter_k", true, "G5")]
        public double acc_filter_k = 1.0;

        // [AutoGuiAttr("relaxation_acc_error", true, "G5")]
        public double relaxation_acc_error = 0.1;

        /// <summary>
        /// Integral gain on speed error, 1/s^2. Removes the steady offset the proportional term leaves when the drag
        /// estimate lags. Not in upstream.
        /// </summary>
        public double Ki_v = 0.05;

        /// <summary>
        /// The integral only accumulates within this speed error, m/s, so setpoint steps don't wind it up.
        /// </summary>
        public double integral_band = 5.0;

        /// <summary>
        /// Largest acceleration the integral may contribute, m/s^2.
        /// </summary>
        public double integral_limit = 2.0;

        /// <summary>
        /// Time constant, s, the engines' spool is driven to follow the wanted throttle with. Engines that spool slower
        /// than this get their throttle request overdriven. Zero turns it off. Not in upstream.
        /// </summary>
        public double spool_time = 0.4;

        /// <summary>
        /// Time constant, s, of the low-pass on the measured acceleration the speed hold works from. At Mach 4 an
        /// elevator kick moves the measured acceleration by up to 20 m/s^2 for a few steps; the engines can't answer
        /// that anyway, and passed through it slammed the throttle to full and then idle.
        /// </summary>
        public double acc_filter_time = 0.3;

        private double filtered_acc = double.NaN;

        private double v_integral = 0.0;
        private float last_throttle = 0.0f;

        public Vector3d surfspd_dir;

        private bool was_breaking_previously = true;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Prograde surface speed setpoint</param>
        public override float ApplyControl(AACtrlState cntrl, float target_value)
        {
            current_v = imodel.surface_v_magnitude;
            setpoint_target = target_value;
            desired_v = apply_damping_limit(target_value);

            // apply breaks if needed
            if (use_breaks)
            {
                if (vessel.LandedOrSplashed())
                {
                    // we're on ground
                    if (current_v > desired_v)
                    {
                        vessel.SetBrakes(true);
                        was_breaking_previously = true;
                    }
                    else
                    {
                        if (was_breaking_previously)
                            vessel.SetBrakes(false);
                        was_breaking_previously = false;
                    }
                }
                else
                {
                    // we're in flight
                    if (current_v > (1.0 + break_margin / 100.0) * desired_v)
                    {
                        vessel.SetBrakes(true);
                        was_breaking_previously = true;
                    }
                    else
                    {
                        if (was_breaking_previously)
                            vessel.SetBrakes(false);
                        was_breaking_previously = false;
                    }
                }
            }
            else
                was_breaking_previously = true;

            if (use_pid)
            {
                if (pid.KI != 0.0)
                {
                    if (pid.KP != 0.0)
                        pid.IntegralClamp = 1.0 / pid.KP;
                    pid.AccumulatorClamp = 1.0 / pid.KI;
                }
                cntrl.mainThrottle = Mathf.Clamp01((float)pid.Control(current_v, desired_v, Time.fixedDeltaTime));
                filtered_acc = double.NaN;
            }
            else
            {
                surfspd_dir = imodel.prev_surface_v.normalized;
                var thrust = imodel.prev_cntrl2world * imodel.engines_thrust_principal;
                prograde_thrust = Vector3.Dot(thrust, surfspd_dir);

                double current_acc = Vector3.Dot(imodel.sum_acc, surfspd_dir);
                debug_current_acc = current_acc;
                if (double.IsNaN(filtered_acc) || acc_filter_time <= 0.0)
                    filtered_acc = current_acc;
                else
                    filtered_acc += (current_acc - filtered_acc) * (1.0 - Math.Exp(-Time.fixedDeltaTime / acc_filter_time));
                current_acc = filtered_acc;
                drag_estimate = current_acc - prograde_thrust / imodel.sum_mass - Vector3d.Dot(imodel.gravity_acc + imodel.noninert_acc, surfspd_dir);

                // Total energy control: the throttle follows a speed reference the engines can deliver, and the
                // climb or descent the cruise controller commands, so thrust moves with the flight path command
                // instead of after the speed has gone
                update_speed_reference();
                v_error = speed_reference - current_v;
                if (Math.Abs(v_error) > integral_band)
                    v_integral = 0.0;
                else
                {
                    // don't integrate further into a saturated throttle
                    var saturated = (last_throttle >= 0.999f && v_error > 0.0) || (last_throttle <= 0.001f && v_error < 0.0);
                    if (!saturated)
                        v_integral += v_error * Time.fixedDeltaTime;
                    if (Ki_v > 0.0)
                        v_integral = Common.Clamp(v_integral, integral_limit / Ki_v);
                }
                desired_acc = speed_reference_acc + Kp_v * v_error + Ki_v * v_integral;
                // Energy rate the flight path command adds or takes, g * (sin gamma_c - sin gamma), against the
                // gravity share the measured acceleration already carries
                var commanded_vs = commanded_vertical_speed;
                commanded_vertical_speed = double.NaN;      // one step only, so a stale command can't linger
                debug_energy_acc = 0.0;
                if (!double.IsNaN(commanded_vs) && current_v > 1.0)
                {
                    var up = (vessel.ReferenceTransform.position - vessel.mainBody.position).normalized;
                    var vs = Vector3d.Dot(imodel.surface_v, up);
                    debug_energy_acc = imodel.gravity_acc.magnitude * (commanded_vs - vs) / current_v;
                    desired_acc += debug_energy_acc;
                }
                if (Math.Abs(desired_acc - current_acc) < relaxation_acc_error)
                {
                    // we're on low error regime, let's smooth out acceleration error using exponential moving average
                    acc_error = Common.simple_filter(desired_acc - current_acc, acc_error, acc_filter_k);
                }
                else
                {
                    acc_error = desired_acc - current_acc;
                }

                thrust_error = acc_error * imodel.sum_mass;
                surfspd_dir = imodel.surface_v.normalized;
                last_throttle = steady_state_throttle(prograde_thrust + thrust_error);
                cntrl.mainThrottle = spool_compensated(last_throttle);

                //prev_thrust = prograde_thrust;
            }

            return cntrl.mainThrottle;
        }

        public double estimated_max_thrust = 0.0;

        #region Speed reference

        // The speed the throttle chases moves toward the target only as fast as the engines can follow, the way
        // the cruise altitude profile moves only as fast as lift allows. Chased directly, a 400 m/s setpoint step
        // asked for 200 m/s^2, and the energy balance in pitch read the shortfall as a reason to dive.

        /// <summary>
        /// Share of the thrust margin (or of the drag, slowing down) the speed reference accelerates with.
        /// </summary>
        public double reference_acc_share = 0.7;

        /// <summary>
        /// Fastest change of the reference's acceleration, g per second. The engines' spool rate caps it further.
        /// </summary>
        public double reference_jerk_g = 0.5;

        /// <summary>
        /// Time constant, s, the reference settles onto the target with once close to it.
        /// </summary>
        public double reference_settle_time = 2.0;

        /// <summary>
        /// Vertical speed the cruise controller commands this step, m/s, for the energy rate term. NaN when no flight
        /// path is commanded; read once and cleared.
        /// </summary>
        internal double commanded_vertical_speed = double.NaN;

        private double speed_reference = double.NaN;
        private float setpoint_target;
        private double speed_reference_acc;

        /// <summary>
        /// Speed the throttle is chasing now, m/s.
        /// </summary>
        internal double SpeedReference => speed_reference;

        /// <summary>
        /// Whether the wanted throttle sits at full, or at idle.
        /// </summary>
        internal bool ThrottleAtFull => last_throttle >= 0.999f;
        internal bool ThrottleAtIdle => last_throttle <= 0.001f;

        internal double debug_energy_acc;

        private void update_speed_reference()
        {
            var dt = Time.fixedDeltaTime;
            if (double.IsNaN(speed_reference))
            {
                speed_reference = current_v;
                speed_reference_acc = 0.0;
            }
            var mass = Math.Max(1e-3, imodel.sum_mass);
            var max_thrust = 0.0;
            for (var i = 0; i < imodel.engines.Count; i++)
                max_thrust += imodel.engines[i].estimated_max_thrust;
            // Flight path the thrust also has to pay for; drag_estimate is negative
            var g = imodel.gravity_acc.magnitude;
            var sin_gamma = 0.0;
            if (!double.IsNaN(commanded_vertical_speed) && current_v > 1.0)
                sin_gamma = Common.Clamp(commanded_vertical_speed / current_v, 1.0);
            var acc_up = Math.Max(0.0, reference_acc_share * (max_thrust / mass + drag_estimate - g * sin_gamma));
            var acc_down = Math.Max(0.0, reference_acc_share * (-drag_estimate + g * sin_gamma));
            // Engines spool at k of the remaining throttle per second, so thrust changes no faster than that
            var jerk_cap = reference_jerk_g * g;
            var jerk_up = jerk_cap;
            var jerk_down = jerk_cap;
            if (spool_state(true, out var rate_up, out var cur_up))
                jerk_up = Math.Min(jerk_cap, Math.Max(0.05, reference_acc_share * rate_up * (1.0 - cur_up) * max_thrust / mass));
            if (spool_state(false, out var rate_down, out var cur_down))
                jerk_down = Math.Min(jerk_cap, Math.Max(0.05, reference_acc_share * rate_down * cur_down * max_thrust / mass));
            // The damping speed limit protects control authority, so the reference doesn't ease down to it
            if (desired_v < setpoint_target && speed_reference > desired_v)
            {
                speed_reference = desired_v;
                speed_reference_acc = Math.Min(0.0, speed_reference_acc);
            }
            var error = desired_v - speed_reference;
            var limit = error > 0.0 ? acc_up : acc_down;
            var ease_jerk = Math.Min(jerk_up, jerk_down);
            var wanted = Math.Min(limit, Math.Sqrt(2.0 * ease_jerk * Math.Abs(error)));
            if (reference_settle_time > 0.0)
                wanted = Math.Min(wanted, Math.Abs(error) / reference_settle_time);
            wanted *= Math.Sign(error);
            speed_reference_acc += Common.Clamp(wanted - speed_reference_acc, -jerk_down * dt, jerk_up * dt);
            speed_reference += speed_reference_acc * dt;
            // Out of margin the reference stays with the vessel, so pitch never trades height for speed the engines
            // can't hold
            if (acc_up <= 0.0 && speed_reference > current_v)
            {
                speed_reference = current_v;
                speed_reference_acc = Math.Min(0.0, speed_reference_acc);
            }
            if (acc_down <= 0.0 && speed_reference < current_v)
            {
                speed_reference = current_v;
                speed_reference_acc = Math.Max(0.0, speed_reference_acc);
            }
        }

        // Spool rate of the slowest response-time engine for raising (or lowering) thrust, and its throttle
        private bool spool_state(bool raising, out double rate, out double current)
        {
            rate = double.PositiveInfinity;
            current = 0.0;
            for (var i = 0; i < imodel.engines.Count; i++)
            {
                var mode = imodel.engines[i].Mode;
                var data = imodel.engines[i].Data;
                if (mode == null || data == null || !mode.useEngineResponseTime)
                    continue;
                var r = raising ? mode.engineAccelerationSpeed : mode.engineDecelerationSpeed;
                if (r < rate)
                {
                    rate = r;
                    current = data.currentThrottle;
                }
            }
            return !double.IsInfinity(rate) && rate > 0.0;
        }

        #endregion

        /// <summary>
        /// Throttle that gives the required thrust once the engines have spooled to it.
        /// </summary>
        /// <remarks>
        /// Upstream solved for the throttle that reaches the required thrust on the very next physics step, through
        /// the engines' spool-up lag. KSP2 engines spool slowly enough that this almost always asked for 0 or 1, so
        /// the throttle flickered while the engines averaged it out. Commanding the end state lets the engines spool
        /// to it smoothly; the speed loop's proportional gain is gentle enough to tolerate the lag.
        /// </remarks>
        private float steady_state_throttle(double required_thrust)
        {
            estimated_max_thrust = 0.0;
            for (var i = 0; i < imodel.engines.Count; i++)
                estimated_max_thrust += imodel.engines[i].estimated_max_thrust;
            debug_predicted_thrust = required_thrust;
            debug_thrust_authority = estimated_max_thrust;
            iter_count = 0;
            if (estimated_max_thrust <= 0.0)
                return 0.1f;
            var throttle = required_thrust / estimated_max_thrust;
            if (double.IsNaN(throttle) || double.IsInfinity(throttle))
                throttle = 0.5;
            return (float)Common.Clamp(throttle, 0.0, 1.0);
        }

        /// <summary>
        /// Throttle request that makes the engines' own throttle reach the wanted one within spool_time.
        /// </summary>
        /// <remarks>
        /// Engines with response time ease their throttle toward the request with Lerp(cur, req, k * dt), a ~2 s lag
        /// on stock jets that made speed hold overshoot and settle slowly. Asking for cur + (wanted - cur) / (k *
        /// spool_time) makes them move at the faster rate until the clamp to 0..1 limits it. The slowest engine sets
        /// the pace.
        /// </remarks>
        private float spool_compensated(float wanted)
        {
            var cur = 0.0;
            var k = double.PositiveInfinity;
            for (var i = 0; i < imodel.engines.Count; i++)
            {
                var mode = imodel.engines[i].Mode;
                var data = imodel.engines[i].Data;
                if (mode == null || data == null || !mode.useEngineResponseTime)
                    continue;
                var c = data.currentThrottle;
                var rate = wanted > c ? mode.engineAccelerationSpeed : mode.engineDecelerationSpeed;
                if (rate < k)
                {
                    k = rate;
                    cur = c;
                }
            }
            debug_spool_rate = double.IsInfinity(k) ? 0.0 : k;
            if (double.IsInfinity(k) || k <= 0.0 || spool_time <= 0.0)
                return wanted;
            var gain = 1.0 / (k * spool_time);
            if (gain <= 1.0)
                return wanted;
            return (float)Common.Clamp(cur + (wanted - cur) * gain, 0.0, 1.0);
        }

        // [AutoGuiAttr("iter_count", false)]
        public int iter_count;

        #region Damping speed limit

        // KSP2 damps rotation by the parts' angularDrag times the dynamic pressure, and PhysX scales the angular
        // velocity by 1 - damping * dt every step. As that nears one the vessel can't rotate at all, whatever the
        // surfaces do. Damping grows with dynamic pressure, so with speed squared at a given air density, and the
        // speed hold stays below the speed where it would reach max_damping_dt.

        /// <summary>
        /// Highest PhysX angular damping per step, damping * dt, the speed hold will accelerate into.
        /// </summary>
        public float max_damping_dt = 0.9f;

        /// <summary>
        /// How far ahead the limit looks while descending, s. The speed hold can only shed speed with drag, so the
        /// damping is limited for the air the vessel is about to be in.
        /// </summary>
        public float damping_lookahead_time = 3.0f;

        /// <summary>
        /// Height over which air density changes by a factor of e near the surface, m. About Kerbin's.
        /// </summary>
        public float density_scale_height = 5600.0f;

        /// <summary>
        /// Speed the damping limit allows, m/s. Infinity without damping.
        /// </summary>
        internal double DebugEnvelopeSpeed = double.PositiveInfinity;

        private float apply_damping_limit(float target)
        {
            double damping_dt = imodel.angular_damping * Time.fixedDeltaTime;
            if (damping_dt <= 1e-4 || current_v < 1.0)
            {
                DebugEnvelopeSpeed = double.PositiveInfinity;
                return target;
            }
            // Damping scales with dynamic pressure, so with density on the way down
            var up = (vessel.ReferenceTransform.position - vessel.mainBody.position).normalized;
            var descent = Math.Max(0.0, -Vector3d.Dot(imodel.surface_v, up)) * damping_lookahead_time;
            damping_dt *= Math.Exp(descent / density_scale_height);
            DebugEnvelopeSpeed = current_v * Math.Sqrt(max_damping_dt / damping_dt);
            return (float)Math.Min(target, DebugEnvelopeSpeed);
        }

        #endregion

        // Diagnostics for FlightRecorder
        internal double debug_current_acc;
        internal double DebugDesiredAcc => desired_acc;
        internal double DebugAccError => acc_error;
        internal double debug_predicted_thrust;
        internal double debug_thrust_authority;
        internal double debug_spool_rate;
        internal float DebugWantedThrottle => last_throttle;

        // [AutoGuiAttr("v_error", false, "G5")]
        private double v_error;

        // [AutoGuiAttr("desired_acc", false, "G5")]
        private double desired_acc;

        // [AutoGuiAttr("acc_error", false, "G5")]
        private double acc_error;

        // [AutoGuiAttr("thrust_error", false, "G5")]
        private double thrust_error;

        // [GlobalSerializable("use_pid")]
        // [AutoGuiAttr("Use PID", true)]
        public bool use_pid = false;

        private PIDController pid = new PIDController();

        // [VesselSerializable("pid_Kp")]
        // [AutoGuiAttr("pid_Kp", true, "G4")]
        public double pid_Kp { get => pid.KP;
            set => pid.KP = value;
        }

        // [VesselSerializable("pid_Ki")]
        // [AutoGuiAttr("pid_Ki", true, "G4")]
        public double pid_Ki { get => pid.KI;
            set => pid.KI = value;
        }

        // [VesselSerializable("pid_Kd")]
        // [AutoGuiAttr("pid_Kd", true, "G4")]
        public double pid_Kd { get => pid.KD;
            set => pid.KD = value;
        }

        /// <summary>
        /// Speed hold is on
        /// </summary>
        public bool spd_control_enabled = false;

        /// <summary>
        /// Current speed setpoint in m/s, wich is maintained by controller
        /// </summary>
        public float setpoint_mps = 100.0f;
    }
}
