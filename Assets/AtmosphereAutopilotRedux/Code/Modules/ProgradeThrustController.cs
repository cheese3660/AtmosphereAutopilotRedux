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
            v_integral = 0.0;
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
            desired_v = target_value;

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
            }
            else
            {
                surfspd_dir = imodel.prev_surface_v.normalized;
                var thrust = imodel.prev_cntrl2world * imodel.engines_thrust_principal;
                prograde_thrust = Vector3.Dot(thrust, surfspd_dir);

                double current_acc = Vector3.Dot(imodel.sum_acc, surfspd_dir);
                debug_current_acc = current_acc;
                drag_estimate = current_acc - prograde_thrust / imodel.sum_mass - Vector3d.Dot(imodel.gravity_acc + imodel.noninert_acc, surfspd_dir);

                v_error = desired_v - current_v;
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
                desired_acc = Kp_v * v_error + Ki_v * v_integral;
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
                cntrl.mainThrottle = steady_state_throttle(prograde_thrust + thrust_error);
                last_throttle = cntrl.mainThrottle;

                //prev_thrust = prograde_thrust;
            }

            return cntrl.mainThrottle;
        }

        public double estimated_max_thrust = 0.0;

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

        // [AutoGuiAttr("iter_count", false)]
        public int iter_count;

        // Diagnostics for FlightRecorder
        internal double debug_current_acc;
        internal double DebugDesiredAcc => desired_acc;
        internal double DebugAccError => acc_error;
        internal double debug_predicted_thrust;
        internal double debug_thrust_authority;

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
