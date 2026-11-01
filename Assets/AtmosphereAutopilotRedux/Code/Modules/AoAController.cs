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

    public abstract class AoAController : AutopilotModule
    {
        private FlightModel imodel;
        private PitchYawAngularVelocityController v_controller;

        private int axis;

        internal AoAController(AAVessel v, string cntrl_name, int axis) : base(v, cntrl_name)
        {
            this.axis = axis;
        }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            switch (axis)
            {
                case PITCH:
                    v_controller = modules[typeof(PitchAngularVelocityController)] as PitchYawAngularVelocityController;
                    lin_model_gen = imodel.pitch_rot_model_gen;
                    lin_model = imodel.pitch_rot_model;
                    break;
                case YAW:
                    v_controller = modules[typeof(YawAngularVelocityController)] as PitchYawAngularVelocityController;
                    lin_model_gen = imodel.yaw_rot_model_gen;
                    lin_model = imodel.yaw_rot_model;
                    break;
            }
        }

        protected override void OnActivate()
        {
            imodel.Activate();
            v_controller.Activate();
        }

        protected override void OnDeactivate()
        {
            imodel.Deactivate();
            v_controller.Deactivate();
        }

        // [AutoGuiAttr("AoA", false, "G6")]
        protected float cur_aoa;

        // [AutoGuiAttr("desired_aoa", false, "G6")]
        protected float desired_aoa;

        // [AutoGuiAttr("output_v", false, "G6")]
        protected float output_v;

        // [AutoGuiAttr("output_acc", false, "G6")]
        protected float output_acc;

        // [AutoGuiAttr("des_aoa_equil_v", false, "G6")]
        public float desired_aoa_equilibr_v;

        // [AutoGuiAttr("v_filter_k", true, "G6")]
        protected float v_filter_k = 0.0f;

        protected LinearSystemModel lin_model_gen, lin_model;

        private Matrix eq_A = new Matrix(2, 2);
        private Matrix eq_b = new Matrix(2, 1);
        private Matrix eq_x;

        // [AutoGuiAttr("relaxation_frame", true, "G6")]
        protected float relaxation_frame = 2.0f;

        // [AutoGuiAttr("relaxation_factor", true, "G6")]
        protected float relaxation_factor = 0.5f;

        // [AutoGuiAttr("cubic_barrier", true, "G6")]
        protected float cubic_barrier = 1.0f;

        // [AutoGuiAttr("cubic_KP", true, "G6")]
        protected float cubic_kp = 0.2f;

        // [AutoGuiAttr("cubic mode", false)]
        protected bool cubic = false;

        public bool user_controlled = true;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired AoA in radians</param>
        /// <param name="target_derivative">Desired AoA derivative</param>
        public float ApplyControl(AACtrlState cntrl, float target_value, float target_derivative)
        {
            if (user_controlled && (
                imodel.dyn_pressure <= (v_controller.moder_cutoff_ias * v_controller.moder_cutoff_ias)
                || !v_controller.moderate_aoa))
            {
                v_controller.user_controlled = true;
                v_controller.ApplyControl(cntrl, 0.0f);
                return 0.0f;
            }

            cur_aoa = imodel.AoA(axis);

            var user_input = Common.Clampf(ControlUtils.get_neutralized_user_input(cntrl, axis), 1.0f);
            if (user_controlled || user_input != 0.0f)
            {
                if (user_input >= 0.0f)
                    desired_aoa = user_input * v_controller.res_max_aoa;
                else
                    desired_aoa = -user_input * v_controller.res_min_aoa;
                user_controlled = true;
            }
            else
                desired_aoa = (float)Common.Clamp(target_value, v_controller.res_min_aoa, v_controller.res_max_aoa);

            // Let's find equilibrium angular v on desired_aoa
            var model = lin_model_gen;
            eq_A[0, 0] = model.A[0, 1];
            eq_A[0, 1] = model.A[0, 2] + model.A[0, 3] + model.B[0, 0];
            eq_A[1, 0] = model.A[1, 1];
            eq_A[1, 1] = model.A[1, 2] + model.B[1, 0] + model.A[1, 3];
            eq_b[0, 0] = target_derivative - (model.A[0, 0] * desired_aoa + model.C[0, 0]);
            eq_b[1, 0] = -(model.A[1, 0] * desired_aoa + model.C[1, 0]);
            eq_A.old_lu = true;
            try
            {
                eq_x = eq_A.SolveWith(eq_b);
                var new_eq_v = eq_x[0, 0];
                if (!double.IsInfinity(new_eq_v) && !double.IsNaN(new_eq_v))
                    desired_aoa_equilibr_v = (float)Common.simple_filter(new_eq_v, desired_aoa_equilibr_v, v_filter_k);
            }
            catch (MSingularException) { }
            //cur_aoa_equilibr_v += 0.5f * (float)get_roll_aoa_deriv();

            // parabolic descend to desired angle of attack
            double error = Common.Clampf(desired_aoa - cur_aoa, Mathf.Abs(v_controller.res_max_aoa - v_controller.res_min_aoa));

            // special workaround for twitches on out of controllable regions for overdamped planes
            var stable_out_of_bounds = false;
            if (v_controller.staticaly_stable &&
                   ((desired_aoa == v_controller.max_input_aoa && error < 0.0) ||
                    (desired_aoa == v_controller.min_input_aoa && error > 0.0)))
                stable_out_of_bounds = true;

            var k = v_controller.transit_max_v * v_controller.transit_max_v / 2.0 / (v_controller.res_max_aoa - v_controller.res_min_aoa);
            var t = -Math.Sqrt(Math.Abs(error / k));
            double descend_v;
            if (t < -cubic_barrier)
            {
                // we're still far away from desired aoa, we'll descend using parabolic function
                cubic = false;
                var t_step = Math.Min(0.0, t + Time.fixedDeltaTime);
                var relaxation = 1.0;
                if (t >= -relaxation_frame * Time.fixedDeltaTime)
                    relaxation = relaxation_factor;
                descend_v = relaxation * k * (t * t - t_step * t_step) * Math.Sign(error) / Time.fixedDeltaTime;
                output_acc = 0.0f;
            }
            else
            {
                // we're close to desired aoa, we'll descend using cubic function
                cubic = true;
                double kacc_quadr = Math.Abs(v_controller.kacc_quadr);
                var k_cubic = kacc_quadr / 6.0 * cubic_kp;
                var t_cubic = -Math.Pow(Math.Abs(error / k_cubic), 0.33);
                var t_step = Math.Min(0.0, t_cubic + Time.fixedDeltaTime);
                if (t >= -relaxation_frame * Time.fixedDeltaTime)
                {
                    descend_v = relaxation_factor * error / Math.Max((relaxation_frame * Time.fixedDeltaTime), Time.fixedDeltaTime);
                }
                else
                {
                    descend_v = k_cubic * (t_step * t_step * t_step - t_cubic * t_cubic * t_cubic) * Math.Sign(error) / Time.fixedDeltaTime;
                }
            }

            if (stable_out_of_bounds)
                descend_v = -0.2 * descend_v;

            output_v = (float)(descend_v + desired_aoa_equilibr_v);

            ControlUtils.neutralize_user_input(cntrl, axis);
            v_controller.user_controlled = false;
            v_controller.ApplyControl(cntrl, output_v);

            return output_v;
        }

        private double get_roll_aoa_deriv()
        {
            var pitch_aoa = new Vector3(imodel.AoA(PITCH), 0.0f, 0.0f);
            var yaw_aoa = new Vector3(0.0f, imodel.AoA(YAW), 0.0f);
            var ang_v = new Vector3(0.0f, 0.0f, imodel.AngularVel(ROLL));
            var plane_vel = Vector3.Cross(ang_v, pitch_aoa + yaw_aoa);
            switch (axis)
            {
                case PITCH:
                    return Vector3.Dot(Vector3.right, plane_vel);
                case YAW:
                    return Vector3.Dot(Vector3.up, plane_vel);
                default:
                    return 0.0;
            }
        }
    }

    public sealed class SideslipController : AoAController
    {
        internal SideslipController(AAVessel v) :
            base(v, "Sideslip controller", YAW) { }
    }

    public sealed class PitchAoAController : AoAController
    {
        internal PitchAoAController(AAVessel v) :
            base(v, "AoA controller", PITCH) { }
    }
}
