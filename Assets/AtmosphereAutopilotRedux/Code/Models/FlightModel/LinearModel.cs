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
using System.Threading;
using UnityEngine;
using System.IO;

using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Modules;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Models
{

    using Vector = VectorArray.Vector;

    public sealed partial class FlightModel : AutopilotModule
    {
        public readonly LinearSystemModel pitch_rot_model_gen = new LinearSystemModel(4, 1);
        public readonly LinearSystemModel roll_rot_model_gen = new LinearSystemModel(3, 3);
        public readonly LinearSystemModel yaw_rot_model_gen = new LinearSystemModel(4, 1);

        public readonly LinearSystemModel pitch_rot_model = new LinearSystemModel(4, 1);
        public readonly LinearSystemModel roll_rot_model = new LinearSystemModel(3, 3);
        public readonly LinearSystemModel yaw_rot_model = new LinearSystemModel(4, 1);

        public readonly LinearSystemModel pitch_rot_model_undelayed = new LinearSystemModel(3, 1);
        public readonly LinearSystemModel roll_rot_model_undelayed = new LinearSystemModel(2, 3);
        public readonly LinearSystemModel yaw_rot_model_undelayed = new LinearSystemModel(3, 1);

        public class PitchYawCoeffs
        {
            public double k0;
            public double k1;
            public double k2;
            public double k0_gen;
            public double k1_gen;
            public double k2_gen;
            public double Cl0;
            public double Cl1;
            public double Cl2;
            public double etq0;
            public double etq1;
            public double et0;
            public double et1;
        }

        public class RollCoeffs
        {
            public double k0;
            public double k1;
            public double k2;
            public double k3;
            public double k4;
            public double k0_gen;
            public double k1_gen;
            public double k2_gen;
            public double k3_gen;
            public double k4_gen;
            public double etq0;
            public double etq1;
        }

        public PitchYawCoeffs pitch_coeffs = new PitchYawCoeffs();
        public PitchYawCoeffs yaw_coeffs = new PitchYawCoeffs();
        public RollCoeffs roll_coeffs = new RollCoeffs();

        private double get_rcs_authority(int axis)
        {
            if (input_buf[axis].GetLast() >= 0.0f)
                return rcs_authority_pos[axis];
            else
                return rcs_authority_neg[axis];
        }

        private void update_pitch_rot_model()
        {
            // fill coeff structs
            pitch_coeffs.Cl0 = pitch_lift_model.pars[0] / 1e3 * dyn_pressure / sum_mass;
            pitch_coeffs.Cl1 = pitch_lift_model.pars[1] / 1e3 * dyn_pressure / sum_mass;
            pitch_coeffs.Cl2 = pitch_lift_model.pars[2] / 1e3 * dyn_pressure / sum_mass;
            pitch_coeffs.etq0 = engines_torque_k0[PITCH] / MOI[PITCH];
            pitch_coeffs.etq1 = engines_torque_k1[PITCH] / MOI[PITCH];
            pitch_coeffs.et0 = -engines_thrust_k0[PITCH] / sum_mass;
            pitch_coeffs.et1 = -engines_thrust_k1[PITCH] / sum_mass;
            pitch_coeffs.k0_gen = pitch_aero_torque_model_gen.pars[0] / 1e2 * dyn_pressure / MOI[PITCH];
            pitch_coeffs.k1_gen = pitch_aero_torque_model_gen.pars[1] / 1e2 * dyn_pressure / MOI[PITCH];
            pitch_coeffs.k2_gen = pitch_aero_torque_model_gen.pars[2] / 1e2 * dyn_pressure / MOI[PITCH];

            // fill pitch_rot_model_gen
            var A = pitch_rot_model_gen.A;
            var B = pitch_rot_model_gen.B;
            var C = pitch_rot_model_gen.C;
            if (dyn_pressure >= 60.0)
            {
                A[0, 0] = -(pitch_coeffs.Cl1 + engines_thrust_principal[ROLL] / sum_mass) / surface_v_magnitude;
                A[0, 2] = -pitch_coeffs.Cl2 / surface_v_magnitude;
                A[1, 0] = pitch_coeffs.k1_gen;
                if (csurf_exponential)
                {
                    // the surface covers csurf_lag_k of the way to the input this step
                    A[1, 2] = pitch_coeffs.k2_gen * (1.0 - csurf_lag_k);
                    B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH] + pitch_coeffs.k2_gen * csurf_lag_k;
                }
                else
                {
                    A[1, 2] = pitch_coeffs.k2_gen;
                    B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH];
                }
                C[0, 0] = -(pitch_gravity_acc + pitch_noninert_acc + pitch_coeffs.Cl0 + pitch_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = pitch_coeffs.k0_gen + pitch_coeffs.etq0;
            }
            else
            {
                A[0, 0] = -engines_thrust_principal[ROLL] / sum_mass / surface_v_magnitude;
                A[0, 2] = 0.0;
                A[1, 0] = 0.0;
                A[1, 2] = 0.0;
                B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH];
                C[0, 0] = -(pitch_gravity_acc + pitch_noninert_acc + pitch_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = pitch_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[1, 3] = pitch_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[1, 0] += pitch_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                B[3, 0] = gimbal_spd_norm;
                A[0, 3] = -pitch_coeffs.et1 / surface_v_magnitude * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] = -pitch_coeffs.et1 / surface_v_magnitude * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[1, 3] = 0.0;
                B[1, 0] += pitch_coeffs.etq1;
                B[3, 0] = 0.0;
                A[0, 3] = 0.0;
                B[0, 0] = -pitch_coeffs.et1 / surface_v_magnitude;
            }
            A[0, 1] = 1.0;
            if (csurf_exponential)
            {
                A[2, 2] = -csurf_lag_rate;
                B[2, 0] = csurf_lag_rate;
                C[2, 0] = 0.0;
            }
            else
            {
                A[2, 2] = 0.0;
                B[2, 0] = 0.0;
                C[2, 0] = csurf_spd;
            }

            pitch_coeffs.k0 = pitch_aero_torque_model.pars[0] / 1e2 * dyn_pressure / MOI[PITCH];
            pitch_coeffs.k1 = pitch_aero_torque_model.pars[1] / 1e2 * dyn_pressure / MOI[PITCH];
            pitch_coeffs.k2 = pitch_aero_torque_model.pars[2] / 1e2 * dyn_pressure / MOI[PITCH];

            // fill pitch_rot_model
            A = pitch_rot_model.A;
            B = pitch_rot_model.B;
            C = pitch_rot_model.C;
            if (dyn_pressure >= 60.0)
            {
                A[0, 0] = -(pitch_coeffs.Cl1 + engines_thrust_principal[ROLL] / sum_mass) / surface_v_magnitude;
                A[0, 2] = -pitch_coeffs.Cl2 / surface_v_magnitude;
                A[1, 0] = pitch_coeffs.k1;
                if (csurf_exponential)
                {
                    // the surface covers csurf_lag_k of the way to the input this step
                    A[1, 2] = pitch_coeffs.k2 * (1.0 - csurf_lag_k);
                    B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH] + pitch_coeffs.k2 * csurf_lag_k;
                }
                else
                {
                    A[1, 2] = pitch_coeffs.k2;
                    B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH];
                }
                C[0, 0] = -(pitch_gravity_acc + pitch_noninert_acc + pitch_coeffs.Cl0 + pitch_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = pitch_coeffs.k0 + pitch_coeffs.etq0;
            }
            else
            {
                A[0, 0] = -engines_thrust_principal[ROLL] / sum_mass / surface_v_magnitude;
                A[0, 2] = 0.0;
                A[1, 0] = 0.0;
                A[1, 2] = 0.0;
                B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH];
                C[0, 0] = -(pitch_gravity_acc + pitch_noninert_acc + pitch_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = pitch_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[1, 3] = pitch_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[1, 0] += pitch_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                B[3, 0] = gimbal_spd_norm;
                A[0, 3] = -pitch_coeffs.et1 / surface_v_magnitude * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] = -pitch_coeffs.et1 / surface_v_magnitude * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[1, 3] = 0.0;
                B[1, 0] += pitch_coeffs.etq1;
                B[3, 0] = 0.0;
                A[0, 3] = 0.0;
                B[0, 0] = -pitch_coeffs.et1 / surface_v_magnitude;
            }
            A[0, 1] = 1.0;
            if (csurf_exponential)
            {
                A[2, 2] = -csurf_lag_rate;
                B[2, 0] = csurf_lag_rate;
                C[2, 0] = 0.0;
            }
            else
            {
                A[2, 2] = 0.0;
                B[2, 0] = 0.0;
                C[2, 0] = csurf_spd;
            }

            // fill pitch_rot_model_undelayed
            A = pitch_rot_model_undelayed.A;
            B = pitch_rot_model_undelayed.B;
            C = pitch_rot_model_undelayed.C;
            if (dyn_pressure >= 60.0)
            {
                A[0, 0] = -(pitch_coeffs.Cl1 + engines_thrust_principal[ROLL] / sum_mass) / surface_v_magnitude;
                A[1, 0] = pitch_coeffs.k1;
                B[0, 0] = -pitch_coeffs.Cl2 / surface_v_magnitude;
                B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH] + pitch_coeffs.k2;
                C[0, 0] = -(pitch_gravity_acc + pitch_noninert_acc + pitch_coeffs.Cl0 + pitch_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = pitch_coeffs.k0 + pitch_coeffs.etq0;
            }
            else
            {
                A[0, 0] = -engines_thrust_principal[ROLL] / sum_mass / surface_v_magnitude;
                A[1, 0] = 0.0;
                B[0, 0] = 0.0;
                B[1, 0] = (reaction_torque[PITCH] + get_rcs_authority(PITCH)) / MOI[PITCH];
                C[0, 0] = -(pitch_gravity_acc + pitch_noninert_acc + pitch_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = pitch_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[1, 2] = pitch_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[1, 0] += pitch_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                A[0, 2] = -pitch_coeffs.et1 / surface_v_magnitude * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] += -pitch_coeffs.et1 / surface_v_magnitude * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[1, 2] = 0.0;
                B[1, 0] += pitch_coeffs.etq1;
                A[0, 2] = 0.0;
                B[0, 0] += -pitch_coeffs.et1 / surface_v_magnitude;
            }
            A[0, 1] = 1.0;

            //Debug.Log("A =\r\n" + A.ToString() + "B =\r\n" + B.ToString() + "C =\r\n" + C.ToString());
        }

        private void update_roll_rot_model()
        {
            // fill coeff structs
            roll_coeffs.etq0 = engines_torque_k0[ROLL] / MOI[ROLL];
            roll_coeffs.etq1 = engines_torque_k1[ROLL] / MOI[ROLL];
            roll_coeffs.k0_gen = roll_aero_torque_model_gen.pars[0] / 1e2 * dyn_pressure / MOI[ROLL];
            roll_coeffs.k1_gen = roll_aero_torque_model_gen.pars[1] / 1e2 * dyn_pressure / MOI[ROLL];
            roll_coeffs.k2_gen = roll_aero_torque_model_gen.pars[2] / surface_v_magnitude * dyn_pressure / MOI[ROLL];
            roll_coeffs.k3_gen = roll_aero_torque_model_gen.pars[3] / 1e2 * dyn_pressure / MOI[ROLL];
            roll_coeffs.k4_gen = roll_aero_torque_model_gen.pars[4] / 1e2 * dyn_pressure / MOI[ROLL];

            // fill roll_rot_model_gen
            var A = roll_rot_model_gen.A;
            var B = roll_rot_model_gen.B;
            var C = roll_rot_model_gen.C;
            if (dyn_pressure > 60.0)
            {
                A[0, 0] = roll_coeffs.k2_gen;
                if (csurf_exponential)
                {
                    // the surface covers csurf_lag_k of the way to the input this step
                    A[0, 1] = roll_coeffs.k3_gen * (1.0 - csurf_lag_k);
                    B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL] + roll_coeffs.k3_gen * csurf_lag_k;
                }
                else
                {
                    A[0, 1] = roll_coeffs.k3_gen;
                    B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL];
                }
                B[0, 1] = roll_coeffs.k4_gen;
                B[0, 2] = roll_coeffs.k1_gen;
                C[0, 0] = roll_coeffs.k0_gen + roll_coeffs.etq0;
            }
            else
            {
                A[0, 0] = 0.0;
                A[0, 0] = 0.0;
                A[0, 1] = 0.0;
                B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL];
                B[0, 1] = 0.0;
                B[0, 2] = 0.0;
                C[0, 0] = roll_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[0, 2] = roll_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] += roll_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                B[2, 0] = gimbal_spd_norm;
            }
            else
            {
                A[0, 2] = 0.0;
                B[0, 0] += roll_coeffs.etq1;
                B[2, 0] = 0.0;
            }
            if (csurf_exponential)
            {
                A[1, 1] = -csurf_lag_rate;
                B[1, 0] = csurf_lag_rate;
                C[1, 0] = 0.0;
            }
            else
            {
                A[1, 1] = 0.0;
                B[1, 0] = 0.0;
                C[1, 0] = csurf_spd;
            }

            roll_coeffs.k0 = roll_aero_torque_model.pars[0] / 1e2 * dyn_pressure / MOI[ROLL];
            roll_coeffs.k1 = roll_aero_torque_model.pars[1] / 1e2 * dyn_pressure / MOI[ROLL];
            roll_coeffs.k2 = roll_aero_torque_model.pars[2] / surface_v_magnitude * dyn_pressure / MOI[ROLL];
            roll_coeffs.k3 = roll_aero_torque_model.pars[3] / 1e2 * dyn_pressure / MOI[ROLL];
            roll_coeffs.k4 = roll_aero_torque_model.pars[4] / 1e2 * dyn_pressure / MOI[ROLL];

            // fill roll_rot_model
            A = roll_rot_model.A;
            B = roll_rot_model.B;
            C = roll_rot_model.C;
            if (dyn_pressure > 60.0)
            {
                A[0, 0] = roll_coeffs.k2;
                if (csurf_exponential)
                {
                    // the surface covers csurf_lag_k of the way to the input this step
                    A[0, 1] = roll_coeffs.k3 * (1.0 - csurf_lag_k);
                    B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL] + roll_coeffs.k3 * csurf_lag_k;
                }
                else
                {
                    A[0, 1] = roll_coeffs.k3;
                    B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL];
                }
                B[0, 1] = roll_coeffs.k4;
                B[0, 2] = roll_coeffs.k1;
                C[0, 0] = roll_coeffs.k0 + roll_coeffs.etq0;
            }
            else
            {
                A[0, 0] = 0.0;
                A[0, 0] = 0.0;
                A[0, 1] = 0.0;
                B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL];
                B[0, 1] = 0.0;
                B[0, 2] = 0.0;
                C[0, 0] = roll_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[0, 2] = roll_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] += roll_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                B[2, 0] = gimbal_spd_norm;
            }
            else
            {
                A[0, 2] = 0.0;
                B[0, 0] += roll_coeffs.etq1;
                B[2, 0] = 0.0;
            }
            if (csurf_exponential)
            {
                A[1, 1] = -csurf_lag_rate;
                B[1, 0] = csurf_lag_rate;
                C[1, 0] = 0.0;
            }
            else
            {
                A[1, 1] = 0.0;
                B[1, 0] = 0.0;
                C[1, 0] = csurf_spd;
            }

            // fill roll_rot_model_undelayed
            A = roll_rot_model_undelayed.A;
            B = roll_rot_model_undelayed.B;
            C = roll_rot_model_undelayed.C;
            if (dyn_pressure > 60.0)
            {
                A[0, 0] = roll_coeffs.k2;
                B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL] + roll_coeffs.k3;
                B[0, 1] = roll_coeffs.k4;
                B[0, 2] = roll_coeffs.k1;
                C[0, 0] = roll_coeffs.k0 + roll_coeffs.etq0;
            }
            else
            {
                A[0, 0] = 0.0;
                B[0, 0] = (reaction_torque[ROLL] + get_rcs_authority(ROLL)) / MOI[ROLL];
                B[0, 1] = 0.0;
                B[0, 2] = 0.0;
                C[0, 0] = 0.0 + roll_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[0, 1] = roll_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] += roll_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[0, 1] = 0.0;
                B[0, 0] += roll_coeffs.etq1;
            }
        }

        private void update_yaw_rot_model()
        {
            // Fill coeff structs
            yaw_coeffs.Cl0 = yaw_lift_model.pars[0] / 1e3 * dyn_pressure / sum_mass;
            yaw_coeffs.Cl1 = yaw_lift_model.pars[1] / 1e3 * dyn_pressure / sum_mass;
            yaw_coeffs.Cl2 = yaw_lift_model.pars[2] / 1e3 * dyn_pressure / sum_mass;
            yaw_coeffs.etq0 = engines_torque_k0[YAW] / MOI[YAW];
            yaw_coeffs.etq1 = engines_torque_k1[YAW] / MOI[YAW];
            yaw_coeffs.et0 = engines_thrust_k0[YAW] / sum_mass;
            yaw_coeffs.et1 = engines_thrust_k1[YAW] / sum_mass;
            yaw_coeffs.k0_gen = yaw_aero_torque_model_gen.pars[0] / 1e2 * dyn_pressure / MOI[YAW];
            yaw_coeffs.k1_gen = yaw_aero_torque_model_gen.pars[1] / 1e2 * dyn_pressure / MOI[YAW];
            yaw_coeffs.k2_gen = yaw_aero_torque_model_gen.pars[2] / 1e2 * dyn_pressure / MOI[YAW];

            // Fill yaw_rot_model_gen
            var A = yaw_rot_model_gen.A;
            var B = yaw_rot_model_gen.B;
            var C = yaw_rot_model_gen.C;
            if (dyn_pressure >= 60.0)
            {
                A[0, 0] = -(yaw_coeffs.Cl1 + engines_thrust_principal[ROLL] / sum_mass) / surface_v_magnitude;
                A[0, 2] = -yaw_coeffs.Cl2 / surface_v_magnitude;
                A[1, 0] = yaw_coeffs.k1_gen;
                if (csurf_exponential)
                {
                    // the surface covers csurf_lag_k of the way to the input this step
                    A[1, 2] = yaw_coeffs.k2_gen * (1.0 - csurf_lag_k);
                    B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW] + yaw_coeffs.k2_gen * csurf_lag_k;
                }
                else
                {
                    A[1, 2] = yaw_coeffs.k2_gen;
                    B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW];
                }
                C[0, 0] = -(yaw_gravity_acc + yaw_noninert_acc + yaw_coeffs.Cl0 + yaw_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = yaw_coeffs.k0_gen + yaw_coeffs.etq0;
            }
            else
            {
                A[0, 0] = -engines_thrust_principal[ROLL] / sum_mass / surface_v_magnitude;
                A[0, 2] = 0.0;
                A[1, 0] = 0.0;
                A[1, 2] = 0.0;
                B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW];
                C[0, 0] = -(yaw_gravity_acc + yaw_noninert_acc + yaw_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = yaw_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[1, 3] = yaw_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[1, 0] += yaw_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                B[3, 0] = gimbal_spd_norm;
                A[0, 3] = -yaw_coeffs.et1 / surface_v_magnitude * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] = -yaw_coeffs.et1 / surface_v_magnitude * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[1, 3] = 0.0;
                B[1, 0] += yaw_coeffs.etq1;
                B[3, 0] = 0.0;
                A[0, 3] = 0.0;
                B[0, 0] = -yaw_coeffs.et1 / surface_v_magnitude;
            }
            A[0, 1] = 1.0;
            if (csurf_exponential)
            {
                A[2, 2] = -csurf_lag_rate;
                B[2, 0] = csurf_lag_rate;
                C[2, 0] = 0.0;
            }
            else
            {
                A[2, 2] = 0.0;
                B[2, 0] = 0.0;
                C[2, 0] = csurf_spd;
            }

            yaw_coeffs.k0 = yaw_aero_torque_model.pars[0] / 1e2 * dyn_pressure / MOI[YAW];
            yaw_coeffs.k1 = yaw_aero_torque_model.pars[1] / 1e2 * dyn_pressure / MOI[YAW];
            yaw_coeffs.k2 = yaw_aero_torque_model.pars[2] / 1e2 * dyn_pressure / MOI[YAW];

            // Fill yaw_rot_model
            A = yaw_rot_model.A;
            B = yaw_rot_model.B;
            C = yaw_rot_model.C;
            if (dyn_pressure >= 60.0)
            {
                A[0, 0] = -(yaw_coeffs.Cl1 + engines_thrust_principal[ROLL] / sum_mass) / surface_v_magnitude;
                A[0, 2] = -yaw_coeffs.Cl2 / surface_v_magnitude;
                A[1, 0] = yaw_coeffs.k1;
                if (csurf_exponential)
                {
                    // the surface covers csurf_lag_k of the way to the input this step
                    A[1, 2] = yaw_coeffs.k2 * (1.0 - csurf_lag_k);
                    B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW] + yaw_coeffs.k2 * csurf_lag_k;
                }
                else
                {
                    A[1, 2] = yaw_coeffs.k2;
                    B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW];
                }
                C[0, 0] = -(yaw_gravity_acc + yaw_noninert_acc + yaw_coeffs.Cl0 + yaw_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = yaw_coeffs.k0 + yaw_coeffs.etq0;
            }
            else
            {
                A[0, 0] = -engines_thrust_principal[ROLL] / sum_mass / surface_v_magnitude;
                A[0, 2] = 0.0;
                A[1, 0] = 0.0;
                A[1, 2] = 0.0;
                B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW];
                C[0, 0] = -(yaw_gravity_acc + yaw_noninert_acc + yaw_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = yaw_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[1, 3] = yaw_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[1, 0] += yaw_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                B[3, 0] = gimbal_spd_norm;
                A[0, 3] = -yaw_coeffs.et1 / surface_v_magnitude * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] = -yaw_coeffs.et1 / surface_v_magnitude * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[1, 3] = 0.0;
                B[1, 0] += yaw_coeffs.etq1;
                B[3, 0] = 0.0;
                A[0, 3] = 0.0;
                B[0, 0] = -yaw_coeffs.et1 / surface_v_magnitude;
            }
            A[0, 1] = 1.0;
            if (csurf_exponential)
            {
                A[2, 2] = -csurf_lag_rate;
                B[2, 0] = csurf_lag_rate;
                C[2, 0] = 0.0;
            }
            else
            {
                A[2, 2] = 0.0;
                B[2, 0] = 0.0;
                C[2, 0] = csurf_spd;
            }

            // Fill yaw_rot_model_undelayed
            A = yaw_rot_model_undelayed.A;
            B = yaw_rot_model_undelayed.B;
            C = yaw_rot_model_undelayed.C;
            if (dyn_pressure >= 60.0)
            {
                A[0, 0] = -(yaw_coeffs.Cl1 + engines_thrust_principal[ROLL] / sum_mass) / surface_v_magnitude;
                A[1, 0] = yaw_coeffs.k1;
                B[0, 0] = -yaw_coeffs.Cl2 / surface_v_magnitude;
                B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW] + yaw_coeffs.k2;
                C[0, 0] = -(yaw_gravity_acc + yaw_noninert_acc + yaw_coeffs.Cl0 + yaw_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = yaw_coeffs.k0 + yaw_coeffs.etq0;
            }
            else
            {
                A[0, 0] = -engines_thrust_principal[ROLL] / sum_mass / surface_v_magnitude;
                A[1, 0] = 0.0;
                B[0, 0] = 0.0;
                B[1, 0] = (reaction_torque[YAW] + get_rcs_authority(YAW)) / MOI[YAW];
                C[0, 0] = -(yaw_gravity_acc + yaw_noninert_acc + yaw_coeffs.et0) / surface_v_magnitude;
                C[1, 0] = yaw_coeffs.etq0;
            }
            if (any_gimbals && any_gimbal_spds)
            {
                A[1, 2] = yaw_coeffs.etq1 * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[1, 0] += yaw_coeffs.etq1 * gimbal_spd_norm * Time.fixedDeltaTime;
                A[0, 2] = -yaw_coeffs.et1 / surface_v_magnitude * (1.0 - gimbal_spd_norm * Time.fixedDeltaTime);
                B[0, 0] += -yaw_coeffs.et1 / surface_v_magnitude * gimbal_spd_norm * Time.fixedDeltaTime;
            }
            else
            {
                A[1, 2] = 0.0;
                B[1, 0] += yaw_coeffs.etq1;
                A[0, 2] = 0.0;
                B[0, 0] += -yaw_coeffs.et1 / surface_v_magnitude;
            }
            A[0, 1] = 1.0;

            //Debug.Log("A =\r\n" + A.ToString() + "B =\r\n" + B.ToString() + "C =\r\n" + C.ToString());
        }
    }
}
