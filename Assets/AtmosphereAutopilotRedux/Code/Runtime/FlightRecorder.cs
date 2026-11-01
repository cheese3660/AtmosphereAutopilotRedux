/*
Atmosphere Autopilot Redux, plugin for Kerbal Space Program 2.
Copyright (C) 2026, Infinity Irrealis.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.
You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Diagnostics: per-physics-step flight recorder for tuning the autopilot. Off unless the hidden Debug/FlightRecorder
// config value is set.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using KSP.Modules;
using KSP.Sim;
using KSP.Sim.impl;
using KSP.Sim.State;
using Redux.Ecs.Structural;
using UnityEngine;

using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Modules;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// Writes one CSV row per physics step, engaged or not, to
    /// %USERPROFILE%/AppData/LocalLow/&lt;company&gt;/&lt;product&gt;/AtmosphereAutopilotRedux/flight_*.csv.
    /// </summary>
    /// <remarks>
    /// Turned on by the hidden Debug/FlightRecorder value in the mod's config json, read once when the mod
    /// initializes. While on, the flight model also runs disengaged so open-loop flight is measured.
    /// </remarks>
    internal sealed class FlightRecorder : IDisposable
    {
        public static bool Enabled = false;

        public static void Configure(bool enabled)
        {
            Enabled = enabled;
            if (Enabled)
                Debug.Log("[AtmosphereAutopilot] Flight recorder enabled");
        }

        private readonly AutopilotContext context;
        private StreamWriter writer;
        private readonly List<string> header = new List<string>();
        private readonly StringBuilder row = new StringBuilder();
        private bool header_written;
        private long tick;

        // hook calls per physics step
        private double last_fixed_time = double.NaN;
        private int calls_this_step;

        // prediction made last step for this step's angular acceleration
        private readonly double[] prev_predicted = new double[3];

        // finite-difference rate of the control transform, the way stock SAS measures it
        private Quaternion last_rotation;
        private bool has_last_rotation;

        // actuators, rescanned when the part count changes
        private int scanned_part_count = -1;
        private readonly List<(string name, Module_ControlSurface module)> surfaces = new List<(string, Module_ControlSurface)>();
        private readonly List<Module_ReactionWheel> wheels = new List<Module_ReactionWheel>();
        private Rigidbody control_body;

        private const int MaxSurfaces = 8;
        private const int MaxEngines = 4;

        private static readonly FieldInfo deflection_field =
            typeof(Module_ControlSurface).GetField("_deflection", BindingFlags.Instance | BindingFlags.NonPublic);

        private static readonly FieldInfo action_field =
            typeof(Module_ControlSurface).GetField("_action", BindingFlags.Instance | BindingFlags.NonPublic);

        private static readonly FieldInfo wheel_input_field =
            typeof(Module_ReactionWheel).GetField("_inputVector", BindingFlags.Instance | BindingFlags.NonPublic);

        public FlightRecorder(AutopilotContext context)
        {
            this.context = context;
        }

        /// <summary>
        /// Count a hook call. Call first thing in the handler, before any early return.
        /// </summary>
        public void CountCall()
        {
            if (Time.fixedTimeAsDouble != last_fixed_time)
            {
                last_fixed_time = Time.fixedTimeAsDouble;
                calls_this_step = 0;
            }
            calls_this_step++;
        }

        public void Record(in FlightCtrlState raw_in, in FlightCtrlState prev_applied, AACtrlState output, bool engaged)
        {
            if (!Enabled)
                return;
            try
            {
                RecordRow(raw_in, prev_applied, output, engaged);
            }
            catch (Exception e)
            {
                Debug.LogError("[AtmosphereAutopilot] Flight recorder failed, disabling it\n" + e);
                Enabled = false;
                Dispose();
            }
        }

        private void RecordRow(in FlightCtrlState raw_in, in FlightCtrlState prev_applied, AACtrlState output, bool engaged)
        {
            var vessel = context.Vessel;
            var m = context.FlightModel;
            Rescan(vessel);

            row.Clear();
            var column = 0;
            void Col(string name, double value)
            {
                if (!header_written)
                    header.Add(name);
                if (column++ > 0)
                    row.Append(',');
                if (double.IsNaN(value) || double.IsInfinity(value))
                    row.Append("nan");
                else
                    row.Append(value.ToString("G6", CultureInfo.InvariantCulture));
            }
            void Flag(string name, bool value) => Col(name, value ? 1 : 0);

            // 1. timing and context
            Col("t", Time.fixedTimeAsDouble);
            Col("tick", tick++);
            Col("dt", Time.fixedDeltaTime);
            Col("calls", calls_this_step);
            Flag("engaged", engaged);
            var active = context.ActiveController;
            Col("mode", active == null ? 0 : active == context.FlyByWire ? 1 : active == context.Cruise ? 2 : 3);
            Flag("seq", m.sequential_dt);
            Flag("landed", vessel.LandedOrSplashed());
            Flag("analytical", StructuralPhysicsService.IsAnalytical(vessel.View.PartOwner));
            Col("q", m.dyn_pressure);
            Col("damp_dt", m.angular_damping * Time.fixedDeltaTime);
            Col("spd", vessel.srfSpeed);
            Col("alt", vessel.altitude);
            Col("alt_tel", vessel.Sim.AltitudeFromSeaLevel);
            Col("mass", m.sum_mass);
            Col("moi_p", m.MOI.x);
            Col("moi_r", m.MOI.y);
            Col("moi_y", m.MOI.z);
            Flag("cs_exp", m.csurf_exponential);
            Col("cs_lag_k", m.csurf_lag_k);
            Col("cs_spd", m.csurf_spd);

            // 2. input path
            Col("in_p", raw_in.inputPitch);
            Col("in_r", raw_in.inputRoll);
            Col("in_y", raw_in.inputYaw);
            Col("st_p", raw_in.pitch);
            Col("st_r", raw_in.roll);
            Col("st_y", raw_in.yaw);
            Col("trim_p", raw_in.pitchTrim);
            Col("trim_r", raw_in.rollTrim);
            Col("trim_y", raw_in.yawTrim);
            Flag("precision", raw_in.precision);
            Col("applied_prev_p", prev_applied.pitch);
            Col("applied_prev_r", prev_applied.roll);
            Col("applied_prev_y", prev_applied.yaw);
            Col("applied_prev_thr", prev_applied.mainThrottle);
            Col("out_p", output.pitch);
            Col("out_r", output.roll);
            Col("out_y", output.yaw);
            Col("out_thr", output.mainThrottle);

            // 4. second and third rate measurements, in the flight model's sign convention
            var world_to_cntrl = m.world_to_cntrl_part;
            var rb_rate = control_body != null ? -(world_to_cntrl * control_body.angularVelocity) : new Vector3(float.NaN, float.NaN, float.NaN);
            var fd_rate = new Vector3(float.NaN, float.NaN, float.NaN);
            var rotation = vessel.ReferenceTransform.rotation;
            if (has_last_rotation)
            {
                var delta = Quaternion.Inverse(last_rotation) * rotation;
                if (delta.w < 0f)
                    delta = new Quaternion(-delta.x, -delta.y, -delta.z, -delta.w);
                delta.ToAngleAxis(out var angle, out var axis);
                fd_rate = -axis * (angle * Mathf.Deg2Rad / Time.fixedDeltaTime);
            }
            last_rotation = rotation;
            has_last_rotation = true;

            var wheel_input = Vector3.zero;
            if (wheel_input_field != null)
                foreach (var wheel in wheels)
                    if (wheel != null)
                        wheel_input += (Vector3)wheel_input_field.GetValue(wheel);

            // 3, 6. per axis controller and model
            var vel = context.VelocityControllers;
            string[] axes = { "p", "r", "y" };
            for (var i = 0; i < 3; i++)
            {
                var a = axes[i];
                var vc = vel[i];
                var acc = vc.DebugAccController;
                Col(a + "_dv", vc.Active ? vc.DebugDesiredV : double.NaN);
                Col(a + "_v", m.AngularVel(i));
                Col(a + "_v_rb", rb_rate[i]);
                Col(a + "_v_fd", fd_rate[i]);
                Col(a + "_da", vc.Active ? vc.DebugOutputAcc : double.NaN);
                Col(a + "_a", m.AngularAccHistory(i).Size > 0 ? m.AngularAcc(i) : double.NaN);
                Col(a + "_pa", prev_predicted[i]);
                prev_predicted[i] = acc.Active ? acc.DebugPredictedAcc : double.NaN;
                Col(a + "_auth", acc.Active ? acc.DebugAuthority : double.NaN);
                Col(a + "_branch", acc.Active ? acc.DebugBranch : -1);
                Col(a + "_auth_an", m.DebugAnalyticAuthority(i));
                Col(a + "_k1_an", m.DebugAnalyticStiffness(i));
                Col(a + "_indi_next", acc.Active ? acc.DebugIndiNext : double.NaN);
                Col(a + "_indi_scale", acc.Active ? acc.DebugIndiScale : double.NaN);
                Col(a + "_indi_fit", acc.Active ? acc.DebugIndiScaleFit : double.NaN);
                Col(a + "_indi_auth", acc.Active ? acc.IndiAuthority : double.NaN);
                Col(a + "_damp_an", i == AutopilotModule.ROLL ? m.DebugAnalyticRollDamping() : double.NaN);
                Col(a + "_cs", m.ControlSurfPos(i));
                Col(a + "_gimbal", m.GimbalPos(i));
                Col(a + "_aoa", m.AoA(i));
                Col(a + "_wheel", wheel_input[i]);
                var k = m.DebugAeroPars(i);
                for (var j = 0; j < k.Length; j++)
                    Col($"{a}_k{j}", k[j]);
                var kg = m.DebugAeroParsGen(i);
                for (var j = 0; j < kg.Length; j++)
                    Col($"{a}_kg{j}", kg[j]);
            }
            // linear model entries the controllers read: acc row, surface coupling, surface state row
            LinearSystemModel pm = m.pitch_rot_model, rm = m.roll_rot_model, ym = m.yaw_rot_model;
            Col("p_A_acc_cs", pm.A[1, 2]); Col("p_B_acc", pm.B[1, 0]); Col("p_C_acc", pm.C[1, 0]);
            Col("p_A_cs_cs", pm.A[2, 2]); Col("p_B_cs", pm.B[2, 0]); Col("p_C_cs", pm.C[2, 0]);
            Col("p_Bu_acc", m.pitch_rot_model_undelayed.B[1, 0]);
            Col("r_A_acc_cs", rm.A[0, 1]); Col("r_B_acc", rm.B[0, 0]); Col("r_C_acc", rm.C[0, 0]);
            Col("r_A_cs_cs", rm.A[1, 1]); Col("r_B_cs", rm.B[1, 0]); Col("r_C_cs", rm.C[1, 0]);
            Col("r_Bu_acc", m.roll_rot_model_undelayed.B[0, 0]);
            Col("y_A_acc_cs", ym.A[1, 2]); Col("y_B_acc", ym.B[1, 0]); Col("y_C_acc", ym.C[1, 0]);
            Col("y_A_cs_cs", ym.A[2, 2]); Col("y_B_cs", ym.B[2, 0]); Col("y_C_cs", ym.C[2, 0]);
            Col("y_Bu_acc", m.yaw_rot_model_undelayed.B[1, 0]);

            // 5. actuators
            for (var i = 0; i < MaxSurfaces; i++)
            {
                var has = i < surfaces.Count && surfaces[i].module != null && deflection_field != null;
                var name = i < surfaces.Count ? $"s{i}_{surfaces[i].name}" : $"s{i}";
                Col(name + "_defl", has ? (float)deflection_field.GetValue(surfaces[i].module) : double.NaN);
                Col(name + "_act", has && action_field != null ? (float)action_field.GetValue(surfaces[i].module) : double.NaN);
            }
            for (var i = 0; i < MaxEngines; i++)
            {
                var data = i < m.engines.Count && m.engines[i].engine != null ? m.engines[i].Data : null;
                Col($"e{i}_req", data != null ? data.RequestedThrottle : double.NaN);
                Col($"e{i}_cur", data != null ? data.currentThrottle : double.NaN);
                Col($"e{i}_thrust", data != null ? data.FinalThrustValue : double.NaN);
            }

            // 7. speed controller
            var tc = context.Speed;
            Flag("t_on", tc.spd_control_enabled);
            Col("t_set", tc.setpoint_mps);
            Col("t_env", tc.DebugEnvelopeSpeed);
            Col("t_vref", tc.SpeedReference);
            Col("t_energy_acc", tc.debug_energy_acc);
            Col("d_vert_int", context.Director.DebugVertIntegral);
            Col("t_da", tc.DebugDesiredAcc);
            Col("t_a", tc.debug_current_acc);
            Col("t_err", tc.DebugAccError);
            Col("t_prograde", tc.prograde_thrust);
            Col("t_drag", tc.drag_estimate);
            Col("t_pred", tc.debug_predicted_thrust);
            Col("t_auth", tc.debug_thrust_authority);
            Col("t_iter", tc.iter_count);
            Col("t_maxT", tc.estimated_max_thrust);
            Col("t_wanted", tc.DebugWantedThrottle);
            Col("t_spool_k", tc.debug_spool_rate);

            // 8. cruise altitude hold and director
            CruiseController cc = context.Cruise;
            DirectorController dc = context.Director;
            Vector3d up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            Col("c_mode", (int)cc.current_mode);
            Col("c_vert", cc.vertical_control ? (cc.height_mode == CruiseController.HeightMode.Altitude ? 1 : 2) : 0);
            Col("c_alt_set", cc.desired_altitude);
            Col("c_vs_set", cc.desired_vertsetpoint);
            Col("c_vs", Vector3d.Dot(vessel.srf_velocity, up));
            Col("c_h_err", cc.debug_height_error);
            Col("c_vs_raw", cc.debug_raw_vert_speed);
            Col("c_vs_cmd", cc.debug_cmd_vert_speed);
            Col("c_vs_max", cc.debug_max_vert_speed);
            Col("c_climb_ang", cc.debug_max_climb_angle);
            Col("c_speed_w", cc.debug_speed_weight);
            Col("c_balance_vs", cc.debug_balance_vs);
            Col("c_relax_k", cc.debug_relax_k);
            Flag("c_apply_acc", cc.debug_apply_acc);
            Col("c_vert_acc", Vector3d.Dot(cc.DebugDesiredVertAcc, up));
            Col("c_drag_f", cc.DebugFilteredDrag);
            Col("d_aoa", dc.desired_aoa);
            Col("d_max_lift", dc.max_lift_acc);
            Col("d_strength", dc.strength);
            Col("d_turn_err", dc.debug_turn_error);
            Col("d_turn_acc", dc.debug_turn_acc);
            Col("d_turn_limit", dc.debug_turn_limit);
            Col("d_roll_err", dc.roll_error);

            if (writer == null)
                Open(vessel);
            if (!header_written)
            {
                writer.WriteLine(string.Join(",", header));
                header_written = true;
            }
            writer.WriteLine(row.ToString());
            if (tick % 50 == 0)
                writer.Flush();
        }

        private void Rescan(AAVessel vessel)
        {
            var part_count = vessel.View.PartOwner.PartsCount;
            if (part_count == scanned_part_count && control_body != null)
                return;
            scanned_part_count = part_count;
            surfaces.Clear();
            wheels.Clear();
            foreach (var part in vessel.View.parts)
            {
                if (part == null)
                    continue;
                foreach (var module in part.Modules.Values)
                {
                    switch (module)
                    {
                        case Module_ControlSurface cs when surfaces.Count < MaxSurfaces:
                            surfaces.Add((part.Name.Replace(',', '_').Replace(' ', '_'), cs));
                            break;
                        case Module_ReactionWheel wheel:
                            wheels.Add(wheel);
                            break;
                    }
                }
            }
            control_body = vessel.ReferenceTransform.GetComponentInParent<Rigidbody>();
            // A new set of surfaces means new columns
            if (header_written)
            {
                Close();
                header.Clear();
                header_written = false;
            }
        }

        private void Open(AAVessel vessel)
        {
            var folder = Path.Combine(Application.persistentDataPath, "AtmosphereAutopilotRedux");
            Directory.CreateDirectory(folder);
            var vessel_name = vessel.Sim.SimulationObject.Name ?? "vessel";
            foreach (var c in Path.GetInvalidFileNameChars())
                vessel_name = vessel_name.Replace(c, '_');
            var path = Path.Combine(folder, $"flight_{DateTime.Now:yyyyMMdd_HHmmss}_{vessel_name}.csv");
            writer = new StreamWriter(path, false, new UTF8Encoding(false));
            Debug.Log("[AtmosphereAutopilot] Flight recorder writing to " + path);
        }

        private void Close()
        {
            writer?.Flush();
            writer?.Dispose();
            writer = null;
        }

        public void Dispose() => Close();
    }
}
