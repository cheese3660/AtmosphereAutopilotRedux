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
using KSP.Sim;
using KSP.Sim.impl;
using UnityEngine;

using AtmosphereAutopilotRedux.Modules;

namespace AtmosphereAutopilotRedux.Models
{
    public sealed partial class FlightModel : AutopilotModule
    {
        public class EngineMoment
        {
            public EngineMoment(Module_Engine m, bool gimbaled)
            {
                engine = m;
                this.gimbaled = gimbaled;
            }
            public Module_Engine engine;
            public bool gimbaled;
            public Vector3 thrust = Vector3.zero;
            public Vector3 potential_torque = Vector3.zero;
            public Vector3 torque_components = Vector3.zero;
            public double estimated_max_thrust = 0.0;
            public double estimated_max_thrust_unlimited = 0.0;

            public Data_Engine Data => engine.DataEngine;
            public Data_Engine.EngineMode Mode => engine.currentEngineModeData;
        }

        public List<EngineMoment> engines = new List<EngineMoment>();

        private void get_engines()
        {
            engines.Clear();
            any_gimbals = false;
            any_gimbal_spds = false;
            gimbal_spd_norm = float.PositiveInfinity;
            foreach (var part in vessel.View.parts)
            {
                if (part == null)
                    continue;
                foreach (var module in part.Modules.Values)
                {
                    if (!(module is Module_Engine eng) || eng.DataEngine == null)
                        continue;
                    if (!eng.isOperational && eng.DataEngine.FinalThrustValue == 0.0f)
                        continue;
                    var gimbaled = false;
                    if (part.Model != null &&
                        part.Model.TryGetModuleData<PartComponentModule_Gimbal, Data_Gimbal>(out var gimbal))
                    {
                        gimbaled = true;
                        any_gimbals = true;
                        // KSP2 gimbals ease toward their target with Lerp(old, target, speed * dt), the same
                        // exponential response the model uses. We read the slowest one instead of forcing them
                        // all to one speed as the KSP1 version did.
                        if (gimbal.useGimbalResponseSpeed)
                        {
                            any_gimbal_spds = true;
                            gimbal_spd_norm = Math.Min(gimbal_spd_norm, gimbal.gimbalResponseSpeed);
                        }
                    }
                    engines.Add(new EngineMoment(eng, gimbaled));
                }
            }
        }

        public float gimbal_spd_norm = float.PositiveInfinity;

        public bool any_gimbals = false;

        public bool any_gimbal_spds = false;

        public Vector3 engines_torque_principal;

        public Vector3 engines_thrust_principal;

        private float abs_thrust;

        private void update_engine_moments()
        {
            engines_torque_principal = Vector3.zero;
            engines_thrust_principal = Vector3.zero;
            abs_thrust = 0.0f;
            for (var i = 0; i < engines.Count; i++)
            {
                var eng = engines[i].engine;
                if (eng == null || eng.DataEngine == null)
                {
                    moments_cycle_counter = 0;      // need to reform parts lists
                    continue;
                }
                var data = eng.DataEngine;
                var mode = eng.currentEngineModeData;
                if (mode?.ThrustTransforms == null || mode.ThrustTransformMultipliers == null)
                    continue;
                var final_thrust = data.FinalThrustValue;
                var throttle = data.currentThrottle;
                var e_thrust = Vector3.zero;
                var e_potent = Vector3.zero;
                var e_max_thrust = 0.0;
                var tcount = Math.Min(mode.ThrustTransforms.Count, mode.ThrustTransformMultipliers.Count);
                for (var j = 0; j < tcount; j++)
                {
                    var trans = mode.ThrustTransforms[j];
                    if (trans == null)
                        continue;
                    var tpos = world_to_cntrl_part * (trans.position - CoM);
                    var tdir = world_to_cntrl_part * (-trans.forward);
                    var torque_arm = -Vector3.Cross(tpos, tdir);       // minus because Unity's left-handed
                    var mult = mode.ThrustTransformMultipliers[j];
                    if (throttle > 0.0f)
                    {
                        double nozzle_max_thrust = final_thrust * mult / throttle;
                        e_max_thrust += nozzle_max_thrust;
                        // KSP1 rotated gimbals back to neutral here. KSP2 gimbal transforms are not ours to move,
                        // so the current nozzle direction stands in for the neutral one.
                        e_potent += torque_arm * (float)nozzle_max_thrust;
                    }
                    else
                        e_max_thrust += mode.maxThrust * mult;
                    engines_torque_principal += torque_arm * final_thrust * mult;
                    engines_thrust_principal += final_thrust * tdir * mult;
                    e_thrust += final_thrust * mult * (-trans.forward);
                }
                abs_thrust += final_thrust;
                engines[i].thrust = e_thrust;
                engines[i].potential_torque = e_potent;
                engines[i].torque_components = e_potent.normalized;
                engines[i].estimated_max_thrust = e_max_thrust * eng.ThrustPercentage * 0.01f;
                engines[i].estimated_max_thrust_unlimited = e_max_thrust;
            }
        }

        private float prev_abs_thrust = 0.0f;

        private Vector3 prev_engines_torque, prev_engines_thrust;

        private Vector3 engines_torque_k0;

        private Vector3 engines_torque_k1;

        private Vector3 engines_thrust_k0;

        private Vector3 engines_thrust_k1;

        private bool gimbals_tested = false;

        // Stupid linear authority of gimbals, verry approximate but simple and effective.
        // engines_torque = engines_torque_k0 + user_input * engines_torque_k1
        // engines_thrust = engines_thrust_k0 + user_input * engines_thrust_k1
        private void get_gimbal_authority()
        {
            if (any_gimbals && (prev_abs_thrust != 0.0f) && (abs_thrust != 0.0f) && (gimbal_buf[0].Size >= 2))
            {
                var scaled_prev_torque = prev_engines_torque / prev_abs_thrust;
                var scaled_cur_torque = engines_torque_principal / abs_thrust;
                var scaled_prev_thrust = prev_engines_thrust / prev_abs_thrust;
                var scaled_cur_thrust = engines_thrust_principal / abs_thrust;
                for (var axis = 0; axis < 3; axis++)
                {
                    // prepare thrust index workaround
                    var t_axis = ROLL;
                    switch (axis)
                    {
                        case PITCH:
                            t_axis = YAW;
                            break;
                        case YAW:
                            t_axis = PITCH;
                            break;
                    }

                    var cur_cntrl = gimbal_buf[axis].GetLast();
                    var last_cntrl = gimbal_buf[axis].GetFromTail(1);
                    if (Math.Abs(cur_cntrl - last_cntrl) > 0.02)            // only significant input signal changes are analyzed
                    {
                        gimbals_tested = true;
                        // torque
                        var k1 = (scaled_cur_torque[axis] - scaled_prev_torque[axis]) / (cur_cntrl - last_cntrl);
                        if (k1 < 0.0f)
                            k1 = 0.0f;
                        var k0 = scaled_cur_torque[axis] - cur_cntrl * k1;
                        engines_torque_k0[axis] = k0 * abs_thrust;
                        engines_torque_k1[axis] = k1 * abs_thrust;
                        // thrust
                        k1 = (scaled_cur_thrust[t_axis] - scaled_prev_thrust[t_axis]) / (cur_cntrl - last_cntrl);
                        k0 = scaled_cur_thrust[t_axis] - cur_cntrl * k1;
                        engines_thrust_k0[axis] = k0 * abs_thrust;
                        engines_thrust_k1[axis] = k1 * abs_thrust;
                    }
                    else
                    {
                        // torque
                        var k1 = 0.0f;
                        if (gimbals_tested)
                            k1 = engines_torque_k1[axis] / abs_thrust;
                        else
                            k1 = MOI[axis] / abs_thrust;
                        var k0 = scaled_cur_torque[axis] - cur_cntrl * k1;
                        engines_torque_k0[axis] = k0 * abs_thrust;
                        // thrust
                        k1 = engines_thrust_k1[axis] / abs_thrust;
                        k0 = scaled_cur_thrust[t_axis] - cur_cntrl * k1;
                        engines_thrust_k0[axis] = k0 * abs_thrust;
                    }
                }
            }
            else
            {
                engines_torque_k0 = engines_torque_principal;
                engines_torque_k1 = Vector3.zero;
                engines_thrust_k0 = engines_thrust_principal;
                engines_thrust_k1 = Vector3.zero;
            }
            prev_abs_thrust = abs_thrust;
            prev_engines_torque = engines_torque_principal;
            prev_engines_thrust = engines_thrust_principal;
        }
    }

}
