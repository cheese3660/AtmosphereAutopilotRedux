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
    public sealed class StandardFlyByWire : StateController
    {
        private PitchAngularVelocityController pc;
        private RollAngularVelocityController rc;
        private YawAngularVelocityController yvc;
        private SideslipController yc;
        private ProgradeThrustController tc;
        private FlightModel im;

        internal StandardFlyByWire(AAVessel v) :
            base(v, "Standard Fly-By-Wire") { }

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            pc = modules[typeof(PitchAngularVelocityController)] as PitchAngularVelocityController;
            rc = modules[typeof(RollAngularVelocityController)] as RollAngularVelocityController;
            yvc = modules[typeof(YawAngularVelocityController)] as YawAngularVelocityController;
            yc = modules[typeof(SideslipController)] as SideslipController;
            tc = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
            im = modules[typeof(FlightModel)] as FlightModel;
        }

        protected override void OnActivate()
        {
            pc.Activate();
            pc.user_controlled = true;
            rc.Activate();
            rc.user_controlled = true;
            yvc.Activate();
            yvc.user_controlled = rocket_mode;
            yc.Activate();
            yc.user_controlled = true;
            tc.Activate();
        }

        protected override void OnDeactivate()
        {
            pc.neutral_offset = 0.0f;
            pc.Deactivate();
            rc.Deactivate();
            yvc.Deactivate();
            yc.Deactivate();
            tc.Deactivate();        // upstream left the thrust controller running
        }

        // [VesselSerializable("rocket_mode")]
        // [AutoGuiAttr("Rocket mode", true)]
        public bool rocket_mode = false;

        public bool RocketMode
        {
            get => rocket_mode;
            set
            {
                if (value != rocket_mode)
                {
                    rocket_mode = value;
                }
            }
        }

        // [AutoGuiAttr("Moderation", true)]
        public bool moderation_switch
        {
            get => (pc.moderate_aoa || pc.moderate_g || yvc.moderate_aoa || yvc.moderate_g);
            set
            {
                if (value != moderation_switch)
                {
                    pc.moderate_aoa = pc.moderate_g = yvc.moderate_aoa = yvc.moderate_g = value;
                }
            }
        }

        // [AutoGuiAttr("Coordinated turn", true)]
        // [VesselSerializable("coord_turn")]
        public bool coord_turn = false;

        public bool Coord_turn
        {
            get => coord_turn;
            set
            {
                if (value != coord_turn)
                {
                    coord_turn = value;
                }
            }
        }

        private bool landed = false;
        private bool need_restore = false;
        private float time_after_takeoff = 0.0f;
        private bool aoa_moder = true;
        private bool g_moder = true;

        /// <summary>
        /// Main control function
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public override void ApplyControl(AACtrlState cntrl)
        {
            if (vessel.LandedOrSplashed())
            {
                landed = true;
                time_after_takeoff = 0.0f;
                return;
            }

            // disable pitch moderation for two seconds after take-off
            if (landed || need_restore)
            {
                if (landed && !need_restore)
                {
                    aoa_moder = pc.moderate_aoa;
                    g_moder = pc.moderate_g;
                    pc.moderate_aoa = false;
                    pc.moderate_g = false;
                    landed = false;
                    need_restore = true;
                }
                if (time_after_takeoff > 1.5f)
                {
                    pc.moderate_aoa = aoa_moder;
                    pc.moderate_g = g_moder;
                    need_restore = false;
                }
                else
                    time_after_takeoff += Time.fixedDeltaTime;
            }

            if (tc.spd_control_enabled)
                tc.ApplyControl(cntrl, tc.setpoint_mps);

			pc.user_controlled = true;
            if (coord_turn)
            {
                // account for yaw velocity in pitch neutral offset to assist coordinated turn
                var up_level_dir = Vector3.ProjectOnPlane(vessel.ReferenceTransform.position - vessel.mainBody.position,
                    vessel.ReferenceTransform.up).normalized;
                var yaw_v_vert_project = Vector3.Dot(im.AngularVel(YAW) * vessel.ReferenceTransform.right, up_level_dir);
                var pitch_vert_project = Vector3.Dot(up_level_dir, -vessel.ReferenceTransform.forward);
                if (pitch_vert_project > 0.0f)
                {
                    var level_pitch_vel = -yaw_v_vert_project / pitch_vert_project;
                    pc.neutral_offset = level_pitch_vel;
                }
                else
                    pc.neutral_offset = 0.0f;
            }
            else
                pc.neutral_offset = 0.0f;
            pc.ApplyControl(cntrl, 0.0f);

			if (rocket_mode)
			{
				yvc.user_controlled = true;
				yvc.ApplyControl(cntrl, 0.0f);
			}
			else
			{
				yc.user_controlled = true;
				yc.ApplyControl(cntrl, 0.0f, 0.0f);
			}
			rc.user_controlled = true;
            rc.ApplyControl(cntrl, 0.0f);
        }
    }
}
