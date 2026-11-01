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

using AtmosphereAutopilotRedux.Runtime;

namespace AtmosphereAutopilotRedux.Modules
{
    /// <summary>
    /// Flight control state controller base class
    /// </summary>
    public abstract class StateController : AutopilotModule
    {
        protected StateController(AAVessel cur_vessel, string module_name)
            : base(cur_vessel, module_name)
        { }

        /// <summary>
        /// Main control function of high-level autopilot.
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        public abstract void ApplyControl(AACtrlState cntrl);

    }

    /// <summary>
    /// Flight control state controller with SIMO base class
    /// </summary>
    public abstract class SISOController : AutopilotModule
    {
        protected SISOController(AAVessel cur_vessel, string module_name)
            : base(cur_vessel, module_name)
        { }

        /// <summary>
        /// Main control function of service autopilot.
        /// </summary>
        /// <param name="cntrl">Control state to change</param>
        /// <param name="target_value">Desired controlled value</param>
        public abstract float ApplyControl(AACtrlState cntrl, float target_value);
    }

    public static class ControlUtils
    {
        public const int PITCH = 0;
        public const int ROLL = 1;
        public const int YAW = 2;

        public static float get_neutralized_user_input(AACtrlState state, int axis)
        {
            switch (axis)
            {
                case PITCH:
                    return state.pitch - state.pitchTrim;
                case ROLL:
                    return state.roll - state.rollTrim;
                case YAW:
                    return state.yaw - state.yawTrim;
                default:
                    return 0.0f;
            }
        }

        public static void neutralize_user_input(AACtrlState state, int axis)
        {
            switch (axis)
            {
                case PITCH:
                    state.pitch = state.pitchTrim;
                    break;
                case ROLL:
                    state.roll = state.rollTrim;
                    break;
                case YAW:
                    state.yaw = state.yawTrim;
                    break;
            }
        }

        public static void set_raw_output(AACtrlState state, int axis, float output)
        {
            switch (axis)
            {
                case PITCH:
                    state.pitch = output;
                    break;
                case ROLL:
                    state.roll = output;
                    break;
                case YAW:
                    state.yaw = output;
                    break;
            }
        }

        public static float getControlFromState(AACtrlState state, int axis)
        {
            switch (axis)
            {
                case PITCH:
                    return state.pitch;
                case ROLL:
                    return state.roll;
                case YAW:
                    return state.yaw;
                default:
                    return 0.0f;
            }
        }
    }
}
