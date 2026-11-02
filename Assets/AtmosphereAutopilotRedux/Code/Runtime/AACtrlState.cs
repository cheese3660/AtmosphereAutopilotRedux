/*
Atmosphere Autopilot Redux, plugin for Kerbal Space Program 2.
Copyright (C) 2026, Infinity Irrealis.
Based on Atmosphere Autopilot, Copyright (C) 2015-2016, Baranin Alexander aka Boris-Barboris.

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

using KSP.Sim.State;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// Mutable copy of the KSP2 FlightCtrlState struct. KSP1's FlightCtrlState was a class, and the ported controllers
    /// pass it around and modify it in place, so we copy in at the start of a tick and back out at the end.
    /// </summary>
    public sealed class AACtrlState
    {
        public float pitch, roll, yaw;
        public float pitchTrim, rollTrim, yawTrim;
        public float mainThrottle;
        public float wheelSteer;
        public bool precision;

        public void CopyFrom(in FlightCtrlState s)
        {
            pitch = s.pitch;
            roll = s.roll;
            yaw = s.yaw;
            pitchTrim = s.pitchTrim;
            rollTrim = s.rollTrim;
            yawTrim = s.yawTrim;
            mainThrottle = s.mainThrottle;
            wheelSteer = s.wheelSteer;
            precision = s.precision;
        }

        public void CopyTo(ref FlightCtrlState s)
        {
            s.pitch = pitch;
            s.roll = roll;
            s.yaw = yaw;
            s.pitchTrim = pitchTrim;
            s.rollTrim = rollTrim;
            s.yawTrim = yawTrim;
            s.mainThrottle = mainThrottle;
            s.wheelSteer = wheelSteer;
        }
    }
}
