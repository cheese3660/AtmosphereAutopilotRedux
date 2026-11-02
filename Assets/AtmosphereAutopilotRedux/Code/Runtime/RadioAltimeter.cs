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

using System;
using UnityEngine;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// Height of the craft's lowest point over the runway, as an airliner's radio altimeter reads it over the gear.
    /// It is the centre of mass's altitude over the runway surface from the airport data, flat before the threshold,
    /// less how far below the centre of mass the craft reaches. A raycast found no ground on approach, and the game's
    /// own terrain altitude is from the vessel's transform to the heightmap, without the runway.
    /// </summary>
    public sealed class RadioAltimeter
    {
        /// <summary>Distance below the centre of mass, m, of the craft's lowest point, from the last calibration.</summary>
        public double LowestPointOffset { get; private set; } = double.NaN;

        /// <summary>Whether the offset was measured on a runway rather than estimated from the colliders.</summary>
        public bool Measured { get; private set; }

        /// <summary>
        /// Takes the offset as measured: the centre of mass's height over the runway while standing on it, or the
        /// estimate corrected by what the altimeter read at touchdown. Collider estimates don't replace it.
        /// </summary>
        public void SetMeasured(double offset)
        {
            if (double.IsNaN(offset) || double.IsInfinity(offset) || offset < 0.0 || offset > 50.0)
                return;
            LowestPointOffset = offset;
            Measured = true;
        }

        /// <summary>
        /// Measures how far below the centre of mass the craft reaches, along the local vertical: its lowest collider
        /// point, wheels at their radius plus suspension travel. Call with the gear down. A collider's closest point
        /// is trusted only within its bounds: for some it is the far query point itself, which once put the gear
        /// a kilometre below the craft.
        /// </summary>
        public void Calibrate(AAVessel vessel)
        {
            if (Measured)
                return;
            var com = (Vector3)vessel.GetWorldPos3D();
            var down = -(Vector3)(vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            var abs_down = new Vector3(Mathf.Abs(down.x), Mathf.Abs(down.y), Mathf.Abs(down.z));
            var lowest = 0.0f;
            foreach (var part in vessel.View.parts)
            {
                if (part?.Colliders == null)
                    continue;
                foreach (var collider in part.Colliders)
                {
                    if (collider == null || !collider.enabled || collider.isTrigger)
                        continue;
                    float depth;
                    if (collider is WheelCollider wheel)
                    {
                        var centre = wheel.transform.TransformPoint(wheel.center);
                        depth = Vector3.Dot(centre - com, down) + wheel.suspensionDistance + wheel.radius;
                    }
                    else
                    {
                        // The bounds' lowest corner is as far down as the collider can reach
                        var b = collider.bounds;
                        depth = Vector3.Dot(b.center - com, down) + Vector3.Dot(b.extents, abs_down);
                        if (!(collider is MeshCollider mesh && !mesh.convex))
                        {
                            var closest = Vector3.Dot(collider.ClosestPoint(com + down * 1000.0f) - com, down);
                            if (closest <= depth)
                                depth = closest;
                        }
                    }
                    if (!float.IsNaN(depth) && !float.IsInfinity(depth))
                        lowest = Mathf.Max(lowest, depth);
                }
            }
            LowestPointOffset = lowest;
        }

        /// <summary>
        /// Height of the lowest point over the runway surface below, m; over the centre of mass before calibration.
        /// </summary>
        public double Height(AAVessel vessel, ApproachPath path)
        {
            var offset = double.IsNaN(LowestPointOffset) ? 0.0 : LowestPointOffset;
            return vessel.altitude - path.SurfaceElevation(vessel) - offset;
        }
    }
}
