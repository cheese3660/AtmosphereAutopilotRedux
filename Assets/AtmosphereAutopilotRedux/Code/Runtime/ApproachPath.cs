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

using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// Where the vessel is relative to one runway end's approach: off the extended centreline, short of the landing
    /// threshold, and against the glideslope. All of it is measured on the planet's surface, the great circle through
    /// the runway, so it stays right far out, where the surface has dropped kilometres below a straight line.
    /// </summary>
    public sealed class ApproachPath
    {
        public Runway Runway { get; }
        public RunwayEnd End { get; }
        public RunwayEnd Far { get; }

        /// <summary>Glideslope angle, degrees.</summary>
        public double GlideslopeDeg;

        public ApproachPath(Runway runway, RunwayEnd end, double glideslope_deg)
        {
            Runway = runway;
            End = end;
            Far = runway.Opposite(end);
            GlideslopeDeg = glideslope_deg;
        }

        /// <summary>Runway course, degrees clockwise from north.</summary>
        public double CourseDeg => SurfaceGeometry.Bearing(End.Threshold, Far.Threshold);

        /// <summary>Elevation of the landing threshold, m above sea level.</summary>
        public double ThresholdAltitude => End.Threshold.Altitude;

        /// <summary>Distance right of the extended centreline, m; negative is left.</summary>
        public double CrossTrack { get; private set; }

        /// <summary>Distance along the centreline to the landing threshold, m; negative once past it.</summary>
        public double AlongTrack { get; private set; }

        /// <summary>Unit horizontal direction of the runway course where the vessel is, physics space.</summary>
        public Vector3d CourseDirection { get; private set; }

        /// <summary>Altitude above sea level the glideslope has where the vessel is, m.</summary>
        public double PathAltitude { get; private set; }

        /// <summary>Distance between the two thresholds, m.</summary>
        public double Length { get; private set; }

        public void Update(AAVessel vessel)
        {
            var body = vessel.mainBody;
            var radius = body.Radius;
            var threshold = (body.GetWorldSurfacePosition(End.Threshold.Latitude, End.Threshold.Longitude,
                End.Threshold.Altitude) - body.position).normalized;
            var far = (body.GetWorldSurfacePosition(Far.Threshold.Latitude, Far.Threshold.Longitude,
                Far.Threshold.Altitude) - body.position).normalized;
            var up = (vessel.GetWorldPos3D() - body.position).normalized;

            // The runway's great circle: the threshold and its course there span its plane, and turning the course
            // a quarter clockwise about the threshold's vertical gives its normal on the right side
            var course_at_threshold = (far - threshold * Vector3d.Dot(far, threshold)).normalized;
            var right = (QuaternionD.AngleAxis(90.0, threshold) * course_at_threshold).normalized;
            this.radius = radius;
            this.threshold = threshold;
            this.course_at_threshold = course_at_threshold;
            this.right = right;
            CrossTrack = radius * Math.Asin(Common.Clamp(Vector3d.Dot(up, right), 1.0));

            // Angle past the threshold, of the vessel's point brought onto the great circle
            var on_path = (up - right * Vector3d.Dot(up, right)).normalized;
            var past = Math.Atan2(Vector3d.Dot(on_path, course_at_threshold), Vector3d.Dot(on_path, threshold));
            AlongTrack = -radius * past + End.DisplacedM;
            CourseDirection = (course_at_threshold * Math.Cos(past) - threshold * Math.Sin(past)).normalized;
            Length = radius * Math.Acos(Common.Clamp(Vector3d.Dot(threshold, far), 1.0));

            PathAltitude = End.Threshold.Altitude + End.ThresholdCrossingHeightM +
                AlongTrack * Math.Tan(GlideslopeDeg * Mathf.Deg2Rad);
        }

        // The runway's frame as of the last Update, unit vectors from the body's centre
        private double radius;
        private Vector3d threshold, course_at_threshold, right;

        /// <summary>
        /// Unit vector from the body's centre to the surface point <paramref name="along" /> before the landing
        /// threshold on the centreline and <paramref name="cross" /> right of it, in the frame of the last Update.
        /// </summary>
        public Vector3d SurfaceDirection(double along, double cross)
        {
            var past = -(along - End.DisplacedM) / radius;
            var on_path = threshold * Math.Cos(past) + course_at_threshold * Math.Sin(past);
            var side = cross / radius;
            return (on_path * Math.Cos(side) + right * Math.Sin(side)).normalized;
        }

        /// <summary>
        /// Elevation of the runway surface, m above sea level, <paramref name="along" /> before the landing threshold:
        /// the threshold's before it, then the flat plane through both thresholds. Runways are flat, not curved with
        /// the planet, so mid-runway the surface sits below a straight blend of the threshold altitudes by
        /// x (L - x) / 2R: 9.7 m at the middle of KSC's 6.8 km runway, as sampled in game.
        /// </summary>
        public double RunwayElevationAt(double along)
        {
            if (along >= 0.0 || Length <= 0.0)
                return End.Threshold.Altitude;
            var past = Math.Min(-along, Length);
            var blend = End.Threshold.Altitude + (Far.Threshold.Altitude - End.Threshold.Altitude) * past / Length;
            return radius > 0.0 ? blend - past * (Length - past) / (2.0 * radius) : blend;
        }

        /// <summary>
        /// Elevation of the surface under the vessel, m above sea level. Over the runway it is the higher of the
        /// flat-plane model and the terrain: KSC's runways are box colliders on a plane well above the terrain, the
        /// Island's is flattened terrain at one altitude, curving with the planet, and either way the runway is the
        /// higher of the two. Before the threshold it is the threshold's elevation.
        /// </summary>
        public double SurfaceElevation(AAVessel vessel)
        {
            var model = RunwayElevationAt(AlongTrack);
            if (AlongTrack >= 0.0 || AlongTrack < -Length || Math.Abs(CrossTrack) > Math.Max(100.0, Runway.WidthM))
                return model;
            var terrain = vessel.Sim.mainBody?.SurfaceProvider?.GetTerrainAltitudeFromCenter(vessel.latitude,
                vessel.longitude);
            if (terrain == null || double.IsNaN(terrain.Value))
                return model;
            return Math.Max(model, terrain.Value - radius);
        }

        /// <summary>Distance along the surface between two unit vectors from the body's centre, m.</summary>
        public double SurfaceDistance(Vector3d a, Vector3d b) =>
            radius * Math.Acos(Common.Clamp(Vector3d.Dot(a, b), 1.0));
    }
}
