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

using KSP.Game;
using KSP.Sim;
using KSP.Sim.impl;
using UnityEngine;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// Celestial body seen from Unity physics space. Stands in for the KSP1 CelestialBody members the autopilot used.
    /// </summary>
    public sealed class AABody
    {
        public CelestialBodyComponent Sim { get; private set; }

        /// <summary>Body center in physics space.</summary>
        public Vector3d position;

        /// <summary>Unit rotation axis (north) in physics space.</summary>
        public Vector3d RotationAxis;

        public double Radius => Sim.radius;

        internal void Refresh(CelestialBodyComponent body, IPhysicsSpaceProvider space)
        {
            Sim = body;
            position = space.PositionToPhysics(body.transform.Position);
            RotationAxis = space.VectorToPhysics(new Vector(body.transform.bodyFrame, Vector3d.up)).normalized;
        }

        /// <summary>
        /// Physics-space position of a surface point.
        /// </summary>
        /// <param name="lat">Latitude in degrees</param>
        /// <param name="lon">Longitude in degrees</param>
        /// <param name="alt">Altitude above sea level in meters</param>
        public Vector3d GetWorldSurfacePosition(double lat, double lon, double alt)
        {
            var pos = new Position(Sim.transform.bodyFrame, Sim.GetRelSurfacePosition(lat, lon, alt));
            return AAVessel.PhysicsSpace.PositionToPhysics(pos);
        }
    }

    /// <summary>
    /// Per-physics-step snapshot of the vessel in Unity physics space. Stands in for the KSP1 Vessel members the
    /// autopilot used, so the ported controllers keep their original shape.
    /// </summary>
    public sealed class AAVessel
    {
        public static IPhysicsSpaceProvider PhysicsSpace => GameManager.Instance.Game.UniverseView.PhysicsSpace;

        public VesselComponent Sim { get; private set; }
        public VesselBehavior View { get; private set; }

        public readonly AABody mainBody = new AABody();

        public AAVessel(VesselComponent sim, VesselBehavior view)
        {
            Sim = sim;
            View = view;
        }

        /// <summary>Control point transform. Up is the nose, right is the pitch axis, forward is belly-down.</summary>
        public Transform ReferenceTransform => View.ControlTransform;

        public Vector3d srf_velocity;
        public Vector3d obt_velocity;
        public double srfSpeed;
        public double obt_speed;
        public double horizontalSrfSpeed;
        public double altitude;
        public double latitude;
        public double longitude;
        public double heightFromTerrain;
        public double atmDensity;
        public double totalMass;
        private Vector3d com;

        /// <summary>Center of mass in physics space.</summary>
        public Vector3d GetWorldPos3D() => com;

        public bool LandedOrSplashed() => Sim.LandedOrSplashed;

        /// <summary>
        /// Refresh the snapshot. Called once per physics step before the autopilot runs.
        /// </summary>
        /// <returns>false when the vessel state can't be read this step</returns>
        public bool Refresh()
        {
            var space = PhysicsSpace;
            if (space == null || Sim.mainBody == null || View == null || View.PartOwner == null)
                return false;

            mainBody.Refresh(Sim.mainBody, space);

            if (!Sim.SimulationObject.Telemetry.TryGetMovementVelocitiesNow(out var orbital, out var surface))
                return false;
            srf_velocity = space.VectorToPhysics(surface);
            obt_velocity = space.VectorToPhysics(orbital);
            srfSpeed = srf_velocity.magnitude;
            obt_speed = obt_velocity.magnitude;

            com = View.PartOwner.GetLiveCenterOfMass();
            var up = (com - mainBody.position).normalized;
            horizontalSrfSpeed = Vector3d.Exclude(up, srf_velocity).magnitude;

            altitude = Sim.AltitudeFromSeaLevel;
            latitude = Sim.Latitude;
            longitude = Sim.Longitude;
            heightFromTerrain = Sim.AltitudeFromTerrain;
            atmDensity = Sim.AtmDensity;
            totalMass = Sim.totalMass;
            return true;
        }

        public void SetBrakes(bool on) => Sim.SetActionGroup(KSPActionGroup.Brakes, on);
    }
}
