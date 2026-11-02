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
using System.Collections.Generic;
using KSP.Modules;
using KSP.Sim;
using UnityEngine;

using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{
    public enum TakeoffPhase
    {
        Off,
        /// <summary>Full thrust down the centreline.</summary>
        Roll,
        /// <summary>Raising the nose from rotate speed until airborne.</summary>
        Rotate,
        /// <summary>Climbing out on the runway course at the climb speed.</summary>
        Climb,
        /// <summary>Takeoff rejected: idle, brakes, holding the centreline to a stop.</summary>
        Rejected
    }

    /// <summary>
    /// Takes off from a runway on top of cruise control, the way an airliner's TOGA and runway modes do: full thrust
    /// down the centreline, rotation at the rotate speed, a climb on the runway course at the climb speed to the
    /// acceleration altitude, then cruise takes over with the cruise tab's altitude and speed. On the ground the
    /// flight model and the director don't apply, so the roll and rotation are flown by rate loops that learn the
    /// craft's steering and elevator as they go. The abort key rejects the takeoff below the rotate speed.
    /// </summary>
    public sealed class TakeoffGuidance : AutopilotModule
    {
        internal TakeoffGuidance(AAVessel v)
            : base(v, "Takeoff guidance")
        { }

        private FlightModel imodel;
        private ProgradeThrustController thrust_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
            approach = modules[typeof(ApproachGuidance)] as ApproachGuidance;
        }

        private ApproachGuidance approach;

        /// <summary>Steering gain learned on the last takeoff roll, or NaN; the landing rollout starts from it.</summary>
        public double LearnedSteeringGain { get; internal set; } = double.NaN;

        public ApproachPath Path { get; private set; }

        public TakeoffPhase Phase { get; private set; } = TakeoffPhase.Off;

        public event Action PhaseChanged;

        /// <summary>What the pilot should know, such as the takeoff being rejected; empty when nothing.</summary>
        public string Warning { get; private set; } = "";

        /// <summary>Speed to raise the nose at, m/s. Also the decision speed: past it an abort is ignored.</summary>
        public double rotate_speed = 70.0;

        /// <summary>Pitch attitude to rotate to, degrees.</summary>
        public double rotate_pitch_deg = 10.0;

        /// <summary>Speed to climb out at, m/s.</summary>
        public double climb_speed = 90.0;

        /// <summary>Speed the climb holds, m/s: the climb speed, or the lift-off speed if that was faster.</summary>
        public double ClimbSpeed { get; private set; } = 90.0;

        /// <summary>Height over the runway, m, where cruise takes over.</summary>
        public double accel_height = 300.0;

        /// <summary>Whether to retract the gear once climbing.</summary>
        public bool gear_up = true;

        /// <summary>Seconds the throttle takes to come up to full.</summary>
        public double throttle_ramp_time = 2.0;

        /// <summary>Pitch rate, degrees per second, to rotate with.</summary>
        public double rotate_rate_deg = 3.0;

        /// <summary>Seconds without ground contact that count as airborne.</summary>
        public double liftoff_time = 0.5;

        /// <summary>Speed, m/s, a rejected takeoff hands back at.</summary>
        public double reject_stop_speed = 5.0;

        /// <summary>Speeds, m/s, below which reversers come back to idle reverse, and are stowed.</summary>
        public double reverse_idle_speed = 30.0, reverse_stow_speed = 10.0;

        /// <summary>Climb rate, m/s, and height over the runway, m, past which the gear comes up.</summary>
        public double gear_up_climb = 2.0, gear_up_height = 10.0;

        /// <summary>Set once a rejected takeoff has stopped; the autopilot disengages.</summary>
        internal bool StopRequested;

        /// <summary>Set once the climb reaches the acceleration altitude; cruise takes over.</summary>
        internal bool ClimbComplete;

        /// <summary>Whether this step's control is flown here rather than through the director.</summary>
        internal bool FliesGround => Phase == TakeoffPhase.Roll || Phase == TakeoffPhase.Rotate ||
            Phase == TakeoffPhase.Rejected;

        /// <summary>Horizontal track to fly in the climb, physics space.</summary>
        internal Vector3d DesiredTrack { get; private set; }

        /// <summary>Altitude above sea level the climb goes to before cruise takes over, m.</summary>
        internal double ClimbAltitude => Path.ThresholdAltitude + accel_height;

        /// <summary>Energy share pitch gives to speed in the climb: 2 holds the climb speed with pitch.</summary>
        public double climb_speed_weight = 2.0;

        /// <summary>Runway left ahead, m.</summary>
        public double RunwayRemaining => Path == null ? 0.0 : Math.Max(0.0, Path.Length + Path.AlongTrack);

        private readonly GroundRoll ground = new GroundRoll();
        private readonly Reversers reversers = new Reversers();
        private double throttle;
        private int stage_wait;
        private bool gear_retracted;

        /// <summary>Whether thrust reversers are deployed.</summary>
        public bool ReversersDeployed => reversers.Deploying;

        /// <summary>
        /// The runway end the vessel stands on, lined up with it: within half the width plus 20 m of the centreline
        /// and 20 degrees of its course, between its threshold and the far end. Null when there isn't one.
        /// </summary>
        public static ApproachPath FindRunway(AAVessel vessel)
        {
            var body = vessel.Sim.mainBody?.bodyName;
            if (body == null)
                return null;
            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            var nose = Vector3d.ProjectOnPlane(vessel.ReferenceTransform.up, up);
            ApproachPath best = null;
            var best_cross = double.PositiveInfinity;
            foreach (var airport in RunwayDatabase.OnBody(body))
                foreach (var runway in airport.Runways)
                    foreach (var end in runway.Ends)
                    {
                        if (runway.Opposite(end) == null)
                            continue;
                        var path = new ApproachPath(runway, end, end.GlideslopeDeg);
                        path.Update(vessel);
                        var cross = Math.Abs(path.CrossTrack);
                        if (cross > 0.5 * runway.WidthM + 20.0 || path.AlongTrack > 100.0 ||
                            path.AlongTrack < -path.Length || Vector3d.Angle(nose, path.CourseDirection) > 20.0)
                            continue;
                        if (cross < best_cross)
                        {
                            best = path;
                            best_cross = cross;
                        }
                    }
            return best;
        }

        /// <summary>
        /// Starts the takeoff from <paramref name="path" />. Cruise must be engaged: this flies on top of it.
        /// </summary>
        public void Start(ApproachPath path)
        {
            Deactivate();
            Path = path;
            Activate();
        }

        /// <summary>
        /// The abort key: rejects the takeoff up to the rotate speed, the decision speed here. Past it the takeoff
        /// goes on, as on airliners past V1.
        /// </summary>
        internal void Abort()
        {
            if (Phase != TakeoffPhase.Roll || imodel.surface_v_magnitude >= rotate_speed)
                return;
            Warning = "REJECTED";
            vessel.SetBrakes(true);
            reversers.Find(vessel);
            SetPhase(TakeoffPhase.Rejected);
        }

        protected override void OnActivate()
        {
            StopRequested = false;
            ClimbComplete = false;
            gear_retracted = false;
            Warning = "";
            throttle = 0.0;
            ground.Reset();
            // Set up before launch nothing is lit yet: stage once, as the pilot would
            stage_wait = 0;
            if (!AnyEngineIgnited())
            {
                vessel.Sim.ActivateNextStage();
                stage_wait = 25;
            }
            vessel.SetBrakes(false);
            SetPhase(Path == null ? TakeoffPhase.Off : TakeoffPhase.Roll);
        }

        protected override void OnDeactivate()
        {
            reversers.Stow();
            SetPhase(TakeoffPhase.Off);
        }

        private void SetPhase(TakeoffPhase phase)
        {
            if (phase == Phase)
                return;
            Phase = phase;
            PhaseChanged?.Invoke();
        }

        private bool AnyEngineIgnited()
        {
            foreach (var part in vessel.View.parts)
            {
                if (part == null)
                    continue;
                foreach (var module in part.Modules.Values)
                    if (module is Module_Engine engine && engine.EngineIgnited)
                        return true;
            }
            return false;
        }

        /// <summary>
        /// Flies the ground roll, rotation or rejected takeoff this step. Cruise calls this instead of its own
        /// control while <see cref="FliesGround" /> is set.
        /// </summary>
        internal void ApplyGround(AACtrlState cntrl)
        {
            var dt = Time.fixedDeltaTime;
            Path.Update(vessel);
            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            var speed = imodel.surface_v_magnitude;

            if (stage_wait > 0 && --stage_wait == 0 && !AnyEngineIgnited())
            {
                Warning = "NO ENGINES";
                Deactivate();
                return;
            }

            ground.Measure(vessel, Path, dt);
            ground.Steer(cntrl, Path, speed, dt);
            // Standing on the runway, the centre of mass's height over it is exactly how far the craft reaches below
            if (Phase == TakeoffPhase.Roll && speed < 5.0 && vessel.LandedOrSplashed())
                approach.SetGearOffset(vessel.altitude - Path.SurfaceElevation(vessel));
            if (ground.SteeringLearned)
                LearnedSteeringGain = ground.SteeringGain;
            var pitch = ground.Pitch;
            var pitch_rate = ground.PitchRate;

            switch (Phase)
            {
                case TakeoffPhase.Roll:
                    throttle = Math.Min(1.0, throttle + dt / Math.Max(dt, throttle_ramp_time));
                    cntrl.mainThrottle = (float)throttle;
                    cntrl.pitch = 0.0f;
                    if (speed >= rotate_speed)
                        SetPhase(TakeoffPhase.Rotate);
                    break;

                case TakeoffPhase.Rotate:
                {
                    cntrl.mainThrottle = 1.0f;
                    var target = rotate_pitch_deg * dgr2rad;
                    var rate = rotate_rate_deg * dgr2rad;
                    var desired_pitch_rate = pitch < target - dgr2rad ? rate : Common.Clamp(target - pitch, rate);
                    // Only once the nose is off the ground does the elevator's effect show in the pitch rate. Until
                    // then the gear holds it, and the elevator builds at its full ramp to break it free.
                    var nose_free = pitch > 0.5 * dgr2rad;
                    var pinned = pitch < 0.3 * dgr2rad && pitch_rate < 0.3 * desired_pitch_rate;
                    ground.PitchRateTo(cntrl, desired_pitch_rate, nose_free, pinned, dt);
                    if (!vessel.LandedOrSplashed() && imodel.SecondsSinceGround >= liftoff_time)
                    {
                        // Climb at the climb speed or the lift-off speed if faster, as airliners' speed reference
                        // climb does, so speed in hand isn't zoomed away into a steep climb
                        ClimbSpeed = Math.Max(climb_speed, speed);
                        SetPhase(TakeoffPhase.Climb);
                    }
                    break;
                }

                case TakeoffPhase.Rejected:
                    // Reverse thrust where the engines have it, while tracking straight
                    cntrl.pitch = 0.0f;
                    cntrl.mainThrottle = reversers.Throttle(speed, reverse_idle_speed, reverse_stow_speed,
                        ground.YawRate, Path.CrossTrack, dt);
                    if (speed < reject_stop_speed)
                        StopRequested = true;
                    break;
            }
        }

        /// <summary>
        /// Advances the climb one physics step. Cruise calls this before building its command.
        /// </summary>
        internal void Update()
        {
            if (Phase != TakeoffPhase.Climb)
                return;
            Path.Update(vessel);
            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            // The extended centreline, steered onto the same way as the localizer
            var lookahead = Math.Max(500.0, 20.0 * imodel.surface_v_magnitude);
            var track = Common.Clamp(-Math.Atan(Path.CrossTrack / lookahead), 45.0 * dgr2rad);
            DesiredTrack = (QuaternionD.AngleAxis(track * rad2dgr, up) * Path.CourseDirection).normalized;

            var height = vessel.altitude - Path.ThresholdAltitude;
            if (gear_up && !gear_retracted && height > gear_up_height &&
                Vector3d.Dot(imodel.surface_v, up) > gear_up_climb)
            {
                vessel.Sim.SetActionGroup(KSPActionGroup.Gear, false);
                gear_retracted = true;
            }
            if (vessel.altitude >= ClimbAltitude - 10.0)
                ClimbComplete = true;
        }

        // Diagnostics for FlightRecorder
        internal double DebugSteerEffectiveness => ground.SteeringGain;
        internal double DebugElevatorEffectiveness => ground.ElevatorEffectiveness;
    }
}
