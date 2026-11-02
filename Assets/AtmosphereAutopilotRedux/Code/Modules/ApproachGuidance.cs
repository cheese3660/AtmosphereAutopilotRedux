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
using KSP.Sim;
using UnityEngine;

using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{
    public enum ApproachPhase
    {
        Off,
        /// <summary>Flying the generated fixes onto the final approach; the localizer arms on the last leg.</summary>
        Transition,
        /// <summary>Localizer and glideslope armed; cruise still flies its own modes.</summary>
        Armed,
        /// <summary>Localizer captured, glideslope armed.</summary>
        Localizer,
        /// <summary>Localizer and glideslope captured.</summary>
        Glideslope,
        /// <summary>Autoland below land_height: on the glideslope and localizer, flare and retard armed.</summary>
        Land,
        /// <summary>Autoland flaring to touchdown, throttle retarding to idle.</summary>
        Flare,
        /// <summary>Autoland on the runway: nose down, centreline, brakes and reversers, to a stop.</summary>
        Rollout,
        /// <summary>Climbing away on the runway course at full thrust, before cruise takes back over.</summary>
        GoAround
    }

    /// <summary>
    /// Flies a runway approach on top of cruise control, the way an airliner's approach mode arms on top of its
    /// heading and altitude modes. With fixes on it first flies a short transition it generates itself: an initial
    /// approach fix abeam the final when coming from the side or behind, then an intermediate fix on the extended
    /// centreline, descending to the approach altitude, so the localizer is always met at a planned angle and the
    /// glideslope from below at the final approach fix. With fixes off cruise keeps flying what is set until the
    /// localizer and then the glideslope capture. Either way speed comes down to the approach speed by the final
    /// approach fix. Going around climbs out on the runway course and hands back to cruise, with the cruise tab's
    /// altitude and speed as the missed approach targets.
    /// </summary>
    public sealed class ApproachGuidance : AutopilotModule
    {
        internal ApproachGuidance(AAVessel v)
            : base(v, "Approach guidance")
        { }

        private FlightModel imodel;
        private ProgradeThrustController thrust_c;

        public override void InitializeDependencies(Dictionary<Type, AutopilotModule> modules)
        {
            imodel = modules[typeof(FlightModel)] as FlightModel;
            thrust_c = modules[typeof(ProgradeThrustController)] as ProgradeThrustController;
        }

        public ApproachPath Path { get; private set; }

        public ApproachPhase Phase { get; private set; } = ApproachPhase.Off;

        /// <summary>
        /// Raised when the phase changes, including on arming and when the approach ends.
        /// </summary>
        public event Action PhaseChanged;

        /// <summary>
        /// What the pilot should know about the approach, such as it not being stabilized; empty when nothing.
        /// </summary>
        public string Warning { get; private set; } = "";

        /// <summary>
        /// Speed to fly the approach at, m/s.
        /// </summary>
        public double approach_speed = 85.0;

        /// <summary>
        /// Height over the threshold, m, where the autopilot hands over to the pilot.
        /// </summary>
        public double minimums = 60.0;

        /// <summary>
        /// Whether to fly the generated fixes onto the final approach, or arm on top of what cruise flies.
        /// </summary>
        public bool fly_fixes = true;

        /// <summary>
        /// Whether to land: flare, retard and roll out to a stop, instead of handing over at minimums.
        /// </summary>
        public bool autoland = false;

        /// <summary>Radio height, m, the autoland's LAND mode engages below.</summary>
        public double land_height = 120.0;

        /// <summary>
        /// Flare law: sink = (height + sink x flare_lead) / flare_time + touchdown_sink. The lead looks ahead through
        /// the lag of building lift, so craft slow to respond still arrive gently.
        /// </summary>
        public double flare_time = 2.0, flare_lead = 1.5, touchdown_sink = 0.7;

        /// <summary>Lowest and highest radio heights, m, the flare starts between.</summary>
        public double flare_min_height = 6.0, flare_max_height = 40.0;

        /// <summary>Seconds the throttle takes to retard to idle in the flare.</summary>
        public double retard_time = 2.0;

        /// <summary>Pitch rate, degrees per second, the nose is lowered with after touchdown.</summary>
        public double derotate_rate_deg = 2.0;

        /// <summary>Seconds the nose must have stopped coming down for the nose wheel to count as down.</summary>
        public double nose_down_time = 0.5;

        /// <summary>Speed, m/s, the rollout hands back at.</summary>
        public double rollout_stop_speed = 5.0;

        /// <summary>Speeds, m/s, below which reversers come back to idle reverse, and are stowed.</summary>
        public double reverse_idle_speed = 30.0, reverse_stow_speed = 10.0;

        /// <summary>
        /// Height over the threshold, m, the transition descends to and the glideslope is met from.
        /// </summary>
        public double approach_height = 600.0;

        /// <summary>
        /// Steepest bank, degrees, the approach turns with, as on airliners.
        /// </summary>
        public double bank_limit_deg = 25.0;

        /// <summary>
        /// Lookahead along the centreline, in seconds of flight, that the localizer steers toward. Never shorter
        /// than the turn radius at bank_limit_deg, or the turn onto it can't keep up.
        /// </summary>
        public double localizer_lookahead_time = 20.0;

        /// <summary>
        /// Steepest angle, degrees, the localizer steers back onto the centreline at.
        /// </summary>
        public double intercept_angle_deg = 45.0;

        /// <summary>
        /// Cross-track distance, m, the localizer captures at whatever the heading; at least 5 degrees of the
        /// lookahead, so a vessel lined up but drifting parallel still captures.
        /// </summary>
        public double capture_distance = 30.0;

        /// <summary>
        /// Smallest cross-track distance, m, the glideslope may capture within. Further out it is the localizer's
        /// full scale, 2.5 degrees either side seen from the far end, as on an ILS.
        /// </summary>
        public double glideslope_capture_cross = 300.0;

        /// <summary>
        /// Above this many times the approach speed the transition slows down first, heading out toward where the
        /// intermediate fix will be, and only then lays out its fixes: at jet speeds a 25 degree turn is tens of
        /// kilometres across.
        /// </summary>
        public double transition_speed_ratio = 1.15;

        /// <summary>
        /// Seconds of flight at the approach speed beyond the intermediate fix that the slowdown heads for, so the
        /// fixes laid out at its end have room to turn onto.
        /// </summary>
        public double slowdown_aim_time = 30.0;

        /// <summary>
        /// Vertical speed per metre off the glideslope, 1/s.
        /// </summary>
        public double glideslope_Kp = 0.3;

        /// <summary>
        /// Steepest descent, as a multiple of the glideslope's, used to get back down onto it.
        /// </summary>
        public double max_path_steepness = 2.5;

        /// <summary>
        /// Seconds of flight on the centreline, between the end of the turn onto it and the final approach fix.
        /// </summary>
        public double fix_settle_time = 45.0;

        /// <summary>
        /// Seconds of flight added to the initial approach fix's offset, so the turn at it ends with time to spare.
        /// </summary>
        public double fix_offset_time = 10.0;

        /// <summary>
        /// Half-angle, degrees, of the cone behind the intermediate fix from which the transition goes straight to
        /// it, without an initial approach fix.
        /// </summary>
        public double straight_in_cone_deg = 30.0;

        /// <summary>
        /// Seconds of flight at the approach speed the deceleration plans to end before the final approach fix.
        /// </summary>
        public double decel_margin_time = 5.0;

        /// <summary>
        /// Height over the threshold, m, by which the approach should be stabilized.
        /// </summary>
        public double stabilized_height = 300.0;

        /// <summary>
        /// Height over the threshold, m, a go-around climbs to on the runway course before cruise takes over.
        /// </summary>
        public double go_around_height = 300.0;

        /// <summary>
        /// Energy share pitch gives to speed in a go-around: 2 holds speed with pitch while the throttle climbs.
        /// </summary>
        public double go_around_speed_weight = 2.0;

        /// <summary>Set when minimums are reached; the autopilot hands over to fly-by-wire.</summary>
        internal bool HandOverRequested;

        /// <summary>Set when a go-around has climbed out; cruise takes back over on the runway course.</summary>
        internal bool GoAroundComplete;

        /// <summary>Set once the rollout has stopped; the autopilot disengages.</summary>
        internal bool StopRequested;

        /// <summary>
        /// Share of the throttle at the start of the flare still allowed this step, retarding from 1 to 0, or NaN.
        /// Cruise scales it by the throttle it had then, so the throttle only comes down: from 1 it let the speed
        /// hold's call for more thrust, as the flare shallowed the path, through for a moment.
        /// </summary>
        internal double ThrottleLimit { get; private set; } = double.NaN;

        /// <summary>Steering gain to start the rollout from, learned on this craft's takeoff roll, or NaN.</summary>
        internal double RolloutSteeringSeed = double.NaN;

        /// <summary>Measured height of the centre of mass over the gear, m, or NaN.</summary>
        public double GearOffset => radio.Measured ? radio.LowestPointOffset : double.NaN;

        /// <summary>Takes the gear's offset below the centre of mass as measured on a runway.</summary>
        public void SetGearOffset(double offset) => radio.SetMeasured(offset);

        /// <summary>Whether this step's control is flown here on the ground rather than through the director.</summary>
        internal bool FliesGround => Phase == ApproachPhase.Rollout;

        /// <summary>Height of the gear over the ground below, m, or NaN when not measured.</summary>
        public double RadioHeight { get; private set; } = double.NaN;

        /// <summary>Whether thrust reversers are deployed.</summary>
        public bool ReversersDeployed => reversers.Deploying;

        private readonly RadioAltimeter radio = new RadioAltimeter();
        private readonly GroundRoll ground = new GroundRoll();
        private readonly Reversers reversers = new Reversers();
        private double flare_height, retard_elapsed, rollout_elapsed, nose_still_time;
        private bool nose_down;

        /// <summary>Horizontal track to fly while the approach steers, physics space.</summary>
        internal Vector3d DesiredTrack { get; private set; }

        /// <summary>Vertical speed to fly while on the glideslope, m/s.</summary>
        internal double DesiredVerticalSpeed { get; private set; }

        /// <summary>Speed the approach flies, m/s, or NaN while the cruise speed stands.</summary>
        internal double SpeedOverride { get; private set; } = double.NaN;

        /// <summary>Altitude above sea level to hold before the glideslope, m, or NaN while cruise's stands.</summary>
        internal double AltitudeOverride => fly_fixes && Path != null &&
            (Phase == ApproachPhase.Transition || Phase == ApproachPhase.Armed || Phase == ApproachPhase.Localizer)
                ? Path.ThresholdAltitude + approach_height
                : double.NaN;

        /// <summary>Altitude above sea level a go-around climbs to before cruise takes back over, m.</summary>
        internal double GoAroundAltitude => Path.ThresholdAltitude + go_around_height;

        internal bool LateralActive => Phase == ApproachPhase.Transition || Phase == ApproachPhase.Localizer ||
            Phase == ApproachPhase.Glideslope || Phase == ApproachPhase.Land || Phase == ApproachPhase.Flare ||
            Phase == ApproachPhase.GoAround;

        /// <summary>Whether the vertical path is the approach's own: the glideslope, then the flare.</summary>
        internal bool VerticalActive => Phase == ApproachPhase.Glideslope || Phase == ApproachPhase.Land ||
            Phase == ApproachPhase.Flare;

        /// <summary>Slowing down before the transition's fixes are laid out.</summary>
        public bool SlowingForTransition => Phase == ApproachPhase.Transition && !fixes_built;

        /// <summary>Sideways acceleration, in g, the approach's bank limit allows.</summary>
        internal double TurnLimitG => Math.Tan(bank_limit_deg * dgr2rad);

        /// <summary>Glideslope altitude minus the vessel's, m: positive below it.</summary>
        public double GlideslopeError { get; private set; }

        /// <summary>Height over the threshold, m.</summary>
        public double HeightAboveThreshold { get; private set; }

        /// <summary>Name of the fix the transition flies to next, or null outside it.</summary>
        public string NextFixName => Phase == ApproachPhase.Transition && next_fix < fixes.Count ? fixes[next_fix].Name : null;

        /// <summary>Distance to the next fix, m.</summary>
        public double NextFixDistance { get; private set; }

        /// <summary>Whether the localizer is armed now: in the transition only on the leg into the last fix.</summary>
        public bool LocalizerArmed => Phase == ApproachPhase.Armed ||
            (Phase == ApproachPhase.Transition && next_fix == fixes.Count - 1);

        private struct Fix
        {
            public string Name;
            public double Along, Cross;
        }

        // The transition, generated on the first step after arming
        private readonly List<Fix> fixes = new List<Fix>();
        private int next_fix;
        private bool fixes_built;
        private double faf_along;

        private bool decelerating;
        private bool go_around_gear_up;
        private double last_check_height = double.NaN;
        private bool stabilized_checked;
        private bool glideslope_missed_checked;
        private double go_around_speed;

        /// <summary>
        /// Arms the approach to <paramref name="path" />. Cruise must be engaged: this flies on top of it.
        /// </summary>
        /// <param name="on_final">Resume an approach that had already captured the localizer, as after loading a
        /// save: no fixes, no slowdown, the localizer and glideslope recapture where the craft already is.</param>
        public void Arm(ApproachPath path, bool on_final = false)
        {
            Deactivate();
            Path = path;
            resume_on_final = on_final;
            Activate();
        }

        private bool resume_on_final;

        /// <summary>
        /// The abort key: goes around, as airliners' TOGA switches do.
        /// </summary>
        internal void Abort()
        {
            if (Active)
                GoAround();
        }

        /// <summary>
        /// Goes around from a captured approach, up to touchdown; does nothing otherwise.
        /// </summary>
        public void GoAround()
        {
            if (Phase != ApproachPhase.Localizer && Phase != ApproachPhase.Glideslope && Phase != ApproachPhase.Land &&
                Phase != ApproachPhase.Flare)
                return;
            ThrottleLimit = double.NaN;
            go_around_speed = Math.Max(approach_speed, imodel.surface_v_magnitude);
            go_around_gear_up = false;
            Warning = "";
            SetPhase(ApproachPhase.GoAround);
        }

        protected override void OnActivate()
        {
            decelerating = false;
            stabilized_checked = false;
            last_check_height = double.NaN;
            glideslope_missed_checked = false;
            HandOverRequested = false;
            GoAroundComplete = false;
            StopRequested = false;
            ThrottleLimit = double.NaN;
            RadioHeight = double.NaN;
            fixes_built = false;
            fixes.Clear();
            next_fix = 0;
            Warning = "";
            if (resume_on_final)
            {
                fixes_built = true;
                decelerating = true;
                resume_on_final = false;
                SetPhase(Path == null ? ApproachPhase.Off : ApproachPhase.Armed);
                return;
            }
            SetPhase(Path == null ? ApproachPhase.Off : fly_fixes ? ApproachPhase.Transition : ApproachPhase.Armed);
        }

        protected override void OnDeactivate()
        {
            SpeedOverride = double.NaN;
            ThrottleLimit = double.NaN;
            reversers.Stow();
            SetPhase(ApproachPhase.Off);
        }

        private void SetPhase(ApproachPhase phase)
        {
            if (phase == Phase)
                return;
            Phase = phase;
            PhaseChanged?.Invoke();
        }

        /// <summary>
        /// Advances the approach one physics step. Cruise calls this before building its command.
        /// </summary>
        internal void Update()
        {
            if (Path == null || Phase == ApproachPhase.Off)
                return;
            Path.Update(vessel);

            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            var velocity = imodel.surface_v;
            var speed = imodel.surface_v_magnitude;
            var hor_vel = Vector3d.ProjectOnPlane(velocity, up);
            var hor_speed = Math.Max(1.0, hor_vel.magnitude);
            var vert_speed = Vector3d.Dot(velocity, up);
            var g = imodel.gravity_acc.magnitude;
            var tan_gs = Math.Tan(Path.GlideslopeDeg * dgr2rad);
            HeightAboveThreshold = vessel.altitude - Path.ThresholdAltitude;
            GlideslopeError = Path.PathAltitude - vessel.altitude;

            if (Phase == ApproachPhase.GoAround)
            {
                DesiredTrack = Path.CourseDirection;
                SpeedOverride = go_around_speed;
                // Gear up once climbing away, as on airliners at positive rate
                if (!go_around_gear_up && vert_speed > 2.0 && HeightAboveThreshold > 10.0)
                {
                    vessel.Sim.SetActionGroup(KSPActionGroup.Gear, false);
                    go_around_gear_up = true;
                }
                if (vessel.altitude >= GoAroundAltitude - 10.0)
                    GoAroundComplete = true;
                return;
            }

            var turn_radius = hor_speed * hor_speed / Math.Max(0.1, g * Math.Tan(bank_limit_deg * dgr2rad));
            var lookahead = Math.Max(Math.Max(hor_speed * localizer_lookahead_time, turn_radius), 500.0);
            if (!fixes_built && (!fly_fixes || speed <= transition_speed_ratio * approach_speed))
                build_fixes(Math.Max(approach_speed, speed), g, tan_gs, up, hor_vel);
            if (Phase == ApproachPhase.Transition && fixes_built)
                update_transition(up, hor_vel, turn_radius);
            else if (Phase == ApproachPhase.Transition)
                update_slowdown(up, hor_vel, g, tan_gs, turn_radius);
            update_localizer(up, hor_vel, turn_radius, lookahead);
            update_glideslope(hor_speed, vert_speed, tan_gs);
            update_speed(speed, tan_gs, up);
            check_progress(speed);
            if (autoland)
                update_landing(vert_speed, hor_speed * tan_gs);
            else if (Phase == ApproachPhase.Glideslope && HeightAboveThreshold <= minimums)
            {
                // At minimums an approach without autoland is the pilot's to land. The autothrottle stays on at the
                // approach speed, as it does on airliners when the autopilot disconnects.
                thrust_c.setpoint_mps = (float)approach_speed;
                thrust_c.spd_control_enabled = true;
                Warning = "MINIMUMS";
                HandOverRequested = true;
            }
        }

        #region Landing

        // LAND below land_height on the radio altimeter, calibrated again then for the gear; the flare starts at the
        // height its law asks for the glideslope's sink there, so it begins without a step; the throttle retards to
        // idle over retard_time; touchdown starts the rollout
        private void update_landing(double vert_speed, double path_sink)
        {
            if (Phase == ApproachPhase.Glideslope && double.IsNaN(radio.LowestPointOffset))
                radio.Calibrate(vessel);
            if (Phase != ApproachPhase.Glideslope && Phase != ApproachPhase.Land && Phase != ApproachPhase.Flare)
                return;
            RadioHeight = radio.Height(vessel, Path);
            var height = RadioHeight;

            if (Phase == ApproachPhase.Glideslope && height <= land_height)
            {
                radio.Calibrate(vessel);
                if (vessel.Sim.GetActionGroupState(KSPActionGroup.Gear) == KSPActionGroupState.False)
                    Warning = "GEAR UP";
                SetPhase(ApproachPhase.Land);
            }
            if (Phase == ApproachPhase.Land)
            {
                flare_height = Common.Clamp((flare_time + flare_lead) * (Math.Max(-vert_speed, path_sink) - touchdown_sink),
                    flare_min_height, flare_max_height);
                if (height <= flare_height)
                {
                    retard_elapsed = 0.0;
                    SetPhase(ApproachPhase.Flare);
                }
            }
            if (Phase == ApproachPhase.Flare)
            {
                var predicted = Math.Max(0.0, height + vert_speed * flare_lead);
                DesiredVerticalSpeed = Math.Max(-predicted / flare_time - touchdown_sink, -path_sink);
                retard_elapsed += Time.fixedDeltaTime;
                ThrottleLimit = Math.Max(0.0, 1.0 - retard_elapsed / Math.Max(Time.fixedDeltaTime, retard_time));
            }
        }

        /// <summary>
        /// Starts the rollout once the wheels touch. Cruise calls this before anything else each step: on the ground
        /// it flies nothing itself, so the touchdown must be caught first.
        /// </summary>
        internal void CheckTouchdown()
        {
            if ((Phase == ApproachPhase.Land || Phase == ApproachPhase.Flare) && vessel.LandedOrSplashed())
                begin_rollout();
        }

        private void begin_rollout()
        {
            // The altimeter should read zero at touchdown: whatever it read is the offset's error
            if (!double.IsNaN(RadioHeight) && Math.Abs(RadioHeight) < 15.0 && !double.IsNaN(radio.LowestPointOffset))
                radio.SetMeasured(radio.LowestPointOffset + RadioHeight);
            ground.Reset();
            if (!double.IsNaN(RolloutSteeringSeed))
                ground.SeedSteering(RolloutSteeringSeed);
            reversers.Find(vessel);
            rollout_elapsed = nose_still_time = 0.0;
            nose_down = false;
            ThrottleLimit = 0.0;
            SetPhase(ApproachPhase.Rollout);
        }

        /// <summary>
        /// Flies the rollout this step: centreline steering, the nose lowered onto its wheel, then brakes, with
        /// reverse thrust where the engines have it, full until reverse_idle_speed, then idle, stowed near a stop.
        /// Cruise calls this instead of its own control while <see cref="FliesGround" /> is set.
        /// </summary>
        internal void ApplyGround(AACtrlState cntrl)
        {
            var dt = Time.fixedDeltaTime;
            Path.Update(vessel);
            var speed = imodel.surface_v_magnitude;
            rollout_elapsed += dt;
            // Bounced back into the air before the nose came down: flare again rather than fly ground laws aloft
            if (!nose_down && !vessel.LandedOrSplashed() && imodel.SecondsSinceGround > 0.5)
            {
                reversers.Stow();
                ThrottleLimit = 0.0;
                SetPhase(ApproachPhase.Flare);
                return;
            }
            ground.Measure(vessel, Path, dt);
            ground.Steer(cntrl, Path, speed, dt);

            if (!nose_down)
            {
                // Lower the nose until it stops coming down: the nose wheel is on the runway
                var rate = -derotate_rate_deg * dgr2rad;
                ground.PitchRateTo(cntrl, rate, true, false, dt);
                nose_still_time = ground.PitchRate > 0.3 * rate ? nose_still_time + dt : 0.0;
                if ((rollout_elapsed > 1.0 && nose_still_time >= nose_down_time) || rollout_elapsed > 8.0)
                {
                    nose_down = true;
                    vessel.SetBrakes(true);
                }
            }
            else
                cntrl.pitch = 0.0f;

            // Reverse thrust where the engines have it, ramped, and idle unless tracking straight
            cntrl.mainThrottle = reversers.Throttle(speed, reverse_idle_speed, reverse_stow_speed, ground.YawRate,
                Path.CrossTrack, dt);
            if (speed < rollout_stop_speed && nose_down)
                StopRequested = true;
        }

        #endregion

        #region Transition

        // Localizer capture lead for a track angle to the centreline: the lookahead's share up to the intercept
        // angle, then the sideways distance the turn from the vessel's angle down to it covers, R (cos s - cos a)
        private double localizer_lead(double angle, double lookahead, double turn_radius)
        {
            var steep = Math.Min(angle, intercept_angle_deg * dgr2rad);
            return lookahead * Math.Tan(steep) + turn_radius * (Math.Cos(steep) - Math.Cos(angle));
        }

        // Distance to slow from speed to the approach speed at the deceleration the speed reference plans with,
        // plus a margin
        private double decel_distance(double speed)
        {
            var decel = thrust_c.reference_acc_share * Math.Max(0.5, -thrust_c.drag_estimate);
            return Math.Max(0.0, speed * speed - approach_speed * approach_speed) / (2.0 * decel) +
                decel_margin_time * approach_speed;
        }

        // The final approach fix is where the glideslope meets the approach altitude. The intermediate fix is out
        // on the centreline by the lead a right-angle localizer intercept takes plus fix_settle_time of flight, and
        // at least far enough to slow to the approach speed by the final approach fix. From within
        // straight_in_cone_deg behind it the transition goes straight there; otherwise first to an initial approach
        // fix abeam it on the vessel's side, far enough out for the turn there to finish before the localizer
        // intercept starts. Sized for the faster of the approach speed and the speed now.
        private void build_fixes(double plan_speed, double g, double tan_gs, Vector3d up, Vector3d hor_vel)
        {
            fixes_built = true;
            faf_along = Math.Max(0.0, approach_height - Path.End.ThresholdCrossingHeightM) / Math.Max(1e-3, tan_gs);
            if (!fly_fixes)
                return;
            var radius = plan_speed * plan_speed / Math.Max(0.1, g * Math.Tan(bank_limit_deg * dgr2rad));
            var lookahead = Math.Max(Math.Max(plan_speed * localizer_lookahead_time, radius), 500.0);
            var lead = localizer_lead(Math.PI / 2.0, lookahead, radius);
            var slow_down = decel_distance(plan_speed);
            var if_along = faf_along + Math.Max(lead + fix_settle_time * plan_speed, slow_down);
            var along = Path.AlongTrack;
            var cross = Path.CrossTrack;

            // Already set up on the final, inside the intermediate fix, with room to slow down and no more than a
            // little above the approach altitude: just arm
            var heading_error = Vector3d.SignedAngle(Path.CourseDirection, hor_vel, up) * dgr2rad;
            if (along > faf_along && along < if_along && along - faf_along >= slow_down &&
                HeightAboveThreshold - approach_height < 50.0 &&
                Math.Abs(cross) <= localizer_lead(Math.Abs(heading_error), lookahead, radius) &&
                Math.Abs(heading_error) < Math.PI / 2.0)
            {
                SetPhase(ApproachPhase.Armed);
                return;
            }
            var behind = along > if_along &&
                Math.Abs(Math.Atan2(cross, along - if_along)) < straight_in_cone_deg * dgr2rad;
            if (!behind)
            {
                var side = cross >= 0.0 ? 1.0 : -1.0;
                fixes.Add(new Fix { Name = "IAF", Along = if_along, Cross = side * (lead + radius + fix_offset_time * plan_speed) });
            }
            fixes.Add(new Fix { Name = "IF", Along = if_along, Cross = 0.0 });
            next_fix = 0;
        }

        // Slowing down, heads for a point a little beyond where the intermediate fix will be at the approach speed,
        // so the slowdown is flown toward the runway instead of wherever cruise was going
        private void update_slowdown(Vector3d up, Vector3d hor_vel, double g, double tan_gs, double turn_radius)
        {
            var radius = approach_speed * approach_speed / Math.Max(0.1, g * Math.Tan(bank_limit_deg * dgr2rad));
            var lookahead = Math.Max(Math.Max(approach_speed * localizer_lookahead_time, radius), 500.0);
            var faf = Math.Max(0.0, approach_height - Path.End.ThresholdCrossingHeightM) / Math.Max(1e-3, tan_gs);
            var aim_along = faf + localizer_lead(Math.PI / 2.0, lookahead, radius) +
                (fix_settle_time + slowdown_aim_time) * approach_speed;
            steer_to(Path.SurfaceDirection(aim_along, 0.0), up, hor_vel, turn_radius);
        }

        // Direct to a point; one behind and inside the turning circle can't be reached by turning toward it, which
        // only circles it, so fly on until it is outside, then turn back. Returns the angle off it, degrees.
        private double steer_to(Vector3d point, Vector3d up, Vector3d hor_vel, double turn_radius)
        {
            var to_point = Vector3d.ProjectOnPlane(point - up, up).normalized;
            NextFixDistance = Path.SurfaceDistance(up, point);
            var off = Vector3d.Angle(hor_vel, to_point);
            DesiredTrack = off > 90.0 && NextFixDistance < 2.0 * turn_radius ? hor_vel.normalized : to_point;
            return off;
        }

        // Direct to each fix, flying by it: the next one is taken a turn's lead early, once the vessel heads for it
        private void update_transition(Vector3d up, Vector3d hor_vel, double turn_radius)
        {
            if (next_fix >= fixes.Count)
            {
                SetPhase(ApproachPhase.Localizer);
                return;
            }
            var fix = fixes[next_fix];
            var fix_dir = Path.SurfaceDirection(fix.Along, fix.Cross);
            var to_fix = Vector3d.ProjectOnPlane(fix_dir - up, up).normalized;
            var heading_to_fix = steer_to(fix_dir, up, hor_vel, turn_radius) < 60.0;
            if (next_fix + 1 < fixes.Count)
            {
                var next = fixes[next_fix + 1];
                var leg = Vector3d.ProjectOnPlane(Path.SurfaceDirection(next.Along, next.Cross) - fix_dir, up).normalized;
                var turn = Vector3d.Angle(to_fix, leg) * dgr2rad;
                if (heading_to_fix && NextFixDistance <= Math.Max(300.0, turn_radius * Math.Tan(turn / 2.0)))
                    next_fix++;
            }
            // Over the intermediate fix without a capture: the localizer takes over from here
            else if (NextFixDistance < 300.0)
                SetPhase(ApproachPhase.Localizer);
        }

        #endregion

        // The localizer steers toward a point a lookahead down the centreline, so the track back onto it is
        // atan(cross track / lookahead), no steeper than intercept_angle_deg. It captures at the lead distance that
        // turn needs.
        private void update_localizer(Vector3d up, Vector3d hor_vel, double turn_radius, double lookahead)
        {
            var cross = Path.CrossTrack;
            if (Phase != ApproachPhase.Transition)
            {
                var track = Common.Clamp(-Math.Atan(cross / lookahead), intercept_angle_deg * dgr2rad);
                DesiredTrack = (QuaternionD.AngleAxis(track * rad2dgr, up) * Path.CourseDirection).normalized;
            }

            if (!LocalizerArmed || Path.AlongTrack <= 0.0)
                return;
            var heading_error = Vector3d.SignedAngle(Path.CourseDirection, hor_vel, up) * dgr2rad;
            var angle = Math.Abs(heading_error);
            if (angle >= 150.0 * dgr2rad)
                return;     // flying away from the runway
            var closing = cross * Math.Sin(heading_error) < 0.0;
            if (Math.Abs(cross) < Math.Max(capture_distance, lookahead * Math.Tan(5.0 * dgr2rad)) ||
                (closing && Math.Abs(cross) <= localizer_lead(angle, lookahead, turn_radius)))
                SetPhase(ApproachPhase.Localizer);
        }

        // The glideslope's vertical speed, plus a pull toward it of glideslope_Kp per metre off, never climbing back
        // up to it and never steeper than max_path_steepness times it. It captures once the localizer is close and
        // that pull, unclamped, asks for no more than the vessel is doing: from below in level flight just before
        // the path, from above only while descending more steeply than the path, as on airliners.
        private void update_glideslope(double hor_speed, double vert_speed, double tan_gs)
        {
            var path_vs = -hor_speed * tan_gs;
            var error = GlideslopeError;
            var law = path_vs + glideslope_Kp * error;
            DesiredVerticalSpeed = Math.Min(0.0, Math.Max(law, max_path_steepness * path_vs));

            var capture_cross = Math.Max(glideslope_capture_cross,
                (Math.Max(0.0, Path.AlongTrack) + Path.Length) * Math.Tan(2.5 * dgr2rad));
            if (Phase != ApproachPhase.Localizer || Path.AlongTrack <= 0.0 ||
                Math.Abs(Path.CrossTrack) > capture_cross)
                return;
            if ((error >= 0.0 && law <= vert_speed + 0.1) || (error < 0.0 && law >= vert_speed - 0.1))
            {
                // Gear down at glideslope capture, as crews do around intercept, well before LAND reads the radio
                // altimeter over it
                vessel.Sim.SetActionGroup(KSPActionGroup.Gear, true);
                SetPhase(ApproachPhase.Glideslope);
            }
        }

        // Slows to the approach speed by the final approach fix, as an airliner's computed deceleration point does:
        // the distance it takes against what is left to fly there, through the remaining fixes in the transition.
        // Already inside it, or before a transition that has to slow first, it slows straight away.
        private void update_speed(double speed, double tan_gs, Vector3d up)
        {
            if (!decelerating)
            {
                // Without fixes the glideslope is met from the height flown now
                var faf = fly_fixes ? faf_along :
                    (HeightAboveThreshold - Path.End.ThresholdCrossingHeightM) / Math.Max(1e-3, tan_gs);
                if (Phase == ApproachPhase.Glideslope || speed <= approach_speed || SlowingForTransition ||
                    remaining_to(faf, up) <= decel_distance(speed))
                    decelerating = true;
            }
            SpeedOverride = decelerating ? approach_speed : double.NaN;
        }

        private double remaining_to(double faf, Vector3d up)
        {
            if (Phase != ApproachPhase.Transition || next_fix >= fixes.Count || !fixes_built)
                return Path.AlongTrack - faf;
            var at = up;
            var total = 0.0;
            for (var i = next_fix; i < fixes.Count; i++)
            {
                var fix = Path.SurfaceDirection(fixes[i].Along, fixes[i].Cross);
                total += Path.SurfaceDistance(at, fix);
                at = fix;
            }
            return total + fixes[fixes.Count - 1].Along - faf;
        }

        // Airliners must be on speed, on the localizer and on the glideslope by 1000 ft over the field. Flying the
        // fixes, the glideslope should also have captured by the final approach fix.
        private void check_progress(double speed)
        {
            if (!glideslope_missed_checked && fly_fixes && Phase == ApproachPhase.Localizer &&
                Path.AlongTrack < faf_along - 1000.0)
            {
                glideslope_missed_checked = true;
                Warning = "G/S NOT CAPTURED";
            }
            // Checked as the approach descends through stabilized_height, not when it starts below it, as after
            // loading a save on short final
            var above = last_check_height > stabilized_height;
            last_check_height = HeightAboveThreshold;
            if (stabilized_checked || Phase != ApproachPhase.Glideslope || HeightAboveThreshold > stabilized_height ||
                !above)
                return;
            stabilized_checked = true;
            var off_speed = Math.Abs(speed - approach_speed) > 10.0;
            var off_path = Math.Abs(GlideslopeError) > Math.Max(15.0, 0.1 * HeightAboveThreshold);
            var off_course = Math.Abs(Path.CrossTrack) > Math.Max(30.0, 0.5 * Path.Runway.WidthM);
            if (off_speed || off_path || off_course)
                Warning = "NOT STABILIZED";
        }
    }
}
