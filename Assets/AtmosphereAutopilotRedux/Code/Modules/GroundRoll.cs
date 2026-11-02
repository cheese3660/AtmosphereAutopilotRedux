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
using KSP.Sim.impl;
using UnityEngine;

using AtmosphereAutopilotRedux.Models;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Modules
{
    /// <summary>
    /// A rate loop that learns its own control effectiveness, for the ground, where the flight model's aerodynamics
    /// don't describe the craft: tyres, nosewheel steering and the nose gear's load all differ from craft to craft.
    /// The rate error asks for an angular acceleration, and the command is that over the effectiveness, which is
    /// estimated from differenced samples so steady moments (thrust asymmetry, the nose gear's load) drop out. An
    /// integral, ramping at a limited rate, takes out what remains.
    /// </summary>
    internal sealed class AdaptiveRateLoop
    {
        public double rate_time = 0.5;
        public double integral_gain = 2.0;
        public double integral_ramp = 1.0;
        public double integral_limit = 1.0;
        public double learn_rate = 0.05;
        public double min_effectiveness, max_effectiveness;

        private readonly double seed;
        private double effectiveness;
        private double integral;
        private double command;
        private double last_rate = double.NaN;
        private readonly Queue<(double acc, double cmd)> history = new Queue<(double, double)>();

        public AdaptiveRateLoop(double seed, double min, double max)
        {
            this.seed = seed;
            min_effectiveness = min;
            max_effectiveness = max;
            Reset();
        }

        public double Effectiveness => effectiveness;

        public void Reset()
        {
            effectiveness = seed;
            integral = 0.0;
            command = 0.0;
            last_rate = double.NaN;
            history.Clear();
        }

        /// <param name="scale">What the effectiveness grows with, such as speed for nosewheel steering.</param>
        /// <param name="learn">Whether the response this step says anything about the effectiveness.</param>
        /// <param name="lag_steps">Physics steps the actuator takes to respond, the span samples are differenced over.</param>
        /// <param name="pinned">Whether something holds the axis still, such as the nose gear before rotation: the
        /// integral then builds at its full ramp instead of in proportion to the error.</param>
        public double Step(double desired_rate, double rate, double scale, bool learn, int lag_steps, double dt,
            bool pinned = false)
        {
            var acc = double.IsNaN(last_rate) ? 0.0 : (rate - last_rate) / dt;
            last_rate = rate;
            history.Enqueue((acc, command));
            while (history.Count > lag_steps + 1)
            {
                var (acc0, cmd0) = history.Dequeue();
                var d_cmd = command - cmd0;
                if (learn && Math.Abs(d_cmd) > 0.005)
                {
                    var estimate = (acc - acc0) / d_cmd / scale;
                    var weight = Math.Min(1.0, d_cmd * d_cmd / 0.0025);
                    effectiveness = Common.Clamp(effectiveness + learn_rate * weight * (estimate - effectiveness),
                        min_effectiveness, max_effectiveness);
                }
            }
            var b = effectiveness * scale;
            var error = desired_rate - rate;
            var step = pinned ? integral_ramp * dt * Math.Sign(error) :
                Common.Clamp(integral_gain * error / rate_time / b * dt, integral_ramp * dt);
            integral = Common.Clamp(integral + step, integral_limit);
            command = Common.Clamp(error / rate_time / b + integral, 1.0);
            return command;
        }
    }

    /// <summary>
    /// Ground steering with a learned steady-state gain. Nosewheel steering turns at a yaw rate of about speed times
    /// the steering angle over the wheelbase, so the yaw rate per unit command per m/s is roughly a constant of the
    /// craft. It is learned from the filtered yaw rate over the filtered command once rolling fast enough for the
    /// tyres to grip, with no differentiation, which made yaw acceleration estimates mostly noise at low speed. KSP2
    /// narrows the nosewheel's angle as speed builds, so learning carries on from small commands too, or a gain
    /// learned early stays too high and steers too little later. (Shrinking the gain whenever the rate fell short
    /// fought the steering's lag and made it weave.) The command is the wanted yaw rate over that gain, feed-forward
    /// only, plus a slow integral on the heading error for steady pulls: craft rock in yaw on their gear, and any
    /// feedback on the measured yaw rate pumped that rocking. It comes in with speed from a standstill, where
    /// steering does little and the heading is noise, and slews no faster than slew_rate.
    /// </summary>
    internal sealed class SteeringLoop
    {
        public double seed = 0.01, min_gain = 0.002, max_gain = 0.2;
        public double learn_speed = 15.0, learn_rate = 0.3, min_learn_command = 0.02;
        public double rate_filter_time = 0.15;
        public double integral_limit = 0.3, slew_rate = 2.0;

        /// <summary>
        /// Integral on the heading error, taken as a yaw rate over integral_time and through the learned gain like
        /// the rest of the command. Added raw, its fixed share of steering turned the craft hard at rollout speeds,
        /// and with the slow yaw response it overshot into a growing weave.
        /// </summary>
        public double heading_integral_gain = 0.1, integral_time = 2.5;

        /// <summary>Lowest speed, m/s, the gain is scaled by, so it can't blow up while barely rolling.</summary>
        public double min_scale_speed = 10.0;

        /// <summary>Speeds, m/s, between which the steering command comes in from nothing to full.</summary>
        public double authority_start = 2.0, authority_full = 10.0;

        private double gain, integral, output, filtered_rate, filtered_output;

        public SteeringLoop() { Reset(); }

        public double Gain => gain;
        public double FilteredRate => filtered_rate;

        public bool Learned { get; private set; }

        public void SetGain(double value)
        {
            if (!double.IsNaN(value) && value > 0.0)
                gain = Common.Clamp(value, min_gain, max_gain);
        }

        public void Reset()
        {
            gain = seed;
            Learned = false;
            integral = output = filtered_rate = filtered_output = 0.0;
        }

        /// <param name="heading_error">How far the heading is off where it should point, rad, for the integral.</param>
        public double Step(double desired_rate, double rate, double heading_error, double speed, double dt)
        {
            var k = 1.0 - Math.Exp(-dt / rate_filter_time);
            filtered_rate += (rate - filtered_rate) * k;
            filtered_output += (output - filtered_output) * k;
            var scale = Math.Max(speed, min_scale_speed);
            if (speed > learn_speed && Math.Abs(filtered_output) > min_learn_command)
            {
                var estimate = filtered_rate / (filtered_output * scale);
                if (estimate > 0.0)
                {
                    gain = Common.Clamp(gain + learn_rate * dt * (estimate - gain), min_gain, max_gain);
                    Learned = true;
                }
            }
            var inverse = 1.0 / (gain * scale);
            integral = Common.Clamp(integral + heading_integral_gain * heading_error / integral_time * inverse * dt,
                integral_limit);
            var command = Common.Clamp(desired_rate * inverse + integral, 1.0);
            command *= Common.Clamp((speed - authority_start) / (authority_full - authority_start), 0.0, 1.0);
            output += Common.Clamp(command - output, slew_rate * dt);
            return output;
        }
    }

    /// <summary>
    /// Steering and pitch on the ground, shared by the takeoff roll and the landing rollout: heading and pitch
    /// attitude of the nose with their rates, feed-forward steering toward the centreline on wheels and rudder, and
    /// the adaptive elevator loop. Ailerons stay neutral, as on airliners in calm air: the gear holds the wings
    /// level, and a wings-level law on sprung gear rocked the craft.
    /// </summary>
    internal sealed class GroundRoll
    {
        /// <summary>Lookahead along the centreline, in seconds of roll, that the steering aims at.</summary>
        public double steer_lookahead_time = 3.0;

        /// <summary>Shortest lookahead, m, so slow rolling doesn't steer hard at small offsets.</summary>
        public double steer_lookahead_min = 60.0;

        /// <summary>Time constant, s, the heading follows the steering's track with.</summary>
        public double heading_time = 2.5;

        /// <summary>Fastest yaw rate, degrees per second, the steering asks for.</summary>
        public double max_yaw_rate_deg = 6.0;

        /// <summary>Sideways acceleration, in g, the steering asks for at most, which caps the yaw rate at speed.</summary>
        public double max_side_acc_g = 0.1;

        private readonly SteeringLoop steering = new SteeringLoop();
        private readonly AdaptiveRateLoop elevator = new AdaptiveRateLoop(1.0, 0.05, 10.0) { integral_ramp = 0.5 };
        private double last_heading = double.NaN, last_pitch = double.NaN;

        /// <summary>Nose heading off the runway course, rad, right positive.</summary>
        public double Heading { get; private set; }

        /// <summary>Nose pitch attitude, rad.</summary>
        public double Pitch { get; private set; }

        public double YawRate { get; private set; }
        public double PitchRate { get; private set; }

        public double SteeringGain => steering.Gain;

        /// <summary>Whether the steering gain was learned this roll rather than left at its seed.</summary>
        public bool SteeringLearned => steering.Learned;

        /// <summary>Starts the steering from a gain learned before, such as on this craft's takeoff roll.</summary>
        public void SeedSteering(double gain) => steering.SetGain(gain);
        public double ElevatorEffectiveness => elevator.Effectiveness;

        public void Reset()
        {
            steering.Reset();
            elevator.Reset();
            last_heading = last_pitch = double.NaN;
        }

        /// <summary>
        /// Reads the nose's heading and pitch, and their rates by differencing. Call once per physics step first.
        /// </summary>
        public void Measure(AAVessel vessel, ApproachPath path, double dt)
        {
            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            var nose = vessel.ReferenceTransform.up;
            var nose_flat = Vector3d.ProjectOnPlane(nose, up);
            Heading = Vector3d.SignedAngle(path.CourseDirection, nose_flat, up) * Mathf.Deg2Rad;
            Pitch = Math.Asin(Common.Clamp(Vector3d.Dot(nose, up), 1.0));
            YawRate = double.IsNaN(last_heading) ? 0.0 : wrap(Heading - last_heading) / dt;
            PitchRate = double.IsNaN(last_pitch) ? 0.0 : (Pitch - last_pitch) / dt;
            last_heading = Heading;
            last_pitch = Pitch;
        }

        /// <summary>
        /// Steers toward a point a lookahead down the centreline: heading time constant to a yaw rate, and the
        /// learned steering for that rate on wheels and rudder alike. Ailerons neutral.
        /// </summary>
        public void Steer(AACtrlState cntrl, ApproachPath path, double speed, double dt)
        {
            var lookahead = Math.Max(steer_lookahead_min, steer_lookahead_time * speed);
            var track = -Math.Atan(path.CrossTrack / lookahead);
            var max_rate = Math.Min(max_yaw_rate_deg * Mathf.Deg2Rad, max_side_acc_g * 9.81 / Math.Max(speed, 1.0));
            var desired_yaw_rate = Common.Clamp((track - Heading) / heading_time, max_rate);
            var steer = steering.Step(desired_yaw_rate, YawRate, track - Heading, speed, dt);
            cntrl.wheelSteer = (float)steer;
            cntrl.yaw = (float)steer;
            cntrl.roll = 0.0f;
        }

        /// <summary>
        /// Elevator for a pitch rate, through the adaptive loop.
        /// </summary>
        /// <param name="free">Whether the pitch rate shows the elevator's effect, rather than the gear holding it.</param>
        /// <param name="pinned">Whether the gear holds the nose against what is asked, so the elevator builds at its
        /// full ramp.</param>
        public void PitchRateTo(AACtrlState cntrl, double desired_rate, bool free, bool pinned, double dt)
        {
            cntrl.pitch = (float)elevator.Step(desired_rate, PitchRate, 1.0, free, 5, dt, pinned);
        }

        private static double wrap(double angle)
        {
            while (angle > Math.PI) angle -= 2.0 * Math.PI;
            while (angle < -Math.PI) angle += 2.0 * Math.PI;
            return angle;
        }
    }

    /// <summary>
    /// The craft's thrust reversers: engines with a mode whose thrust points back along the nose. KSP2 resolves only
    /// the active mode's nozzle transforms, so each mode's are looked up on the part by the names the mode
    /// declares; a mode named for reverse thrust counts when its transforms can't be found.
    /// </summary>
    internal sealed class Reversers
    {
        private struct Reverser
        {
            public Module_Engine Engine;
            public int Normal, Reverse;
        }

        private readonly List<Reverser> reversers = new List<Reverser>();

        public bool Any => reversers.Count > 0;

        /// <summary>Whether they are switched to reverse and done changing mode.</summary>
        public bool Deployed { get; private set; }

        public bool Deploying { get; private set; }

        public void Find(AAVessel vessel)
        {
            level = 0.0;
            last_cross = double.NaN;
            reversers.Clear();
            Deployed = Deploying = false;
            var nose = (Vector3)vessel.ReferenceTransform.up;
            foreach (var part in vessel.View.parts)
            {
                if (part == null)
                    continue;
                foreach (var module in part.Modules.Values)
                {
                    if (!(module is Module_Engine engine) || engine.DataEngine?.engineModes == null ||
                        engine.DataEngine.engineModes.Length < 2 || !engine.EngineIgnited)
                        continue;
                    var data = engine.DataEngine;
                    var normal = data.currentEngineModeIndex;
                    for (var i = 0; i < data.engineModes.Length; i++)
                    {
                        if (i == normal || !is_reverse(part, data.engineModes[i], nose))
                            continue;
                        reversers.Add(new Reverser { Engine = engine, Normal = normal, Reverse = i });
                        break;
                    }
                }
            }
        }

        private static bool is_reverse(PartBehavior part, Data_Engine.EngineMode mode, Vector3 nose)
        {
            var thrust = Vector3.zero;
            var found = false;
            if (mode.ThrustTransformNamesMultipliers != null && mode.ThrustTransformNamesMultipliers.Length > 0)
            {
                foreach (var group in mode.ThrustTransformNamesMultipliers)
                {
                    var t = part.FindModelTransform(group.ThrustTransformName);
                    if (t == null)
                        continue;
                    thrust += -t.forward * group.ThrustTransformMultiplier;
                    found = true;
                }
            }
            else if (!string.IsNullOrEmpty(mode.thrustVectorTransformName))
            {
                foreach (var t in part.FindModelTransforms(mode.thrustVectorTransformName))
                {
                    if (t == null)
                        continue;
                    thrust += -t.forward;
                    found = true;
                }
            }
            if (found && thrust.sqrMagnitude > 1e-6f)
                return Vector3.Dot(thrust.normalized, nose) < -0.2f;
            return (mode.engineID ?? "").IndexOf("reverse", StringComparison.OrdinalIgnoreCase) >= 0 ||
                (mode.EngineDisplayName ?? "").IndexOf("reverse", StringComparison.OrdinalIgnoreCase) >= 0;
        }

        /// <summary>Switches them into reverse; call each step while wanted, it tracks the mode change.</summary>
        public void Deploy()
        {
            if (!Deploying)
            {
                foreach (var r in reversers)
                    if (r.Engine != null)
                        r.Engine.ChangeEngineMode(r.Reverse);
                Deploying = true;
            }
            Deployed = Any;
            foreach (var r in reversers)
                if (r.Engine == null || r.Engine.DataEngine.State == EngineState.ChangingMode ||
                    r.Engine.DataEngine.currentEngineModeIndex != r.Reverse)
                    Deployed = false;
        }

        /// <summary>Seconds full reverse takes to come in, and to go back to idle.</summary>
        public double spool_in_time = 2.0, spool_out_time = 0.5;

        /// <summary>
        /// Yaw rate, degrees per second, and centreline offset, m, past which reverse drops to idle: reverse thrust
        /// weakens directional control, and a weave grew into a wing strike under full reverse.
        /// </summary>
        public double max_yaw_rate_deg = 3.0, max_cross_track = 3.0;

        private double level, last_cross = double.NaN;

        /// <summary>
        /// Throttle for this step's reverse thrust, between idle and full: full above <paramref name="idle_speed" />
        /// while the craft tracks straight, idle otherwise, ramped both ways. Deploys or stows as speed asks.
        /// </summary>
        public float Throttle(double speed, double idle_speed, double stow_speed, double yaw_rate, double cross_track,
            double dt)
        {
            var cross_rate = double.IsNaN(last_cross) ? 0.0 : (cross_track - last_cross) / dt;
            last_cross = cross_track;
            if (!Any || speed <= stow_speed)
            {
                Stow();
                level = 0.0;
                return 0.0f;
            }
            Deploy();
            var straight = Math.Abs(yaw_rate) < max_yaw_rate_deg * Mathf.Deg2Rad &&
                !(Math.Abs(cross_track) > max_cross_track && cross_track * cross_rate > 0.0);
            var target = Deployed && straight && speed > idle_speed ? 1.0 : 0.0;
            level += target > level
                ? Math.Min(target - level, dt / Math.Max(dt, spool_in_time))
                : Math.Max(target - level, -dt / Math.Max(dt, spool_out_time));
            return (float)level;
        }

        /// <summary>Switches them back to forward thrust.</summary>
        public void Stow()
        {
            if (!Deploying)
                return;
            foreach (var r in reversers)
                if (r.Engine != null)
                    r.Engine.ChangeEngineMode(r.Normal);
            Deploying = Deployed = false;
        }
    }
}
