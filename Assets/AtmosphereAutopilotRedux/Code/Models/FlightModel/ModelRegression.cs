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
using System.Threading;
using UnityEngine;
using System.IO;

using AtmosphereAutopilotRedux.MathUtils;
using AtmosphereAutopilotRedux.Modules;
using AtmosphereAutopilotRedux.Runtime;
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.Models
{

    using Vector = VectorArray.Vector;
    using TrainerTask = OnlineLinTrainer.LinApproxTask;

    public sealed partial class FlightModel : AutopilotModule
    {
        //public Vector3d model_acc = Vector3d.zero;

        private LinApprox pitch_aero_torque_model = new LinApprox(2);
        private LinApprox pitch_aero_torque_model_gen = new LinApprox(2);
        private LinApprox roll_aero_torque_model = new LinApprox(4);
        private LinApprox roll_aero_torque_model_gen = new LinApprox(4);
        private LinApprox yaw_aero_torque_model = new LinApprox(2);
        private LinApprox yaw_aero_torque_model_gen = new LinApprox(2);
        private LinApprox pitch_lift_model = new LinApprox(2);
        private LinApprox yaw_lift_model = new LinApprox(2);

        private TrainerTask pitch_aero_torque_task = new TrainerTask();
        private TrainerTask pitch_aero_torque_task_gen = new TrainerTask();
        private TrainerTask roll_aero_torque_task = new TrainerTask();
        private TrainerTask roll_aero_torque_task_gen = new TrainerTask();
        private TrainerTask yaw_aero_torque_task = new TrainerTask();
        private TrainerTask yaw_aero_torque_task_gen = new TrainerTask();
        private TrainerTask pitch_lift_task = new TrainerTask();
        private TrainerTask yaw_lift_task = new TrainerTask();

        private const int IMM_BUF_SIZE = 10;

        private OnlineLinTrainer pitch_trainer, roll_trainer, yaw_trainer;
        private OnlineLinTrainer pitch_lift_trainer, yaw_lift_trainer;

        private bool lin_models_seeded = false;

        #region Persistence

        // The learned models, in the order they are saved. A new order needs a new save format.
        private LinApprox[] persisted_models => new[]
        {
            pitch_aero_torque_model, pitch_aero_torque_model_gen,
            roll_aero_torque_model, roll_aero_torque_model_gen,
            yaw_aero_torque_model, yaw_aero_torque_model_gen,
            pitch_lift_model, yaw_lift_model,
        };

        /// <summary>
        /// Whether the models hold something learned (or restored), not just the seed.
        /// </summary>
        internal bool HasLearned { get; private set; }

        /// <summary>
        /// Copies of the learned model coefficients, for the save.
        /// </summary>
        internal double[][] ExportModels()
        {
            var models = persisted_models;
            var result = new double[models.Length][];
            for (var i = 0; i < models.Length; i++)
                lock (models[i].tpars)
                    result[i] = (double[])models[i].tpars.Clone();
            return result;
        }

        /// <summary>
        /// Starts the models from saved coefficients instead of the seed, which after a load asked for a quarter radian
        /// of AoA at Mach 3: its lift slope is zero. False, changing nothing, when the data doesn't fit.
        /// </summary>
        internal bool ImportModels(double[][] data)
        {
            var models = persisted_models;
            if (data == null || data.Length != models.Length)
                return false;
            for (var i = 0; i < models.Length; i++)
            {
                if (data[i] == null || data[i].Length != models[i].tpars.Length)
                    return false;
                foreach (var value in data[i])
                    if (double.IsNaN(value) || double.IsInfinity(value))
                        return false;
            }
            for (var i = 0; i < models.Length; i++)
            {
                lock (models[i].tpars)
                {
                    data[i].CopyTo(models[i].tpars, 0);
                    data[i].CopyTo(models[i].pars, 0);
                }
                models[i].signalUpdated();
            }
            lin_models_seeded = true;
            HasLearned = true;
            return true;
        }

        #endregion

        // Initial dumb aero values. Upstream did this in the constructor, before control surfaces and mass were known,
        // and never signalled the models, so the seed never took effect. We seed after the first full parts scan.
        private void seed_lin_models()
        {
            if (HasControlSurfaces)
            {
                seed_lin_model(pitch_aero_torque_model, 2, 10.0);
                seed_lin_model(pitch_aero_torque_model_gen, 2, 10.0);
                seed_lin_model(roll_aero_torque_model, 3, 10.0);
                seed_lin_model(roll_aero_torque_model_gen, 3, 10.0);
                seed_lin_model(yaw_aero_torque_model, 2, 10.0);
                seed_lin_model(yaw_aero_torque_model_gen, 2, 10.0);
            }
            seed_lin_model(pitch_lift_model, 2, sum_mass);
            seed_lin_model(yaw_lift_model, 2, sum_mass);
            lin_models_seeded = true;
        }

        private static void seed_lin_model(LinApprox model, int index, double value)
        {
            lock (model.tpars)
                model.tpars[index] = value;
            model.signalUpdated();
        }

        private void initialize_lin_tainers()
        {
            //  Initialize pitch rotation trainer
            pitch_aero_torque_task.linmodel = pitch_aero_torque_model;
            pitch_aero_torque_task.base_gen_weight = 0.0001f;
            pitch_aero_torque_task.linear_time_decay = 0.005f;
            pitch_aero_torque_task.nonlin_time_decay = 0.01f;
            pitch_aero_torque_task.validator = (a, b) => pitch_validator(PITCH, pitch_aero_torque_model, a, b);

            pitch_aero_torque_task_gen.linmodel = pitch_aero_torque_model_gen;
            pitch_aero_torque_task_gen.base_gen_weight = 0.5f;
            pitch_aero_torque_task_gen.linear_time_decay = 0.005f;
            pitch_aero_torque_task_gen.nonlin_time_decay = 0.01f;
            pitch_aero_torque_task_gen.validator = (a, b) => pitch_validator(PITCH, pitch_aero_torque_model_gen, a, b);

            pitch_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, pitch_input_method, pitch_output_method, pitch_aero_torque_task_gen, pitch_aero_torque_task);
            pitch_trainer.max_value_decay = 0.001f;
            pitch_trainer.linear_err_criteria = 0.01f;
            pitch_trainer.nonlin_trigger = 100;
            pitch_trainer.nonlin_cutoff_time = 500;

            //  Initialize roll rotation trainer
            roll_aero_torque_task.linmodel = roll_aero_torque_model;
            roll_aero_torque_task.base_gen_weight = 0.001f;
            roll_aero_torque_task.linear_time_decay = 0.005f;
            roll_aero_torque_task.nonlin_time_decay = 0.01f;
            roll_aero_torque_task.validator = (a, b) => roll_validator(roll_aero_torque_model, a, b);

            roll_aero_torque_task_gen.linmodel = roll_aero_torque_model_gen;
            roll_aero_torque_task_gen.base_gen_weight = 0.5f;
            roll_aero_torque_task_gen.linear_time_decay = 0.005f;
            roll_aero_torque_task_gen.nonlin_time_decay = 0.01f;
            roll_aero_torque_task_gen.validator = (a, b) => roll_validator(roll_aero_torque_model_gen, a, b);

            roll_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05, 0.05, 0.05 }, 
                new int[] { 15, 15, 15, 15 }, roll_input_method, roll_output_method, roll_aero_torque_task_gen, roll_aero_torque_task);
            roll_trainer.max_value_decay = 0.001f;
            roll_trainer.linear_err_criteria = 0.02f;
            roll_trainer.nonlin_trigger = 100;
            roll_trainer.nonlin_cutoff_time = 500;

            //  Initialize yaw rotation trainer
            yaw_aero_torque_task.linmodel = yaw_aero_torque_model;
            yaw_aero_torque_task.base_gen_weight = 0.0001f;
            yaw_aero_torque_task.linear_time_decay = 0.005f;
            yaw_aero_torque_task.nonlin_time_decay = 0.01f;
            yaw_aero_torque_task.validator = (a, b) => pitch_validator(YAW, yaw_aero_torque_model, a, b);

            yaw_aero_torque_task_gen.linmodel = yaw_aero_torque_model_gen;
            yaw_aero_torque_task_gen.base_gen_weight = 0.5f;
            yaw_aero_torque_task_gen.linear_time_decay = 0.005f;
            yaw_aero_torque_task_gen.nonlin_time_decay = 0.01f;
            yaw_aero_torque_task_gen.validator = (a, b) => pitch_validator(YAW, yaw_aero_torque_model_gen, a, b);

            yaw_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, yaw_input_method, yaw_output_method, yaw_aero_torque_task_gen, yaw_aero_torque_task);
            yaw_trainer.max_value_decay = 0.001f;
            yaw_trainer.linear_err_criteria = 0.01f;
            yaw_trainer.nonlin_trigger = 100;
            yaw_trainer.nonlin_cutoff_time = 500;

            //  Initialize pitch lift trainer
            pitch_lift_task.linmodel = pitch_lift_model;
            pitch_lift_task.base_gen_weight = 2.0f;
            pitch_lift_task.linear_time_decay = 0.002f;
            pitch_lift_task.nonlin_time_decay = 0.005f;
            pitch_lift_task.validator = (a, b) => pitch_lift_validator(pitch_lift_model, a, b);

            pitch_lift_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, pitch_lift_input_method, pitch_lift_output_method, pitch_lift_task);
            pitch_lift_trainer.max_value_decay = 0.0005f;
            pitch_lift_trainer.linear_err_criteria = 0.0005f;
            pitch_lift_trainer.nonlin_trigger = 100;
            pitch_lift_trainer.nonlin_cutoff_time = 500;

            //  Initialize yaw lift trainer
            yaw_lift_task.linmodel = yaw_lift_model;
            yaw_lift_task.base_gen_weight = 2.0f;
            yaw_lift_task.linear_time_decay = 0.002f;
            yaw_lift_task.nonlin_time_decay = 0.005f;
            yaw_lift_task.validator = (a, b) => pitch_lift_validator(yaw_lift_model, a, b);

            yaw_lift_trainer = new OnlineLinTrainer(IMM_BUF_SIZE, new double[] { 0.01, 0.05 },
                new int[] { 20, 20 }, yaw_lift_input_method, yaw_lift_output_method, yaw_lift_task);
            yaw_lift_trainer.max_value_decay = 0.0005f;
            yaw_lift_trainer.linear_err_criteria = 0.001f;
            yaw_lift_trainer.nonlin_trigger = 100;
            yaw_lift_trainer.nonlin_cutoff_time = 500;
        }

        /// <summary>
        /// Current dynamic pressure = density * air_speed^2
        /// </summary>
        public double dyn_pressure = 1.0;

        // Trainer input methods
        private void pitch_input_method(Vector v)
        {
            v[0] = aoa_buf[PITCH].GetFromTail(1);   // we need AoA from previous physics frame
            // KSP2 computes aero forces from the deflection before this step's surface update, so the acceleration
            // we just measured answers to the surface position one step older than in KSP1
            v[1] = csurf_buf[PITCH].GetFromTail(aero_delay_steps);
        }

        private double pitch_output_method()
        {
            return (TorqueAcc(PITCH) -
                (reaction_torque[PITCH] * input_buf[PITCH].GetLast() + engines_torque_principal[PITCH] +
                get_rcs_torque(PITCH, input_buf[PITCH].GetLast())) / MOI[PITCH]) * MOI[PITCH] / dyn_pressure * 1e2;
        }

        private bool pitch_validator(int axis, LinApprox linmodel, bool[] changed, bool[] reassigned)
        {
            var ret = false;
            changed.CopyTo(reassigned, 0);
            // Terms from the game's lift model are held, and the rest is fitted around them
            var stiffness = held_analytic_stiffness(axis);
            if (!double.IsNaN(stiffness))
            {
                linmodel.tpars[1] = stiffness;
                reassigned[0] = false;
                ret = true;
            }
            else if (!changed[0] && linmodel.tpars[1] == 0.0)
            {
                // AoA hasn't moved enough to fit its term yet. The game's wing-only value has the right sign and
                // about a quarter of the size, which beats assuming neutral stability until the first manoeuvre.
                var estimate = analytic_stiffness_estimate(axis);
                if (!double.IsNaN(estimate))
                {
                    linmodel.tpars[1] = estimate;
                    ret = true;
                }
            }
            var analytic = held_analytic_authority(axis);
            if (HasControlSurfaces && !double.IsNaN(analytic))
            {
                linmodel.tpars[2] = analytic;
                reassigned[1] = false;
                ret = true;
            }
            else if (HasControlSurfaces)
            {
                if (linmodel.tpars[2] < 1e-6)       // authority can't be negative or zero
                {
                    linmodel.tpars[2] = MOI[0] / 1000.0;
                    reassigned[1] = false;
                    ret = true;
                }
            }
            else
                linmodel.tpars[2] = 0.0;
            return ret;
        }

        private void roll_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].GetFromTail(1);
            v[1] = angular_v_buf[ROLL].GetFromTail(1) / vessel.srfSpeed * 1e2;
            v[2] = csurf_buf[ROLL].GetFromTail(aero_delay_steps);
            v[3] = csurf_buf[YAW].GetFromTail(aero_delay_steps);
        }

        private double roll_output_method()
        {
            return (TorqueAcc(ROLL) -
                (reaction_torque[ROLL] * input_buf[ROLL].GetLast() + engines_torque_principal[ROLL] +
                get_rcs_torque(ROLL, input_buf[ROLL].GetLast())) / MOI[ROLL]) * MOI[ROLL] / dyn_pressure * 1e2;
        }

        private bool roll_validator(LinApprox linmodel, bool[] changed, bool[] reassigned)
        {
            var ret = false;
            changed.CopyTo(reassigned, 0);
            var damping = held_analytic_roll_damping();
            if (!double.IsNaN(damping))
            {
                // Roll damping comes from the game's wing lift
                linmodel.tpars[2] = damping;
                reassigned[1] = false;
                ret = true;
            }
            else if (linmodel.tpars[2] > 0.0)      // friction must kill rotation
            {
                linmodel.tpars[2] = 0.0;
                reassigned[1] = false;
                ret = true;
            }
            // Terms from the game's lift model are held, and the rest is fitted around them
            var stiffness = held_analytic_stiffness(ROLL);
            if (!double.IsNaN(stiffness))
            {
                linmodel.tpars[1] = stiffness;
                reassigned[0] = false;
                ret = true;
            }
            var analytic = held_analytic_authority(ROLL);
            if (HasControlSurfaces && !double.IsNaN(analytic))
            {
                linmodel.tpars[3] = analytic;
                reassigned[2] = false;
                ret = true;
            }
            else if (HasControlSurfaces)
            {
                if (linmodel.tpars[3] < 1e-6)      // authority can't be negative or zero
                {
                    linmodel.tpars[3] = MOI[ROLL] / 1000.0;
                    reassigned[2] = false;
                    ret = true;
                }
            }
            else
                linmodel.tpars[3] = 0.0;
            return ret;
        }

        private void yaw_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].GetFromTail(1);
            v[1] = csurf_buf[YAW].GetFromTail(aero_delay_steps);
        }

        private double yaw_output_method()
        {
            return (TorqueAcc(YAW) -
                (reaction_torque[YAW] * input_buf[YAW].GetLast() + engines_torque_principal[YAW] +
                get_rcs_torque(YAW, input_buf[YAW].GetLast())) / MOI[YAW]) * MOI[YAW] / dyn_pressure * 1e2;
        }

        private void pitch_lift_input_method(Vector v)
        {
            v[0] = aoa_buf[PITCH].GetFromTail(1);
            v[1] = csurf_buf[PITCH].GetFromTail(aero_delay_steps);
        }

        private double pitch_lift_output_method()
        {
            return lift_acc / dyn_pressure * 1e3 * sum_mass;        // we approximate lift force, not acceleration
        }

        private bool pitch_lift_validator(LinApprox linmodel, bool[] changed, bool[] reassigned)
        {
            // plane airframe must provide positive lift derivative
            double sign = Math.Sign(Math.Abs(AoA(PITCH)) - 90.0 * dgr2rad);
            if (linmodel.tpars[1] * sign > 0.0)
            {
                changed.CopyTo(reassigned, 0);
                linmodel.tpars[1] = -sign * sum_mass / 10.0;
                reassigned[0] = false;
                return true;
            }
            return false;
        }

        private void yaw_lift_input_method(Vector v)
        {
            v[0] = aoa_buf[YAW].GetFromTail(1);
            v[1] = csurf_buf[YAW].GetFromTail(aero_delay_steps);
        }

        private double yaw_lift_output_method()
        {
            return slide_acc / dyn_pressure * 1e3 * sum_mass;
        }

        // Training inputs updating
        private void update_training_inputs()
        {
            var dt = (int)Math.Max(1, Math.Round(Time.fixedDeltaTime * 100.0f));
            if (!vessel.LandedOrSplashed() && dyn_pressure >= 60.0 && csurf_buf[PITCH].Size > aero_delay_steps)
            {
                // Trained in flight from here on, so not the seed any more
                HasLearned = true;
                pitch_trainer.UpdateState(dt);
                pitch_trainer.min_output_value = (float)(0.5 * MOI[PITCH] / dyn_pressure * 1e2);

                roll_trainer.UpdateState(dt);
                roll_trainer.min_output_value = (float)(1.0 * MOI[ROLL] / dyn_pressure * 1e2);

                yaw_trainer.UpdateState(dt);
                yaw_trainer.min_output_value = (float)(0.5 * MOI[YAW] / dyn_pressure * 1e2);

                pitch_lift_trainer.UpdateState(dt);
                pitch_lift_trainer.min_output_value = (float)(10.0 / dyn_pressure * 1e3 * sum_mass);

                yaw_lift_trainer.UpdateState(dt);
                yaw_lift_trainer.min_output_value = (float)(2.0 / dyn_pressure * 1e3 * sum_mass);
            }
        }

        // Training methods
        //[AutoGuiAttr("pitch_cpu", false)]
        private int pitch_cpu = 0;

        //[AutoGuiAttr("roll_cpu", false)]
        private int roll_cpu = 5;

        //[AutoGuiAttr("yaw_cpu", false)]
        private int yaw_cpu = 0;

        //[AutoGuiAttr("pitch_lift_cpu", false)]
        private int pitch_lift_cpu = 0;

        //[AutoGuiAttr("yaw_lift_cpu", false)]
        private int yaw_lift_cpu = 5;

        //[AutoGuiAttr("CPU per update", true)]
        private int CPU_TIME_FOR_FIXEDUPDATE = 5;

        private const int TORQUE_DESCEND_COST = 10;
        private const int LIFT_DESCEND_COST = 10;

        private void update_cpu()
        {
            Interlocked.Add(ref pitch_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref roll_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref pitch_lift_cpu, CPU_TIME_FOR_FIXEDUPDATE);
            Interlocked.Add(ref yaw_lift_cpu, CPU_TIME_FOR_FIXEDUPDATE);
        }

        private bool train_pitch_ann()
        {
            if (pitch_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref pitch_cpu, -TORQUE_DESCEND_COST);
                pitch_trainer.Train();
                return true;
            }
            return false;
        }

        private bool train_roll_ann()
        {
            if (roll_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref roll_cpu, -TORQUE_DESCEND_COST);
                roll_trainer.Train();
                return true;
            }
            return false;
        }

        private bool train_yaw_ann()
        {
            if (yaw_cpu >= TORQUE_DESCEND_COST)
            {
                Interlocked.Add(ref yaw_cpu, -TORQUE_DESCEND_COST);
                yaw_trainer.Train();
                return true;
            }
            return false;
        }

        private bool train_pitch_lift()
        {
            if (pitch_lift_cpu >= LIFT_DESCEND_COST)
            {
                Interlocked.Add(ref pitch_lift_cpu, -LIFT_DESCEND_COST);
                pitch_lift_trainer.Train();
                return true;
            }
            return false;
        }

        private bool train_yaw_lift()
        {
            if (yaw_lift_cpu >= LIFT_DESCEND_COST)
            {
                Interlocked.Add(ref yaw_lift_cpu, -LIFT_DESCEND_COST);
                yaw_lift_trainer.Train();
                return true;
            }
            return false;
        }

        // Diagnostics for FlightRecorder: learned aero torque coefficients per axis
        internal double[] DebugAeroPars(int axis) =>
            axis == PITCH ? pitch_aero_torque_model.pars : axis == ROLL ? roll_aero_torque_model.pars : yaw_aero_torque_model.pars;

        internal double[] DebugAeroParsGen(int axis) =>
            axis == PITCH ? pitch_aero_torque_model_gen.pars : axis == ROLL ? roll_aero_torque_model_gen.pars : yaw_aero_torque_model_gen.pars;

        private void update_model_acc()
        {
            pitch_aero_torque_model.update_from_training();
            pitch_aero_torque_model_gen.update_from_training();
            roll_aero_torque_model.update_from_training();
            roll_aero_torque_model_gen.update_from_training();
            yaw_aero_torque_model.update_from_training();
            yaw_aero_torque_model_gen.update_from_training();
            pitch_lift_model.update_from_training();
            yaw_lift_model.update_from_training();
        }
    }
}
