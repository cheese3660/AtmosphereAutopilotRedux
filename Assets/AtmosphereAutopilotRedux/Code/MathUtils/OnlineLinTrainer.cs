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
using AtmosphereAutopilotRedux.Utils;

namespace AtmosphereAutopilotRedux.MathUtils
{

    using Vector = VectorArray.Vector;

    public class OnlineLinTrainer
    {
        // Buffers, that will be updated by calls from Unity event cycle

        // Immediate buffer
        private CircularBufferAA<Vector> imm_buf_inputs;
        private VectorArray imm_buf_vectors;
        private CircularBufferAA<double> imm_buf_outputs;

        // Buffers for linear regression

        // Immediate buffer
        private CircularBufferAA<Vector> imm_training_inputs;
        private VectorArray imm_training_vectors;

        private CircularBufferAA<double> imm_training_outputs;
        // Gradient buffer
        private VectorArray grad_training_vectors;

        private CircularBufferAA<GenStruct> grad_training;
        // Generalization buffer is being used as generality augmentor
        private CircularBufferAA<GenStruct>[] gen_buffers;
        // Error weight buffers
        //List<double> imm_error_weights = new List<double>();
        //List<double> grad_error_weights = new List<double>();
        //List<double> gen_error_weights = new List<double>();

        // Views to adapt inputs for Approximator
        private ListView<Vector> input_view;
        private ListView<double> output_view;
        private ListView<double> weight_view;

        // flag of buffers update
        private volatile bool updated = false;

        // time is used for generalization measures aging
        private int cur_time = 0;

        // models to train
        public readonly List<LinApproxTask> tasks = new List<LinApproxTask>();
        //public readonly LinApprox linmodel, genmodel;
        private readonly int input_count;

        private struct GenStruct
        {
            public Vector coord;
            public double val;
            public int birth;
            public GenStruct(double value, int time, Vector v)
            {
                val = value;
                birth = time;
                coord = v;
            }
        }

        public OnlineLinTrainer(int imm_buf_size, double[] gen_triggers, int[] gen_buf_sizes,
            Action<Vector> input_method, Func<double> output_method, params LinApproxTask[] tasks)
        {
            foreach (var t in tasks)
                this.tasks.Add(t);
            input_count = tasks[0].linmodel.input_count;
            linearity_check_task = tasks[0];
            // Immediate and gradient buffer initialization
            imm_buf_inputs = new CircularBufferAA<Vector>(imm_buf_size, true);
            imm_buf_vectors = new VectorArray(input_count, imm_buf_size);
            imm_training_inputs = new CircularBufferAA<Vector>(imm_buf_size, true);
            imm_training_vectors = new VectorArray(input_count, imm_buf_size);
            grad_training = new CircularBufferAA<GenStruct>(imm_buf_size / 2, true);
            grad_training_vectors = new VectorArray(input_count, imm_buf_size / 2);
            imm_buf_outputs = new CircularBufferAA<double>(imm_buf_size, true);
            imm_training_outputs = new CircularBufferAA<double>(imm_buf_size, true);
            // bind vectors in circular buffers to vector arrays
            for (var i = 0; i < imm_buf_size; i++)
            {
                imm_buf_inputs[i] = imm_buf_vectors[i];
                imm_training_inputs[i] = imm_training_vectors[i];
                if (i < imm_buf_size / 2)
                    grad_training[i] = new GenStruct(0.0, 0, grad_training_vectors[i]);
            }
            // Generalization space initialization
            this.gen_triggers = gen_triggers;
            gen_space_size = 0;
            gen_buffers = new CircularBufferAA<GenStruct>[input_count];
            for (var i = 0; i < input_count; i++)
            {
                gen_buffers[i] = new CircularBufferAA<GenStruct>(gen_buf_sizes[i], true);
                gen_space_size += gen_buf_sizes[i];
                var gen_array = new VectorArray(input_count, gen_buf_sizes[i]);
                for (var j = 0; j < gen_buf_sizes[i]; j++)
                    gen_buffers[i][j] = new GenStruct(0.0, 0, gen_array[j]);
            }
            // Delegates assignment
            input_update_dlg = input_method;
            output_update_dlg = output_method;
            // Preallocate buffers for matrix operations in linear approximator
            foreach (var t in tasks)
                t.linmodel.preallocate((int)(imm_buf_size * 1.5) + gen_buf_sizes.Sum());
            // Misc
            inputs_changed = new bool[input_count];
            nothing_changed = new bool[input_count];
            reassigned = new bool[input_count];
        }

        /// <summary>
        /// Approximator to train on this particular data set with weight parameters
        /// </summary>
        public class LinApproxTask
        {
            public LinApprox linmodel;

            // [AutoGuiAttr("base gen weight", true, "G7")]
            public volatile float base_gen_weight;

            // [AutoGuiAttr("linear decay factor", true, "G6")]
            public volatile float linear_time_decay;

            // [AutoGuiAttr("nonlinear decay factor", true, "G6")]
            public volatile float nonlin_time_decay;

            // [AutoGuiAttr("zero immediate", true)]
            public volatile bool zero_immediate = true;

            public delegate bool critiqueDlg(bool[] inputs_changed, bool[] values_overwritten);

            public critiqueDlg validator;
        }

        #region DataSourceThread

        //[AutoGuiAttr("time_elapsed", false)]
        protected int last_time_elapsed = 0;

        // [AutoGuiAttr("iteration_time", false)]
        protected volatile int iteration_time = 0;

        public void UpdateState(int time_elapsed)
        {
            update_immediate_buffer();
            last_time_elapsed = time_elapsed;
            updated = true;
        }

        private Action<Vector> input_update_dlg;                // new state input getter
        private Func<double> output_update_dlg;                 // new state output getter

        private void update_immediate_buffer()
        {
            lock (imm_buf_inputs)
            {
                // inputs
                var input = imm_buf_inputs.GetWritingCell();
                input_update_dlg(input);
                imm_buf_inputs.Put(input);
                // outputs
                imm_buf_outputs.Put(output_update_dlg());
            }
        }

        #endregion

        #region TrainingThread

        private bool[] nothing_changed;
        private bool[] reassigned;

        public void Train()
        {
            if (updated)
            {
                update_train_buf();
                updated = false;
                if (cur_time > time_reset)
                    reset_time();
                update_weights();
                if (input_view == null)
                    create_views();
                if (input_view != null)
                {
                    if (input_view.Count > 0)
                    {
                        reset_view_caches();
                        update_singularity(input_view);
                        for (var i = 0; i < tasks.Count; i++)
                        {
                            var linmodel = tasks[i].linmodel;
                            set_parameters(tasks[i]);
                            linmodel.weighted_lsqr(input_view, output_view, weight_view, inputs_changed);
                            var redo = tasks[i].validator(inputs_changed, reassigned);
                            if (redo)
                                linmodel.weighted_lsqr(input_view, output_view, weight_view, reassigned);
                            if (tasks[i].zero_immediate)
                                linmodel.weighted_lsqr(imm_training_inputs, imm_training_outputs, imm_error_weights, nothing_changed);
                            linmodel.signalUpdated();
                        }
                        check_linearity();
                        clear_old_records();
                    }
                }
            }
        }

        // [AutoGuiAttr("min_output_value", false, "G6")]
        public volatile float min_output_value = 0.01f;

        // [AutoGuiAttr("max_output_value", false, "G6")]
        protected double max_output_value = 0.01;         // maximum absolute value of model output reached in past

        //[AutoGuiAttr("inputs_changed", false)]
        private readonly bool[] inputs_changed;         // flags that input has changed

        //int added_to_imm = 0;                   // how many inputs where added to training imm_buf during last Train() call

        // [AutoGuiAttr("max value decay", true, "G6")]
        public volatile float max_value_decay = 0.001f;

        // [AutoGuiAttr("gradient_sensitivity", true, "G6")]
        public volatile float gradient_sensitivity = 0.05f;

        private void update_train_buf()
        {
            // There are new inputs from Unity thread
            lock (imm_buf_inputs)
            {
                var count = imm_buf_inputs.Size;
                iteration_time = count * last_time_elapsed;
                //added_to_imm = count;
                while (count > 0)
                {
                    cur_time += last_time_elapsed;
                    if (imm_training_inputs.Size == imm_training_inputs.Capacity)       // if we'll be overwriting immediate buffer
                    {
                        // let's update gradient buffers if needed
                        var cur_output = imm_training_outputs[1];
                        var prev_output = imm_training_outputs[0];
                        if (Math.Abs((prev_output - cur_output) / max_output_value) >= gradient_sensitivity)
                        {
                            // we need to put the output we'll be overwriting soon into gradient buffer
                            var wrt = grad_training.GetWritingCell().coord;
                            var v2write = imm_training_inputs[0];
                            v2write.DeepCopy(wrt);
                            grad_training.Put(new GenStruct(imm_training_outputs[0], cur_time - imm_training_inputs.Size * last_time_elapsed, wrt));
                        }
                        // let's update generalization buffers
                        var new_coord = imm_training_inputs[0];
                        for (var j = 0; j < input_count; j++)
                        {
                            var cur_add_time = cur_time - imm_training_inputs.Size * last_time_elapsed;
                            var prev_cell = gen_buffers[j].GetLast();
                            if (gen_buffers[j].Size == 0 ||
                                Math.Abs(prev_cell.coord[j] - new_coord[j]) >
                                (gen_buffers[j].Size / (double)gen_buffers[j].Capacity) * gen_triggers[j])
                            {
                                var cell = gen_buffers[j].GetWritingCell();
                                new_coord.DeepCopy(cell.coord);
                                gen_buffers[j].Put(new GenStruct(prev_output, cur_add_time, cell.coord));
                            }
                        }
                    }
                    var writing_cell = imm_training_inputs.GetWritingCell();
                    imm_buf_inputs.Get().DeepCopy(writing_cell);
                    imm_training_inputs.Put(writing_cell);
                    var new_output = imm_buf_outputs.Get();
                    max_output_value = max_output_value * (1.0 - max_value_decay * last_time_elapsed);
                    max_output_value = Math.Max(Math.Max(max_output_value, Math.Abs(new_output)), min_output_value);
                    imm_training_outputs.Put(new_output);
                    count--;
                }
            }
        }

        public double[] gen_triggers;

        private ListView<GenStruct> gen_list_view;

        private ListSelector<GenStruct, Vector> gen_inputs_selector;
        private ListSelector<GenStruct, double> gen_output_selector;

        private ListSelector<GenStruct, Vector> grad_input_selector;
        private ListSelector<GenStruct, double> grad_output_selector;

        private List<int> imm_weights_dummys = new List<int>();
        private ListSelector<int, double> imm_error_weights;
        private ListSelector<GenStruct, double> gen_error_weights;
        private ListSelector<GenStruct, double> grad_error_weights;

        private void create_views()
        {
            gen_list_view = new ListView<GenStruct>(gen_buffers);

            gen_inputs_selector = new ListSelector<GenStruct, Vector>(gen_list_view, cv => cv.coord);
            gen_output_selector = new ListSelector<GenStruct, double>(gen_list_view, cv => cv.val);
            grad_input_selector = new ListSelector<GenStruct, Vector>(grad_training, gs => gs.coord);
            grad_output_selector = new ListSelector<GenStruct, double>(grad_training, gs => gs.val);

            imm_error_weights = new ListSelector<int, double>(imm_weights_dummys, imm_weight_func);
            gen_error_weights = new ListSelector<GenStruct, double>(gen_list_view, gen_weight_func);
            grad_error_weights = new ListSelector<GenStruct, double>(grad_training, grad_weight_func);

            input_view = new ListView<Vector>(imm_training_inputs, grad_input_selector, gen_inputs_selector);
            output_view = new ListView<double>(imm_training_outputs, grad_output_selector, gen_output_selector);
            weight_view = new ListView<double>(imm_error_weights, grad_error_weights, gen_error_weights);
        }

        private void reset_view_caches()
        {
            input_view.reset_cached_index();
            output_view.reset_cached_index();
            weight_view.reset_cached_index();
        }

        // Age decay factors for weights of inputs

        private float weight_time_decay = 0.02f;

        private float base_gen_weight = 0.1f;

        //float linear_time_decay = 0.02f;

        //float nonlin_time_decay = 0.5f;

        // [AutoGuiAttr("nonlin_cutoff_time", true)]
        public volatile int nonlin_cutoff_time = 1000;

        // [AutoGuiAttr("linear criteria", true, "G6")]
        public volatile float linear_err_criteria = 0.05f;

        // [AutoGuiAttr("linear", false)]
        public volatile bool linear = false;

        // [AutoGuiAttr("linear_param", false, "G6")]
        public volatile float linear_param;

        // [AutoGuiAttr("nonlin_time", false)]
        private int nonlin_time = 0;

        /// <summary>
        /// What approximator to use as linearity judge
        /// </summary>
        public LinApproxTask linearity_check_task;

        private void check_linearity()
        {
            if (imm_training_inputs.Size > 0)
            {
                var sum_error = 0.0;
                var linmodel = linearity_check_task.linmodel;
                for (var i = 0; i < imm_training_inputs.Size; i++)
                {
                    var input = imm_training_inputs[i];
                    var true_output = imm_training_outputs[i];
                    var lin_output = linmodel.eval_training(input);
                    var scaled_err = (lin_output - true_output) / max_output_value;
                    sum_error += Math.Abs(scaled_err);
                }
                //for (int i = 0; i < grad_training.Size; i++)
                //{
                //    Vector input = grad_training[i].coord;
                //    double true_output = grad_training[i].val;
                //    double lin_output = linmodel.eval_training(input);
                //    double scaled_err = (lin_output - true_output) / max_output_value;
                //    sum_error += Math.Abs(scaled_err);
                //}
                //Vector input = imm_training_inputs.getLast();
                //double true_output = imm_training_outputs.getLast();
                //double lin_output = linmodel.eval_training(input);
                //sum_error = Math.Abs((lin_output - true_output) / max_output_value);
                linear_param = (float)(sum_error / (double)(imm_training_inputs.Size));// + grad_training.Size));
                linear = linear_param < linear_err_criteria;
                //linear_param = (float)sum_error;
                //linear = sum_error < linear_err_criteria;
                if (linear)
                    nonlin_time = 0;
                else
                    nonlin_time = Math.Min(1000, nonlin_time + iteration_time);
            }
        }

        private void set_parameters(LinApproxTask task)
        {
            base_gen_weight = task.base_gen_weight;
            if (linear)
                weight_time_decay = task.linear_time_decay;
            else
                weight_time_decay = task.nonlin_time_decay;
        }

        // [AutoGuiAttr("gen_space_fill_k", false, "G6")]
        public volatile float gen_space_fill_k;

        // [AutoGuiAttr("nonlin_trigger", true)]
        public int nonlin_trigger = 100;

        private int gen_space_size;

        private double imm_weight_func(int index)
        {
            var birth = cur_time - (imm_training_inputs.Size - index - 1) * last_time_elapsed;
            return getAgeWeight(birth);
        }

        private double gen_weight_func(GenStruct record)
        {
            return base_gen_weight * getAgeWeight(record.birth);
        }

        private double grad_weight_func(GenStruct record)
        {
            return getAgeWeight(record.birth);
        }

        //double popul_ratio = 1.0;

        private void update_weights()
        {
            // Immediate buffer error weights
            while (imm_training_inputs.Size > imm_weights_dummys.Count)
                imm_weights_dummys.Add(imm_weights_dummys.Count);
            // Evaluate generalization population ratio
            //popul_ratio = (double)imm_training_inputs.Size / (double)gen_error_weights.Count;
        }

        private void clear_old_records()
        {
            // Gradient buffer cleaning
            var k = 0;
            while (k < grad_training.Size)
            {
                if (linear || (cur_time - grad_training[k].birth) < nonlin_cutoff_time || nonlin_time < nonlin_trigger)
                    k++;
                else
                {
                    grad_training.Get();
                    break;
                }
            }
            // Generalization buffer cleaning
            var gen_count = 0;
            for (var i = 0; i < gen_buffers.Length; i++)
            {
                k = 0;
                while (k < gen_buffers[i].Size - input_count - 2)
                {
                    if (linear || (cur_time - gen_buffers[i][k].birth) < nonlin_cutoff_time || nonlin_time < nonlin_trigger)
                        k++;
                    else
                    {
                        gen_buffers[i].Get();
                        break;
                    }
                }
                gen_count += gen_buffers[i].Size;
            }
            gen_space_fill_k = gen_count / (float)gen_space_size;
        }

        // An input counts as varied, and its coefficient gets fitted, only once it spans one step of the
        // generalization grid. Upstream took any difference at all, so float noise on a nearly constant input was
        // fitted, and next to the constant term that gives a coefficient of noise over noise, large and of either
        // sign. An input that hasn't varied keeps its current coefficient.
        private void update_singularity(IList<Vector> input_list)
        {
            for (var j = 0; j < input_count; j++)
            {
                var min = double.PositiveInfinity;
                var max = double.NegativeInfinity;
                for (var i = 0; i < input_list.Count; i++)
                {
                    var x = input_list[i][j];
                    if (x < min)
                        min = x;
                    if (x > max)
                        max = x;
                }
                inputs_changed[j] = max - min > gen_triggers[j];
            }
        }

        private const int time_reset = int.MaxValue / 2;

        // [AutoGuiAttr("max age", true)]
        public volatile int max_age = 3000;

        private void reset_time()
        {
            for (var i = 0; i < gen_buffers.Length; i++)
                for (var j = 0; j < gen_buffers[i].Size; j++)
                {
                    var cur_struct = gen_buffers[i][j];
                    var cur_age = Math.Min(cur_time - cur_struct.birth, max_age);
                    gen_buffers[i][j] = new GenStruct(cur_struct.val, max_age - cur_age, cur_struct.coord);
                }
            for (var i = 0; i < grad_training.Size; i++)
            {
                var cur_age = Math.Min(cur_time - grad_training[i].birth, max_age);
                grad_training[i] = new GenStruct(grad_training[i].val, max_age - cur_age, grad_training[i].coord);
            }
            cur_time = max_age;
        }

        private double getAgeWeight(int birth)
        {
            var age = Math.Min(cur_time - birth, max_age);
            var result = 1.0 / (weight_time_decay * age + 1.0);
            return result;
        }

        #endregion

    }
}