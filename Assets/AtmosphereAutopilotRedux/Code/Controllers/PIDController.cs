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
using System.ComponentModel;
using AtmosphereAutopilotRedux.Utils;
using KSP.Sim.impl;
using UnityEngine;

namespace AtmosphereAutopilotRedux.Controllers
{

    /// <summary>
    /// PID-based input value controller
    /// </summary>
    public class PIDController
    {
        /// <summary>
        /// Proportional gain coefficient
        /// </summary>
        public double KP { get => kp;
	        set => kp = value;
        }
        protected double kp = 1.0;

        /// <summary>
        /// Integral gain coefficient
        /// </summary>
        public double KI { get => ki;
	        set => ki = value;
        }
        protected double ki = 1.0;

        /// <summary>
        /// Diffirential gain coefficient
        /// </summary>
        public double KD { get => kd;
	        set => kd = value;
        }
        protected double kd = 0.01;

        /// <summary>
        /// Maximum error, wich lets integral component to raise
        /// </summary>
        public double IntegralClamp { get => iclamp;
	        set => iclamp = value;
        }
        protected double iclamp = 1.0;

        /// <summary>
        /// Maximum accumulator derivative
        /// </summary>
        public double AccumulDerivClamp { get => adclamp;
	        set => adclamp = value;
        }
        protected double adclamp = 1.0;

        /// <summary>
        /// Maximum magnitude for integral component of controller reaction
        /// </summary>
        public double AccumulatorClamp { get => aclamp;
	        set => aclamp = value;
        }
        protected double aclamp = 0.1;

        /// <summary>
        /// Accumulator gain, is multiplied on error * dt to get accumulator change
        /// </summary>
        public double IntegralGain { get => igain;
	        set => igain = value;
        }
        protected double igain = 1.0;

		/// <summary>
		/// Input derivative value from the last control iteration
		/// </summary>
		public double InputDerivative => derivative;

		protected double derivative;			            // input derivative value

		/// <summary>
		/// Current accumulator value
		/// </summary>
        public double Accumulator { get => i_accumulator;
	        set => i_accumulator = value;
        }

        // Main step variables
        protected double i_accumulator = 0.0;               // current integral accumulator state
        protected double last_dt = 1.0;                     // last delta time
        protected double[] input_stack = new double[3];     // contains last 3 input values, needed for correct integration and differentiation

		/// <summary>
		/// Last error value. Error = desire - input
		/// </summary>
		public double LastError => last_error;

		protected double last_error = 0.0;	

        public double Control(double input, double desire, double dt)
        {
            var error = desire - input;
            var new_dt = dt;

            // proportional component
            var proportional = error * kp;

            // diffirential component
            if (!dt_is_constant(new_dt))
            {
                // dt has changed
                clean_value_stack(input);
                new_dt = Time.fixedDeltaTime;
                last_error = error;
            }
            update_value_stack(input);
            if (kd != 0.0)
                derivative = (input_stack[0] - 4 * input_stack[1] + 3 * input_stack[2]) / new_dt * 0.5;
            var diffirential = -derivative * kd;

            // integral component             
            if (ki != 0.0)
            {
                var d_integral = Math.Abs(error) > iclamp ? 0.0 : new_dt * 0.5 * (error + last_error);       // raw diffirential
                d_integral = Common.Clamp(igain * d_integral, adclamp * new_dt);                                // clamp it
                i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);                               // accumulate
            }
            var integral = i_accumulator * ki;

            // update previous values
            last_dt = new_dt;
            last_error = error;

            return proportional + integral + diffirential;
        }

        /// <summary>
        /// Control function with manually provided input derivative
        /// </summary>
		public double Control(double input, double input_d, double desire, double dt)
		{
			var error = desire - input;
			var new_dt = dt;

			// proportional component
			var proportional = error * kp;

			// diffirential component
			derivative = input_d;
			var diffirential = -input_d * kd;

			// integral component       
            if (ki != 0.0)
            {
                var d_integral = Math.Abs(error) > iclamp ? 0.0 : new_dt * 0.5 * (error + last_error);       // raw diffirential
                d_integral = Common.Clamp(igain * d_integral, adclamp * new_dt);                                // clamp it
                i_accumulator = Common.Clamp(i_accumulator + d_integral, aclamp);                               // accumulate
            }
			var integral = i_accumulator * ki;

			// update previous values
			last_dt = new_dt;
			last_error = error;

			return proportional + integral + diffirential;
		}

		/// <summary>
		/// Clear accumulator
		/// </summary>
        public void clear()
        {
            i_accumulator = 0.0;
        }

        protected void update_value_stack(double new_input)     // update value_stack with new input
        {
            input_stack[0] = input_stack[1];
            input_stack[1] = input_stack[2];
            input_stack[2] = new_input;
        }

        protected void clean_value_stack(double fill_value)      // need to clean value stack
        {
            for (var i = 0; i < 3; i++)
                input_stack[i] = fill_value;
        }

        protected bool dt_is_constant(double new_dt)            // return true if new dt is roughly the same as old one
        {
            if (Math.Abs(new_dt / last_dt - 1.0) < 0.1)
                return true;
            else
                return false;
        }

    }
}
