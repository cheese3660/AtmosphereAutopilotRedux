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
using AtmosphereAutopilotRedux.Runtime;

namespace AtmosphereAutopilotRedux.Modules
{

    /// <summary>
    /// Represents autopilot module, wich can be turned on and off.
    /// Derived classes need to implement OnActivate and OnDeactivate.
    /// </summary>
    public abstract class AutopilotModule
    {
        public const int PITCH = 0;
        public const int ROLL = 1;
        public const int YAW = 2;

        public const float rad2dgr = (float)(180.0 / Math.PI);
        public const float dgr2rad = (float)(Math.PI / 180.0);

        protected AAVessel vessel = null;
        protected bool enabled = false;
        protected string module_name;

        protected AutopilotModule(AAVessel v, string module_name)
        {
            vessel = v;
            this.module_name = module_name;
        }

        /// <summary>
        /// If this module is dependent on other modules, you should get references to
        /// required ones in this call.
        /// </summary>
        /// <param name="modules">Map of autopilot modules, instanced for this vessel.</param>
        public virtual void InitializeDependencies(Dictionary<Type, AutopilotModule> modules) { }

        public void Activate()
        {
            if (!enabled)
                OnActivate();
            enabled = true;
        }

        public string ModuleName => module_name;

        protected abstract void OnActivate();

        public void Deactivate()
        {
            if (enabled)
                OnDeactivate();
            enabled = false;
        }

        protected abstract void OnDeactivate();

        public bool Active
        {
            get => enabled;
            set
            {
                if (value)
                    Activate();
                else
                    Deactivate();
            }
        }
    }
}
