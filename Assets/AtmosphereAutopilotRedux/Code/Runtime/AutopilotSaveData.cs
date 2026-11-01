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
using KSP.Game;
using SpaceWarp2.API.Backend.SaveGameManager;
using SpaceWarp2.API.SaveGameManager;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// What the autopilot was doing on one vessel: which mode was engaged and the window's settings for it. Enums are
    /// stored by name so a renamed or added value can't silently shift the others.
    /// </summary>
    [Serializable]
    public sealed class VesselAutopilotState
    {
        public string Engaged = "Off";
        public string Course = "Hold";
        public int Heading;
        public string WaypointId = "";
        public string Vertical = "Off";
        public int Altitude;
        public int VerticalSpeed;
        public bool SpeedHold;
        public float Speed;

        // What the flight model learned, so a load or a switch back doesn't start it from the seed. Only valid for the
        // vessel as it was: mass and part count say whether it still is.
        public double Mass;
        public int PartCount;
        public double[][] Models;
        public double[] IndiScales;
    }

    /// <summary>
    /// The autopilot's part of a save: one record per vessel, keyed by the vessel's global id. Learned models are kept
    /// with the vessel they were learned on and dropped when it changes.
    /// </summary>
    [Serializable]
    public sealed class AutopilotSaveData
    {
        public Dictionary<string, VesselAutopilotState> Vessels = new Dictionary<string, VesselAutopilotState>();
    }

    /// <summary>
    /// Keeps the autopilot's records in the save through Space Warp's per-save mod data.
    /// </summary>
    public static class AutopilotSaves
    {
        private const string SaveGuid = "AtmosphereAutopilotRedux";

        /// <summary>
        /// The records of the save loaded now. Space Warp copies a loaded save into this same instance.
        /// </summary>
        public static readonly AutopilotSaveData Data = new AutopilotSaveData();

        /// <summary>
        /// Raised just before the save is written, so the active vessel's record can be brought up to date.
        /// </summary>
        public static event Action Saving;

        /// <summary>
        /// Raised after a save's records were loaded.
        /// </summary>
        public static event Action Loaded;

        private static bool registered;

        public static void Register()
        {
            if (registered)
                return;
            ModSaves.RegisterSaveLoadGameData<AutopilotSaveData>(SaveGuid, OnSave, OnLoad, OnNew, Data,
                PersistenceKind.PerSave);
            registered = true;
        }

        public static VesselAutopilotState Find(string vesselId)
        {
            if (string.IsNullOrEmpty(vesselId) || Data.Vessels == null)
                return null;
            return Data.Vessels.TryGetValue(vesselId, out var state) ? state : null;
        }

        public static void Store(string vesselId, VesselAutopilotState state)
        {
            if (string.IsNullOrEmpty(vesselId) || state == null)
                return;
            Data.Vessels ??= new Dictionary<string, VesselAutopilotState>();
            Data.Vessels[vesselId] = state;
        }

        private static void OnSave(AutopilotSaveData data)
        {
            Saving?.Invoke();
            prune(data);
        }

        private static void OnLoad(AutopilotSaveData data)
        {
            data.Vessels ??= new Dictionary<string, VesselAutopilotState>();
            Loaded?.Invoke();
        }

        // A save without our data starts with no records, not with the last save's
        private static AutopilotSaveData OnNew(AutopilotSaveData data)
        {
            data.Vessels = new Dictionary<string, VesselAutopilotState>();
            return data;
        }

        // Records of vessels that no longer exist would only pile up
        private static void prune(AutopilotSaveData data)
        {
            var universe = GameManager.Instance?.Game?.UniverseModel;
            if (universe == null || data.Vessels == null || data.Vessels.Count == 0)
                return;
            var alive = new HashSet<string>();
            foreach (var vessel in universe.GetAllVessels())
                if (vessel != null)
                    alive.Add(vessel.GlobalId.ToString());
            var gone = new List<string>();
            foreach (var id in data.Vessels.Keys)
                if (!alive.Contains(id))
                    gone.Add(id);
            foreach (var id in gone)
                data.Vessels.Remove(id);
        }
    }
}
