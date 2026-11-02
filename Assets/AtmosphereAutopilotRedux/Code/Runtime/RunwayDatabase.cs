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
using Newtonsoft.Json;
using UnityEngine;

namespace AtmosphereAutopilotRedux.Runtime
{
    /// <summary>
    /// A point on a body's surface: latitude and longitude in degrees, altitude above sea level in metres.
    /// </summary>
    [Serializable]
    public sealed class SurfacePoint
    {
        [JsonProperty("lat")] public double Latitude;
        [JsonProperty("lon")] public double Longitude;
        [JsonProperty("alt")] public double Altitude;
    }

    /// <summary>
    /// One end of a runway, as an approach to it sees it: the threshold landing starts from and its approach
    /// defaults.
    /// </summary>
    [Serializable]
    public sealed class RunwayEnd
    {
        [JsonProperty("designator")] public string Designator = "";

        /// <summary>Start of the landing surface on the runway centreline.</summary>
        [JsonProperty("threshold")] public SurfacePoint Threshold;

        /// <summary>How far down the runway landing starts past the threshold, m.</summary>
        [JsonProperty("displaced_m")] public double DisplacedM;

        /// <summary>Usable surface past this end, for stopping only, m.</summary>
        [JsonProperty("overrun_m")] public double OverrunM;

        /// <summary>Default glideslope angle, degrees.</summary>
        [JsonProperty("glideslope_deg")] public double GlideslopeDeg = 3.0;

        /// <summary>Height the glideslope crosses the threshold at, m.</summary>
        [JsonProperty("tch_m")] public double ThresholdCrossingHeightM = 15.0;

        /// <summary>Default height over the threshold, m, an approach flying its fixes meets the glideslope from.
        /// Higher where terrain rises near the approach.</summary>
        [JsonProperty("approach_height_m")] public double ApproachHeightM = 600.0;
    }

    /// <summary>
    /// A runway: its two ends, which set its centreline, heading and length, and its width.
    /// </summary>
    [Serializable]
    public sealed class Runway
    {
        [JsonProperty("name")] public string Name = "";
        [JsonProperty("width_m")] public double WidthM;
        [JsonProperty("ends")] public List<RunwayEnd> Ends = new List<RunwayEnd>();

        [JsonIgnore] public Airport Airport { get; internal set; }

        /// <summary>
        /// The end an approach to <paramref name="end" /> rolls out towards.
        /// </summary>
        public RunwayEnd Opposite(RunwayEnd end) => Ends.Count == 2 ? (Ends[0] == end ? Ends[1] : Ends[0]) : null;

        /// <summary>
        /// Length between the two thresholds, m, along the surface of a body of the given radius.
        /// </summary>
        public double Length(double body_radius) =>
            Ends.Count == 2 ? SurfaceGeometry.Distance(Ends[0].Threshold, Ends[1].Threshold, body_radius) : 0.0;
    }

    /// <summary>
    /// An airport: one per data asset, so a mod adds one by shipping a file and changes one by patching it by name.
    /// </summary>
    [Serializable]
    public sealed class Airport
    {
        [JsonProperty("id")] public string Id = "";
        [JsonProperty("body")] public string Body = "";
        [JsonProperty("name")] public string Name = "";
        [JsonProperty("runways")] public List<Runway> Runways = new List<Runway>();
    }

    /// <summary>
    /// Great-circle geometry on a body's surface, enough to fly to a runway: distance and initial bearing.
    /// </summary>
    public static class SurfaceGeometry
    {
        public static double Distance(SurfacePoint a, SurfacePoint b, double radius)
        {
            var lat1 = a.Latitude * Mathf.Deg2Rad;
            var lat2 = b.Latitude * Mathf.Deg2Rad;
            var dlat = lat2 - lat1;
            var dlon = (b.Longitude - a.Longitude) * Mathf.Deg2Rad;
            var h = Math.Sin(dlat / 2) * Math.Sin(dlat / 2) +
                Math.Cos(lat1) * Math.Cos(lat2) * Math.Sin(dlon / 2) * Math.Sin(dlon / 2);
            return 2.0 * radius * Math.Asin(Math.Min(1.0, Math.Sqrt(h)));
        }

        /// <summary>
        /// Initial bearing from <paramref name="a" /> to <paramref name="b" />, degrees clockwise from north.
        /// </summary>
        public static double Bearing(SurfacePoint a, SurfacePoint b)
        {
            var lat1 = a.Latitude * Mathf.Deg2Rad;
            var lat2 = b.Latitude * Mathf.Deg2Rad;
            var dlon = (b.Longitude - a.Longitude) * Mathf.Deg2Rad;
            var y = Math.Sin(dlon) * Math.Cos(lat2);
            var x = Math.Cos(lat1) * Math.Sin(lat2) - Math.Sin(lat1) * Math.Cos(lat2) * Math.Cos(dlon);
            var bearing = Math.Atan2(y, x) * Mathf.Rad2Deg;
            return bearing < 0.0 ? bearing + 360.0 : bearing;
        }
    }

    /// <summary>
    /// Every airport the game knows, from the JSON assets under <see cref="Label" />. Space Warp reads them in the boot
    /// flow after Patch Manager has patched them, and again whenever the feature set changes, so another mod's airports
    /// count only while that mod is on.
    /// </summary>
    public static class RunwayDatabase
    {
        /// <summary>
        /// Addressables label of the airport assets.
        /// </summary>
        public const string Label = "aa_airports";

        private static readonly List<Airport> airports = new List<Airport>();

        public static IReadOnlyList<Airport> Airports => airports;

        /// <summary>
        /// Raised after the airports were cleared or one was added.
        /// </summary>
        public static event Action Changed;

        /// <summary>
        /// Registers the airport assets with Space Warp's loading flow. Call in OnPreInitialized at the latest.
        /// </summary>
        public static void Register(string mod_id)
        {
            SpaceWarp2.API.Loading.Loading.AddSetLoadingAction<TextAsset>(mod_id, "Airports", Label, Add, Clear);
        }

        /// <summary>
        /// Forgets the airports of the previous feature set.
        /// </summary>
        public static void Clear()
        {
            airports.Clear();
            Changed?.Invoke();
        }

        /// <summary>
        /// Airports on the named body.
        /// </summary>
        public static IEnumerable<Airport> OnBody(string body)
        {
            foreach (var airport in airports)
                if (string.Equals(airport.Body, body, StringComparison.OrdinalIgnoreCase))
                    yield return airport;
        }

        /// <summary>
        /// Reads one airport asset. A broken one is reported and left out; it shouldn't take the others with it.
        /// </summary>
        public static void Add(TextAsset asset)
        {
            Airport airport;
            try
            {
                airport = JsonConvert.DeserializeObject<Airport>(asset.text);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[AtmosphereAutopilotRedux] Airport asset '{asset.name}' isn't valid JSON: {e.Message}");
                return;
            }
            if (airport == null || string.IsNullOrEmpty(airport.Id) || string.IsNullOrEmpty(airport.Body))
            {
                Debug.LogWarning($"[AtmosphereAutopilotRedux] Airport asset '{asset.name}' has no id or body");
                return;
            }
            airport.Runways ??= new List<Runway>();
            airport.Runways.RemoveAll(runway =>
            {
                var valid = runway != null && runway.Ends != null && runway.Ends.Count == 2 &&
                    runway.Ends.TrueForAll(end => end?.Threshold != null);
                if (!valid)
                    Debug.LogWarning($"[AtmosphereAutopilotRedux] Airport '{airport.Id}': runway " +
                        $"'{runway?.Name}' needs two ends with thresholds, left out");
                return !valid;
            });
            foreach (var runway in airport.Runways)
                runway.Airport = airport;
            airports.RemoveAll(existing => existing.Id == airport.Id);
            airports.Add(airport);
            Changed?.Invoke();
        }
    }
}
