using System;
using System.Collections.Generic;
using AtmosphereAutopilotRedux.Modules;
using AtmosphereAutopilotRedux.Runtime;
using KSP.Game;
using KSP.Sim.Definitions;
using KSP.Sim.impl;
using UitkForKsp2.API;
using UitkForKsp2.Controls;
using UnityEngine;
using UnityEngine.UIElements;

namespace AtmosphereAutopilotRedux
{
    public class WindowController : MonoBehaviour
    {
        private VisualElement _root;

        private Label _noControl;
        private Button _fbwButton, _cruiseButton;
        private VisualElement _speedControls;
        private FloatField _speedInput;
        private Toggle _speedEnabled;
        private VisualElement _cruiseControls;
        private Button _courseHold, _courseHeading, _courseWaypoint;
        private VisualElement _headingRow, _waypointRow;
        private IntegerField _headingInput;
        private DropdownField _waypointField;
        private Button _verticalOff, _verticalAltitude, _verticalSpeed;
        private VisualElement _altitudeRow, _vsRow;
        private IntegerField _altitudeInput;
        private IntegerField _vsInput;

        private enum Course { Hold, Heading, Waypoint }

        private enum Vertical { Off, Altitude, Speed }

        // The cruise settings live in the window, so they carry over to a new vessel
        private Course _course = Course.Hold;
        private Vertical _vertical = Vertical.Off;

        private const string NoWaypoints = "No Waypoints";

        // Waypoints on the active vessel's body, in dropdown order
        private readonly List<WaypointComponent> _waypoints = new List<WaypointComponent>();

        private AARuntime _runtime;
        private AutopilotContext _context;

        public void Awake()
        {
            _root = GetComponent<PanelRenderer>().GetWindowRoot();
            _root?.CenterByDefault();
            var shell = _root as AppShell ?? _root?.Q<AppShell>();
            if (shell != null)
                shell.CloseClicked += () => isOpen = false;

            _noControl = _root.Q<Label>("NoControl");
            _fbwButton = _root.Q<Button>("FBW");
            _cruiseButton = _root.Q<Button>("Cruise");
            _speedControls = _root.Q<VisualElement>("SpeedControls");
            _speedInput = _root.Q<FloatField>("SpeedInput");
            _speedEnabled = _root.Q<Toggle>("SpeedEnabled");
            _cruiseControls = _root.Q<VisualElement>("CruiseControls");
            _courseHold = _root.Q<Button>("CourseHold");
            _courseHeading = _root.Q<Button>("CourseHeading");
            _courseWaypoint = _root.Q<Button>("CourseWaypoint");
            _headingRow = _root.Q<VisualElement>("HeadingRow");
            _waypointRow = _root.Q<VisualElement>("WaypointRow");
            _headingInput = _root.Q<IntegerField>("HeadingInput");
            _waypointField = _root.Q<DropdownField>("WaypointField");
            _verticalOff = _root.Q<Button>("VerticalOff");
            _verticalAltitude = _root.Q<Button>("VerticalAltitude");
            _verticalSpeed = _root.Q<Button>("VerticalSpeed");
            _altitudeRow = _root.Q<VisualElement>("AltitudeRow");
            _vsRow = _root.Q<VisualElement>("VSRow");
            _altitudeInput = _root.Q<IntegerField>("AltitudeInput");
            _vsInput = _root.Q<IntegerField>("VSInput");

            _fbwButton.clicked += () => ToggleMode(_context?.FlyByWire);
            _cruiseButton.clicked += () => ToggleMode(_context?.Cruise);

            _speedInput.RegisterValueChangedCallback(_ => PushSpeed());
            _speedEnabled.RegisterValueChangedCallback(evt =>
            {
                // Start from where the vessel is now
                if (evt.newValue && _context != null && _context.Vessel.Refresh())
                    _speedInput.SetValueWithoutNotify(Mathf.Round((float)_context.Vessel.srfSpeed));
                PushSpeed();
            });

            _courseHold.clicked += () => SelectCourse(Course.Hold);
            _courseHeading.clicked += () => SelectCourse(Course.Heading);
            _courseWaypoint.clicked += () => SelectCourse(Course.Waypoint);
            _headingInput.RegisterValueChangedCallback(_ => PushHeading());
            _waypointField.RegisterValueChangedCallback(_ => PushWaypoint());

            _verticalOff.clicked += () => SelectVertical(Vertical.Off);
            _verticalAltitude.clicked += () => SelectVertical(Vertical.Altitude);
            _verticalSpeed.clicked += () => SelectVertical(Vertical.Speed);
            _altitudeInput.RegisterValueChangedCallback(_ => PushVertical());
            _vsInput.RegisterValueChangedCallback(_ => PushVertical());

            RefreshSelectors();

            _runtime = GetComponent<AARuntime>();
            if (_runtime != null)
            {
                _runtime.ContextChanged += OnContextChanged;
                SetContext(_runtime.Context);
            }
            else
                RefreshModeButtons();
        }

        private void OnDestroy()
        {
            if (_runtime != null)
                _runtime.ContextChanged -= OnContextChanged;
            SetContext(null);
        }

        public bool isOpen;

        private float _lastWaypointUpdate;

        public void Update()
        {
            if (!isOpen)
            {
                _root.style.display = DisplayStyle.None;
                return;
            }
            _root.style.display = DisplayStyle.Flex;

            var noControl = _context == null || _context.Vessel.Sim.ControlStatus == VesselControlState.NoControl;
            _noControl.style.display = noControl ? DisplayStyle.Flex : DisplayStyle.None;

            if (!(Time.unscaledTime - _lastWaypointUpdate > 1f)) return;
            _lastWaypointUpdate = Time.unscaledTime;
            RefreshWaypoints();
            PullCruiseState();
        }

        #region Context

        private void OnContextChanged() => SetContext(_runtime.Context);

        private void SetContext(AutopilotContext context)
        {
            if (_context != null)
            {
                _context.ActiveControllerChanged -= RefreshModeButtons;
                _context.Cruise.WaypointReached -= PullCruiseState;
            }
            _context = context;
            if (_context != null)
            {
                _context.ActiveControllerChanged += RefreshModeButtons;
                _context.Cruise.WaypointReached += PullCruiseState;
                // The settings live in the window, so a new vessel picks up whatever is set there
                RefreshWaypoints();
                PushSpeed();
                PushHeading();
                PushWaypoint();
                PushLateral();
                PushVertical();
            }
            RefreshModeButtons();
        }

        private void ToggleMode(StateController controller)
        {
            if (_context == null || controller == null)
                return;
            if (_context.ActiveController == controller)
                _context.Disengage();
            else
                _context.Engage(controller);
        }

        private void RefreshModeButtons()
        {
            var active = _context?.ActiveController;
            _fbwButton.EnableInClassList("active", active != null && active == _context.FlyByWire);
            var cruise = active != null && active == _context.Cruise;
            _cruiseButton.EnableInClassList("active", cruise);
            // Speed hold works under either autopilot; course and vertical modes only under cruise
            _speedControls.style.display = active != null ? DisplayStyle.Flex : DisplayStyle.None;
            _cruiseControls.style.display = cruise ? DisplayStyle.Flex : DisplayStyle.None;
        }

        private void SelectCourse(Course course)
        {
            if (course == _course)
                return;
            _course = course;
            switch (course)
            {
                // Start from where the vessel is now
                case Course.Heading when _context != null && _context.Vessel.Refresh():
                    _headingInput.SetValueWithoutNotify(Mathf.RoundToInt((float)_context.Cruise.current_course()) % 360);
                    break;
                case Course.Waypoint when SelectedWaypoint() == null && _waypoints.Count > 0:
                    _waypointField.index = 0;
                    break;
            }

            RefreshSelectors();
            PushLateral();
        }

        private void SelectVertical(Vertical vertical)
        {
            if (vertical == _vertical)
                return;
            _vertical = vertical;
            // Start from where the vessel is now
            if (_context != null && _context.Vessel.Refresh())
            {
                var vessel = _context.Vessel;
                switch (vertical)
                {
                    case Vertical.Altitude:
                        _altitudeInput.SetValueWithoutNotify(Mathf.RoundToInt((float)vessel.altitude));
                        break;
                    case Vertical.Speed:
                    {
                        var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
                        _vsInput.SetValueWithoutNotify(Mathf.RoundToInt((float)Vector3d.Dot(vessel.srf_velocity, up)));
                        break;
                    }
                }
            }
            RefreshSelectors();
            PushVertical();
        }

        private void RefreshSelectors()
        {
            _courseHold.EnableInClassList("active", _course == Course.Hold);
            _courseHeading.EnableInClassList("active", _course == Course.Heading);
            _courseWaypoint.EnableInClassList("active", _course == Course.Waypoint);
            _headingRow.style.display = _course == Course.Heading ? DisplayStyle.Flex : DisplayStyle.None;
            _waypointRow.style.display = _course == Course.Waypoint ? DisplayStyle.Flex : DisplayStyle.None;

            _verticalOff.EnableInClassList("active", _vertical == Vertical.Off);
            _verticalAltitude.EnableInClassList("active", _vertical == Vertical.Altitude);
            _verticalSpeed.EnableInClassList("active", _vertical == Vertical.Speed);
            _altitudeRow.style.display = _vertical == Vertical.Altitude ? DisplayStyle.Flex : DisplayStyle.None;
            _vsRow.style.display = _vertical == Vertical.Speed ? DisplayStyle.Flex : DisplayStyle.None;
        }

        #endregion

        #region UI to autopilot

        private void PushSpeed()
        {
            if (_context == null)
                return;
            _context.Speed.setpoint_mps = Mathf.Max(0f, _speedInput.value);
            _context.Speed.spd_control_enabled = _speedEnabled.value;
        }

        private void PushHeading()
        {
            if (_context == null)
                return;
            double course = _headingInput.value % 360;
            _context.Cruise.desired_course = course < 0.0 ? course + 360.0 : course;
        }

        private void PushWaypoint()
        {
            if (_context == null)
                return;
            var waypoint = SelectedWaypoint();
            if (waypoint == null)
                return;
            waypoint.MainBody.GetLatLonAltFromRadius(waypoint.transform.Position, out var lat, out var lon, out _);
            _context.Cruise.desired_latitude = lat;
            _context.Cruise.desired_longitude = lon;
        }

        // Heading hold and waypoint are both lateral modes; with neither on the cruise controller holds its current
        // great circle
        private void PushLateral()
        {
            if (_context == null)
                return;
            var cruise = _context.Cruise;
            switch (_course)
            {
                case Course.Waypoint when SelectedWaypoint() != null:
                    PushWaypoint();
                    cruise.WaypointMode = true;
                    break;
                case Course.Heading:
                    PushHeading();
                    cruise.CourseHoldMode = true;
                    break;
                default:
                    cruise.LevelFlightMode = true;
                    break;
            }
        }

        private void PushVertical()
        {
            if (_context == null)
                return;
            var cruise = _context.Cruise;
            cruise.desired_altitude = _altitudeInput.value;
            cruise.desired_vertsetpoint = _vsInput.value;
            switch (_vertical)
            {
                case Vertical.Altitude:
                    cruise.height_mode = CruiseController.HeightMode.Altitude;
                    cruise.vertical_control = true;
                    break;
                case Vertical.Speed:
                    cruise.height_mode = CruiseController.HeightMode.VerticalSpeed;
                    cruise.vertical_control = true;
                    break;
                default:
                    cruise.vertical_control = false;
                    break;
            }
        }

        #endregion

        #region Autopilot to UI

        // The cruise controller can change mode by itself: it drops to level flight when a waypoint is reached, and
        // when heading hold is asked for near the poles
        private void PullCruiseState()
        {
            if (_context == null)
                return;
            var cruise = _context.Cruise;
            var course = cruise.WaypointMode ? Course.Waypoint : cruise.CourseHoldMode ? Course.Heading : Course.Hold;
            if (course == _course) return;
            _course = course;
            RefreshSelectors();
        }

        private void RefreshWaypoints()
        {
            var selected = SelectedWaypoint();
            _waypoints.Clear();
            var choices = new List<string>();
            var body = _context?.Vessel.Sim.mainBody;
            var universe = GameManager.Instance?.Game?.UniverseModel;
            if (body != null && universe != null)
            {
                foreach (var waypoint in universe.GetAllWaypoints())
                {
                    if (waypoint?.MainBody != body)
                        continue;
                    _waypoints.Add(waypoint);
                    // Names can repeat, and the dropdown is keyed by its strings
                    var name = waypoint.SimulationObject.Name;
                    var choice = name;
                    for (var i = 2; choices.Contains(choice); i++)
                        choice = $"{name} ({i})";
                    choices.Add(choice);
                }
            }
            _waypointField.choices = choices;
            _courseWaypoint.SetEnabled(choices.Count > 0);
            _waypointField.SetEnabled(choices.Count > 0);

            var index = selected != null ? _waypoints.IndexOf(selected) : -1;
            if (index < 0 && choices.Count > 0)
                index = 0;
            _waypointField.SetValueWithoutNotify(index >= 0 ? choices[index] : NoWaypoints);

            // The waypoint being flown to is gone
            if (_course != Course.Waypoint ||
                (choices.Count != 0 && (selected == null || _waypoints.IndexOf(selected) >= 0))) return;
            if (choices.Count == 0)
                _course = Course.Hold;
            RefreshSelectors();
            PushLateral();
        }

        private WaypointComponent SelectedWaypoint()
        {
            var index = _waypointField.index;
            return index >= 0 && index < _waypoints.Count ? _waypoints[index] : null;
        }

        #endregion
    }
}
