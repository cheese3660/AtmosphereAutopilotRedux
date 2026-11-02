using System;
using System.Collections.Generic;
using AtmosphereAutopilotRedux.Modules;
using AtmosphereAutopilotRedux.Runtime;
using KSP.Game;
using KSP.Sim.Definitions;
using KSP.Sim.impl;
using Redux.UI.AppBar;
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
        private VisualElement _engageHint;

        private Label _fmaSpeed, _fmaSpeedArmed, _fmaLateral, _fmaLateralArmed, _fmaVertical, _fmaVerticalArmed;
        private Button _tabCruise, _tabApproach, _tabTakeoff;
        private VisualElement _cruiseTab, _approachTab, _takeoffTab;

        private Label _toRunway, _toCourse, _toRemaining, _toSpeed, _takeoffWarning;
        private FloatField _rotateSpeedInput, _rotatePitchInput, _climbSpeedInput;
        private IntegerField _accelHeightInput;
        private Toggle _gearUpToggle;
        private Button _startTakeoff, _cancelTakeoff;

        // The runway end the vessel stands on, found again every second while the takeoff tab is open
        private ApproachPath _takeoffRunway;

        private VisualElement _approachSetup, _approachActive;
        private DropdownField _airportField;
        private VisualElement _runwayButtons;
        private Label _rwyLength, _rwyCourse, _rwyDistance, _rwyElevation, _goAroundInfo;
        private FloatField _glideslopeInput, _approachSpeedInput;
        private IntegerField _minimumsInput;
        private Toggle _flyFixes, _autoland;
        private VisualElement _approachHeightRow;
        private IntegerField _approachHeightInput;
        private Button _armApproach, _goAround, _disarmApproach;
        private Label _appDistanceLabel;
        private Label _approachTitle, _locValue, _gsValue, _appDistance, _appHeight, _appSpeed, _appVS, _approachWarning;
        private VisualElement _locMarker, _gsMarker;

        private enum Course { Hold, Heading, Waypoint }

        private enum Vertical { Off, Altitude, Speed }

        private enum Tab { Cruise, Approach, Takeoff }

        private Tab _tab = Tab.Cruise;

        // The cruise settings live in the window. Each vessel's are kept in the save (AutopilotSaves) and come back
        // when it is flown again; a vessel without a record picks up whatever is set
        private Course _course = Course.Hold;
        private Vertical _vertical = Vertical.Off;

        private const string NoWaypoints = "No Waypoints";

        // Waypoints on the active vessel's body, in dropdown order
        private readonly List<WaypointComponent> _waypoints = new List<WaypointComponent>();

        private const string NoAirports = "No Airports";

        // Airports on the active vessel's body, in dropdown order, and the runway end the approach is set up for
        private readonly List<Airport> _airports = new List<Airport>();
        private string _airportsBody;
        private Airport _airport;
        private Runway _runway;
        private RunwayEnd _runwayEnd;

        private AARuntime _runtime;
        private AutopilotContext _context;

        public void Awake()
        {
            _root = GetComponent<PanelRenderer>().GetWindowRoot();
            _root?.CenterByDefault();
            var shell = _root as AppShell ?? _root?.Q<AppShell>();
            if (shell != null)
                shell.CloseClicked += () =>
                {
                    // Turn the app bar button off too, or it stays lit and the next press only turns it off. The
                    // toggle's callback closes the window; setting it here covers a button the lookup can't find,
                    // such as one in a collapsed tray.
                    AppBar.SetAppBarButtonIndicator(AtmosphereAutopilotReduxPlugin.AppButtonId, false);
                    isOpen = false;
                };

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
            _engageHint = _root.Q<VisualElement>("EngageHint");

            _fmaSpeed = _root.Q<Label>("FmaSpeed");
            _fmaSpeedArmed = _root.Q<Label>("FmaSpeedArmed");
            _fmaLateral = _root.Q<Label>("FmaLateral");
            _fmaLateralArmed = _root.Q<Label>("FmaLateralArmed");
            _fmaVertical = _root.Q<Label>("FmaVertical");
            _fmaVerticalArmed = _root.Q<Label>("FmaVerticalArmed");
            _tabCruise = _root.Q<Button>("TabCruise");
            _tabApproach = _root.Q<Button>("TabApproach");
            _tabTakeoff = _root.Q<Button>("TabTakeoff");
            _takeoffTab = _root.Q<VisualElement>("TakeoffTab");
            _toRunway = _root.Q<Label>("ToRunway");
            _toCourse = _root.Q<Label>("ToCourse");
            _toRemaining = _root.Q<Label>("ToRemaining");
            _toSpeed = _root.Q<Label>("ToSpeed");
            _takeoffWarning = _root.Q<Label>("TakeoffWarning");
            _rotateSpeedInput = _root.Q<FloatField>("RotateSpeedInput");
            _rotatePitchInput = _root.Q<FloatField>("RotatePitchInput");
            _climbSpeedInput = _root.Q<FloatField>("ClimbSpeedInput");
            _accelHeightInput = _root.Q<IntegerField>("AccelHeightInput");
            _gearUpToggle = _root.Q<Toggle>("GearUpToggle");
            _startTakeoff = _root.Q<Button>("StartTakeoff");
            _cancelTakeoff = _root.Q<Button>("CancelTakeoff");
            _cruiseTab = _root.Q<VisualElement>("CruiseTab");
            _approachTab = _root.Q<VisualElement>("ApproachTab");

            _approachSetup = _root.Q<VisualElement>("ApproachSetup");
            _approachActive = _root.Q<VisualElement>("ApproachActive");
            _airportField = _root.Q<DropdownField>("AirportField");
            _runwayButtons = _root.Q<VisualElement>("RunwayButtons");
            _rwyLength = _root.Q<Label>("RwyLength");
            _rwyCourse = _root.Q<Label>("RwyCourse");
            _rwyDistance = _root.Q<Label>("RwyDistance");
            _rwyElevation = _root.Q<Label>("RwyElevation");
            _goAroundInfo = _root.Q<Label>("GoAroundInfo");
            _glideslopeInput = _root.Q<FloatField>("GlideslopeInput");
            _approachSpeedInput = _root.Q<FloatField>("ApproachSpeedInput");
            _minimumsInput = _root.Q<IntegerField>("MinimumsInput");
            _flyFixes = _root.Q<Toggle>("FlyFixes");
            _autoland = _root.Q<Toggle>("Autoland");
            _approachHeightRow = _root.Q<VisualElement>("ApproachHeightRow");
            _approachHeightInput = _root.Q<IntegerField>("ApproachHeightInput");
            _armApproach = _root.Q<Button>("ArmApproach");
            _goAround = _root.Q<Button>("GoAround");
            _disarmApproach = _root.Q<Button>("DisarmApproach");
            _approachTitle = _root.Q<Label>("ApproachTitle");
            _locValue = _root.Q<Label>("LocValue");
            _gsValue = _root.Q<Label>("GsValue");
            _appDistance = _root.Q<Label>("AppDistance");
            _appDistanceLabel = _root.Q<Label>("AppDistanceLabel");
            _appHeight = _root.Q<Label>("AppHeight");
            _appSpeed = _root.Q<Label>("AppSpeed");
            _appVS = _root.Q<Label>("AppVS");
            _approachWarning = _root.Q<Label>("ApproachWarning");
            _locMarker = _root.Q<VisualElement>("LocMarker");
            _gsMarker = _root.Q<VisualElement>("GsMarker");

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

            _tabCruise.clicked += () => SelectTab(Tab.Cruise);
            _tabApproach.clicked += () => SelectTab(Tab.Approach);
            _tabTakeoff.clicked += () => SelectTab(Tab.Takeoff);
            _rotateSpeedInput.RegisterValueChangedCallback(_ => PushTakeoff());
            _rotatePitchInput.RegisterValueChangedCallback(_ => PushTakeoff());
            _climbSpeedInput.RegisterValueChangedCallback(_ => PushTakeoff());
            _accelHeightInput.RegisterValueChangedCallback(_ => PushTakeoff());
            _gearUpToggle.RegisterValueChangedCallback(_ => PushTakeoff());
            _startTakeoff.clicked += StartTakeoff;
            _cancelTakeoff.clicked += () => _context?.Takeoff.Deactivate();
            _airportField.RegisterValueChangedCallback(_ => SelectAirport(_airportField.index));
            _glideslopeInput.RegisterValueChangedCallback(_ => PushApproach());
            _approachSpeedInput.RegisterValueChangedCallback(_ => PushApproach());
            _minimumsInput.RegisterValueChangedCallback(_ => PushApproach());
            _flyFixes.RegisterValueChangedCallback(_ => PushApproach());
            _autoland.RegisterValueChangedCallback(_ => PushApproach());
            _approachHeightInput.RegisterValueChangedCallback(_ => PushApproach());
            _armApproach.clicked += ArmApproach;
            _goAround.clicked += () => _context?.Approach.GoAround();
            _disarmApproach.clicked += () => _context?.Approach.Deactivate();

            // Window.Create only does this for TextFields, so typing a number would also fire flight keys
            _speedInput.DisableGameInputOnFocus();
            _headingInput.DisableGameInputOnFocus();
            _altitudeInput.DisableGameInputOnFocus();
            _vsInput.DisableGameInputOnFocus();
            _glideslopeInput.DisableGameInputOnFocus();
            _approachSpeedInput.DisableGameInputOnFocus();
            _minimumsInput.DisableGameInputOnFocus();
            _approachHeightInput.DisableGameInputOnFocus();
            _rotateSpeedInput.DisableGameInputOnFocus();
            _rotatePitchInput.DisableGameInputOnFocus();
            _climbSpeedInput.DisableGameInputOnFocus();
            _accelHeightInput.DisableGameInputOnFocus();

            RefreshSelectors();
            SelectTab(Tab.Cruise);
            RunwayDatabase.Changed += RefreshAirports;

            AutopilotSaves.Saving += StoreActiveState;
            AutopilotSaves.Loaded += OnSaveLoaded;
            _runtime = GetComponent<AARuntime>();
            if (_runtime != null)
            {
                _runtime.ContextChanging += StoreState;
                _runtime.ContextChanged += OnContextChanged;
                SetContext(_runtime.Context);
            }
            else
                RefreshModeButtons();
        }

        private void OnDestroy()
        {
            AutopilotSaves.Saving -= StoreActiveState;
            AutopilotSaves.Loaded -= OnSaveLoaded;
            RunwayDatabase.Changed -= RefreshAirports;
            if (_runtime != null)
            {
                _runtime.ContextChanging -= StoreState;
                _runtime.ContextChanged -= OnContextChanged;
            }
            SetContext(null);
        }

        public bool isOpen;

        private float _lastWaypointUpdate;

        public void Update()
        {
            UpdateRestore();
            if (!isOpen)
            {
                _root.style.display = DisplayStyle.None;
                return;
            }
            _root.style.display = DisplayStyle.Flex;

            var noControl = _context == null || _context.Vessel.Sim.ControlStatus == VesselControlState.NoControl;
            _noControl.style.display = noControl ? DisplayStyle.Flex : DisplayStyle.None;

            RefreshFma();
            if (_tab == Tab.Approach)
                RefreshApproachView();
            else if (_tab == Tab.Takeoff)
                RefreshTakeoffView();

            if (!(Time.unscaledTime - _lastWaypointUpdate > 1f)) return;
            _lastWaypointUpdate = Time.unscaledTime;
            RefreshWaypoints();
            if (_context?.Vessel.Sim.mainBody?.bodyName != _airportsBody)
                RefreshAirports();
            PullCruiseState();
            if (_tab == Tab.Takeoff)
                FindTakeoffRunway();
        }

        #region Context

        private void OnContextChanged() => SetContext(_runtime.Context);

        private void SetContext(AutopilotContext context)
        {
            if (_context != null)
            {
                _context.ActiveControllerChanged -= RefreshModeButtons;
                _context.Cruise.WaypointReached -= PullCruiseState;
                _context.Approach.PhaseChanged -= PullCruiseState;
                _context.Takeoff.PhaseChanged -= PullCruiseState;
            }
            _context = context;
            if (_context != null)
            {
                _context.ActiveControllerChanged += RefreshModeButtons;
                _context.Cruise.WaypointReached += PullCruiseState;
                _context.Approach.PhaseChanged += PullCruiseState;
                _context.Takeoff.PhaseChanged += PullCruiseState;
                RefreshWaypoints();
                RefreshAirports();
                ApplyState(AutopilotSaves.Find(VesselId(_context)));
                PushAll();
            }
            RefreshModeButtons();
        }

        private void ToggleMode(StateController controller)
        {
            if (_context == null || controller == null)
                return;
            _restoreController = null;      // the pilot decides now
            _restoreApproach = false;
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
            _engageHint.style.display = active == null ? DisplayStyle.Flex : DisplayStyle.None;
        }

        private void SelectTab(Tab tab)
        {
            _tab = tab;
            _tabCruise.EnableInClassList("tab_active", tab == Tab.Cruise);
            _tabApproach.EnableInClassList("tab_active", tab == Tab.Approach);
            _tabTakeoff.EnableInClassList("tab_active", tab == Tab.Takeoff);
            _takeoffTab.style.display = tab == Tab.Takeoff ? DisplayStyle.Flex : DisplayStyle.None;
            if (tab == Tab.Takeoff)
            {
                FindTakeoffRunway();
                RefreshTakeoffView();
            }
            _cruiseTab.style.display = tab == Tab.Cruise ? DisplayStyle.Flex : DisplayStyle.None;
            _approachTab.style.display = tab == Tab.Approach ? DisplayStyle.Flex : DisplayStyle.None;
            if (tab == Tab.Approach)
                RefreshApproachView();
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

        #region Saved state

        // Engages the vessel's saved mode once it has flown off rails for this many physics steps, enough for the
        // differences the controllers work from to span real flight
        private const int restore_steps = 3;

        private StateController _restoreController;
        private bool _restoreApproach, _restoreOnFinal;
        private AutopilotContext _restoreContext;
        private double _restoreFlyingSince = double.NaN;

        private static string VesselId(AutopilotContext context) => context?.Vessel.Sim?.GlobalId.ToString();

        private VesselAutopilotState CaptureState(AutopilotContext context)
        {
            var active = context.ActiveController;
            // A restore still waiting counts as engaged, so saving or switching straight after a load keeps it
            if (active == null && _restoreContext == context)
                active = _restoreController;
            var waypoint = SelectedWaypoint();
            var model = context.FlightModel;
            double[][] models = null;
            double[] scales = null;
            if (model.HasLearned)
            {
                models = model.ExportModels();
                scales = new double[context.VelocityControllers.Length];
                for (var i = 0; i < scales.Length; i++)
                    scales[i] = context.VelocityControllers[i].DebugAccController.DebugIndiScale;
            }
            return new VesselAutopilotState
            {
                Mass = context.Vessel.Sim.totalMass,
                PartCount = PartCount(context),
                Models = models,
                IndiScales = scales,
                Engaged = active == null ? "Off" : active == context.Cruise ? "Cruise" : "FlyByWire",
                Course = _course.ToString(),
                Heading = _headingInput.value,
                WaypointId = waypoint != null ? waypoint.GlobalId.ToString() : "",
                Vertical = _vertical.ToString(),
                Altitude = _altitudeInput.value,
                VerticalSpeed = _vsInput.value,
                SpeedHold = _speedEnabled.value,
                Speed = _speedInput.value,
                ApproachAirport = _airport?.Id ?? "",
                ApproachRunway = _runway?.Name ?? "",
                ApproachEnd = _runwayEnd?.Designator ?? "",
                Glideslope = _glideslopeInput.value,
                ApproachSpeed = _approachSpeedInput.value,
                Minimums = _minimumsInput.value,
                FlyFixes = _flyFixes.value,
                Autoland = _autoland.value,
                ApproachHeight = _approachHeightInput.value,
                RotateSpeed = _rotateSpeedInput.value,
                RotatePitch = _rotatePitchInput.value,
                ClimbSpeed = _climbSpeedInput.value,
                AccelHeight = _accelHeightInput.value,
                GearUp = _gearUpToggle.value,
                GearOffset = double.IsNaN(context.Approach.GearOffset) ? 0.0 : context.Approach.GearOffset,
                SteeringGain = double.IsNaN(context.Takeoff.LearnedSteeringGain) ? 0.0 : context.Takeoff.LearnedSteeringGain,
                ApproachArmed = ApproachInProgress(context) || (_restoreContext == context && _restoreApproach),
                ApproachPhase = _restoreContext == context && _restoreApproach && _restoreOnFinal
                    ? ApproachPhase.Localizer.ToString()
                    : context.Approach.Phase.ToString(),
            };
        }

        // Armed or captured; a go-around is on its way back to cruise, which the record covers
        private static bool ApproachInProgress(AutopilotContext context)
        {
            var phase = context.Approach.Phase;
            return phase == ApproachPhase.Transition || phase == ApproachPhase.Armed ||
                phase == ApproachPhase.Localizer || phase == ApproachPhase.Glideslope ||
                phase == ApproachPhase.Land || phase == ApproachPhase.Flare;
        }

        // The context is about to be disposed, which disengages it, so its record is taken first
        private void StoreState(AutopilotContext context)
        {
            if (context != null && context == _context)
                AutopilotSaves.Store(VesselId(context), CaptureState(context));
        }

        private void StoreActiveState() => StoreState(_context);

        private void OnSaveLoaded()
        {
            if (_context == null)
                return;
            RefreshAirports();
            ApplyState(AutopilotSaves.Find(VesselId(_context)));
            PushAll();
        }

        // Puts a vessel's record into the window and lines up its mode to engage. The caller pushes the settings.
        private void ApplyState(VesselAutopilotState state)
        {
            _restoreController = null;
            if (state == null || _context == null)
                return;
            if (Enum.TryParse(state.Course, out Course course))
                _course = course;
            if (Enum.TryParse(state.Vertical, out Vertical vertical))
                _vertical = vertical;
            _headingInput.SetValueWithoutNotify(state.Heading);
            _altitudeInput.SetValueWithoutNotify(state.Altitude);
            _vsInput.SetValueWithoutNotify(state.VerticalSpeed);
            _speedEnabled.SetValueWithoutNotify(state.SpeedHold);
            _speedInput.SetValueWithoutNotify(state.Speed);
            _glideslopeInput.SetValueWithoutNotify(state.Glideslope);
            _approachSpeedInput.SetValueWithoutNotify(state.ApproachSpeed);
            _minimumsInput.SetValueWithoutNotify(state.Minimums);
            _flyFixes.SetValueWithoutNotify(state.FlyFixes);
            _autoland.SetValueWithoutNotify(state.Autoland);
            _approachHeightInput.SetValueWithoutNotify(state.ApproachHeight);
            _rotateSpeedInput.SetValueWithoutNotify(state.RotateSpeed);
            _rotatePitchInput.SetValueWithoutNotify(state.RotatePitch);
            _climbSpeedInput.SetValueWithoutNotify(state.ClimbSpeed);
            _accelHeightInput.SetValueWithoutNotify(state.AccelHeight);
            _gearUpToggle.SetValueWithoutNotify(state.GearUp);
            if (state.GearOffset > 0.0)
                _context.Approach.SetGearOffset(state.GearOffset);
            if (state.SteeringGain > 0.0)
                _context.Takeoff.LearnedSteeringGain = state.SteeringGain;
            SelectRunwayEnd(state.ApproachAirport, state.ApproachRunway, state.ApproachEnd);
            var index = _waypoints.FindIndex(w => w.GlobalId.ToString() == state.WaypointId);
            if (index >= 0)
                _waypointField.index = index;
            else if (_course == Course.Waypoint)
                _course = Course.Hold;      // the waypoint is gone
            RefreshSelectors();
            RestoreModel(state);

            _restoreController = state.Engaged == "Cruise" ? _context.Cruise :
                state.Engaged == "FlyByWire" ? (StateController)_context.FlyByWire : null;
            _restoreApproach = state.ApproachArmed && _restoreController == _context.Cruise;
            // Past the localizer capture the approach resumes on final; before it, it starts over
            _restoreOnFinal = Enum.TryParse(state.ApproachPhase, out ApproachPhase saved_phase) &&
                (saved_phase == ApproachPhase.Localizer || saved_phase == ApproachPhase.Glideslope ||
                 saved_phase == ApproachPhase.Land || saved_phase == ApproachPhase.Flare);
            _restoreContext = _context;
            _restoreFlyingSince = double.NaN;
        }

        private static int PartCount(AutopilotContext context) =>
            context.Vessel.Sim?.SimulationObject?.PartOwner?.PartCount ?? 0;

        // Largest mass change, as a share, the saved model is still trusted across
        private const double model_mass_tolerance = 0.02;

        // The learned model only fits the vessel it was learned on; staged, docked or refuelled it starts fresh
        private void RestoreModel(VesselAutopilotState state)
        {
            if (state.Models == null || _context == null)
                return;
            var mass = _context.Vessel.Sim.totalMass;
            if (state.PartCount != PartCount(_context) ||
                Math.Abs(mass - state.Mass) > model_mass_tolerance * Math.Max(1e-3, state.Mass))
                return;
            if (!_context.FlightModel.ImportModels(state.Models) || state.IndiScales == null)
                return;
            var controllers = _context.VelocityControllers;
            for (var i = 0; i < controllers.Length && i < state.IndiScales.Length; i++)
                controllers[i].DebugAccController.RestoreIndiScale(state.IndiScales[i]);
        }

        private void PushAll()
        {
            PushSpeed();
            PushHeading();
            PushWaypoint();
            PushLateral();
            PushVertical();
            PushApproach();
            PushTakeoff();
        }

        // Waits for the vessel to fly, not just exist: paused, warping on rails or without control doesn't count
        private void UpdateRestore()
        {
            if (_restoreController == null)
                return;
            if (_context == null || _restoreContext != _context)
            {
                _restoreController = null;
                return;
            }
            var game = GameManager.Instance?.Game;
            var warp = game?.ViewController?.TimeWarp;
            var flying = game != null && !game.ViewController.IsPaused &&
                (warp == null || !warp.IsWarping || warp.IsPhysicsTimeWarp) &&
                _context.Vessel.Sim.ControlStatus != VesselControlState.NoControl;
            if (!flying)
            {
                _restoreFlyingSince = double.NaN;
                return;
            }
            if (double.IsNaN(_restoreFlyingSince))
                _restoreFlyingSince = Time.fixedTimeAsDouble;
            if (Time.fixedTimeAsDouble - _restoreFlyingSince < restore_steps * Time.fixedDeltaTime - 1e-6)
                return;
            var controller = _restoreController;
            _restoreController = null;
            if (_context.ActiveController != controller)
                _context.Engage(controller);
            if (_restoreApproach)
                ArmApproach(_restoreOnFinal);
            _restoreApproach = _restoreOnFinal = false;
        }

        #endregion

        #region Keyboard nudges

        internal bool AltitudeHeld => _vertical == Vertical.Altitude;

        internal bool VerticalSpeedHeld => _vertical == Vertical.Speed;

        internal void NudgeSpeed(int delta)
        {
            _speedInput.SetValueWithoutNotify(Mathf.Max(0f, Mathf.Round(_speedInput.value) + delta));
            PushSpeed();
        }

        internal void NudgeVertical(int delta)
        {
            switch (_vertical)
            {
                case Vertical.Altitude:
                    _altitudeInput.SetValueWithoutNotify(Mathf.Max(0, _altitudeInput.value + delta));
                    break;
                case Vertical.Speed:
                    _vsInput.SetValueWithoutNotify(_vsInput.value + delta);
                    break;
                default:
                    return;
            }
            PushVertical();
        }

        internal void NudgeHeading(int delta)
        {
            switch (_course)
            {
                case Course.Waypoint:
                    return;
                // Steering off the held great circle starts a heading hold from the current course
                case Course.Hold:
                    SelectCourse(Course.Heading);
                    break;
            }
            var heading = (_headingInput.value + delta) % 360;
            _headingInput.SetValueWithoutNotify(heading < 0 ? heading + 360 : heading);
            PushHeading();
        }

        #endregion

        #region Autopilot to UI

        // The autopilot can change its settings by itself: cruise drops to level flight when a waypoint is reached,
        // and when heading hold is asked for near the poles; a go-around hands back on the runway course with
        // altitude hold; and minimums leave the autothrottle on at the approach speed
        private void PullCruiseState()
        {
            if (_context == null)
                return;
            var cruise = _context.Cruise;
            var course = cruise.WaypointMode ? Course.Waypoint : cruise.CourseHoldMode ? Course.Heading : Course.Hold;
            var vertical = !cruise.vertical_control ? Vertical.Off :
                cruise.height_mode == CruiseController.HeightMode.Altitude ? Vertical.Altitude : Vertical.Speed;
            if (course == Course.Heading)
            {
                var heading = Mathf.RoundToInt((float)cruise.desired_course) % 360;
                if (heading != _headingInput.value)
                    _headingInput.SetValueWithoutNotify(heading);
            }
            var speed = _context.Speed;
            if (_speedEnabled.value != speed.spd_control_enabled)
                _speedEnabled.SetValueWithoutNotify(speed.spd_control_enabled);
            if (!Mathf.Approximately(_speedInput.value, speed.setpoint_mps))
                _speedInput.SetValueWithoutNotify(speed.setpoint_mps);
            if (course == _course && vertical == _vertical) return;
            _course = course;
            _vertical = vertical;
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

        #region Mode annunciator

        // Active modes on top, armed ones below, as on an airliner's flight mode annunciator
        private void RefreshFma()
        {
            var active = _context?.ActiveController;
            var cruise = active != null && active == _context.Cruise;
            string speed = null, lateral = null, vertical = null, speed_armed = null;
            string lateral_armed = null, vertical_armed = null;
            if (active != null && _context.Speed.spd_control_enabled)
                speed = "SPEED";
            if (cruise)
            {
                lateral = _course == Course.Waypoint ? "WPT" : _course == Course.Heading ? "HDG" : "HOLD";
                vertical = _vertical == Vertical.Altitude ? "ALT" : _vertical == Vertical.Speed ? "V/S" : null;
                switch (_context.Approach.Phase)
                {
                    // Slowing down first, heading out toward the final before the fixes are laid out
                    case ApproachPhase.Transition when _context.Approach.SlowingForTransition:
                        lateral = "APP";
                        vertical = "ALT";
                        vertical_armed = "G/S";
                        break;
                    case ApproachPhase.Transition:
                        lateral = _context.Approach.NextFixName ?? "IF";
                        vertical = "ALT";
                        lateral_armed = _context.Approach.LocalizerArmed ? "LOC" : null;
                        vertical_armed = "G/S";
                        break;
                    case ApproachPhase.Armed:
                        lateral_armed = "LOC";
                        vertical_armed = "G/S";
                        break;
                    case ApproachPhase.Localizer:
                        lateral = "LOC";
                        vertical_armed = "G/S";
                        break;
                    case ApproachPhase.Glideslope:
                        lateral = "LOC";
                        vertical = "G/S";
                        if (_context.Approach.autoland)
                        {
                            lateral_armed = "LAND";
                            vertical_armed = "LAND";
                        }
                        break;
                    case ApproachPhase.Land:
                        lateral = "LAND";
                        vertical = "LAND";
                        lateral_armed = "ROLLOUT";
                        vertical_armed = "FLARE";
                        speed_armed = "RETARD";
                        break;
                    case ApproachPhase.Flare:
                        speed = "RETARD";
                        lateral = "LAND";
                        vertical = "FLARE";
                        lateral_armed = "ROLLOUT";
                        break;
                    case ApproachPhase.Rollout:
                        speed = _context.Approach.ReversersDeployed ? "REV" : "IDLE";
                        lateral = "ROLLOUT";
                        vertical = null;
                        break;
                    case ApproachPhase.GoAround:
                        speed = "TOGA";
                        lateral = "GA TRK";
                        vertical = "GA CLB";
                        break;
                }
            }
            if (cruise)
                switch (_context.Takeoff.Phase)
                {
                    case TakeoffPhase.Roll:
                        speed = "TOGA";
                        lateral = "RWY";
                        vertical = null;
                        vertical_armed = "ROTATE";
                        break;
                    case TakeoffPhase.Rotate:
                        speed = "TOGA";
                        lateral = "RWY";
                        vertical = "ROTATE";
                        break;
                    case TakeoffPhase.Climb:
                        speed = "TOGA";
                        lateral = "RWY TRK";
                        vertical = "CLB";
                        break;
                    case TakeoffPhase.Rejected:
                        speed = _context.Takeoff.ReversersDeployed ? "REV" : "IDLE";
                        lateral = "RWY";
                        vertical = null;
                        break;
                }
            SetFma(_fmaSpeed, speed);
            SetFma(_fmaSpeedArmed, speed_armed);
            SetFma(_fmaLateral, lateral);
            SetFma(_fmaLateralArmed, lateral_armed);
            SetFma(_fmaVertical, vertical);
            SetFma(_fmaVerticalArmed, vertical_armed);
        }

        private static void SetFma(Label label, string text)
        {
            text ??= "-";
            if (label.text != text)
                label.text = text;
            label.EnableInClassList("fma_none", text == "-");
        }

        #endregion

        #region Takeoff

        private void FindTakeoffRunway()
        {
            var takeoff = _context?.Takeoff;
            if (takeoff != null && takeoff.Phase != TakeoffPhase.Off)
            {
                _takeoffRunway = takeoff.Path;
                return;
            }
            _takeoffRunway = _context != null && _context.Vessel.Refresh() && _context.Vessel.LandedOrSplashed()
                ? TakeoffGuidance.FindRunway(_context.Vessel)
                : null;
        }

        private void PushTakeoff()
        {
            if (_context == null)
                return;
            var takeoff = _context.Takeoff;
            takeoff.rotate_speed = Mathf.Max(1f, _rotateSpeedInput.value);
            takeoff.rotate_pitch_deg = Mathf.Clamp(_rotatePitchInput.value, 1f, 25f);
            takeoff.climb_speed = Mathf.Max(1f, _climbSpeedInput.value);
            takeoff.accel_height = Mathf.Max(50, _accelHeightInput.value);
            takeoff.gear_up = _gearUpToggle.value;
        }

        // The takeoff flies on top of cruise with the autothrottle on: cruise takes over at the acceleration
        // altitude with the cruise tab's altitude and speed
        private void StartTakeoff()
        {
            FindTakeoffRunway();
            if (_context == null || _takeoffRunway == null)
                return;
            if (_context.Vessel.Sim.ControlStatus == VesselControlState.NoControl)
                return;
            _restoreController = null;
            _context.Approach.Deactivate();
            if (_context.ActiveController != _context.Cruise)
                _context.Engage(_context.Cruise);
            if (!_speedEnabled.value)
            {
                _speedEnabled.SetValueWithoutNotify(true);
                PushSpeed();
            }
            PushTakeoff();
            _context.Takeoff.Start(_takeoffRunway);
        }

        private void RefreshTakeoffView()
        {
            var takeoff = _context?.Takeoff;
            var running = takeoff != null && takeoff.Phase != TakeoffPhase.Off;
            _startTakeoff.style.display = running ? DisplayStyle.None : DisplayStyle.Flex;
            _cancelTakeoff.style.display = running ? DisplayStyle.Flex : DisplayStyle.None;
            _startTakeoff.SetEnabled(_takeoffRunway != null);
            var warning = takeoff?.Warning ?? "";
            if (!running && _takeoffRunway == null && warning.Length == 0)
                warning = "NOT LINED UP ON A RUNWAY";
            _takeoffWarning.text = warning;
            _takeoffWarning.style.display = warning.Length > 0 ? DisplayStyle.Flex : DisplayStyle.None;

            var path = _takeoffRunway;
            if (path == null)
            {
                _toRunway.text = _toCourse.text = _toRemaining.text = "-";
            }
            else
            {
                if (!running && _context.Vessel.Refresh())
                    path.Update(_context.Vessel);
                _toRunway.text = $"{path.Runway.Airport?.Id} {path.End.Designator}";
                _toCourse.text = $"{Mathf.RoundToInt((float)path.CourseDeg) % 360:000}°";
                _toRemaining.text = $"{Math.Max(0.0, path.Length + path.AlongTrack) / 1000.0:0.0} km";
            }
            _toSpeed.text = _context != null ? $"{_context.Vessel.srfSpeed:0} m/s" : "-";
        }

        #endregion

        #region Approach

        private void RefreshAirports()
        {
            var body = _context?.Vessel.Sim.mainBody?.bodyName;
            _airportsBody = body;
            _airports.Clear();
            if (body != null)
                _airports.AddRange(RunwayDatabase.OnBody(body));
            var choices = new List<string>();
            foreach (var airport in _airports)
            {
                // Names can repeat, and the dropdown is keyed by its strings
                var name = string.IsNullOrEmpty(airport.Name) ? airport.Id : airport.Name;
                var choice = name;
                for (var i = 2; choices.Contains(choice); i++)
                    choice = $"{name} ({i})";
                choices.Add(choice);
            }
            _airportField.choices = choices;
            _airportField.SetEnabled(choices.Count > 0);
            // The database is rebuilt on reloads, so the selection is found again by id
            var index = _airport != null ? _airports.FindIndex(a => a.Id == _airport.Id) : -1;
            if (index < 0 && choices.Count > 0)
                index = 0;
            _airportField.SetValueWithoutNotify(index >= 0 ? choices[index] : NoAirports);
            SelectRunwayEnd(index >= 0 ? _airports[index].Id : null, _runway?.Name, _runwayEnd?.Designator);
        }

        private void SelectAirport(int index)
        {
            if (index < 0 || index >= _airports.Count || _airports[index] == _airport)
                return;
            SelectRunwayEnd(_airports[index].Id, null, null);
        }

        // Picks a runway end by name, keeping the airport even when the end isn't found
        private void SelectRunwayEnd(string airport_id, string runway_name, string designator)
        {
            _airport = _airports.Find(a => a.Id == airport_id) ?? (_airports.Count > 0 ? _airports[0] : null);
            // A restored approach picks its airport here, after the dropdown was filled, so the dropdown follows
            var index = _airport != null ? _airports.IndexOf(_airport) : -1;
            if (index >= 0 && index < _airportField.choices.Count)
                _airportField.SetValueWithoutNotify(_airportField.choices[index]);
            _runway = null;
            _runwayEnd = null;
            if (_airport != null && !string.IsNullOrEmpty(designator))
                foreach (var runway in _airport.Runways)
                {
                    if (!string.IsNullOrEmpty(runway_name) && runway.Name != runway_name)
                        continue;
                    var end = runway.Ends.Find(e => e.Designator == designator);
                    if (end == null)
                        continue;
                    _runway = runway;
                    _runwayEnd = end;
                    break;
                }
            BuildRunwayButtons();
        }

        private void PickRunwayEnd(Runway runway, RunwayEnd end)
        {
            _runway = runway;
            _runwayEnd = end;
            // A new runway brings its own glideslope and approach altitude
            _glideslopeInput.SetValueWithoutNotify((float)end.GlideslopeDeg);
            _approachHeightInput.SetValueWithoutNotify(Mathf.RoundToInt((float)end.ApproachHeightM));
            PushApproach();
            BuildRunwayButtons();
        }

        private void BuildRunwayButtons()
        {
            _runwayButtons.Clear();
            if (_airport == null)
                return;
            foreach (var runway in _airport.Runways)
                foreach (var end in runway.Ends)
                {
                    var button = new OabButton(() => PickRunwayEnd(runway, end)) { text = end.Designator };
                    button.AddToClassList("segment_button");
                    button.EnableInClassList("active", end == _runwayEnd);
                    _runwayButtons.Add(button);
                }
            _armApproach.SetEnabled(_runwayEnd != null);
        }

        private void PushApproach()
        {
            if (_context == null)
                return;
            var approach = _context.Approach;
            approach.approach_speed = Mathf.Max(1f, _approachSpeedInput.value);
            approach.minimums = Mathf.Max(0, _minimumsInput.value);
            approach.autoland = _autoland.value;
            approach.approach_height = Mathf.Max(100, _approachHeightInput.value);
            // The transition is generated when armed; switching fixes on or off takes effect on the next arm
            if (approach.Phase == ApproachPhase.Off)
                approach.fly_fixes = _flyFixes.value;
            _approachHeightRow.style.display = _flyFixes.value ? DisplayStyle.Flex : DisplayStyle.None;
            if (approach.Path != null && approach.Phase != ApproachPhase.Off)
                approach.Path.GlideslopeDeg = Mathf.Clamp(_glideslopeInput.value, 0.5f, 15f);
        }

        // The approach flies on top of cruise with the autothrottle on, as on airliners: arming from fly-by-wire or
        // disengaged engages cruise first
        private void ArmApproach() => ArmApproach(false);

        private void ArmApproach(bool on_final)
        {
            if (_context == null || _runwayEnd == null || _runway.Opposite(_runwayEnd) == null)
                return;
            if (_context.Vessel.Sim.ControlStatus == VesselControlState.NoControl)
                return;
            _restoreController = null;
            if (_context.ActiveController != _context.Cruise)
                _context.Engage(_context.Cruise);
            if (!_speedEnabled.value)
            {
                if (_context.Vessel.Refresh())
                    _speedInput.SetValueWithoutNotify(Mathf.Round((float)_context.Vessel.srfSpeed));
                _speedEnabled.SetValueWithoutNotify(true);
                PushSpeed();
            }
            PushApproach();
            _context.Takeoff.Deactivate();
            _context.Approach.fly_fixes = _flyFixes.value;
            var glideslope = Mathf.Clamp(_glideslopeInput.value, 0.5f, 15f);
            _context.Approach.Arm(new ApproachPath(_runway, _runwayEnd, glideslope), on_final);
        }

        private void RefreshApproachView()
        {
            var approach = _context?.Approach;
            var in_progress = approach != null && approach.Phase != ApproachPhase.Off;
            _approachSetup.style.display = in_progress ? DisplayStyle.None : DisplayStyle.Flex;
            _approachActive.style.display = in_progress ? DisplayStyle.Flex : DisplayStyle.None;
            var warning = approach?.Warning ?? "";
            _approachWarning.text = warning;
            _approachWarning.style.display = warning.Length > 0 ? DisplayStyle.Flex : DisplayStyle.None;
            _goAround.SetEnabled(approach != null &&
                (approach.Phase == ApproachPhase.Localizer || approach.Phase == ApproachPhase.Glideslope ||
                 approach.Phase == ApproachPhase.Land || approach.Phase == ApproachPhase.Flare));
            if (in_progress)
                RefreshApproachActive(approach);
            else
                RefreshApproachSetup();
        }

        private void RefreshApproachSetup()
        {
            _goAroundInfo.text = $"{_altitudeInput.value} m · {_speedInput.value:0} m/s";
            var far = _runwayEnd != null ? _runway.Opposite(_runwayEnd) : null;
            if (far == null || _context == null)
            {
                _rwyLength.text = _rwyCourse.text = _rwyDistance.text = _rwyElevation.text = "-";
                return;
            }
            var vessel = _context.Vessel.Sim;
            var radius = vessel.mainBody.radius;
            var here = new SurfacePoint { Latitude = vessel.Latitude, Longitude = vessel.Longitude };
            _rwyLength.text = $"{_runway.Length(radius):0} m";
            _rwyCourse.text = $"{Mathf.RoundToInt((float)SurfaceGeometry.Bearing(_runwayEnd.Threshold, far.Threshold)) % 360:000}°";
            _rwyDistance.text = $"{SurfaceGeometry.Distance(here, _runwayEnd.Threshold, radius) / 1000.0:0.0} km";
            _rwyElevation.text = $"{_runwayEnd.Threshold.Altitude:0} m";
        }

        // Localizer full scale is the 2.5 degrees either side an ILS localizer gives, seen from the far end;
        // glideslope full scale the 0.7 degrees of an ILS glideslope
        private const double loc_full_scale_deg = 2.5;
        private const double gs_full_scale_deg = 0.7;

        private void RefreshApproachActive(ApproachGuidance approach)
        {
            var path = approach.Path;
            _approachTitle.text = $"{path.Runway.Airport?.Id} {path.End.Designator} · {path.GlideslopeDeg:0.0}° · " +
                $"{approach.approach_speed:0} M/S";
            var along = Math.Max(0.0, path.AlongTrack);
            var loc_scale = Math.Max(0.5 * path.Runway.WidthM,
                (along + path.Length) * Math.Tan(loc_full_scale_deg * Mathf.Deg2Rad));
            var gs_scale = Math.Max(10.0, along * Math.Tan(gs_full_scale_deg * Mathf.Deg2Rad));
            var cross = path.CrossTrack;
            var high = -approach.GlideslopeError;
            // Markers show where the vessel is: right of the centreline, and above the glideslope, to the right
            SetMarker(_locMarker, cross / Math.Max(1.0, loc_scale));
            SetMarker(_gsMarker, high / gs_scale);
            _locValue.text = $"{FormatDeviation(cross)} {(cross >= 0.0 ? "R" : "L")}";
            _gsValue.text = $"{FormatDeviation(high)} {(high >= 0.0 ? "hi" : "lo")}";

            var vessel = _context.Vessel;
            var up = (vessel.GetWorldPos3D() - vessel.mainBody.position).normalized;
            // Flying the fixes, the distance is to the next one, named in the label
            var fix = approach.NextFixName;
            _appDistanceLabel.text = fix ?? "Distance";
            _appDistance.text = $"{(fix != null ? approach.NextFixDistance : path.AlongTrack) / 1000.0:0.0} km";
            // The radio altimeter once it reads, the height over the threshold before
            _appHeight.text = double.IsNaN(approach.RadioHeight)
                ? $"{approach.HeightAboveThreshold:0} m"
                : $"{approach.RadioHeight:0} m RA";
            _appSpeed.text = $"{vessel.srfSpeed:0} m/s";
            _appVS.text = $"{Vector3d.Dot(vessel.srf_velocity, up):0.0} m/s";
        }

        // Kilometres past 999 m, so a long way off the centreline still fits beside the scale
        private static string FormatDeviation(double metres)
        {
            var abs = Math.Abs(metres);
            return abs >= 999.5 ? $"{abs / 1000.0:0.0} km" : $"{abs:0} m";
        }

        private static void SetMarker(VisualElement marker, double deviation)
        {
            marker.style.left = Length.Percent((float)(50.0 + 50.0 * Math.Max(-1.0, Math.Min(1.0, deviation))));
        }

        #endregion
    }
}
