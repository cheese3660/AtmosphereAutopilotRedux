using AtmosphereAutopilotRedux.Runtime;
using KSP.Game;
using KSP.Input;
using UnityEngine;
using UnityEngine.InputSystem;

namespace AtmosphereAutopilotRedux
{
    /// <summary>
    /// Keyboard nudges for the autopilot setpoints, read from the game's own flight bindings so rebinds carry over:
    /// throttle up/down moves the held speed, pitch the held altitude or vertical speed, yaw the held heading.
    /// While speed hold is on, the game's throttle keys are switched off so they can't fight it.
    /// </summary>
    public class SetpointKeys : MonoBehaviour
    {
        /// <summary>Held speed change per second of throttle up/down, m/s.</summary>
        public float speed_rate = 10f;

        /// <summary>Held altitude change per second of pitch input, m.</summary>
        public float altitude_rate = 100f;

        /// <summary>Held vertical speed change per second of pitch input, m/s.</summary>
        public float vertical_speed_rate = 5f;

        /// <summary>Held heading change per second of yaw input, degrees.</summary>
        public float heading_rate = 15f;

        private AARuntime _runtime;
        private WindowController _window;

        // Fractions of a unit carried between frames, since the fields hold whole numbers
        private float _speedSteps, _verticalSteps, _headingSteps;

        private bool _throttleKeysBlocked;

        // Our own copy of throttle up/down. The game's is switched off under speed hold, since the flight input
        // handler applies it to the throttle every frame, and a disabled action reads nothing.
        private InputAction _throttleDelta;

        private void Awake()
        {
            _runtime = GetComponent<AARuntime>();
            _window = GetComponent<WindowController>();
        }

        private void OnDisable()
        {
            var input = GameManager.Instance?.Game?.Input;
            if (input != null)
                BlockThrottleKeys(input.Flight, false);
            _throttleDelta?.Disable();
        }

        private void OnDestroy()
        {
            _throttleDelta?.Dispose();
            _throttleDelta = null;
        }

        private void Update()
        {
            var input = GameManager.Instance?.Game?.Input;
            if (input == null)
                return;
            var flight = input.Flight;
            var context = _runtime != null ? _runtime.Context : null;
            var engaged = context?.ActiveController != null;
            BlockThrottleKeys(flight, engaged && context.Speed.spd_control_enabled);
            if (!engaged || _window == null)
            {
                _speedSteps = _verticalSteps = _headingSteps = 0f;
                return;
            }

            var dt = Time.unscaledDeltaTime;
            if (_throttleDelta == null)
            {
                // Clone keeps the player's rebinds
                _throttleDelta = flight.ThrottleDelta.Clone();
                _throttleDelta.Enable();
            }
            // The copy sits outside the game's input locks, so only listen while the flight map is live
            var throttleInput = flight.ThrottleMax.actionMap.enabled ? _throttleDelta.ReadValue<float>() : 0f;
            var speed = context.Speed.spd_control_enabled
                ? Steps(ref _speedSteps, throttleInput, speed_rate, dt)
                : Steps(ref _speedSteps, 0f, 0f, dt);
            if (speed != 0)
                _window.NudgeSpeed(speed);

            var cruise = context.ActiveController == context.Cruise;
            // Pitch is negative nose down, so it lowers the target
            var verticalRate = _window.AltitudeHeld ? altitude_rate : _window.VerticalSpeedHeld ? vertical_speed_rate : 0f;
            var vertical = Steps(ref _verticalSteps, cruise ? flight.Pitch.ReadValue<float>() : 0f, verticalRate, dt);
            if (vertical != 0)
                _window.NudgeVertical(vertical);

            var heading = Steps(ref _headingSteps, cruise ? flight.Yaw.ReadValue<float>() : 0f, heading_rate, dt);
            if (heading != 0)
                _window.NudgeHeading(heading);
        }

        // Whole units from a held input. The remainder carries to the next frame and is dropped on release.
        private static int Steps(ref float accumulated, float input, float rate, float dt)
        {
            if (input == 0f || rate == 0f)
            {
                accumulated = 0f;
                return 0;
            }
            accumulated += input * rate * dt;
            var steps = (int)accumulated;
            accumulated -= steps;
            return steps;
        }

        private void BlockThrottleKeys(GameInput.FlightActions flight, bool block)
        {
            if (block)
            {
                // An input lock lifting re-enables the whole map, so this is re-applied every frame
                if (flight.ThrottleMax.enabled)
                    flight.ThrottleMax.Disable();
                if (flight.ThrottleCutoff.enabled)
                    flight.ThrottleCutoff.Disable();
                if (flight.ThrottleDelta.enabled)
                    flight.ThrottleDelta.Disable();
            }
            else if (_throttleKeysBlocked)
            {
                Restore(flight.ThrottleMax);
                Restore(flight.ThrottleCutoff);
                Restore(flight.ThrottleDelta);
            }
            _throttleKeysBlocked = block;
        }

        // Only hand an action back while its map is live; enabling one in a disabled map would let it fire through
        // an input lock. A map enabled later brings its actions back by itself.
        private static void Restore(InputAction action)
        {
            if (!action.enabled && action.actionMap.enabled)
                action.Enable();
        }
    }
}
