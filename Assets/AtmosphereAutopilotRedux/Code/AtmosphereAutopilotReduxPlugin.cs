using SpaceWarp2.API.Configuration;
using SpaceWarp2.API.Configuration.Attributes;
using SpaceWarp2.API.Loading;
using SpaceWarp2.API.Mods;
using SpaceWarp2.UI.API.Appbar;
using UitkForKsp2.API;
using UnityEngine;
using UnityEngine.AddressableAssets;
using UnityEngine.ResourceManagement.AsyncOperations;
using UnityEngine.UIElements;

namespace AtmosphereAutopilotRedux
{
    /* Extend KerbalMod instead if you need the MonoBehaviour update loop/references to game stuff like SW 1.x mods */
    public class AtmosphereAutopilotReduxPlugin : GeneralMod
    {
        private VisualTreeAsset _windowUxml;
        private Sprite _icon;
        private WindowController _controller;

        [ConfigSection("Debug")]
        [ConfigValue("FlightRecorder",
            "Write a CSV of every physics step to AppData/LocalLow/.../AtmosphereAutopilotRedux for tuning the autopilot.")]
        [ConfigMetadata(ConfigTags.Hidden)]
        public bool FlightRecorderEnabled = false;

        public override void OnInitialized()
        {
            SWConfiguration.Bind(this);
            Runtime.FlightRecorder.Configure(FlightRecorderEnabled);

            var handle = Addressables.LoadAssetAsync<VisualTreeAsset>(
                "Assets/AtmosphereAutopilotRedux/UI/MainWindow/AtmosphereAutopilotWindow.uxml");
            
            handle.Completed += (h) =>
            {
                if (h.Status != AsyncOperationStatus.Succeeded) return;
                
                _windowUxml = h.Result;
                var options = WindowOptions.Default;
                options.WindowId = "AtmosphereAutopilotRedux";

                var window = Window.Create(options, _windowUxml);
                // The runtime goes first so the window controller finds it in Awake
                window.gameObject.AddComponent<Runtime.AARuntime>();
                _controller = window.gameObject.AddComponent<WindowController>();
                window.gameObject.AddComponent<SetpointKeys>();
            };


            var handle2 = Addressables.LoadAssetAsync<Sprite>("Assets/AtmosphereAutopilotRedux/UI/icon.png");
            handle2.Completed += (h) =>
            {
                if (h.Status != AsyncOperationStatus.Succeeded) return;
                _icon = h.Result;
                // Registered once here, not per campaign: Space Warp builds the flight app bar from its list
                // whenever flight opens, and loading a save straight into flight can do that before our
                // campaign callback runs. The list already hides the button while the mod is disabled.
                Appbar.RegisterAppButton("Atmosphere Autopilot", "BTN-AtmosphereAutopilot", _icon,
                    on =>
                    {
                        if (_controller != null) _controller.isOpen = on;
                    });
            };
        }
    }
}