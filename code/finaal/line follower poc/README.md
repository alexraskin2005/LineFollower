ESP32 WiFi service

What I added

- src/wifi_service.h / src/wifi_service.cpp: a simple WiFi service that:
  - loads/stores Wifi credentials in EEPROM using EEPROMAnything (via PlatformIO lib_deps)
  - attempts to connect to WiFi on startup
  - if connection fails, starts an AP named "ESP32_Config" and serves a minimal HTML form to enter SSID and password
  - saves credentials and restarts
- src/main.cpp updated to initialize the service and print status to Serial
- platformio.ini: added lib_deps = EEPROMAnything
 - src/storage.h: centralized EEPROM storage helper for all project parameters (WiFi credentials start at address 0)

How it works

- Stored credentials are read using the `Storage` helper. WiFi credentials use `STORAGE_WIFI_CRED_ADDR` (0) by default. If SSID is empty the service assumes no credentials are present.
- On first boot, or if connection fails within 15 seconds, the device creates an open AP (password "configureme") and serves a tiny web page at http://192.168.4.1/ where you can enter SSID and password.
- After submitting the form the device saves credentials to EEPROM and restarts.

Assumptions and notes

- Uses Arduino ESP32 core WiFi and WebServer. These are provided by the `espressif32` PlatformIO platform.
- The project cannot be built in this environment (PlatformIO not available here). Build and upload on your machine using PlatformIO in VSCode or the CLI.
- The Storage helper uses `EEPROM.put`/`EEPROM.get` (and `EEPROM.commit`). The `EEPROMAnything` library is declared in `platformio.ini` so PlatformIO will fetch it.

How to build locally (Windows PowerShell)

# Install PlatformIO (if not installed)
# Follow PlatformIO installation instructions for VSCode or CLI: https://platformio.org/install/cli

# From project root:
platformio run
platformio run -t upload

Troubleshooting

- If AP page doesn't load, check Serial output for messages and try connecting to WiFi network named "ESP32_Config".
- The configuration portal runs for 5 minutes; if you miss it, power-cycle the board.

Next improvements

- Use WiFiManager or AsyncWebServer for a more robust captive portal (recommended)
- Encrypt stored password or use Preferences (non-EEPROM) API
- Add captive portal DNS to redirect any URL to the portal
