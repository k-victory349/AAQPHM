// Copy this file to secrets.h (same folder) and fill in your real value.
// secrets.h is gitignored — never commit the real device key.

// Must match DEVICE_API_KEY in the backend's .env (Render dashboard too,
// if the deployed service doesn't read from a committed .env).
#define DEVICE_API_KEY "replace-with-the-backend-DEVICE_API_KEY-value"

// Temporary bring-up fallback, used only when no credentials are saved yet
// in flash (via BLE/serial provisioning). Remove once real provisioning exists.
#define WIFI_FALLBACK_SSID "replace-with-your-wifi-ssid"
#define WIFI_FALLBACK_PASSWORD "replace-with-your-wifi-password"
