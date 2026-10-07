#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <time.h>

#define TINY_GSM_MODEM_SIM800
#define TINY_GSM_RX_BUFFER 1024
#include <TinyGsmClient.h>

#include <DHT.h>
#include <HardwareSerial.h>
#include <math.h>
#include "driver/gpio.h"
#include <NimBLEDevice.h>

#include "secrets.h"   // defines DEVICE_API_KEY — copy secrets.h.example to secrets.h first

// ---------------- Pins ----------------
#define GSM_RX_PIN 26
#define GSM_TX_PIN 27
#define GSM_PWRKEY_PIN 5
#define DHTPIN 4
#define DHTTYPE DHT22
#define MQ135_PIN 34
#define BUZZER_PIN 25
#define PMOS_GATE_PIN 23
#define PMS_SET_PIN 18
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

// ---------------- Timing ----------------
#define SENSOR_WARMUP_MS 30000          // PMS5003 needs about 30 s after wake for stable data
#define SLEEP_INTERVAL_SEC 60
#define WIFI_CONNECT_TIMEOUT_MS 12000
#define BLE_WINDOW_MS 45000           // includes time for the user to type the PIN
#define BLE_ACK_TIMEOUT_MS 10000
#define GSM_RESPONSE_TIMEOUT_MS 10000

// ---------------- Queue / payload ----------------
#define CHANGE_THRESHOLD_PCT 10.0
#define MAX_QUEUE_SIZE 60
#define PAYLOAD_BUF_SIZE 8192           // 60 readings at ~130 bytes each fits with room to spare

// ---------------- MQ135 ----------------
#define MQ135_RLOAD 10.0f               // kOhm, load resistor on the module
#define MQ135_RZERO 76.63f              // kOhm, from your clean-air calibration
#define MQ135_VC 3.3f                   // supply voltage of the sensor's load circuit
#define MQ135_DIVIDER_GAIN 1.0f         // if AO goes through a divider, set to (R1+R2)/R2
#define MQ135_SAMPLES 10
#define MQ135_MAX_PPM 99999.0f

// ---------------- Network ----------------
const char apn[] = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Go backend (Render), NOT the Vercel frontend — the frontend has no API routes.
const char* serverApiUrl = "https://ambiant-air-monitor-backend.onrender.com/api/v1/devices/readings";

// 1 = HTTPS straight to Render from the SIM800L.
//     The SIM800L's TLS stack is old and Render may reject it.
// 0 = plain HTTP to a relay you control, which forwards to Render over HTTPS.
#define GSM_USE_TLS 1
#if GSM_USE_TLS
const char gsmServerHost[] = "ambiant-air-monitor-backend.onrender.com";
const int gsmServerPort = 443;
#else
const char gsmServerHost[] = "your-relay-host.example.com";
const int gsmServerPort = 80;
#endif
const char gsmServerPath[] = "/api/v1/devices/readings";

// ---------------- BLE ----------------
#define SERVICE_UUID "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_PASSKEY 102026              // 6-digit PIN the phone must enter to pair

// ---------------- Data types and RTC state ----------------
struct SensorReading {
  bool valid;
  uint32_t seq;       // increments every reading, survives deep sleep
  uint32_t t;         // device clock in seconds when the reading was taken
  float humidity;
  float temperature;
  float pm1_0;
  float pm2_5;
  float pm10;
  float voc_ppm;
};

RTC_DATA_ATTR float activePM25Threshold = 45.0;
RTC_DATA_ATTR float activeVOCThreshold = 2500.0;
RTC_DATA_ATTR float activeTempThreshold = 30.0;

RTC_DATA_ATTR SensorReading rtcQueue[MAX_QUEUE_SIZE];
RTC_DATA_ATTR int rtcQueueCount = 0;
RTC_DATA_ATTR uint32_t rtcNextSeq = 0;
RTC_DATA_ATTR float lastUploadedPM25 = -1.0;
RTC_DATA_ATTR float lastUploadedVOC = -1.0;
RTC_DATA_ATTR float lastUploadedTemp = -1.0;

// ---------------- Prototypes ----------------
SensorReading sampleAllSensors();
bool readPMS5003(float &pm1, float &pm25, float &pm10);
float readMQ135Compensated(float humidity, float temperature);
bool hasChangedSignificantly(const SensorReading &r);
void triggerAlert(const SensorReading &r);
bool connectWiFi();
void pushToRTCQueue(const SensorReading &r);
void dropOldestFromQueue(int n);
int buildBatchJsonPayload(char* buffer, size_t maxLen, uint32_t nowSec);
void parseAndUpdateThresholds(const String& payload);
bool uploadReadingToVercel(const char* jsonPayload);
bool transmitViaBLE(const char* jsonPayload);
bool sendPayloadOverBLE(const char* jsonPayload);
bool transmitViaGSM(const char* jsonPayload);
void shutdownGSM();
void powerDownAndSleep();
void printReading(const SensorReading &r);
uint32_t nowSeconds();

// ---------------- Globals ----------------
HardwareSerial gsmSerial(1);
TinyGsm modem(gsmSerial);
#if GSM_USE_TLS
TinyGsmClientSecure gsmClient(modem);
#else
TinyGsmClient gsmClient(modem);
#endif

Preferences preferences;
DHT dht(DHTPIN, DHTTYPE);
HardwareSerial pmsSerial(2);

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
volatile bool deviceConnected = false;
volatile bool deviceAuthenticated = false;
volatile bool clientSubscribed = false;
volatile bool bleAckReceived = false;
volatile bool wifiCredsUpdated = false;
volatile uint16_t peerMTU = 23;

// ---------------- BLE callbacks ----------------
class MyServerCallbacks : public NimBLEServerCallbacks {
  void onConnect(NimBLEServer* s, NimBLEConnInfo& connInfo) override {
    deviceConnected = true;
    deviceAuthenticated = false;
    peerMTU = connInfo.getMTU();
    Serial.println("Phone connected via BLE. Requesting PIN pairing...");
    // Ask the phone to pair straight away so the PIN prompt appears
    NimBLEDevice::startSecurity(connInfo.getConnHandle());
  }
  void onDisconnect(NimBLEServer* s, NimBLEConnInfo& connInfo, int reason) override {
    deviceConnected = false;
    deviceAuthenticated = false;
    clientSubscribed = false;
    Serial.println("Phone disconnected.");
  }
  uint32_t onPassKeyDisplay() override {
    return BLE_PASSKEY;
  }
  void onAuthenticationComplete(NimBLEConnInfo& connInfo) override {
    if (connInfo.isEncrypted() && connInfo.isAuthenticated()) {
      deviceAuthenticated = true;
      Serial.println("BLE pairing successful. Connection is encrypted.");
    } else {
      Serial.println("BLE pairing failed (wrong PIN?). Disconnecting.");
      if (pServer) pServer->disconnect(connInfo.getConnHandle());
    }
  }
  void onMTUChange(uint16_t mtu, NimBLEConnInfo& connInfo) override {
    peerMTU = mtu;
    Serial.printf("BLE MTU negotiated: %u\n", mtu);
  }
};

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
  void onWrite(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo) override {
    // The characteristic already demands an authenticated link; this is a second check.
    if (!connInfo.isAuthenticated()) {
      Serial.println("BLE: write from unpaired phone ignored.");
      return;
    }
    std::string value = chr->getValue();
    if (value.empty()) return;
    String raw = String(value.c_str());

    String cmd = raw;
    cmd.trim();

    // App confirms it received the whole batch
    if (cmd == "ACK") {
      bleAckReceived = true;
      Serial.println("BLE: app acknowledged the batch.");
      return;
    }

    // App forwards a threshold update, same JSON format as the server reply
    if (cmd.startsWith("{")) {
      parseAndUpdateThresholds(cmd);
      return;
    }

    // Wi-Fi provisioning, format "SSID:PASSWORD" (split at the first colon)
    int sep = raw.indexOf(':');
    if (sep > 0) {
      String newSSID = raw.substring(0, sep);
      String newPass = raw.substring(sep + 1);
      preferences.begin("wifi_config", false);
      preferences.putString("ssid", newSSID);
      preferences.putString("password", newPass);
      preferences.end();
      wifiCredsUpdated = true;
      Serial.printf("Saved new Wi-Fi credentials for SSID: %s\n", newSSID.c_str());
    }
  }

  void onSubscribe(NimBLECharacteristic* chr, NimBLEConnInfo& connInfo, uint16_t subValue) override {
    clientSubscribed = (subValue & 0x0001);   // bit 0 = notifications enabled
    Serial.printf("BLE: notifications %s\n", clientSubscribed ? "enabled" : "disabled");
  }
};

// ---------------- Main flow ----------------
void setup() {
  Serial.begin(115200);

  gpio_hold_dis((gpio_num_t)PMOS_GATE_PIN);
  gpio_hold_dis((gpio_num_t)PMS_SET_PIN);
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PMOS_GATE_PIN, OUTPUT);
  pinMode(PMS_SET_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(PMOS_GATE_PIN, LOW);   // sensor rail on
  digitalWrite(PMS_SET_PIN, HIGH);    // PMS5003 awake

  Serial.println("\n--- AeroGuard System Booting ---");
  Serial.printf("Active Thresholds -> PM2.5: %.1f ug/m3 | VOC: %.1f ppm | Temp: %.1f C\n",
                activePM25Threshold, activeVOCThreshold, activeTempThreshold);
  Serial.printf("RTC Queue contains %d/%d pending reading(s)\n", rtcQueueCount, MAX_QUEUE_SIZE);

  dht.begin();
  delay(SENSOR_WARMUP_MS);

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  pmsSerial.setTimeout(100);

  SensorReading current = sampleAllSensors();
  printReading(current);

  if (!current.valid) {
    Serial.println("Skipping queuing & upload phase due to invalid sensor reading.");
    powerDownAndSleep();
    return;
  }

  current.seq = rtcNextSeq++;
  current.t = nowSeconds();

  bool alertActive = (current.pm2_5 > activePM25Threshold) ||
                     (current.voc_ppm > activeVOCThreshold) ||
                     (current.temperature > activeTempThreshold);
  if (alertActive) triggerAlert(current);

  pushToRTCQueue(current);

  bool shouldUpload = hasChangedSignificantly(current) ||
                      alertActive ||
                      (rtcQueueCount >= MAX_QUEUE_SIZE);

  if (!shouldUpload) {
    Serial.println("Threshold not met. Reading queued in RTC RAM for future upload.");
    powerDownAndSleep();
    return;
  }

  static char jsonPayload[PAYLOAD_BUF_SIZE];
  int queuedBefore = rtcQueueCount;
  int included = 0;
  bool dataUploaded = false;

  // Payload is rebuilt before each attempt so age_s stays accurate.
  Serial.println("Upload criteria met. Attempting Wi-Fi upload...");
  if (connectWiFi()) {
    included = buildBatchJsonPayload(jsonPayload, sizeof(jsonPayload), nowSeconds());
    dataUploaded = uploadReadingToVercel(jsonPayload);
  }

  if (!dataUploaded) {
    Serial.println("Wi-Fi failed. Falling back to AeroGuard App over BLE...");
    included = buildBatchJsonPayload(jsonPayload, sizeof(jsonPayload), nowSeconds());
    dataUploaded = transmitViaBLE(jsonPayload);
  }

  if (!dataUploaded && wifiCredsUpdated) {
    Serial.println("New Wi-Fi credentials received over BLE. Retrying Wi-Fi...");
    if (connectWiFi()) {
      included = buildBatchJsonPayload(jsonPayload, sizeof(jsonPayload), nowSeconds());
      dataUploaded = uploadReadingToVercel(jsonPayload);
    }
  }

  if (!dataUploaded) {
    Serial.println("BLE timeout/failure. Falling back to GSM (SIM800L)...");
    included = buildBatchJsonPayload(jsonPayload, sizeof(jsonPayload), nowSeconds());
    dataUploaded = transmitViaGSM(jsonPayload);
  }

  if (dataUploaded) {
    Serial.printf("Batch upload successful (%d reading(s)). Removing them from the queue.\n", included);
    dropOldestFromQueue(included);
    if (included == queuedBefore) {
      lastUploadedPM25 = current.pm2_5;
      lastUploadedVOC = current.voc_ppm;
      lastUploadedTemp = current.temperature;
    }
  } else {
    Serial.printf("All connectivity options failed. %d reading(s) stored in RTC memory.\n",
                  rtcQueueCount);
  }

  powerDownAndSleep();
}

void loop() {}

// ---------------- Time ----------------
// The ESP32 system clock keeps running through deep sleep, so this is a
// monotonic seconds counter across wake cycles (not wall-clock time).
uint32_t nowSeconds() {
  return (uint32_t)time(nullptr);
}

// ---------------- Wi-Fi ----------------
bool connectWiFi() {
  preferences.begin("wifi_config", true);
  String ssid = preferences.getString("ssid", "");
  String password = preferences.getString("password", "");
  preferences.end();

  if (ssid.length() == 0) {
    // TODO: temporary bring-up fallback until BLE/serial provisioning is implemented.
    // Saved Preferences (once set by a real provisioning flow) always win over this.
    ssid = WIFI_FALLBACK_SSID;
    password = WIFI_FALLBACK_PASSWORD;
  }

  if (ssid.length() == 0) {
    Serial.println("No saved Wi-Fi networks found in storage.");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());
  Serial.printf("Connecting to Wi-Fi SSID: %s", ssid.c_str());

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi connection failed.");
    WiFi.disconnect(true);
    WiFi.mode(WIFI_OFF);
    return false;
  }

  Serial.print("Wi-Fi connected! IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

bool uploadReadingToVercel(const char* jsonPayload) {
  WiFiClientSecure client;
  client.setInsecure();   // no certificate validation (prototype limitation)
  HTTPClient http;
  http.setTimeout(10000);
  bool success = false;

  if (http.begin(client, serverApiUrl)) {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-Device-Key", DEVICE_API_KEY);
    int code = http.POST((uint8_t*)jsonPayload, strlen(jsonPayload));
    if (code >= 200 && code < 300) {
      String body = http.getString();
      parseAndUpdateThresholds(body);
      success = true;
    } else {
      Serial.printf("Server returned HTTP %d\n", code);
    }
    http.end();
  }

  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  return success;
}

// ---------------- BLE ----------------
// App protocol:
//   1. Connect. The phone shows a pairing prompt; enter the PIN (BLE_PASSKEY).
//      After the first successful pairing the phone stays bonded and is not asked again.
//   2. Request a larger MTU (Android: requestMtu(247)), enable notifications.
//   3. Collect notification chunks until one equals "END". Concatenate = JSON array.
//   4. Write "ACK" to the characteristic as soon as the JSON is stored on the phone.
//   Optional writes: "SSID:PASSWORD" for Wi-Fi, or threshold JSON from the server.
bool transmitViaBLE(const char* jsonPayload) {
  deviceConnected = false;
  deviceAuthenticated = false;
  clientSubscribed = false;
  bleAckReceived = false;
  peerMTU = 23;

  NimBLEDevice::init("AeroGuard_Node");
  NimBLEDevice::setMTU(247);

  // Security: bonding on, MITM protection on, LE Secure Connections on.
  // DISPLAY_ONLY means the ESP32 "shows" a fixed PIN and the phone must type it.
  NimBLEDevice::setSecurityAuth(true, true, true);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_DISPLAY_ONLY);
  NimBLEDevice::setSecurityPasskey(BLE_PASSKEY);

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService* pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID,
      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::READ_ENC | NIMBLE_PROPERTY::READ_AUTHEN |
      NIMBLE_PROPERTY::WRITE | NIMBLE_PROPERTY::WRITE_ENC | NIMBLE_PROPERTY::WRITE_AUTHEN |
      NIMBLE_PROPERTY::NOTIFY);
  pCharacteristic->setCallbacks(new CharacteristicCallbacks());
  pService->start();

  NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(true);
  NimBLEDevice::startAdvertising();
  Serial.println("BLE advertising started. Waiting up to 45 s for a paired AeroGuard App...");

  bool delivered = false;
  unsigned long start = millis();

  while (millis() - start < BLE_WINDOW_MS) {
    if (deviceConnected && deviceAuthenticated && clientSubscribed) {
      delay(300);   // give the app time to finish MTU negotiation
      if (sendPayloadOverBLE(jsonPayload)) {
        unsigned long ackStart = millis();
        while (!bleAckReceived && deviceConnected && millis() - ackStart < BLE_ACK_TIMEOUT_MS) {
          delay(50);
        }
        delivered = bleAckReceived;
        Serial.println(delivered ? "BLE delivery confirmed by app."
                                 : "BLE data sent but no ACK received.");
      } else {
        Serial.println("BLE send interrupted.");
      }
      break;
    }
    delay(100);
  }

  NimBLEDevice::deinit(true);
  deviceConnected = false;
  deviceAuthenticated = false;
  clientSubscribed = false;
  return delivered;
}

bool sendPayloadOverBLE(const char* jsonPayload) {
  size_t mtu = peerMTU < 23 ? 23 : peerMTU;
  size_t chunkSize = mtu - 3;           // ATT header takes 3 bytes
  if (chunkSize > 244) chunkSize = 244;

  size_t totalLen = strlen(jsonPayload);
  Serial.printf("Sending %u bytes over BLE in %u-byte chunks\n",
                (unsigned)totalLen, (unsigned)chunkSize);

  for (size_t offset = 0; offset < totalLen; offset += chunkSize) {
    if (!deviceConnected) return false;
    size_t len = std::min(chunkSize, totalLen - offset);
    pCharacteristic->setValue((const uint8_t*)(jsonPayload + offset), len);
    if (!pCharacteristic->notify()) {
      delay(50);
      if (!pCharacteristic->notify()) return false;
    }
    delay(20);
  }

  pCharacteristic->setValue((const uint8_t*)"END", 3);
  return pCharacteristic->notify();
}

// ---------------- GSM ----------------
void shutdownGSM() {
  modem.gprsDisconnect();
  modem.poweroff();
  gsmSerial.end();
}

bool transmitViaGSM(const char* jsonPayload) {
  Serial.println("\n--- Waking up SIM800L GSM Module ---");
  gsmSerial.begin(9600, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  if (GSM_PWRKEY_PIN >= 0) {
    pinMode(GSM_PWRKEY_PIN, OUTPUT);
    digitalWrite(GSM_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(GSM_PWRKEY_PIN, HIGH);
    delay(1500);
    digitalWrite(GSM_PWRKEY_PIN, LOW);
    delay(3000);
  }

  Serial.print("Initializing SIM800L modem...");
  if (!modem.restart()) {
    Serial.println(" Failed! Modem not responding.");
    gsmSerial.end();
    return false;
  }
  Serial.println(" OK");

  Serial.print("Waiting for cellular network...");
  if (!modem.waitForNetwork(30000L)) {
    Serial.println(" Network registration timed out.");
    shutdownGSM();
    return false;
  }
  Serial.println(" Connected to network!");

  Serial.printf("Connecting to APN (%s)...", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" GPRS connection failed.");
    shutdownGSM();
    return false;
  }
  Serial.println(" GPRS Connected!");

  if (!gsmClient.connect(gsmServerHost, gsmServerPort)) {
    Serial.println("GSM socket connection failed.");
    shutdownGSM();
    return false;
  }

  // Header lines each end with exactly one CRLF; one blank line ends the header.
  gsmClient.printf("POST %s HTTP/1.1\r\n", gsmServerPath);
  gsmClient.printf("Host: %s\r\n", gsmServerHost);
  gsmClient.print("Content-Type: application/json\r\n");
  gsmClient.printf("X-Device-Key: %s\r\n", DEVICE_API_KEY);
  gsmClient.printf("Content-Length: %u\r\n", (unsigned)strlen(jsonPayload));
  gsmClient.print("Connection: close\r\n\r\n");
  gsmClient.print(jsonPayload);

  String response = "";
  unsigned long lastData = millis();
  while ((gsmClient.connected() || gsmClient.available()) &&
         millis() - lastData < GSM_RESPONSE_TIMEOUT_MS) {
    while (gsmClient.available()) {
      response += (char)gsmClient.read();
      lastData = millis();
    }
    delay(1);
  }
  gsmClient.stop();

  // Status line looks like "HTTP/1.1 200 OK"
  int statusCode = 0;
  int sp = response.indexOf(' ');
  if (response.startsWith("HTTP/") && sp != -1) {
    statusCode = response.substring(sp + 1, sp + 4).toInt();
  }

  bool gsmSuccess = (statusCode >= 200 && statusCode < 300);
  if (gsmSuccess) {
    Serial.println("GSM transmission successful!");
    int bodyStart = response.indexOf("\r\n\r\n");
    parseAndUpdateThresholds(bodyStart != -1 ? response.substring(bodyStart + 4) : response);
  } else {
    Serial.printf("GSM upload failed (HTTP status %d).\n", statusCode);
  }

  shutdownGSM();
  return gsmSuccess;
}

// ---------------- Thresholds ----------------
void parseAndUpdateThresholds(const String& payload) {
  if (payload.indexOf("\"updateThresholds\":true") == -1 &&
      payload.indexOf("\"updateThresholds\": true") == -1) {
    return;
  }

  int pmIndex = payload.indexOf("\"pm25Thresh\":");
  if (pmIndex != -1) {
    float v = payload.substring(pmIndex + 13).toFloat();
    if (v > 0) activePM25Threshold = v;
  }
  int vocIndex = payload.indexOf("\"vocThresh\":");
  if (vocIndex != -1) {
    float v = payload.substring(vocIndex + 12).toFloat();
    if (v > 0) activeVOCThreshold = v;
  }
  int tempIndex = payload.indexOf("\"tempThresh\":");
  if (tempIndex != -1) {
    float v = payload.substring(tempIndex + 13).toFloat();
    if (v > 0) activeTempThreshold = v;
  }

  Serial.printf("Thresholds updated -> PM2.5: %.1f | VOC: %.1f | Temp: %.1f\n",
                activePM25Threshold, activeVOCThreshold, activeTempThreshold);
}

// ---------------- Queue ----------------
void pushToRTCQueue(const SensorReading &r) {
  if (rtcQueueCount < MAX_QUEUE_SIZE) {
    rtcQueue[rtcQueueCount++] = r;
  } else {
    for (int i = 0; i < MAX_QUEUE_SIZE - 1; i++) rtcQueue[i] = rtcQueue[i + 1];
    rtcQueue[MAX_QUEUE_SIZE - 1] = r;
  }
}

void dropOldestFromQueue(int n) {
  if (n <= 0) return;
  if (n >= rtcQueueCount) {
    rtcQueueCount = 0;
    return;
  }
  for (int i = 0; i < rtcQueueCount - n; i++) rtcQueue[i] = rtcQueue[i + n];
  rtcQueueCount -= n;
}

// Builds a JSON array of queued readings, oldest first.
// Returns how many readings fit; never writes past maxLen.
int buildBatchJsonPayload(char* buffer, size_t maxLen, uint32_t nowSec) {
  if (maxLen < 3) {
    if (maxLen > 0) buffer[0] = '\0';
    return 0;
  }

  size_t offset = 0;
  int included = 0;
  buffer[offset++] = '[';

  for (int i = 0; i < rtcQueueCount; i++) {
    const SensorReading &r = rtcQueue[i];
    uint32_t age = (nowSec >= r.t) ? (nowSec - r.t) : 0;

    char item[256];
    int n = snprintf(item, sizeof(item),
        "%s{\"seq\":%lu,\"age_s\":%lu,\"pm1_0\":%.1f,\"pm2_5\":%.1f,\"pm10\":%.1f,"
        "\"voc_ppm\":%.1f,\"temperature\":%.1f,\"humidity\":%.1f}",
        (included > 0) ? "," : "",
        (unsigned long)r.seq, (unsigned long)age,
        r.pm1_0, r.pm2_5, r.pm10, r.voc_ppm, r.temperature, r.humidity);

    if (n < 0 || (size_t)n >= sizeof(item)) break;
    if (offset + (size_t)n + 2 > maxLen) break;   // keep room for ']' and '\0'

    memcpy(buffer + offset, item, n);
    offset += n;
    included++;
  }

  buffer[offset++] = ']';
  buffer[offset] = '\0';

  Serial.printf("Payload built: %d of %d reading(s), %u bytes\n",
                included, rtcQueueCount, (unsigned)offset);
  return included;
}

// ---------------- Sensors ----------------
SensorReading sampleAllSensors() {
  SensorReading r = {};
  r.valid = true;

  r.humidity = dht.readHumidity();
  r.temperature = dht.readTemperature();
  if (isnan(r.humidity) || isnan(r.temperature)) r.valid = false;

  if (!readPMS5003(r.pm1_0, r.pm2_5, r.pm10)) r.valid = false;

  r.voc_ppm = r.valid ? readMQ135Compensated(r.humidity, r.temperature) : 0.0f;
  return r;
}

bool readPMS5003(float &pm1, float &pm25, float &pm10) {
  const int FRAME_LEN = 32;
  uint8_t buf[FRAME_LEN];

  while (pmsSerial.available()) pmsSerial.read();   // drop stale bytes

  unsigned long start = millis();
  while (millis() - start < 3000) {
    if (pmsSerial.available() < 1) {
      delay(1);
      continue;
    }
    if (pmsSerial.read() != 0x42) continue;

    unsigned long t = millis();
    while (pmsSerial.available() < 1 && millis() - t < 100) delay(1);
    if (pmsSerial.read() != 0x4D) continue;

    buf[0] = 0x42;
    buf[1] = 0x4D;
    if (pmsSerial.readBytes(buf + 2, FRAME_LEN - 2) != FRAME_LEN - 2) continue;

    uint16_t frameLen = (buf[2] << 8) | buf[3];
    if (frameLen != 28) continue;

    uint16_t checksum = 0;
    for (int i = 0; i < FRAME_LEN - 2; i++) checksum += buf[i];
    uint16_t received = (buf[30] << 8) | buf[31];
    if (checksum != received) continue;

    // Atmospheric-environment values (bytes 10 to 15)
    pm1 = (buf[10] << 8) | buf[11];
    pm25 = (buf[12] << 8) | buf[13];
    pm10 = (buf[14] << 8) | buf[15];
    return true;
  }
  return false;
}

float readMQ135Compensated(float humidity, float temperature) {
  uint32_t mvSum = 0;
  for (int i = 0; i < MQ135_SAMPLES; i++) {
    mvSum += analogReadMilliVolts(MQ135_PIN);
    delay(5);
  }
  float vOut = (mvSum / (float)MQ135_SAMPLES) / 1000.0f * MQ135_DIVIDER_GAIN;

  if (vOut <= 0.01f) return 0.0f;
  if (vOut >= MQ135_VC - 0.01f) return MQ135_MAX_PPM;   // saturated

  float rs = ((MQ135_VC - vOut) / vOut) * MQ135_RLOAD;
  float ratio = rs / MQ135_RZERO;
  float ppm = 116.6020682f * powf(ratio, -2.769034857f);

  float humidityCorrection = 1.0f - ((humidity - 33.0f) * 0.002f);
  if (humidityCorrection < 0.5f) humidityCorrection = 0.5f;
  ppm *= humidityCorrection;

  if (isnan(ppm) || ppm < 0.0f) ppm = 0.0f;
  if (ppm > MQ135_MAX_PPM) ppm = MQ135_MAX_PPM;
  return ppm;
}

bool hasChangedSignificantly(const SensorReading &r) {
  if (lastUploadedPM25 < 0 || lastUploadedVOC < 0 || lastUploadedTemp < 0) return true;
  float pm25ChangePct = fabs(r.pm2_5 - lastUploadedPM25) / max(1.0f, lastUploadedPM25) * 100.0f;
  float vocChangePct = fabs(r.voc_ppm - lastUploadedVOC) / max(1.0f, lastUploadedVOC) * 100.0f;
  float tempChangePct = fabs(r.temperature - lastUploadedTemp) / max(1.0f, lastUploadedTemp) * 100.0f;
  return (pm25ChangePct > CHANGE_THRESHOLD_PCT) ||
         (vocChangePct > CHANGE_THRESHOLD_PCT) ||
         (tempChangePct > CHANGE_THRESHOLD_PCT);
}

void triggerAlert(const SensorReading &r) {
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

// ---------------- Power ----------------
void powerDownAndSleep() {
  digitalWrite(PMS_SET_PIN, LOW);
  pmsSerial.end();
  pinMode(PMS_RX_PIN, INPUT);
  pinMode(PMS_TX_PIN, INPUT);
  digitalWrite(PMOS_GATE_PIN, HIGH);   // sensor rail off

  gpio_hold_en((gpio_num_t)PMS_SET_PIN);
  gpio_hold_en((gpio_num_t)PMOS_GATE_PIN);
  gpio_deep_sleep_hold_en();

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SEC * 1000000ULL);
  esp_deep_sleep_start();
}

void printReading(const SensorReading &r) {
  Serial.printf("Temp: %.1f C | RH: %.1f%%\n", r.temperature, r.humidity);
  Serial.printf("PM1.0: %.1f | PM2.5: %.1f | PM10: %.1f ug/m3\n", r.pm1_0, r.pm2_5, r.pm10);
  Serial.printf("Gas (MQ135 estimate): %.1f ppm\n", r.voc_ppm);
}
