#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>

#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include <DHT.h>
#include <HardwareSerial.h>
#include <math.h>
#include "driver/gpio.h"
#include <NimBLEDevice.h>

#define GSM_RX_PIN      26
#define GSM_TX_PIN      27
#define GSM_PWRKEY_PIN  5

const char apn[]      = "internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

const char gsmServerHost[] = "ambiant-air-monitor-frontend.vercel.app";
const int  gsmServerPort   = 443;
const char gsmServerPath[] = "/api/readings";

HardwareSerial gsmSerial(1);
TinyGsm modem(gsmSerial);
TinyGsmClient gsmClient(modem);

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

NimBLEServer* pServer = nullptr;
NimBLECharacteristic* pCharacteristic = nullptr;
bool deviceConnected = false;

class MyServerCallbacks: public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo) override {
      deviceConnected = true;
      Serial.println("BLE Client Connected.");
    }
    void onDisconnect(NimBLEServer* pServer, NimBLEConnInfo& connInfo, int reason) override {
      deviceConnected = false;
      Serial.println("BLE Client Disconnected.");
    }
};

#define DHTPIN 4
#define DHTTYPE DHT22
#define MQ135_PIN 34
#define BUZZER_PIN 25
#define PMOS_GATE_PIN 23
#define PMS_SET_PIN 18
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

#define WIFI_SSID "Vee_baby"
#define WIFI_PASSWORD "tokenapprovalchecker"

const char* serverApiUrl = "https://ambiant-air-monitor-frontend.vercel.app/api/readings";

#define MQ135_RLOAD 10.0
#define MQ135_RZERO 76.63
#define CHANGE_THRESHOLD_PCT 10.0

RTC_DATA_ATTR float activePM25Threshold = 45.0;
RTC_DATA_ATTR float activeVOCThreshold  = 500.0;
RTC_DATA_ATTR float activeTempThreshold = 26.0;

#define SLEEP_INTERVAL_SEC 60

struct SensorReading {
  bool valid;
  float humidity;
  float temperature;
  float pm1_0;
  float pm2_5;
  float pm10;
  float voc_ppm;
};

#define MAX_QUEUE_SIZE 60

RTC_DATA_ATTR SensorReading rtcQueue[MAX_QUEUE_SIZE];
RTC_DATA_ATTR int rtcQueueCount = 0;

RTC_DATA_ATTR float lastUploadedPM25 = -1.0;
RTC_DATA_ATTR float lastUploadedVOC  = -1.0;
RTC_DATA_ATTR float lastUploadedTemp = -1.0;

DHT dht(DHTPIN, DHTTYPE);
HardwareSerial pmsSerial(2);

SensorReading sampleAllSensors();
bool readPMS5003(float &pm1, float &pm25, float &pm10);
float readMQ135Compensated(float humidity, float temperature);
bool hasChangedSignificantly(const SensorReading &r);
void triggerAlert(const SensorReading &r);
bool connectWiFi();

void pushToRTCQueue(const SensorReading &r);
size_t buildBatchJsonPayload(char* buffer, size_t maxLen);
void parseAndUpdateThresholds(const String& payload);

bool uploadReadingToVercel(const char* jsonPayload); 
bool transmitViaBLE(const char* jsonPayload);
bool transmitViaGSM(const char* jsonPayload);

void powerDownAndSleep();
void printReading(const SensorReading &r);

void setup() {
  Serial.begin(115200);

  gpio_hold_dis((gpio_num_t)PMOS_GATE_PIN);
  gpio_hold_dis((gpio_num_t)PMS_SET_PIN);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(PMOS_GATE_PIN, OUTPUT);
  pinMode(PMS_SET_PIN, OUTPUT);

  digitalWrite(PMOS_GATE_PIN, LOW);
  digitalWrite(PMS_SET_PIN, HIGH);

  Serial.println("\n--- AeroGuard System Booting ---");
  Serial.printf("Active Thresholds -> PM2.5: %.1f ug/m3 | VOC: %.1f ppm | Temp: %.1f C\n", 
                activePM25Threshold, activeVOCThreshold, activeTempThreshold);
  Serial.printf("RTC Queue contains %d / %d pending reading(s)\n", rtcQueueCount, MAX_QUEUE_SIZE);
  Serial.println("Energized rail & SET pin. Warming up sensors (10s)...");
  delay(10000); 

  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);
  dht.begin();

  SensorReading currentData = sampleAllSensors();
  printReading(currentData);

  if (currentData.valid) {
    bool alertActive = (currentData.pm2_5 > activePM25Threshold) || 
                       (currentData.voc_ppm > activeVOCThreshold) || 
                       (currentData.temperature > activeTempThreshold);
    if (alertActive) {
      triggerAlert(currentData);
    }

    pushToRTCQueue(currentData);

    bool shouldUpload = hasChangedSignificantly(currentData) || 
                         alertActive || 
                         (rtcQueueCount >= MAX_QUEUE_SIZE);

    if (shouldUpload) {
      Serial.println("Upload criteria met. Constructing batch JSON payload...");
      
      static char jsonPayload[6000];
      buildBatchJsonPayload(jsonPayload, sizeof(jsonPayload));

      bool dataUploaded = false;

      Serial.println("Attempting WiFi upload...");
      if (connectWiFi()) {
        dataUploaded = uploadReadingToVercel(jsonPayload);
      }
    
      if (!dataUploaded) {
        Serial.println("WiFi failed. Falling back to On-Demand BLE...");
        dataUploaded = transmitViaBLE(jsonPayload);
      }

      if (!dataUploaded) {
        Serial.println("BLE timeout/failure. Falling back to GSM (SIM7600)...");
        dataUploaded = transmitViaGSM(jsonPayload);
      }

      if (dataUploaded) {
        Serial.println("Batch upload successful! Clearing RTC Queue.");
        rtcQueueCount = 0;
        lastUploadedPM25 = currentData.pm2_5;
        lastUploadedVOC  = currentData.voc_ppm;
        lastUploadedTemp = currentData.temperature;
      } else {
        Serial.printf("All connectivity options failed. %d reading(s) stored in RTC memory for next wake.\n", rtcQueueCount);
      }
    } else {
      Serial.println("Threshold not met. Reading queued in RTC RAM for future upload.");
    }
  } else {
    Serial.println("Skipping queuing & upload phase due to invalid sensor reading.");
  }
  
  powerDownAndSleep();
}

void loop() {}

void parseAndUpdateThresholds(const String& payload) {
  if (payload.indexOf("\"updateThresholds\":true") != -1 || payload.indexOf("\"updateThresholds\": true") != -1) {
    Serial.println("\n>>> PERSONALIZED THRESHOLD UPDATE DETECTED FROM SERVER <<<");

    int pmIndex = payload.indexOf("\"pm25Thresh\":");
    if (pmIndex != -1) {
      float newVal = payload.substring(pmIndex + 13).toFloat();
      if (newVal > 0) {
        activePM25Threshold = newVal;
        Serial.printf("  -> Updated RTC activePM25Threshold: %.1f ug/m3\n", activePM25Threshold);
      }
    }

    int vocIndex = payload.indexOf("\"vocThresh\":");
    if (vocIndex != -1) {
      float newVal = payload.substring(vocIndex + 12).toFloat();
      if (newVal > 0) {
        activeVOCThreshold = newVal;
        Serial.printf("  -> Updated RTC activeVOCThreshold: %.1f ppm\n", activeVOCThreshold);
      }
    }

    int tempIndex = payload.indexOf("\"tempThresh\":");
    if (tempIndex != -1) {
      float newVal = payload.substring(tempIndex + 13).toFloat();
      if (newVal > 0) {
        activeTempThreshold = newVal;
        Serial.printf("  -> Updated RTC activeTempThreshold: %.1f C\n", activeTempThreshold);
      }
    }
  }
}

void pushToRTCQueue(const SensorReading &r) {
  if (rtcQueueCount < MAX_QUEUE_SIZE) {
    rtcQueue[rtcQueueCount++] = r;
  } else {
    for (int i = 0; i < MAX_QUEUE_SIZE - 1; i++) {
      rtcQueue[i] = rtcQueue[i + 1];
    }
    rtcQueue[MAX_QUEUE_SIZE - 1] = r;
    Serial.println("RTC Queue full. Overwrote oldest reading.");
  }
}

size_t buildBatchJsonPayload(char* buffer, size_t maxLen) {
  size_t offset = 0;
  offset += snprintf(buffer + offset, maxLen - offset, "[");
  
  for (int i = 0; i < rtcQueueCount; i++) {
    offset += snprintf(buffer + offset, maxLen - offset,
      "{\"pm1_0\":%.1f,\"pm2_5\":%.1f,\"pm10\":%.1f,\"voc_ppm\":%.1f,\"temperature\":%.1f,\"humidity\":%.1f}%s",
      rtcQueue[i].pm1_0, rtcQueue[i].pm2_5, rtcQueue[i].pm10, 
      rtcQueue[i].voc_ppm, rtcQueue[i].temperature, rtcQueue[i].humidity,
      (i < rtcQueueCount - 1) ? "," : "");
    
    if (offset >= maxLen - 1) break;
  }
  
  snprintf(buffer + offset, maxLen - offset, "]");
  return strlen(buffer);
}

SensorReading sampleAllSensors() {
  SensorReading r;
  r.valid = true;
  r.humidity = dht.readHumidity();
  r.temperature = dht.readTemperature();

  if (isnan(r.humidity) || isnan(r.temperature)) {
    Serial.println("DHT22 read failed!");
    r.valid = false;
  }

  if (!readPMS5003(r.pm1_0, r.pm2_5, r.pm10)) {
    Serial.println("PMS5003 read failed!");
    r.valid = false;
  }

  if (r.valid) {
    r.voc_ppm = readMQ135Compensated(r.humidity, r.temperature);
  } else {
    r.voc_ppm = 0.0;
  }

  return r;
}

bool readPMS5003(float &pm1, float &pm25, float &pm10) {
  const int FRAME_LEN = 32;
  uint8_t buf[FRAME_LEN];

  while (pmsSerial.available()) pmsSerial.read();

  unsigned long start = millis();
  while (pmsSerial.available() < FRAME_LEN) {
    if (millis() - start > 3000) return false;
  }

  if (pmsSerial.peek() != 0x42) {
    while (pmsSerial.available() && pmsSerial.peek() != 0x42) {
      pmsSerial.read();
    }
    if (pmsSerial.available() < FRAME_LEN) return false;
  }

  pmsSerial.readBytes(buf, FRAME_LEN);
  if (buf[0] != 0x42 || buf[1] != 0x4D) return false;

  uint16_t checksum = 0;
  for (int i = 0; i < FRAME_LEN - 2; i++) checksum += buf[i];
  uint16_t received = (buf[30] << 8) | buf[31];
  if (checksum != received) return false;

  pm1 = (buf[10] << 8) | buf[11];
  pm25 = (buf[12] << 8) | buf[13];
  pm10 = (buf[14] << 8) | buf[15];
  return true;
}

float readMQ135Compensated(float humidity, float temperature) {
  int raw = analogRead(MQ135_PIN);
  float voltage = raw * (3.3f / 4095.0f);

  if (voltage <= 0.01f) return 0.0f;

  float rs = ((3.3f - voltage) / voltage) * MQ135_RLOAD;
  float ratio = rs / MQ135_RZERO;
  float ppm = 116.6020682f * powf(ratio, -2.769034857f);
  float humidityCorrection = 1.0f - ((humidity - 33.0f) * 0.002f);
  if (humidityCorrection < 0.5f) humidityCorrection = 0.5f;
  ppm *= humidityCorrection;
  return ppm;
}

bool hasChangedSignificantly(const SensorReading &r) {
  if (lastUploadedPM25 < 0 || lastUploadedVOC < 0 || lastUploadedTemp < 0) return true;
  float pm25ChangePct = fabs(r.pm2_5 - lastUploadedPM25) / max(1.0f, lastUploadedPM25) * 100.0f;
  float vocChangePct = fabs(r.voc_ppm - lastUploadedVOC) / max(1.0f, lastUploadedVOC) * 100.0f;
  float tempChangePct = fabs(r.temperature - lastUploadedTemp) / max(1.0f, lastUploadedTemp) * 100.0f;
  return (pm25ChangePct > CHANGE_THRESHOLD_PCT) || (vocChangePct > CHANGE_THRESHOLD_PCT) || (tempChangePct > CHANGE_THRESHOLD_PCT);
}

void triggerAlert(const SensorReading &r) {
  Serial.println(">>> ACTIVE THRESHOLD EXCEEDED - ALERT TRIGGERED <<<");
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(200);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  unsigned long start = millis();

  while (WiFi.status() != WL_CONNECTED && millis() - start < 12000) {
    delay(250);
    Serial.print(".");
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi connection failed");
    WiFi.disconnect(true);
    return false;
  }
  Serial.print("Wi-Fi connected, IP: ");
  Serial.println(WiFi.localIP());
  return true;
}

bool uploadReadingToVercel(const char* jsonPayload) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  bool success = false;

  if (http.begin(client, serverApiUrl)) {
    http.addHeader("Content-Type", "application/json");

    int httpResponseCode = http.POST(jsonPayload);

    if (httpResponseCode > 0 && httpResponseCode < 400) {
      Serial.printf("Data uploaded to Vercel via WiFi. Response code: %d\n", httpResponseCode);
      
      String responseBody = http.getString();
      Serial.printf("Server Response: %s\n", responseBody.c_str());
      
      parseAndUpdateThresholds(responseBody);
      
      success = true;
    } else {
      Serial.printf("Error sending POST request: %s\n", http.errorToString(httpResponseCode).c_str());
    }
    http.end();
  }
  
  WiFi.disconnect(true);
  return success;
}

bool transmitViaBLE(const char* jsonPayload) {
  NimBLEDevice::init("AeroGuard_Node");
  
  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  NimBLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY
                    );
  
  pService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->enableScanResponse(false);
  
  NimBLEDevice::startAdvertising();
  
  Serial.println("BLE Advertising started. Waiting up to 30s for connection...");

  unsigned long startWaitTime = millis();
  bool dataSent = false;

  while (millis() - startWaitTime < 30000) {
    if (deviceConnected) {
      delay(500);
      pCharacteristic->setValue((const uint8_t*)jsonPayload, strlen(jsonPayload));
      pCharacteristic->notify();
      Serial.println("Success! Data transmitted over BLE.");
      dataSent = true;
      delay(1000); 
      break;
    }
    delay(100);
  }

  NimBLEDevice::deinit(true);
  deviceConnected = false; 
  return dataSent;
}

bool transmitViaGSM(const char* jsonPayload) {
  Serial.println("\n--- Waking up SIM7600 GSM Module ---");
  bool gsmSuccess = false;

  gsmSerial.begin(115200, SERIAL_8N1, GSM_RX_PIN, GSM_TX_PIN);

  if (GSM_PWRKEY_PIN >= 0) {
    pinMode(GSM_PWRKEY_PIN, OUTPUT);
    digitalWrite(GSM_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(GSM_PWRKEY_PIN, HIGH);
    delay(1000); 
    digitalWrite(GSM_PWRKEY_PIN, LOW);
    delay(3000); 
  }

  Serial.print("Initializing SIM7600 modem...");
  if (!modem.restart()) {
    Serial.println(" Failed! Modem not responding.");
    return false;
  }
  Serial.println(" OK");

  Serial.print("Waiting for network registration...");
  if (!modem.waitForNetwork(30000L)) {
    Serial.println(" Network registration failed/timed out.");
    modem.poweroff();
    return false;
  }
  Serial.println(" Connected to cellular network!");

  Serial.printf("Connecting to APN (%s)...", apn);
  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    Serial.println(" GPRS/LTE connection failed.");
    modem.poweroff();
    return false;
  }
  Serial.println(" GPRS Connected!");

  Serial.println("Sending HTTPS POST request to Vercel via SIM7600 Direct Socket...");

  if (!gsmClient.connect(gsmServerHost, gsmServerPort)) {
    Serial.println("GSM Socket Connection Failed.");
    modem.gprsDisconnect();
    modem.poweroff();
    gsmSerial.end();
    return false;
  }

  gsmClient.printf("POST %s HTTP/1.1\r\n", gsmServerPath);
  gsmClient.printf("Host: %s\r\n", gsmServerHost);
  gsmClient.println("Content-Type: application/json");
  gsmClient.printf("Content-Length: %d\r\n", strlen(jsonPayload));
  gsmClient.println("Connection: close\r\n");
  gsmClient.print(jsonPayload);

  unsigned long timeout = millis();
  String responseBody = "";
  while (gsmClient.connected() && millis() - timeout < 10000) {
    while (gsmClient.available()) {
      char c = gsmClient.read();
      responseBody += c;
      timeout = millis();
    }
  }
  gsmClient.stop();

  if (responseBody.indexOf("200 OK") != -1 || responseBody.indexOf("201") != -1 || responseBody.indexOf("\"success\":true") != -1) {
    Serial.println("GSM Transmission Successful!");
    Serial.printf("GSM Response: %s\n", responseBody.c_str());
    
    parseAndUpdateThresholds(responseBody);
    gsmSuccess = true;
  } else {
    Serial.println("GSM Upload Failed or timed out.");
    if (responseBody.length() > 0) {
      Serial.printf("Raw Response: %s\n", responseBody.c_str());
    }
  }

  Serial.println("Disconnecting cellular network & powering down SIM7600...");
  modem.gprsDisconnect();
  modem.poweroff();
  gsmSerial.end();

  return gsmSuccess;
}

void powerDownAndSleep() {
  Serial.println("Executing dual-stage sensor power down & line isolation...");

  digitalWrite(PMS_SET_PIN, LOW);

  pmsSerial.end();
  pinMode(PMS_RX_PIN, INPUT); 
  pinMode(PMS_TX_PIN, INPUT); 

  digitalWrite(PMOS_GATE_PIN, HIGH);

  gpio_hold_en((gpio_num_t)PMS_SET_PIN);    
  gpio_hold_en((gpio_num_t)PMOS_GATE_PIN); 
  gpio_deep_sleep_hold_en();

  Serial.println("Sensors de-energized, SET pin low, UART isolated.");
  Serial.printf("Entering deep sleep for %d seconds\n", SLEEP_INTERVAL_SEC);
  Serial.flush();

  esp_sleep_enable_timer_wakeup((uint64_t)SLEEP_INTERVAL_SEC * 1000000ULL);
  esp_deep_sleep_start();
}

void printReading(const SensorReading &r) {
  Serial.println("---- Sensor Reading ----");
  Serial.printf("Temp: %.1f C RH: %.1f %%\n", r.temperature, r.humidity);
  Serial.printf("PM1.0: %.1f PM2.5: %.1f PM10: %.1f ug/m3\n", r.pm1_0, r.pm2_5, r.pm10);
  Serial.printf("VOC/CO2 index: %.1f ppm\n", r.voc_ppm);
  Serial.println("-------------------------");
}