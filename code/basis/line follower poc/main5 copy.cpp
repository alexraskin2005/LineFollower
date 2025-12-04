#include <Arduino.h>
#include "EEPROMAnything.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>

// Control loop timing
unsigned long previous = 0;
unsigned long calculationTime = 0;

// Data streaming
uint8_t cyclesSinceLastSend = 0;
const uint8_t cyclesPerSend = 4;

unsigned long lastSeenLineTime = 1000;
unsigned long lapStartTime = 0;
unsigned long lapTime = 0;
uint8_t crossingCount = 0;
bool crossingDetected = false;

// State variables
bool running = false;
int forwardSpeed = 0;
bool motorsEnabled = false;
bool ledWifiStatus = false;

// PID variables
float error = 0.0f;
float linePosition = 0.0f;
float correction = 0.0f;
float scale = 1.0f;
float lastError = 0.0f;
float iTerm = 0.0f;

// Motor variables
int m1 = 0, m2 = 0;
uint8_t m1_pwm_fwd = 0, m1_pwm_bwd = 0;
uint8_t m2_pwm_fwd = 0, m2_pwm_bwd = 0;

// Pin definitions
const uint8_t pinWifiStatus = 42;
const uint8_t motorPins[4] = {17, 18, 45, 46}; // m1_fwd, m1_bwd, m2_fwd, m2_bwd
const uint8_t sensorPins[8] = {10, 9, 8, 7, 6, 5, 4, 2};

// Sensor data
int sensors[8] = {0};
float mappedSensors[8] = {0.0f};

// Configuration structure
struct param_t {
  unsigned long cycleTime;
  char ssid[32];
  char password[64];
  int defaultSpeed;
  float diff;
  int blackValues[8];
  int whiteValues[8];
  double kp;
  double ki;
  double kd;
  int maxTimeWithoutLine;
  int crossingsPerLap;
} params;

// WiFi objects
WebServer server(80);
DNSServer dnsServer;
WiFiServer serverTCP(1234);
WiFiClient client;

// String buffer for data streaming
String dataBuffer;

// Forward declarations
void setupMotors();
void readSensors();
void computePID();
void updateMotors();
void dataStreaming();
void startRunTimeServer();
void wifiTask(void *parameter);
String buildStatusJSON();

// Task handle
TaskHandle_t wifiTaskHandle = NULL;

// ------------------- WiFi / TCP Task -------------------
void wifiTask(void *parameter) {
  Serial.print("WiFi task running on core: ");
  Serial.println(xPortGetCoreID());
  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      server.handleClient();
      dataStreaming();
    }
    vTaskDelay(1);
  }
}

// ------------------- WiFi Connect -------------------
bool tryConnectWithParams(unsigned long timeoutMs = 15000) {
  if (strlen(params.ssid) == 0) {
    Serial.println("No SSID configured");
    return false;
  }

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);

  Serial.print("Connecting to ");
  Serial.println(params.ssid);

  WiFi.begin(params.ssid, params.password);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected, IP: ");
      Serial.println(WiFi.localIP());

      if (MDNS.begin("linefollower")) {
        MDNS.addService("http", "tcp", 80);
        Serial.println("mDNS started: http://linefollower.local");
      }

      return true;
    }
    delay(200);
    Serial.print(".");
  }

  Serial.println("\nConnection failed");
  return false;
}

// ------------------- Config Portal -------------------
void startConfigPortal(unsigned long portalTimeoutMs = 300000UL) {
  Serial.println("Starting config portal...");

  const char* apName = "lineFollower_Wifi_Config";
  const char* apPass = "configureme";

  IPAddress apIP(192,168,4,1);
  IPAddress apGateway(192,168,4,1);
  IPAddress apSubnet(255,255,255,0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(apName, apPass);

  Serial.print("AP started: ");
  Serial.println(WiFi.softAPIP());

  dnsServer.start(53, "*", apIP);

  server.on("/", [](){
    String page = "<html><body><h2>WiFi Setup</h2>"
                  "<form action=\"/save\" method=\"POST\">"
                  "SSID:<br><input name=\"ssid\" value=\"";
    page += params.ssid;
    page += "\"><br>Password:<br>"
            "<input type=\"password\" name=\"pwd\"><br><br>"
            "<input type=\"submit\" value=\"Save\"></form></body></html>";
    
    server.send(200, "text/html", page);
  });

  server.on("/save", HTTP_POST, [](){
    if (server.hasArg("ssid") && server.hasArg("pwd")) {
      server.arg("ssid").toCharArray(params.ssid, sizeof(params.ssid));
      server.arg("pwd").toCharArray(params.password, sizeof(params.password));
      EEPROM_writeAnything(0, params);

      server.send(200, "text/html", "Saved. Restarting...");
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, "text/plain", "Missing fields");
    }
  });

  server.begin();
  Serial.println("Config portal web server started");

  unsigned long started = millis();
  while (millis() - started < portalTimeoutMs) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(10);
    digitalWrite(pinWifiStatus, !digitalRead(pinWifiStatus));
  }

  Serial.println("Config portal timeout");
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
}

// ------------------- Start WiFi -------------------
void startWifi() {
  Serial.println("Starting WiFi...");

  if (tryConnectWithParams()) {
    Serial.println("WiFi connected.");
    WiFi.softAPdisconnect(true);
    WiFi.mode(WIFI_STA);
    return;
  }

  Serial.println("Falling back to config portal...");
  startConfigPortal();
}

// ------------------- Build JSON -------------------
String buildStatusJSON() {
    String json;
    json = "{\"timeStamp\":";
    json += millis();
    json += ",\"ssid\":\"";
    json += params.ssid;
    json += "\",\"cycleTime\":";
    json += params.cycleTime;
    json += ",\"calculationTime\":";
    json += calculationTime;
    json += ",\"running\":";
    json += running;
    json += ",\"motorsEnabled\":";
    json += motorsEnabled;
    json += ",\"kp\":";
    json += params.kp;
    
    float cycleTimeInSec = params.cycleTime * 0.000001f;
    json += ",\"ki\":";
    json += (params.ki / cycleTimeInSec);
    json += ",\"kd\":";
    json += (params.kd * cycleTimeInSec);
    json += ",\"linePosition\":";
    json += linePosition;
    json += ",\"error\":";
    json += error;
    json += ",\"correction\":";
    json += correction;
    json += ",\"motor1\":";
    json += m1;
    json += ",\"motor2\":";
    json += m2;
    json += ",\"defaultSpeed\":";
    json += params.defaultSpeed;
    json += ",\"diff\":";
    json += params.diff;
    json += ",\"maxTimeWithoutLine\":";
    json += params.maxTimeWithoutLine;
    json += ",\"crossingsPerLap\":";
    json += params.crossingsPerLap;
    json += ",\"lapTime\":";
    json += lapTime;
    json += ",\"crossingCount\":";
    json += crossingCount;
    
    // Arrays
    json += ",\"rawSensors\":[";
    for (uint8_t i = 0; i < 8; i++) {
        json += sensors[i];
        if (i < 7) json += ',';
    }

    json += "],\"sensors\":[";
    for (uint8_t i = 0; i < 8; i++) {
        json += mappedSensors[i];
        if (i < 7) json += ',';
    }

    json += "],\"blackValues\":[";
    for (uint8_t i = 0; i < 8; i++) {
        json += params.blackValues[i];
        if (i < 7) json += ',';
    }

    json += "],\"whiteValues\":[";
    for (uint8_t i = 0; i < 8; i++) {
        json += params.whiteValues[i];
        if (i < 7) json += ',';
    }

    json += "]}";
    return json;
}

// ------------------- Data Streaming -------------------
void dataStreaming() {
    if (!client || !client.connected()) {
        client = serverTCP.available();
    }

    if (++cyclesSinceLastSend < cyclesPerSend) return;
    cyclesSinceLastSend = 0;

    if (!client || !client.connected()) return;

    dataBuffer = buildStatusJSON();
    client.println(dataBuffer);
}

// ------------------- HTTP Server -------------------
void startRunTimeServer() {
  Serial.println("Setting up HTTP server...");

  server.on("/", HTTP_GET, [](){
    server.send(200, "text/plain", "LineFollower Controller - OK");
  });

  server.on("/test", HTTP_GET, [](){
    server.send(200, "text/plain", "Server is alive!");
  });

  server.on("/start", HTTP_ANY, [](){
    running = true;
    server.send(200, "text/plain", "Started");
  });

  server.on("/stop", HTTP_ANY, [](){
    running = false;
    iTerm = 0;
    server.send(200, "text/plain", "Stopped");
  });

  server.on("/enable", HTTP_ANY, [](){
    motorsEnabled = true;
    server.send(200, "text/plain", "Motors enabled");
  });

  server.on("/disable", HTTP_ANY, [](){
    motorsEnabled = false;
    setupMotors();
    server.send(200, "text/plain", "Motors disabled");
  });

  server.on("/debug", HTTP_GET, [](){
    server.send(200, "application/json", buildStatusJSON());
  });

  // ... Include your /set, /calibrate, /status routes here as before ...

  server.begin();
  Serial.println("HTTP server started on port 80");
}

// ------------------- Utility Functions -------------------
inline bool checkAllBlack(const float readings[8]) {
  for (uint8_t i = 1; i < 7; i++) {
    if (readings[i] < 600.0f) return false;
  }
  return true;
}

inline bool checkAllWhite(const float readings[8]) {
  for (uint8_t i = 1; i < 7; i++) {
    if (readings[i] > 600.0f) return false;
  }
  return true;
}

float getLinePosition(const int readings[8]) {
  uint8_t index = 1;
  for (uint8_t i = 2; i <= 6; i++) {
    if (readings[i] > readings[index]) index = i;
  }

  if (index == 1) index = 2;
  else if (index == 6) index = 5;

  long sZero = readings[index];
  long sMinusOne = readings[index - 1];
  long sPlusOne = readings[index + 1];

  float b = (sPlusOne - sMinusOne) * 0.5f;
  float a = sPlusOne - b - sZero;

  if (fabs(a) < 0.0001f) a = 0.0001f;

  float position = -b / (2.0f * a);
  if (isnan(position) || isinf(position)) position = 0.0f;
  
  position += index - 3.5f;
  position *= 8.0f;
  
  return position;
}

void computePID() {
  float output = error * params.kp;

  iTerm += params.ki * error;
  iTerm = constrain(iTerm, -510.0f, 510.0f);
  output += iTerm;

  output += params.kd * (error - lastError);
  lastError = error;

  correction = constrain(output, -510.0f, 510.0f);
}

void setupMotors() {
  for (uint8_t i = 0; i < 4; i++) {
    analogWrite(motorPins[i], 0);
  }
}

void readSensors() {
  for (uint8_t i = 0; i < 8; i++) {
    if (sensorPins[i]) {
      sensors[i] = analogRead(sensorPins[i]);
      mappedSensors[i] = map(sensors[i], params.whiteValues[i], params.blackValues[i], 0, 1000);
    }
  }
}

void updateMotors() {
  if (!motorsEnabled) {
    setupMotors();
    return;
  }

  m1_pwm_fwd = m1 > 0 ? m1 : 0;
  m1_pwm_bwd = m1 < 0 ? -m1 : 0;
  m2_pwm_fwd = m2 > 0 ? m2 : 0;
  m2_pwm_bwd = m2 < 0 ? -m2 : 0;

  analogWrite(motorPins[0], m1_pwm_fwd);
  analogWrite(motorPins[1], m1_pwm_bwd);
  analogWrite(motorPins[2], m2_pwm_fwd);
  analogWrite(motorPins[3], m2_pwm_bwd);
}

// ------------------- Setup -------------------
void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n===== LineFollower ESP32-S3 Starting =====");

  setupMotors();
  EEPROM.begin(512);
  EEPROM_readAnything(0, params);

  for (uint8_t i = 0; i < 8; i++) {
    if (sensorPins[i]) pinMode(sensorPins[i], INPUT);
  }
  pinMode(pinWifiStatus, OUTPUT);

  dataBuffer.reserve(600);

  startWifi();
  delay(1000);

  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(pinWifiStatus, HIGH);

    serverTCP.begin();
    serverTCP.setNoDelay(true);
    Serial.print("TCP server started on 1234, IP: ");
    Serial.println(WiFi.localIP());

    startRunTimeServer();

    xTaskCreate(
      wifiTask,
      "WiFiTask",
      8192,
      NULL,
      1,
      &wifiTaskHandle
    );
  } else {
    Serial.println("WiFi not connected, skipping servers");
    digitalWrite(pinWifiStatus, LOW);
  }

  Serial.println("Setup complete\n");
}

// ------------------- Loop -------------------
void loop() {
  unsigned long current = micros();

  if (current - previous >= params.cycleTime) {
    previous = current;

    readSensors();

    linePosition = getLinePosition(sensors);
    linePosition = constrain(linePosition, -33.0f, 33.0f);
    if (checkAllBlack(mappedSensors)){
      linePosition = 0.0f;
    }
    error = linePosition;

    forwardSpeed = running ? params.defaultSpeed : 0;

    if (params.diff == 0.0f) {
      computePID();
      if(checkAllWhite(mappedSensors)) {
        running = false;
        iTerm = 0.0f;
        correction = 0.0f;
      } else {
        lastSeenLineTime = millis();
      }
    } else {
      if (!checkAllWhite(mappedSensors)) {
        computePID();
        lastSeenLineTime = millis();
      } else if (running) {
        correction = correction > 0 ? 510.0f : -510.0f;
      }

      if (millis() - lastSeenLineTime > params.maxTimeWithoutLine) {
        running = false;
        iTerm = 0.0f;
        correction = 0.0f;
      }
    }

    float absCorrection = fabs(correction);
    scale = (forwardSpeed + absCorrection > 255.0f) ? 255.0f / (forwardSpeed + absCorrection) : 1.0f;

    if (correction > 0.0f) {
      m1 = (forwardSpeed + correction) * scale;
      m2 = m1 - correction;
    } else if (correction < 0.0f) {
      m2 = (forwardSpeed - correction) * scale;
      m1 = m2 + correction;
    } else {
      m1 = forwardSpeed;
      m2 = forwardSpeed;
    }

    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);

    updateMotors();

    calculationTime = micros() - current;
  }
}
