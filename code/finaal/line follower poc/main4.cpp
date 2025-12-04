#include <Arduino.h>
#include "EEPROMAnything.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>

// control loop timing
unsigned long previous = 0;
unsigned long calculationTime = 0;
unsigned long calculationTimeSum = 0;
int cycleCount = 0;
// data streaming
int cyclesSinceLastSend = 0;
int cyclesPerSend = 4; // send data every n cycles

unsigned long lastSeenLineTime = 1000; // time when line was last seen

bool running = false;
int forwardSpeed = 0;
// PID
float error = 0, linePosition = 0, correction = 0, scale = 1.0f;

// motors
double m1_pwm_fwd = 0, m1_pwm_bwd = 0, m2_pwm_fwd = 0, m2_pwm_bwd = 0;
double m1 = 0, m2 = 0, m1_raw = 0, m2_raw = 0; // motor PWM values and speed commands
bool motorsEnabled = false;

bool ledWifiStatus = false;
int pinWifiStatus = 3;

// sensors
float mappedSensors[8] = {}; // mapped sensor readings 0..1000 (0 white - 1000 black)
int sensors[8];               // raw readings 0..4095
int sensorPins[8] = {0, 33, 32, 35, 34, 39, 36, 0}; // 0 means not present

struct param_t {
  unsigned long cycleTime = 2000; // default microseconds between cycles (keep consistent with usage)
  char ssid[32] = "";
  char password[64] = "";
  int defaultSpeed = 300; // default motor speed
  int maxSpeed = 800;     // maximum allowed motor speed
  int blackValues[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  int whiteValues[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
  double kp = 1.5;
  double ki = 0.0;
  double kd = 0.0;
  int maxTimeWithoutLine = 2000; // ms
  int maxTimeDriveFullPower = 2000; // ms
} params;

/// WiFi and servers
WebServer server(80);
DNSServer dnsServer;
WiFiServer serverTCP(1234);
WiFiClient client;

// PWM channels for ESP32 ledc (one per motor direction)
const int PWM_CHANNEL_M1_FWD = 0;
const int PWM_CHANNEL_M1_BWD = 1;
const int PWM_CHANNEL_M2_FWD = 2;
const int PWM_CHANNEL_M2_BWD = 3;
const int PWM_FREQ = 20000; // 20 kHz
const int PWM_RES = 8;      // 8-bit resolution (0..255)

bool tryConnectWithParams(unsigned long timeoutMs = 15000) { // try connecting with saved info
  if (strlen(params.ssid) == 0) return false;

  WiFi.mode(WIFI_STA);
  Serial.print("Connecting to "); Serial.println(params.ssid);
  Serial.print("With password: "); Serial.println(params.password);
  WiFi.begin(params.ssid, params.password);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print("Connected to ");
      Serial.print(params.ssid);
      Serial.print(" with IP: ");
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(200);
  }
  return false;
}

void startConfigPortal(unsigned long portalTimeoutMs = 5 * 60 * 1000) { // start configuration portal to set wifi credentials
  const char *apName = "lineFollower_Wifi_Config";
  const char *apPass = "configureme";

  // Configure AP IP and start AP
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(apName, apPass);

  const byte DNS_PORT = 53;
  dnsServer.start(DNS_PORT, "*", apIP);

  server.on("/", [&]() {
    String page = "<html><body>"
                  "<h2>LineFollower WiFi Setup</h2>"
                  "<form action=\"/save\" method=\"POST\">"
                  "SSID:<br><input type=\"text\" name=\"ssid\" value=\"" + String(params.ssid) + "\"><br>"
                                                                                               "Password:<br><input type=\"password\" name=\"pwd\"><br><br>"
                                                                                               "<input type=\"submit\" value=\"Save\">"
                                                                                               "</form></body></html>";
    server.send(200, "text/html", page);
  });

  server.on("/save", HTTP_POST, [&]() {
    if (server.hasArg("ssid") && server.hasArg("pwd")) {
      String ss = server.arg("ssid");
      String pw = server.arg("pwd");
      ss.toCharArray(params.ssid, sizeof(params.ssid));
      pw.toCharArray(params.password, sizeof(params.password));
      Serial.println("Saving new credentials:");
      Serial.print("SSID: ");
      Serial.println(params.ssid);
      Serial.print("Password: ");
      Serial.println(params.password);
      EEPROM_writeAnything(0, params);
      server.send(200, "text/html", "Saved credentials. Device will restart and attempt to connect.");
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, "text/plain", "Missing fields");
    }
  });

  server.onNotFound([&]() {
    server.send(404, "text/plain", "Not found");
  });

  server.begin();

  unsigned long started = millis();
  while (millis() - started < portalTimeoutMs) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(50);
    digitalWrite(pinWifiStatus, !digitalRead(pinWifiStatus)); // Blink WiFi status LED
  }

  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
}

void startWifi() {
  if (tryConnectWithParams()) {
    return;
  }
  Serial.println("Connection failed, starting config portal...");
  startConfigPortal();
}

void dataStreaming() {
  if (!client || !client.connected()) {
    client = serverTCP.available(); // Accept new client
  }

  cyclesSinceLastSend++;
  if (client && client.connected() && cyclesSinceLastSend >= cyclesPerSend) {
    cyclesSinceLastSend = 0;

    // Build compact JSON manually (ok) -- can be replaced with StaticJsonDocument for safety
    String output = "{";
    output += "\"timeStamp\":" + String(millis()) + ",";
    output += "\"ssid\":\"" + String(params.ssid) + "\",";
    output += "\"cycleTime\":" + String(params.cycleTime) + ",";
    output += "\"calculationTime\":" + String(calculationTime) + ",";
    output += "\"running\":" + String(running ? 1 : 0) + ",";
    output += "\"motorsEnabled\":" + String(motorsEnabled ? 1 : 0) + ",";
    output += "\"kp\":" + String(params.kp) + ",";
    output += "\"ki\":" + String(params.ki) + ",";
    output += "\"kd\":" + String(params.kd) + ",";
    output += "\"linePosition\":" + String(linePosition) + ",";
    output += "\"error\":" + String(error) + ",";
    output += "\"correction\":" + String(correction) + ",";
    output += "\"motor1\":" + String(m1) + ",";
    output += "\"motor2\":" + String(m2) + ",";
    output += "\"defaultSpeed\":" + String(params.defaultSpeed) + ",";
    output += "\"maxSpeed\":" + String(params.maxSpeed) + ",";
    output += "\"maxTimeWithoutLine\":" + String(params.maxTimeWithoutLine) + ",";
    output += "\"maxTimeDriveFullPower\":" + String(params.maxTimeDriveFullPower) + ",";

    output += "\"rawSensors\":[";
    for (int i = 0; i < 8; i++) {
      output += String(sensors[i]);
      if (i < 7) output += ",";
    }
    output += "],";

    output += "\"sensors\":[";
    for (int i = 0; i < 8; i++) {
      output += String(mappedSensors[i], 2);
      if (i < 7) output += ",";
    }
    output += "],";

    output += "\"blackValues\":[";
    for (int i = 0; i < 8; i++) {
      output += String(params.blackValues[i]);
      if (i < 7) output += ",";
    }
    output += "],";

    output += "\"whiteValues\":[";
    for (int i = 0; i < 8; i++) {
      output += String(params.whiteValues[i]);
      if (i < 7) output += ",";
    }
    output += "]";

    output += "}";

    client.println(output);
  }
}

void startRunTimeServer() {
  // start/stop
  server.on("/start", HTTP_ANY, []() {
    running = true;
    Serial.println("Control loop started via /start");
    server.send(200, "text/plain", "Started");
  });

  server.on("/stop", HTTP_ANY, []() {
    running = false;
    Serial.println("Control loop stopped via /stop");
    server.send(200, "text/plain", "Stopped");
  });

  server.on("/enable", HTTP_ANY, []() {
    motorsEnabled = true;
    Serial.println("motors enabled");
    server.send(200, "text/plain", "motors enabled");
  });

  server.on("/disable", HTTP_ANY, []() {
    motorsEnabled = false;
    Serial.println("motors disabled");
    server.send(200, "text/plain", "motors disabled");
  });

  // set parameters
  server.on("/set", HTTP_ANY, []() {
    bool changed = false;
    if (server.hasArg("cycleTime")) {
      params.cycleTime = (unsigned long)atol(server.arg("cycleTime").c_str());
      Serial.print("Set cycleTime to ");
      Serial.println(params.cycleTime);
      changed = true;
    }
    if (server.hasArg("ssid")) {
      server.arg("ssid").toCharArray(params.ssid, sizeof(params.ssid));
      Serial.print("Set SSID to ");
      Serial.println(params.ssid);
      changed = true;
    }
    if (server.hasArg("password")) {
      server.arg("password").toCharArray(params.password, sizeof(params.password));
      Serial.print("Set password to ");
      Serial.println(params.password);
      changed = true;
    }
    if (server.hasArg("defaultSpeed")) {
      params.defaultSpeed = atoi(server.arg("defaultSpeed").c_str());
      Serial.print("Set defaultSpeed to ");
      Serial.println(params.defaultSpeed);
      changed = true;
    }
    if (server.hasArg("maxSpeed")) {
      params.maxSpeed = atoi(server.arg("maxSpeed").c_str());
      Serial.print("Set maxSpeed to ");
      Serial.println(params.maxSpeed);
      changed = true;
    }
    if (server.hasArg("kp")) {
      params.kp = atof(server.arg("kp").c_str());
      Serial.print("Set kp to ");
      Serial.println(params.kp);
      changed = true;
    }
    if (server.hasArg("ki")) {
      params.ki = atof(server.arg("ki").c_str());
      Serial.print("Set ki to ");
      Serial.println(params.ki);
      changed = true;
    }
    if (server.hasArg("kd")) {
      params.kd = atof(server.arg("kd").c_str());
      Serial.print("Set kd to ");
      Serial.println(params.kd);
      changed = true;
    }
    if (server.hasArg("maxTimeDriveFullPower")) {
      params.maxTimeDriveFullPower = atoi(server.arg("maxTimeDriveFullPower").c_str());
      Serial.print("maxTimeDriveFullPower set to ");
      Serial.println(params.maxTimeDriveFullPower);
      changed = true;
    }
    if (server.hasArg("maxTimeWithoutLine")) {
      params.maxTimeWithoutLine = atoi(server.arg("maxTimeWithoutLine").c_str());
      Serial.print("maxTimeWithoutLine set to ");
      Serial.println(params.maxTimeWithoutLine);
      changed = true;
    }

    if (changed) {
      EEPROM_writeAnything(0, params);
      server.send(200, "text/plain", "Parameters updated and saved");
    } else {
      server.send(400, "text/plain", "No valid parameters provided");
    }
  });

  // calibration
  server.on("/calibrate/white", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) params.whiteValues[i] = analogRead(sensorPins[i]);
      else params.whiteValues[i] = 0;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: saved WHITE values");
    server.send(200, "text/plain", "White calibration saved");
  });

  server.on("/calibrate/black", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) params.blackValues[i] = analogRead(sensorPins[i]);
      else params.blackValues[i] = 4095;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: saved BLACK values");
    server.send(200, "text/plain", "Black calibration saved");
  });

  server.on("/calibrate/reset", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      params.whiteValues[i] = 0;
      params.blackValues[i] = 4095;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: reset to defaults");
    server.send(200, "text/plain", "Calibration reset to defaults");
  });

  server.begin();
  Serial.println("Runtime HTTP server started on port 80");
}

// Return true if ALL sensors indicate black (i.e., mapped reading >= threshold)
bool checkAllBlack(const float readings[8], float threshold = 800.0f) {
  for (int i = 1; i < 7; ++i) {
    // *** SAFETY FIX: guard against NaN/Inf
    if (!isfinite(readings[i]) || readings[i] < 0.0f) return false;
    if (readings[i] < threshold) { // if any reading below threshold -> not all black
      return false;
    }
  }
  return true;
}

// Return true if ALL sensors indicate white (i.e., mapped reading <= threshold)
bool checkAllWhite(const float readings[8], float threshold = 200.0f) {
  for (int i = 0; i < 8; ++i) {
    // *** SAFETY FIX: guard against NaN/Inf
    if (!isfinite(readings[i]) || readings[i] < 0.0f) return false;
    if (readings[i] > threshold) { // if any reading above threshold -> not all white
      return false;
    }
  }
  return true;
}

// Determine line position from raw sensor readings (as in original)
// returns position in mm relative to center (approx)
float getLinePosition(int readings[8]) {
  // *** SAFETY FIX: validate input range and clamp
  for (int i = 0; i < 8; ++i) {
    if (readings[i] < 0) readings[i] = 0;
    if (readings[i] > 10000) readings[i] = 10000; // very large spike protection
  }

  int index = 1;
  for (int i = 2; i <= 6; i++) {
    if (readings[i] > readings[index]) index = i;
  }
  if (index == 1) index = 2;
  else if (index == 6) index = 5;

  long sZero = readings[index];
  long sMinusOne = readings[index - 1];
  long sPlusOne = readings[index + 1];

  float b = ((float)(sPlusOne - sMinusOne)) / 2.0f;
  float a = sPlusOne - b - sZero;

  // handle degenerate a
  float position = 0.0f;
  if (fabs(a) > 1e-6f) {
    position = -b / (2.0f * a);
  } else {
    position = 0.0f;
  }

  // *** SAFETY FIX: check NaN/Inf and clamp
  if (!isfinite(position)) position = 0.0f;

  position += index - 3.5f;
  position *= 8.0f; // sensor distance in mm

  // *** SAFETY FIX: hard clamp
  position = constrain(position, -33.0f, 33.0f);

  return position;
}

// Improved PID compute with safe first-run handling
float pidCompute(float error, double kp, double ki, double kd) {
  static double integral = 0.0;
  static float lastError = 0.0f;
  static unsigned long lastTime = 0;
  static bool firstRun = true;

  unsigned long now = millis();
  if (firstRun) {
    lastTime = now;
    lastError = error;
    firstRun = false;
    return 0.0f; // no correction first iteration
  }

  float dt = (now - lastTime) / 1000.0f;
  if (dt <= 0.0f) dt = 1e-3f; // guard
  lastTime = now;

  // *** SAFETY FIX: guard large dt (e.g. after long blocking ops)
  if (dt > 1.0f) dt = 1.0f;

  // Handle NaN/Inf error
  if (!isfinite(error)) error = 0.0f;

  integral += error * dt;

  // *** SAFETY FIX: anti-windup (smaller clamp to be conservative)
  const double INTEGRAL_MAX = 5000.0;
  if (integral > INTEGRAL_MAX) integral = INTEGRAL_MAX;
  if (integral < -INTEGRAL_MAX) integral = -INTEGRAL_MAX;

  float derivative = (error - lastError) / dt;
  lastError = error;

  double out = kp * error + ki * integral + kd * derivative;

  // remove nan/inf
  if (!isfinite(out)) out = 0.0;

  // *** SAFETY FIX: clamp PID output to avoid runaway values
  out = constrain(out, -2000.0, 2000.0);

  return (float)out;
}

// Setup function
void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");

  // Initialize PWM channels for ESP32 (recommended approach)
  ledcSetup(PWM_CHANNEL_M1_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_M1_BWD, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_M2_FWD, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CHANNEL_M2_BWD, PWM_FREQ, PWM_RES);

  // Attach channels to pins
  ledcAttachPin(25, PWM_CHANNEL_M1_FWD);
  ledcAttachPin(26, PWM_CHANNEL_M1_BWD);
  ledcAttachPin(27, PWM_CHANNEL_M2_FWD);
  ledcAttachPin(14, PWM_CHANNEL_M2_BWD);

  // Ensure EEPROM initialized before reading
  EEPROM.begin(512);
  EEPROM_readAnything(0, params);

  // Configure sensor pins as inputs
  for (int i = 0; i < 8; ++i) {
    if (sensorPins[i] != 0) pinMode(sensorPins[i], INPUT);
  }
  pinMode(pinWifiStatus, OUTPUT);

  startWifi();
  delay(1000);
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(pinWifiStatus, HIGH); // Turn on WiFi status LED
    serverTCP.begin();
    Serial.println("TCP server started");
    startRunTimeServer();
  } else {
    Serial.println("No WiFi - running without network");
  }
}

// Helper: map sensor with calibration, handle inverted calibration
float mapSensorWithCalibration(int raw, int whiteCal, int blackCal) {
  // *** SAFETY FIX: clamp raw ADC to expected range
  if (raw < 0) raw = 0;
  if (raw > 5000) raw = 5000;

  // raw range 0..4095; we want mapped 0 (white) .. 1000 (black)
  if (whiteCal == blackCal) return 0.0f; // avoid div by zero
  // If calibration is inverted (white > black) swap them
  int minC = min(whiteCal, blackCal);
  int maxC = max(whiteCal, blackCal);

  // protect calibration values
  if (minC == maxC) return 0.0f;

  // Constrain raw to calibration range then map
  int r = constrain(raw, minC, maxC);
  float mapped;
  if (whiteCal < blackCal) {
    // normal: whiteCal -> 0, blackCal -> 1000
    mapped = (float)(r - whiteCal) * 1000.0f / (float)(blackCal - whiteCal);
  } else {
    // inverted: whiteCal > blackCal
    mapped = (float)(whiteCal - r) * 1000.0f / (float)(whiteCal - blackCal);
  }
  // *** SAFETY FIX: guard NaN/Inf and clamp
  if (!isfinite(mapped)) mapped = 0.0f;
  mapped = constrain(mapped, 0.0f, 1000.0f);
  return mapped;
}

void loop() {
  unsigned long current = micros();
  // Ensure params.cycleTime has a sane minimum to avoid hogging CPU
  if (params.cycleTime < 50) params.cycleTime = 50; // 50 microseconds minimum

  if (current - previous >= params.cycleTime) {
    previous = current;

    // handle wifi clients
    dataStreaming();
    server.handleClient();
    yield(); // allow background tasks

    // Read sensors
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) {
        // *** SAFETY FIX: protect analogRead spikes
        int raw = analogRead(sensorPins[i]);
        if (raw < 0) raw = 0;
        if (raw > 5000) raw = 5000;
        sensors[i] = raw;

        mappedSensors[i] = mapSensorWithCalibration(sensors[i], params.whiteValues[i], params.blackValues[i]);

        // *** SAFETY FIX: ensure mappedSensors valid
        if (!isfinite(mappedSensors[i])) mappedSensors[i] = 0.0f;
        mappedSensors[i] = constrain(mappedSensors[i], 0.0f, 1000.0f);
      } else {
        sensors[i] = 0;
        mappedSensors[i] = 0.0f;
      }
    }

    // calculate error
    linePosition = getLinePosition(sensors); // in mm

    // *** SAFETY FIX: catch invalid outputs
    if (!isfinite(linePosition)) linePosition = 0.0f;

    linePosition = constrain(linePosition, -33.0f, 33.0f); // limit to sensor array width
    error = linePosition;
    if (checkAllBlack(mappedSensors)){
        error = 0.0f;
        correction = 0.0f;
        Serial.println("All black detected");
    }



    // *** SAFETY FIX: guard error
    if (!isfinite(error)) error = 0.0f;
    error = constrain(error, -33.0f, 33.0f);

    // motor control PID
    if (running) forwardSpeed = params.defaultSpeed;
    else forwardSpeed = 0;

    // compute correction only when there's line (not all white)
    if (!checkAllWhite(mappedSensors)) {
      correction = pidCompute(error, params.kp, params.ki, params.kd);
      lastSeenLineTime = millis();
    } 
    else {
      // if we lost the line and running, apply a last known strategy:
      if (running) {
        if(checkAllWhite(mappedSensors)==true && correction > 0.0f) correction = 100.0f;
        if(checkAllWhite(mappedSensors)==true && correction < 0.0f) correction = -100.0f;
        // keep previous correction (or optionally set to a small search value)
      } 
      else {
        error = 0.0f;
        correction = 0.0f;
      }
    }

    // *** SAFETY FIX: guard correction
    if (!isfinite(correction)) correction = 0.0f;
    correction = constrain(correction, -1000.0f, 1000.0f);

    // if running and lost for long time, stop
    if (millis() - lastSeenLineTime > (unsigned long)params.maxTimeWithoutLine) {
      running = false;
    }

    // compute scale safely
    double denom = (double)forwardSpeed + fabs(correction);
    if (denom <= 0.0) {
      scale = 1.0;
    } else if (denom > (double)params.maxSpeed) {
      scale = (double)params.maxSpeed / denom;
    } else {
      scale = 1.0;
    }

    m1_raw = forwardSpeed + correction;
    m2_raw = forwardSpeed - correction;

    m1 = m1_raw * scale;
    m2 = m2_raw * scale;

    m1 = constrain(m1, -100.0, 100.0);
    m2 = constrain(m2, -100.0, 100.0);

    m1_pwm_fwd = m1 > 0 ? map(m1, 0, 100, 0, 255) : 0;
    m1_pwm_bwd = m1 < 0 ? map(-m1, 0, 100, 0, 255) : 0;

    m2_pwm_fwd = m2 > 0 ? map(m2, 0, 100, 0, 255) : 0;
    m2_pwm_bwd = m2 < 0 ? map(-m2, 0, 100, 0, 255) : 0; 

    // *** SAFETY FIX: ensure pwm within bounds

    analogWrite(25, m1_pwm_fwd);
    analogWrite(26, m1_pwm_bwd);
    analogWrite(27, m2_pwm_fwd);
    analogWrite(14, m2_pwm_bwd);

    calculationTime = micros() - current;
  }
}
