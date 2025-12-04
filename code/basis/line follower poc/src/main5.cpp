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
  char ssid[32] ;
  char password[64] ;
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

// String buffer for data streaming (reuse to avoid fragmentation)
String dataBuffer;

// Forward declarations
void setupMotors();
void readSensors();
void computePID();
void updateMotors();
void dataStreaming();

bool tryConnectWithParams(unsigned long timeoutMs = 15000) {
  if (strlen(params.ssid) == 0) return false;

  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false); // Don't wear out flash
  
  Serial.print(F("Connecting to ")); 
  Serial.println(params.ssid);
  
  WiFi.begin(params.ssid, params.password);

  unsigned long start = millis();
  while (millis() - start < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) {
      Serial.print(F("Connected with IP: "));
      Serial.println(WiFi.localIP());
      return true;
    }
    delay(200);
  }
  return false;
}

void startConfigPortal(unsigned long portalTimeoutMs = 300000UL) {
  const char* apName = "lineFollower_Wifi_Config";
  const char* apPass = "configureme";

  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(apName, apPass);

  const byte DNS_PORT = 53;
  dnsServer.start(DNS_PORT, F("*"), apIP);

  server.on("/", []() {
    String page = F("<html><body>"
                  "<h2>LineFollower WiFi Setup</h2>"
                  "<form action=\"/save\" method=\"POST\">"
                  "SSID:<br><input type=\"text\" name=\"ssid\" value=\"");
    page += params.ssid;
    page += F("\"><br>Password:<br><input type=\"password\" name=\"pwd\"><br><br>"
              "<input type=\"submit\" value=\"Save\">"
              "</form></body></html>");
    server.send(200, F("text/html"), page);
  });

  server.on("/save", HTTP_POST, []() {
    if (server.hasArg(F("ssid")) && server.hasArg(F("pwd"))) {
      String ss = server.arg(F("ssid"));
      String pw = server.arg(F("pwd"));
      ss.toCharArray(params.ssid, sizeof(params.ssid));
      pw.toCharArray(params.password, sizeof(params.password));
      
      EEPROM_writeAnything(0, params);
      server.send(200, F("text/html"), F("Saved. Restarting..."));
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, F("text/plain"), F("Missing fields"));
    }
  });

  server.onNotFound([]() {
    server.send(404, F("text/plain"), F("Not found"));
  });

  server.begin();

  unsigned long started = millis();
  while (millis() - started < portalTimeoutMs) {
    dnsServer.processNextRequest();
    server.handleClient();
    delay(50);
    digitalWrite(pinWifiStatus, !digitalRead(pinWifiStatus));
  }

  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
}

void startWifi() {
  if (tryConnectWithParams()) return;
  
  Serial.println(F("Starting config portal..."));
  startConfigPortal();
}

void dataStreaming() {
  if (!client || !client.connected()) {
    client = serverTCP.available();
  }

  if (++cyclesSinceLastSend < cyclesPerSend) return;
  cyclesSinceLastSend = 0;

  if (!client || !client.connected()) return;

  // Build JSON efficiently
  dataBuffer = "{\"timeStamp\":";
  dataBuffer += millis();
  dataBuffer += F(",\"ssid\":\"");
  dataBuffer += params.ssid;
  dataBuffer += F("\",\"cycleTime\":");
  dataBuffer += params.cycleTime;
  dataBuffer += F(",\"calculationTime\":");
  dataBuffer += calculationTime;
  dataBuffer += F(",\"running\":");
  dataBuffer += running;
  dataBuffer += F(",\"motorsEnabled\":");
  dataBuffer += motorsEnabled;
  dataBuffer += F(",\"kp\":");
  dataBuffer += params.kp;
  
  float cycleTimeInSec = params.cycleTime * 0.000001f;
  dataBuffer += F(",\"ki\":");
  dataBuffer += (params.ki / cycleTimeInSec);
  dataBuffer += F(",\"kd\":");
  dataBuffer += (params.kd * cycleTimeInSec);
  dataBuffer += F(",\"linePosition\":");
  dataBuffer += linePosition;
  dataBuffer += F(",\"error\":");
  dataBuffer += error;
  dataBuffer += F(",\"correction\":");
  dataBuffer += correction;
  dataBuffer += F(",\"motor1\":");
  dataBuffer += m1;
  dataBuffer += F(",\"motor2\":");
  dataBuffer += m2;
  dataBuffer += F(",\"defaultSpeed\":");
  dataBuffer += params.defaultSpeed;
  dataBuffer += F(",\"diff\":");
  dataBuffer += params.diff;
  dataBuffer += F(",\"maxTimeWithoutLine\":");
  dataBuffer += params.maxTimeWithoutLine;
  dataBuffer += F(",\"crossingsPerLap\":");
  dataBuffer += params.crossingsPerLap;
  dataBuffer += F(",\"lapTime\":");
  dataBuffer += lapTime;
  dataBuffer += F(",\"crossingCount\":");
  dataBuffer += crossingCount;
  
  // Arrays
  dataBuffer += F(",\"rawSensors\":[");
  for (uint8_t i = 0; i < 8; i++) {
    dataBuffer += sensors[i];
    if (i < 7) dataBuffer += ',';
  }
  
  dataBuffer += F("],\"sensors\":[");
  for (uint8_t i = 0; i < 8; i++) {
    dataBuffer += mappedSensors[i];
    if (i < 7) dataBuffer += ',';
  }
  
  dataBuffer += F("],\"blackValues\":[");
  for (uint8_t i = 0; i < 8; i++) {
    dataBuffer += params.blackValues[i];
    if (i < 7) dataBuffer += ',';
  }
  
  dataBuffer += F("],\"whiteValues\":[");
  for (uint8_t i = 0; i < 8; i++) {
    dataBuffer += params.whiteValues[i];
    if (i < 7) dataBuffer += ',';
  }
  
  dataBuffer += F("]}");
  
  client.println(dataBuffer);
}

void startRunTimeServer() {
  server.on("/start", HTTP_ANY, []() {
    running = true;
    server.send(200, F("text/plain"), F("Started"));
  });
  
  server.on("/stop", HTTP_ANY, []() {
    running = false;
    iTerm = 0.0f; // Reset integral term
    server.send(200, F("text/plain"), F("Stopped"));
  });

  server.on("/enable", HTTP_ANY, []() {
    motorsEnabled = true;
    server.send(200, F("text/plain"), F("Motors enabled"));
  });

  server.on("/disable", HTTP_ANY, []() {
    motorsEnabled = false;
    setupMotors(); // Stop motors
    server.send(200, F("text/plain"), F("Motors disabled"));
  });

  server.on("/set", HTTP_ANY, []() {
    bool changed = false;
    
    if (server.hasArg(F("cycleTime"))) {
      unsigned long newCycleTime = server.arg(F("cycleTime")).toInt();
      if (newCycleTime > 0) {
        float ratio = (float)newCycleTime / (float)params.cycleTime;
        params.ki *= ratio;
        params.kd /= ratio;
        params.cycleTime = newCycleTime;
        changed = true;
      }
    }
    
    if (server.hasArg(F("ssid"))) {
      server.arg(F("ssid")).toCharArray(params.ssid, sizeof(params.ssid));
      changed = true;
    }
    
    if (server.hasArg(F("password"))) {
      server.arg(F("password")).toCharArray(params.password, sizeof(params.password));
      changed = true;
    }
    
    if (server.hasArg(F("defaultSpeed"))) {
      params.defaultSpeed = constrain(server.arg(F("defaultSpeed")).toInt(), 0, 255);
      changed = true;
    }
    
    if (server.hasArg(F("diff"))) {
      params.diff = server.arg(F("diff")).toFloat();
      changed = true;
    }
    
    if (server.hasArg(F("kp"))) {
      params.kp = server.arg(F("kp")).toFloat();
      changed = true;
    }
    
    if (server.hasArg(F("ki"))) {
      float cycleTimeInSec = params.cycleTime * 0.000001f;
      params.ki = server.arg(F("ki")).toFloat() * cycleTimeInSec;
      changed = true;
    }
    
    if (server.hasArg(F("kd"))) {
      float cycleTimeInSec = params.cycleTime * 0.000001f;
      params.kd = server.arg(F("kd")).toFloat() / cycleTimeInSec;
      changed = true;
    }
    
    if (server.hasArg(F("crossingsPerLap"))) {
      params.crossingsPerLap = server.arg(F("crossingsPerLap")).toInt();
      changed = true;
    }
    
    if (server.hasArg(F("maxTimeWithoutLine"))) {
      params.maxTimeWithoutLine = server.arg(F("maxTimeWithoutLine")).toInt();
      changed = true;
    }

    if (changed) {
      EEPROM_writeAnything(0, params);
      server.send(200, F("text/plain"), F("Updated"));
    } else {
      server.send(400, F("text/plain"), F("No valid params"));
    }
  });

  server.on("/calibrate/white", HTTP_ANY, []() {
    for (uint8_t i = 0; i < 8; i++) {
      params.whiteValues[i] = sensorPins[i] ? analogRead(sensorPins[i]) : 0;
    }
    EEPROM_writeAnything(0, params);
    server.send(200, F("text/plain"), F("White calibrated"));
  });

  server.on("/calibrate/black", HTTP_ANY, []() {
    for (uint8_t i = 0; i < 8; i++) {
      params.blackValues[i] = sensorPins[i] ? analogRead(sensorPins[i]) : 0;
    }
    EEPROM_writeAnything(0, params);
    server.send(200, F("text/plain"), F("Black calibrated"));
  });
  
  server.on("/calibrate/reset", HTTP_ANY, []() {
    for (uint8_t i = 0; i < 8; i++) {
      params.whiteValues[i] = 0;
      params.blackValues[i] = 4095;
    }
    EEPROM_writeAnything(0, params);
    server.send(200, F("text/plain"), F("Reset"));
  });

  server.begin();
}

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

void setup() {
  Serial.begin(9600); // Faster baud rate
  Serial.println(F("Starting..."));
  
  setupMotors();
  
  EEPROM.begin(512);
  EEPROM_readAnything(0, params);
  
  // Configure sensor pins
  for (uint8_t i = 0; i < 8; i++) {
    if (sensorPins[i]) pinMode(sensorPins[i], INPUT);
  }
  pinMode(pinWifiStatus, OUTPUT);
  
  // Reserve string buffer
  dataBuffer.reserve(600);
  
  startWifi();
  delay(1000);
  
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(pinWifiStatus, HIGH);
    serverTCP.begin();
    serverTCP.setNoDelay(true); // Disable Nagle's algorithm
    startRunTimeServer();
  }
}

void loop() {
  unsigned long current = micros();
  
  if (current - previous >= params.cycleTime) {
    previous = current;

    // WiFi handling
    dataStreaming();
    server.handleClient();

    // Sensor reading
    readSensors();

    // Line position calculation
    linePosition = getLinePosition(sensors);
    linePosition = constrain(linePosition, -33.0f, 33.0f);
    if (checkAllBlack(mappedSensors)){
      linePosition = 0.0f;
      //crossingDetected = true;
    }
    error = linePosition;

    // Crossing detection
    /*if (crossingDetected && !checkAllBlack(mappedSensors)) { //passed crossing
      crossingDetected = false;
      if(crossingCount == 0) lapStartTime = millis();
      crossingCount++;
      if(crossingCount == params.crossingsPerLap) lapTime = millis() - lapStartTime;*/
    //}
   /* if(!running){
      crossingCount = 0;
      crossingDetected = false;
    }*/

    // Start/stop logic
    forwardSpeed = running ? params.defaultSpeed : 0;

    // PID computation
    if (params.diff == 0.0f) { // stop when line lost mode
      computePID();
      if(checkAllWhite(mappedSensors)) {
        running = false;
        iTerm = 0.0f;
        correction = 0.0f;
      } else {
        lastSeenLineTime = millis();
      }
    } else { // try finding line when line lost mode
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

    // Motor speed calculation
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