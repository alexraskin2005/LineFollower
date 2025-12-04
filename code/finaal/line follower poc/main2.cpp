#include <Arduino.h>
#include "EEPROMAnything.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <WebSocketsServer.h> // <--- add this library (arduinoWebSockets)

bool debug;
unsigned long previous, calculationTime;
bool running = false; // control loop enabled/disabled by server
int sensors[8]; // array to hold sensor readings
int sensorPins[8] = {0,32, 33, 34, 35, 36, 39,0}; // GPIO pins connected to the sensors]

// PID constants — tune these!
double Kp = 1.5;
double Ki = 0.0;
double Kd = 0.8;

// PID state
double lastError = 0;
double integral = 0;

// Calibration arrays for each sensor: black=dark, white=light
u_int8_t mappedSensors[8] = {0};
double m1_pwm = 0, m2_pwm = 0, m1 = 0, m2 = 0, m1_raw, m2_raw; // motor PWM values and speed commands
int forwardSpeed = 0; // forward speed command

double error, linePosition, correction;
int tresholdBlack = 80;

WebServer server(80);
DNSServer dnsServer;

// WebSocket server on port 81
WebSocketsServer webSocket = WebSocketsServer(81);
bool wsClientConnected = false;
unsigned long lastBroadcast = 0;
unsigned long minBroadcastInterval = 50; // ms - rate limit broadcasts (tune if needed)

struct param_t
{
  unsigned long cycleTime;
  char ssid[32];
  char password[64];
  int defaultSpeed; // default motor speed
  int maxSpeed;     // maximum allowed motor speed
  int blackValues[8] = {0,0,0,0,0,0,0,0};
  int whiteValues[8] = {4095,4095,4095,4095,4095,4095,4095,4095};
  double ki;
  double kp;
  double kd;

} params;

// params helper
void resetParamsInEEPROM() {
  memset(&params, 0, sizeof(params));
  EEPROM_writeAnything(0, params);
}

// WebSocket event handler
void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("WebSocket[%u] Disconnected\n", num);
      // check if any clients still connected
      wsClientConnected = (webSocket.connectedClients() > 0);
      break;
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("WebSocket[%u] Connected from %s\n", num, ip.toString().c_str());
      wsClientConnected = true;
      // optionally send an initial message
      String initMsg = "{\"type\":\"welcome\",\"ip\":\"" + ip.toString() + "\"}";
      webSocket.sendTXT(num, initMsg);
      break;
    }
    case WStype_TEXT:
      // if you want to accept commands from client via websocket, parse payload here.
      Serial.printf("WebSocket[%u] RX: %s\n", num, payload);
      break;
    default:
      break;
  }
}

// Build the same JSON used by /debug
String buildDebugJSON() {
  String json = "{";
  json += "\"timestamp\":" + String(millis()) + ",";
  json += "\"cycleTime\":" + String(params.cycleTime) + ",";
  json += "\"ssid\":\"" + String(params.ssid) + "\",";
  json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
  json += "\"calculationTime\":" + String(calculationTime);
  json += ",\"defaultSpeed\":" + String(params.defaultSpeed);
  json += ",\"maxSpeed\":" + String(params.maxSpeed);
  json += ",\"running\":" + String(running ? "true" : "false");
  json += ",\"kp\":" + String(params.kp);
  json += ",\"ki\":" + String(params.ki);
  json += ",\"kd\":" + String(params.kd);
  json += ",\"error\":" + String(error);
  json += ",\"correction\":" + String(correction);
  json += ",\"m1\":" + String(m1);
  json += ",\"m2\":" + String(m2);
  json += ",\"sensors\": [";
  for (int i = 0; i < 8; ++i) {
    json += String(sensors[i]);
    if (i < 7) json += ",";
  }
  json += "]";
  json += ",\"mappedSensors\": [";
  for (int i = 0; i < 8; ++i) {
    json += String(mappedSensors[i]);
    if (i < 7) json += ",";
  }
  json += "]";
  json += ",\"whiteValues\": [";
  for (int i = 0; i < 8; ++i) {
    json += String(params.whiteValues[i]);
    if (i < 7) json += ",";
  }
  json += "]";
  json += ",\"blackValues\": [";
  for (int i = 0; i < 8; ++i) {
    json += String(params.blackValues[i]);
    if (i < 7) json += ",";
  }
  json += "]";
  json += "}";
  return json;
}

// Try to connect using credentials in params. Returns true on success.
bool tryConnectWithParams(unsigned long timeoutMs = 15000) {
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

// Starts a simple configuration portal... (unchanged)
void startConfigPortal(unsigned long portalTimeoutMs = 5 * 60 * 1000) {
  const char* apName = "lineFollower_Wifi_Config";
  const char* apPass = "configureme";

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
      Serial.print("SSID: "); Serial.println(params.ssid);
      Serial.print("Password: "); Serial.println(params.password);
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
    delay(10);
  }

  server.stop();
  dnsServer.stop();
  WiFi.softAPdisconnect(true);
}

// Try connecting using saved params, otherwise start portal to collect them.
void connectOrStartPortal() {
  if (tryConnectWithParams()) return;
  Serial.println("Connection failed, starting config portal...");
  startConfigPortal();
}

// Start a runtime HTTP server with endpoints to set and view parameters
void startRuntimeServer() {
  // /set (unchanged) ...
  server.on("/set", HTTP_ANY, []() {
    bool changed = false;
    if (server.hasArg("cycleTime")) {
      String v = server.arg("cycleTime");
      unsigned long ct = (unsigned long)atol(v.c_str());
      params.cycleTime = ct;
      Serial.print("Set cycleTime to "); Serial.println(ct);
      changed = true;
    }
    if (server.hasArg("ssid")) {
      String s = server.arg("ssid");
      s.toCharArray(params.ssid, sizeof(params.ssid));
      Serial.print("Set SSID to "); Serial.println(params.ssid);
      changed = true;
    }
    if (server.hasArg("password")) {
      String p = server.arg("password");
      p.toCharArray(params.password, sizeof(params.password));
      Serial.print("Set password to "); Serial.println(params.password);
      changed = true;
    }
    if (server.hasArg("defaultSpeed")) {
      String ds = server.arg("defaultSpeed");
      params.defaultSpeed = atoi(ds.c_str());
      Serial.print("Set defaultSpeed to "); Serial.println(params.defaultSpeed);
      changed = true;
    }
    if (server.hasArg("maxSpeed")) {
      String ms = server.arg("maxSpeed");
      params.maxSpeed = atoi(ms.c_str());
      Serial.print("Set maxSpeed to "); Serial.println(params.maxSpeed);
      changed = true;
    }
    if (server.hasArg("kp")) {
      String s = server.arg("kp");
      params.kp = atof(s.c_str());
      Serial.print("Set kp to "); Serial.println(params.kp);
      changed = true;
    }
    if (server.hasArg("ki")) {
      String s = server.arg("ki");
      params.ki = atof(s.c_str());
      Serial.print("Set ki to "); Serial.println(params.ki);
      changed = true;
    }
    if (server.hasArg("kd")) {
      String s = server.arg("kd");
      params.kd = atof(s.c_str());
      Serial.print("Set kd to "); Serial.println(params.kd);
      changed = true;
    }

    if (changed) {
      EEPROM_writeAnything(0, params);
      server.send(200, "text/plain", "Parameters saved\n");
    } else {
      server.send(400, "text/plain", "No parameters provided\n");
    }
  });

  // /start and /stop (unchanged)
  server.on("/start", HTTP_ANY, []() {
    running = true;
    Serial.println("Control loop started via /start");
    server.send(200, "text/plain", "Started\n");
  });

  server.on("/stop", HTTP_ANY, []() {
    running = false;
    Serial.println("Control loop stopped via /stop");
    server.send(200, "text/plain", "Stopped\n");
  });

  // /debug still available (uses buildDebugJSON())
  server.on("/debug", HTTP_GET, []() {
    String json = buildDebugJSON();
    server.send(200, "application/json", json);
    Serial.println("Served /debug");
  });

  // Calibration endpoints (unchanged)...
  server.on("/calibrate/white", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) params.whiteValues[i] = analogRead(sensorPins[i]);
      else params.whiteValues[i] = 0;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: saved WHITE values (path)");
    server.send(200, "text/plain", "Saved white calibration values\n");
  });

  server.on("/calibrate/black", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) params.blackValues[i] = analogRead(sensorPins[i]);
      else params.blackValues[i] = 0;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: saved BLACK values (path)");
    server.send(200, "text/plain", "Saved black calibration values\n");
  });

  server.on("/calibrate/reset", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      params.whiteValues[i] = 4095;
      params.blackValues[i] = 0;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: reset to defaults (path)");
    server.send(200, "text/plain", "Reset calibration to defaults\n");
  });

  server.on("/", HTTP_GET, []() {
    String page = "<html><head><meta charset=\"utf-8\"><title>Line Follower - Device</title></head><body>";
    page += "<h2>Line Follower — Device Control</h2>";
    page += "<p>Use the links and examples below to inspect and configure the device.</p>";
    page += "<ul>";
    page += "<li><a href=\"/debug\">/debug</a> — JSON with current params, sensors and calibration values.</li>";
    page += "<li><a href=\"/calibrate/white\">/set/calibrate/white</a> — sample current sensors and save as <code>whiteValues</code>.</li>";
    page += "<li><a href=\"/calibrate/black\">/set/calibrate/black</a> — sample current sensors and save as <code>blackValues</code>.</li>";
    page += "<li><a href=\"/calibrate/reset\">/set/calibrate/reset</a> — reset calibration to defaults (white=4095, black=0).</li>";
    page += "<li><a href=\"/start\">/start</a> — start the control loop.</li>";
    page += "<li><a href=\"/stop\">/stop</a> — stop the control loop and motors.</li>";
    page += "</ul>";
    page += "<h3>WebSocket</h3>";
    page += "<p>WebSocket is available at <code>ws://{device_ip}:81</code>. Connect to receive JSON updates (same schema as /debug).</p>";
    page += "</body></html>";
    server.send(200, "text/html", page);
  });

  server.begin();
  Serial.println("Runtime HTTP server started on port 80");

  if (MDNS.begin("linefollower")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started: http://linefollower.local");
  } else {
    Serial.println("Error setting up mDNS responder");
  }

  // start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("WebSocket server started on port 81");
}

float getLinePosition(uint8_t readings[8]) {
  int maxIndex = 0;
  uint8_t maxValue = readings[0];
  for (int i = 1; i < 8; i++) {
    if (readings[i] > maxValue) {
      maxValue = readings[i];
      maxIndex = i;
    }
  }
  if (maxIndex < 2 || maxIndex > 5) {
    return -1;
  }
  float x[5] = {-2, -1, 0, 1, 2};
  float y[5];
  for (int i = 0; i < 5; i++) {
    y[i] = (float)readings[maxIndex - 2 + i];
  }
  float sumX2 = 0, sumX = 0, sumY = 0, sumXY = 0, sumX2Y = 0;
  for (int i = 0; i < 5; i++) {
    sumX2 += x[i] * x[i];
    sumX += x[i];
    sumY += y[i];
    sumXY += x[i] * y[i];
    sumX2Y += x[i] * x[i] * y[i];
  }
  float denom = 5 * sumX2 - sumX * sumX;
  if (denom == 0) return -1;
  float b = (5 * sumXY - sumX * sumY) / denom;
  float a = (5 * sumX2Y - sumX2 * sumY) / denom;
  float vertexOffset = -b / (2 * a);
  float position = maxIndex + vertexOffset;
  position = constrain(position, 0, 7);
  return position * (66.0 / 7.0);
}

bool checkAllBlack(uint8_t readings[8]){
  for (int i = 0; i < 8; ++i) {
    if (readings[i] < tresholdBlack) {
      return false;
    }
  }
  return true;
}

double computePID(double error) {
  double dt = 0.02;  // 20 ms loop time
  integral += error * dt;
  double derivative = (error - lastError) / dt;
  double output = Kp * error + Ki * integral + Kd * derivative;
  lastError = error;
  return output;
}

void setup() {
  Serial.begin(9600);
  delay(100);
  Serial.println("Starting...");
  EEPROM.begin(512);
  EEPROM_readAnything(0, params);
  for (int i = 0; i < 8; ++i) {
    if (sensorPins[i] != 0) pinMode(sensorPins[i], INPUT);
  }
  connectOrStartPortal();
  if (WiFi.status() == WL_CONNECTED) {
    startRuntimeServer();
  }
}

void loop() 
{
  server.handleClient();
  webSocket.loop(); // handle websocket traffic

  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  {
    previous = current;

    if (running) {
      forwardSpeed = params.defaultSpeed;
    } else {
      forwardSpeed = 0;
    }

    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) {
        sensors[i] = analogRead(sensorPins[i]);
        mappedSensors[i] = map(sensors[i], params.whiteValues[i], params.blackValues[i], 0, 100);
      }
    }

    if(checkAllBlack(mappedSensors)){
      error=0;
    } else {
      linePosition = getLinePosition(mappedSensors);
      error = 33.0 - linePosition;
    }
    correction = computePID(error);

    m1_raw=forwardSpeed+correction;
    m2_raw=forwardSpeed-correction;
    double scale = 1.0;
    if (m1_raw > params.maxSpeed || m2_raw > params.maxSpeed) {
      scale = params.maxSpeed / max(m1_raw, m2_raw);
    }
    if (m1_raw < 0 || m2_raw < 0) {
      scale = 2.0;
    }

    m1 = m1_raw * scale;
    m2 = m2_raw * scale;
    m1_pwm = map(m1,0,100,0,255);
    m2_pwm = map(m2,0,100,0,255); 

    analogWrite(25, m1_pwm);
    analogWrite(26, m2_pwm);
  }  

  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;

  // Broadcast debug JSON to all WS clients, but rate-limited
  if (wsClientConnected) {
    unsigned long nowMs = millis();
    if (nowMs - lastBroadcast >= minBroadcastInterval) {
      String json = buildDebugJSON();
      webSocket.broadcastTXT(json);
      lastBroadcast = nowMs;
    }
  }
}
