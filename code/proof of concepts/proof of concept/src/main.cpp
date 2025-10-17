#include <Arduino.h>
#include "EEPROMAnything.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>

bool debug;
unsigned long previous, calculationTime;
bool running = false; // control loop enabled/disabled by server
int sensors[8]; // array to hold sensor readings
int sensorPins[8] = {0,32, 33, 34, 35, 36, 39,0}; // GPIO pins connected to the sensors]
int m1,m2; // motor speeds


WebServer server(80);
DNSServer dnsServer;


struct param_t
{
  unsigned long cycleTime;
  char ssid[32];
  char password[64];
} params;




//params helper
void resetParamsInEEPROM() {
  memset(&params, 0, sizeof(params));
  EEPROM_writeAnything(0, params);
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

// Starts a simple configuration portal where user can enter SSID and password.
// Saves into EEPROM and restarts when credentials are submitted.
void startConfigPortal(unsigned long portalTimeoutMs = 5 * 60 * 1000) {
  const char* apName = "lineFollower_Wifi_Config";
  const char* apPass = "configureme";

  // Configure AP IP and start AP
  IPAddress apIP(192, 168, 4, 1);
  IPAddress apGateway(192, 168, 4, 1);
  IPAddress apSubnet(255, 255, 255, 0);

  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(apIP, apGateway, apSubnet);
  WiFi.softAP(apName, apPass);

  // Start DNS server to capture all hostnames and redirect to AP IP (captive portal)
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

  if (tryConnectWithParams()) {
    // connected
    return;
  }
  // start portal to collect new credentials
  Serial.println("Connection failed, starting config portal...");
  startConfigPortal();

}

// Start a runtime HTTP server with endpoints to set and view parameters
void startRuntimeServer() {

  ////////// /set - set parameters via GET or POST args
  server.on("/set", HTTP_ANY, []() {
    bool changed = false;
    if (server.hasArg("cycleTime")) {
      String v = server.arg("cycleTime");
      unsigned long ct = (unsigned long)atol(v.c_str());
      params.cycleTime = ct;
      Serial.print("Set cycleTime to "); Serial.println(ct);
      changed = true;
    }

    if (server.hasArg("m1")) {
    m1 = server.arg("m1").toInt();  
    Serial.print("Set m1 to "); Serial.println(m1);
    changed = true;
    }

     if (server.hasArg("m2")) {
    m2 = server.arg("m2").toInt();  
    Serial.print("Set m2 to "); Serial.println(m2);
    changed = true;
    }
   


    if (changed) {
      EEPROM_writeAnything(0, params);
      server.send(200, "text/plain", "Parameters saved\n");
    } else {
      server.send(400, "text/plain", "No parameters provided\n");
    }
  });


  /////////// /start - enable the control loop

  server.on("/start", HTTP_ANY, []() {
    running = true;
    Serial.println("Control loop started via /start");
    server.send(200, "text/plain", "Started\n");
  });

  /////////// /stop - disable the control loop and stop motors
  server.on("/stop", HTTP_ANY, []() {
    running = false;
    // immediately stop motors
    Serial.println("Control loop stopped via /stop");
    server.send(200, "text/plain", "Stopped\n");
  });

  /////////// /debug - return current parameters as JSON
  server.on("/debug", HTTP_GET, []() {
    String json = "{";
    // timestamp first (milliseconds since boot)
    json += "\"timestamp\":" + String(millis()) + ",";
    json += "\"cycleTime\":" + String(params.cycleTime) + ",";
    json += "\"calculationTime\":" + String(calculationTime);
    json += "\"ssid\":\"" + String(params.ssid) + "\",";
    json += "\"ip\":\"" + WiFi.localIP().toString() + "\",";
    json += ",\"running\":" + String(running ? "true" : "false");
    json += ",\"m1\":" + String(m1);
    json += ",\"m2\":" + String(m2);

    // append sensor array
    json += ",\"sensors\": [";
    for (int i = 0; i < 8; ++i) {
      json += String(sensors[i]);
      if (i < 7) json += ",";
    }
    json += "]";
    json += "}";
    server.send(200, "application/json", json);
    Serial.println("Served /debug");
  });

 

 
  //////// begin server
  server.begin();
  Serial.println("Runtime HTTP server started on port 80");

  /////////// Start mDNS responder so device is reachable as http://linefollower.local
  if (MDNS.begin("linefollower")) {
    MDNS.addService("http", "tcp", 80);
    Serial.println("mDNS responder started: http://linefollower.local");
  } else {
    Serial.println("Error setting up mDNS responder");
  }
}


void IRAM_ATTR handleInterrupt() {
  running = !running;  // Toggle state
}



void setup() {
  // initialize Serial first so prints appear
  Serial.begin(9600);
  delay(100);
  Serial.println("Starting...");

  // Ensure EEPROM is initialized before reading
  EEPROM.begin(512);
  EEPROM_readAnything(0, params);
  // Configure sensor pins as inputs
  for (int i = 0; i < 8; ++i) {
    if (sensorPins[i] != 0) pinMode(sensorPins[i], INPUT);
  }

  //start/stop interupt
  pinMode(23, INPUT_PULLUP);
  attachInterrupt(23, handleInterrupt, FALLING);

  connectOrStartPortal();
  // If connected start runtime HTTP server
  if (WiFi.status() == WL_CONNECTED) {
    startRuntimeServer();
  }
}

void loop() 
{
  // Let the web server handle incoming clients
  server.handleClient();

  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  {
    previous = current;


    if (running) {
      if(m1<0){
        analogWrite(25, 0);
        analogWrite(19, m1*-1);
      }
      else{
        analogWrite(25, m1);
        analogWrite(19, 0);
      }
      if(m2<0){
        analogWrite(26, 0);
        analogWrite(21, m2*-1);
      }
      else{
        analogWrite(26, m2);
        analogWrite(21, 0);
      }

    }
    else{
        analogWrite(25, 0);
        analogWrite(26, 0);
        analogWrite(19, 0);
        analogWrite(21, 0);
    }

    // Read 8 sensors into the sensors[] array
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) {
        sensors[i] = analogRead(sensorPins[i]);
      }
    }

  }  
  unsigned long difference = micros() - current;
  if (difference > calculationTime) calculationTime = difference;

}

