#include <Arduino.h>
#include "EEPROMAnything.h"
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoJson.h>


//control loop timing
unsigned long previous, calculationTime, calculationTimeSum;
int cycleCount = 0;
//data streaming
int cyclesSinceLastSend = 0;
int cyclesPerSend = 4; //send data every n cycles

uint lastSeenLineTime =  1000; //time when line was last seen
uint lastSeenCurveTime = 0; //time when last curve was detected


bool running;
int forwardSpeed = 0; 
// PID
float error, linePosition, correction, scale, lastError = 0.0f, iTerm;

//motors
double m1_pwm_fwd,m1_pwm_bwd, m2_pwm_fwd,m2_pwm_bwd, m1, m2,m1_raw,m2_raw; // motor PWM values and speed commands
bool motorsEnabled = false;

bool ledWifiStatus = false;
int pinWifiStatus = 3; 



float mappedSensors[8] = {};//mapped sensor readings
int sensors[8]; // array to hold sensor readings
int sensorPins[8] = {0,33, 32, 35, 34, 39, 36,0}; // GPIO pins connected to the sensors]

struct param_t
{
  unsigned long cycleTime;
  char ssid[32];
  char password[64];
  int defaultSpeed; // default motor speed
  float diff;     // maximum allowed motor speed
  int blackValues[8] = {0,0,0,0,0,0,0,0};
  int whiteValues[8] = {4095,4095,4095,4095,4095,4095,4095,4095};
  double kp = 1.5;
  double ki = 0.0;
  double kd = 0.0;
  int maxTimeWithoutLine = 2000; // ms
  int maxTimeDriveFullPower = 2000; // ms
} params;



///wifi

  WebServer server(80);
  DNSServer dnsServer;
  WiFiServer serverTCP(1234);
  WiFiClient client;

bool tryConnectWithParams(unsigned long timeoutMs = 15000) { //try connecting with saved info
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

void startConfigPortal(unsigned long portalTimeoutMs = 5 * 60 * 1000) { //start configuration portal to set wifi credentials

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
  // start portal to collect new credentials
  Serial.println("Connection failed, starting config portal...");
  startConfigPortal();
}




void dataStreaming(){

  if (!client || !client.connected()) {
    client = serverTCP.available(); // Accept new client
  }

  cyclesSinceLastSend++;
  if (client && client.connected() && cyclesSinceLastSend >= cyclesPerSend) {
    cyclesSinceLastSend = 0;
    
   /* // Check if client buffer has space (ADD THIS BLOCK)
    int availableSpace = client.availableForWrite();
    if (availableSpace < 512) { // Need at least 512 bytes for our JSON
      Serial.println("Client buffer full, skipping send");
      return; // Skip this cycle if buffer is too full
    }*/
    
    

    /*//build JSON
    StaticJsonDocument<512> doc; // adjust size for number of variables

    //add all values to JSON
    doc["timeStamp"] = millis();
    doc["ssid"] = params.ssid;
    doc["cycleTime"] = params.cycleTime;
    doc["calculationTime"] = calculationTime;
    doc["running"] = running;
    doc["kp"] = params.kp;
    doc["ki"] = params.ki;
    doc["kd"] = params.kd;
    doc["linePosition"] = linePosition;
    doc["error"] = error;
    doc["correction"] = correction;
    doc["motor1"] = m1;
    doc["motor2"] = m2;
    doc["defaultSpeed"] = params.defaultSpeed;
    doc["maxSpeed"] = params.maxSpeed;
    // Add sensor readings
    JsonArray arr = doc.createNestedArray("rawSensors");
    for (int i = 0; i < 8; i++) arr.add(sensors[i]);
    JsonArray arr0 = doc.createNestedArray("sensors");
    for (int i = 0; i < 8; i++) arr0.add(mappedSensors[i]);
    // Add calibration values
    JsonArray arr1 = doc.createNestedArray("blackValues");
    for (int i = 0; i < 8; i++) arr1.add(params.blackValues[i]);
    JsonArray arr2 = doc.createNestedArray("whiteValues");
    for (int i = 0; i < 8; i++) arr2.add(params.whiteValues[i]);

    // Serialize JSON and send
    String output;
    serializeJson(doc, output);*/

    String output = "{";
    output += "\"timeStamp\":" + String(millis()) + ",";
    output += "\"ssid\":\"" + String(params.ssid) + "\",";
    output += "\"cycleTime\":" + String(params.cycleTime) + ",";
    output += "\"calculationTime\":" + String(calculationTime) + ",";
    output += "\"running\":" + String(running) + ",";
    output += "\"motorsEnabled\":" + String(motorsEnabled) + ",";
    output += "\"kp\":" + String(params.kp) + ",";
    float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
    float ki = params.ki / cycleTimeInSec;
    output += "\"ki\":" + String(ki) + ",";
    float kd = params.kd * cycleTimeInSec;
    output += "\"kd\":" + String(kd) + ",";
    output += "\"linePosition\":" + String(linePosition) + ",";
    output += "\"error\":" + String(error) + ",";
    output += "\"correction\":" + String(correction) + ",";
    output += "\"motor1\":" + String(m1) + ",";
    output += "\"motor2\":" + String(m2) + ",";
    output += "\"defaultSpeed\":" + String(params.defaultSpeed) + ",";
    output += "\"diff\":" + String(params.diff) + ",";
    output += "\"maxTimeWithoutLine\":" + String(params.maxTimeWithoutLine) + ",";
    output += "\"maxTimeDriveFullPower\":" + String(params.maxTimeDriveFullPower) + ",";
    // Arrays
    output += "\"rawSensors\":[";
    for (int i = 0; i < 8; i++) {
      output += String(sensors[i]);
      if (i < 7) output += ",";
    }
    output += "],";

    output += "\"sensors\":[";
    for (int i = 0; i < 8; i++) {
      output += String(mappedSensors[i]);
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

void startRunTimeServer(){
  // handle incoming start/stop commands
  server.on("/start", HTTP_ANY, []() {
    running = true;
    Serial.println("Control loop started via /start");
    server.send(200, "text/plain", "Started"); // ADD THIS
  });
  
  server.on("/stop", HTTP_ANY, []() {
    running = false;
    Serial.println("Control loop stopped via /stop");
    server.send(200, "text/plain", "Stopped"); // ADD THIS
  });

  server.on("/enable", HTTP_ANY, []() {
    motorsEnabled = true;
    Serial.println("motors enabled");
    server.send(200, "text/plain", "motors enabled"); // ADD THIS
  });

  server.on("/disable", HTTP_ANY, []() {
    motorsEnabled = false;
    Serial.println("motors disabled");
    server.send(200, "text/plain", "motors disabled"); // ADD THIS
  });

  // handle set commands
  server.on("/set", HTTP_ANY, []() {
    bool changed = false;
    if (server.hasArg("cycleTime")) {
      String v = server.arg("cycleTime");
      unsigned long ct = (unsigned long)atol(v.c_str());
      long newCycleTime = ct;
      float ratio = ((float) newCycleTime) / ((float) params.cycleTime);

      params.ki *= ratio;
      params.kd /= ratio;

      params.cycleTime = newCycleTime;
      Serial.print("Set cycleTime to "); Serial.println(params.cycleTime);
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
    if (server.hasArg("diff")) {
      String ms = server.arg("diff");
      params.diff = atof(ms.c_str());
      Serial.print("Set diff to "); Serial.println(params.diff);
      changed = true;
    }
    // PID params
    if (server.hasArg("kp")) {
      String s = server.arg("kp");
      params.kp = atof(s.c_str());
      Serial.print("Set kp to "); Serial.println(params.kp);
      changed = true;
    }
    if (server.hasArg("ki")) {
      String s = server.arg("ki");
      float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
      params.ki = atof(s.c_str()) * cycleTimeInSec;
      Serial.print("Set ki to "); Serial.println(params.ki);
      changed = true;
    }
    if (server.hasArg("kd")) {
      String s = server.arg("kd");
      float cycleTimeInSec = ((float) params.cycleTime) / 1000000;
      params.kd = atof(s.c_str()) / cycleTimeInSec;
      Serial.print("Set kd to "); Serial.println(params.kd);
      changed = true;
    }
    if (server.hasArg("maxTimeDriveFullPower")) {
      String ms = server.arg("maxTimeDriveFullPower");
      params.maxTimeDriveFullPower = atoi(ms.c_str());
      Serial.print("maxTimeDriveFullPower set to"); Serial.println(params.maxTimeDriveFullPower);
      changed = true;
    }
    if (server.hasArg("maxTimeWithoutLine")) {
      String ms = server.arg("maxTimeWithoutLine");
      params.maxTimeWithoutLine= atoi(ms.c_str());
      Serial.print("maxTimeWithoutLine set to"); Serial.println(params.maxTimeWithoutLine);
      changed = true;
      changed = true;
    }

    if (changed) {
      EEPROM_writeAnything(0, params);
      server.send(200, "text/plain", "Parameters updated and saved"); // ADD THIS
    } else {
      server.send(400, "text/plain", "No valid parameters provided"); // ADD THIS
    }
  });

  // handle calibration commands
  server.on("/calibrate/white", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) params.whiteValues[i] = analogRead(sensorPins[i]);
      else params.whiteValues[i] = 0;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: saved WHITE values");
    server.send(200, "text/plain", "White calibration saved"); // ADD THIS
  });

  server.on("/calibrate/black", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) params.blackValues[i] = analogRead(sensorPins[i]);
      else params.blackValues[i] = 0;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: saved BLACK values");
    server.send(200, "text/plain", "Black calibration saved"); // ADD THIS
  });
  
  server.on("/calibrate/reset", HTTP_ANY, []() {
    for (int i = 0; i < 8; ++i) {
      params.whiteValues[i] = 0;
      params.blackValues[i] = 4095;
    }
    EEPROM_writeAnything(0, params);
    Serial.println("Calibration: reset to defaults");
    server.send(200, "text/plain", "Calibration reset to defaults"); // ADD THIS
  });

  server.begin();
  Serial.println("Runtime HTTP server started on port 80");
}



bool checkAllBlack(float readings[8]){
  for (int i = 0; i < 8; ++i) {
    if (readings[i] < 800) { // threshold for black
      return false; // Found a sensor that is not black
    }
  }
  return true; // All sensors are black
}

bool checkAllWhite(float readings[8]){
  for (int i = 0; i < 8; ++i) {
    if (readings[i] > 800) {
      return false; 
    }
  }
  return true; 
}

float getLinePosition(int readings[8]){
int index = 1;
for (int i = 2; i <= 6; i++) if (readings[i] > readings[index]) index = i;

if (index == 1) index = 2;
else if (index == 6) index = 5;

long sZero = readings[index];
long sMinusOne = readings[index-1];
long sPlusOne = readings[index+1];

float b = ((float) (sPlusOne - sMinusOne)) / 2.0f;
float a = sPlusOne - b - sZero;

if (a == 0.0f) a = 0.0001f; // protect against divide by zero

float position = -b / (2.0f * a);  
if (isnan(position) || isinf(position)) position = 0.0f; // sanity check
position += index - 3.5f;   
position *= 8.0f;  //sensor distance in mm
return position;

}

float pidCompute(float error, float kp, float ki, float kd) {
    /* bereken error = setpoint - positie */


  /* proportioneel regelen */
  float output = error * params.kp;

  /* integrerend regelen */
  iTerm += params.ki*error;
  iTerm = constrain(iTerm, -510, 510);
  output += iTerm;

  /* differentiërend regelen */
  output += params.kd * (error - lastError);
  lastError = error;

  /* output begrenzen tot wat fysiek mogelijk is */
  output = constrain(output, -510, 510);
  
  return output;
}




void setup(){
  Serial.begin(9600);
  Serial.println("Starting...");
        analogWrite(25, 0);
        analogWrite(26, 0);
        analogWrite(27, 0);
        analogWrite(14, 0);

  // Ensure EEPROM is initialized before reading
  EEPROM.begin(512);
  EEPROM_readAnything(0, params);
  // Configure sensor pins as inputs
  for (int i = 0; i < 8; ++i) {
    if (sensorPins[i] != 0) pinMode(sensorPins[i], 0x01); //INPUT
  }
  pinMode(pinWifiStatus, 0x03); //OUTPUT
  
  startWifi();
  delay(1000);
  if (WiFi.status() == WL_CONNECTED) {
    digitalWrite(pinWifiStatus, HIGH); // Turn on WiFi status LED
    serverTCP.begin();
    Serial.println("TCP server started");
    startRunTimeServer();

  }
}

void loop() 
{
  unsigned long current = micros();
  if (current - previous >= params.cycleTime)
  { previous = current;


    //handle wifi clients//
    dataStreaming();
    server.handleClient(); //handle incoming http commands
    yield(); //allow background tasks



    // Read 8 sensors// 
    for (int i = 0; i < 8; ++i) {
      if (sensorPins[i] != 0) {
        sensors[i] = analogRead(sensorPins[i]);
        mappedSensors[i] = map(sensors[i], params.whiteValues[i], params.blackValues[i], 0, 1000); // white->0, black->1000
      }
    }

    //calculate error//
    linePosition = getLinePosition(sensors); // in mm
    linePosition = constrain(linePosition, -33.0, 33.0); //limit to sensor array width
    if(checkAllBlack(mappedSensors)) linePosition = 0; //center if all black
    error = linePosition;
    



    //start stop robot//
    if(running)forwardSpeed = params.defaultSpeed;
    else forwardSpeed = 0;
    //slow down if on long straight//
    if(abs(correction) > 230 && running){
      lastSeenCurveTime = millis();
      forwardSpeed = params.defaultSpeed;
    }
    if(millis()-lastSeenCurveTime > params.maxTimeDriveFullPower){
      forwardSpeed = params.defaultSpeed * 0.7;
    }

    //caluclate correction values//
    if(params.diff==0){ //dont correct when line lost
      correction = pidCompute(error, params.kp, params.ki, params.kd);
    }
    else{ //correct when line lost

      if(checkAllWhite(mappedSensors)==false){ //line seen, compute pid
        correction = pidCompute(error, params.kp, params.ki, params.kd);
        lastSeenLineTime = millis();
      }
      else{ //line lost, turn max in same direction as last correction
        if(checkAllWhite(mappedSensors)==true && correction > 0 && running) correction = 510;
        if(checkAllWhite(mappedSensors)==true && correction < 0 && running) correction = -510;
      }

      if(millis() - lastSeenLineTime > params.maxTimeWithoutLine){ //if line lost for too long, stop
        running=false;
      }
    }


    /*if(running){
     if (correction >= 0)
      {
        m1 = constrain(forwardSpeed + correction, -255, 255);
        m2 = constrain(m1 - correction, -255, 255);
        m1 = m2 + correction;
      }
      else
      {
        m2 = constrain(params.defaultSpeed - params.diff * correction, -255, 255);
        m1 = constrain(m2 + correction, -255, 255);
        m2 = m1 - correction;
      }
     
    }
    else{
      m1 = 0;
      m2 = 0;
    } */

    //calculate motor speeds//
    if(forwardSpeed+abs(correction)>255) scale = 255/(forwardSpeed+abs(correction)); 
    else scale = 1;
  
    if(correction>0) {    
    m1_raw=forwardSpeed+correction;
    m1 = m1_raw * scale;
    m2 = m1 - correction;
    }

    if(correction<0){
    m2_raw=forwardSpeed-correction;
    m2 = m2_raw * scale;
    m1 = m2 + correction;
    }

    if(correction==0){
      m1 = forwardSpeed;
      m2 = forwardSpeed;
    }

    

    m1 = constrain(m1, -255, 255);
    m2 = constrain(m2, -255, 255);
    
    m1_pwm_fwd = m1 > 0 ? m1 : 0;
    m1_pwm_bwd = m1 < 0 ? -m1 : 0;

    m2_pwm_fwd = m2 > 0 ? m2 : 0;
    m2_pwm_bwd = m2 < 0 ? -m2 : 0; 

  
    analogWrite(25, m1_pwm_fwd);
    analogWrite(26, m1_pwm_bwd);
    analogWrite(27, m2_pwm_fwd);
    analogWrite(14, m2_pwm_bwd);



  
  /*cycleCount++;
  calculationTimeSum +=  micros() - current;
  if(cycleCount>=100){
    calculationTime = calculationTimeSum;/cycleCount;
    calculationTimeSum = 0;
    cycleCount = 0;
    } */
  calculationTime = micros() - current;
  }
}