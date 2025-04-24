#include <Wire.h>
#include <Deneyap_DerinlikOlcer.h>  // Deneyap TOF library
#include <WiFi.h>
#include <WebServer.h>

// ----- USER CONFIG -----
const char* ssid     = "ESP32-Bot";
const char* password = "Password1!";

// Bias to boost left motor (Motor B)
float leftMotorBias = 1.1;

// Base speed (0–255)
int motorSpeed = 200;

// ----- PIN DEFINITIONS -----
// ToF sensor I2C pins
#define SDA_PIN         21   // I2C SDA
#define SCL_PIN         22   // I2C SCL
// ToF sensor shutdown pin
#define TOF_XSHUT_PIN   13   // XSHUT wired to D13

// Motor A (Right Motor)
const int motorA_IN1 = 27;
const int motorA_IN2 = 26;
const int motorA_EN  = 14;

// Motor B (Left Motor)
const int motorB_IN1 = 25;
const int motorB_IN2 = 33;
const int motorB_EN  = 32;

// Reflectance sensor at the front (active LOW)
const int frontSensorPin = 35;

// PWM settings
const int pwmFreq       = 30000;
const int pwmResolution = 8;

// Wall-follow flag
bool wallFollowActive = false;

// Deneyap ToF sensor
TofRangeFinder ranger;
unsigned long lastSensorResetTime = 0;
const unsigned long RESET_INTERVAL = 60000;  // Reset sensor every 60 s
int lastValidDistance = 100;                 // Last valid reading in mm

// Rate limiting for sensor reads
unsigned long lastSensorRead     = 0;
const unsigned long MIN_READ_INTERVAL = 50; // ms

// WiFi RSSI caching
int cachedRssi = -70;  // Best RSSI from last scan

// Asynchronous WiFi scan state
unsigned long lastRssiScan      = 0;
const unsigned long RSSI_SCAN_INTERVAL = 5000; // 5 seconds
bool scanningNetworks = false;

// Web server
WebServer server(80);

// Debug blink LED
const int LED_PIN = 2;  // ESP32 onboard LED

// HTML control page
const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html, body { margin:0; padding:0; user-select:none; }
    #status, #distance, #reflect, #rssi { padding:0.5em; font-family: monospace; text-align:center; }
    #container { display: flex; flex-direction: column; align-items: center; gap: 10px; margin-top: 10px; }
    #dpad { display: grid; grid-template-columns: 50px 50px 50px; grid-template-rows: 50px 50px 50px; gap: 5px;
      grid-template-areas: 
        ". up ."
        "left stop right"
        ". down .";
    }
    #dpad .up    { grid-area: up; }
    #dpad .left  { grid-area: left; }
    #dpad .stop  { grid-area: stop; }
    #dpad .right { grid-area: right; }
    #dpad .down  { grid-area: down; }
    #dpad button { width: 50px; height: 50px; font-size: 1.5em; touch-action: none; user-select: none; }
    #actions { display: flex; gap: 10px; }
    #actions button { width: 80px; height: 80px; font-size: 1.2em; }
  </style>
</head>
<body>
  <div id="status">Connected to ESP32-Bot</div>
  <div id="distance">Distance: -- mm</div>
  <div id="reflect">Reflect: --</div>
  <div id="rssi">RSSI: -- dBm</div>
  <div id="container">
    <div id="dpad">
      <button class="up btn" onmousedown="startRC('forward')" onmouseup="stopRC()" ontouchstart="startRC('forward')" ontouchend="stopRC()">&uarr;</button>
      <button class="left btn" onmousedown="startRC('left')" onmouseup="stopRC()" ontouchstart="startRC('left')" ontouchend="stopRC()">&larr;</button>
      <button class="stop btn" onmousedown="startRC('stop')" onmouseup="stopRC()" onclick="send('stop')">&#9632;</button>
      <button class="right btn" onmousedown="startRC('right')" onmouseup="stopRC()" ontouchstart="startRC('right')" ontouchend="stopRC()">&rarr;</button>
      <button class="down btn" onmousedown="startRC('backward')" onmouseup="stopRC()" ontouchstart="startRC('backward')" ontouchend="stopRC()">&darr;</button>
    </div>
    <div id="actions">
      <button onclick="send('wall')">WF</button>
      <button onclick="send('reset_tof')">Reset ToF</button>
    </div>
  </div>
  <script>
    let stopTimer;
    function send(cmd) {
      fetch('/move?dir='+cmd)
        .then(r=>r.text())
        .then(t=>document.getElementById('status').innerText = t);
    }
    function startRC(cmd) {
      clearTimeout(stopTimer);
      send(cmd);
    }
    function stopRC() {
      stopTimer = setTimeout(() => send('stop'), 50);
    }
    setInterval(() => {
      fetch('/sensors')
        .then(r=>r.json())
        .then(data => {
          document.getElementById('distance').innerText = 'Distance: ' + data.distance + ' mm';
          document.getElementById('reflect').innerText  = 'Reflect: ' + (data.reflect ? 'Obstacle' : 'Clear');
          document.getElementById('rssi').innerText     = 'RSSI: ' + data.rssi + ' dBm';
        });
    }, 500);
  </script>
)rawliteral";

// Prototypes
void stopMotors();
void motorAForward(int);
void motorABackward(int);
void motorBForward(int);
void motorBBackward(int);
void initTofSensor();
void resetTofSensor();
uint16_t readSafeDistance();

void setup() {
  Serial.begin(115200);
  delay(100);
  
  // Debug LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // Turn on LED to indicate startup
  
  Serial.println("\n\n--- ESP32 Bot Starting ---");

  // Motor pins
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_EN,  OUTPUT);
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
  pinMode(motorB_EN,  OUTPUT);

  // Reflectance sensor
  pinMode(frontSensorPin, INPUT);

  // ToF XSHUT pin
  pinMode(TOF_XSHUT_PIN, OUTPUT);
  digitalWrite(TOF_XSHUT_PIN, HIGH);

  // I2C + ToF init
  initTofSensor();

  // PWM setup
  ledcAttach(motorA_EN, pwmFreq, pwmResolution);
  ledcAttach(motorB_EN, pwmFreq, pwmResolution);

  // Enable AP mode
  WiFi.mode(WIFI_AP);
  Serial.println("Starting WiFi AP...");
  if (WiFi.softAP(ssid, password)) {
    Serial.printf("AP '%s' started, IP=%s\n", ssid, WiFi.softAPIP().toString().c_str());
    debugBlink(2);
  } else {
    Serial.println("Failed to start AP!");
    debugBlink(10);
  }

  // HTTP routes
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", CONTROL_PAGE); });
  server.on("/move", HTTP_GET, []() {
    String dir = server.arg("dir");
    String resp = "Unknown command";
    if (dir == "reset_tof") { resetTofSensor(); resp = "ToF Sensor Reset"; }
    else if (digitalRead(frontSensorPin)==LOW && dir!="stop" && dir!="wall") {
      motorABackward(motorSpeed);
      motorBBackward(int(motorSpeed*leftMotorBias));
      delay(500); stopMotors(); wallFollowActive=false; resp="Obstacle! Reversed";
    }
    else if (dir=="forward")  { wallFollowActive=false; motorAForward(motorSpeed); motorBForward(int(motorSpeed*leftMotorBias)); resp="Forward"; }
    else if (dir=="backward") { wallFollowActive=false; motorABackward(motorSpeed); motorBBackward(int(motorSpeed*leftMotorBias)); resp="Backward"; }
    else if (dir=="left")     { wallFollowActive=false; motorAForward(motorSpeed); motorBBackward(int(motorSpeed*leftMotorBias)); resp="Left"; }
    else if (dir=="right")    { wallFollowActive=false; motorABackward(motorSpeed); motorBForward(int(motorSpeed*leftMotorBias)); resp="Right"; }
    else if (dir=="stop")     { wallFollowActive=false; stopMotors(); resp="Stopped"; }
    else if (dir=="wall")     { wallFollowActive=true; resp="Wall-Follow ON"; }
    server.send(200, "text/plain", resp);
  });
  server.on("/sensors", HTTP_GET, []() {
    static uint16_t cachedDistance = 100;
    static bool     cachedReflect  = false;
    static unsigned long lastUpdate = 0;
    const unsigned long UPDATE_INT = 250;
    unsigned long now = millis();
    if (now - lastUpdate >= UPDATE_INT) {
      cachedDistance = readSafeDistance();
      cachedReflect  = (digitalRead(frontSensorPin)==LOW);
      lastUpdate = now;
    }
    String j = String("{\"distance\":") + cachedDistance +
               ",\"reflect\":" + (cachedReflect?"1":"0") +
               ",\"rssi\":" + cachedRssi + "}";
    server.send(200, "application/json", j);
  });
  server.on("/ping", HTTP_GET, [](){ server.send(200,"text/plain","pong"); });

  server.begin();
  debugBlink(3);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  server.handleClient();
  unsigned long now = millis();

  // Periodic ToF reset
  if (now - lastSensorResetTime > RESET_INTERVAL) {
    resetTofSensor();
  }

  // Asynchronous WiFi RSSI scan every 5s
  if (!scanningNetworks && now - lastRssiScan >= RSSI_SCAN_INTERVAL) {
    scanningNetworks = true;
    WiFi.scanDelete();
    WiFi.scanNetworks(true);  // async scan
  }
  if (scanningNetworks) {
    int n = WiFi.scanComplete();
    if (n >= 0) {
      int best = -128;
      for (int i = 0; i < n; i++) {
        int r = WiFi.RSSI(i);
        if (r > best) best = r;
      }
      cachedRssi = best;
      scanningNetworks = false;
      lastRssiScan = now;
    }
  }

  // Obstacle avoidance
  if (digitalRead(frontSensorPin) == LOW) {
    motorABackward(motorSpeed);
    motorBBackward(int(motorSpeed*leftMotorBias));
    delay(500);
    stopMotors();
    wallFollowActive = false;
    return;
  }

  // Wall-follow logic
  static unsigned long lastWF = 0;
  if (wallFollowActive && now - lastWF >= 100) {
    lastWF = now;
    uint16_t dist = readSafeDistance();
    if (dist < 2000) {
      int err = int(dist) - 100;
      if (abs(err) < 20) {
        motorAForward(motorSpeed);
        motorBForward(int(motorSpeed*leftMotorBias));
      } else if (err > 0) {
        motorAForward(motorSpeed);
        motorBForward(int((motorSpeed/2)*leftMotorBias));
      } else {
        motorAForward(int(motorSpeed/2));
        motorBForward(int(motorSpeed*leftMotorBias));
      }
    } else {
      motorAForward(motorSpeed);
      motorBForward(int(motorSpeed*leftMotorBias));
    }
  }
}

// --- Helpers ---

void debugBlink(int c) {
  for (int i = 0; i < c; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void initTofSensor() {
  digitalWrite(TOF_XSHUT_PIN, LOW);
  delay(10);
  digitalWrite(TOF_XSHUT_PIN, HIGH);
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);
  ranger.begin(0x29);
  Serial.println("ToF sensor initialized");
  lastSensorResetTime = millis();
}

void resetTofSensor() {
  Serial.println("Resetting ToF sensor");
  digitalWrite(TOF_XSHUT_PIN, LOW);
  delay(10);
  digitalWrite(TOF_XSHUT_PIN, HIGH);
  delay(10);
  initTofSensor();
}

uint16_t readSafeDistance() {
  unsigned long now = millis();
  if (now - lastSensorRead < MIN_READ_INTERVAL) return lastValidDistance;
  lastSensorRead = now;
  uint16_t cm = ranger.ReadDistance();
  if (cm == 0) {
    resetTofSensor();
    return lastValidDistance;
  }
  lastValidDistance = cm * 10;
  return lastValidDistance;
}

void stopMotors() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  ledcWrite(0, 0);
  digitalWrite(motorB_IN1, LOW);
  digitalWrite(motorB_IN2, LOW);
  ledcWrite(1, 0);
}

void motorAForward(int s) { digitalWrite(motorA_IN1, LOW); digitalWrite(motorA_IN2, HIGH); ledcWrite(0, constrain(s,0,255)); }
void motorABackward(int s){ digitalWrite(motorA_IN1, HIGH);digitalWrite(motorA_IN2, LOW);ledcWrite(0, constrain(s,0,255)); }
void motorBForward(int s) { digitalWrite(motorB_IN1, LOW); digitalWrite(motorB_IN2, HIGH); ledcWrite(1, constrain(s,0,255)); }
void motorBBackward(int s){ digitalWrite(motorB_IN1, HIGH);digitalWrite(motorB_IN2, LOW);ledcWrite(1, constrain(s,0,255)); }
