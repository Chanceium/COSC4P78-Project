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

// Web server
WebServer server(80);

// HTML control page
const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html, body { margin:0; padding:0; user-select:none; }
    #status, #distance, #reflect { padding:0.5em; font-family: monospace; text-align:center; }
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
        });
    }, 500);
  </script>
</body>
</html>
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
  digitalWrite(TOF_XSHUT_PIN, HIGH);  // ensure sensor is enabled

  // I2C on custom pins + ToF init
  initTofSensor();

  // PWM setup
  ledcAttach(motorA_EN, pwmFreq, pwmResolution);
  ledcAttach(motorB_EN, pwmFreq, pwmResolution);

  // Start Wi-Fi AP
  WiFi.softAP(ssid, password);
  Serial.printf("AP '%s' started, IP=%s\n", ssid, WiFi.softAPIP().toString().c_str());

  // HTTP handlers
  server.on("/", []() {
    server.send_P(200, "text/html", CONTROL_PAGE);
  });

  server.on("/move", []() {
    String dir = server.arg("dir");
    if (dir == "reset_tof") {
      resetTofSensor();
      server.send(200, "text/plain", "ToF Sensor Reset");
      return;
    }
    // obstacle avoidance
    if (digitalRead(frontSensorPin) == LOW && dir != "stop" && dir != "wall") {
      motorABackward(motorSpeed);
      motorBBackward(int(motorSpeed * leftMotorBias));
      delay(2000);
      stopMotors();
      wallFollowActive = false;
      server.send(200, "text/plain", "Obstacle! Reversed then stopped.");
      return;
    }
    if (dir == "forward") {
      wallFollowActive = false;
      motorAForward(motorSpeed);
      motorBForward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Forward");
    }
    else if (dir == "backward") {
      wallFollowActive = false;
      motorABackward(motorSpeed);
      motorBBackward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Backward");
    }
    else if (dir == "left") {
      wallFollowActive = false;
      motorAForward(motorSpeed);
      motorBBackward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Left");
    }
    else if (dir == "right") {
      wallFollowActive = false;
      motorABackward(motorSpeed);
      motorBForward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Right");
    }
    else if (dir == "stop") {
      wallFollowActive = false;
      stopMotors();
      server.send(200, "text/plain", "Stopped");
    }
    else if (dir == "wall") {
      wallFollowActive = true;
      server.send(200, "text/plain", "Wall-Follow ON");
    }
  });

  server.on("/sensors", []() {
    static uint16_t cachedDistance = 100;
    static bool cachedReflect = false;
    static unsigned long lastSensorUpdate = 0;
    const unsigned long SENSOR_UPDATE_INTERVAL = 250;

    unsigned long now = millis();
    if (now - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
      cachedDistance = readSafeDistance();
      cachedReflect = (digitalRead(frontSensorPin) == LOW);
      lastSensorUpdate = now;
    }
    String json = String("{\"distance\":") + cachedDistance +
                  ",\"reflect\":" + (cachedReflect ? "1" : "0") + "}";
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();
  unsigned long now = millis();
  static unsigned long lastLoop = 0;
  if (now - lastLoop < 10) {
    delay(1);
    return;
  }
  lastLoop = now;

  // periodic hardware reset
  if (now - lastSensorResetTime > RESET_INTERVAL) {
    resetTofSensor();
  }

  // immediate obstacle avoidance
  if (digitalRead(frontSensorPin) == LOW) {
    motorABackward(motorSpeed);
    motorBBackward(int(motorSpeed * leftMotorBias));
    delay(2000);
    stopMotors();
    wallFollowActive = false;
    return;
  }

  // wall-follow logic
  static unsigned long lastWFRead = 0;
  if (wallFollowActive && now - lastWFRead >= 100) {
    lastWFRead = now;
    uint16_t dist = readSafeDistance();
    if (dist < 2000) {
      int err = int(dist) - 100;
      if (abs(err) < 20) {
        motorAForward(motorSpeed);
        motorBForward(int(motorSpeed * leftMotorBias));
      }
      else if (err > 0) {
        motorAForward(motorSpeed);
        int adj = (motorSpeed > 1 ? motorSpeed / 2 : 1);
        motorBForward(int(adj * leftMotorBias));
      }
      else {
        int adj = (motorSpeed > 1 ? motorSpeed / 2 : 1);
        motorAForward(adj);
        motorBForward(int(motorSpeed * leftMotorBias));
      }
      if (dist != lastValidDistance) {
        Serial.printf("Wall distance: %u mm\n", dist);
      }
    }
    else {
      Serial.println("Invalid distance reading in wall follow mode");
      motorAForward(motorSpeed);
      motorBForward(int(motorSpeed * leftMotorBias));
    }
  }
}

// --- ToF Helpers ---
void initTofSensor() {
  digitalWrite(TOF_XSHUT_PIN, HIGH);
  delay(10);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(10);
  ranger.begin(0x29);
  lastSensorResetTime = millis();
  Serial.println("ToF sensor initialized");
}

void resetTofSensor() {
  Serial.println("Resetting ToF sensor via XSHUT");
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

  uint16_t mm = cm * 10;
  lastValidDistance = mm;
  return mm;
}

// --- Motor Helpers ---
void stopMotors() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  ledcWrite(motorA_EN, 0);
  digitalWrite(motorB_IN1, LOW);
  digitalWrite(motorB_IN2, LOW);
  ledcWrite(motorB_EN, 0);
}

void motorAForward(int spd) {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, HIGH);
  ledcWrite(motorA_EN, constrain(spd, 0, 255));
}

void motorABackward(int spd) {
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  ledcWrite(motorA_EN, constrain(spd, 0, 255));
}

void motorBForward(int spd) {
  digitalWrite(motorB_IN1, LOW);
  digitalWrite(motorB_IN2, HIGH);
  ledcWrite(motorB_EN, constrain(spd, 0, 255));
}

void motorBBackward(int spd) {
  digitalWrite(motorB_IN1, HIGH);
  digitalWrite(motorB_IN2, LOW);
  ledcWrite(motorB_EN, constrain(spd, 0, 255));
}
