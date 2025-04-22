#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WebServer.h>

// ----- USER CONFIG -----
const char* ssid = "ESP32-Bot";
const char* password = "Password1!";

// Bias to boost left motor (Motor B)
float leftMotorBias = 1.13;

// Base speed (0–255)
int motorSpeed = 200;

// ----- PIN DEFINITIONS -----
// ToF sensor on the left
#define SDA_PIN 21
#define SCL_PIN 22
#define TOF_XSHUT_PIN 19  // Add XSHUT pin for ToF reset capability

// Motor A (Right Motor)
const int motorA_IN1 = 27;
const int motorA_IN2 = 26;
const int motorA_EN = 14;

// Motor B (Left Motor)
const int motorB_IN1 = 25;
const int motorB_IN2 = 33;
const int motorB_EN = 32;

// Reflectance sensor at the front (active LOW)
const int frontSensorPin = 35;

// PWM settings
const int pwmFreq = 30000;
const int pwmResolution = 8;

// Wall-follow flag
bool wallFollowActive = false;

// ToF sensor
VL53L0X tofSensor;
unsigned long lastSensorResetTime = 0;
const unsigned long RESET_INTERVAL = 60000;  // Reset sensor every 60 seconds (reduced frequency)
const int MAX_VALID_DISTANCE = 2000;         // Maximum valid distance in mm
int lastValidDistance = 100;                 // Store last valid reading
unsigned long lastTimeoutTime = 0;
int timeoutCount = 0;

// Rate limiting variables for all sensor operations
unsigned long lastSensorRead = 0;
const unsigned long MIN_READ_INTERVAL = 50;  // Minimum time between sensor reads

// Web server
WebServer server(80);

// HTML control page with sensor readouts
const char CONTROL_PAGE[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    html, body { margin:0; padding:0; user-select:none; }
    #status, #distance, #reflect { padding:0.5em; font-family: monospace; text-align:center; }
    #container {
      display: flex;
      flex-direction: column;
      align-items: center;
      gap: 10px;
      margin-top: 10px;
    }
    #dpad {
      display: grid;
      grid-template-columns: 50px 50px 50px;
      grid-template-rows: 50px 50px 50px;
      gap: 5px;
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
    #dpad button {
      width: 50px;
      height: 50px;
      font-size: 1.5em;
      touch-action: none;
      user-select: none;
    }
    #actions {
      display: flex;
      gap: 10px;
    }
    #actions button {
      width: 80px;
      height: 80px;
      font-size: 1.2em;
    }
  </style>
</head>
<body>
  <div id="status">Connected to ESP32‑Bot</div>
  <div id="distance">Distance: -- mm</div>
  <div id="reflect">Reflect: --</div>
  <div id="container">
    <div id="dpad">
      <button class="up btn"
        onmousedown="startRC('forward')" onmouseup="stopRC()"
        ontouchstart="startRC('forward')" ontouchend="stopRC()">&uarr;</button>
      <button class="left btn"
        onmousedown="startRC('left')" onmouseup="stopRC()"
        ontouchstart="startRC('left')" ontouchend="stopRC()">&larr;</button>
      <button class="stop btn"
        onmousedown="startRC('stop')" onmouseup="stopRC()"
        onclick="send('stop')">&#9632;</button>
      <button class="right btn"
        onmousedown="startRC('right')" onmouseup="stopRC()"
        ontouchstart="startRC('right')" ontouchend="stopRC()">&rarr;</button>
      <button class="down btn"
        onmousedown="startRC('backward')" onmouseup="stopRC()"
        ontouchstart="startRC('backward')" ontouchend="stopRC()">&darr;</button>
    </div>
    <div id="actions">
      <button onclick="send('wall')">WF</button>
      <button onclick="send('reset_tof')">Reset ToF</button>
    </div>
  </div>
  <script>
    let stopTimer;
    function send(cmd) {
      fetch('/move?dir=' + cmd)
        .then(r => r.text())
        .then(t => document.getElementById('status').innerText = t);
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
        .then(r => r.json())
        .then(data => {
          document.getElementById('distance').innerText = 'Distance: ' + data.distance + ' mm';
          document.getElementById('reflect').innerText = 'Reflect: ' + (data.reflect ? 'Obstacle' : 'Clear');
        });
    }, 500);
  </script>
</body>
</html>
)rawliteral";

// Motor helper prototypes
void stopMotors();
void motorAForward(int);
void motorABackward(int);
void motorBForward(int);
void motorBBackward(int);

// ToF sensor helpers
void initTofSensor();
void resetTofSensor();
uint16_t readSafeDistance();

void setup() {
  Serial.begin(115200);
  delay(100);

  // Motor pins
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_EN, OUTPUT);
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
  pinMode(motorB_EN, OUTPUT);

  // Reflectance sensor
  pinMode(frontSensorPin, INPUT);

  // XSHUT pin for ToF reset capability
  if (TOF_XSHUT_PIN > 0) {
    pinMode(TOF_XSHUT_PIN, OUTPUT);
    digitalWrite(TOF_XSHUT_PIN, LOW);  // Reset the sensor
    delay(100);
    digitalWrite(TOF_XSHUT_PIN, HIGH); // Enable the sensor
    delay(100);
  }

  // I2C + ToF init
  Wire.begin(SDA_PIN, SCL_PIN);
  initTofSensor();

  // PWM setup
  ledcAttach(motorA_EN, pwmFreq, pwmResolution);
  ledcAttach(motorB_EN, pwmFreq, pwmResolution);

  // Start Wi-Fi AP
  WiFi.softAP(ssid, password);
  Serial.printf("AP '%s' started, IP=%s\n", ssid, WiFi.softAPIP().toString().c_str());

  // Serve control page
  server.on("/", []() {
    server.send_P(200, "text/html", CONTROL_PAGE);
  });

  // Move commands
  server.on("/move", []() {
    String dir = server.arg("dir");
    
    if (dir == "reset_tof") {
      resetTofSensor();
      server.send(200, "text/plain", "ToF Sensor Reset");
      return;
    }
    
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
    } else if (dir == "backward") {
      wallFollowActive = false;
      motorABackward(motorSpeed);
      motorBBackward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Backward");
    } else if (dir == "left") {
      wallFollowActive = false;
      motorAForward(motorSpeed);
      motorBBackward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Left");
    } else if (dir == "right") {
      wallFollowActive = false;
      motorABackward(motorSpeed);
      motorBForward(int(motorSpeed * leftMotorBias));
      server.send(200, "text/plain", "Right");
    } else if (dir == "stop") {
      wallFollowActive = false;
      stopMotors();
      server.send(200, "text/plain", "Stopped");
    } else if (dir == "wall") {
      wallFollowActive = true;
      server.send(200, "text/plain", "Wall‑Follow ON");
    }
  });

  // Prevent excessive sensor reading by caching readings
  static uint16_t cachedDistance = 100;  // Initialize with a reasonable default
  static bool cachedReflect = false;
  static unsigned long lastSensorUpdate = 0;
  const unsigned long SENSOR_UPDATE_INTERVAL = 250; // 250ms between updates
    
  // Sensor readings
  server.on("/sensors", []() {
    // Rate limit sensor readings to avoid buffer overflow
    unsigned long currentTime = millis();
    if (currentTime - lastSensorUpdate >= SENSOR_UPDATE_INTERVAL) {
      cachedDistance = readSafeDistance();
      cachedReflect = digitalRead(frontSensorPin) == LOW;
      lastSensorUpdate = currentTime;
    }
    
    String json = String("{\"distance\":") + cachedDistance + String(",\"reflect\":") + (cachedReflect ? "1" : "0") + String("}");
    server.send(200, "application/json", json);
  });

  server.begin();
}

void loop() {
  server.handleClient();

  // Static variables to manage loop timing and prevent buffer overruns
  static unsigned long lastLoopTime = 0;
  const unsigned long MAIN_LOOP_INTERVAL = 10; // 10ms minimum between loop iterations
  
  // Rate limit the main loop to prevent buffer overflow
  unsigned long currentTime = millis();
  if (currentTime - lastLoopTime < MAIN_LOOP_INTERVAL) {
    delay(1); // Short yield to other processes
    return;
  }
  lastLoopTime = currentTime;
  
  // Check if sensor needs periodic reset (less frequently)
  if (currentTime - lastSensorResetTime > RESET_INTERVAL) {
    resetTofSensor();
  }

  // Global front sensor guard
  if (digitalRead(frontSensorPin) == LOW) {
    motorABackward(motorSpeed);
    motorBBackward(int(motorSpeed * leftMotorBias));
    delay(2000);
    stopMotors();
    wallFollowActive = false;
    return;
  }

  // Add rate limiting for sensor polling
  static unsigned long lastWallFollowSensorRead = 0;
  const unsigned long SENSOR_READ_INTERVAL = 100; // 100ms between readings
  
  if (wallFollowActive) {
    // Only read the sensor at specific intervals to avoid buffer overflow
    unsigned long currentTime = millis();
    if (currentTime - lastWallFollowSensorRead >= SENSOR_READ_INTERVAL) {
      lastWallFollowSensorRead = currentTime;
      
      uint16_t dist = readSafeDistance();
      
      // Only adjust motors if we have a valid reading
      if (dist < MAX_VALID_DISTANCE) {
        int err = int(dist) - 100;
        if (abs(err) < 20) {
          // In the sweet spot - go straight
          motorAForward(motorSpeed);
          motorBForward(int(motorSpeed * leftMotorBias));
        } else if (err > 0) {
          // Too far from wall - steer toward it
          motorAForward(motorSpeed);
          // Ensure we never divide by zero or use a zero speed
          int adjustedSpeed = (motorSpeed > 1) ? (motorSpeed / 2) : 1;
          motorBForward(int(adjustedSpeed * leftMotorBias));
        } else {
          // Too close to wall - steer away
          // Ensure we never divide by zero or use a zero speed
          int adjustedSpeed = (motorSpeed > 1) ? (motorSpeed / 2) : 1;
          motorAForward(adjustedSpeed);
          motorBForward(int(motorSpeed * leftMotorBias));
        }
        
        // Log healthy reading
        if (dist != lastValidDistance) {
          Serial.print("Wall distance: ");
          Serial.print(dist);
          Serial.println(" mm");
        }
      } else {
        // Invalid reading, stop or use last known good value
        Serial.println("Invalid distance reading in wall follow mode");
        // Fall back to last valid distance or just drive straight
        motorAForward(motorSpeed);
        motorBForward(int(motorSpeed * leftMotorBias));
      }
    }
    
    // Short delay to prevent CPU hogging
    delay(10);
  }
}

// --- New ToF Functions ---

void initTofSensor() {
  tofSensor.init();
  tofSensor.setTimeout(500);
  tofSensor.setMeasurementTimingBudget(20000);
  
  // Try using single shot mode instead of continuous
  // tofSensor.startContinuous(33);  // Original code
  
  // Set signal rate limit higher to reduce chance of timeouts
  tofSensor.setSignalRateLimit(0.25);
  
  // Use more conservative settings for better reliability
  tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 16);
  tofSensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 12);
  
  // Use single measurement mode instead of continuous to prevent buffer overflow
  // tofSensor.startContinuous(50);  // Removing continuous mode
  
  lastSensorResetTime = millis();
  Serial.println("ToF sensor initialized");
}

void resetTofSensor() {
  Serial.println("Resetting ToF sensor");
  
  // Hardware reset if XSHUT pin is connected
  if (TOF_XSHUT_PIN > 0) {
    digitalWrite(TOF_XSHUT_PIN, LOW);
    delay(50);
    digitalWrite(TOF_XSHUT_PIN, HIGH);
    delay(50);
  }
  
  // Clear I2C bus
  Wire.flush();
  
  // Re-init the sensor (no need to stop continuous since we're not using it)
  // tofSensor.stopContinuous(); // Removed as we're using single mode
  delay(100);
  
  // Reset the Wire/I2C interface
  Wire.end();
  delay(100);
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  
  // Re-initialize the sensor
  initTofSensor();
  
  lastSensorResetTime = millis();
  timeoutCount = 0;
}

uint16_t readSafeDistance() {
  // Rate-limit sensor reads to prevent I2C buffer issues
  unsigned long currentTime = millis();
  if (currentTime - lastSensorRead < MIN_READ_INTERVAL) {
    return lastValidDistance;  // Return previous reading if called too soon
  }
  lastSensorRead = currentTime;
  
  uint16_t distance = 0;
  
  // Safety check - protect against errors
  try {
    // Use single-shot reading instead of continuous to prevent buffer overflow
    distance = tofSensor.readRangeSingleMillimeters();
  } catch (...) {
    Serial.println("Error reading sensor");
    return lastValidDistance;
  }
  
  // Check for timeout or invalid reading
  if (tofSensor.timeoutOccurred() || distance > MAX_VALID_DISTANCE || distance == 65535) {
    Serial.print("ToF sensor timeout or invalid reading: ");
    Serial.println(distance);
    
    timeoutCount++;
    lastTimeoutTime = millis();
    
    // If we've had multiple timeouts in a row, reset the sensor
    if (timeoutCount > 5) {
      resetTofSensor();
    }
    
    return lastValidDistance; // Return the last valid reading
  }
  
  // Valid reading
  timeoutCount = 0;
  lastValidDistance = distance;
  return distance;
}

// --- MOTOR HELPERS ---
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
