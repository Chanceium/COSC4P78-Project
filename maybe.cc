#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <math.h>
#include <map>

// ----- WIFI CONFIGURATION -----
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Create asynchronous web server and WebSocket on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ----- PIN DEFINITIONS -----
// I2C pins for the VL53L0X ToF sensor
#define SDA_PIN 12    // Updated to D12 for the ToF sensor
#define SCL_PIN 13    // Updated to D13 for the ToF sensor

// Motor A (Left Motor) pins
const int motorA_IN1 = 27;
const int motorA_IN2 = 26;
const int motorA_EN  = 14;  // PWM enable for Motor A

// Motor B (Right Motor) pins
const int motorB_IN1 = 25;
const int motorB_IN2 = 33;
const int motorB_EN  = 32;  // PWM enable for Motor B

// Reflectance sensor at the front
const int frontSensorPin = 35;  // Digital input pin

// ----- GLOBAL OBJECTS -----
VL53L0X tofSensor;  // ToF sensor object

// ----- PWM CONFIGURATION -----
const int pwmFreq = 30000;
const int pwmResolution = 8;
const int pwmChannelA = 0;  // PWM channel for Motor A
const int pwmChannelB = 1;  // PWM channel for Motor B

// Default motor speed (0–255)
int motorSpeed = 200;

// ----- DYNAMIC GRID MAPPING CONFIGURATION -----
// Each grid cell represents a 100mm x 100mm area.
const int CELL_SIZE_MM = 100;

// We use a sparse mapping technique by storing only occupied cells.
// A cell is identified by its (x, y) grid coordinates.
struct CellCoord {
  int x;
  int y;
  // Needed for ordering in std::map:
  bool operator<(const CellCoord& other) const {
    if (x != other.x)
      return x < other.x;
    return y < other.y;
  }
};

// Sparse map: if a cell is occupied, we set its value to 1.
std::map<CellCoord, int> gridMap;

// Dynamic boundaries for the mapped area.
int minX = 0, maxX = 0, minY = 0, maxY = 0;

// ----- ROBOT STATE VARIABLES -----
// The robot’s position is tracked as its center (in mm) and its heading (in radians).
float posX = 0.0;    // in mm
float posY = 0.0;    // in mm
float heading = 0.0; // radians (0 means "east")
unsigned long lastTime = 0;  // For dead reckoning timing

// The robot’s physical size (~100mm x 100mm)
const int ROBOT_SIZE_MM = 100;

// ----- UTILITY FUNCTION -----
// Convert a position (in mm) to a grid cell coordinate using rounding.
// A value of 0 mm maps to cell 0; 100 mm maps to cell 1; -100 mm to cell -1.
int posToCell(float pos) {
  return (int) round(pos / (float)CELL_SIZE_MM);
}

// ----- FUNCTION: Send Mapping Data via WebSocket -----
// In addition to sending the occupied cells, we now send a robot footprint (as a bounding box)
// based on its center (posX, posY) and its size (ROBOT_SIZE_MM).
void sendMapData(uint16_t leftDistance) {
  // Calculate the wall point using the left ToF sensor.
  // The wall is assumed to be to the left (i.e. heading + PI/2).
  float wallX = posX + leftDistance * cos(heading + PI / 2);
  float wallY = posY + leftDistance * sin(heading + PI / 2);

  // Convert positions to grid cell indices.
  int robotCenterCellX = posToCell(posX);
  int robotCenterCellY = posToCell(posY);
  int wallCellX  = posToCell(wallX);
  int wallCellY  = posToCell(wallY);

  // Calculate the robot's footprint in mm:
  // It extends half its size in each direction from its center.
  float halfSize = ROBOT_SIZE_MM / 2.0;
  int robotMinCellX = posToCell(posX - halfSize);
  int robotMaxCellX = posToCell(posX + halfSize);
  int robotMinCellY = posToCell(posY - halfSize);
  int robotMaxCellY = posToCell(posY + halfSize);

  // Mark the wall cell as occupied.
  CellCoord cellWall = { wallCellX, wallCellY };
  gridMap[cellWall] = 1;

  // Update dynamic boundaries using both the robot's footprint and the wall cell.
  if (robotMinCellX < minX) minX = robotMinCellX;
  if (robotMaxCellX > maxX) maxX = robotMaxCellX;
  if (robotMinCellY < minY) minY = robotMinCellY;
  if (robotMaxCellY > maxY) maxY = robotMaxCellY;

  if (wallCell.x < minX) minX = wallCell.x;
  if (wallCell.x > maxX) maxX = wallCell.x;
  if (wallCell.y < minY) minY = wallCell.y;
  if (wallCell.y > maxY) maxY = wallCell.y;

  // Build a JSON string:
  // 1. "cells": an array of occupied cell coordinates.
  // 2. "robot": an object representing the robot footprint as a bounding box.
  // 3. "bounds": current dynamic boundaries of the mapped grid.
  String json = "{";
  json += "\"cells\":[";
  bool first = true;
  for (auto const& kv : gridMap) {
    if (!first) {
      json += ",";
    }
    json += "{\"x\":" + String(kv.first.x) + ",\"y\":" + String(kv.first.y) + "}";
    first = false;
  }
  json += "],";
  json += "\"robot\":{\"minX\":" + String(robotMinCellX) +
          ",\"minY\":" + String(robotMinCellY) +
          ",\"maxX\":" + String(robotMaxCellX) +
          ",\"maxY\":" + String(robotMaxCellY) + "},";
  json += "\"bounds\":{\"minX\":" + String(minX) +
          ",\"minY\":" + String(minY) +
          ",\"maxX\":" + String(maxX) +
          ",\"maxY\":" + String(maxY) + "}";
  json += "}";
  
  ws.textAll(json);
}

// ----- WEBSOCKET EVENT HANDLER -----
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
                        AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  }
}

void setup() {
  Serial.begin(115200);

  // ----- SETUP WIFI -----
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("WiFi connected. IP address: " + WiFi.localIP().toString());

  // ----- SETUP WEBSOCKET & SERVER -----
  ws.onEvent(onWebSocketEvent);
  server.addHandler(&ws);
  
  // Serve an HTML page that displays the dynamic grid map.
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html",
      "<!DOCTYPE html><html><head><title>Dynamic Robot Map</title></head>"
      "<body><h1>Live Robot Map</h1>"
      "<canvas id='mapCanvas' width='500' height='500' style='border:1px solid #000;'></canvas>"
      "<script>"
      "var canvas = document.getElementById('mapCanvas');"
      "var ctx = canvas.getContext('2d');"
      "var cellSize = 10; // Each grid cell drawn as 10x10 pixels"
      "var ws = new WebSocket('ws://' + window.location.hostname + '/ws');"
      "ws.onmessage = function(event) {"
        "var data = JSON.parse(event.data);"
        "ctx.clearRect(0, 0, canvas.width, canvas.height);"
        "/* Calculate offset so that the minimum cell is drawn at (0,0) */"
        "var offsetX = data.bounds.minX;"
        "var offsetY = data.bounds.minY;"
        "/* Draw each occupied cell (blue) */"
        "data.cells.forEach(function(cell) {"
          "var x = (cell.x - offsetX) * cellSize;"
          "var y = (cell.y - offsetY) * cellSize;"
          "ctx.fillStyle = 'blue';"
          "ctx.fillRect(x, y, cellSize, cellSize);"
        "});"
        "/* Draw robot footprint (red rectangle) */"
        "var rx = (data.robot.minX - offsetX) * cellSize;"
        "var ry = (data.robot.minY - offsetY) * cellSize;"
        "var rwidth = (data.robot.maxX - data.robot.minX + 1) * cellSize;"
        "var rheight = (data.robot.maxY - data.robot.minY + 1) * cellSize;"
        "ctx.fillStyle = 'red';"
        "ctx.fillRect(rx, ry, rwidth, rheight);"
        "/* Optionally, draw grid lines based on dynamic bounds */"
        "var gridWidth = data.bounds.maxX - data.bounds.minX + 1;"
        "var gridHeight = data.bounds.maxY - data.bounds.minY + 1;"
        "ctx.strokeStyle = 'gray';"
        "for (var i = 0; i <= gridWidth; i++) {"
          "ctx.beginPath();"
          "ctx.moveTo(i * cellSize, 0);"
          "ctx.lineTo(i * cellSize, gridHeight * cellSize);"
          "ctx.stroke();"
        "}"
        "for (var j = 0; j <= gridHeight; j++) {"
          "ctx.beginPath();"
          "ctx.moveTo(0, j * cellSize);"
          "ctx.lineTo(gridWidth * cellSize, j * cellSize);"
          "ctx.stroke();"
        "}"
      "};"
      "</script></body></html>");
  });
  
  server.begin();

  // ----- SETUP MOTOR & SENSOR PINS -----
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_EN, OUTPUT);
  
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  
  pinMode(frontSensorPin, INPUT);
  
  // Initialize I2C for the ToF sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  
  tofSensor.init();
  tofSensor.setTimeout(500);
  
  // Configure PWM channels for motor control
  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcAttachPin(motorA_EN, pwmChannelA);
  
  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(motorB_EN, pwmChannelB);
  
  Serial.println("Setup complete. Starting wall following and dynamic grid mapping...");
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;  // Elapsed time in seconds
  lastTime = currentTime;
  
  // ----- DEAD RECKONING: UPDATE ROBOT POSITION -----
  // Assume a constant forward speed of 100 mm/s.
  posX += 100.0 * dt * cos(heading);
  posY += 100.0 * dt * sin(heading);
  
  // ----- CHECK FRONT REFLECTANCE SENSOR -----
  int frontState = digitalRead(frontSensorPin);
  
  if (frontState == HIGH) {
    Serial.println("Front sensor triggered. Backing up and turning right.");
    
    // Back up: run both motors in reverse.
    motorABackward(motorSpeed);
    motorBBackward(motorSpeed);
    delay(500);
    stopMotors();
    delay(200);
    
    // Update heading: assume a 90° right turn.
    heading -= PI / 2.0;
    if (heading < 0) heading += 2 * PI;
    
    // Execute turn: left motor forward, right motor backward.
    motorAForward(motorSpeed);
    motorBBackward(motorSpeed);
    delay(300);
    stopMotors();
    delay(200);
  } else {
    // ----- WALL-FOLLOWING USING THE LEFT ToF SENSOR -----
    uint16_t leftDistance = tofSensor.readRangeSingleMillimeters();
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" mm");
    
    // Adjust motor speeds based on sensor readings.
    if (leftDistance < 60) {
      Serial.println("Too close to wall. Adjusting right.");
      motorAForward(motorSpeed / 2);  // Slow left motor
      motorBForward(motorSpeed);
    } else if (leftDistance > 100) {
      Serial.println("Too far from wall. Adjusting left.");
      motorAForward(motorSpeed);
      motorBForward(motorSpeed / 2);  // Slow right motor
    } else {
      Serial.println("Wall distance OK. Moving straight.");
      motorAForward(motorSpeed);
      motorBForward(motorSpeed);
    }
    
    // Send updated mapping data (this updates the sparse map and dynamic bounds).
    sendMapData(leftDistance);
  }
  
  delay(50);  // Small delay for sensor stability
}

// ----- MOTOR CONTROL FUNCTIONS -----
void stopMotors() {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, LOW);
  ledcWrite(pwmChannelA, 0);
  
  digitalWrite(motorB_IN1, LOW);
  digitalWrite(motorB_IN2, LOW);
  ledcWrite(pwmChannelB, 0);
}

void motorAForward(int speed) {
  digitalWrite(motorA_IN1, HIGH);
  digitalWrite(motorA_IN2, LOW);
  ledcWrite(pwmChannelA, speed);
}

void motorABackward(int speed) {
  digitalWrite(motorA_IN1, LOW);
  digitalWrite(motorA_IN2, HIGH);
  ledcWrite(pwmChannelA, speed);
}

void motorBForward(int speed) {
  digitalWrite(motorB_IN1, HIGH);
  digitalWrite(motorB_IN2, LOW);
  ledcWrite(pwmChannelB, speed);
}

void motorBBackward(int speed) {
  digitalWrite(motorB_IN1, LOW);
  digitalWrite(motorB_IN2, HIGH);
  ledcWrite(pwmChannelB, speed);
}
