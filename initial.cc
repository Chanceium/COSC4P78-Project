#include <Wire.h>
#include <VL53L0X.h>

// ----- PIN DEFINITIONS -----
// I2C pins for the VL53L0X ToF sensor (mounted on the left)
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
VL53L0X tofSensor;  // Create a ToF sensor object

// ----- PWM CONFIGURATION -----
const int pwmFreq = 30000;
const int pwmResolution = 8;
const int pwmChannelA = 0;  // PWM channel for Motor A enable
const int pwmChannelB = 1;  // PWM channel for Motor B enable

// Default motor speed (0–255)
int motorSpeed = 200;

void setup() {
  Serial.begin(115200);

  // Set motor control pins as outputs
  pinMode(motorA_IN1, OUTPUT);
  pinMode(motorA_IN2, OUTPUT);
  pinMode(motorA_EN, OUTPUT);
  
  pinMode(motorB_IN1, OUTPUT);
  pinMode(motorB_IN2, OUTPUT);
  pinMode(motorB_EN, OUTPUT);
  
  // Set up the reflectance sensor input
  pinMode(frontSensorPin, INPUT);
  
  // Initialize I2C on the designated pins for the ToF sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  
  // Initialize the VL53L0X sensor
  tofSensor.init();
  tofSensor.setTimeout(500);
  // Optionally, you can start continuous measurements:
  // tofSensor.startContinuous();

  // Configure the ESP32 LEDC PWM channels for motor enable control
  ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
  ledcAttachPin(motorA_EN, pwmChannelA);
  
  ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
  ledcAttachPin(motorB_EN, pwmChannelB);
  
  Serial.println("Setup complete. Starting wall following...");
}

void loop() {
  // ----- CHECK THE FRONT REFLECTANCE SENSOR -----
  int frontState = digitalRead(frontSensorPin);
  
  if (frontState == HIGH) {
    // Reflectance sensor is triggered:
    // 1. Back up for a short duration.
    // 2. Stop.
    // 3. Turn right (by running left motor forward and right motor backward briefly).
    Serial.println("Front sensor triggered. Backing up and turning right.");
    
    // Back up: run both motors in reverse
    motorABackward(motorSpeed);
    motorBBackward(motorSpeed);
    delay(500);  // Adjust backup time as needed
    stopMotors();
    delay(200);  // Brief pause
    
    // Turn right: left motor forward and right motor backward
    motorAForward(motorSpeed);
    motorBBackward(motorSpeed);
    delay(300);  // Adjust turning duration to suit your geometry
    stopMotors();
    delay(200);
  } else {
    // ----- WALL-FOLLOWING USING THE LEFT TOF SENSOR -----
    // Read the distance (in millimeters) from the VL53L0X sensor
    uint16_t leftDistance = tofSensor.readRangeSingleMillimeters();
    Serial.print("Left Distance: ");
    Serial.print(leftDistance);
    Serial.println(" mm");
    
    // Adjust drive based on wall distance:
    // - If too close (< 60 mm): steer right (reduce left motor speed).
    // - If too far (> 100 mm): steer left (reduce right motor speed).
    // - Otherwise, drive straight.
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
  }
  
  // Short delay for sensor stability
  delay(50);
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
