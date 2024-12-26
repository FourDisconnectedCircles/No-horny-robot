#include <Arduino.h>
#include <Servo.h>
#include <SimpleKalmanFilter.h>

// Define servo objects
Servo bodyServo;
Servo handServo1;
Servo handServo2;

// Define pins
const int bodyServoPin = 4;
const int handServo1Pin = 2;
const int handServo2Pin = 3;
const int relayPin = 5;

// Kalman filter for smoothing target angle
SimpleKalmanFilter angleKalmanFilter(1, 2, 0.01); // Process noise, measurement noise, and estimated error

// PID controller variables
float kp = 0.8; // Proportional gain
float ki = 0.2; // Integral gain
float kd = 0.0; // Derivative gain
float previousError = 0;
float integral = 0;
unsigned long previousTime = 0;

// Servo positions
int currentBodyAngle = 90;  // Start at center
int targetBodyAngle = 90;   // Initial target angle
int smoothedTargetAngle = 90; // Filtered target angle

// Deadband tolerance (angles within this range won't trigger movement)
const int deadband = 4; // Adjust this value to control sensitivity

void moveServoFast(Servo &servo, int currentPos, int targetPos, int step = 10, int delayTime = 5) {
  if (currentPos < targetPos) {
    for (int pos = currentPos; pos <= targetPos; pos += step) {
      servo.write(pos);
      delay(delayTime);
    }
  } else {
    for (int pos = currentPos; pos >= targetPos; pos -= step) {
      servo.write(pos);
      delay(delayTime);
    }
  }
  servo.write(targetPos); // Ensure it reaches the exact target position
}

void setup() {
  // Attach servos to pins
  bodyServo.attach(bodyServoPin);
  handServo1.attach(handServo1Pin);
  handServo2.attach(handServo2Pin);

  // Initialize servos to default positions
  bodyServo.write(currentBodyAngle); // Center position for body tracking
  handServo1.write(0);               // Default position for Servo1
  handServo2.write(180);             // Default position for Servo2

  // Initialize relay pin
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, LOW);

  // Begin serial communication
  Serial.begin(9600);
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); // Read command from Serial

    if (command.startsWith("B")) {
      // Body tracking command: B[angle]
      int angle = command.substring(1).toInt();
      if (angle >= 0 && angle <= 180) {
        targetBodyAngle = angle;
      }
    } else if (command == "H1") {
      // Hand detected: Activate additional servos and relay
      int currentPos1 = handServo1.read();
      int currentPos2 = handServo2.read();
      moveServoFast(handServo1, currentPos1, 90); // Move Servo1 to 90
      moveServoFast(handServo2, currentPos2, 90); // Move Servo2 to 90
      digitalWrite(relayPin, HIGH);              // Turn on relay
    } else if (command == "H0") {
      // No hand detected: Reset additional servos and relay
      int currentPos1 = handServo1.read();
      int currentPos2 = handServo2.read();
      moveServoFast(handServo1, currentPos1, 0); // Move Servo1 to default
      moveServoFast(handServo2, currentPos2, 180);   // Move Servo2 to default
      digitalWrite(relayPin, LOW);                // Turn off relay
    }
  }

  // Smooth the targetBodyAngle using the Kalman filter
  smoothedTargetAngle = angleKalmanFilter.updateEstimate(targetBodyAngle);

  // Update the body tracking servo position using the PID controller
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
  if (deltaTime > 0.01) { // Update at ~100 Hz

    // Only move if the difference between the current and target angles exceeds the deadband
    float error = smoothedTargetAngle - currentBodyAngle;
    if (abs(error) > deadband) {
      integral += error * deltaTime;
      float derivative = (error - previousError) / deltaTime;

      // Calculate PID output
      float pidOutput = (kp * error) + (ki * integral) + (kd * derivative);

      // Update the currentBodyAngle smoothly
      currentBodyAngle += pidOutput;
      if (currentBodyAngle < 0) currentBodyAngle = 0;
      if (currentBodyAngle > 180) currentBodyAngle = 180;

      // Move the servo to the new position
      bodyServo.write(currentBodyAngle);

      // Update PID variables
      previousError = error;
      previousTime = currentTime;
    }
  }
}
