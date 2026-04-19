
/*
 * Cave Explorer Robot - Rubik Pi Controller Firmware
 *
 * Supported sensor modes:
 *  1) ANALOG (fallback): front/left/right analog sensors
 *  2) ULTRASONIC: two HC-SR04/JSN-SR04T sensors
 *     - Ground sensor points downward
 *     - Top sensor can scan 180° via optional servo
 *
 * Runtime serial commands:
 *   SET_SENSOR_MODE:ANALOG
 *   SET_SENSOR_MODE:ULTRASONIC
 *   GET_SENSOR_MODE
 */

#include <Arduino.h>
#include <Servo.h>

// -----------------------
// Motor pins
// -----------------------
#define MOTOR_LEFT_FWD 5
#define MOTOR_LEFT_BWD 6
#define MOTOR_RIGHT_FWD 9
#define MOTOR_RIGHT_BWD 10

// -----------------------
// Analog sensor pins
// -----------------------
#define SENSOR_FRONT A0
#define SENSOR_LEFT  A1
#define SENSOR_RIGHT A2

// -----------------------
// Ultrasonic sensor pins
// -----------------------
// Ground-facing ultrasonic
#define GROUND_TRIG_PIN 2
#define GROUND_ECHO_PIN 3

// Top-facing ultrasonic (optionally mounted on servo)
#define TOP_TRIG_PIN 4
#define TOP_ECHO_PIN 7

// Optional servo for top sensor sweep
#define USE_TOP_SERVO 1
#define TOP_SERVO_PIN 8
#define TOP_LEFT_ANGLE 160
#define TOP_FRONT_ANGLE 90
#define TOP_RIGHT_ANGLE 20

// Thresholds (tune for your robot/environment)
#define ANALOG_FRONT_THRESHOLD 300
#define TOP_FRONT_STOP_CM 25
#define GROUND_MIN_SAFE_CM 5

bool autoMode = false;
String sensorMode = "ULTRASONIC"; // ANALOG or ULTRASONIC

long topLeftCm = 200;
long topFrontCm = 200;
long topRightCm = 200;
long groundCm = 200;

#if USE_TOP_SERVO
Servo topServo;
#endif

long readUltrasonicCm(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 25000UL);  // ~4m timeout
  if (duration <= 0) return 400;
  return (long)(duration * 0.0343 / 2.0);
}

void readAnalogSensors(int &frontRaw, int &leftRaw, int &rightRaw) {
  frontRaw = analogRead(SENSOR_FRONT);
  leftRaw = analogRead(SENSOR_LEFT);
  rightRaw = analogRead(SENSOR_RIGHT);
}

void readUltrasonicSensors() {
  groundCm = readUltrasonicCm(GROUND_TRIG_PIN, GROUND_ECHO_PIN);

#if USE_TOP_SERVO
  topServo.write(TOP_LEFT_ANGLE);
  delay(110);
  topLeftCm = readUltrasonicCm(TOP_TRIG_PIN, TOP_ECHO_PIN);

  topServo.write(TOP_FRONT_ANGLE);
  delay(110);
  topFrontCm = readUltrasonicCm(TOP_TRIG_PIN, TOP_ECHO_PIN);

  topServo.write(TOP_RIGHT_ANGLE);
  delay(110);
  topRightCm = readUltrasonicCm(TOP_TRIG_PIN, TOP_ECHO_PIN);
#else
  long d = readUltrasonicCm(TOP_TRIG_PIN, TOP_ECHO_PIN);
  topLeftCm = d;
  topFrontCm = d;
  topRightCm = d;
#endif
}

void setup() {
  Serial.begin(115200);

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  pinMode(SENSOR_FRONT, INPUT);
  pinMode(SENSOR_LEFT, INPUT);
  pinMode(SENSOR_RIGHT, INPUT);

  pinMode(GROUND_TRIG_PIN, OUTPUT);
  pinMode(GROUND_ECHO_PIN, INPUT);
  pinMode(TOP_TRIG_PIN, OUTPUT);
  pinMode(TOP_ECHO_PIN, INPUT);

#if USE_TOP_SERVO
  topServo.attach(TOP_SERVO_PIN);
  topServo.write(TOP_FRONT_ANGLE);
#endif

  Serial.println("Robot Ready!");
}

void moveForward() {
  analogWrite(MOTOR_LEFT_FWD, 200);
  analogWrite(MOTOR_RIGHT_FWD, 200);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void moveBackward() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, 200);
  analogWrite(MOTOR_RIGHT_BWD, 200);
}

void turnLeft() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 200);
  analogWrite(MOTOR_LEFT_BWD, 150);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void turnRight() {
  analogWrite(MOTOR_LEFT_FWD, 200);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, 150);
}

void stopMotors() {
  analogWrite(MOTOR_LEFT_FWD, 0);
  analogWrite(MOTOR_RIGHT_FWD, 0);
  analogWrite(MOTOR_LEFT_BWD, 0);
  analogWrite(MOTOR_RIGHT_BWD, 0);
}

void processCommand(String cmd) {
  cmd.trim();

  if (cmd == "FORWARD") {
    moveForward();
    Serial.println("OK");
  } else if (cmd == "BACKWARD") {
    moveBackward();
    Serial.println("OK");
  } else if (cmd == "LEFT") {
    turnLeft();
    Serial.println("OK");
  } else if (cmd == "RIGHT") {
    turnRight();
    Serial.println("OK");
  } else if (cmd == "STOP") {
    stopMotors();
    Serial.println("OK");
  } else if (cmd == "START_AUTO") {
    autoMode = true;
    Serial.println("Auto mode ON");
  } else if (cmd == "STOP_AUTO") {
    autoMode = false;
    stopMotors();
    Serial.println("Auto mode OFF");
  } else if (cmd == "CALIBRATE") {
    Serial.println("Calibrated");
  } else if (cmd == "GET_SENSOR_MODE") {
    Serial.print("MODE:");
    Serial.println(sensorMode);
  } else if (cmd.startsWith("SET_SENSOR_MODE:")) {
    String mode = cmd.substring(String("SET_SENSOR_MODE:").length());
    mode.trim();
    mode.toUpperCase();
    if (mode == "ANALOG" || mode == "ULTRASONIC") {
      sensorMode = mode;
      Serial.print("MODE_SET:");
      Serial.println(sensorMode);
    } else {
      Serial.println("ERR:INVALID_MODE");
    }
  }
}

void autonomousNavigation() {
  bool obstacleAhead = false;
  bool preferLeft = false;

  if (sensorMode == "ANALOG") {
    int frontRaw = 0, leftRaw = 0, rightRaw = 0;
    readAnalogSensors(frontRaw, leftRaw, rightRaw);

    Serial.print("MODE=ANALOG F=");
    Serial.print(frontRaw);
    Serial.print(" L=");
    Serial.print(leftRaw);
    Serial.print(" R=");
    Serial.println(rightRaw);

    obstacleAhead = frontRaw < ANALOG_FRONT_THRESHOLD;
    preferLeft = leftRaw > rightRaw;
  } else {
    readUltrasonicSensors();

    Serial.print("MODE=ULTRASONIC G=");
    Serial.print(groundCm);
    Serial.print(" TL=");
    Serial.print(topLeftCm);
    Serial.print(" TF=");
    Serial.print(topFrontCm);
    Serial.print(" TR=");
    Serial.println(topRightCm);

    if (groundCm < GROUND_MIN_SAFE_CM) {
      moveBackward();
      delay(120);
      turnLeft();
      return;
    }

    obstacleAhead = topFrontCm < TOP_FRONT_STOP_CM;
    preferLeft = topLeftCm > topRightCm;
  }

  if (obstacleAhead) {
    if (preferLeft) {
      turnLeft();
    } else {
      turnRight();
    }
  } else {
    moveForward();
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\\n');
    processCommand(command);
  }

  if (autoMode) {
    autonomousNavigation();
  }

  delay(50);
}
