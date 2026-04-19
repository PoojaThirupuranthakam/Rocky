"""
Deploy to Real Robot
This script helps you deploy your tested AI to actual hardware.
"""

import argparse
import os
import time

import serial


class RobotDeployer:
    """Handles deployment to physical robot hardware."""

    def __init__(
        self,
        port="/dev/ttyUSB0",
        baudrate=115200,
        sensor_mode="ultrasonic",
        hardware_target="rubik-pi",
    ):
        """
        Initialize connection to robot.

        Common ports:
        - Raspberry Pi: /dev/ttyACM0 or /dev/ttyUSB0
        - macOS: /dev/cu.usbserial-* or /dev/cu.usbmodem*
        - Windows: COM3, COM4, etc.
        """
        self.port = port
        self.baudrate = baudrate
        self.sensor_mode = self._normalize_sensor_mode(sensor_mode)
        self.hardware_target = (hardware_target or "rubik-pi").strip().lower()
        self.connection = None

        print("🤖 Robot Deployment Tool")
        print("=" * 50)
        print(f"Hardware target: {self.hardware_target}")
        print(f"Sensor mode: {self.sensor_mode.upper()}")

    def _normalize_sensor_mode(self, sensor_mode):
        mode = (sensor_mode or "ultrasonic").strip().lower()
        return "analog" if mode == "analog" else "ultrasonic"

    def detect_port(self):
        """Auto-detect robot serial port."""
        import serial.tools.list_ports

        print("🔍 Scanning for connected robots...")
        ports = serial.tools.list_ports.comports()

        if not ports:
            print("❌ No serial devices found!")
            return None

        print("\nAvailable ports:")
        for i, port in enumerate(ports):
            print(f"  {i+1}. {port.device} - {port.description}")

        try:
            choice = int(input("\nSelect port number (or 0 to cancel): "))
            if 0 < choice <= len(ports):
                return ports[choice - 1].device
        except Exception:
            pass

        return None

    def connect(self):
        """Establish connection to robot."""
        if not self.port or self.port == "auto":
            self.port = self.detect_port()
            if not self.port:
                return False

        try:
            print(f"\n📡 Connecting to {self.port} at {self.baudrate} baud...")
            self.connection = serial.Serial(self.port, self.baudrate, timeout=1)
            time.sleep(2)
            print("✅ Connected!")

            # Best-effort configuration of sensor mode on firmware side.
            self.set_sensor_mode(self.sensor_mode, quiet=True)
            return True
        except serial.SerialException as e:
            print(f"❌ Connection failed: {e}")
            return False

    def send_command(self, command):
        """Send command to robot and return one-line response."""
        if not self.connection:
            print("❌ Not connected to robot!")
            return None

        try:
            self.connection.write(f"{command}\n".encode())
            response = self.connection.readline().decode(errors="ignore").strip()
            return response
        except Exception as e:
            print(f"❌ Command failed: {e}")
            return None

    def set_sensor_mode(self, mode, quiet=False):
        """Set sensor mode on device and locally: analog or ultrasonic."""
        normalized = self._normalize_sensor_mode(mode)
        response = self.send_command(f"SET_SENSOR_MODE:{normalized.upper()}")
        self.sensor_mode = normalized

        if not quiet:
            if response:
                print(f"✅ Sensor mode set to {normalized.upper()} (device: {response})")
            else:
                print(f"ℹ️ Sensor mode set locally to {normalized.upper()} (no ACK)")

    def upload_ai_model(self):
        """Upload trained AI model to robot (optional legacy path)."""
        model_path = "models/navigation_model.pkl"

        if not os.path.exists(model_path):
            print(f"❌ Model not found: {model_path}")
            print("   Run simulation/training if your firmware uses model upload.")
            return False

        print("\n📦 Uploading AI model...")
        print("   This may take a moment...")

        with open(model_path, "rb") as f:
            model_data = f.read()

        size = len(model_data)
        self.send_command(f"UPLOAD_MODEL:{size}")

        chunk_size = 64
        for i in range(0, size, chunk_size):
            chunk = model_data[i : i + chunk_size]
            self.connection.write(chunk)
            time.sleep(0.01)

            progress = min(i + chunk_size, size) / size * 100
            print(f"   Progress: {progress:.1f}%", end="\r")

        print("\n✅ AI model uploaded!")
        return True

    def calibrate_sensors(self):
        """Calibrate robot sensors."""
        print("\n🎯 Calibrating sensors...")
        print("   Place robot in open space and press Enter")
        input()

        self.send_command("CALIBRATE")
        time.sleep(2)
        print("✅ Sensor calibration command sent")

    def test_motors(self):
        """Test robot motors."""
        print("\n⚙️  Testing motors...")

        tests = [
            ("FORWARD", "Moving forward"),
            ("STOP", "Stopping"),
            ("BACKWARD", "Moving backward"),
            ("STOP", "Stopping"),
            ("LEFT", "Turning left"),
            ("STOP", "Stopping"),
            ("RIGHT", "Turning right"),
            ("STOP", "Stopping"),
        ]

        for command, description in tests:
            print(f"   {description}...")
            self.send_command(command)
            time.sleep(1)

        print("✅ Motor test complete!")

    def run_autonomous_mode(self):
        """Start autonomous navigation and stream telemetry."""
        print("\n🤖 Starting autonomous mode...")
        print("   Press Ctrl+C to stop\n")

        try:
            self.send_command("START_AUTO")
            while True:
                data = self.connection.readline().decode(errors="ignore").strip()
                if data:
                    print(f"   {data}")
                time.sleep(0.05)
        except KeyboardInterrupt:
            print("\n\n⏹️  Stopping autonomous mode...")
            self.send_command("STOP_AUTO")
            self.send_command("STOP")

    def disconnect(self):
        """Close connection to robot."""
        if self.connection:
            self.connection.close()
            print("\n👋 Disconnected from robot")


def build_rubik_pi_firmware(default_sensor_mode="ultrasonic"):
    """Generate firmware template for Rubik Pi serial motor-controller firmware."""
    default_mode = "ANALOG" if default_sensor_mode == "analog" else "ULTRASONIC"

    template = r'''
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
String sensorMode = "__DEFAULT_SENSOR_MODE__"; // ANALOG or ULTRASONIC

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
'''

    return template.replace("__DEFAULT_SENSOR_MODE__", default_mode)


def build_rp2040_firmware(default_sensor_mode="ultrasonic"):
    """Backward-compatible alias for older Pico W naming."""
    return build_rubik_pi_firmware(default_sensor_mode)


def parse_args():
    parser = argparse.ArgumentParser(description="Deploy/control cave robot over serial")
    parser.add_argument("--port", default="auto", help="Serial port (default: auto)")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate")
    parser.add_argument(
        "--hardware-target",
        choices=["rubik-pi", "pico-w", "generic"],
        default="rubik-pi",
        help="Target compute platform label for deployment messaging",
    )
    parser.add_argument(
        "--sensor-mode",
        choices=["analog", "ultrasonic"],
        default="ultrasonic",
        help="Default mode for both deployer and generated firmware",
    )
    return parser.parse_args()


def save_firmware(sensor_mode):
    firmware_code = build_rubik_pi_firmware(sensor_mode)
    with open("rubik_pi_firmware.ino", "w", encoding="utf-8") as f:
        f.write(firmware_code)
    print("📝 Rubik Pi firmware template saved to: rubik_pi_firmware.ino")


def main(args):
    deployer = RobotDeployer(
        port=args.port,
        baudrate=args.baudrate,
        sensor_mode=args.sensor_mode,
    hardware_target=args.hardware_target,
    )

    if not deployer.connect():
        print("\n💡 Tips:")
        print("   - Check USB cable connection")
        if args.hardware_target == "rubik-pi":
            print("   - Verify Rubik Pi is powered and serial peripheral/firmware is running")
        elif args.hardware_target == "pico-w":
            print("   - Verify Pico W is powered and has firmware flashed")
        else:
            print("   - Verify target controller is powered and has firmware flashed")
        print("   - Confirm correct serial port and board drivers")
        return

    while True:
        print("\n" + "=" * 50)
        print("DEPLOYMENT MENU")
        print("=" * 50)
        print("1. Upload AI Model")
        print("2. Calibrate Sensors")
        print("3. Test Motors")
        print("4. Run Autonomous Mode")
        print("5. Send Custom Command")
        print("6. Set Sensor Mode (analog/ultrasonic)")
        print("7. Exit")
        print()

        choice = input("Select option: ").strip()

        if choice == "1":
            deployer.upload_ai_model()
        elif choice == "2":
            deployer.calibrate_sensors()
        elif choice == "3":
            deployer.test_motors()
        elif choice == "4":
            deployer.run_autonomous_mode()
        elif choice == "5":
            cmd = input("Enter command: ")
            response = deployer.send_command(cmd)
            print(f"Response: {response}")
        elif choice == "6":
            mode = input("Enter sensor mode [analog/ultrasonic]: ").strip().lower()
            if mode in ("analog", "ultrasonic"):
                deployer.set_sensor_mode(mode)
            else:
                print("Invalid sensor mode!")
        elif choice == "7":
            break
        else:
            print("Invalid option!")

    deployer.disconnect()


if __name__ == "__main__":
    args = parse_args()
    save_firmware(args.sensor_mode)
    print()
    main(args)
