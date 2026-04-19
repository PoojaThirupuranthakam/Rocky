#!/usr/bin/env python3
"""
Rubik Pi Native Runtime (NO .ino)

This runtime is intended to run directly on Rubik Pi. It supports:
1. Analog fallback sensors
2. Ultrasonic fallback sensors
3. USB serial LiDAR input from a device connected to Rubik Pi

For real hardware, replace the motor and optional GPIO sensor stubs with
board-specific control code.
"""

from __future__ import annotations

import argparse
import math
import struct
import time
from dataclasses import dataclass

import serial

try:
    import gpiod  # type: ignore
except ImportError:
    gpiod = None

DEFAULT_SENSOR_MODE = "lidar_usb"
DEFAULT_LIDAR_PORT = "/dev/ttyUSB0"
DEFAULT_LIDAR_BAUDRATE = 230400
LIDAR_STALE_TIMEOUT_S = 1.0
MAIN_LOOP_DELAY_S = 0.05
LD19_PACKET_LENGTH = 47
LD19_HEADER = 0x54
LD19_VERLEN = 0x2C
LD19_POINTS_PER_PACKET = 12
LD19_FULL_SCAN_DEGREES = 360
LIDAR_SECTOR_HALF_WIDTH_DEG = 20

# Adjust these if the sensor is mounted with a different forward heading.
LIDAR_FRONT_CENTER_DEG = 0
LIDAR_RIGHT_CENTER_DEG = 90
LIDAR_LEFT_CENTER_DEG = 270

# ----------------------------------------------------------------------------
# Hardware mapping (EDIT for your board/wiring)
# ----------------------------------------------------------------------------
GPIO_CHIP = "/dev/gpiochip0"

# 4x 28BYJ-48 stepper motors driven by 4x ULN2003 boards.
# Replace these line numbers with the actual GPIO lines used on Rubik Pi.
FRONT_LEFT_MOTOR_PINS = [5, 8, 10, 11]
REAR_LEFT_MOTOR_PINS = [12, 13, 15, 16]
FRONT_RIGHT_MOTOR_PINS = [18, 19, 21, 22]
REAR_RIGHT_MOTOR_PINS = [23, 24, 26, 27]

HALF_STEP_SEQUENCE = [
    (1, 0, 0, 0),
    (1, 1, 0, 0),
    (0, 1, 0, 0),
    (0, 1, 1, 0),
    (0, 0, 1, 0),
    (0, 0, 1, 1),
    (0, 0, 0, 1),
    (1, 0, 0, 1),
]
STEP_DELAY_S = 0.0015
MOVE_STEPS_PER_ACTION = 24
TURN_STEPS_PER_ACTION = 18

GROUND_TRIG_PIN = 2
GROUND_ECHO_PIN = 3
TOP_TRIG_PIN = 4
TOP_ECHO_PIN = 7

ANALOG_FRONT_PIN = 0
ANALOG_LEFT_PIN = 1
ANALOG_RIGHT_PIN = 2

TOP_FRONT_STOP_CM = 25.0
GROUND_MIN_SAFE_CM = 5.0
LIDAR_FRONT_STOP_CM = 35.0


@dataclass
class LidarSnapshot:
    front_cm: float
    left_cm: float
    right_cm: float
    min_cm: float
    mean_cm: float
    std_cm: float
    scan_cm: list[float]
    timestamp: float


class StepperMotor28BYJ:
    def __init__(self, name: str, pins: list[int], dry_run: bool, chip_path: str):
        if len(pins) != 4:
            raise ValueError(f"{name} requires exactly 4 GPIO pins")
        self.name = name
        self.pins = list(pins)
        self.dry_run = dry_run
        self.chip_path = chip_path
        self._phase_index = 0
        self._chip = None
        self._lines = None

    def setup(self) -> None:
        if self.dry_run or gpiod is None:
            return

        chip = gpiod.Chip(self.chip_path)
        lines = [chip.get_line(pin) for pin in self.pins]
        for line in lines:
            line.request(consumer=self.name, type=gpiod.LINE_REQ_DIR_OUT, default_vals=[0])
        self._chip = chip
        self._lines = lines
        self.release()

    def step(self, direction: int, steps: int, delay_s: float) -> None:
        if steps <= 0:
            return
        for _ in range(steps):
            self._phase_index = (self._phase_index + direction) % len(HALF_STEP_SEQUENCE)
            self._write_phase(HALF_STEP_SEQUENCE[self._phase_index])
            time.sleep(delay_s)

    def release(self) -> None:
        self._write_phase((0, 0, 0, 0))

    def close(self) -> None:
        self.release()
        if self._lines:
            for line in self._lines:
                try:
                    line.release()
                except Exception:
                    pass
        self._lines = None
        self._chip = None

    def _write_phase(self, phase: tuple[int, int, int, int]) -> None:
        if self.dry_run or self._lines is None:
            return
        for line, value in zip(self._lines, phase):
            line.set_value(int(value))


class UsbLidarReader:
    """
    Reads raw LD19 packets from a USB serial port on Rubik Pi.

    The LD19 streams one-way UART data continuously at 230400 baud once it has
    spun up. The packet structure used below follows the common LD19 layout:
    0x54 header, 0x2C verlen, 12 points per packet, little-endian fields.
    """

    def __init__(self, port: str, baudrate: int, timeout: float = 0.05):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.connection: serial.Serial | None = None
        self.latest_snapshot: LidarSnapshot | None = None
        self.scan_bins_mm: list[float] = [0.0] * LD19_FULL_SCAN_DEGREES

    def connect(self) -> None:
        self.connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        print(f"[LiDAR] Connected on {self.port} @ {self.baudrate}")

    def close(self) -> None:
        if self.connection is not None:
            self.connection.close()
            self.connection = None

    def read_latest(self) -> LidarSnapshot | None:
        if self.connection is None:
            self.connect()

        assert self.connection is not None
        packets_read = 0
        while self.connection.in_waiting >= 2:
            packet = self._read_packet()
            if packet is None:
                break
            snapshot = self._parse_packet(packet)
            if snapshot is not None:
                self.latest_snapshot = snapshot
                packets_read += 1

        if self.latest_snapshot is None:
            return None

        age = time.time() - self.latest_snapshot.timestamp
        if age > LIDAR_STALE_TIMEOUT_S:
            return None
        return self.latest_snapshot

    def _read_packet(self) -> bytes | None:
        assert self.connection is not None

        while True:
            first = self.connection.read(1)
            if not first:
                return None
            if first[0] != LD19_HEADER:
                continue

            second = self.connection.read(1)
            if not second:
                return None
            if second[0] != LD19_VERLEN:
                continue

            remainder = self.connection.read(LD19_PACKET_LENGTH - 2)
            if len(remainder) != LD19_PACKET_LENGTH - 2:
                return None
            return first + second + remainder

    def _parse_packet(self, packet: bytes) -> LidarSnapshot | None:
        if len(packet) != LD19_PACKET_LENGTH:
            return None

        try:
            _, verlen, speed_dps, start_angle_raw = struct.unpack_from("<BBHH", packet, 0)
        except struct.error:
            return None

        if verlen != LD19_VERLEN:
            return None

        start_angle_deg = start_angle_raw / 100.0
        distances_mm: list[float] = []
        for index in range(LD19_POINTS_PER_PACKET):
            offset = 6 + index * 3
            try:
                distance_mm, intensity = struct.unpack_from("<HB", packet, offset)
            except struct.error:
                return None
            _ = intensity
            distances_mm.append(float(distance_mm))

        try:
            end_angle_raw, timestamp_ms, crc = struct.unpack_from("<HHB", packet, 42)
        except struct.error:
            return None

        _ = speed_dps
        _ = timestamp_ms
        _ = crc
        end_angle_deg = end_angle_raw / 100.0
        if end_angle_deg < start_angle_deg:
            end_angle_deg += 360.0

        if LD19_POINTS_PER_PACKET == 1:
            step_deg = 0.0
        else:
            step_deg = (end_angle_deg - start_angle_deg) / (LD19_POINTS_PER_PACKET - 1)

        for index, distance_mm in enumerate(distances_mm):
            angle_deg = (start_angle_deg + index * step_deg) % 360.0
            bin_index = int(round(angle_deg)) % LD19_FULL_SCAN_DEGREES
            self.scan_bins_mm[bin_index] = max(0.0, distance_mm)

        return self._snapshot_from_bins()

    def _snapshot_from_bins(self) -> LidarSnapshot | None:
        valid_mm = [value for value in self.scan_bins_mm if value > 0.0]
        if not valid_mm:
            return None

        front_vals = self._sector_values(LIDAR_FRONT_CENTER_DEG)
        left_vals = self._sector_values(LIDAR_LEFT_CENTER_DEG)
        right_vals = self._sector_values(LIDAR_RIGHT_CENTER_DEG)

        scan_cm = [value / 10.0 if value > 0.0 else 0.0 for value in self.scan_bins_mm]
        valid_cm = [value / 10.0 for value in valid_mm]
        mean_cm = sum(valid_cm) / len(valid_cm)
        variance = sum((value - mean_cm) ** 2 for value in valid_cm) / len(valid_cm)
        return LidarSnapshot(
            front_cm=min(front_vals) / 10.0,
            left_cm=min(left_vals) / 10.0,
            right_cm=min(right_vals) / 10.0,
            min_cm=min(valid_cm),
            mean_cm=mean_cm,
            std_cm=math.sqrt(variance),
            scan_cm=scan_cm,
            timestamp=time.time(),
        )

    def _sector_values(self, center_deg: int) -> list[float]:
        values = []
        for offset in range(-LIDAR_SECTOR_HALF_WIDTH_DEG, LIDAR_SECTOR_HALF_WIDTH_DEG + 1):
            angle = (center_deg + offset) % LD19_FULL_SCAN_DEGREES
            reading = self.scan_bins_mm[angle]
            if reading > 0.0:
                values.append(reading)
        if not values:
            values = [12000.0]
        return values


class RubikPiRobot:
    def __init__(
        self,
        sensor_mode=DEFAULT_SENSOR_MODE,
        dry_run=True,
        lidar_port=DEFAULT_LIDAR_PORT,
        lidar_baudrate=DEFAULT_LIDAR_BAUDRATE,
    ):
        self.sensor_mode = sensor_mode
        self.dry_run = dry_run
        self.lidar = None
        self.motors = {
            "front_left": StepperMotor28BYJ("front_left", FRONT_LEFT_MOTOR_PINS, dry_run, GPIO_CHIP),
            "rear_left": StepperMotor28BYJ("rear_left", REAR_LEFT_MOTOR_PINS, dry_run, GPIO_CHIP),
            "front_right": StepperMotor28BYJ("front_right", FRONT_RIGHT_MOTOR_PINS, dry_run, GPIO_CHIP),
            "rear_right": StepperMotor28BYJ("rear_right", REAR_RIGHT_MOTOR_PINS, dry_run, GPIO_CHIP),
        }
        for motor in self.motors.values():
            motor.setup()
        if self.sensor_mode == "lidar_usb":
            self.lidar = UsbLidarReader(port=lidar_port, baudrate=lidar_baudrate)

    # ---------------------------
    # Motor control for 4x 28BYJ-48 + ULN2003
    # ---------------------------
    def move_forward(self):
        self._drive(left_direction=1, right_direction=-1, steps=MOVE_STEPS_PER_ACTION, label="FORWARD")

    def move_backward(self):
        self._drive(left_direction=-1, right_direction=1, steps=MOVE_STEPS_PER_ACTION, label="BACKWARD")

    def turn_left(self):
        self._drive(left_direction=-1, right_direction=-1, steps=TURN_STEPS_PER_ACTION, label="LEFT")

    def turn_right(self):
        self._drive(left_direction=1, right_direction=1, steps=TURN_STEPS_PER_ACTION, label="RIGHT")

    def stop(self):
        for motor in self.motors.values():
            motor.release()
        self._log("STOP")

    # ---------------------------
    # Sensor stubs
    # ---------------------------
    def read_ultrasonic_cm(self, trig_pin, echo_pin):
        _ = (trig_pin, echo_pin)
        return 100.0

    def read_analog(self, channel):
        _ = channel
        return 500.0

    def read_sensors(self):
        if self.sensor_mode == "analog":
            return {
                "mode": "analog",
                "front": self.read_analog(ANALOG_FRONT_PIN),
                "left": self.read_analog(ANALOG_LEFT_PIN),
                "right": self.read_analog(ANALOG_RIGHT_PIN),
            }

        if self.sensor_mode == "ultrasonic":
            return {
                "mode": "ultrasonic",
                "ground_cm": self.read_ultrasonic_cm(GROUND_TRIG_PIN, GROUND_ECHO_PIN),
                "top_front_cm": self.read_ultrasonic_cm(TOP_TRIG_PIN, TOP_ECHO_PIN),
                "top_left_cm": 100.0,
                "top_right_cm": 100.0,
            }

        if self.lidar is None:
            raise RuntimeError("LiDAR mode selected but no LiDAR reader is configured")

        snapshot = self.lidar.read_latest()
        if snapshot is None:
            return {
                "mode": "lidar_usb",
                "connected": False,
                "front_cm": 0.0,
                "left_cm": 0.0,
                "right_cm": 0.0,
                "min_cm": 0.0,
                "mean_cm": 0.0,
                "std_cm": 0.0,
                "scan_cm": [],
            }

        return {
            "mode": "lidar_usb",
            "connected": True,
            "front_cm": snapshot.front_cm,
            "left_cm": snapshot.left_cm,
            "right_cm": snapshot.right_cm,
            "min_cm": snapshot.min_cm,
            "mean_cm": snapshot.mean_cm,
            "std_cm": snapshot.std_cm,
            "scan_cm": snapshot.scan_cm,
        }

    def autonomous_step(self):
        sensors = self.read_sensors()

        if sensors["mode"] == "analog":
            obstacle_ahead = sensors["front"] < 300.0
            prefer_left = sensors["left"] > sensors["right"]
        elif sensors["mode"] == "ultrasonic":
            if sensors["ground_cm"] < GROUND_MIN_SAFE_CM:
                self.move_backward()
                time.sleep(0.12)
                self.turn_left()
                return
            obstacle_ahead = sensors["top_front_cm"] < TOP_FRONT_STOP_CM
            prefer_left = sensors["top_left_cm"] > sensors["top_right_cm"]
        else:
            if not sensors["connected"]:
                self.stop()
                self._log("WAITING_FOR_LIDAR")
                return
            obstacle_ahead = sensors["front_cm"] < LIDAR_FRONT_STOP_CM
            prefer_left = sensors["left_cm"] >= sensors["right_cm"]

        if obstacle_ahead:
            self.turn_left() if prefer_left else self.turn_right()
        else:
            self.move_forward()

    def close(self):
        if self.lidar is not None:
            self.lidar.close()
        for motor in self.motors.values():
            motor.close()

    def _log(self, action):
        prefix = "[DRY-RUN] " if self.dry_run else "[TODO-HW] "
        print(f"{prefix}{action}")

    def _drive(self, left_direction: int, right_direction: int, steps: int, label: str) -> None:
        self._log(label)
        self.motors["front_left"].step(left_direction, steps, STEP_DELAY_S)
        self.motors["rear_left"].step(left_direction, steps, STEP_DELAY_S)
        self.motors["front_right"].step(right_direction, steps, STEP_DELAY_S)
        self.motors["rear_right"].step(right_direction, steps, STEP_DELAY_S)


def parse_args():
    parser = argparse.ArgumentParser(description="Run cave robot natively on Rubik Pi")
    parser.add_argument(
        "--sensor-mode",
        choices=["analog", "ultrasonic", "lidar_usb"],
        default=DEFAULT_SENSOR_MODE,
        help="Sensor mode for native runtime",
    )
    parser.add_argument(
        "--lidar-port",
        default=DEFAULT_LIDAR_PORT,
        help="USB serial port for LiDAR adapter data",
    )
    parser.add_argument(
        "--lidar-baudrate",
        type=int,
        default=DEFAULT_LIDAR_BAUDRATE,
        help="Serial baud rate for LiDAR adapter data",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Disable dry-run logging once motor GPIO code is implemented",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    robot = RubikPiRobot(
        sensor_mode=args.sensor_mode,
        dry_run=not args.live,
        lidar_port=args.lidar_port,
        lidar_baudrate=args.lidar_baudrate,
    )
    print("Rubik Pi native runtime started")
    print(f"Sensor mode: {robot.sensor_mode}")
    if robot.sensor_mode == "lidar_usb":
        print(f"LiDAR port: {args.lidar_port} @ {args.lidar_baudrate}")
    print("Press Ctrl+C to stop")

    try:
        while True:
            robot.autonomous_step()
            time.sleep(MAIN_LOOP_DELAY_S)
    except KeyboardInterrupt:
        robot.stop()
        print("\nStopped")
    finally:
        robot.close()


if __name__ == "__main__":
    main()
