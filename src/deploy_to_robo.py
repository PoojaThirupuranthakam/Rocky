"""
Dual-path deployment helper for Rubik Pi projects.

Supports both:
1) .ino firmware generation (for external microcontroller flow)
2) Native Rubik Pi Python runtime generation with USB LiDAR support
"""

import argparse
from pathlib import Path

from deploy_to_robot import build_rubik_pi_firmware


def build_native_rubik_pi_runtime(default_sensor_mode: str = "ultrasonic") -> str:
    """Generate the native runtime from the checked-in Rubik Pi template."""
    sensor_mode = "analog" if default_sensor_mode == "analog" else "lidar_usb"
    template_path = Path(__file__).resolve().parent.parent / "rubik_pi_runtime.py"
    template = template_path.read_text(encoding="utf-8")
    return template.replace(
        'DEFAULT_SENSOR_MODE = "lidar_usb"',
        f'DEFAULT_SENSOR_MODE = "{sensor_mode}"',
        1,
    )


def write_file(path: Path, content: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate deployment artifacts for Rubik Pi: .ino and/or native Python runtime"
    )
    parser.add_argument(
        "--mode",
        choices=["ino", "native", "both"],
        default="both",
        help="Which deployment artifact(s) to generate",
    )
    parser.add_argument(
        "--sensor-mode",
        choices=["analog", "ultrasonic", "lidar_usb"],
        default="lidar_usb",
        help="Default mode embedded into generated artifacts",
    )
    parser.add_argument(
        "--ino-output",
        default="rubik_pi_firmware.ino",
        help="Output path for generated .ino firmware",
    )
    parser.add_argument(
        "--native-output",
        default="rubik_pi_runtime.py",
        help="Output path for generated native Rubik Pi runtime",
    )
    return parser.parse_args()


def main():
    args = parse_args()

    generated = []

    if args.mode in ("ino", "both"):
        ino_path = Path(args.ino_output)
        ino_code = build_rubik_pi_firmware("ultrasonic" if args.sensor_mode == "lidar_usb" else args.sensor_mode)
        write_file(ino_path, ino_code)
        generated.append(str(ino_path))
        print(f"Generated .ino firmware: {ino_path}")

    if args.mode in ("native", "both"):
        native_path = Path(args.native_output)
        native_code = build_native_rubik_pi_runtime(args.sensor_mode)
        write_file(native_path, native_code)
        generated.append(str(native_path))
        print(f"Generated native runtime: {native_path}")

    if generated:
        print("\nDone. Generated artifact(s):")
        for item in generated:
            print(f"  - {item}")


if __name__ == "__main__":
    main()
