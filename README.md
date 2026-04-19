# Cave Explorer Robot - Hackathon Project

## Quick Start Guide

This project simulates an AI-powered robot navigating through a virtual cave environment using embedded AI for offline navigation.

## System Requirements
- macOS (Apple Silicon or Intel)
- Python 3.8+
- 8GB RAM minimum
- 10GB free disk space

## Installation Steps

### 1. Install Dependencies
```bash
cd cave-robot
chmod +x setup.sh
./setup.sh
```

### 2. Activate Environment
```bash
source venv/bin/activate
```

### 3. Run Simulation
```bash
python src/run_simulation.py
```

Default run uses the hybrid sensing stack:

- Top-mounted 2D LiDAR (180° frontal scan)
- Ground-facing ultrasonic (near-ground obstacle guard)

To run explicitly with this layout:

```bash
python src/run_simulation.py --sensor-layout lidar_top_ground_ultrasonic
```

To run with the legacy two-ultrasonic 180° pair (top + floor):

```bash
python src/run_simulation.py --sensor-layout dual_arc_180
```

To simulate a smaller robot footprint for tighter passages:

```bash
python src/run_simulation.py --sensor-layout dual_arc_180 --robot-radius 10
```

To start with a faster or slower robot speed:

```bash
python src/run_simulation.py --robot-speed-scale 1.4
```

Default simulator robot radius is `13` pixels (derived from 7x5x5 in robot profile at 0.5 cm/px).

Choose navigator backend:

Default run now uses `model` mode.

```bash
python src/run_simulation.py --navigator planner
```

Use model-based navigator (loads `models/navigation_model.pkl` or trains if missing):

```bash
python src/run_simulation.py --navigator model
```

Force retraining of model before run:

```bash
python src/run_simulation.py --navigator model --retrain-model
```

### Reinforcement Learning (Q-learning)

Train a tabular RL policy:

```bash
python src/train_rl.py --episodes 300 --model-path models/rl_q_table.pkl
```

Train with planner-imitation warm-start + RL fine-tuning (recommended):

```bash
python src/train_rl.py --warmstart-episodes 120 --episodes 320 --model-path models/rl_q_table.pkl
```

Evaluate the trained RL policy:

```bash
python src/eval_rl.py --model-path models/rl_q_table.pkl --episodes 40
```

Run simulator with the RL navigator:

```bash
python src/run_simulation.py --navigator rl --rl-model-path models/rl_q_table.pkl
```

Run simulator with RL online learning updates enabled (model updates while sim runs):

```bash
python src/run_simulation.py --navigator rl --rl-model-path models/rl_q_table.pkl --rl-online-update --rl-autosave-interval 200
```

By default, simulation in RL mode is inference-only unless `--rl-online-update` is provided.

Train model explicitly and print training metrics:

```bash
python src/train_model.py --retrain
```

### 4. Deploy to Real Robot
```bash
python src/deploy_to_robot.py
```

Rubik Pi-targeted deploy flow:

```bash
python src/deploy_to_robot.py --hardware-target rubik-pi
```

Generate deployment artifacts for both workflows (`.ino` + no-`.ino` native runtime):

```bash
python src/deploy_to_robo.py --mode both
```

Generate only native no-`.ino` runtime:

```bash
python src/deploy_to_robo.py --mode native
```

### Sensor Mode Configuration (Analog or Ultrasonic)

You can now choose sensor mode at startup and switch later from the deployment menu.

```bash
python src/deploy_to_robot.py --sensor-mode ultrasonic
```

Or fallback to analog:

```bash
python src/deploy_to_robot.py --sensor-mode analog
```

For your **Rubik Pi** setup with **2 ultrasonic sensors (HC-SR04/JSN-SR04T)**, generated `rubik_pi_firmware.ino` is configured for:

- Ground-facing ultrasonic sensor:
    - `GROUND_TRIG_PIN = 2`
    - `GROUND_ECHO_PIN = 3`
- Top ultrasonic sensor (180° scan via optional servo):
    - `TOP_TRIG_PIN = 4`
    - `TOP_ECHO_PIN = 7`
    - `TOP_SERVO_PIN = 8`

Analog fallback pins remain available:

- `SENSOR_FRONT = A0`
- `SENSOR_LEFT = A1`
- `SENSOR_RIGHT = A2`

You can also switch runtime mode over serial command:

- `SET_SENSOR_MODE:ULTRASONIC`
- `SET_SENSOR_MODE:ANALOG`

## Project Structure
```
cave-robot/
├── setup.sh              # Installation script
├── requirements.txt      # Python dependencies
├── src/
│   ├── cave_environment.py   # Virtual cave simulator
│   ├── robot_controller.py   # Robot control logic
│   ├── ai_navigator.py       # Embedded AI navigation
│   ├── sensor_simulator.py   # Simulated sensors
│   ├── run_simulation.py     # Main simulation runner
│   └── deploy_to_robot.py    # Real robot deployment
├── models/
│   └── navigation_model.tflite  # Embedded AI model
├── worlds/
│   └── cave_world.py         # Cave environment definition
└── config/
    ├── robot_config.yaml     # Robot configuration
    └── simulation_config.yaml # Simulation settings
```

## Controls
- **W**: Move forward
- **S**: Move backward
- **A**: Turn left
- **D**: Turn right
- **[** / **]**: Decrease / increase robot speed scale during simulation
- **Space**: Toggle AI navigation
- **Q**: Quit

## Key Features
1. ✅ Offline AI navigation (no internet needed)
2. ✅ Realistic cave physics simulation
3. ✅ Collision detection
4. ✅ Hybrid sensing: top 2D LiDAR + ground ultrasonic (with legacy modes)
5. ✅ Easy deployment to real hardware

## Hardware Compatibility
Designed to work with:
- Raspberry Pi 4 + Arduino
- ESP32-based robots
- Any robot with serial/GPIO interface

## Troubleshooting
See TROUBLESHOOTING.md for common issues and solutions.

### Model retrains after sensor changes

This is expected when sensor feature schema changes. The model loader validates
feature dimensions/schema metadata and automatically retrains if incompatible.
