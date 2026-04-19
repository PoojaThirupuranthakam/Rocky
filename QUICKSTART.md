# Cave Explorer Robot - Quick Reference

## Installation (5 minutes)

```bash
cd cave-robot
chmod +x setup.sh
./setup.sh
source venv/bin/activate
```

## Run Simulation

```bash
python src/run_simulation.py
```

Optional explicit sensor layout (default is already `dual_arc_180`):

```bash
python src/run_simulation.py --sensor-layout lidar_top_ground_ultrasonic
```

Legacy mode (two 180° ultrasonic-style arcs):

```bash
python src/run_simulation.py --sensor-layout dual_arc_180
```

## Keyboard Controls

| Key | Action |
|-----|--------|
| W | Move Forward |
| S | Move Backward |
| A | Turn Left |
| D | Turn Right |
| SPACE | Toggle AI Mode |
| H | Toggle Debug Info |
| R | Reset Simulation |
| Q/ESC | Quit |

## Deploy to Hardware

```bash
python src/deploy_to_robot.py
```

## Hardware Requirements (Real Robot)

### Minimum Setup:
- **Microcontroller**: Arduino Uno/Nano or ESP32
- **Motors**: 2x DC motors with wheels
- **Motor Driver**: L298N or L293D
- **Sensors**: Top 2D LiDAR + 1x ground-facing ultrasonic
- **Power**: 7.4V LiPo battery or 6x AA batteries
- **Optional**: Raspberry Pi for onboard AI processing

### Wiring Guide:
```
Arduino Uno Connections:
- Motor Left Forward: Pin 5 (PWM)
- Motor Left Backward: Pin 6 (PWM)
- Motor Right Forward: Pin 9 (PWM)
- Motor Right Backward: Pin 10 (PWM)
- Ground Ultrasonic TRIG/ECHO: configurable digital pins
- Top LiDAR: serial/UART or USB depending on module
- Power: VIN (7-12V)
- Ground: GND (common)
```

## Troubleshooting

### Simulation won't start
```bash
# Install pygame separately
pip install pygame
```

### Import errors
```bash
# Reinstall dependencies
pip install -r requirements.txt
```

### Robot not detected
```bash
# List serial ports (macOS/Linux)
ls /dev/tty.*

# List serial ports (Windows)
# Use Device Manager
```

### Slow performance
- Reduce FPS in `run_simulation.py` (line 23): `self.fps = 30`
- Reduce cave complexity in `CaveEnvironment` initialization (for custom test runs)

## Tips for Hackathon

1. **Test in simulation first** - Save time debugging
2. **Start with manual mode** - Understand the environment
3. **Use AI mode for complex paths** - Let the AI navigate
4. **Monitor sensor readings** - Press H to show debug info
5. **Battery matters** - Bring extra batteries for real robot

## Next Steps

1. ✅ Run simulation
2. ✅ Test manual control
3. ✅ Enable AI mode
4. ✅ Upload Arduino code to robot
5. ✅ Deploy AI model
6. ✅ Test in real cave!

Good luck! 🚀
