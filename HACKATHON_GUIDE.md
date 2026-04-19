# 🤖 CAVE EXPLORER ROBOT - ONE-DAY HACKATHON GUIDE

## **Complete Step-by-Step Instructions**

---

## **PART 1: SOFTWARE INSTALLATION (30-45 minutes)**

### **Step 1: Install System Dependencies**

```bash
# Install Homebrew if you don't have it
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install SDL2 libraries (required for graphics)
brew install sdl2 sdl2_image sdl2_mixer sdl2_ttf
```

### **Step 2: Set Up Project**

```bash
# Navigate to project directory
cd /Users/surendrathirupuranthakam/java/pooja/agents/cave-robot

# Make setup script executable and run it
chmod +x setup.sh
./setup.sh

# Activate virtual environment
source venv/bin/activate

# Install pygame specifically
pip install pygame
```

**✅ Verification:** Run `python --version` and `pip list | grep pygame`

---

## **PART 2: TEST VIRTUAL SIMULATION (45-60 minutes)**

### **Step 3: Run the Simulation**

```bash
# From cave-robot directory
python src/run_simulation.py
```

**You should see:**
- A window opens showing a procedural cave environment
- Robot (blue circle) at the starting position (green circle)
- Goal location (gold circle) on the opposite side
- Debug information panel at the bottom

### **Step 4: Manual Testing**

**Controls:**
- **W** - Move forward
- **S** - Move backward  
- **A** - Turn left
- **D** - Turn right
- **H** - Toggle debug info
- **R** - Reset to start
- **Q/ESC** - Quit

**Practice for 10-15 minutes:**
1. Navigate toward the goal manually
2. Watch sensor beams (cyan lines) detect walls
3. Observe collision detection (robot turns red when hitting walls)
4. Note battery percentage decreasing

### **Step 5: Test AI Navigation**

1. Press **R** to reset to start position
2. Press **SPACE** to toggle AI mode ON
3. Watch the AI navigate autonomously
4. Observe:
   - AI confidence scores in debug panel
   - Obstacle avoidance behavior
   - Path-finding toward goal

**Expected Behavior:**
- Robot avoids obstacles automatically
- Turns when detecting walls ahead
- Gradually makes progress toward goal
- May not be perfect (it's learning-based!)

---

## **PART 3: HARDWARE SETUP (2-3 hours)**

### **Step 6: Hardware Components Needed**

**Minimum Robot Setup:**
```
✓ Arduino Uno or ESP32 ($10-25)
✓ L298N Motor Driver ($5)
✓ 2x DC Motors with wheels ($10)
✓ 3x HC-SR04 Ultrasonic Sensors ($3 each)
✓ Chassis (can be DIY cardboard)
✓ 7.4V LiPo battery or 6x AA battery pack ($10-15)
✓ Jumper wires
✓ USB cable for programming
```

**Optional Enhancements:**
```
○ Raspberry Pi 4 for onboard AI ($35)
○ Camera module for vision ($15)
○ LIDAR sensor for better mapping ($50)
○ LED lights for cave visibility
○ Battery voltage monitor
```

### **Step 7: Circuit Wiring**

**Arduino Connections:**

```
Motor Driver (L298N) → Arduino:
  IN1 → Pin 5
  IN2 → Pin 6
  IN3 → Pin 9
  IN4 → Pin 10
  VCC → 5V
  GND → GND
  +12V → Battery Positive
  GND → Battery Negative & Arduino GND

Front Sensor (HC-SR04) → Arduino:
  VCC → 5V
  TRIG → Pin A0
  ECHO → Pin A1
  GND → GND

Left Sensor → A2, A3
Right Sensor → A4, A5

Motors → Motor Driver:
  Left Motor → OUT1, OUT2
  Right Motor → OUT3, OUT4
```

**⚠️ IMPORTANT:** Common ground between Arduino, motor driver, and battery!

### **Step 8: Upload Arduino Firmware**

```bash
# Open Arduino IDE
# Copy firmware code
cd /Users/surendrathirupuranthakam/java/pooja/agents/cave-robot
cat src/deploy_to_robot.py

# Find the Arduino code at bottom (ARDUINO_CODE section)
# Or create new file:
nano robot_firmware.ino
```

**In Arduino IDE:**
1. Tools → Board → Arduino Uno (or your board)
2. Tools → Port → /dev/cu.usbserial-* (your port)
3. Upload the firmware
4. Open Serial Monitor (9600 baud)
5. You should see "Robot Ready!"

---

## **PART 4: DEPLOY AI TO ROBOT (1-2 hours)**

### **Step 9: Test Robot Communication**

```bash
# Activate virtual environment
source venv/bin/activate

# Run deployment tool
python src/deploy_to_robot.py
```

**Follow prompts:**
1. Select serial port (or auto-detect)
2. Connection should succeed
3. Main menu appears

### **Step 10: System Testing**

**From deployment menu:**

**A) Test Motors:**
```
Option 3: Test Motors
```
- Robot should move forward, backward, turn left, turn right
- If reversed, swap motor wires

**B) Calibrate Sensors:**
```
Option 2: Calibrate Sensors
```
- Place robot in open space
- Press Enter
- Wait for "Calibrated!" message

**C) Upload AI Model:**
```
Option 1: Upload AI Model
```
- Trained model uploads to Arduino
- Progress bar shows completion

### **Step 11: First Autonomous Run**

**In deployment menu:**
```
Option 4: Run Autonomous Mode
```

**Expected Behavior:**
- Robot reads sensors continuously
- Displays sensor values in terminal
- Avoids obstacles automatically
- Navigates forward when path is clear

**Press Ctrl+C to stop**

---

## **PART 5: CAVE TESTING (Final Phase)**

### **Step 12: Pre-Cave Checklist**

```
□ Battery fully charged
□ All sensors responding
□ Motors tested and working
□ AI model uploaded
□ Autonomous mode tested indoors
□ Backup battery available
□ Robot fits through narrow passages
□ LED lights attached (if dark cave)
```

### **Step 13: Cave Deployment**

**Since there's NO INTERNET in cave:**

1. **All AI runs locally** - The embedded model is on the Arduino
2. **No cloud connection needed** - Everything is offline
3. **Sensors work independently** - No GPS, pure dead reckoning

**Testing Protocol:**
1. Start at cave entrance
2. Place robot facing inward
3. Activate autonomous mode
4. Monitor from safe distance
5. Retrieve after test run

**Safety:**
- Tether robot with string for retrieval
- Test in short sections first
- Bring flashlight to monitor
- Have spare batteries

---

## **TROUBLESHOOTING**

### **Simulation Issues:**

**Problem:** pygame import error
```bash
brew install sdl2 sdl2_image sdl2_mixer sdl2_ttf
pip install pygame
```

**Problem:** Slow performance
- Edit `src/run_simulation.py`
- Change `self.fps = 60` to `self.fps = 30`
- Reduce cave complexity

### **Hardware Issues:**

**Problem:** Robot doesn't move
- Check battery voltage (should be >6V)
- Verify motor driver connections
- Test motors directly with battery

**Problem:** Sensors not reading
- Check 5V power supply
- Verify TRIG/ECHO connections
- Test individual sensors with examples

**Problem:** Robot spins in circles
- Motors may be reversed
- Swap one motor's wires
- Or fix in code (swap IN1/IN2)

**Problem:** Can't upload to Arduino
- Install CH340 drivers (for cheap clones)
- Try different USB cable
- Check board selection

---

## **OPTIMIZATION TIPS**

### **For Better Simulation:**
1. Increase AI training iterations in `ai_navigator.py`
2. Add more sensor readings (5-direction instead of 3)
3. Implement SLAM (mapping) algorithm
4. Add multiple goal checkpoints

### **For Better Robot:**
1. Add wheel encoders for odometry
2. Use LIDAR instead of ultrasonic
3. Add IMU for orientation
4. Implement PID control for smooth movement
5. Add camera for visual SLAM

---

## **TIME ALLOCATION (One-Day Hackathon)**

```
09:00 - 10:00  Software setup & installation
10:00 - 11:00  Test simulation & train AI
11:00 - 12:00  Hardware assembly
12:00 - 13:00  Lunch break
13:00 - 14:00  Wire electronics
14:00 - 15:00  Upload firmware & test
15:00 - 16:00  Deploy AI model
16:00 - 17:00  Indoor autonomous testing
17:00 - 18:00  Cave testing & refinement
```

---

## **DEMO SCRIPT (For Judges)**

**"Hi! This is my AI-powered cave explorer robot."**

1. **Show simulation:**
   - "I created this virtual cave environment for testing"
   - Toggle between manual and AI mode
   - Point out sensor visualization
   
2. **Explain AI:**
   - "The AI uses a simple neural network"
   - "Works 100% offline - no internet needed"
   - "Trained on obstacle avoidance patterns"
   
3. **Hardware demo:**
   - "Built with Arduino and basic sensors"
   - Power on and show autonomous mode
   - Demonstrate obstacle avoidance
   
4. **Cave readiness:**
   - "Designed for zero connectivity"
   - "All processing is embedded"
   - "Sensors work in dark environments"

---

## **KEY FEATURES TO HIGHLIGHT**

✅ **Offline AI** - No internet required
✅ **Realistic simulation** - Test before deploying
✅ **Embedded model** - Runs on Arduino
✅ **Simple hardware** - Under $100 total cost
✅ **Scalable** - Can add more sensors/features
✅ **Safe testing** - Simulation prevents hardware damage

---

## **NEXT STEPS AFTER HACKATHON**

1. Add camera-based navigation
2. Implement SLAM for mapping
3. Multi-robot coordination
4. Real-time 3D cave mapping
5. Autonomous sample collection
6. Emergency beacon system

---

## **RESOURCES & REFERENCES**

- **Simulation Code:** `/cave-robot/src/`
- **Arduino Firmware:** `robot_firmware.ino`
- **Hardware Guide:** `QUICKSTART.md`
- **Python Documentation:** Comments in source code

**Community:**
- Arduino Forums: https://forum.arduino.cc
- Robotics Stack Exchange: https://robotics.stackexchange.com
- GitHub Discussions: (your repo)

---

## **GOOD LUCK! 🚀**

Remember:
- Start simple, iterate fast
- Test in simulation first
- Keep hardware modular
- Document as you go
- Have fun! This is amazing work!

**Built with ❤️ for cave exploration robotics**
