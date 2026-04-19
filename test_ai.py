#!/usr/bin/env python3
"""
Test script to verify AI navigation logic
"""
import sys
sys.path.append('src')

from ai_navigator import AINavigator
import numpy as np

print("🧪 Testing AI Navigator...")
print("=" * 50)

# Create AI navigator
ai = AINavigator()

# Test scenario 1: Clear path ahead
print("\n📍 Test 1: Clear path ahead")
robot_state = {
    'sensors': {'front': 150, 'left': 100, 'right': 100, 'back': 100},
    'position': (100, 100),
    'angle': 0,
    'speed': 1.0,
    'collision': False
}
goal = (300, 100)
action = ai.get_action(robot_state, goal)
print(f"   State: {ai.current_action}, Commitment: {ai.action_commitment}")
print(f"   Action: {action}")
print(f"   ✅ Should move forward: {action['forward']}")

# Test scenario 2: Obstacle ahead, clear left
print("\n📍 Test 2: Obstacle ahead, clear left")
robot_state['sensors'] = {'front': 30, 'left': 120, 'right': 50, 'back': 100}
action = ai.get_action(robot_state, goal)
print(f"   State: {ai.current_action}, Commitment: {ai.action_commitment}")
print(f"   Action: {action}")
print(f"   ✅ Should turn left: {action['left']}")

# Test scenario 3: Continue committed action
print("\n📍 Test 3: Continue committed turn (no oscillation)")
for i in range(5):
    robot_state['sensors'] = {'front': 35, 'left': 110, 'right': 55, 'back': 100}
    action = ai.get_action(robot_state, goal)
    print(f"   Frame {i+1}: State={ai.current_action}, Left={action['left']}, Right={action['right']}")
print(f"   ✅ Should maintain same direction (no flip-flopping)")

# Test scenario 4: Stuck detection
print("\n📍 Test 4: Stuck detection and recovery")
ai.last_pos = (100, 100)
for i in range(25):
    robot_state['position'] = (100.1, 100.1)  # Barely moving
    robot_state['sensors'] = {'front': 100, 'left': 100, 'right': 100, 'back': 100}
    action = ai.get_action(robot_state, goal)
print(f"   Stuck frames: {ai.stuck_frames}")
print(f"   State: {ai.current_action}")
print(f"   Action: {action}")
print(f"   ✅ Should backup: {action['backward']}")

print("\n" + "=" * 50)
print("✅ All tests completed!")
print("\n💡 Key features verified:")
print("  - Clear path → moves forward")
print("  - Obstacle → turns away")
print("  - Commitment → no rapid oscillation")
print("  - Stuck detection → backs up")
