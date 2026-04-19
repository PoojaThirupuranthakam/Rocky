#!/usr/bin/env python3
"""
Comprehensive AI Navigation Test - No GUI Required
Tests collision recovery, stuck detection, and navigation logic
"""
import sys
sys.path.append('src')

from ai_navigator import AINavigator
import numpy as np

def print_header(text):
    print("\n" + "="*60)
    print(f"  {text}")
    print("="*60)

def test_collision_recovery():
    """Test that collision triggers proper recovery sequence"""
    print_header("TEST 1: Collision Recovery Sequence")
    
    ai = AINavigator()
    
    # Simulate collision scenario
    robot_state = {
        'sensors': {'front': 15, 'left': 80, 'right': 40, 'back': 100},
        'position': (100, 100),
        'angle': 0,
        'speed': 1.5,
        'collision': True  # COLLISION!
    }
    goal = (300, 100)
    
    print("\n📍 Initial state: COLLISION DETECTED")
    print(f"   Front: {robot_state['sensors']['front']}px")
    print(f"   Left: {robot_state['sensors']['left']}px, Right: {robot_state['sensors']['right']}px")
    
    # Frame 1: Should enter recovery mode and backup
    action1 = ai.get_action(robot_state, goal)
    print(f"\n✓ Frame 1:")
    print(f"   Recovery Mode: {ai.recovery_mode}")
    print(f"   Action: {ai.current_action}")
    print(f"   Commitment: {ai.action_commitment}")
    print(f"   Backing up: {action1['backward']}")
    
    assert ai.recovery_mode == True, "❌ Should enter recovery mode on collision"
    assert action1['backward'] == True, "❌ Should backup on collision"
    assert ai.action_commitment >= 15, "❌ Should commit to backing up"
    print("   ✅ PASS: Enters recovery mode and backs up")
    
    # Simulate backing up (update position, no collision now)
    robot_state['collision'] = False
    robot_state['position'] = (95, 100)  # Moved back
    robot_state['sensors']['front'] = 25  # Still close to wall
    
    # Frames 2-18: Should continue backing up
    print(f"\n✓ Frames 2-18: Continue backup")
    for i in range(17):
        action = ai.get_action(robot_state, goal)
        if i == 0 or i == 16:
            print(f"   Frame {i+2}: Action={ai.current_action}, Backward={action['backward']}, Commitment={ai.action_commitment}")
    
    # Frame 19: Backup complete, should start turning
    robot_state['sensors']['front'] = 40  # Still blocked
    action19 = ai.get_action(robot_state, goal)
    print(f"\n✓ Frame 19: Backup complete")
    print(f"   Action: {ai.current_action}")
    print(f"   Recovery Mode: {ai.recovery_mode}")
    print(f"   Turn Direction: {ai.turn_direction}")
    
    assert ai.recovery_mode == True, "❌ Should still be in recovery mode"
    assert ai.current_action in ['TURN_LEFT', 'TURN_RIGHT'], "❌ Should be turning now"
    print(f"   ✅ PASS: Transitions to turning ({ai.current_action})")
    
    # Simulate turning until clear
    robot_state['sensors']['front'] = 45
    for i in range(10):
        ai.get_action(robot_state, goal)
    
    print(f"\n✓ Frames 20-29: Turning...")
    print(f"   Front distance: {robot_state['sensors']['front']}px (still < 70)")
    print(f"   Recovery Mode: {ai.recovery_mode}")
    
    # Clear the path
    robot_state['sensors']['front'] = 120  # NOW CLEAR!
    action_clear = ai.get_action(robot_state, goal)
    
    print(f"\n✓ Path now clear (120px)")
    print(f"   Recovery Mode: {ai.recovery_mode}")
    print(f"   Action: {ai.current_action}")
    print(f"   Moving forward: {action_clear['forward']}")
    
    assert ai.recovery_mode == False, "❌ Should exit recovery when clear"
    assert ai.current_action == 'FORWARD', "❌ Should move forward when clear"
    print("   ✅ PASS: Exits recovery and moves forward")
    
    print("\n" + "🎉 COLLISION RECOVERY TEST: PASSED!")
    return True

def test_stuck_detection():
    """Test that robot detects when stuck and recovers"""
    print_header("TEST 2: Stuck Detection")
    
    ai = AINavigator()
    
    robot_state = {
        'sensors': {'front': 100, 'left': 100, 'right': 100, 'back': 100},
        'position': (100, 100),
        'angle': 0,
        'speed': 0.5,
        'collision': False
    }
    goal = (300, 100)
    
    print("\n📍 Robot at position (100, 100)")
    print("   Trying to move but not making progress...")
    
    # Simulate 20 frames of being stuck (position barely changes)
    for frame in range(20):
        robot_state['position'] = (100.05, 100.05)  # Barely moving
        ai.get_action(robot_state, goal)
        if frame % 5 == 0:
            print(f"   Frame {frame}: stuck_counter={ai.stuck_frames}")
    
    print(f"\n✓ After 20 frames stuck:")
    print(f"   Stuck counter: {ai.stuck_frames}")
    
    # One more frame should trigger recovery
    action = ai.get_action(robot_state, goal)
    
    print(f"\n✓ Frame 21:")
    print(f"   Recovery Mode: {ai.recovery_mode}")
    print(f"   Action: {ai.current_action}")
    print(f"   Backing up: {action['backward']}")
    
    assert ai.recovery_mode == True, "❌ Should enter recovery when stuck"
    assert action['backward'] == True, "❌ Should backup when stuck"
    print("   ✅ PASS: Detects stuck and enters recovery")
    
    print("\n🎉 STUCK DETECTION TEST: PASSED!")
    return True

def test_obstacle_avoidance():
    """Test that robot avoids obstacles without collision"""
    print_header("TEST 3: Proactive Obstacle Avoidance")
    
    ai = AINavigator()
    
    # Obstacle ahead at 45px (< 50px threshold)
    robot_state = {
        'sensors': {'front': 45, 'left': 120, 'right': 60, 'back': 100},
        'position': (100, 100),
        'angle': 0,
        'speed': 1.0,
        'collision': False
    }
    goal = (300, 100)
    
    print("\n📍 Obstacle detected at 45px (threshold: 50px)")
    print(f"   Left: {robot_state['sensors']['left']}px")
    print(f"   Right: {robot_state['sensors']['right']}px")
    
    action = ai.get_action(robot_state, goal)
    
    print(f"\n✓ AI Decision:")
    print(f"   Action: {ai.current_action}")
    print(f"   Turning: Left={action['left']}, Right={action['right']}")
    print(f"   Forward: {action['forward']}")
    
    assert action['left'] == True or action['right'] == True, "❌ Should turn when obstacle close"
    assert action['forward'] == False, "❌ Should not move forward into obstacle"
    assert ai.action_commitment > 0, "❌ Should commit to turn"
    
    # Should turn toward more open space (left: 120 > right: 60)
    assert action['left'] == True, "❌ Should turn left (more space)"
    
    print(f"   ✅ PASS: Turns left (toward open space)")
    print(f"   Commitment: {ai.action_commitment} frames")
    
    print("\n🎉 OBSTACLE AVOIDANCE TEST: PASSED!")
    return True

def test_commitment_system():
    """Test that robot commits to decisions (no oscillation)"""
    print_header("TEST 4: Decision Commitment (Anti-Oscillation)")
    
    ai = AINavigator()
    
    # Start a turn
    robot_state = {
        'sensors': {'front': 45, 'left': 85, 'right': 80, 'back': 100},
        'position': (100, 100),
        'angle': 0,
        'speed': 1.0,
        'collision': False
    }
    goal = (300, 100)
    
    action1 = ai.get_action(robot_state, goal)
    initial_action = ai.current_action
    initial_commitment = ai.action_commitment
    
    print(f"\n📍 Initial decision: {initial_action}")
    print(f"   Commitment: {initial_commitment} frames")
    
    # Even if sensors change slightly, should maintain action
    actions_taken = []
    for i in range(10):
        # Sensors fluctuate slightly
        robot_state['sensors']['left'] = 80 + (i % 3) * 5
        robot_state['sensors']['right'] = 85 - (i % 3) * 5
        action = ai.get_action(robot_state, goal)
        actions_taken.append(ai.current_action)
    
    print(f"\n✓ Actions over 10 frames:")
    print(f"   {actions_taken}")
    
    # Should all be the same (committed)
    unique_actions = set(actions_taken[:initial_commitment])
    assert len(unique_actions) == 1, "❌ Should maintain same action during commitment"
    
    print(f"   ✅ PASS: Maintains {initial_action} consistently")
    print(f"   No oscillation detected!")
    
    print("\n🎉 COMMITMENT SYSTEM TEST: PASSED!")
    return True

def test_goal_seeking():
    """Test that robot moves toward goal when path is clear"""
    print_header("TEST 5: Goal-Seeking Behavior")
    
    ai = AINavigator()
    
    # Clear path ahead
    robot_state = {
        'sensors': {'front': 150, 'left': 100, 'right': 100, 'back': 100},
        'position': (100, 100),
        'angle': 0,  # Facing right (0 radians)
        'speed': 1.0,
        'collision': False
    }
    goal = (300, 100)  # Straight ahead
    
    print(f"\n📍 Robot at (100, 100), Goal at (300, 100)")
    print(f"   Clear path: front={robot_state['sensors']['front']}px")
    print(f"   Facing goal (angle aligned)")
    
    action = ai.get_action(robot_state, goal)
    
    print(f"\n✓ AI Decision:")
    print(f"   Action: {ai.current_action}")
    print(f"   Forward: {action['forward']}")
    print(f"   Turning: {action['left'] or action['right']}")
    
    assert action['forward'] == True, "❌ Should move forward when path clear and aligned"
    assert ai.current_action == 'FORWARD', "❌ Should be in FORWARD state"
    
    print(f"   ✅ PASS: Moves forward toward goal")
    
    # Now test turning toward goal when misaligned
    # First, clear any existing commitments
    ai.action_commitment = 0
    ai.current_action = 'FORWARD'
    
    robot_state['angle'] = np.pi / 2  # Facing up (90 degrees)
    # Goal is still to the right, so need to turn
    
    print(f"\n📍 Robot facing up, goal to the right")
    print(f"   Angle difference: ~90 degrees")
    print(f"   Cleared previous commitment")
    
    action2 = ai.get_action(robot_state, goal)
    
    print(f"\n✓ AI Decision:")
    print(f"   Action: {ai.current_action}")
    print(f"   Turning: Left={action2['left']}, Right={action2['right']}")
    
    assert action2['left'] == True or action2['right'] == True, "❌ Should turn toward goal"
    print(f"   ✅ PASS: Turns toward goal when misaligned")
    
    print("\n🎉 GOAL-SEEKING TEST: PASSED!")
    return True

def main():
    """Run all tests"""
    print("\n")
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║     🧪 COMPREHENSIVE AI NAVIGATION TEST SUITE                ║")
    print("║     Testing collision recovery & navigation logic            ║")
    print("╚══════════════════════════════════════════════════════════════╝")
    
    tests = [
        ("Collision Recovery", test_collision_recovery),
        ("Stuck Detection", test_stuck_detection),
        ("Obstacle Avoidance", test_obstacle_avoidance),
        ("Commitment System", test_commitment_system),
        ("Goal Seeking", test_goal_seeking),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        try:
            if test_func():
                passed += 1
        except AssertionError as e:
            failed += 1
            print(f"\n❌ TEST FAILED: {test_name}")
            print(f"   Error: {e}")
        except Exception as e:
            failed += 1
            print(f"\n💥 TEST ERROR: {test_name}")
            print(f"   Exception: {e}")
    
    print("\n")
    print("╔══════════════════════════════════════════════════════════════╗")
    print("║                    📊 TEST SUMMARY                           ║")
    print("╠══════════════════════════════════════════════════════════════╣")
    print(f"║  ✅ Passed: {passed}/{len(tests)}")
    print(f"║  ❌ Failed: {failed}/{len(tests)}")
    print("╚══════════════════════════════════════════════════════════════╝")
    
    if failed == 0:
        print("\n🎉 ALL TESTS PASSED! AI navigation is working correctly.")
        print("\n📋 Summary of verified behaviors:")
        print("   ✓ Collision triggers recovery mode")
        print("   ✓ Recovery: backup → turn → verify clear → forward")
        print("   ✓ Stuck detection activates after 15 frames")
        print("   ✓ Proactive obstacle avoidance (< 50px)")
        print("   ✓ Decision commitment prevents oscillation")
        print("   ✓ Goal-seeking when path is clear")
        print("\n✅ READY FOR LIVE SIMULATION TEST")
    else:
        print("\n⚠️  Some tests failed. Review the output above.")
    
    return failed == 0

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
