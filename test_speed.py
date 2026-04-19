#!/usr/bin/env python3
"""
SPEED & COLLISION TEST
Tests that robot reaches goal quickly without collisions
"""
import argparse
import copy
import math
import random
import sys
import time

import numpy as np

sys.path.append('src')

from cave_environment import CaveEnvironment
from robot_controller import RobotController
from ai_navigator import AINavigator as PlannerNavigator
from ai_navigator_final import AINavigator as ModelNavigator


def parse_args():
    parser = argparse.ArgumentParser(description="Speed/collision test for planner/model navigator")
    parser.add_argument(
        "--navigator",
        choices=["planner", "model", "both"],
        default="planner",
        help="Navigator to test: planner, model, or both side-by-side",
    )
    parser.add_argument("--seed", type=int, default=123, help="Random seed for deterministic map generation")
    parser.add_argument("--max-frames", type=int, default=1000, help="Maximum frames per run")
    parser.add_argument(
        "--person-height-cm",
        type=float,
        default=165.0,
        help="Person height in cm used for cave-clearance shaping",
    )
    parser.add_argument(
        "--person-width-cm",
        type=float,
        default=45.0,
        help="Person width in cm used for cave-clearance shaping",
    )
    parser.add_argument(
        "--obstacle-level",
        choices=["light", "medium", "heavy", "extra-heavy"],
        default="medium",
        help="Obstacle density profile for cave generation",
    )
    return parser.parse_args()


def run_single(
    navigator_kind,
    seed,
    max_frames,
    person_height_cm,
    person_width_cm,
    obstacle_level,
    cave_template=None,
):
    random.seed(seed)
    np.random.seed(seed)

    cave = (
        copy.deepcopy(cave_template)
        if cave_template is not None
        else CaveEnvironment(
            width=800,
            height=600,
            obstacle_level=obstacle_level,
            person_height_cm=person_height_cm,
            person_width_cm=person_width_cm,
            cm_per_pixel=0.5,
        )
    )
    start_x, start_y = cave.start_pos
    robot = RobotController(start_x, start_y, cave, sensor_layout='lidar_top_ground_ultrasonic')
    goal = cave.goal_pos
    distance = math.sqrt((goal[0] - start_x) ** 2 + (goal[1] - start_y) ** 2)

    if navigator_kind == "model":
        ai = ModelNavigator(
            model_path='models/navigation_model.pkl',
            cave_env=cave,
            training_config={'parity_mode': False, 'planner_anchor_mode': True},
        )
    else:
        ai = PlannerNavigator(cave)

    print()
    print(f"🏗️  [{navigator_kind}] Setting up test environment...")
    print(f"📍 Start: ({start_x:.0f}, {start_y:.0f})")
    print(f"🎯 Goal:  ({goal[0]:.0f}, {goal[1]:.0f})")
    print(f"📏 Distance: {distance:.0f} pixels")
    print("🤖 Running AI navigation test...")
    print("   Target: < 500 frames (8.3 seconds @ 60 FPS)")

    frames = 0
    collisions = 0
    last_collision_frame = -10
    reached_goal = False
    dist_to_goal = distance
    start_time = time.time()

    while frames < max_frames:
        robot_state = robot.get_state()
        action = ai.get_action(robot_state, goal)

        if action['forward']:
            robot.move_forward()
        if action['backward']:
            robot.move_backward()
        if action['left']:
            robot.turn_left()
        if action['right']:
            robot.turn_right()

        robot.update()

        if robot.collision and frames - last_collision_frame > 5:
            collisions += 1
            last_collision_frame = frames

        dx = robot.x - goal[0]
        dy = robot.y - goal[1]
        dist_to_goal = math.sqrt(dx * dx + dy * dy)

        if dist_to_goal < 30:
            reached_goal = True
            break

        if frames % 100 == 0:
            print(f"   [{navigator_kind}] Frame {frames}: Distance={dist_to_goal:.0f}px, Collisions={collisions}")

        frames += 1

    elapsed_time = time.time() - start_time

    result = {
        "navigator": navigator_kind,
        "reached_goal": reached_goal,
        "frames": frames,
        "collisions": collisions,
        "dist_to_goal": dist_to_goal,
        "elapsed_time": elapsed_time,
        "efficiency": (distance / 3.5 / max(1, frames)) * 100,
        "oscillation": ai.get_oscillation_stats() if hasattr(ai, "get_oscillation_stats") else None,
    }
    return result


def print_result(result):
    kind = result["navigator"]
    print()
    print(f"## [{kind}] RESULTS")
    if result["reached_goal"]:
        print("✅ SUCCESS! Robot reached goal")
        print(f"   Frames: {result['frames']}")
        print(f"   Time: {result['frames']/60:.1f} seconds @ 60 FPS")
        print(f"   Real time: {result['elapsed_time']:.2f} seconds")
        print(f"   Collisions: {result['collisions']}")
        print(f"   Efficiency: {result['efficiency']:.1f}%")
    else:
        print("❌ TIMEOUT")
        print(f"   Final distance to goal: {result['dist_to_goal']:.0f}px")
        print(f"   Collisions: {result['collisions']}")

    if result["oscillation"] is not None:
        osc = result["oscillation"]
        print(
            "🔁 Steering flips: "
            f"total={osc['steer_flip_total']}, "
            f"last_{osc['window_size']}={osc['steer_flip_last_100']}"
        )


def is_strict_pass(result):
    return result["reached_goal"] and result["frames"] < 500 and result["collisions"] <= 3


def main():
    args = parse_args()

    print("╔══════════════════════════════════════════════════════════════╗")
    print("║     🚀 SPEED & COLLISION AVOIDANCE TEST                      ║")
    print("╚══════════════════════════════════════════════════════════════╝")

    modes = [args.navigator] if args.navigator != "both" else ["planner", "model"]
    results = []
    shared_cave = None
    if args.navigator == "both":
        random.seed(args.seed)
        np.random.seed(args.seed)
        shared_cave = CaveEnvironment(
            width=800,
            height=600,
            obstacle_level=args.obstacle_level,
            person_height_cm=args.person_height_cm,
            person_width_cm=args.person_width_cm,
            cm_per_pixel=0.5,
        )

    for idx, mode in enumerate(modes):
        run_seed = args.seed
        results.append(
            run_single(
                mode,
                run_seed,
                args.max_frames,
                args.person_height_cm,
                args.person_width_cm,
                args.obstacle_level,
                cave_template=shared_cave,
            )
        )

    print("\n" + "=" * 64)
    for result in results:
        print_result(result)

    if len(results) == 2:
        print("\n📊 Comparison summary")
        for r in results:
            status = "PASS" if is_strict_pass(r) else ("REACHED" if r["reached_goal"] else "FAIL")
            print(
                f"- {r['navigator']}: {status}, frames={r['frames']}, "
                f"collisions={r['collisions']}, dist={r['dist_to_goal']:.0f}px"
            )

    if all(not r["reached_goal"] for r in results):
        print("❌ FAIL: No navigator reached goal")
        sys.exit(1)

    if all(is_strict_pass(r) for r in results):
        print("🎉 PASS: All tested navigators meet strict speed/collision target")
    else:
        print("⚠️  PASS WITH ISSUES: At least one navigator reached goal, but not all met strict target")
    sys.exit(0)


if __name__ == "__main__":
    main()
