"""Evaluate a trained tabular Q-learning policy in cave episodes."""

from __future__ import annotations

import argparse
import os
import random
import sys

import numpy as np

sys.path.append(os.path.dirname(__file__))

from cave_environment import CaveEnvironment
from robot_controller import RobotController
from rl_q_learning import QTableAgent, apply_action, discretize_state, goal_features, safe_action_mask


def parse_args():
    parser = argparse.ArgumentParser(description="Evaluate trained RL cave policy")
    parser.add_argument("--model-path", default="models/rl_q_table.pkl")
    parser.add_argument("--episodes", type=int, default=40)
    parser.add_argument("--max-steps", type=int, default=420)
    parser.add_argument("--seed", type=int, default=123)
    parser.add_argument("--obstacle-level", choices=["light", "medium", "heavy", "extra-heavy", "mixed"], default="mixed")
    parser.add_argument("--sensor-layout", default="lidar_top_ground_ultrasonic")
    parser.add_argument("--robot-radius", type=int, default=13)
    return parser.parse_args()


def sample_obstacle_level(arg_level: str) -> str:
    if arg_level != "mixed":
        return arg_level
    return random.choices(
        ["light", "medium", "heavy", "extra-heavy"],
        weights=[0.20, 0.45, 0.25, 0.10],
        k=1,
    )[0]


def main():
    args = parse_args()
    if not os.path.exists(args.model_path):
        raise FileNotFoundError(f"RL model not found: {args.model_path}")

    random.seed(args.seed)
    np.random.seed(args.seed)

    agent = QTableAgent.load(args.model_path)

    successes = 0
    collisions = 0
    step_counts = []

    print("=" * 60)
    print("📈 RL POLICY EVALUATION")
    print("=" * 60)

    for ep in range(1, args.episodes + 1):
        level = sample_obstacle_level(args.obstacle_level)
        cave = CaveEnvironment(
            width=640,
            height=480,
            complexity=0.13,
            obstacle_level=level,
            scatter_obstacles_in_path=False,
            robot_radius=args.robot_radius,
        )
        sx, sy = cave.start_pos
        robot = RobotController(sx, sy, cave, sensor_layout=args.sensor_layout, robot_radius=args.robot_radius)

        goal = cave.goal_pos
        state = robot.get_state()
        reached = False

        for step in range(1, args.max_steps + 1):
            s_key = discretize_state(state, goal)
            allowed_now = safe_action_mask(state, goal)
            action = agent.select_action(s_key, greedy=True, allowed_actions=allowed_now)
            apply_action(robot, action)
            robot.update()
            state = robot.get_state()

            if state.get("collision"):
                collisions += 1

            dist, _ = goal_features(state, goal)
            if dist < 30.0:
                reached = True
                step_counts.append(step)
                break

        if reached:
            successes += 1
        else:
            step_counts.append(args.max_steps)

        if ep == 1 or ep % 10 == 0 or ep == args.episodes:
            print(f"episode {ep:3d}/{args.episodes} done")

    success_rate = 100.0 * successes / max(1, args.episodes)
    avg_steps = float(np.mean(step_counts)) if step_counts else 0.0

    print("\nResults")
    print(f"- Success rate: {success_rate:.1f}% ({successes}/{args.episodes})")
    print(f"- Average steps: {avg_steps:.1f}")
    print(f"- Collision ticks: {collisions}")


if __name__ == "__main__":
    main()
