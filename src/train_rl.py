"""Train a tabular Q-learning policy for cave robot navigation."""

from __future__ import annotations

import argparse
import os
import random
import sys
from typing import Tuple

import numpy as np

sys.path.append(os.path.dirname(__file__))

from cave_environment import CaveEnvironment
from robot_controller import RobotController
from ai_navigator import AINavigator as PlannerNavigator
from rl_q_learning import (
    QLearningConfig,
    QTableAgent,
    apply_action,
    command_to_action_id,
    discretize_state,
    goal_features,
    safe_action_mask,
)


def parse_args():
    parser = argparse.ArgumentParser(description="Train tabular Q-learning policy for cave robot")
    parser.add_argument("--episodes", type=int, default=260, help="Training episodes")
    parser.add_argument("--max-steps", type=int, default=420, help="Max steps per episode")
    parser.add_argument("--model-path", default="models/rl_q_table.pkl", help="Output Q-table path")
    parser.add_argument("--seed", type=int, default=42, help="Random seed")
    parser.add_argument("--obstacle-level", choices=["light", "medium", "heavy", "extra-heavy", "mixed"], default="mixed")
    parser.add_argument("--sensor-layout", default="lidar_top_ground_ultrasonic")

    parser.add_argument("--lr", type=float, default=0.12, help="Q-learning rate")
    parser.add_argument("--gamma", type=float, default=0.97, help="Discount factor")
    parser.add_argument("--epsilon-start", type=float, default=1.0)
    parser.add_argument("--epsilon-min", type=float, default=0.06)
    parser.add_argument("--epsilon-decay", type=float, default=0.993)
    parser.add_argument("--robot-radius", type=int, default=13)
    parser.add_argument(
        "--warmstart-episodes",
        type=int,
        default=80,
        help="Planner imitation episodes to warm-start Q-table before RL fine-tuning",
    )
    parser.add_argument(
        "--warmstart-max-steps",
        type=int,
        default=220,
        help="Max steps per planner imitation episode",
    )
    parser.add_argument(
        "--warmstart-bonus",
        type=float,
        default=0.9,
        help="Extra reward bonus for planner-labeled actions during warm-start",
    )

    return parser.parse_args()


def sample_obstacle_level(arg_level: str, episode_idx: int, total_episodes: int) -> str:
    if arg_level != "mixed":
        return arg_level

    phase = float(episode_idx) / max(1.0, float(total_episodes))
    if phase < 0.35:
        weights = [0.45, 0.40, 0.12, 0.03]
    elif phase < 0.70:
        weights = [0.22, 0.46, 0.25, 0.07]
    else:
        weights = [0.12, 0.38, 0.34, 0.16]

    return random.choices(["light", "medium", "heavy", "extra-heavy"], weights=weights, k=1)[0]


def build_episode_world(obstacle_level: str, sensor_layout: str, robot_radius: int) -> Tuple[CaveEnvironment, RobotController]:
    cave = CaveEnvironment(
        width=640,
        height=480,
        complexity=0.13,
        obstacle_level=obstacle_level,
        scatter_obstacles_in_path=False,
        robot_radius=robot_radius,
    )
    sx, sy = cave.start_pos
    robot = RobotController(sx, sy, cave, sensor_layout=sensor_layout, robot_radius=robot_radius)
    return cave, robot


def is_opposite_action(a: int | None, b: int) -> bool:
    if a is None:
        return False
    opposites = {
        1: 2,
        2: 1,
        3: 4,
        4: 3,
        6: 7,
        7: 6,
    }
    return opposites.get(int(a)) == int(b)


def compute_reward(
    prev_state,
    next_state,
    goal,
    reached_goal: bool,
    action_id: int,
    prev_action_id: int | None,
    revisit_count: int,
) -> float:
    prev_dist, prev_heading = goal_features(prev_state, goal)
    next_dist, next_heading = goal_features(next_state, goal)
    progress = prev_dist - next_dist

    reward = -0.03  # living cost
    reward += 0.75 * progress

    # Reward turning toward goal heading.
    heading_improve = abs(prev_heading) - abs(next_heading)
    reward += 0.38 * heading_improve

    # Encourage movement while mostly facing goal.
    speed = float(next_state.get("speed", 0.0) or 0.0)
    if speed > 0.35 and abs(next_heading) < 0.75:
        reward += 0.06

    if next_state.get("collision"):
        reward -= 3.5

    sensors = next_state.get("sensors", {}) or {}
    front = min(
        float(sensors.get("lidar_front", sensors.get("top_front", sensors.get("top", 999.0)))),
        float(sensors.get("ground_ultrasonic", sensors.get("ground_front", sensors.get("ground", 999.0)))),
    )
    if front < 12:
        reward -= 1.9
    elif front < 18:
        reward -= 0.9
    elif front < 26:
        reward -= 0.2

    # Penalize reverse-heavy choices unless space is very tight ahead.
    if action_id in (5, 6, 7) and front > 16:
        reward -= 0.22

    # Penalize spinning in place when not constrained.
    if action_id in (3, 4) and front > 28:
        reward -= 0.09

    # Penalize immediate left-right / right-left oscillation.
    if is_opposite_action(prev_action_id, action_id):
        reward -= 0.32

    # Penalize repeatedly revisiting same coarse region (looping behavior).
    if revisit_count > 2:
        reward -= min(0.9, 0.08 * (revisit_count - 2))

    if reached_goal:
        reward += 120.0

    return float(reward)


def run_planner_warmstart(agent: QTableAgent, args) -> None:
    warm_eps = max(0, int(args.warmstart_episodes))
    if warm_eps <= 0:
        return

    print("\n🧭 Planner imitation warm-start")
    print(f"episodes={warm_eps}, max_steps={args.warmstart_max_steps}")

    copied_steps = 0
    warm_successes = 0

    for ep in range(1, warm_eps + 1):
        level = sample_obstacle_level(args.obstacle_level, ep, warm_eps)
        cave, robot = build_episode_world(level, args.sensor_layout, args.robot_radius)
        planner = PlannerNavigator(cave)
        goal = cave.goal_pos
        state = robot.get_state()

        reached = False
        for _step in range(1, max(1, int(args.warmstart_max_steps)) + 1):
            s_key = discretize_state(state, goal)
            teacher_cmd = planner.get_action(state, goal)
            teacher_action = command_to_action_id(teacher_cmd)

            # Keep teacher action safe if needed.
            allowed_now = safe_action_mask(state, goal)
            if allowed_now and teacher_action not in allowed_now:
                q_vals = agent.predict_q_values(s_key)
                teacher_action = max(allowed_now, key=lambda a: float(q_vals[a]))

            apply_action(robot, teacher_action)
            robot.update()
            next_state = robot.get_state()
            next_dist, _ = goal_features(next_state, goal)
            reached_goal = next_dist < 30.0

            imitation_reward = float(args.warmstart_bonus)
            if next_state.get("collision"):
                imitation_reward -= 1.6
            if reached_goal:
                imitation_reward += 40.0

            n_key = discretize_state(next_state, goal)
            allowed_next = safe_action_mask(next_state, goal)
            agent.update(
                s_key,
                int(teacher_action),
                imitation_reward,
                n_key,
                bool(reached_goal),
                next_allowed_actions=allowed_next,
            )

            copied_steps += 1
            state = next_state
            if reached_goal:
                reached = True
                break

        warm_successes += 1 if reached else 0
        if ep == 1 or ep % 20 == 0 or ep == warm_eps:
            sr = 100.0 * warm_successes / max(1, ep)
            print(f"warm_ep={ep:4d}/{warm_eps} states={len(agent.q_table):5d} teacher_success={sr:5.1f}%")

    print(f"Warm-start copied transitions: {copied_steps}")


def main():
    args = parse_args()

    random.seed(args.seed)
    np.random.seed(args.seed)

    config = QLearningConfig(
        learning_rate=max(1e-4, float(args.lr)),
        gamma=min(0.999, max(0.1, float(args.gamma))),
        epsilon_start=max(0.0, min(1.0, float(args.epsilon_start))),
        epsilon_min=max(0.0, min(1.0, float(args.epsilon_min))),
        epsilon_decay=min(0.9999, max(0.8, float(args.epsilon_decay))),
    )
    if config.epsilon_min > config.epsilon_start:
        config.epsilon_min = config.epsilon_start

    agent = QTableAgent(config=config)

    run_planner_warmstart(agent, args)

    print("=" * 60)
    print("🤖 Q-LEARNING TRAINING")
    print("=" * 60)
    print(f"episodes={args.episodes}, max_steps={args.max_steps}")

    success_window = []
    reward_window = []

    for ep in range(1, args.episodes + 1):
        level = sample_obstacle_level(args.obstacle_level, ep, args.episodes)
        cave, robot = build_episode_world(level, args.sensor_layout, args.robot_radius)
        goal = cave.goal_pos

        state = robot.get_state()
        total_reward = 0.0
        success = False
        no_progress_steps = 0
        prev_action_id = None
        visit_counter = {}

        prev_dist, _ = goal_features(state, goal)

        for step in range(1, args.max_steps + 1):
            s_key = discretize_state(state, goal)
            allowed_now = safe_action_mask(state, goal)
            action_id = agent.select_action(s_key, allowed_actions=allowed_now)

            apply_action(robot, action_id)
            robot.update()
            next_state = robot.get_state()

            next_dist, _ = goal_features(next_state, goal)
            reached_goal = next_dist < 30.0

            pos_x, pos_y = next_state.get("position", (0.0, 0.0))
            cell = (int(pos_x // 24), int(pos_y // 24))
            visit_counter[cell] = visit_counter.get(cell, 0) + 1
            revisit_count = int(visit_counter[cell])

            reward = compute_reward(
                state,
                next_state,
                goal,
                reached_goal,
                action_id,
                prev_action_id,
                revisit_count,
            )
            if next_dist > prev_dist - 0.6:
                no_progress_steps += 1
                if no_progress_steps > 45:
                    reward -= 0.8
            else:
                no_progress_steps = 0

            done = bool(reached_goal)
            if no_progress_steps > 95:
                done = True
                reward -= 3.0

            n_key = discretize_state(next_state, goal)
            allowed_next = safe_action_mask(next_state, goal)
            agent.update(
                s_key,
                action_id,
                reward,
                n_key,
                done,
                next_allowed_actions=allowed_next,
            )

            state = next_state
            prev_action_id = action_id
            prev_dist = next_dist
            total_reward += reward

            if done:
                success = reached_goal
                break

        agent.decay_epsilon()
        success_window.append(1 if success else 0)
        reward_window.append(total_reward)
        if len(success_window) > 25:
            success_window.pop(0)
            reward_window.pop(0)

        if ep == 1 or ep % 20 == 0 or ep == args.episodes:
            sr = 100.0 * (sum(success_window) / max(1, len(success_window)))
            avg_r = float(np.mean(reward_window)) if reward_window else 0.0
            print(
                f"ep={ep:4d}/{args.episodes} "
                f"eps={agent.epsilon:.3f} "
                f"table_states={len(agent.q_table):5d} "
                f"window_success={sr:5.1f}% "
                f"window_reward={avg_r:7.2f}"
            )

    metadata = {
        "episodes": int(args.episodes),
        "max_steps": int(args.max_steps),
        "seed": int(args.seed),
        "sensor_layout": args.sensor_layout,
        "obstacle_level": args.obstacle_level,
        "robot_radius": int(args.robot_radius),
    }
    agent.save(args.model_path, metadata=metadata)

    print("\n✅ Training complete")
    print(f"Saved RL policy: {args.model_path}")
    print(f"State entries: {len(agent.q_table)}")
    print(f"Final epsilon: {agent.epsilon:.4f}")


if __name__ == "__main__":
    main()
