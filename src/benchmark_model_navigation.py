"""Non-GUI benchmark for model navigator performance."""

import math
import random

from cave_environment import CaveEnvironment
from robot_controller import RobotController
from ai_navigator_final import AINavigator


def run_benchmark(episodes=8, frame_limit=1600, stall_limit=180):
    reached = 0
    total_collisions = 0
    total_stall_events = 0

    for ep in range(episodes):
        random.seed(100 + ep)
        cave = CaveEnvironment(800, 600, complexity=0.13)
        model = AINavigator(
            model_path='models/navigation_model.pkl',
            cave_env=cave,
            training_config={'parity_mode': False, 'planner_anchor_mode': True},
        )
        sx, sy = cave.start_pos
        robot = RobotController(
            sx,
            sy,
            cave,
            sensor_layout='lidar_top_ground_ultrasonic',
            robot_radius=8,
        )
        goal = cave.goal_pos

        last_pos = (robot.x, robot.y)
        stall_frames = 0
        collisions = 0
        last_collision_frame = -10
        done = False

        for frame in range(frame_limit):
            state = robot.get_state()
            action = model.get_action(state, goal)

            if action.get('forward'):
                robot.move_forward()
            if action.get('backward'):
                robot.move_backward()
            if action.get('left'):
                robot.turn_left()
            if action.get('right'):
                robot.turn_right()

            robot.update()

            if robot.collision and frame - last_collision_frame > 4:
                collisions += 1
                last_collision_frame = frame

            moved = abs(robot.x - last_pos[0]) + abs(robot.y - last_pos[1])
            stall_frames = stall_frames + 1 if moved < 0.22 else max(0, stall_frames - 1)
            last_pos = (robot.x, robot.y)

            # Allow longer no-motion windows because planner recovery/scan can
            # intentionally spend time rotating/backtracking in tight traps.
            if stall_frames > stall_limit:
                total_stall_events += 1
                break

            if math.hypot(robot.x - goal[0], robot.y - goal[1]) < 30:
                reached += 1
                done = True
                break

        total_collisions += collisions
        print(f"EP{ep}: reached={done} collisions={collisions} stall_frames={stall_frames}")

    print(
        "SUMMARY "
        f"reached={reached}/{episodes} "
        f"avg_collisions={total_collisions / episodes:.2f} "
        f"stall_events={total_stall_events}"
    )


if __name__ == '__main__':
    run_benchmark()
