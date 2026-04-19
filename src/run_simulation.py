"""
Main Simulation Runner
Run this to test your robot in the virtual cave
"""

import argparse
import os
import pygame
import sys
from cave_environment import CaveEnvironment
from robot_controller import RobotController


DEFAULT_CM_PER_PIXEL = 0.5  # 1 px = 0.5 cm
INCH_TO_CM = 2.54
DEFAULT_ROBOT_SIZE_IN = (7.0, 5.0, 5.0)  # L x W x H (inches)
DEFAULT_ROBOT_SIZE_CM = tuple(v * INCH_TO_CM for v in DEFAULT_ROBOT_SIZE_IN)
DEFAULT_PERSON_SIZE_CM = (165.0, 45.0)  # H x W (adult baseline)
DEFAULT_ROBOT_RADIUS_PX = int(round((DEFAULT_ROBOT_SIZE_CM[1] / 2.0) / DEFAULT_CM_PER_PIXEL))


class CaveSimulation:
    """Main simulation class"""

    def __init__(
        self,
        width=800,
        height=600,
        sensor_layout='lidar_top_ground_ultrasonic',
        robot_radius=DEFAULT_ROBOT_RADIUS_PX,
        robot_speed_scale=1.0,
        person_height_cm=DEFAULT_PERSON_SIZE_CM[0],
        person_width_cm=DEFAULT_PERSON_SIZE_CM[1],
        obstacle_level='medium',
        scatter_obstacles_in_path=False,
        navigator='planner',
        retrain_model=False,
        rl_model_path='models/rl_q_table.pkl',
        rl_online_update=False,
        rl_autosave_interval=300,
        post_goal_action='reset',
    ):
        pygame.init()

        # Display setup
        self.width = width
        self.height = height
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("Cave Explorer Robot - Hackathon Simulator")

        # Clock for FPS control
        self.clock = pygame.time.Clock()
        self.fps = 60

    # Robot/profile config
        self.sensor_layout = sensor_layout
        self.robot_radius = max(4, int(robot_radius))
        self.robot_speed_scale = max(0.25, min(3.0, float(robot_speed_scale)))
        self.person_height_cm = max(80.0, float(person_height_cm))
        self.person_width_cm = max(20.0, float(person_width_cm))
        self.obstacle_level = str(obstacle_level).strip().lower()
        self.scatter_obstacles_in_path = bool(scatter_obstacles_in_path)
        if self.obstacle_level not in {'light', 'medium', 'heavy', 'extra-heavy'}:
            self.obstacle_level = 'medium'
        self.navigator_type = navigator
        self.rl_model_path = rl_model_path
        self.rl_online_update = bool(rl_online_update)
        self.rl_autosave_interval = max(1, int(rl_autosave_interval))
        self.post_goal_action = str(post_goal_action).strip().lower()
        if self.post_goal_action not in {'reset', 'return', 'shuttle'}:
            self.post_goal_action = 'reset'

        # Create cave environment
        print("🏗️  Building virtual cave...")
        self.cave = CaveEnvironment(
            width,
            height,
            complexity=0.13,
            obstacle_level=self.obstacle_level,
            scatter_obstacles_in_path=self.scatter_obstacles_in_path,
            robot_radius=self.robot_radius,
            person_width_cm=self.person_width_cm,
            person_height_cm=self.person_height_cm,
            cm_per_pixel=DEFAULT_CM_PER_PIXEL,
        )

        # Create robot
        print("🤖 Spawning robot...")
        start_x, start_y = self.cave.start_pos
        self.robot = RobotController(
            start_x,
            start_y,
            self.cave,
            sensor_layout=self.sensor_layout,
            robot_radius=self.robot_radius,
            speed_scale=self.robot_speed_scale,
        )
        self.start_pos = (start_x, start_y)
        self.primary_goal = self.cave.goal_pos
        self.current_target = self.primary_goal
        self.returning_to_start = False
        self._target_reached_latch = False

        # Create AI navigator
        print("🧠 Initializing AI navigator...")
        self.ai = self._create_navigator(retrain_model)

        # Simulation state
        self.running = True
        self.ai_mode = False
        self.show_debug = True

        # Fonts
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)

        print("✅ Simulation ready!")
        print("\nControls:")
        print("  W/S: Forward/Backward")
        print("  A/D: Turn Left/Right")
        print("  SPACE: Toggle AI Mode")
        print("  H: Toggle Debug Info")
        print("  [ / ]: Decrease / Increase robot speed")
        print("  T: Cycle Post-Goal Action (reset/return/shuttle)")
        print("  R: Reset Simulation")
        print("  Q/ESC: Quit\n")
        print(f"Navigator: {self.navigator_type}")
        if self.navigator_type == 'rl':
            print(f"RL model path: {self.rl_model_path}")
            print(f"RL online update: {'ON' if self.rl_online_update else 'OFF'}")
            if self.rl_online_update:
                print(f"RL autosave interval: every {self.rl_autosave_interval} updates")
        print(f"Post-goal action: {self.post_goal_action}")
        print(f"Sensor layout: {self.sensor_layout}")
        print(f"Obstacle level: {self.obstacle_level}")
        print(f"Scatter obstacles in path: {'ON' if self.scatter_obstacles_in_path else 'OFF'}")
        print(f"Robot radius: {self.robot_radius}px (diameter: {self.robot_radius * 2}px)")
        print(
            "Sensor range: "
            f"{self.robot.ultrasonic_max_distance_px}px "
            f"(~{self.robot.ultrasonic_max_distance_px * DEFAULT_CM_PER_PIXEL:.1f} cm / "
            f"{(self.robot.ultrasonic_max_distance_px * DEFAULT_CM_PER_PIXEL) / 30.48:.2f} ft)"
        )
        print(
            "Robot size (cm): "
            f"{DEFAULT_ROBOT_SIZE_CM[0]:.1f}x{DEFAULT_ROBOT_SIZE_CM[1]:.1f}x{DEFAULT_ROBOT_SIZE_CM[2]:.1f} "
            f"({DEFAULT_ROBOT_SIZE_IN[0]:.1f}x{DEFAULT_ROBOT_SIZE_IN[1]:.1f}x{DEFAULT_ROBOT_SIZE_IN[2]:.1f} in) "
            f"@ {DEFAULT_CM_PER_PIXEL:.2f} cm/px"
        )
        print(f"Robot speed scale: x{self.robot.speed_scale:.2f}")
        print(f"Person profile: {self.person_height_cm:.1f}cm tall, {self.person_width_cm:.1f}cm wide")
        if self.navigator_type == 'model':
            metrics = getattr(self.ai, 'training_metrics', None) or {}
            val_acc = metrics.get('val_action_accuracy')
            train_loss = metrics.get('train_loss')
            if val_acc is not None:
                print(f"Model val_action_acc: {val_acc:.3f}")
            if train_loss is not None:
                print(f"Model train_loss: {train_loss:.4f}")

    def _create_navigator(self, retrain_model=False):
        """Create selected navigator mode."""
        if self.navigator_type == 'rl':
            from ai_navigator_rl import AINavigator as RLNavigator

            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = self.rl_model_path
            if not os.path.isabs(model_path):
                model_path = os.path.join(project_root, model_path)
            return RLNavigator(
                model_path=model_path,
                cave_env=self.cave,
                planner_fallback=True,
            )

        if self.navigator_type == 'model':
            from ai_navigator_final import AINavigator as ModelNavigator

            project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(project_root, 'models', 'navigation_model.pkl')
            if retrain_model and os.path.exists(model_path):
                print(f"♻️ Retraining model: removing old model at {model_path}")
                os.remove(model_path)
            return ModelNavigator(
                model_path=model_path,
                cave_env=self.cave,
                training_config={'sensor_layout': self.sensor_layout},
            )

        from ai_navigator import AINavigator as PlannerNavigator
        return PlannerNavigator(self.cave)

    def handle_input(self):
        """Handle keyboard input"""
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.running = False

            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_q or event.key == pygame.K_ESCAPE:
                    self.running = False
                elif event.key == pygame.K_SPACE:
                    self.ai_mode = not self.ai_mode
                    mode = "ON" if self.ai_mode else "OFF"
                    print(f"🤖 AI Mode: {mode}")
                elif event.key == pygame.K_h:
                    self.show_debug = not self.show_debug
                elif event.key == pygame.K_t:
                    self.cycle_post_goal_action()
                elif event.key in (pygame.K_LEFTBRACKET, pygame.K_MINUS):
                    self.adjust_robot_speed(-0.1)
                elif event.key in (pygame.K_RIGHTBRACKET, pygame.K_EQUALS):
                    self.adjust_robot_speed(0.1)
                elif event.key == pygame.K_r:
                    self.reset()

        # Manual controls (only when AI is off)
        if not self.ai_mode:
            keys = pygame.key.get_pressed()
            if keys[pygame.K_w]:
                self.robot.move_forward()
            if keys[pygame.K_s]:
                self.robot.move_backward()
            if keys[pygame.K_a]:
                self.robot.turn_left()
            if keys[pygame.K_d]:
                self.robot.turn_right()

    def cycle_post_goal_action(self):
        """Cycle post-goal behavior at runtime."""
        modes = ('reset', 'return', 'shuttle')
        try:
            idx = modes.index(self.post_goal_action)
        except ValueError:
            idx = 0
        self.post_goal_action = modes[(idx + 1) % len(modes)]
        print(f"🔁 Post-goal action: {self.post_goal_action}")

    def adjust_robot_speed(self, delta: float):
        """Adjust robot speed multiplier at runtime."""
        if hasattr(self.robot, 'adjust_speed_scale'):
            new_scale = self.robot.adjust_speed_scale(delta)
            self.robot_speed_scale = float(new_scale)
            print(f"⚡ Robot speed scale: x{new_scale:.2f}")

    def update(self):
        """Update simulation state"""
        state_before = None
        commands = None

        # AI control
        if self.ai_mode:
            state_before = self.robot.get_state()
            if self.navigator_type == 'rl' and hasattr(self.ai, 'get_action'):
                try:
                    commands = self.ai.get_action(
                        state_before,
                        self.current_target,
                        explore=self.rl_online_update,
                    )
                except TypeError:
                    commands = self.ai.get_action(state_before, self.current_target)
            else:
                commands = self.ai.get_action(state_before, self.current_target)

            if commands['forward']:
                self.robot.move_forward()
            if commands['backward']:
                self.robot.move_backward()
            if commands['left']:
                self.robot.turn_left()
            if commands['right']:
                self.robot.turn_right()

        # Update robot physics
        self.robot.update()
        state_after = self.robot.get_state()

        # Check if active target reached (with latch to avoid repeated triggers)
        target_x, target_y = self.current_target
        dist_to_target = ((self.robot.x - target_x) ** 2 + (self.robot.y - target_y) ** 2) ** 0.5
        reached_target = dist_to_target < 30

        # Optional online RL update from this transition.
        if (
            self.ai_mode
            and self.navigator_type == 'rl'
            and self.rl_online_update
            and state_before is not None
            and commands is not None
            and hasattr(self.ai, 'online_step')
        ):
            self.ai.online_step(
                state_before,
                commands,
                state_after,
                self.current_target,
                reached_goal=reached_target,
                autosave_interval=self.rl_autosave_interval,
            )

        if reached_target and not self._target_reached_latch:
            if self.current_target == self.primary_goal:
                print("🎉 DESTINATION REACHED!")
                if self.post_goal_action in {'return', 'shuttle'}:
                    self.current_target = self.start_pos
                    self.returning_to_start = True
                    if hasattr(self.ai, 'reset_runtime'):
                        self.ai.reset_runtime()
                    print("↩️ Switching target: return to START")
                else:
                    self.reset()
            else:
                print("🏁 START POINT REACHED!")
                if self.post_goal_action == 'shuttle':
                    self.current_target = self.primary_goal
                    self.returning_to_start = False
                    if hasattr(self.ai, 'reset_runtime'):
                        self.ai.reset_runtime()
                    print("🎯 Switching target: go to DESTINATION")
                elif self.post_goal_action == 'return':
                    self.ai_mode = False
                    print("✅ Return complete. AI mode turned OFF.")
                else:
                    self.reset()

        self._target_reached_latch = reached_target

    def render(self):
        """Render everything to screen"""
        # Clear screen (dark background for cave)
        self.screen.fill((20, 20, 20))

        # Draw cave
        self.cave.render(self.screen)

        # Draw robot
        self.robot.render(self.screen)

        # Draw UI
        self.render_ui()
        self.render_terrain_legend()

        # Update display
        pygame.display.flip()

    def render_terrain_legend(self):
        """Render legend for symbolic cave markers."""
        legend = self.cave.get_marker_legend() if hasattr(self.cave, 'get_marker_legend') else []
        if not legend:
            return

        legend_width = 260
        legend_height = 20 + len(legend) * 16
        x0 = self.width - legend_width - 10
        y0 = 10

        s = pygame.Surface((legend_width, legend_height), pygame.SRCALPHA)
        s.fill((0, 0, 0, 150))
        self.screen.blit(s, (x0, y0))

        title = self.small_font.render("Terrain Symbols", True, (220, 220, 220))
        self.screen.blit(title, (x0 + 8, y0 + 4))

        for idx, (name, symbol, color) in enumerate(legend):
            yy = y0 + 22 + idx * 16
            symbol_text = self.small_font.render(symbol, True, color)
            label_text = self.small_font.render(name.replace('_', ' '), True, (210, 210, 210))
            self.screen.blit(symbol_text, (x0 + 8, yy))
            self.screen.blit(label_text, (x0 + 32, yy))

    def render_ui(self):
        """Render user interface elements"""
        if not self.show_debug:
            return

        # Status panel background
        panel_height = 145
        s = pygame.Surface((self.width, panel_height), pygame.SRCALPHA)
        s.fill((0, 0, 0, 180))
        self.screen.blit(s, (0, self.height - panel_height))

        # Robot state
        state = self.robot.get_state()
        y_offset = self.height - panel_height + 10

        # Mode indicator
        mode_text = "🤖 AI MODE" if self.ai_mode else "👤 MANUAL MODE"
        mode_color = (0, 255, 0) if self.ai_mode else (255, 255, 0)
        text = self.font.render(mode_text, True, mode_color)
        self.screen.blit(text, (10, y_offset))

        # Sensor layout
        layout_text = f"Sensor Layout: {self.sensor_layout}"
        text = self.small_font.render(layout_text, True, (180, 220, 255))
        self.screen.blit(text, (220, y_offset + 5))

        nav_text = f"Navigator: {self.navigator_type}"
        text = self.small_font.render(nav_text, True, (180, 220, 255))
        self.screen.blit(text, (220, y_offset + 22))

        size_text = f"Robot radius: {self.robot_radius}px"
        text = self.small_font.render(size_text, True, (180, 220, 255))
        self.screen.blit(text, (220, y_offset + 39))

        person_text = f"Person: H {self.person_height_cm:.0f}cm  W {self.person_width_cm:.0f}cm"
        text = self.small_font.render(person_text, True, (180, 220, 255))
        self.screen.blit(text, (220, y_offset + 56))

        if self.current_target == self.primary_goal:
            target_text = "Target: DESTINATION"
        else:
            target_text = "Target: START"
        text = self.small_font.render(target_text, True, (180, 255, 180))
        self.screen.blit(text, (220, y_offset + 73))

        # Position
        pos_text = f"Position: ({int(state['position'][0])}, {int(state['position'][1])})"
        text = self.small_font.render(pos_text, True, (200, 200, 200))
        self.screen.blit(text, (10, y_offset + 25))

        # Speed
        speed_text = f"Speed: {state['speed']:.2f}  (x{self.robot.speed_scale:.2f})"
        text = self.small_font.render(speed_text, True, (200, 200, 200))
        self.screen.blit(text, (10, y_offset + 45))

        sensors = state['sensors']
        if self.sensor_layout in ('lidar_top_ground_ultrasonic', 'lidar_2d_floor'):
            sensor_front = float(sensors.get('lidar_front', sensors.get('lidar_min', 0.0)))
            sensor_front = min(sensor_front, float(sensors.get('ground_ultrasonic', sensors.get('ground_front', sensor_front))))
            sensor_left = min(
                float(sensors.get('lidar_left', sensor_front)),
                float(sensors.get('ground_left', sensors.get('ground_front', sensor_front))),
            )
            sensor_right = min(
                float(sensors.get('lidar_right', sensor_front)),
                float(sensors.get('ground_right', sensors.get('ground_front', sensor_front))),
            )
        else:
            sensor_front = min(
                sensors.get('top_front', sensors.get('top', 0.0)),
                sensors.get('ground_front', sensors.get('ground', 0.0)),
            )
            sensor_left = min(
                sensors.get('top_left', sensor_front),
                sensors.get('ground_left', sensor_front),
            )
            sensor_right = min(
                sensors.get('top_right', sensor_front),
                sensors.get('ground_right', sensor_front),
            )
        sensor_text = (
            f"Clearance: Front:{sensor_front:.0f} "
            f"Left:{sensor_left:.0f} "
            f"Right:{sensor_right:.0f}"
        )
        text = self.small_font.render(sensor_text, True, (0, 255, 255))
        self.screen.blit(text, (10, y_offset + 65))

        if self.sensor_layout in ('lidar_top_ground_ultrasonic', 'lidar_2d_floor'):
            ext_text = (
                f"Top LiDAR2D: min:{sensors.get('lidar_min', 0):.0f} "
                f"mean:{sensors.get('lidar_mean', 0):.0f} "
                f"std:{sensors.get('lidar_std', 0):.1f} "
                f"beams:{len(sensors.get('lidar_scan', []))}   "
                f"Ground US:{sensors.get('ground_ultrasonic', sensors.get('ground_front', 0)):.0f}"
            )
        else:
            ext_text = (
                f"Top180: L:{sensors.get('top_left', 0):.0f} "
                f"F:{sensors.get('top_front', sensors.get('top', 0)):.0f} "
                f"R:{sensors.get('top_right', 0):.0f}   "
                f"Floor180: L:{sensors.get('ground_left', 0):.0f} "
                f"F:{sensors.get('ground_front', sensors.get('ground', 0)):.0f} "
                f"R:{sensors.get('ground_right', 0):.0f}"
            )
        text = self.small_font.render(ext_text, True, (150, 255, 200))
        self.screen.blit(text, (10, y_offset + 85))

        # Battery
        battery_text = f"Battery: {state['battery']:.1f}%"
        battery_color = (0, 255, 0) if state['battery'] > 50 else (255, 165, 0)
        text = self.small_font.render(battery_text, True, battery_color)
        self.screen.blit(text, (10, y_offset + 110))

        # AI confidence (if in AI mode)
        if self.ai_mode:
            confidence = self.ai.get_decision_confidence(
                state,
                self.current_target,
            )
            conf_text = (
                f"AI: F:{confidence['forward']:.2f} "
                f"L:{confidence['left']:.2f} "
                f"R:{confidence['right']:.2f}"
            )
            text = self.small_font.render(conf_text, True, (255, 100, 255))
            self.screen.blit(text, (300, y_offset + 30))

        # Model metrics badge (only when model navigator is active)
        if self.navigator_type == 'model':
            metrics = getattr(self.ai, 'training_metrics', None) or {}
            val_acc = metrics.get('val_action_accuracy')
            train_loss = metrics.get('train_loss')
            if val_acc is not None:
                metric_text = f"Model val_acc: {val_acc:.3f}"
                if train_loss is not None:
                    metric_text += f"  loss: {train_loss:.3f}"
                text = self.small_font.render(metric_text, True, (120, 255, 120))
                self.screen.blit(text, (300, y_offset + 68))

        # Distance to active target
        tx, ty = self.current_target
        dist = ((self.robot.x - tx) ** 2 + (self.robot.y - ty) ** 2) ** 0.5
        goal_label = "Destination" if self.current_target == self.primary_goal else "Start"
        goal_text = f"Distance to {goal_label}: {dist:.0f}px"
        text = self.small_font.render(goal_text, True, (255, 215, 0))
        self.screen.blit(text, (300, y_offset + 50))

        # FPS
        fps_text = f"FPS: {int(self.clock.get_fps())}"
        text = self.small_font.render(fps_text, True, (150, 150, 150))
        self.screen.blit(text, (self.width - 80, y_offset + 10))

    def reset(self):
        """Reset simulation"""
        print("🔄 Resetting simulation...")
        start_x, start_y = self.cave.start_pos
        self.robot = RobotController(
            start_x,
            start_y,
            self.cave,
            sensor_layout=self.sensor_layout,
            robot_radius=self.robot_radius,
            speed_scale=self.robot_speed_scale,
        )
        if hasattr(self.ai, 'reset_runtime'):
            self.ai.reset_runtime()
        self.current_target = self.primary_goal
        self.returning_to_start = False
        self._target_reached_latch = False
        self.ai_mode = False

    def run(self):
        """Main simulation loop"""
        print("🚀 Starting simulation...")

        while self.running:
            self.handle_input()
            self.update()
            self.render()
            self.clock.tick(self.fps)

        if (
            self.navigator_type == 'rl'
            and self.rl_online_update
            and hasattr(self.ai, 'agent')
            and getattr(self.ai, 'agent', None) is not None
        ):
            try:
                self.ai.agent.save(self.ai.model_path, metadata={'saved_on_exit': True})
                stats = self.ai.get_online_stats() if hasattr(self.ai, 'get_online_stats') else {}
                print(f"💾 Saved RL model on exit: {self.ai.model_path} ({stats})")
            except Exception as e:
                print(f"⚠️ Failed to save RL model on exit: {e}")

        print("👋 Simulation ended")
        pygame.quit()
        sys.exit()


def parse_args():
    parser = argparse.ArgumentParser(description="Cave robot simulator")
    parser.add_argument(
        '--sensor-layout',
        choices=['dual_arc_180', 'top_ground_180', 'lidar_top_ground_ultrasonic', 'lidar_2d_floor'],
        default='lidar_top_ground_ultrasonic',
        help='Sensor profile: dual top/floor arcs, or hybrid top 2D LiDAR + ground ultrasonic (legacy lidar_2d_floor alias supported)',
    )
    parser.add_argument(
        '--robot-radius',
        type=int,
        default=DEFAULT_ROBOT_RADIUS_PX,
        help=(
            'Robot collision radius in pixels '
            f'(default: {DEFAULT_ROBOT_RADIUS_PX}, derived from 7x5x5 in at {DEFAULT_CM_PER_PIXEL:.2f} cm/px)'
        ),
    )
    parser.add_argument(
        '--robot-speed-scale',
        type=float,
        default=1.0,
        help='Robot speed multiplier (0.25 to 3.0). Can also be adjusted during simulation with [ and ] keys.',
    )
    parser.add_argument(
        '--person-height-cm',
        type=float,
        default=DEFAULT_PERSON_SIZE_CM[0],
        help='Person height in centimeters used to shape required cave clearance',
    )
    parser.add_argument(
        '--person-width-cm',
        type=float,
        default=DEFAULT_PERSON_SIZE_CM[1],
        help='Person body width in centimeters used to shape required cave clearance',
    )
    parser.add_argument(
        '--obstacle-level',
        choices=['light', 'medium', 'heavy', 'extra-heavy'],
        default='medium',
        help='Obstacle density profile for cave generation',
    )
    parser.add_argument(
        '--scatter-obstacles-in-path',
        dest='scatter_obstacles_in_path',
        action='store_true',
        help='Enable extra scattered blocker clusters near direct start->goal line (default: OFF)',
    )
    parser.add_argument(
        '--no-scatter-obstacles-in-path',
        dest='scatter_obstacles_in_path',
        action='store_false',
        help='Disable extra scattered blocker clusters near direct start->goal line',
    )
    parser.set_defaults(scatter_obstacles_in_path=False)
    parser.add_argument(
        '--navigator',
        choices=['planner', 'model', 'rl'],
        default='model',
        help='Navigation backend: planner (A* + rules), model (legacy neural policy), or rl (tabular Q-learning policy)',
    )
    parser.add_argument(
        '--retrain-model',
        action='store_true',
        help='When using --navigator model, delete existing model and train fresh',
    )
    parser.add_argument(
        '--rl-model-path',
        default='models/rl_q_table.pkl',
        help='When using --navigator rl, path to trained Q-table policy file',
    )
    parser.add_argument(
        '--rl-online-update',
        action='store_true',
        help='When using --navigator rl, enable online Q-learning updates during simulation (opt-in)',
    )
    parser.add_argument(
        '--rl-autosave-interval',
        type=int,
        default=300,
        help='When online RL update is enabled, save Q-table every N updates (default: 300)',
    )
    parser.add_argument(
        '--post-goal-action',
        choices=['reset', 'return', 'shuttle'],
        default='reset',
        help='What to do after reaching destination: reset (default), return to start once, or shuttle continuously',
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print("=" * 50)
    print("🤖 CAVE EXPLORER ROBOT SIMULATOR")
    print("   Hackathon Edition")
    print("=" * 50)
    print()

    sim = CaveSimulation(
        width=800,
        height=600,
        sensor_layout=args.sensor_layout,
        robot_radius=args.robot_radius,
        robot_speed_scale=args.robot_speed_scale,
        person_height_cm=args.person_height_cm,
        person_width_cm=args.person_width_cm,
        obstacle_level=args.obstacle_level,
        scatter_obstacles_in_path=args.scatter_obstacles_in_path,
        navigator=args.navigator,
        retrain_model=args.retrain_model,
        rl_model_path=args.rl_model_path,
        rl_online_update=args.rl_online_update,
        rl_autosave_interval=args.rl_autosave_interval,
        post_goal_action=args.post_goal_action,
    )
    sim.run()
