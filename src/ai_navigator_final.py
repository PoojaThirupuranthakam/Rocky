"""
FINAL WORKING VERSION - Balanced speed and safety
"""

import random

import numpy as np
import pickle
import os

INCH_TO_CM = 2.54


class SimpleNeuralNetwork:
    def __init__(self, input_size=10, hidden_size=8, output_size=4):
        self.weights1 = np.random.randn(input_size, hidden_size) * 0.5
        self.bias1 = np.zeros((1, hidden_size))
        self.weights2 = np.random.randn(hidden_size, output_size) * 0.5
        self.bias2 = np.zeros((1, output_size))
    
    def sigmoid(self, x):
        return 1 / (1 + np.exp(-np.clip(x, -500, 500)))
    
    def forward(self, x):
        hidden = self.sigmoid(np.dot(x, self.weights1) + self.bias1)
        output = self.sigmoid(np.dot(hidden, self.weights2) + self.bias2)
        return hidden, output

    def predict(self, x):
        _, output = self.forward(x)
        return output

    def train(
        self,
        x_train,
        y_train,
        epochs=220,
        lr=0.03,
        batch_size=256,
        verbose=True,
        sample_weights=None,
    ):
        n = x_train.shape[0]
        if n == 0:
            return {"loss": 0.0}

        if sample_weights is None:
            sample_weights = np.ones((n,), dtype=np.float32)
        else:
            sample_weights = np.asarray(sample_weights, dtype=np.float32)
            if sample_weights.shape[0] != n:
                raise ValueError("sample_weights length must match x_train rows")
            sample_weights = np.clip(sample_weights, 1e-4, None)

        for epoch in range(epochs):
            indices = np.random.permutation(n)
            x_train = x_train[indices]
            y_train = y_train[indices]
            sample_weights = sample_weights[indices]

            for start in range(0, n, batch_size):
                end = min(start + batch_size, n)
                xb = x_train[start:end]
                yb = y_train[start:end]
                wb = sample_weights[start:end]

                hidden, pred = self.forward(xb)

                # BCE with sigmoid output => dZ2 = pred - y
                norm = np.sum(wb) + 1e-8
                weight_scale = (wb.reshape(-1, 1) / norm) * xb.shape[0]
                dz2 = (pred - yb) * weight_scale
                dw2 = (hidden.T @ dz2) / xb.shape[0]
                db2 = np.mean(dz2, axis=0, keepdims=True)

                dz1 = (dz2 @ self.weights2.T) * hidden * (1 - hidden)
                dw1 = (xb.T @ dz1) / xb.shape[0]
                db1 = np.mean(dz1, axis=0, keepdims=True)

                self.weights2 -= lr * dw2
                self.bias2 -= lr * db2
                self.weights1 -= lr * dw1
                self.bias1 -= lr * db1

            if verbose and (epoch + 1) % 50 == 0:
                p = np.clip(self.predict(x_train), 1e-7, 1 - 1e-7)
                loss_rows = -np.mean(y_train * np.log(p) + (1 - y_train) * np.log(1 - p), axis=1)
                loss = float(np.sum(loss_rows * sample_weights) / (np.sum(sample_weights) + 1e-8))
                print(f"   epoch {epoch + 1}/{epochs} loss={loss:.4f}")

        p = np.clip(self.predict(x_train), 1e-7, 1 - 1e-7)
        final_rows = -np.mean(y_train * np.log(p) + (1 - y_train) * np.log(1 - p), axis=1)
        final_loss = float(np.sum(final_rows * sample_weights) / (np.sum(sample_weights) + 1e-8))
        return {"loss": float(final_loss)}
    
    def save(self, filepath, metadata=None):
        payload = {
            'w1': self.weights1,
            'b1': self.bias1,
            'w2': self.weights2,
            'b2': self.bias2,
            'metadata': metadata or {},
        }
        with open(filepath, 'wb') as f:
            pickle.dump(payload, f)
    
    def load(self, filepath):
        with open(filepath, 'rb') as f:
            d = pickle.load(f)
        self.weights1, self.bias1, self.weights2, self.bias2 = d['w1'], d['b1'], d['w2'], d['b2']
        return d.get('metadata', {})


class AINavigator:
    """Balanced navigation - fast but safe"""
    
    def __init__(
        self,
        model_path='models/navigation_model.pkl',
        cave_env=None,
        training_config=None,
    ):
        self.training_config = {
            'samples': 16000,
            'epochs': 220,
            'lr': 0.03,
            'batch_size': 256,
            'teacher': 'planner_rollout',
            'parity_mode': False,
            'planner_anchor_mode': True,
            'rollout_max_steps': 240,
            'collision_weight': 1.8,
            'near_wall_weight': 1.4,
            'hard_ratio': 0.45,
            'forward_progress_weight': 1.35,
            'turn_loop_penalty': 0.70,
            'forward_clearance_px': 52.0,
            'forward_heading_align_rad': 0.55,
            'cm_per_pixel': 0.5,
            'robot_length_cm': 7.0 * INCH_TO_CM,
            'robot_width_cm': 5.0 * INCH_TO_CM,
            'robot_height_cm': 5.0 * INCH_TO_CM,
            'person_width_min_cm': 28.0,
            'person_width_max_cm': 56.0,
            'person_height_min_cm': 100.0,
            'person_height_max_cm': 190.0,
            'feature_dim': 10,
            'hidden_size': 8,
            'feature_schema': 'lidar2d_top_ground_ultrasonic_v2',
            'sensor_layout': 'lidar_top_ground_ultrasonic',
            **(training_config or {}),
        }

        self.feature_dim = int(self.training_config.get('feature_dim', 9))
        hidden_size = int(self.training_config.get('hidden_size', 8))

        self.model_path = model_path
        self.model = SimpleNeuralNetwork(input_size=self.feature_dim, hidden_size=hidden_size)
        self.training_metrics = None
        self.last_model_scores = {'forward': 0.5, 'backward': 0.1, 'left': 0.2, 'right': 0.2}
        self.cave_env = cave_env
        self.fallback_planner = None
        if self.cave_env is not None:
            from ai_navigator import AINavigator as PlannerNavigator
            self.fallback_planner = PlannerNavigator(self.cave_env)

        self.bad_event_streak = 0
        self.fallback_frames = 0
        self.recovery_mode = False
        self.recovery_frames = 0
        self.last_pos = (0, 0)
        self.stuck_count = 0
        self.no_progress_frames = 0
        self.last_goal_distance = None
        self.action_commitment = 0
        self.committed_action = None
        self.progress_boost_frames = 0
        self.hard_block_frames = 0

        if os.path.exists(model_path) and self._is_saved_model_compatible(model_path):
            print(f"📦 Loading AI model")
            self.training_metrics = self.model.load(model_path)
            if self.training_metrics:
                val_acc = self.training_metrics.get('val_action_accuracy')
                if val_acc is not None:
                    print(f"   validation action accuracy: {val_acc:.3f}")
        else:
            if os.path.exists(model_path):
                print("♻️ Existing model is incompatible with hybrid LiDAR+ground-ultrasonic feature schema; retraining...")
            print("🧠 Training...")
            self._train()

    def _is_saved_model_compatible(self, filepath):
        try:
            with open(filepath, 'rb') as f:
                payload = pickle.load(f)
            w1 = payload.get('w1')
            if w1 is None or len(w1.shape) != 2:
                return False

            metadata = payload.get('metadata', {}) or {}
            schema = str(metadata.get('feature_schema', ''))
            dim_ok = int(w1.shape[0]) == int(self.feature_dim)
            schema_ok = schema == str(self.training_config.get('feature_schema'))
            return dim_ok and schema_ok
        except Exception:
            return False

    def reset_runtime(self):
        """Reset non-persistent runtime state between episodes/resets."""
        self.bad_event_streak = 0
        self.fallback_frames = 0
        self.recovery_mode = False
        self.recovery_frames = 0
        self.last_pos = (0, 0)
        self.stuck_count = 0
        self.no_progress_frames = 0
        self.last_goal_distance = None
        self.action_commitment = 0
        self.committed_action = None
        self.progress_boost_frames = 0
        self.hard_block_frames = 0

    def _extract_directional_clearance(self, sensors):
        """Derive front/left/right clearances from top LiDAR + ground ultrasonic (or legacy top+ground sectors)."""
        ground_front = sensors.get('ground_ultrasonic', sensors.get('ground_front', sensors.get('ground')))
        cm_per_pixel = float(getattr(self.cave_env, 'cm_per_pixel', 0.5) or 0.5)
        ground_guard_px = max(1.0, 22.86 / cm_per_pixel)  # ~0.75 ft

        lidar_front = sensors.get('lidar_front')
        lidar_left = sensors.get('lidar_left')
        lidar_right = sensors.get('lidar_right')
        if lidar_front is not None and lidar_left is not None and lidar_right is not None:
            front = float(lidar_front)
            left = float(lidar_left)
            right = float(lidar_right)
            if ground_front is not None:
                g_front = float(ground_front)
                if g_front < ground_guard_px:
                    front = min(front, g_front)
            return front, left, right

        lidar_scan = sensors.get('lidar_scan')
        if isinstance(lidar_scan, (list, tuple)) and len(lidar_scan) >= 3:
            n = len(lidar_scan)
            rel_angles = [(-np.pi + (2.0 * np.pi * i) / n) for i in range(n)]

            def angle_wrap(rad):
                return (rad + np.pi) % (2.0 * np.pi) - np.pi

            sector_half = np.pi / 6.0
            front_vals = [float(v) for v, rel in zip(lidar_scan, rel_angles) if abs(angle_wrap(rel)) <= sector_half]
            left_vals = [float(v) for v, rel in zip(lidar_scan, rel_angles) if abs(angle_wrap(rel - np.pi / 4.0)) <= sector_half]
            right_vals = [float(v) for v, rel in zip(lidar_scan, rel_angles) if abs(angle_wrap(rel + np.pi / 4.0)) <= sector_half]
            if not front_vals:
                front_vals = [float(v) for v in lidar_scan]
            front = min(front_vals)
            left = min(left_vals) if left_vals else front
            right = min(right_vals) if right_vals else front
            if ground_front is not None:
                g_front = float(ground_front)
                if g_front < ground_guard_px:
                    front = min(front, g_front)
            return front, left, right

        top_front = float(sensors.get('top_front', sensors.get('top', 0.0)))
        top_left = float(sensors.get('top_left', top_front))
        top_right = float(sensors.get('top_right', top_front))

        ground_front = float(sensors.get('ground_front', sensors.get('ground', top_front)))
        ground_left = float(sensors.get('ground_left', sensors.get('ground', top_left)))
        ground_right = float(sensors.get('ground_right', sensors.get('ground', top_right)))

        front = min(top_front, ground_front)
        left = min(top_left, ground_left)
        right = min(top_right, ground_right)
        return front, left, right

    def _extract_ground_ultrasonic(self, sensors, front_fallback):
        ground = sensors.get('ground_ultrasonic', sensors.get('ground_front', sensors.get('ground', front_fallback)))
        try:
            return float(ground)
        except (TypeError, ValueError):
            return float(front_fallback)

    def _extract_lidar_stats(self, sensors, front, left, right):
        lidar_scan = sensors.get('lidar_scan')
        if isinstance(lidar_scan, (list, tuple)) and len(lidar_scan) > 0:
            arr = np.asarray(lidar_scan, dtype=np.float32)
            return float(np.min(arr)), float(np.mean(arr)), float(np.std(arr))

        lidar_min = float(sensors.get('lidar_min', min(front, left, right)))
        lidar_mean = float(sensors.get('lidar_mean', (front + left + right) / 3.0))
        lidar_std = float(sensors.get('lidar_std', np.std([front, left, right])))
        return lidar_min, lidar_mean, lidar_std

    def _build_features(self, front, left, right, lidar_min, lidar_mean, lidar_std, ground_ultrasonic, angle_diff, collision, speed=0.0):
        return np.array([
            min(front, 150) / 150.0,
            min(left, 150) / 150.0,
            min(right, 150) / 150.0,
            min(lidar_min, 150) / 150.0,
            min(lidar_mean, 150) / 150.0,
            min(lidar_std, 60) / 60.0,
            min(ground_ultrasonic, 120) / 120.0,
            angle_diff / np.pi,
            1.0 if collision else 0.0,
            np.clip(abs(speed) / 2.0, 0.0, 1.0),
        ], dtype=np.float32)

    def _expert_action_vector(self, front, left, right, angle_diff, collision):
        """Teacher policy used for offline supervised training."""
        # Output order: [forward, backward, left, right]
        if collision or front < 20:
            # emergency: back up and turn toward space
            return np.array([0, 1, 1 if left > right else 0, 1 if right > left else 0], dtype=np.float32)

        if front < 40:
            return np.array([0, 0, 1 if left > right else 0, 1 if right > left else 0], dtype=np.float32)

        # Side corridor preference if one side is clearly more open
        if max(left, right) > front + 12:
            return np.array([1, 0, 1 if left > right else 0, 1 if right > left else 0], dtype=np.float32)

        need_turn = abs(angle_diff) > 0.2
        return np.array([
            1,
            0,
            1 if need_turn and angle_diff > 0 else 0,
            1 if need_turn and angle_diff < 0 else 0,
        ], dtype=np.float32)

    def _action_dict_to_vector(self, action):
        return np.array([
            1.0 if action.get('forward', False) else 0.0,
            1.0 if action.get('backward', False) else 0.0,
            1.0 if action.get('left', False) else 0.0,
            1.0 if action.get('right', False) else 0.0,
        ], dtype=np.float32)

    def _generate_dataset(self, samples=16000):
        x = np.zeros((samples, self.feature_dim), dtype=np.float32)
        y = np.zeros((samples, 4), dtype=np.float32)
        w = np.ones((samples,), dtype=np.float32)

        collision_weight = float(self.training_config.get('collision_weight', 1.8))
        near_wall_weight = float(self.training_config.get('near_wall_weight', 1.4))
        forward_progress_weight = float(self.training_config.get('forward_progress_weight', 1.35))
        turn_loop_penalty = float(self.training_config.get('turn_loop_penalty', 0.70))
        forward_clearance_px = float(self.training_config.get('forward_clearance_px', 52.0))
        forward_heading_align_rad = float(self.training_config.get('forward_heading_align_rad', 0.55))

        for i in range(samples):
            front = np.random.uniform(0, 150)
            left = np.random.uniform(0, 150)
            right = np.random.uniform(0, 150)
            angle_diff = np.random.uniform(-np.pi, np.pi)
            collision = np.random.rand() < 0.04
            lidar_min = max(0.0, min(front, left, right) - np.random.uniform(0.0, 5.0))
            lidar_mean = np.clip((front + left + right) / 3.0 + np.random.uniform(-5.0, 5.0), 0.0, 150.0)
            lidar_std = np.clip(np.std([front, left, right]) + np.random.uniform(0.0, 4.0), 0.0, 60.0)
            ground_ultrasonic = np.clip(front + np.random.uniform(-22.0, 6.0), 0.0, 120.0)

            speed = np.random.uniform(0, 2.0)
            x[i] = self._build_features(
                front,
                left,
                right,
                lidar_min,
                lidar_mean,
                lidar_std,
                ground_ultrasonic,
                angle_diff,
                collision,
                speed,
            )
            action_vec = self._expert_action_vector(front, left, right, angle_diff, collision)

            clear_progress_state = (
                not collision
                and front > forward_clearance_px
                and min(left, right) > (0.72 * forward_clearance_px)
                and ground_ultrasonic > (0.82 * forward_clearance_px)
            )
            heading_aligned = abs(angle_diff) < forward_heading_align_rad

            # Penalize stationary turn loops in moderately open corridors.
            turning_in_place = (
                action_vec[0] < 0.5
                and action_vec[1] < 0.5
                and (action_vec[2] > 0.5 or action_vec[3] > 0.5)
            )
            if clear_progress_state and turning_in_place and heading_aligned:
                action_vec = np.array([1, 0, 0, 0], dtype=np.float32)

            y[i] = action_vec

            sample_w = 1.0
            if collision:
                sample_w *= collision_weight
            if front < 40 or min(left, right) < 30:
                sample_w *= near_wall_weight

            if clear_progress_state and heading_aligned:
                sample_w *= forward_progress_weight

            if turning_in_place and clear_progress_state:
                sample_w *= turn_loop_penalty

            w[i] = sample_w

        split = int(samples * 0.8)
        return x[:split], y[:split], x[split:], y[split:], w[:split], w[split:]

    def _generate_planner_rollout_dataset(self, samples=16000, rollout_max_steps=280):
        """Generate imitation-learning dataset from planner trajectories."""
        from cave_environment import CaveEnvironment
        from robot_controller import RobotController
        from ai_navigator import AINavigator as PlannerNavigator

        feat_rows = []
        label_rows = []
        weight_rows = []

        target = int(samples)
        episodes = max(24, target // 220)
        hard_ratio = float(self.training_config.get('hard_ratio', 0.45))
        collision_weight = float(self.training_config.get('collision_weight', 1.8))
        near_wall_weight = float(self.training_config.get('near_wall_weight', 1.4))
        forward_progress_weight = float(self.training_config.get('forward_progress_weight', 1.35))
        turn_loop_penalty = float(self.training_config.get('turn_loop_penalty', 0.70))
        forward_clearance_px = float(self.training_config.get('forward_clearance_px', 52.0))
        forward_heading_align_rad = float(self.training_config.get('forward_heading_align_rad', 0.55))
        cm_per_pixel = float(self.training_config.get('cm_per_pixel', 0.5))
        robot_width_cm = float(self.training_config.get('robot_width_cm', 5.0 * INCH_TO_CM))
        person_width_min_cm = float(self.training_config.get('person_width_min_cm', 28.0))
        person_width_max_cm = float(self.training_config.get('person_width_max_cm', 56.0))
        person_height_min_cm = float(self.training_config.get('person_height_min_cm', 100.0))
        person_height_max_cm = float(self.training_config.get('person_height_max_cm', 190.0))

        base_radius = max(4, int(round((robot_width_cm / 2.0) / max(0.1, cm_per_pixel))))

        for _ in range(episodes):
            if len(feat_rows) >= target:
                break

            if random.random() < hard_ratio:
                complexity = random.uniform(0.12, 0.16)
                radius = max(4, base_radius + random.choice([0, 1]))
            else:
                complexity = random.uniform(0.09, 0.13)
                radius = max(4, base_radius + random.choice([-1, 0]))

            person_width_cm = random.uniform(person_width_min_cm, person_width_max_cm)
            person_height_cm = random.uniform(person_height_min_cm, person_height_max_cm)

            cave = CaveEnvironment(
                800,
                600,
                complexity=complexity,
                robot_radius=radius,
                person_width_cm=person_width_cm,
                person_height_cm=person_height_cm,
                cm_per_pixel=cm_per_pixel,
            )
            planner = PlannerNavigator(cave)
            sx, sy = cave.start_pos
            robot = RobotController(
                sx,
                sy,
                cave,
                sensor_layout=str(self.training_config.get('sensor_layout', 'lidar_top_ground_ultrasonic')),
                robot_radius=radius,
            )
            goal = cave.goal_pos

            for _step in range(int(rollout_max_steps)):
                state = robot.get_state()
                sensors = state['sensors']
                front, left, right = self._extract_directional_clearance(sensors)
                lidar_min, lidar_mean, lidar_std = self._extract_lidar_stats(sensors, front, left, right)
                ground_ultrasonic = self._extract_ground_ultrasonic(sensors, front)
                dx = goal[0] - state['position'][0]
                dy = goal[1] - state['position'][1]
                goal_angle = np.arctan2(dy, dx)
                angle_diff = (goal_angle - state['angle'] + np.pi) % (2 * np.pi) - np.pi

                action = planner.get_action(state, goal)
                action_vec = self._action_dict_to_vector(action)

                clear_progress_state = (
                    front > forward_clearance_px
                    and min(left, right) > (0.72 * forward_clearance_px)
                    and ground_ultrasonic > (0.82 * forward_clearance_px)
                    and not state.get('collision', False)
                )
                heading_aligned = abs(angle_diff) < forward_heading_align_rad

                turning_in_place = (
                    action_vec[0] < 0.5
                    and action_vec[1] < 0.5
                    and (action_vec[2] > 0.5 or action_vec[3] > 0.5)
                )
                if clear_progress_state and turning_in_place and heading_aligned:
                    action = {
                        'forward': True,
                        'backward': False,
                        'left': False,
                        'right': False,
                    }
                    action_vec = np.array([1, 0, 0, 0], dtype=np.float32)

                feat_rows.append(
                    self._build_features(
                        front,
                        left,
                        right,
                        lidar_min,
                        lidar_mean,
                        lidar_std,
                        ground_ultrasonic,
                        angle_diff,
                        state.get('collision', False),
                        state.get('speed', 0.0),
                    )
                )
                label_rows.append(action_vec)

                sample_w = 1.0
                if state.get('collision', False):
                    sample_w *= collision_weight
                if front < 40 or min(left, right) < 30:
                    sample_w *= near_wall_weight

                if clear_progress_state and heading_aligned:
                    sample_w *= forward_progress_weight

                if turning_in_place and clear_progress_state:
                    sample_w *= turn_loop_penalty

                weight_rows.append(sample_w)

                if action.get('forward'):
                    robot.move_forward()
                if action.get('backward'):
                    robot.move_backward()
                if action.get('left'):
                    robot.turn_left()
                if action.get('right'):
                    robot.turn_right()
                robot.update()

                if np.hypot(robot.x - goal[0], robot.y - goal[1]) < 30:
                    break
                if len(feat_rows) >= target:
                    break

        # Fallback fill to ensure requested sample count.
        if len(feat_rows) < target:
            missing = target - len(feat_rows)
            x_rules, y_rules, _, _, w_rules, _ = self._generate_dataset(samples=max(2000, missing))
            feat_rows.extend(list(x_rules[:missing]))
            label_rows.extend(list(y_rules[:missing]))
            weight_rows.extend(list(w_rules[:missing]))

        x = np.array(feat_rows[:target], dtype=np.float32)
        y = np.array(label_rows[:target], dtype=np.float32)
        w = np.array(weight_rows[:target], dtype=np.float32)
        split = int(target * 0.8)
        return x[:split], y[:split], x[split:], y[split:], w[:split], w[split:]

    def _evaluate(self, x_val, y_val):
        pred = self.model.predict(x_val)
        pred_action = np.argmax(pred, axis=1)
        true_action = np.argmax(y_val, axis=1)
        action_acc = float(np.mean(pred_action == true_action))

        pred_bin = (pred >= 0.5).astype(np.float32)
        bit_acc = float(np.mean(pred_bin == y_val))
        return action_acc, bit_acc
    
    def _train(self):
        cfg = self.training_config
        teacher = str(cfg.get('teacher', 'rules'))
        if teacher == 'planner_rollout':
            x_train, y_train, x_val, y_val, w_train, w_val = self._generate_planner_rollout_dataset(
                samples=int(cfg['samples']),
                rollout_max_steps=int(cfg.get('rollout_max_steps', 280)),
            )
        else:
            x_train, y_train, x_val, y_val, w_train, w_val = self._generate_dataset(samples=int(cfg['samples']))
        train_stats = self.model.train(
            x_train,
            y_train,
            epochs=int(cfg['epochs']),
            lr=float(cfg['lr']),
            batch_size=int(cfg['batch_size']),
            verbose=True,
            sample_weights=w_train,
        )
        val_action_acc, val_bit_acc = self._evaluate(x_val, y_val)

        self.training_metrics = {
            'train_loss': train_stats['loss'],
            'val_action_accuracy': val_action_acc,
            'val_bit_accuracy': val_bit_acc,
            'samples_train': int(x_train.shape[0]),
            'samples_val': int(x_val.shape[0]),
            'epochs': int(cfg['epochs']),
            'lr': float(cfg['lr']),
            'batch_size': int(cfg['batch_size']),
            'samples_total': int(cfg['samples']),
            'teacher': teacher,
            'collision_weight': float(cfg.get('collision_weight', 1.8)),
            'near_wall_weight': float(cfg.get('near_wall_weight', 1.4)),
            'hard_ratio': float(cfg.get('hard_ratio', 0.45)),
            'forward_progress_weight': float(cfg.get('forward_progress_weight', 1.35)),
            'turn_loop_penalty': float(cfg.get('turn_loop_penalty', 0.70)),
            'forward_clearance_px': float(cfg.get('forward_clearance_px', 52.0)),
            'forward_heading_align_rad': float(cfg.get('forward_heading_align_rad', 0.55)),
            'cm_per_pixel': float(cfg.get('cm_per_pixel', 0.5)),
            'robot_length_cm': float(cfg.get('robot_length_cm', 7.0 * INCH_TO_CM)),
            'robot_width_cm': float(cfg.get('robot_width_cm', 5.0 * INCH_TO_CM)),
            'robot_height_cm': float(cfg.get('robot_height_cm', 5.0 * INCH_TO_CM)),
            'person_width_min_cm': float(cfg.get('person_width_min_cm', 28.0)),
            'person_width_max_cm': float(cfg.get('person_width_max_cm', 56.0)),
            'person_height_min_cm': float(cfg.get('person_height_min_cm', 100.0)),
            'person_height_max_cm': float(cfg.get('person_height_max_cm', 190.0)),
            'feature_dim': int(self.feature_dim),
            'feature_schema': str(cfg.get('feature_schema', 'lidar2d_top_ground_ultrasonic_v2')),
            'sensor_layout': str(cfg.get('sensor_layout', 'lidar_top_ground_ultrasonic')),
        }

        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
        self.model.save(self.model_path, metadata=self.training_metrics)
        print(
            "✅ Ready! "
            f"val_action_acc={val_action_acc:.3f}, "
            f"val_bit_acc={val_bit_acc:.3f}"
        )

    def _model_action(self, front, left, right, lidar_min, lidar_mean, lidar_std, ground_ultrasonic, angle_diff, collision, speed):
        x = self._build_features(
            front,
            left,
            right,
            lidar_min,
            lidar_mean,
            lidar_std,
            ground_ultrasonic,
            angle_diff,
            collision,
            speed,
        ).reshape(1, -1)
        scores = self.model.predict(x)[0]
        self.last_model_scores = {
            'forward': float(scores[0]),
            'backward': float(scores[1]),
            'left': float(scores[2]),
            'right': float(scores[3]),
        }

        action = {
            'forward': scores[0] > 0.43,
            'backward': scores[1] > 0.6,
            'left': scores[2] > 0.5,
            'right': scores[3] > 0.5,
        }

        # Resolve contradictory outputs
        if action['left'] and action['right']:
            action['left'] = left > right
            action['right'] = right > left
        if action['forward'] and action['backward']:
            action['backward'] = False

        return action

    def _safe_turn_action(self, left, right, backward=False):
        turn_left = left > right
        return {
            'forward': False,
            'backward': backward,
            'left': turn_left,
            'right': not turn_left,
        }

    def _go_around_action(self, front, left, right, angle_diff=0.0):
        """Small offset maneuver around front obstacle, then quickly re-align to goal."""
        turn_left = left >= right
        side_clear = left if turn_left else right
        forward_ok = side_clear > 18 and front > 10

        # If front is already opening up, prioritize steering back to destination.
        if front > 44 and abs(angle_diff) < 0.26:
            return {
                'forward': True,
                'backward': False,
                'left': False,
                'right': False,
            }

        goal_left = angle_diff > 0.10 and left > 14
        goal_right = angle_diff < -0.10 and right > 14

        if front > 24 and (goal_left or goal_right):
            return {
                'forward': forward_ok,
                'backward': False,
                'left': goal_left,
                'right': goal_right,
            }

        return {
            'forward': forward_ok,
            'backward': False,
            'left': turn_left,
            'right': not turn_left,
        }

    def _hybrid_action(self, planner_action, model_action, front, left, right, angle_diff):
        """Planner-led arbitration with model only as local refinement."""
        action = dict(planner_action)

        # Tight-space override: do not trust learned forward near obstacles.
        if front < 24:
            if max(left, right) > 24:
                return self._go_around_action(front, left, right, angle_diff)
            return self._safe_turn_action(left, right, backward=True)

        if front < 34:
            if max(left, right) > 24:
                return self._go_around_action(front, left, right, angle_diff)
            return self._safe_turn_action(left, right, backward=False)

        # Safety-critical corridor: use planner commands directly.
        if front < 50 or min(left, right) < 28:
            return dict(planner_action)

        # If planner already has a clean forward move and heading is near-correct,
        # keep it straight to avoid unnecessary model-induced weaving.
        if planner_action.get('forward', False) and front > 72 and abs(angle_diff) < 0.14:
            return {
                'forward': True,
                'backward': False,
                'left': False,
                'right': False,
            }

        # In medium clearance, preserve planner intent and only enforce steering.
        if front < 55:
            action['backward'] = False
            if not action['left'] and not action['right']:
                action['left'] = left > right
                action['right'] = right > left
            action['forward'] = action['forward'] and front > 36
            return action

        # Open space: planner stays primary, but model can refine turn choice.
        model_turn_gap = abs(
            self.last_model_scores.get('left', 0.0) - self.last_model_scores.get('right', 0.0)
        )
        if model_action.get('left') != model_action.get('right') and model_turn_gap > 0.28:
            if action.get('forward', False):
                action['left'] = model_action['left']
                action['right'] = model_action['right']

        # If planner is rotating in place but it is safe and goal is nearly ahead,
        # bias toward forward to avoid idle spinning.
        if (
            not action.get('forward', False)
            and not action.get('backward', False)
            and front > 62
            and abs(angle_diff) < 0.52
        ):
            action['forward'] = True

        # If still no turn command and heading is off, add deterministic turn.
        if not action['left'] and not action['right'] and abs(angle_diff) > 0.35:
            action['left'] = angle_diff > 0
            action['right'] = angle_diff < 0

        # Resolve contradictions.
        if action['left'] and action['right']:
            action['left'] = left > right
            action['right'] = right > left
        if action['forward'] and action['backward']:
            action['backward'] = False

        return action
    
    def get_action(self, robot_state, goal_pos):
        """Balanced navigation with commitment to avoid oscillation"""
        sensors = robot_state['sensors']
        front, left, right = self._extract_directional_clearance(sensors)
        lidar_min, lidar_mean, lidar_std = self._extract_lidar_stats(sensors, front, left, right)
        ground_ultrasonic = self._extract_ground_ultrasonic(sensors, front)

        pos = robot_state['position']
        angle = robot_state['angle']
        speed = robot_state.get('speed', 0.0)
        collision = robot_state['collision']
        dx = goal_pos[0] - pos[0]
        dy = goal_pos[1] - pos[1]
        goal_angle = np.arctan2(dy, dx)
        angle_diff = (goal_angle - angle + np.pi) % (2 * np.pi) - np.pi

        cm_per_pixel = float(getattr(self.cave_env, 'cm_per_pixel', 0.5) or 0.5) if self.cave_env is not None else 0.5
        one_foot_px = max(1.0, 30.48 / cm_per_pixel)

        # Parity-grade mode: mirror planner behavior for consistent cross-seed outcomes.
        if self.fallback_planner is not None and bool(self.training_config.get('parity_mode', True)):
            # Keep model confidence signals updated for HUD/debugging even in parity mode.
            self._model_action(front, left, right, lidar_min, lidar_mean, lidar_std, ground_ultrasonic, angle_diff, collision, speed)
            return self.fallback_planner.get_action(robot_state, goal_pos)

        # Planner-anchored non-parity mode: planner controls execution,
        # model still evaluates confidence for observability.
        if self.fallback_planner is not None and bool(self.training_config.get('planner_anchor_mode', True)):
            self._model_action(front, left, right, lidar_min, lidar_mean, lidar_std, ground_ultrasonic, angle_diff, collision, speed)
            return self.fallback_planner.get_action(robot_state, goal_pos)

        # Detect if stuck
        moved = abs(pos[0] - self.last_pos[0]) + abs(pos[1] - self.last_pos[1])
        if moved < 0.18:
            self.stuck_count += 1
        else:
            self.stuck_count = max(0, self.stuck_count - 2)
        self.last_pos = pos

        # Goal-progress watchdog: catch long spinning without meaningful progress.
        goal_dist = float(np.hypot(goal_pos[0] - pos[0], goal_pos[1] - pos[1]))
        if self.last_goal_distance is not None:
            if goal_dist > self.last_goal_distance - 0.12:
                self.no_progress_frames += 1
            else:
                self.no_progress_frames = max(0, self.no_progress_frames - 2)
        self.last_goal_distance = goal_dist

        hard_stop_px = max(12.0, one_foot_px * 0.34)
        side_guard_px = one_foot_px * 0.78
        side_hard_px = max(9.0, hard_stop_px * 0.82)
        ground_guard_px = one_foot_px * 0.82
        ground_hard_px = max(9.0, hard_stop_px * 0.78)
        min_clearance = min(front, left, right, ground_ultrasonic)

        # Two-tier envelope: 1-foot preferred buffer + smaller hard-stop gate.
        hard_envelope = (
            front < hard_stop_px
            or min(left, right) < side_hard_px
            or ground_ultrasonic < ground_hard_px
        )

        if (
            self.fallback_planner is not None
            and self.no_progress_frames > 72
            and goal_dist > 120
            and self.fallback_frames == 0
        ):
            self.fallback_frames = 140
            self.no_progress_frames = max(0, self.no_progress_frames - 24)

        if self.fallback_frames > 0 and self.fallback_planner is not None:
            self.fallback_frames -= 1
            return self.fallback_planner.get_action(robot_state, goal_pos)

        if hard_envelope:
            self.hard_block_frames += 1
        else:
            self.hard_block_frames = max(0, self.hard_block_frames - 3)

        if hard_envelope:
            # Anti-stall escape in borderline-clear corridors.
            if (
                (self.no_progress_frames > 48 or self.stuck_count > 10)
                and not collision
                and front > (one_foot_px * 0.86)
                and max(left, right) > (one_foot_px * 0.72)
                and ground_ultrasonic > (one_foot_px * 0.78)
                and abs(angle_diff) < 0.72
            ):
                self.no_progress_frames = max(0, self.no_progress_frames - 20)
                self.stuck_count = max(0, self.stuck_count - 6)
                turn_left = left >= right
                return {
                    'forward': True,
                    'backward': False,
                    'left': turn_left,
                    'right': not turn_left,
                }

            if self.hard_block_frames > 26:
                self.hard_block_frames = 0
                self.no_progress_frames = max(0, self.no_progress_frames - 14)
                if self.fallback_planner is not None:
                    self.fallback_frames = max(self.fallback_frames, 120)
                turn_left = left >= right
                return {
                    'forward': False,
                    'backward': True,
                    'left': turn_left,
                    'right': not turn_left,
                }

            turn_left = left >= right
            boxed_side = left < side_guard_px and right < side_guard_px
            return {
                'forward': False,
                'backward': bool(min_clearance < (0.88 * one_foot_px) or boxed_side),
                'left': turn_left,
                'right': not turn_left,
            }

        # Planner-guided progress boost (distillation at inference time):
        # if safe but progress is flat, temporarily follow planner guidance
        # with forward bias to break local minima.
        if self.fallback_planner is not None and not collision:
            if (
                self.no_progress_frames > 42
                and front > (one_foot_px * 0.95)
                and min(left, right) > (one_foot_px * 0.72)
                and ground_ultrasonic > (one_foot_px * 0.82)
                and abs(angle_diff) < 0.92
            ):
                self.progress_boost_frames = max(self.progress_boost_frames, 28)

            if self.progress_boost_frames > 0:
                self.progress_boost_frames -= 1
                boost_action = self.fallback_planner.get_action(robot_state, goal_pos)

                if (
                    front > (one_foot_px * 0.90)
                    and ground_ultrasonic > (one_foot_px * 0.80)
                    and abs(angle_diff) < 0.95
                ):
                    boost_action['forward'] = True
                    boost_action['backward'] = False

                return boost_action
        
        # RECOVERY MODE - After collision or stuck
        long_progress_plateau = self.no_progress_frames > 56 and goal_dist > 120
        if collision or self.stuck_count > 16 or long_progress_plateau:
            self.recovery_mode = True
            self.recovery_frames = 0
            self.stuck_count = 0
            self.no_progress_frames = 0
            self.action_commitment = 0  # Cancel any commitment
            self.bad_event_streak += 1
        else:
            self.bad_event_streak = max(0, self.bad_event_streak - 1)

        # If repeated bad events happen, hand control to planner for a while.
        if self.fallback_planner is not None and self.bad_event_streak >= 2:
            self.fallback_frames = 220
            self.bad_event_streak = 0

        if self.fallback_frames > 0 and self.fallback_planner is not None:
            self.fallback_frames -= 1
            return self.fallback_planner.get_action(robot_state, goal_pos)
        
        if self.recovery_mode:
            self.recovery_frames += 1
            # Phase 1: Strong backup
            if self.recovery_frames <= 18:
                return {'forward': False, 'backward': True, 'left': False, 'right': False}
            # Phase 2: Turn toward open space
            elif self.recovery_frames <= 44:
                # Turn toward whichever side has more space
                return {'forward': False, 'backward': False, 
                       'left': left > right, 'right': right > left}
            # Phase 3: Exit recovery
            else:
                self.recovery_mode = False
                self.recovery_frames = 0
        
        # Near-goal snap: in the final stretch, avoid extra weaving and just close out.
        if goal_dist < 78 and front > 30 and abs(angle_diff) < 0.36:
            return {'forward': True, 'backward': False, 'left': False, 'right': False}

        # Post-avoidance settle: once front is open and heading is close,
        # stop side-turning and drive directly toward destination.
        if front > 42 and abs(angle_diff) < 0.22:
            self.action_commitment = 0
            return {'forward': True, 'backward': False, 'left': False, 'right': False}

        # Proactive pre-collision sidestep: react slightly early and offset
        # around obstacle while maintaining forward goal progress.
        if (
            front < 46
            and not collision
            and max(left, right) > 24
            and self.stuck_count < 12
        ):
            action = self._go_around_action(front, left, right, angle_diff)
            self.committed_action = action
            self.action_commitment = 1
            return action
        
        # Use committed action if still valid
        if self.action_commitment > 0:
            self.action_commitment -= 1
            return self.committed_action
        
        # CRITICAL DANGER: Very close - turn only, no forward
        if front < 30:
            if max(left, right) > 22:
                action = self._go_around_action(front, left, right, angle_diff)
            else:
                action = self._safe_turn_action(left, right, backward=False)
            self.committed_action = action
            self.action_commitment = 2  # Keep sidestep very brief; re-align to goal quickly
            return action
        
        # Hybrid policy: planner leads globally; model refines locally.
        model_action = self._model_action(front, left, right, lidar_min, lidar_mean, lidar_std, ground_ultrasonic, angle_diff, collision, speed)
        planner_action = {
            'forward': True,
            'backward': False,
            'left': angle_diff > 0.15,
            'right': angle_diff < -0.15,
        }
        if self.fallback_planner is not None:
            planner_action = self.fallback_planner.get_action(robot_state, goal_pos)

        if self.fallback_planner is not None and bool(self.training_config.get('planner_anchor_mode', True)):
            # Planner-anchored mode (non-parity): keep planner safety/progress behavior,
            # allow model to bias steering only in clearly open space.
            action = dict(planner_action)
            model_turn_gap = abs(
                self.last_model_scores.get('left', 0.0) - self.last_model_scores.get('right', 0.0)
            )
            open_space = front > 96 and min(left, right) > 62 and not collision and self.stuck_count < 8
            if open_space and model_turn_gap > 0.55 and model_action.get('left') != model_action.get('right'):
                action['left'] = model_action['left']
                action['right'] = model_action['right']
                action['forward'] = True
                action['backward'] = False
        else:
            action = self._hybrid_action(planner_action, model_action, front, left, right, angle_diff)

        # Guard rails to keep behavior safe
        if front < 55:
            action['backward'] = False
            # force an evasive steering decision when front is constrained
            if not action['left'] and not action['right']:
                action['left'] = left > right
                action['right'] = right > left

        # Avoid unproductive standstill
        if not (action['forward'] or action['backward']):
            action['forward'] = front > 45
            if not action['forward']:
                action['left'] = left > right
                action['right'] = right > left
        
        # Light commitment to avoid oscillation
        if action['left'] or action['right']:
            self.committed_action = action
            self.action_commitment = 1
        
        return action
    
    def get_decision_confidence(self, robot_state, goal_pos):
        return self.last_model_scores
