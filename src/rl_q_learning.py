"""Utilities for discrete-action Q-learning in the cave robot simulator."""

from __future__ import annotations

import math
import os
import pickle
import random
from dataclasses import dataclass
from typing import Dict, List, Sequence, Tuple

import numpy as np

# Discrete action space used by both trainer and runtime navigator.
ACTION_LABELS = {
    0: "forward",
    1: "forward_left",
    2: "forward_right",
    3: "turn_left",
    4: "turn_right",
    5: "backward",
    6: "backward_left",
    7: "backward_right",
}
N_ACTIONS = len(ACTION_LABELS)


def _safe_float(v, default=0.0):
    try:
        return float(v)
    except Exception:
        return float(default)


def get_directional_clearances(state: dict) -> Tuple[float, float, float, float]:
    """Extract front/left/right/back clearances from current sensor payload."""
    sensors = (state or {}).get("sensors", {}) or {}

    lidar_front = _safe_float(sensors.get("lidar_front", sensors.get("lidar_min", 0.0)), 0.0)
    lidar_left = _safe_float(sensors.get("lidar_left", lidar_front), lidar_front)
    lidar_right = _safe_float(sensors.get("lidar_right", lidar_front), lidar_front)

    ground_front = _safe_float(sensors.get("ground_ultrasonic", sensors.get("ground_front", lidar_front)), lidar_front)
    ground_left = _safe_float(sensors.get("ground_left", ground_front), ground_front)
    ground_right = _safe_float(sensors.get("ground_right", ground_front), ground_front)

    top_front = _safe_float(sensors.get("top_front", sensors.get("top", lidar_front)), lidar_front)
    top_left = _safe_float(sensors.get("top_left", lidar_left), lidar_left)
    top_right = _safe_float(sensors.get("top_right", lidar_right), lidar_right)

    front = min(lidar_front, ground_front, top_front)
    left = min(lidar_left, ground_left, top_left)
    right = min(lidar_right, ground_right, top_right)
    back = _safe_float(sensors.get("back", 120.0), 120.0)
    return front, left, right, back


def goal_features(state: dict, goal: Tuple[float, float]) -> Tuple[float, float]:
    """Return distance-to-goal and heading error in radians."""
    x, y = (state or {}).get("position", (0.0, 0.0))
    angle = _safe_float((state or {}).get("angle", 0.0), 0.0)

    dx = float(goal[0] - x)
    dy = float(goal[1] - y)
    dist = math.hypot(dx, dy)

    goal_angle = math.atan2(dy, dx)
    heading_error = (goal_angle - angle + math.pi) % (2.0 * math.pi) - math.pi
    return dist, heading_error


def discretize_state(state: dict, goal: Tuple[float, float]) -> Tuple[int, int, int, int, int]:
    """Convert raw continuous state into compact bins for table-based Q-learning."""
    front, left, right, _back = get_directional_clearances(state)
    goal_dist, heading_error = goal_features(state, goal)

    # Distance bins in pixels (empirically stable with this simulator scale).
    dist_bins = np.array([12, 20, 30, 45, 70, 100], dtype=float)
    goal_bins = np.array([30, 60, 100, 160, 230], dtype=float)
    heading_bins = np.array([-2.3, -1.2, -0.35, 0.35, 1.2, 2.3], dtype=float)

    front_bin = int(np.digitize(front, dist_bins))
    left_bin = int(np.digitize(left, dist_bins))
    right_bin = int(np.digitize(right, dist_bins))
    goal_bin = int(np.digitize(goal_dist, goal_bins))
    heading_bin = int(np.digitize(heading_error, heading_bins))

    return front_bin, left_bin, right_bin, goal_bin, heading_bin


def safe_action_mask(state: dict, goal: Tuple[float, float] | None = None) -> List[int]:
    """Return actions that are currently safe/plausible for exploration and policy rollout."""
    front, left, right, back = get_directional_clearances(state)

    # Base action candidates by frontal risk.
    if front < 10:
        allowed = {3, 4, 5, 6, 7}
    elif front < 16:
        allowed = {1, 2, 3, 4, 5, 6, 7}
    else:
        allowed = set(range(N_ACTIONS))

    # Remove actions steering into near-side walls.
    if left < 10:
        allowed -= {1, 3, 6}
    if right < 10:
        allowed -= {2, 4, 7}

    # If backing space is tight, avoid reverse-heavy actions unless absolutely required.
    if back < 10 and front >= 10:
        allowed -= {5, 6, 7}

    # Goal-aware nudges when clear: avoid unnecessary backward actions.
    if goal is not None and front > 24:
        _dist, heading_error = goal_features(state, goal)
        if abs(heading_error) < 0.9:
            allowed -= {5, 6, 7}

    if not allowed:
        # Guaranteed fallback to avoid dead-end masking.
        if left > right:
            return [3, 5]
        return [4, 5]

    return sorted(allowed)


def action_to_command(action_id: int) -> dict:
    """Map discrete action id -> simulator motor command dictionary."""
    action_id = int(action_id)
    commands = {
        "forward": False,
        "backward": False,
        "left": False,
        "right": False,
    }
    if action_id == 0:
        commands["forward"] = True
    elif action_id == 1:
        commands["forward"] = True
        commands["left"] = True
    elif action_id == 2:
        commands["forward"] = True
        commands["right"] = True
    elif action_id == 3:
        commands["left"] = True
    elif action_id == 4:
        commands["right"] = True
    elif action_id == 5:
        commands["backward"] = True
    elif action_id == 6:
        commands["backward"] = True
        commands["left"] = True
    elif action_id == 7:
        commands["backward"] = True
        commands["right"] = True
    return commands


def command_to_action_id(commands: dict) -> int:
    """Map simulator command dict -> nearest discrete action id."""
    if not isinstance(commands, dict):
        return 0

    forward = bool(commands.get("forward"))
    backward = bool(commands.get("backward"))
    left = bool(commands.get("left"))
    right = bool(commands.get("right"))

    if forward and left and not right:
        return 1
    if forward and right and not left:
        return 2
    if forward and not backward:
        return 0

    if backward and left and not right:
        return 6
    if backward and right and not left:
        return 7
    if backward and not forward:
        return 5

    if left and not right:
        return 3
    if right and not left:
        return 4

    return 0


def apply_action(robot, action_id: int) -> None:
    """Apply a discrete action onto the existing RobotController API."""
    cmd = action_to_command(action_id)
    if cmd["forward"]:
        robot.move_forward()
    if cmd["backward"]:
        robot.move_backward()
    if cmd["left"]:
        robot.turn_left()
    if cmd["right"]:
        robot.turn_right()


@dataclass
class QLearningConfig:
    learning_rate: float = 0.12
    gamma: float = 0.97
    epsilon_start: float = 1.0
    epsilon_min: float = 0.06
    epsilon_decay: float = 0.993


class QTableAgent:
    """Simple tabular Q-learning agent over discretized simulator state."""

    def __init__(self, config: QLearningConfig | None = None):
        self.config = config or QLearningConfig()
        self.epsilon = float(self.config.epsilon_start)
        self.q_table: Dict[Tuple[int, ...], np.ndarray] = {}

    def _ensure_state(self, state_key: Tuple[int, ...]) -> np.ndarray:
        if state_key not in self.q_table:
            self.q_table[state_key] = np.zeros(N_ACTIONS, dtype=np.float32)
        return self.q_table[state_key]

    def predict_q_values(self, state_key: Tuple[int, ...]) -> np.ndarray:
        return self._ensure_state(state_key)

    def select_action(
        self,
        state_key: Tuple[int, ...],
        greedy: bool = False,
        allowed_actions: Sequence[int] | None = None,
    ) -> int:
        q_vals = self._ensure_state(state_key)
        candidates = list(allowed_actions) if allowed_actions else list(range(N_ACTIONS))
        if not candidates:
            candidates = list(range(N_ACTIONS))

        if (not greedy) and (random.random() < self.epsilon):
            return int(random.choice(candidates))

        candidate_q = np.array([q_vals[a] for a in candidates], dtype=float)
        max_q = float(np.max(candidate_q))
        best_idx = np.flatnonzero(np.isclose(candidate_q, max_q))
        best = [candidates[i] for i in best_idx.tolist()]
        if not best:
            return int(candidates[int(np.argmax(candidate_q))])
        return int(random.choice(best))

    def update(
        self,
        state_key: Tuple[int, ...],
        action: int,
        reward: float,
        next_state_key: Tuple[int, ...],
        done: bool,
        next_allowed_actions: Sequence[int] | None = None,
    ) -> None:
        q_vals = self._ensure_state(state_key)
        next_q = self._ensure_state(next_state_key)

        target = float(reward)
        if not done:
            if next_allowed_actions:
                next_candidates = [int(a) for a in next_allowed_actions]
                if next_candidates:
                    next_max = max(float(next_q[a]) for a in next_candidates)
                else:
                    next_max = float(np.max(next_q))
            else:
                next_max = float(np.max(next_q))
            target += self.config.gamma * next_max

        td_error = target - float(q_vals[action])
        q_vals[action] += self.config.learning_rate * td_error

    def decay_epsilon(self) -> None:
        self.epsilon = max(
            float(self.config.epsilon_min),
            float(self.epsilon * self.config.epsilon_decay),
        )

    def save(self, model_path: str, metadata: dict | None = None) -> None:
        os.makedirs(os.path.dirname(model_path) or ".", exist_ok=True)
        payload = {
            "q_table": self.q_table,
            "epsilon": float(self.epsilon),
            "config": vars(self.config),
            "metadata": metadata or {},
        }
        with open(model_path, "wb") as f:
            pickle.dump(payload, f)

    @classmethod
    def load(cls, model_path: str) -> "QTableAgent":
        with open(model_path, "rb") as f:
            payload = pickle.load(f)

        config = payload.get("config", {}) or {}
        agent = cls(QLearningConfig(**config))
        agent.q_table = payload.get("q_table", {}) or {}
        agent.epsilon = float(payload.get("epsilon", agent.config.epsilon_min))
        return agent


def q_values_to_confidence(q_vals: np.ndarray) -> Dict[str, float]:
    """Compress action-value vector into directional confidence values."""
    q = np.asarray(q_vals, dtype=float)
    if q.size != N_ACTIONS:
        q = np.zeros(N_ACTIONS, dtype=float)

    # Stable softmax-like scaling for interpretability in the HUD.
    centered = q - np.max(q)
    exp_q = np.exp(np.clip(centered, -8.0, 8.0))
    probs = exp_q / (np.sum(exp_q) + 1e-9)

    forward = float(probs[0] + probs[1] + probs[2])
    left = float(probs[1] + probs[3] + probs[6])
    right = float(probs[2] + probs[4] + probs[7])

    return {
        "forward": max(0.0, min(1.0, forward)),
        "left": max(0.0, min(1.0, left)),
        "right": max(0.0, min(1.0, right)),
    }
