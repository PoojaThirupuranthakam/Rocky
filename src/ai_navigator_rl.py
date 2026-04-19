"""Runtime navigator that uses a trained tabular Q-learning policy."""

from __future__ import annotations

import os
from typing import Optional

from rl_q_learning import (
    QTableAgent,
    action_to_command,
    command_to_action_id,
    discretize_state,
    goal_features,
    get_directional_clearances,
    q_values_to_confidence,
    safe_action_mask,
)


class AINavigator:
    """Drop-in navigator compatible with run_simulation.py expectations."""

    def __init__(
        self,
        model_path: str = "models/rl_q_table.pkl",
        cave_env=None,
        planner_fallback: bool = True,
    ):
        self.model_path = model_path
        self.cave_env = cave_env
        self.agent: Optional[QTableAgent] = None
        self.fallback = None
        self._last_action_id: Optional[int] = None
        self._online_updates = 0

        if os.path.exists(model_path):
            self.agent = QTableAgent.load(model_path)
            print(f"📦 Loaded RL policy: {model_path}")
            print(f"   states in table: {len(self.agent.q_table)}")
        else:
            print(f"⚠️ RL model not found at {model_path}")
            if planner_fallback:
                from ai_navigator import AINavigator as PlannerNavigator

                print("↪ Falling back to planner navigator")
                self.fallback = PlannerNavigator(cave_env)

    def get_action(self, state, goal, explore: bool = False):
        if self.agent is None:
            if self.fallback is not None:
                return self.fallback.get_action(state, goal)
            return {"forward": False, "backward": False, "left": True, "right": False}

        s_key = discretize_state(state, goal)
        allowed_now = safe_action_mask(state, goal)
        q_vals = self.agent.predict_q_values(s_key)
        ranked = sorted(allowed_now, key=lambda a: float(q_vals[a]), reverse=True)

        front, _left, _right, _back = get_directional_clearances(state)
        if ranked and not explore:
            action_id = ranked[0]
        else:
            action_id = self.agent.select_action(
                s_key,
                greedy=not explore,
                allowed_actions=allowed_now,
            )

        # Prevent left-right turn ping-pong when we have room ahead.
        if self._last_action_id is not None and front > 18 and len(ranked) > 1:
            opposite_pairs = {
                1: 2,
                2: 1,
                3: 4,
                4: 3,
                6: 7,
                7: 6,
            }
            if opposite_pairs.get(self._last_action_id) == action_id:
                for candidate in ranked[1:]:
                    if opposite_pairs.get(self._last_action_id) != candidate:
                        action_id = candidate
                        break

        self._last_action_id = int(action_id)
        return action_to_command(action_id)

    def online_step(self, prev_state, commands, next_state, goal, reached_goal=False, autosave_interval=300):
        """Apply one online Q-learning update from a simulator transition."""
        if self.agent is None:
            return

        s_key = discretize_state(prev_state, goal)
        n_key = discretize_state(next_state, goal)
        action_id = command_to_action_id(commands)

        prev_dist, _ = goal_features(prev_state, goal)
        next_dist, _ = goal_features(next_state, goal)
        progress = prev_dist - next_dist

        reward = -0.03 + 0.55 * float(progress)
        if next_state.get("collision"):
            reward -= 2.4

        front, _left, _right, _back = get_directional_clearances(next_state)
        if front < 12:
            reward -= 1.1
        elif front < 20:
            reward -= 0.45

        if reached_goal:
            reward += 70.0

        allowed_next = safe_action_mask(next_state, goal)
        self.agent.update(
            s_key,
            int(action_id),
            float(reward),
            n_key,
            bool(reached_goal),
            next_allowed_actions=allowed_next,
        )

        # Very conservative decay during online updates.
        self.agent.epsilon = max(self.agent.config.epsilon_min, self.agent.epsilon * 0.9998)

        self._online_updates += 1
        if autosave_interval and autosave_interval > 0 and (self._online_updates % int(autosave_interval) == 0):
            self.agent.save(self.model_path, metadata={"online_updates": int(self._online_updates)})

    def get_online_stats(self):
        return {"online_updates": int(self._online_updates)}

    def get_decision_confidence(self, state, goal):
        if self.agent is None:
            if self.fallback is not None:
                return self.fallback.get_decision_confidence(state, goal)
            return {"forward": 0.0, "left": 0.5, "right": 0.5}

        s_key = discretize_state(state, goal)
        q_vals = self.agent.predict_q_values(s_key)
        return q_values_to_confidence(q_vals)

    def reset_runtime(self):
        self._last_action_id = None
        if self.fallback is not None and hasattr(self.fallback, "reset_runtime"):
            self.fallback.reset_runtime()
