"""A* path-planning navigator for reliable cave navigation."""

import heapq
import math


class AINavigator:
    """Global planner + local waypoint follower."""

    def __init__(self, cave_env=None):
        self.cave_env = cave_env
        self.grid_size = 10
        self.occupancy = None
        self.path_world = []
        self.path_idx = 0
        self.replan_cooldown = 0
        self.last_goal = None
        self.stuck_frames = 0
        self.last_pos = None
        self.recovery_mode = False
        self.recovery_phase = None
        self.recovery_frames = 0
        self.recovery_turn_left = True
        self.recovery_reverse_frames = 30
        self.recovery_turn_frames = 3
        self.recovery_forward_frames = 16
        self.recovery_stall_frames = 0
        self.recovery_level = 0
        self.active_recovery_reverse_frames = self.recovery_reverse_frames
        self.active_recovery_turn_frames = self.recovery_turn_frames
        self.obstacle_avoid_mode = False
        self.obstacle_avoid_frames = 0
        self.obstacle_avoid_turn_left = True
        self.obstacle_offset_target = None
        self.obstacle_avoid_cooldown = 0
        self.scan_mode = False
        self.scan_last_angle = None
        self.scan_accumulated = 0.0
        self.scan_frames = 0
        self.scan_samples = {}
        self.scan_cooldown = 0
        self.scan_memory = []
        self.safety_pause_frames = 0
        self.was_within_one_foot_zone = False
        self.last_goal_distance = None
        self.no_progress_frames = 0
        self.progress_window_start_dist = None
        self.progress_window_frames = 0
        self.steer_last_dir = 0
        self.steer_flip_total = 0
        self.steer_flip_window_events = []
        self.steer_flip_window_count = 0
        self.steer_window_size = 100
        self.steer_frame_count = 0
        self.same_area_anchor = None
        self.same_area_frames = 0
        self.long_stuck_trigger_frames = 120  # ~2s at 60 FPS
        self.same_area_radius_px = 14.0
        self.hard_block_frames = 0

        if self.cave_env is not None:
            self._build_occupancy()

    def set_environment(self, cave_env):
        self.cave_env = cave_env
        self._build_occupancy()
        self.path_world = []
        self.path_idx = 0

    def _update_oscillation_metrics(self, action):
        """Track steering direction flips to quantify oscillation."""
        turn_dir = 1 if action.get("left") else (-1 if action.get("right") else 0)

        self.steer_frame_count += 1
        flip = 0
        if turn_dir != 0 and self.steer_last_dir != 0 and turn_dir != self.steer_last_dir:
            flip = 1
            self.steer_flip_total += 1
        if turn_dir != 0:
            self.steer_last_dir = turn_dir

        self.steer_flip_window_events.append(flip)
        self.steer_flip_window_count += flip
        if len(self.steer_flip_window_events) > self.steer_window_size:
            removed = self.steer_flip_window_events.pop(0)
            self.steer_flip_window_count -= removed

    def get_oscillation_stats(self):
        return {
            "steer_flip_total": self.steer_flip_total,
            "steer_flip_last_100": self.steer_flip_window_count,
            "window_size": self.steer_window_size,
            "frames_tracked": self.steer_frame_count,
        }

    def _build_occupancy(self):
        """Build a coarse occupancy grid inflated for robot safety."""
        if self.cave_env is None:
            return

        h = self.cave_env.height
        w = self.cave_env.width
        gh = h // self.grid_size
        gw = w // self.grid_size

        occ = [[False for _ in range(gw)] for _ in range(gh)]

        # Match compact robot footprint more closely; too-large inflation can
        # block narrow but traversable corridors and cause apparent stalling.
        inflate_px = 9
        sample_step = 2
        for gy in range(gh):
            cy = gy * self.grid_size + self.grid_size // 2
            for gx in range(gw):
                cx = gx * self.grid_size + self.grid_size // 2
                blocked = False
                # Sample a tiny local patch to avoid narrow unsafe cells
                for dy in range(-inflate_px, inflate_px + 1, sample_step):
                    if blocked:
                        break
                    for dx in range(-inflate_px, inflate_px + 1, sample_step):
                        if dx * dx + dy * dy > inflate_px * inflate_px:
                            continue
                        px = cx + dx
                        py = cy + dy
                        if px < 0 or py < 0 or px >= w or py >= h:
                            blocked = True
                            break
                        if self.cave_env.cave_map[py][px] == 1:
                            blocked = True
                            break
                occ[gy][gx] = blocked

        self.occupancy = occ

    def _world_to_grid(self, x, y):
        gx = int(max(0, min((self.cave_env.width - 1) // self.grid_size, x // self.grid_size)))
        gy = int(max(0, min((self.cave_env.height - 1) // self.grid_size, y // self.grid_size)))
        return gx, gy

    def _grid_to_world(self, gx, gy):
        return (
            gx * self.grid_size + self.grid_size // 2,
            gy * self.grid_size + self.grid_size // 2,
        )

    def _nearest_free(self, node):
        """Find nearest free cell via BFS from requested node."""
        gx, gy = node
        h = len(self.occupancy)
        w = len(self.occupancy[0])

        if 0 <= gy < h and 0 <= gx < w and not self.occupancy[gy][gx]:
            return node

        q = [(gx, gy)]
        seen = {(gx, gy)}
        dirs = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while q:
            cx, cy = q.pop(0)
            for dx, dy in dirs:
                nx, ny = cx + dx, cy + dy
                if (nx, ny) in seen:
                    continue
                if nx < 0 or ny < 0 or ny >= h or nx >= w:
                    continue
                if not self.occupancy[ny][nx]:
                    return (nx, ny)
                seen.add((nx, ny))
                q.append((nx, ny))

        return node

    def _a_star(self, start, goal):
        h = len(self.occupancy)
        w = len(self.occupancy[0])
        dirs = [
            (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
            (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
        ]

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_heap = []
        heapq.heappush(open_heap, (0.0, start))

        came_from = {}
        g_score = {start: 0.0}

        while open_heap:
            _, current = heapq.heappop(open_heap)
            if current == goal:
                # Reconstruct
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path

            cx, cy = current
            for dx, dy, step_cost in dirs:
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if self.occupancy[ny][nx]:
                    continue
                # prevent cutting corners diagonally through walls
                if dx != 0 and dy != 0:
                    if self.occupancy[cy][nx] or self.occupancy[ny][cx]:
                        continue

                neighbor = (nx, ny)
                tentative = g_score[current] + step_cost
                if tentative < g_score.get(neighbor, float("inf")):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative
                    f = tentative + heuristic(neighbor, goal)
                    heapq.heappush(open_heap, (f, neighbor))

        return []

    def _simplify_path(self, grid_path):
        """Reduce jitter by keeping only turning points."""
        if len(grid_path) <= 2:
            return grid_path

        simplified = [grid_path[0]]
        prev_dx = grid_path[1][0] - grid_path[0][0]
        prev_dy = grid_path[1][1] - grid_path[0][1]

        for i in range(1, len(grid_path) - 1):
            dx = grid_path[i + 1][0] - grid_path[i][0]
            dy = grid_path[i + 1][1] - grid_path[i][1]
            if dx != prev_dx or dy != prev_dy:
                simplified.append(grid_path[i])
            prev_dx, prev_dy = dx, dy

        simplified.append(grid_path[-1])
        return simplified

    def _plan_path(self, start_pos, goal_pos):
        if self.cave_env is None or self.occupancy is None:
            return []

        start = self._nearest_free(self._world_to_grid(start_pos[0], start_pos[1]))
        goal = self._nearest_free(self._world_to_grid(goal_pos[0], goal_pos[1]))
        grid_path = self._a_star(start, goal)
        if not grid_path:
            return []

        grid_path = self._simplify_path(grid_path)
        return [self._grid_to_world(gx, gy) for gx, gy in grid_path]

    @staticmethod
    def _angle_diff(a, b):
        return (a - b + math.pi) % (2 * math.pi) - math.pi

    def _update_scan_memory(self, robot_state):
        """Accumulate local obstacle distances while rotating in place."""
        heading = robot_state["angle"]
        sensors = robot_state["sensors"]
        front_dist, _, _ = self._extract_directional_clearance(sensors)
        front_dist = float(front_dist)

        # Record every 15° sector with conservative nearest-hit memory.
        sector_deg = int(round(math.degrees(heading) / 15.0) * 15) % 360
        prev = self.scan_samples.get(sector_deg)
        if prev is None or front_dist < prev:
            self.scan_samples[sector_deg] = front_dist

    def _extract_directional_clearance(self, sensors):
        """Derive front/left/right clearances from top LiDAR + ground ultrasonic (or legacy top+ground sectors)."""
        ground_front = sensors.get("ground_ultrasonic", sensors.get("ground_front", sensors.get("ground")))
        cm_per_pixel = float(getattr(self.cave_env, "cm_per_pixel", 0.5) or 0.5)
        ground_guard_px = max(1.0, 22.86 / cm_per_pixel)  # ~0.75 ft: near-ground hazard guard

        lidar_front = sensors.get("lidar_front")
        lidar_left = sensors.get("lidar_left")
        lidar_right = sensors.get("lidar_right")
        if lidar_front is not None and lidar_left is not None and lidar_right is not None:
            front = float(lidar_front)
            left = float(lidar_left)
            right = float(lidar_right)
            if ground_front is not None:
                g_front = float(ground_front)
                if g_front < ground_guard_px:
                    front = min(front, g_front)
            return front, left, right

        lidar_scan = sensors.get("lidar_scan")
        if isinstance(lidar_scan, (list, tuple)) and len(lidar_scan) >= 3:
            n = len(lidar_scan)
            rel_angles = [(-math.pi + (2.0 * math.pi * i) / n) for i in range(n)]

            def angle_wrap(rad):
                return (rad + math.pi) % (2.0 * math.pi) - math.pi

            sector_half = math.pi / 6.0
            front_vals = [float(v) for v, rel in zip(lidar_scan, rel_angles) if abs(angle_wrap(rel)) <= sector_half]
            left_vals = [float(v) for v, rel in zip(lidar_scan, rel_angles) if abs(angle_wrap(rel - math.pi / 4.0)) <= sector_half]
            right_vals = [float(v) for v, rel in zip(lidar_scan, rel_angles) if abs(angle_wrap(rel + math.pi / 4.0)) <= sector_half]
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

        top_front = float(sensors.get("top_front", sensors.get("top", 0.0)))
        top_left = float(sensors.get("top_left", top_front))
        top_right = float(sensors.get("top_right", top_front))

        ground_front = float(sensors.get("ground_front", sensors.get("ground", top_front)))
        ground_left = float(sensors.get("ground_left", sensors.get("ground", top_left)))
        ground_right = float(sensors.get("ground_right", sensors.get("ground", top_right)))

        front = min(top_front, ground_front)
        left = min(top_left, ground_left)
        right = min(top_right, ground_right)
        return front, left, right

    def _finalize_scan_plan(self, robot_state, goal_pos):
        """Choose safest goal-directed heading after a full scan and seed next path."""
        if self.cave_env is None:
            return

        one_foot_px = float(getattr(self.cave_env, 'cm_per_pixel', 0.5) or 0.5)
        one_foot_px = max(1.0, 30.48 / one_foot_px)
        goal_angle = math.atan2(goal_pos[1] - robot_state["position"][1], goal_pos[0] - robot_state["position"][0])

        samples = self.scan_samples or {}
        self.scan_memory = sorted(samples.items())

        best_heading = goal_angle
        if samples:
            clear_candidates = []
            all_candidates = []
            for deg, dist in samples.items():
                heading = math.radians(deg)
                ang_err = abs(self._angle_diff(goal_angle, heading))
                # Lower score is better.
                score = ang_err - min(dist, one_foot_px * 1.5) / (one_foot_px * 3.0)
                all_candidates.append((score, heading, dist))
                if dist >= (one_foot_px - 1.5):
                    clear_candidates.append((score, heading, dist))

            if clear_candidates:
                clear_candidates.sort(key=lambda x: x[0])
                best_heading = clear_candidates[0][1]
            elif all_candidates:
                # No fully clear ray: choose best compromise (closest to goal with max clearance bias).
                all_candidates.sort(key=lambda x: x[0])
                best_heading = all_candidates[0][1]

        # Seed a short safe offset waypoint, then continue to destination.
        step = min(36.0, max(18.0, one_foot_px * 0.65))
        px, py = robot_state["position"]
        tx = px + math.cos(best_heading) * step
        ty = py + math.sin(best_heading) * step
        margin = 12
        tx = max(margin, min(self.cave_env.width - margin, tx))
        ty = max(margin, min(self.cave_env.height - margin, ty))

        # Force a local bias first, then shortest path recompute to goal.
        local_waypoint = (tx, ty)
        local_path = self._plan_path((px, py), local_waypoint)
        goal_path = self._plan_path(local_waypoint, goal_pos)

        combined = []
        if local_path:
            combined.extend(local_path)
        else:
            combined.append(local_waypoint)

        if goal_path:
            # avoid duplicate join node
            if combined and goal_path and math.hypot(combined[-1][0] - goal_path[0][0], combined[-1][1] - goal_path[0][1]) < 1.0:
                combined.extend(goal_path[1:])
            else:
                combined.extend(goal_path)
        else:
            combined.append(goal_pos)

        self.path_world = combined
        self.path_idx = 0
        self.last_goal = goal_pos
        self.replan_cooldown = 16

    def _follow_waypoint(self, robot_state, waypoint):
        x, y = robot_state["position"]
        angle = robot_state["angle"]
        sensors = robot_state["sensors"]

        dx = waypoint[0] - x
        dy = waypoint[1] - y
        target_angle = math.atan2(dy, dx)
        angle_diff = (target_angle - angle + math.pi) % (2 * math.pi) - math.pi

        front, left, right = self._extract_directional_clearance(sensors)

        def finalize(action):
            self._update_oscillation_metrics(action)
            return action

        def choose_goal_aligned_direction(allow_front=True):
            """Choose a collision-safe heading (front/left/right) closest to goal direction."""
            candidates = []
            if allow_front and front >= 24:
                candidates.append(("front", 0.0, front))
            if left >= 30:
                candidates.append(("left", math.pi / 2, left))
            if right >= 30:
                candidates.append(("right", -math.pi / 2, right))

            if not candidates:
                return None

            best = None
            for name, rel_angle, clearance in candidates:
                candidate_angle = angle + rel_angle
                candidate_error = (target_angle - candidate_angle + math.pi) % (2 * math.pi) - math.pi
                away_penalty = 0.55 if abs(candidate_error) > (math.pi / 2) else 0.0
                # Lower score is better: prefer heading toward goal, then more clearance.
                score = abs(candidate_error) + away_penalty - min(clearance, 120.0) / 240.0
                if best is None or score < best[0]:
                    best = (score, name, candidate_error)

            return best

        # Immediate safety override
        if front < 22:
            # Prefer a go-around maneuver when a side is available.
            turn_left = left >= right
            side_clear = left if turn_left else right
            return finalize({
                "forward": side_clear > 18 and front > 10,
                "backward": False,
                "left": turn_left,
                "right": not turn_left,
            })

        # Side-clearance preference:
        # If a side corridor is noticeably wider than the front corridor,
        # pick the collision-safe option that still best aligns to destination.
        side_advantage = 22
        side_open_threshold = 52
        best_side = max(left, right)
        use_side_bias = (
            best_side >= side_open_threshold
            and best_side >= front + side_advantage
        )

        if use_side_bias:
            preferred = choose_goal_aligned_direction(allow_front=True)
            if preferred is not None:
                _, choice, choice_error = preferred
                if abs(choice_error) > (math.pi / 2):
                    return finalize({
                        "forward": False,
                        "backward": False,
                        "left": angle_diff > 0,
                        "right": angle_diff < 0,
                    })
                if choice == "front":
                    return finalize({
                        "forward": True,
                        "backward": False,
                        "left": abs(angle_diff) > 0.22 and angle_diff > 0,
                        "right": abs(angle_diff) > 0.22 and angle_diff < 0,
                    })
                return finalize({
                    "forward": front > 18,
                    "backward": False,
                    "left": choice == "left",
                    "right": choice == "right",
                })

        # Proactive near-collision avoidance (before actual collision)
        # Keep moving when possible, but pick a side that is both safe and
        # better aligned with destination heading.
        if front < 36:
            preferred_side = choose_goal_aligned_direction(allow_front=False)
            if preferred_side is not None:
                _, choice, choice_error = preferred_side
                if abs(choice_error) <= (math.pi / 2):
                    return finalize({
                        "forward": front > 24,
                        "backward": False,
                        "left": choice == "left",
                        "right": choice == "right",
                    })
            turn_left = left > right
            return finalize({
                "forward": front > 24,
                "backward": False,
                "left": turn_left,
                "right": not turn_left,
            })

        # If we're already pointed near destination and front is clear,
        # prioritize clean forward motion without unnecessary rotation.
        forward_clear_threshold = 70
        heading_lock_threshold = 0.08
        front_is_dominant = front >= max(left, right) - 6
        if (
            front >= forward_clear_threshold
            and abs(angle_diff) <= heading_lock_threshold
            and front_is_dominant
        ):
            return finalize({
                "forward": True,
                "backward": False,
                "left": False,
                "right": False,
            })

        turn_threshold = 0.18
        strong_turn_threshold = 0.45

        if abs(angle_diff) > strong_turn_threshold:
            # If there's enough space ahead, prefer moving forward while turning
            # instead of doing a large in-place turn.
            if front > 48:
                return finalize({
                    "forward": True,
                    "backward": False,
                    "left": angle_diff > 0,
                    "right": angle_diff < 0,
                })
            return finalize({
                "forward": False,
                "backward": False,
                "left": angle_diff > 0,
                "right": angle_diff < 0,
            })

        if abs(angle_diff) > turn_threshold:
            return finalize({
                "forward": True,
                "backward": False,
                "left": angle_diff > 0,
                "right": angle_diff < 0,
            })

        return finalize({"forward": True, "backward": False, "left": False, "right": False})

    def get_action(self, robot_state, goal_pos):
        if self.cave_env is None:
            # Safe fallback if env wasn't wired
            sensors = robot_state["sensors"]
            front, left, right = self._extract_directional_clearance(sensors)
            turn_left = left > right
            if front < 20:
                return {"forward": False, "backward": True, "left": turn_left, "right": not turn_left}
            return {"forward": True, "backward": False, "left": False, "right": False}

        pos = robot_state["position"]
        collision = robot_state["collision"]
        sensors = robot_state["sensors"]
        front, left, right = self._extract_directional_clearance(sensors)
        cm_per_pixel = float(getattr(self.cave_env, 'cm_per_pixel', 0.5) or 0.5)
        one_foot_px = max(1.0, 30.48 / cm_per_pixel)
        safe_buffer_px = one_foot_px
        hard_stop_px = max(12.0, safe_buffer_px * 0.34)
        min_clearance = min(front, left, right)
        side_guard_px = safe_buffer_px * 0.78
        side_hard_px = max(9.0, hard_stop_px * 0.82)
        near_obstacle_within_1ft = (front < safe_buffer_px) or (min(left, right) < side_guard_px)
        hard_blocked = (front < hard_stop_px) or (min(left, right) < side_hard_px)
        very_close_obstacle = min_clearance < max(10.0, hard_stop_px * 0.88)
        if hard_blocked:
            self.hard_block_frames += 1
        else:
            self.hard_block_frames = max(0, self.hard_block_frames - 3)

        if self.scan_cooldown > 0:
            self.scan_cooldown -= 1
        if self.obstacle_avoid_cooldown > 0:
            self.obstacle_avoid_cooldown -= 1
        if self.safety_pause_frames > 0:
            self.safety_pause_frames -= 1

        dx_goal = goal_pos[0] - pos[0]
        dy_goal = goal_pos[1] - pos[1]
        goal_angle = math.atan2(dy_goal, dx_goal)
        angle_diff_goal = (goal_angle - robot_state["angle"] + math.pi) % (2 * math.pi) - math.pi
        goal_dist = math.hypot(dx_goal, dy_goal)

        # Detect long moving loops with little net progress toward destination.
        if self.last_goal_distance is not None:
            if goal_dist > self.last_goal_distance - 0.18:
                self.no_progress_frames += 1
            else:
                self.no_progress_frames = max(0, self.no_progress_frames - 3)
        self.last_goal_distance = goal_dist

        # Long-window progress watchdog: catches looping trajectories that keep
        # moving but make very little net progress toward the destination.
        if self.progress_window_start_dist is None:
            self.progress_window_start_dist = goal_dist
            self.progress_window_frames = 0
        self.progress_window_frames += 1

        window_progress_px = self.progress_window_start_dist - goal_dist
        if window_progress_px >= 18.0:
            self.progress_window_start_dist = goal_dist
            self.progress_window_frames = 0

        # Same-area watchdog: if robot keeps orbiting in nearly the same region
        # for ~2 seconds, trigger an aggressive backtrack + fresh path search.
        if self.same_area_anchor is None:
            self.same_area_anchor = pos
            self.same_area_frames = 0
        else:
            area_drift = math.hypot(pos[0] - self.same_area_anchor[0], pos[1] - self.same_area_anchor[1])
            if area_drift <= self.same_area_radius_px:
                self.same_area_frames += 1
            else:
                self.same_area_anchor = pos
                self.same_area_frames = 0

        long_stuck_same_area = self.same_area_frames >= self.long_stuck_trigger_frames
        long_no_progress_loop = self.no_progress_frames >= 92 and goal_dist > 120

        if (long_stuck_same_area or long_no_progress_loop) and not self.recovery_mode:
            self.obstacle_avoid_mode = False
            self.obstacle_avoid_frames = 0
            self.obstacle_offset_target = None
            self.scan_mode = False
            self.recovery_level = 4
            self.active_recovery_reverse_frames = 84 if long_stuck_same_area else 68
            self.active_recovery_turn_frames = max(self.active_recovery_turn_frames, 14)
            self.recovery_mode = True
            self.recovery_phase = "reverse"
            self.recovery_frames = 0
            self.recovery_turn_left = left >= right
            self.path_world = []
            self.path_idx = 0
            self.replan_cooldown = 0
            self.same_area_anchor = pos
            self.same_area_frames = 0
            self.no_progress_frames = 0
            self.progress_window_start_dist = goal_dist
            self.progress_window_frames = 0
            return {
                "forward": False,
                "backward": True,
                "left": False,
                "right": False,
            }

        def choose_recovery_turn_left():
            """Pick recovery turn that favors both free space and destination heading."""
            left_clear = left
            right_clear = right
            goal_left = angle_diff_goal > 0

            # If one side is clearly safer, prefer it first.
            if left_clear > right_clear + 10:
                return True
            if right_clear > left_clear + 10:
                return False

            left_score = (0.65 if goal_left else 0.0) + min(left_clear, 120.0) / 120.0
            right_score = (0.65 if not goal_left else 0.0) + min(right_clear, 120.0) / 120.0
            return left_score >= right_score

        def choose_avoid_turn_left():
            """Prefer side with more clearance, with goal heading as a tie-breaker."""
            left_clear = left
            right_clear = right
            if left_clear > right_clear + 8:
                return True
            if right_clear > left_clear + 8:
                return False
            return angle_diff_goal > 0

        def build_offset_target(turn_left):
            """Create a short lateral offset target to slip around an upcoming obstacle."""
            travel_angle = goal_angle
            side_angle = travel_angle + (math.pi / 2 if turn_left else -math.pi / 2)
            offset_dist = 26.0
            tx = pos[0] + math.cos(side_angle) * offset_dist
            ty = pos[1] + math.sin(side_angle) * offset_dist

            # Keep target inside map bounds so we don't steer into walls.
            margin = 12
            tx = max(margin, min(self.cave_env.width - margin, tx))
            ty = max(margin, min(self.cave_env.height - margin, ty))
            return (tx, ty)

        # --- Pre-collision safety scan mode ---
    # If something is detected within the safety envelope, stop and do a full 360° scan,
        # then choose the shortest safe route toward destination.
        if self.scan_mode:
            self.scan_frames += 1
            self._update_scan_memory(robot_state)

            current_angle = robot_state["angle"]
            if self.scan_last_angle is None:
                self.scan_last_angle = current_angle
            else:
                delta = (current_angle - self.scan_last_angle) % (2 * math.pi)
                if delta > 0:
                    self.scan_accumulated += delta
                self.scan_last_angle = current_angle

            if self.scan_accumulated >= (2 * math.pi - 0.20) or self.scan_frames >= 64:
                self.scan_mode = False
                self.scan_last_angle = None
                self.scan_accumulated = 0.0
                self.scan_frames = 0
                self.scan_cooldown = 28
                self._finalize_scan_plan(robot_state, goal_pos)
                return {"forward": False, "backward": False, "left": False, "right": False}

            return {"forward": False, "backward": False, "left": True, "right": False}

    # Enter a short hard stop when first crossing the safety zone.
        if (
            near_obstacle_within_1ft
            and not collision
            and not self.scan_mode
            and not self.obstacle_avoid_mode
            and not self.was_within_one_foot_zone
            and self.safety_pause_frames == 0
        ):
            self.safety_pause_frames = 2
            self.was_within_one_foot_zone = True
            return {"forward": False, "backward": False, "left": False, "right": False}

        if min_clearance > (safe_buffer_px + 2.0):
            self.was_within_one_foot_zone = False

        if self.safety_pause_frames > 0:
            return {"forward": False, "backward": False, "left": False, "right": False}

        # Anti-stall escape: if trapped in a long no-progress loop and almost clear,
        # allow a cautious forward+turn nudge to break local minima.
        if (
            self.no_progress_frames > 84
            and not collision
            and front > (safe_buffer_px * 0.84)
            and max(left, right) > (safe_buffer_px * 0.72)
        ):
            self.no_progress_frames = max(0, self.no_progress_frames - 28)
            turn_left = left >= right
            return {
                "forward": True,
                "backward": False,
                "left": turn_left,
                "right": not turn_left,
            }

        # Hard safety envelope: do not allow forward drive while any top/ground
    # directional clearance is below 1 foot.
        if hard_blocked:
            self.obstacle_avoid_mode = False
            self.obstacle_avoid_frames = 0
            self.obstacle_offset_target = None

            turn_left = choose_recovery_turn_left()
            boxed_side = left < side_hard_px and right < side_hard_px

            if self.hard_block_frames > 28:
                self.hard_block_frames = 0
                self.no_progress_frames = max(0, self.no_progress_frames - 14)
                self.path_world = []
                self.path_idx = 0
                self.replan_cooldown = 0
                return {
                    "forward": False,
                    "backward": True,
                    "left": turn_left,
                    "right": not turn_left,
                }

            return {
                "forward": False,
                "backward": bool(very_close_obstacle or boxed_side),
                "left": turn_left,
                "right": not turn_left,
            }

        # --- Obstacle go-around mode ---
        # When front is blocked but a side is available, commit to a brief
        # forward+turn maneuver to go around instead of repeatedly reversing.
        if self.obstacle_avoid_mode:
            self.obstacle_avoid_frames += 1

            side_clear = left if self.obstacle_avoid_turn_left else right
            opposite_clear = right if self.obstacle_avoid_turn_left else left

            if collision:
                self.obstacle_avoid_mode = False
                self.obstacle_avoid_frames = 0
                self.obstacle_offset_target = None
                self.obstacle_avoid_cooldown = 10
            elif (
                self.obstacle_avoid_frames > 2
                and front > 32
                and abs(angle_diff_goal) < 0.48
            ):
                self.obstacle_avoid_mode = False
                self.obstacle_avoid_frames = 0
                self.obstacle_offset_target = None
                self.obstacle_avoid_cooldown = 8
            elif self.obstacle_avoid_frames >= 8:
                self.obstacle_avoid_mode = False
                self.obstacle_avoid_frames = 0
                self.obstacle_offset_target = None
                self.obstacle_avoid_cooldown = 8
            elif side_clear < 14 and front < 18 and opposite_clear < 14:
                self.obstacle_avoid_mode = False
                self.obstacle_avoid_frames = 0
                self.obstacle_offset_target = None
                self.obstacle_avoid_cooldown = 10
            elif front < 20:
                # Too close for safe in-place sidestep: bail out with a brief
                # reverse-turn to create separation before re-attempting go-around.
                self.obstacle_avoid_mode = False
                self.obstacle_avoid_frames = 0
                self.obstacle_offset_target = None
                self.obstacle_avoid_cooldown = 8
                return {
                    "forward": False,
                    "backward": True,
                    "left": self.obstacle_avoid_turn_left,
                    "right": not self.obstacle_avoid_turn_left,
                }
            else:
                # Strict 1-foot safety envelope: do not move forward while any
                # front/side reading is still within the boundary.
                can_forward = (
                    (not hard_blocked)
                    and front >= max(24.0, hard_stop_px + 6.0)
                    and side_clear >= (side_hard_px + 4.0)
                    and opposite_clear >= (side_hard_px + 2.0)
                )

                # Short side-step, then immediately bias back to goal heading.
                if self.obstacle_avoid_frames <= 3 and self.obstacle_offset_target is not None:
                    ox, oy = self.obstacle_offset_target
                    target_angle = math.atan2(oy - pos[1], ox - pos[0])
                    offset_diff = (target_angle - robot_state["angle"] + math.pi) % (2 * math.pi) - math.pi
                    steer_left = offset_diff > 0.09 and left > 14
                    steer_right = offset_diff < -0.09 and right > 14
                else:
                    steer_left = angle_diff_goal > 0.10 and left > 14
                    steer_right = angle_diff_goal < -0.10 and right > 14

                if not steer_left and not steer_right:
                    steer_left = self.obstacle_avoid_turn_left
                    steer_right = not self.obstacle_avoid_turn_left

                return {
                    "forward": can_forward,
                    "backward": False,
                    "left": steer_left,
                    "right": steer_right,
                }

        if (
            not collision
            and front < (hard_stop_px + 2.0)
            and max(left, right) > 22
            and self.stuck_frames < 32
            and self.obstacle_avoid_cooldown == 0
        ):
            self.obstacle_avoid_mode = True
            self.obstacle_avoid_frames = 0
            self.obstacle_avoid_turn_left = choose_avoid_turn_left()
            self.obstacle_offset_target = build_offset_target(self.obstacle_avoid_turn_left)
            return {
                "forward": False,
                "backward": False,
                "left": self.obstacle_avoid_turn_left,
                "right": not self.obstacle_avoid_turn_left,
            }

        # --- Recovery state machine ---
        # Phase 1: reverse enough to clear obstacle
        # Phase 2: commit to a bigger turn to escape local trap
        # Phase 3: brief forward push to re-enter normal navigation
        if self.recovery_mode:
            self.recovery_frames += 1

            # Track motion during recovery (normal stuck detector is below and
            # would be skipped by early returns in this block).
            recovery_moved = None
            if self.last_pos is not None:
                recovery_moved = abs(pos[0] - self.last_pos[0]) + abs(pos[1] - self.last_pos[1])
            self.last_pos = pos
            if recovery_moved is not None:
                if recovery_moved < 0.30:
                    self.stuck_frames += 1
                    self.recovery_stall_frames += 1
                else:
                    self.stuck_frames = max(0, self.stuck_frames - 3)
                    self.recovery_stall_frames = max(0, self.recovery_stall_frames - 2)

            if self.recovery_phase == "reverse":
                reverse_ineffective = self.stuck_frames > 12 or self.recovery_stall_frames > 10
                if self.recovery_frames >= self.active_recovery_reverse_frames or reverse_ineffective:
                    self.recovery_phase = "turn"
                    self.recovery_frames = 0
                    self.stuck_frames = 0
                    self.recovery_stall_frames = 0
                return {
                    "forward": False,
                    "backward": True,
                    # Arc while reversing to avoid getting pinned against flat walls.
                    "left": self.recovery_turn_left,
                    "right": not self.recovery_turn_left,
                }

            if self.recovery_phase == "turn":
                if self.recovery_frames >= self.active_recovery_turn_frames:
                    self.recovery_phase = "forward"
                    self.recovery_frames = 0
                return {
                    # Keep recovery turn in-place to avoid wide arcing turns
                    # into nearby walls. Forward is allowed only in forward phase.
                    "forward": False,
                    "backward": False,
                    "left": self.recovery_turn_left,
                    "right": not self.recovery_turn_left,
                }

            # forward phase
            if self.recovery_frames >= self.recovery_forward_frames:
                self.recovery_mode = False
                self.recovery_phase = None
                self.recovery_frames = 0
                self.stuck_frames = 0
                self.recovery_stall_frames = 0
                self.replan_cooldown = 0
                self.path_world = []
                self.path_idx = 0
                self.recovery_level = max(0, self.recovery_level - 1)

            # During recovery push-forward, keep steering toward destination
            # when safe so we don't drift away from goal.
            goal_steer = abs(angle_diff_goal) > 0.14
            steer_left = goal_steer and angle_diff_goal > 0 and left > 16
            steer_right = goal_steer and angle_diff_goal < 0 and right > 16
            can_forward = front > 18

            return {
                "forward": can_forward,
                "backward": False,
                "left": steer_left if can_forward else (angle_diff_goal > 0),
                "right": steer_right if can_forward else (angle_diff_goal < 0),
            }

        # Stuck detection
        if self.last_pos is not None:
            moved = abs(pos[0] - self.last_pos[0]) + abs(pos[1] - self.last_pos[1])
            if moved < 0.25:
                self.stuck_frames += 1
            else:
                self.stuck_frames = max(0, self.stuck_frames - 2)
        self.last_pos = pos

        need_replan = False
        if not self.path_world or self.path_idx >= len(self.path_world):
            need_replan = True
        if self.last_goal != goal_pos:
            need_replan = True
        if collision or self.stuck_frames > 25:
            need_replan = True
        if long_stuck_same_area:
            need_replan = True
        if self.no_progress_frames > 130:
            need_replan = True
            self.no_progress_frames = 0
        if self.progress_window_frames > 220:
            need_replan = True
            self.progress_window_start_dist = goal_dist
            self.progress_window_frames = 0
        if self.replan_cooldown <= 0 and front < 16:
            need_replan = True
        # Replan earlier when obstacle is approaching to reduce hard collisions
        if self.replan_cooldown <= 0 and front < 32:
            need_replan = True

        reverse_needed = collision or front < 22 or self.stuck_frames > 36 or long_stuck_same_area
        scan_worthwhile = (
            collision
            or (front < 18 and max(left, right) < 24)
            or (self.stuck_frames > 44 and front < 28)
            or long_stuck_same_area
        )

        if reverse_needed and scan_worthwhile and (not self.scan_mode) and self.scan_cooldown == 0:
            # 360° scan is reserved for cases that would otherwise require reverse.
            self.scan_mode = True
            self.scan_last_angle = robot_state["angle"]
            self.scan_accumulated = 0.0
            self.scan_frames = 0
            self.scan_samples = {}
            self._update_scan_memory(robot_state)
            self.obstacle_avoid_mode = False
            self.obstacle_avoid_frames = 0
            self.obstacle_offset_target = None
            return {"forward": False, "backward": False, "left": False, "right": False}

        if reverse_needed:
            self.obstacle_avoid_mode = False
            self.obstacle_avoid_frames = 0
            self.obstacle_offset_target = None
            self.scan_mode = False
            # Adaptive recovery: keep initial turns small, but enlarge them if
            # repeated recovery events happen to avoid local collision loops.
            self.recovery_level = min(4, self.recovery_level + 1)
            self.active_recovery_reverse_frames = 18 + self.recovery_level * 5
            self.active_recovery_turn_frames = 2 + self.recovery_level * 2
            if long_stuck_same_area:
                # User-facing behavior: if stuck in same area > ~2s,
                # backtrack considerably before trying a new path.
                self.recovery_level = 4
                self.active_recovery_reverse_frames = max(self.active_recovery_reverse_frames, 84)
                self.active_recovery_turn_frames = max(self.active_recovery_turn_frames, 14)
                self.path_world = []
                self.path_idx = 0
                self.replan_cooldown = 0
                self.same_area_anchor = pos
                self.same_area_frames = 0
            self.recovery_mode = True
            self.recovery_phase = "reverse"
            self.recovery_frames = 0
            self.stuck_frames = 0
            self.recovery_stall_frames = 0
            self.recovery_turn_left = choose_recovery_turn_left()
            return {
                "forward": False,
                "backward": True,
                "left": False,
                "right": False,
            }

        if need_replan:
            self.path_world = self._plan_path(pos, goal_pos)
            self.path_idx = 0
            self.last_goal = goal_pos
            self.replan_cooldown = 20
            self.stuck_frames = 0
        else:
            self.replan_cooldown = max(0, self.replan_cooldown - 1)

        if not self.path_world:
            # If no path exists, use conservative obstacle-avoidance fallback
            turn_left = left > right
            if front < 22:
                return {"forward": False, "backward": True, "left": turn_left, "right": not turn_left}
            return {"forward": True, "backward": False, "left": turn_left, "right": not turn_left}

        # Advance waypoints
        while self.path_idx < len(self.path_world):
            wx, wy = self.path_world[self.path_idx]
            if math.hypot(wx - pos[0], wy - pos[1]) < 18:
                self.path_idx += 1
            else:
                break

        if self.path_idx >= len(self.path_world):
            # final heading into goal
            self.path_world = [goal_pos]
            self.path_idx = 0

        # Near goal: favor direct steering to the actual goal to avoid orbiting
        # around stale coarse-grid waypoints.
        goal_dist = math.hypot(goal_pos[0] - pos[0], goal_pos[1] - pos[1])
        if goal_dist < 260:
            direct_action = self._follow_waypoint(robot_state, goal_pos)
            # If close to goal and front is clear enough, bias toward forward progress.
            if front > 42:
                direct_action["forward"] = True
                direct_action["backward"] = False
            return direct_action

        waypoint = self.path_world[self.path_idx]
        return self._follow_waypoint(robot_state, waypoint)

    def get_decision_confidence(self, robot_state, goal_pos):
        if self.path_world:
            return {"forward": 0.9, "backward": 0.1, "left": 0.3, "right": 0.3}
        return {"forward": 0.6, "backward": 0.4, "left": 0.4, "right": 0.4}
