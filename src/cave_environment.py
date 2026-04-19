"""
Virtual Cave Environment Generator
Creates realistic cave structures for robot testing
"""

import pygame
import numpy as np
from noise import pnoise2
import random


class CaveEnvironment:
    """Generates and manages a procedural cave environment"""
    
    def __init__(
        self,
        width=800,
        height=600,
        complexity=0.1,
        obstacle_level='medium',
        scatter_obstacles_in_path=False,
        robot_radius=5,
        person_width_cm=45.0,
        person_height_cm=165.0,
        cm_per_pixel=0.5,
    ):
        self.width = width
        self.height = height
        self.complexity = complexity
        self.obstacle_level = str(obstacle_level).strip().lower()
        self.obstacle_profiles = {
            'light': {
                'noise_bias': 0.02,
                'struct_mult': 0.80,
                'marker_mult': 0.80,
            },
            'medium': {
                'noise_bias': 0.00,
                'struct_mult': 1.00,
                'marker_mult': 1.00,
            },
            'heavy': {
                'noise_bias': -0.015,
                'struct_mult': 1.25,
                'marker_mult': 1.25,
            },
            'extra-heavy': {
                'noise_bias': -0.03,
                'struct_mult': 1.55,
                'marker_mult': 1.55,
            },
        }
        if self.obstacle_level not in self.obstacle_profiles:
            self.obstacle_level = 'medium'
        self.scatter_obstacles_in_path = bool(scatter_obstacles_in_path)
        self.robot_radius = max(4, int(robot_radius))
        self.person_width_cm = max(20.0, float(person_width_cm))
        self.person_height_cm = max(80.0, float(person_height_cm))
        self.cm_per_pixel = max(0.1, float(cm_per_pixel))

        # Required half-width clearance in pixels for the person profile.
        person_half_width_px = (self.person_width_cm / 2.0) / self.cm_per_pixel
        # Taller profiles get a small extra turning/sway margin in narrow passages.
        height_scale = np.clip(self.person_height_cm / 170.0, 0.8, 1.25)
        required_half_clearance_px = max(self.robot_radius + 2, int(round(person_half_width_px * height_scale)))

        # Keep at least one comfortable body-width corridor plus turning margin.
        self.corridor_radius = max(24, required_half_clearance_px + 8)
        self.start_goal_clear_radius = max(50, required_half_clearance_px + 16)
        self.cave_map = None
        self.obstacles = set()
        self.start_pos = None
        self.goal_pos = None
        self.terrain_markers = {
            'water': [],
            'unstable_ground': [],
            'hole': [],
            'narrow_space': [],
            'jump_space': [],
            'debris': [],
        }
        self.safe_corridor_points = set()
        
        # Generate cave
        self.generate_cave()
    
    def generate_cave(self):
        """Generate procedural cave using Perlin noise"""
        print("🏔️  Generating cave environment...")
        
        # Create empty map
        self.cave_map = np.zeros((self.height, self.width))
        
        # Use Perlin noise for natural cave walls
        scale = 50.0
        octaves = 6
        persistence = 0.5
        lacunarity = 2.0
        base_seed = random.randint(0, 10_000)
        profile = self.obstacle_profiles[self.obstacle_level]
        wall_threshold = self.complexity + float(profile['noise_bias'])
        
        for y in range(self.height):
            for x in range(self.width):
                noise_val = pnoise2(
                    x / scale,
                    y / scale,
                    octaves=octaves,
                    persistence=persistence,
                    lacunarity=lacunarity,
                    repeatx=self.width,
                    repeaty=self.height,
                    base=base_seed
                )
                
                # Create walls based on noise threshold
                if noise_val > wall_threshold:
                    self.cave_map[y][x] = 1  # Wall
                    self.obstacles.add((x, y))
        
        # Ensure start and goal areas are clear
        self.start_pos = (100, 100)
        self.goal_pos = (self.width - 100, self.height - 100)
        
        # Clear circular areas around start and goal
        self._clear_area(self.start_pos, self.start_goal_clear_radius)
        self._clear_area(self.goal_pos, self.start_goal_clear_radius)
        
        # Add denser stalactites/boulders for a richer obstacle field.
        density_scale = (1.0 + float(np.clip(self.complexity, 0.05, 0.25)) * 2.0) * float(profile['struct_mult'])

        # Add some stalactites (hanging obstacles)
        self._add_stalactites(max(12, int(round(10 * density_scale))))

        # Add scattered boulders for richer cave structure
        self._add_boulders(max(140, int(round(110 * density_scale))))

        # Guarantee at least one navigable corridor from start to goal
        self._carve_guaranteed_path()

        # Optional: place additional scattered blockers near direct start->goal line.
        # Default is OFF to keep the traversable corridor visually clean.
        if self.scatter_obstacles_in_path:
            self._add_line_of_sight_blockers()

        # Mark cave zones with symbolic terrain hints.
        self._generate_terrain_markers()
        
        print(f"✅ Cave generated: {len(self.obstacles)} obstacle points")

    def _is_free_zone(self, x, y, radius=6):
        """Return True if a circular area is obstacle-free and not near start/goal."""
        sx, sy = self.start_pos
        gx, gy = self.goal_pos
        if (x - sx) ** 2 + (y - sy) ** 2 < (self.start_goal_clear_radius + 20) ** 2:
            return False
        if (x - gx) ** 2 + (y - gy) ** 2 < (self.start_goal_clear_radius + 20) ** 2:
            return False

        # Keep guaranteed corridor clear of symbol hazards.
        corridor_guard = max(10, self.corridor_radius - 8)
        guard_sq = corridor_guard * corridor_guard
        for px, py in self.safe_corridor_points:
            if (x - px) ** 2 + (y - py) ** 2 <= guard_sq:
                return False

        for yy in range(max(0, y - radius), min(self.height, y + radius + 1)):
            for xx in range(max(0, x - radius), min(self.width, x + radius + 1)):
                if (xx - x) ** 2 + (yy - y) ** 2 <= radius * radius:
                    if self.cave_map[yy][xx] == 1:
                        return False
        return True

    def _generate_terrain_markers(self):
        """Place symbolic terrain markers on free cave floor."""
        self.terrain_markers = {k: [] for k in self.terrain_markers}
        marker_scale = 1.35 * float(self.obstacle_profiles[self.obstacle_level]['marker_mult'])
        marker_counts = {
            'water': int(round(10 * marker_scale)),
            'unstable_ground': int(round(8 * marker_scale)),
            'hole': int(round(6 * marker_scale)),
            'narrow_space': int(round(8 * marker_scale)),
            'jump_space': int(round(6 * marker_scale)),
            'debris': int(round(10 * marker_scale)),
        }
        symbol_radius = max(5, self.robot_radius + 1)

        for marker_type, target_count in marker_counts.items():
            placed = 0
            attempts = 0
            while placed < target_count and attempts < target_count * 140:
                attempts += 1
                x = random.randint(25, self.width - 25)
                y = random.randint(25, self.height - 25)
                if not self._is_free_zone(x, y, radius=6):
                    continue
                self.terrain_markers[marker_type].append((x, y))
                self._mark_symbol_obstacle((x, y), symbol_radius)
                placed += 1

    def _mark_symbol_obstacle(self, center, radius):
        """Convert a symbolic marker into a real circular obstacle."""
        cx, cy = center
        r_sq = radius * radius
        for y in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
            for x in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                dx = x - cx
                dy = y - cy
                if dx * dx + dy * dy <= r_sq:
                    self.cave_map[y][x] = 1
                    self.obstacles.add((x, y))

    def _mark_stretched_obstacle(self, center, major_radius, minor_radius, angle_rad, roughness=0.24):
        """Paint a noisy stretched ellipse (cloud-like obstacle blob)."""
        cx, cy = center
        major_radius = max(3, int(major_radius))
        minor_radius = max(2, int(minor_radius))

        cos_a = float(np.cos(angle_rad))
        sin_a = float(np.sin(angle_rad))

        # Per-shape noise seed so each stretched blob has a unique edge.
        noise_seed = random.randint(0, 10_000)
        pad = max(major_radius, minor_radius) + 2

        for y in range(max(0, cy - pad), min(self.height, cy + pad + 1)):
            for x in range(max(0, cx - pad), min(self.width, cx + pad + 1)):
                dx = x - cx
                dy = y - cy

                # Rotate into local ellipse frame.
                local_major = dx * cos_a + dy * sin_a
                local_minor = -dx * sin_a + dy * cos_a

                base = (local_major / major_radius) ** 2 + (local_minor / minor_radius) ** 2
                edge_noise = pnoise2(
                    (x + noise_seed) / 18.0,
                    (y - noise_seed) / 18.0,
                    octaves=2,
                    persistence=0.5,
                    lacunarity=2.0,
                    repeatx=max(1, self.width),
                    repeaty=max(1, self.height),
                    base=noise_seed,
                )
                threshold = 1.0 + roughness * edge_noise

                if base <= threshold:
                    self.cave_map[y][x] = 1
                    self.obstacles.add((x, y))

    def get_marker_legend(self):
        """Legend metadata for UI rendering."""
        return [
            ('water', '~', (80, 150, 255)),
            ('unstable_ground', '!', (245, 190, 90)),
            ('hole', 'O', (60, 60, 60)),
            ('narrow_space', '||', (210, 120, 255)),
            ('jump_space', '^', (120, 255, 140)),
            ('debris', '*', (190, 190, 190)),
        ]

    def _carve_guaranteed_path(self):
        """Carve a safe curved corridor from start to goal."""
        sx, sy = self.start_pos
        gx, gy = self.goal_pos

        # Two-segment path with a deliberate bend to avoid a straight shot.
        mid_x = (sx + gx) // 2
        midpoint_y = (sy + gy) // 2
        bend_dir = random.choice([-1, 1])
        min_bend = max(80, self.corridor_radius * 2)
        bend = bend_dir * random.randint(min_bend, min_bend + 50)
        mid_y = int(np.clip(midpoint_y + bend, 60, self.height - 60))
        points = [(sx, sy), (mid_x, mid_y), (gx, gy)]

        corridor_radius = self.corridor_radius
        for i in range(len(points) - 1):
            x1, y1 = points[i]
            x2, y2 = points[i + 1]
            steps = max(abs(x2 - x1), abs(y2 - y1), 1)
            for t in range(0, steps + 1, 3):
                x = int(x1 + (x2 - x1) * (t / steps))
                y = int(y1 + (y2 - y1) * (t / steps))
                self._clear_area((x, y), corridor_radius)
                self.safe_corridor_points.add((x, y))

    def _add_line_of_sight_blockers(self):
        """Place obstacle clusters along the direct start->goal segment."""
        sx, sy = self.start_pos
        gx, gy = self.goal_pos
        profile = self.obstacle_profiles[self.obstacle_level]
        level = self.obstacle_level

        # Heavier modes intentionally occlude the straight line much more.
        # Boosted counts to create visibly denser scattered fields around path.
        base_count = max(8, int(round(10 * float(profile['struct_mult']))))
        if level == 'light':
            blocker_count = base_count + 1
        elif level == 'medium':
            blocker_count = base_count + 3
        elif level == 'heavy':
            blocker_count = base_count + 6
        else:  # extra-heavy
            blocker_count = base_count + 10

        blocker_radius = max(18, self.corridor_radius - 4)
        if level == 'medium':
            blocker_radius += 1
        elif level == 'heavy':
            blocker_radius += 2
        elif level == 'extra-heavy':
            blocker_radius += 4

        corridor_guard = max(14, self.corridor_radius - 8)
        guard_sq = corridor_guard * corridor_guard

        # Build one or more blocker bands around the direct segment.
        seg_dx = gx - sx
        seg_dy = gy - sy
        seg_len = max(float(np.hypot(seg_dx, seg_dy)), 1.0)
        n_x = -seg_dy / seg_len
        n_y = seg_dx / seg_len
        tangent_angle = float(np.arctan2(seg_dy, seg_dx))

        lateral_offsets = [0]
        if level == 'medium':
            spread = int(max(10, self.corridor_radius * 0.45))
            lateral_offsets.extend([-spread, spread])
        elif level == 'heavy':
            spread = int(max(10, self.corridor_radius * 0.50))
            lateral_offsets.extend([-spread, spread])
            spread_far = int(max(16, self.corridor_radius * 0.85))
            lateral_offsets.extend([-spread_far, spread_far])
        elif level == 'extra-heavy':
            spread1 = int(max(12, self.corridor_radius * 0.55))
            spread2 = int(max(20, self.corridor_radius * 0.95))
            spread3 = int(max(28, self.corridor_radius * 1.25))
            lateral_offsets.extend([-spread1, spread1, -spread2, spread2, -spread3, spread3])

        def _try_place_blocker(cx, cy, radius):
            cx = int(np.clip(cx + random.randint(-12, 12), 25, self.width - 25))
            cy = int(np.clip(cy + random.randint(-12, 12), 25, self.height - 25))

            if (cx - sx) ** 2 + (cy - sy) ** 2 < (self.start_goal_clear_radius + 28) ** 2:
                return
            if (cx - gx) ** 2 + (cy - gy) ** 2 < (self.start_goal_clear_radius + 28) ** 2:
                return

            # Keep at least one carved corridor traversable.
            for px, py in self.safe_corridor_points:
                if (cx - px) ** 2 + (cy - py) ** 2 <= guard_sq:
                    return

            # Cloud-like stretched obstacles: elongated mostly along path direction
            # with a little random tilt so it doesn't look too artificial.
            major_radius = int(radius * random.uniform(1.7, 2.7))
            minor_radius = int(radius * random.uniform(0.55, 0.95))
            angle_jitter = np.deg2rad(random.uniform(-22.0, 22.0))
            blob_angle = tangent_angle + angle_jitter
            roughness = random.uniform(0.18, 0.34)

            self._mark_stretched_obstacle(
                (cx, cy),
                major_radius=major_radius,
                minor_radius=minor_radius,
                angle_rad=blob_angle,
                roughness=roughness,
            )

        for i in range(1, blocker_count + 1):
            t = i / (blocker_count + 1)
            base_x = sx + (gx - sx) * t
            base_y = sy + (gy - sy) * t

            for offset in lateral_offsets:
                cx = int(base_x + n_x * offset)
                cy = int(base_y + n_y * offset)

                _try_place_blocker(cx, cy, blocker_radius)

        # Extra scatter pass: random mini-clusters hugging the same line/bands
        # to create a less uniform and denser obstacle field around the path.
        extra_clusters = max(6, blocker_count // 2)
        for _ in range(extra_clusters):
            t = random.uniform(0.08, 0.92)
            base_x = sx + (gx - sx) * t
            base_y = sy + (gy - sy) * t
            offset = random.choice(lateral_offsets)
            cx = int(base_x + n_x * offset)
            cy = int(base_y + n_y * offset)

            cluster_radius = max(10, blocker_radius - random.randint(4, 9))
            _try_place_blocker(cx, cy, cluster_radius)
    
    def _clear_area(self, center, radius):
        """Clear a circular area in the cave"""
        cx, cy = center
        radius_sq = radius * radius
        for y in range(max(0, cy - radius), min(self.height, cy + radius)):
            for x in range(max(0, cx - radius), min(self.width, cx + radius)):
                dx = x - cx
                dy = y - cy
                if (dx * dx + dy * dy) <= radius_sq:
                    self.cave_map[y][x] = 0
                    self.obstacles.discard((x, y))
    
    def _add_stalactites(self, count):
        """Add hanging obstacles (stalactites)"""
        for _ in range(count):
            x = random.randint(150, self.width - 150)
            y = random.randint(50, 150)
            height = random.randint(30, 80)
            width = random.randint(10, 20)
            
            # Create stalactite shape
            for dy in range(height):
                for dx in range(-width // 2, width // 2):
                    if 0 <= y + dy < self.height and 0 <= x + dx < self.width:
                        self.cave_map[y + dy][x + dx] = 1
                        self.obstacles.add((x + dx, y + dy))

    def _add_boulders(self, count):
        """Add circular boulders across the cave for denser obstacle fields."""
        for _ in range(count):
            cx = random.randint(40, self.width - 40)
            cy = random.randint(40, self.height - 40)
            radius = random.randint(4, 10)
            r_sq = radius * radius

            for y in range(max(0, cy - radius), min(self.height, cy + radius + 1)):
                for x in range(max(0, cx - radius), min(self.width, cx + radius + 1)):
                    dx = x - cx
                    dy = y - cy
                    if dx * dx + dy * dy <= r_sq:
                        self.cave_map[y][x] = 1
                        self.obstacles.add((x, y))
    
    def is_collision(self, x, y, radius=10):
        """Check if a point collides with cave walls"""
        x, y = int(x), int(y)
        
        # Check boundaries
        if x < radius or x >= self.width - radius:
            return True
        if y < radius or y >= self.height - radius:
            return True
        
        # Check cave walls in radius
        for dy in range(-radius, radius):
            for dx in range(-radius, radius):
                check_x = x + dx
                check_y = y + dy
                
                if 0 <= check_y < self.height and 0 <= check_x < self.width:
                    if self.cave_map[check_y][check_x] == 1:
                        dist = np.sqrt(dx**2 + dy**2)
                        if dist <= radius:
                            return True
        
        return False
    
    def get_distance_to_wall(self, x, y, angle, max_distance=200):
        """
        Simulate ultrasonic/LIDAR sensor
        Returns distance to nearest wall in given direction
        """
        x, y = int(x), int(y)
        dx = np.cos(angle)
        dy = np.sin(angle)
        
        for distance in range(1, max_distance):
            check_x = int(x + dx * distance)
            check_y = int(y + dy * distance)
            
            if check_x < 0 or check_x >= self.width:
                return distance
            if check_y < 0 or check_y >= self.height:
                return distance
            
            if self.cave_map[check_y][check_x] == 1:
                return distance
        
        return max_distance
    
    def render(self, screen):
        """Render cave environment to pygame screen"""
        # Draw cave walls (dark gray/brown)
        for y in range(self.height):
            for x in range(self.width):
                if self.cave_map[y][x] == 1:
                    color = (60 + random.randint(-10, 10), 
                            50 + random.randint(-10, 10), 
                            40 + random.randint(-10, 10))
                    pygame.draw.rect(screen, color, (x, y, 1, 1))
        
        # Draw start position (green)
        pygame.draw.circle(screen, (0, 255, 0), self.start_pos, 30, 2)
        
        # Draw goal position (gold)
        pygame.draw.circle(screen, (255, 215, 0), self.goal_pos, 30, 2)

        # Draw symbolic cave markers
        self._render_terrain_markers(screen)

    def _render_terrain_markers(self, screen):
        """Draw meaningful terrain symbols on top of obstacle hazard circles."""
        # Base: every marker is still an avoidable obstacle zone.
        all_points = []
        for points in self.terrain_markers.values():
            all_points.extend(points)
        for x, y in all_points:
            pygame.draw.circle(screen, (130, 130, 130), (x, y), 4)
            pygame.draw.circle(screen, (200, 200, 200), (x, y), 4, 1)

        # Water (~)
        for x, y in self.terrain_markers.get('water', []):
            pygame.draw.arc(screen, (80, 150, 255), (x - 4, y - 2, 4, 4), np.pi * 0.1, np.pi * 0.9, 1)
            pygame.draw.arc(screen, (80, 150, 255), (x, y - 2, 4, 4), np.pi * 0.1, np.pi * 0.9, 1)

        # Unstable ground (!)
        for x, y in self.terrain_markers.get('unstable_ground', []):
            pygame.draw.line(screen, (245, 190, 90), (x, y - 3), (x, y + 1), 1)
            pygame.draw.circle(screen, (245, 190, 90), (x, y + 3), 1)

        # Hole (O)
        for x, y in self.terrain_markers.get('hole', []):
            pygame.draw.circle(screen, (35, 35, 35), (x, y), 2)
            pygame.draw.circle(screen, (70, 70, 70), (x, y), 3, 1)

        # Narrow space (||)
        for x, y in self.terrain_markers.get('narrow_space', []):
            pygame.draw.line(screen, (210, 120, 255), (x - 1, y - 3), (x - 1, y + 3), 1)
            pygame.draw.line(screen, (210, 120, 255), (x + 1, y - 3), (x + 1, y + 3), 1)

        # Jump space (^)
        for x, y in self.terrain_markers.get('jump_space', []):
            pygame.draw.polygon(screen, (120, 255, 140), [(x, y - 3), (x - 3, y + 2), (x + 3, y + 2)], 1)

        # Debris (*)
        for x, y in self.terrain_markers.get('debris', []):
            pygame.draw.line(screen, (190, 190, 190), (x - 2, y), (x + 2, y), 1)
            pygame.draw.line(screen, (190, 190, 190), (x, y - 2), (x, y + 2), 1)
            pygame.draw.line(screen, (190, 190, 190), (x - 2, y - 2), (x + 2, y + 2), 1)
