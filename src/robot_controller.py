"""
Robot Controller - Handles robot movement and physics
"""

import pygame
import numpy as np


class RobotController:
    """Simulates a differential drive robot with realistic physics"""
    
    def __init__(
        self,
        x,
        y,
        cave_env,
        sensor_layout='lidar_top_ground_ultrasonic',
        robot_radius=5,
        speed_scale=1.0,
    ):
        self.x = x
        self.y = y
        self.angle = 0  # Heading in radians
        self.speed = 0
        self.angular_velocity = 0
        
        # Robot physical properties
        self.radius = max(4, int(robot_radius))  # 7x5x5in baseline footprint (default radius≈13px)
        self.base_max_speed = 2.35  # Slightly faster to reduce timeout risk
        self.base_max_angular_velocity = 0.072  # Improve turning in tighter corridors
        self.base_acceleration = 0.18  # Reach useful speed sooner
        self.friction = 0.91  # Keep stability while preserving momentum
        self.speed_scale = 1.0
        self.max_speed = self.base_max_speed
        self.max_angular_velocity = self.base_max_angular_velocity
        self.acceleration = self.base_acceleration
        self.set_speed_scale(speed_scale)
        
        # Environment reference
        self.cave_env = cave_env
        self.sensor_layout = sensor_layout

        # Ultrasonic sensing horizon: 120 cm (~3.94 ft), converted to pixels.
        # Keep stop/detour policy at 1 foot in navigator logic, but allow sensing farther ahead.
        cm_per_pixel = float(getattr(self.cave_env, 'cm_per_pixel', 0.5) or 0.5)
        self.ultrasonic_max_distance_px = max(1, int(round(120.0 / cm_per_pixel)))
        
        # Sensor readings
        self.sensor_distances = {
            'top': 0,
            'ground': 0,
            'back': 0,
            # Sector breakdown from top sensor arc (for navigation decisions).
            'top_left': 0,
            'top_front': 0,
            'top_right': 0,
            # Optional ground arc sector breakdown.
            'ground_left': 0,
            'ground_front': 0,
            'ground_right': 0,
            'ground_ultrasonic': 0,
            # 2D floor LiDAR profile
            'lidar_left': 0,
            'lidar_front': 0,
            'lidar_right': 0,
            'lidar_min': 0,
            'lidar_mean': 0,
            'lidar_std': 0,
            'lidar_scan': [],
        }
        
        # State
        self.collision = False
        self.battery = 100.0

        # Prime sensors immediately so first AI frame doesn't see all zeros.
        self.update_sensors()

    def set_speed_scale(self, speed_scale):
        """Set global speed multiplier for linear + angular movement."""
        try:
            scale = float(speed_scale)
        except Exception:
            scale = 1.0

        # Keep a practical range for controllability and simulation stability.
        scale = max(0.25, min(3.0, scale))
        self.speed_scale = scale
        self.max_speed = self.base_max_speed * scale
        self.max_angular_velocity = self.base_max_angular_velocity * scale
        self.acceleration = self.base_acceleration * scale

    def adjust_speed_scale(self, delta):
        """Increment/decrement speed multiplier by delta and return new scale."""
        self.set_speed_scale(self.speed_scale + float(delta))
        return self.speed_scale
        
    def update_sensors(self):
        """Update all sensor readings"""
        if self.sensor_layout in ('lidar_top_ground_ultrasonic', 'lidar_2d_floor'):
            self._update_lidar_top_ground_ultrasonic_sensors()
            return

        # Two-sensor architecture: top + floor, each 180° frontal arc.
        self._update_dual_arc_180_sensors()

    def _distance(self, angle, max_distance=None):
        if max_distance is None:
            max_distance = self.ultrasonic_max_distance_px
        return self.cave_env.get_distance_to_wall(
            self.x, self.y, angle, max_distance=max_distance
        )

    def _update_dual_arc_180_sensors(self):
        """Two sensors only: top and ground, each scanning a 180° frontal arc."""
        arc_degs = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]

        samples = [self._distance(self.angle + np.deg2rad(d)) for d in arc_degs]

        left_idx = [0, 1, 2, 3, 4]
        front_idx = [4, 5, 6, 7, 8]
        right_idx = [8, 9, 10, 11, 12]

        top_left = min(samples[i] for i in left_idx)
        top_front = min(samples[i] for i in front_idx)
        top_right = min(samples[i] for i in right_idx)
        top_min = min(samples)

        # Ground sensor in a top-down sim is approximated conservatively from the same arc.
        # We keep it slightly conservative by taking the minimum over the same sectors.
        ground_left = top_left
        ground_front = top_front
        ground_right = top_right
        ground_min = min(ground_left, ground_front, ground_right)

        back = self._distance(self.angle + np.pi)

        self.sensor_distances['top'] = top_min
        self.sensor_distances['ground'] = ground_min
        self.sensor_distances['back'] = back
        self.sensor_distances['top_left'] = top_left
        self.sensor_distances['top_front'] = top_front
        self.sensor_distances['top_right'] = top_right
        self.sensor_distances['ground_left'] = ground_left
        self.sensor_distances['ground_front'] = ground_front
        self.sensor_distances['ground_right'] = ground_right

    def _update_lidar_top_ground_ultrasonic_sensors(self):
        """Hybrid sensing: top-mounted 2D LiDAR (360°) + ground-facing ultrasonic."""
        beam_count = 36
        rel_angles = np.linspace(-np.pi, np.pi, beam_count, endpoint=False)
        scan = [
            float(self._distance(self.angle + rel))
            for rel in rel_angles
        ]

        # Sector minima from full 360° scan for navigation-facing directions
        # (front, front-left, front-right), preserving prior control semantics.
        def angle_wrap(rad):
            return (rad + np.pi) % (2 * np.pi) - np.pi

        sector_half = np.pi / 6.0
        front_vals = [d for d, rel in zip(scan, rel_angles) if abs(angle_wrap(rel)) <= sector_half]
        left_vals = [d for d, rel in zip(scan, rel_angles) if abs(angle_wrap(rel - np.pi / 4.0)) <= sector_half]
        right_vals = [d for d, rel in zip(scan, rel_angles) if abs(angle_wrap(rel + np.pi / 4.0)) <= sector_half]

        lidar_left = min(left_vals) if left_vals else min(scan)
        lidar_front = min(front_vals) if front_vals else min(scan)
        lidar_right = min(right_vals) if right_vals else min(scan)

        lidar_min = min(scan)
        lidar_mean = float(np.mean(scan))
        lidar_std = float(np.std(scan))

        # Ground-facing ultrasonic proxy (single sensor): narrow forward cone,
        # shorter range than LiDAR to mimic near-ground obstacle detection.
        cm_per_pixel = float(getattr(self.cave_env, 'cm_per_pixel', 0.5) or 0.5)
        ground_ultra_max_px = max(1, int(round(70.0 / cm_per_pixel)))
        ground_arc_degs = [-18, -10, -5, 0, 5, 10, 18]
        ground_samples = [
            float(self._distance(self.angle + np.deg2rad(deg), max_distance=ground_ultra_max_px))
            for deg in ground_arc_degs
        ]
        ground_front = min(ground_samples) if ground_samples else min(scan)
        half = max(1, len(ground_samples) // 2)
        ground_left = min(ground_samples[:half]) if ground_samples else ground_front
        ground_right = min(ground_samples[-half:]) if ground_samples else ground_front

        back = self._distance(self.angle + np.pi)

        # Populate LiDAR-specific fields.
        self.sensor_distances['lidar_scan'] = scan
        self.sensor_distances['lidar_left'] = lidar_left
        self.sensor_distances['lidar_front'] = lidar_front
        self.sensor_distances['lidar_right'] = lidar_right
        self.sensor_distances['lidar_min'] = lidar_min
        self.sensor_distances['lidar_mean'] = lidar_mean
        self.sensor_distances['lidar_std'] = lidar_std

        # Backward-compatible fields so existing navigation logic still works.
        self.sensor_distances['top_left'] = lidar_left
        self.sensor_distances['top_front'] = lidar_front
        self.sensor_distances['top_right'] = lidar_right
        self.sensor_distances['ground_left'] = ground_left
        self.sensor_distances['ground_front'] = ground_front
        self.sensor_distances['ground_right'] = ground_right
        self.sensor_distances['ground_ultrasonic'] = ground_front
        self.sensor_distances['top'] = lidar_min
        self.sensor_distances['ground'] = ground_front
        self.sensor_distances['back'] = back

    def _update_lidar_floor_2d_sensors(self):
        """Legacy alias retained for backward compatibility."""
        self._update_lidar_top_ground_ultrasonic_sensors()
    
    def move_forward(self):
        """Accelerate forward"""
        self.speed = min(self.speed + self.acceleration, self.max_speed)
    
    def move_backward(self):
        """Accelerate backward"""
        self.speed = max(self.speed - self.acceleration, -self.max_speed)
    
    def turn_left(self):
        """Turn left"""
        self.angular_velocity = min(
            self.angular_velocity + 0.02,  # Faster angular acceleration
            self.max_angular_velocity
        )
    
    def turn_right(self):
        """Turn right"""
        self.angular_velocity = max(
            self.angular_velocity - 0.02,  # Faster angular acceleration
            -self.max_angular_velocity
        )
    
    def update(self, dt=1.0):
        """Update robot position and physics"""
        # Apply friction
        self.speed *= self.friction
        self.angular_velocity *= 0.9
        
        # Update heading
        self.angle += self.angular_velocity
        self.angle = self.angle % (2 * np.pi)
        
        # Calculate new position
        new_x = self.x + self.speed * np.cos(self.angle)
        new_y = self.y + self.speed * np.sin(self.angle)
        
        # Check collision
        if not self.cave_env.is_collision(new_x, new_y, self.radius):
            self.x = new_x
            self.y = new_y
            self.collision = False
        else:
            self.collision = True
            # Instead of stopping completely, bounce back slightly
            self.speed *= -0.3  # Reverse with reduced speed
            # Try to move away from collision
            bounce_x = self.x - 0.5 * np.cos(self.angle)
            bounce_y = self.y - 0.5 * np.sin(self.angle)
            if not self.cave_env.is_collision(bounce_x, bounce_y, self.radius):
                self.x = bounce_x
                self.y = bounce_y
        
        # Update sensors
        self.update_sensors()
        
        # Drain battery (very slowly)
        self.battery = max(0, self.battery - 0.001)
    
    def get_state(self):
        """Get current robot state for AI"""
        return {
            'position': (self.x, self.y),
            'angle': self.angle,
            'speed': self.speed,
            'sensors': self.sensor_distances,
            'collision': self.collision,
            'battery': self.battery
        }
    
    def render(self, screen):
        """Render robot on screen"""
        # Robot body (circle)
        color = (255, 0, 0) if self.collision else (100, 100, 255)
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), self.radius)
        
        # Direction indicator (line from center)
        end_x = self.x + self.radius * 1.5 * np.cos(self.angle)
        end_y = self.y + self.radius * 1.5 * np.sin(self.angle)
        pygame.draw.line(screen, (255, 255, 0), 
                        (int(self.x), int(self.y)), 
                        (int(end_x), int(end_y)), 3)
        
        # Render sensor beams
        self._render_sensors(screen)
    
    def _render_sensors(self, screen):
        """Visualize sensor readings"""
        if self.sensor_layout in ('lidar_top_ground_ultrasonic', 'lidar_2d_floor'):
            scan = self.sensor_distances.get('lidar_scan', [])
            if not scan:
                return
            rel_angles = np.linspace(-np.pi, np.pi, len(scan), endpoint=False)
            for rel, distance in zip(rel_angles, scan):
                ray_angle = self.angle + rel
                end_x = self.x + distance * np.cos(ray_angle)
                end_y = self.y + distance * np.sin(ray_angle)
                pygame.draw.line(
                    screen,
                    (220, 120, 255, 80),
                    (int(self.x), int(self.y)),
                    (int(end_x), int(end_y)),
                    1,
                )
                pygame.draw.circle(screen, (255, 180, 255), (int(end_x), int(end_y)), 1)

            ground_range = float(self.sensor_distances.get('ground_ultrasonic', 0.0))
            ground_arc_degs = [-18, -10, -5, 0, 5, 10, 18]
            for deg in ground_arc_degs:
                ray_angle = self.angle + np.deg2rad(deg)
                end_x = self.x + ground_range * np.cos(ray_angle)
                end_y = self.y + ground_range * np.sin(ray_angle)
                pygame.draw.line(
                    screen,
                    (120, 255, 120, 90),
                    (int(self.x), int(self.y)),
                    (int(end_x), int(end_y)),
                    2,
                )
                pygame.draw.circle(screen, (120, 255, 120), (int(end_x), int(end_y)), 2)
            return

        if self.sensor_layout in ('dual_arc_180', 'top_ground_180'):
            arc_degs = [-90, -75, -60, -45, -30, -15, 0, 15, 30, 45, 60, 75, 90]
            for deg in arc_degs:
                ray_angle = self.angle + np.deg2rad(deg)
                distance = self._distance(ray_angle)
                end_x = self.x + distance * np.cos(ray_angle)
                end_y = self.y + distance * np.sin(ray_angle)

                pygame.draw.line(
                    screen,
                    (0, 255, 255, 80),
                    (int(self.x), int(self.y)),
                    (int(end_x), int(end_y)),
                    1,
                )
                pygame.draw.circle(screen, (255, 255, 0), (int(end_x), int(end_y)), 2)
            return
