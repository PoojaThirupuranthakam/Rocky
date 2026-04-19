#!/usr/bin/env python3
"""
Visual debugging tool - shows robot navigation in real-time
Press SPACE to run AI, use W/A/S/D for manual control
"""
import sys
sys.path.append('src')

from cave_environment import CaveEnvironment
from robot_controller import RobotController
from ai_navigator import AINavigator
import pygame

print("🎮 MANUAL TESTING MODE")
print("=" * 60)
print("Controls:")
print("  SPACE - Toggle AI auto-navigation")
print("  W - Move forward")
print("  S - Move backward")
print("  A - Turn left")
print("  D - Turn right")
print("  Q - Quit")
print("=" * 60)

pygame.init()
screen = pygame.display.set_mode((800, 600))
pygame.display.set_caption("Cave Robot - Manual Test")
clock = pygame.time.Clock()

# Create environment
cave = CaveEnvironment(800, 600)
robot = RobotController(cave.start_pos[0], cave.start_pos[1], cave)
ai = AINavigator(cave)

ai_mode = False
running = True
frames = 0

while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_SPACE:
                ai_mode = not ai_mode
                print(f"🤖 AI Mode: {'ON' if ai_mode else 'OFF'}")
            if event.key == pygame.K_q:
                running = False
    
    # Get action
    if ai_mode:
        robot_state = {
            'sensors': robot.sensor_distances,
            'position': (robot.x, robot.y),
            'angle': robot.angle,
            'collision': robot.collision
        }
        action = ai.get_action(robot_state, cave.goal_pos)
    else:
        keys = pygame.key.get_pressed()
        action = {
            'forward': keys[pygame.K_w],
            'backward': keys[pygame.K_s],
            'left': keys[pygame.K_a],
            'right': keys[pygame.K_d]
        }
    
    # Update
    robot.update(action, cave.obstacles)
    
    # Render
    screen.fill((20, 20, 30))
    cave.render(screen)
    robot.render(screen)
    
    # Stats
    dx = cave.goal_pos[0] - robot.x
    dy = cave.goal_pos[1] - robot.y
    dist = (dx*dx + dy*dy) ** 0.5
    
    font = pygame.font.Font(None, 24)
    mode_text = "AI MODE" if ai_mode else "MANUAL"
    mode_color = (0, 255, 0) if ai_mode else (255, 255, 0)
    text = font.render(f"{mode_text} | Distance: {dist:.0f}px | Frame: {frames}", True, mode_color)
    screen.blit(text, (10, 10))
    
    pygame.display.flip()
    clock.tick(60)
    frames += 1

pygame.quit()
print(f"\n✅ Test complete after {frames} frames")
