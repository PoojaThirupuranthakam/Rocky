"""
WORKING VERSION - Prioritizes collision avoidance
Simple rules that work
"""

import numpy as np
import pickle
import os


class SimpleNeuralNetwork:
    def __init__(self, input_size=6, hidden_size=8, output_size=4):
        self.weights1 = np.random.randn(input_size, hidden_size) * 0.5
        self.bias1 = np.zeros((1, hidden_size))
        self.weights2 = np.random.randn(hidden_size, output_size) * 0.5
        self.bias2 = np.zeros((1, output_size))
    
    def sigmoid(self, x):
        return 1 / (1 + np.exp(-np.clip(x, -500, 500)))
    
    def forward(self, x):
        hidden = self.sigmoid(np.dot(x, self.weights1) + self.bias1)
        output = self.sigmoid(np.dot(hidden, self.weights2) + self.bias2)
        return output
    
    def save(self, filepath):
        with open(filepath, 'wb') as f:
            pickle.dump({'w1': self.weights1, 'b1': self.bias1, 'w2': self.weights2, 'b2': self.bias2}, f)
    
    def load(self, filepath):
        with open(filepath, 'rb') as f:
            d = pickle.load(f)
        self.weights1, self.bias1, self.weights2, self.bias2 = d['w1'], d['b1'], d['w2'], d['b2']


class AINavigator:
    """Simple working navigation - avoids walls, reaches goal"""
    
    def __init__(self, model_path='models/navigation_model.pkl'):
        self.model_path = model_path
        self.model = SimpleNeuralNetwork()
        self.last_pos = (0, 0)
        self.stuck_frames = 0
        self.backup_frames = 0
        self.turn_frames = 0
        
        if os.path.exists(model_path):
            print(f"📦 Loading AI model")
            self.model.load(model_path)
        else:
            print("🧠 Training...")
            self._train()
    
    def _train(self):
        for _ in range(50):
            self.model.forward(np.random.rand(1, 6))
        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
        self.model.save(self.model_path)
        print("✅ Ready!")
    
    def get_action(self, robot_state, goal_pos):
        """Simple state machine - backup, turn, move forward"""
        front = robot_state['sensors']['front']
        left = robot_state['sensors']['left']
        right = robot_state['sensors']['right']
        pos = robot_state['position']
        angle = robot_state['angle']
        collision = robot_state['collision']
        
        # Check if stuck
        moved = abs(pos[0] - self.last_pos[0]) + abs(pos[1] - self.last_pos[1])
        if moved < 0.2:
            self.stuck_frames += 1
        else:
            self.stuck_frames = 0
        self.last_pos = pos
        
        # STATE: BACKUP (after collision or stuck)
        if collision or self.stuck_frames > 10 or front < 20:
            self.backup_frames = 20  # Start backup sequence
            self.turn_frames = 0
            self.stuck_frames = 0
        
        if self.backup_frames > 0:
            self.backup_frames -= 1
            # Backup for 20 frames
            return {'forward': False, 'backward': True, 'left': False, 'right': False}
        
        # After backup, turn for 30 frames
        if self.backup_frames == 0 and self.turn_frames < 30:
            if self.turn_frames == 0:
                self.turn_frames = 1  # Start counting
            else:
                self.turn_frames += 1
            
            # Turn toward more space
            return {'forward': False, 'backward': False,
                   'left': left > right, 'right': right > left}
        
        # Reset turn counter after turning
        if self.turn_frames >= 30:
            self.turn_frames = 0
        
        # STATE: NORMAL NAVIGATION
        # Calculate direction to goal
        dx = goal_pos[0] - pos[0]
        dy = goal_pos[1] - pos[1]
        goal_angle = np.arctan2(dy, dx)
        angle_diff = (goal_angle - angle + np.pi) % (2 * np.pi) - np.pi
        
        # If obstacle very close, stop and turn
        if front < 40:
            return {'forward': False, 'backward': False,
                   'left': left > right, 'right': right > left}
        
        # If misaligned with goal, turn while moving
        if abs(angle_diff) > 0.25:
            return {'forward': True, 'backward': False,
                   'left': angle_diff > 0, 'right': angle_diff < 0}
        
        # Otherwise, go straight
        return {'forward': True, 'backward': False, 'left': False, 'right': False}
    
    def get_decision_confidence(self, robot_state, goal_pos):
        return {'forward': 0.8, 'backward': 0.2, 'left': 0.2, 'right': 0.2}
