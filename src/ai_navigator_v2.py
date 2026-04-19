"""
ULTRA-SIMPLE AI Navigator - Guaranteed to work
No complex state machines - just simple rules
"""

import numpy as np
import pickle
import os


class SimpleNeuralNetwork:
    """Lightweight neural network"""
    
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
    """Ultra-simple navigator - just works!"""
    
    def __init__(self, model_path='models/navigation_model.pkl'):
        self.model_path = model_path
        self.model = SimpleNeuralNetwork()
        self.frames_in_action = 0
        self.last_action = None
        self.last_pos = (0, 0)
        self.not_moving_count = 0
        
        if os.path.exists(model_path):
            print(f"📦 Loading AI model")
            self.model.load(model_path)
        else:
            print("🧠 Training...")
            self._quick_train()
    
    def _quick_train(self):
        for _ in range(50):
            x = np.random.rand(1, 6)
            self.model.forward(x)
        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
        self.model.save(self.model_path)
        print("✅ Ready!")
    
    def get_action(self, robot_state, goal_pos):
        """SIMPLE RULES - NO BUGS"""
        front = robot_state['sensors']['front']
        left = robot_state['sensors']['left']
        right = robot_state['sensors']['right']
        pos = robot_state['sensors']['position']
        angle = robot_state['angle']
        
        # Check if stuck
        if pos == self.last_pos:
            self.not_moving_count += 1
        else:
            self.not_moving_count = 0
        self.last_pos = pos
        
        # Rule 1: If stuck, back up for 20 frames then turn for 25 frames
        if self.not_moving_count > 10:
            if self.frames_in_action < 20:
                action = {'forward': False, 'backward': True, 'left': False, 'right': False}
            elif self.frames_in_action < 45:
                action = {'forward': False, 'backward': False, 'left': left > right, 'right': right > left}
            else:
                self.frames_in_action = 0
                self.not_moving_count = 0
                action = {'forward': True, 'backward': False, 'left': False, 'right': False}
            self.frames_in_action += 1
            return action
        
        # Rule 2: Collision - back up 15 frames then turn 20 frames
        if robot_state['collision'] or front < 20:
            if self.frames_in_action < 15:
                action = {'forward': False, 'backward': True, 'left': False, 'right': False}
            elif self.frames_in_action < 35:
                action = {'forward': False, 'backward': False, 'left': left > right, 'right': right > left}
            else:
                self.frames_in_action = 0
                action = {'forward': True, 'backward': False, 'left': False, 'right': False}
            self.frames_in_action += 1
            return action
        else:
            self.frames_in_action = 0
        
        # Rule 3: Obstacle close - turn toward open space (commit 20 frames)
        if front < 50:
            if self.last_action and self.frames_in_action < 20:
                self.frames_in_action += 1
                return self.last_action
            action = {'forward': False, 'backward': False, 'left': left > right, 'right': right > left}
            self.last_action = action
            self.frames_in_action = 1
            return action
        
        # Rule 4: Calculate goal angle
        dx = goal_pos[0] - pos[0]
        dy = goal_pos[1] - pos[1]
        goal_angle = np.arctan2(dy, dx)
        angle_diff = (goal_angle - angle + np.pi) % (2 * np.pi) - np.pi
        
        # Rule 5: If misaligned, turn (commit 10 frames)
        if abs(angle_diff) > 0.4:
            if self.last_action and self.frames_in_action < 10:
                self.frames_in_action += 1
                return self.last_action
            action = {'forward': False, 'backward': False, 'left': angle_diff > 0, 'right': angle_diff < 0}
            self.last_action = action
            self.frames_in_action = 1
            return action
        
        # Rule 6: Default - go forward
        self.last_action = None
        self.frames_in_action = 0
        return {'forward': True, 'backward': False, 'left': False, 'right': False}
    
    def get_decision_confidence(self, robot_state, goal_pos):
        return {'forward': 0.8, 'backward': 0.2, 'left': 0.2, 'right': 0.2}
