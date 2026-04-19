"""
AI Navigator - Simple and Robust Version
Uses basic state machine with clear logic - NO OSCILLATION
"""

import numpy as np
import pickle
import os


class SimpleNeuralNetwork:
    """Lightweight neural network for embedded systems"""
    
    def __init__(self, input_size=6, hidden_size=8, output_size=4):
        self.input_size = input_size
        self.hidden_size = hidden_size
        self.output_size = output_size
        
        # Initialize weights randomly
        self.weights1 = np.random.randn(input_size, hidden_size) * 0.5
        self.bias1 = np.zeros((1, hidden_size))
        self.weights2 = np.random.randn(hidden_size, output_size) * 0.5
        self.bias2 = np.zeros((1, output_size))
    
    def sigmoid(self, x):
        return 1 / (1 + np.exp(-np.clip(x, -500, 500)))
    
    def forward(self, x):
        """Forward pass through network"""
        hidden = self.sigmoid(np.dot(x, self.weights1) + self.bias1)
        output = self.sigmoid(np.dot(hidden, self.weights2) + self.bias2)
        return output
    
    def save(self, filepath):
        model_data = {
            'weights1': self.weights1,
            'bias1': self.bias1,
            'weights2': self.weights2,
            'bias2': self.bias2
        }
        with open(filepath, 'wb') as f:
            pickle.dump(model_data, f)
    
    def load(self, filepath):
        with open(filepath, 'rb') as f:
            model_data = pickle.load(f)
        self.weights1 = model_data['weights1']
        self.bias1 = model_data['bias1']
        self.weights2 = model_data['weights2']
        self.bias2 = model_data['bias2']


class AINavigator:
    """Simple, robust navigation - state machine based"""
    
    def __init__(self, model_path='models/navigation_model.pkl'):
        self.model_path = model_path
        self.model = SimpleNeuralNetwork()
        
        # State machine variables
        self.action_commitment = 0  # How long to continue current action
        self.current_action = 'FORWARD'
        self.last_pos = None
        self.stuck_frames = 0
        
        # Load or train model
        if os.path.exists(model_path):
            print(f"📦 Loading AI model from {model_path}")
            self.model.load(model_path)
        else:
            print("🧠 Training basic model...")
            self._train_basic_model()
    
    def _train_basic_model(self):
        """Quick training on basic patterns"""
        training_data = []
        
        # Pattern 1: Clear ahead -> go forward
        for _ in range(50):
            front_dist = np.random.uniform(100, 150)
            left_dist = np.random.uniform(80, 150)
            right_dist = np.random.uniform(80, 150)
            inputs = np.array([front_dist, left_dist, right_dist, 0, 100, 1])
            outputs = np.array([1, 0, 0, 0])  # forward
            training_data.append((inputs, outputs))
        
        # Pattern 2: Blocked ahead, clear left -> turn left
        for _ in range(50):
            front_dist = np.random.uniform(0, 40)
            left_dist = np.random.uniform(100, 150)
            right_dist = np.random.uniform(20, 60)
            inputs = np.array([front_dist, left_dist, right_dist, 0.5, 100, 0])
            outputs = np.array([0, 0, 1, 0])  # left
            training_data.append((inputs, outputs))
        
        # Pattern 3: Blocked ahead, clear right -> turn right
        for _ in range(50):
            front_dist = np.random.uniform(0, 40)
            left_dist = np.random.uniform(20, 60)
            right_dist = np.random.uniform(100, 150)
            inputs = np.array([front_dist, left_dist, right_dist, -0.5, 100, 0])
            outputs = np.array([0, 0, 0, 1])  # right
            training_data.append((inputs, outputs))
        
        # Train
        learning_rate = 0.01
        for epoch in range(100):
            for inputs, targets in training_data:
                inputs_2d = inputs.reshape(1, -1)
                predicted = self.model.forward(inputs_2d)
                error = (targets.reshape(1, -1) - predicted)
                
                hidden = self.model.sigmoid(np.dot(inputs_2d, self.model.weights1) + self.model.bias1)
                self.model.weights2 += learning_rate * hidden.T @ error
                self.model.bias2 += learning_rate * error
                
                hidden_error = error @ self.model.weights2.T
                self.model.weights1 += learning_rate * inputs_2d.T @ (hidden_error * hidden * (1 - hidden))
                self.model.bias1 += learning_rate * (hidden_error * hidden * (1 - hidden))
        
        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
        self.model.save(self.model_path)
        print("✅ Model trained!")
    
    def get_action(self, robot_state, goal_pos):
        """
        SIMPLE DECISION LOGIC:
        1. If committed to an action, continue it
        2. If stuck, backup
        3. If obstacle close, turn away
        4. Otherwise, go forward toward goal
        """
        sensors = robot_state['sensors']
        pos = robot_state['position']
        angle = robot_state['angle']
        
        front_dist = sensors['front']
        left_dist = sensors['left']
        right_dist = sensors['right']
        
        # Check if stuck (not moving)
        if self.last_pos is not None:
            moved = ((pos[0] - self.last_pos[0])**2 + (pos[1] - self.last_pos[1])**2)**0.5
            if moved < 0.3:
                self.stuck_frames += 1
            else:
                self.stuck_frames = 0
        self.last_pos = pos
        
        # If we're committed to an action, continue it
        if self.action_commitment > 0:
            self.action_commitment -= 1
            
            if self.current_action == 'BACKUP':
                return {'forward': False, 'backward': True, 'left': False, 'right': False}
            elif self.current_action == 'TURN_LEFT':
                return {'forward': False, 'backward': False, 'left': True, 'right': False}
            elif self.current_action == 'TURN_RIGHT':
                return {'forward': False, 'backward': False, 'left': False, 'right': True}
            elif self.current_action == 'FORWARD':
                return {'forward': True, 'backward': False, 'left': False, 'right': False}
        
        # DECISION TIME - choose new action
        
        # Priority 1: STUCK - backup and turn
        if self.stuck_frames > 20:
            self.current_action = 'BACKUP'
            self.action_commitment = 20
            self.stuck_frames = 0
            return self.get_action(robot_state, goal_pos)
        
        # Priority 2: COLLISION IMMINENT - backup
        if robot_state['collision'] or front_dist < 25:
            self.current_action = 'BACKUP'
            self.action_commitment = 12
            return self.get_action(robot_state, goal_pos)
        
        # Priority 3: OBSTACLE CLOSE - turn away
        if front_dist < 60:
            if left_dist > right_dist:
                self.current_action = 'TURN_LEFT'
                self.action_commitment = 15
            else:
                self.current_action = 'TURN_RIGHT'
                self.action_commitment = 15
            return self.get_action(robot_state, goal_pos)
        
        # Priority 4: PATH CLEAR - navigate toward goal
        # Calculate goal direction
        dx = goal_pos[0] - pos[0]
        dy = goal_pos[1] - pos[1]
        goal_angle = np.arctan2(dy, dx)
        angle_diff = (goal_angle - angle + np.pi) % (2 * np.pi) - np.pi
        
        # If facing wrong direction, turn
        if abs(angle_diff) > 0.5:  # More than ~30 degrees off
            if angle_diff > 0:
                self.current_action = 'TURN_LEFT'
                self.action_commitment = 10
            else:
                self.current_action = 'TURN_RIGHT'
                self.action_commitment = 10
            return self.get_action(robot_state, goal_pos)
        
        # Default: move forward
        self.current_action = 'FORWARD'
        self.action_commitment = 5  # Small commitment to avoid jitter
        return {'forward': True, 'backward': False, 'left': False, 'right': False}
    
    def get_decision_confidence(self, robot_state, goal_pos):
        """Return confidence scores for UI"""
        return {
            'forward': 0.8 if self.current_action == 'FORWARD' else 0.2,
            'backward': 0.8 if self.current_action == 'BACKUP' else 0.1,
            'left': 0.8 if self.current_action == 'TURN_LEFT' else 0.1,
            'right': 0.8 if self.current_action == 'TURN_RIGHT' else 0.1
        }
