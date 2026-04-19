#!/bin/bash

echo "🤖 Cave Robot Hackathon Setup Script"
echo "====================================="

# Check Python version
echo "📋 Checking Python version..."
python3 --version

# Create virtual environment
echo "🔧 Creating virtual environment..."
python3 -m venv venv

# Activate virtual environment
source venv/bin/activate

# Upgrade pip
echo "⬆️  Upgrading pip..."
pip install --upgrade pip

# Install dependencies
echo "📦 Installing dependencies..."
pip install -r requirements.txt

# Download embedded AI model
echo "🧠 Setting up embedded AI model..."
python3 -c "
import os
os.makedirs('models', exist_ok=True)
print('Models directory created')
"

echo "✅ Setup complete!"
echo ""
echo "Next steps:"
echo "1. source venv/bin/activate"
echo "2. python src/run_simulation.py"
