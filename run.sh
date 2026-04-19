#!/bin/bash

# 🤖 Cave Robot - Quick Run Script
# Just run this to start the simulation!

echo "🤖 Starting Cave Explorer Robot Simulator..."
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

# Check if in correct directory
if [ ! -f "src/run_simulation.py" ]; then
    echo "❌ Error: Could not find src/run_simulation.py in $SCRIPT_DIR"
    exit 1
fi

# Check if venv exists
if [ ! -d "venv" ]; then
    echo "⚠️  Virtual environment not found. Running setup..."
    "$SCRIPT_DIR/setup.sh"
fi

# Run simulation
echo "🚀 Launching simulation..."
echo ""
echo "Controls:"
echo "  W/S: Forward/Backward"
echo "  A/D: Turn Left/Right"
echo "  SPACE: Toggle AI Mode"
echo "  H: Toggle Debug Info"
echo "  R: Reset"
echo "  Q: Quit"
echo ""

"$SCRIPT_DIR/venv/bin/python" "$SCRIPT_DIR/src/run_simulation.py"
