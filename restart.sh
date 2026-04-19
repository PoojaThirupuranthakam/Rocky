#!/bin/bash

# Quick restart script for cave robot simulation

echo "🔄 Restarting Cave Robot Simulation..."
echo ""

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR" || exit 1

# Kill any existing simulation processes
pkill -f "run_simulation.py" 2>/dev/null

# Small delay
sleep 1

# Delete old AI model to force retraining with new logic
if [ -f "$SCRIPT_DIR/models/navigation_model.pkl" ]; then
    echo "🗑️  Removing old AI model (will retrain with new logic)..."
    rm "$SCRIPT_DIR/models/navigation_model.pkl"
fi

# Restart simulation
echo "🚀 Starting improved simulation..."
echo ""
"$SCRIPT_DIR/run.sh"
