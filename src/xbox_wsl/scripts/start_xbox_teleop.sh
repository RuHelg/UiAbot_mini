#!/bin/bash
# Auto-configure and start Xbox controller for ROS2

echo "========================================="
echo "Xbox Controller Teleop Setup"
echo "========================================="
echo ""

# Find workspace root automatically by looking for install/setup.bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$SCRIPT_DIR"

# Search upwards for install/setup.bash
while [ "$WS_ROOT" != "/" ]; do
    if [ -f "$WS_ROOT/install/setup.bash" ]; then
        break
    fi
    WS_ROOT="$(dirname "$WS_ROOT")"
done

# Source workspace
echo "1. Sourcing workspace..."
if [ -f "$WS_ROOT/install/setup.bash" ]; then
    source "$WS_ROOT/install/setup.bash"
    echo "   ✓ Workspace sourced from: $WS_ROOT"
else
    echo "   ✗ Could not find workspace with install/setup.bash"
    echo "   Searched from: $SCRIPT_DIR"
    exit 1
fi

# Find config file using ros2 pkg prefix
PACKAGE_SHARE=$(ros2 pkg prefix xbox_wsl 2>/dev/null)/share/xbox_wsl
TELEOP_PARAMS_FILE="$PACKAGE_SHARE/config/xbox_teleop.yaml"

if [ ! -f "$TELEOP_PARAMS_FILE" ]; then
    # Fallback to source directory
    TELEOP_PARAMS_FILE="$SCRIPT_DIR/../config/xbox_teleop.yaml"
    echo "   Using config from source: $TELEOP_PARAMS_FILE"
fi

if [ ! -f "$TELEOP_PARAMS_FILE" ]; then
    echo "   ✗ Could not find xbox_teleop.yaml config file"
    exit 1
fi

# Start xboxdrv in background
echo "2. Starting xboxdrv..."
sudo pkill xboxdrv 2>/dev/null
sleep 1
sudo xboxdrv --detach-kernel-driver --silent &
XBOXDRV_PID=$!
sleep 2

# Fix permissions
echo "3. Fixing permissions..."
sudo chmod 666 /dev/input/event0 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Permissions fixed"
else
    echo "   ✗ Failed to fix permissions (is xboxdrv running?)"
    kill $XBOXDRV_PID 2>/dev/null
    exit 1
fi

# Start joy node in background
echo "4. Starting joy_node..."
ros2 run joy joy_node --ros-args -p dev:=/dev/input/event0 &
JOY_PID=$!
sleep 2

if ps -p $JOY_PID > /dev/null; then
    echo "   ✓ Joy node started (PID: $JOY_PID)"
else
    echo "   ✗ Joy node failed to start"
    kill $XBOXDRV_PID 2>/dev/null
    exit 1
fi

# Start teleop_twist_joy node with config file
echo "5. Starting teleop_twist_joy node..."
ros2 run teleop_twist_joy teleop_node \
    --ros-args \
    --params-file "${TELEOP_PARAMS_FILE}" \
    -r __node:=teleop_twist_joy &
TELEOP_PID=$!
sleep 2

if ps -p $TELEOP_PID > /dev/null; then
    echo "   ✓ Teleop node started (PID: $TELEOP_PID)"
else
    echo "   ✗ Teleop node failed to start"
    kill $JOY_PID $XBOXDRV_PID 2>/dev/null
    exit 1
fi

echo ""
echo "========================================="
echo "Xbox Controller Ready!"
echo "========================================="
echo "Move the LEFT STICK to control the robot:"
echo "  - Up/Down: Forward/Backward"
echo "  - Left/Right: Rotate"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Wait for Ctrl+C
trap "echo ''; echo 'Stopping all nodes...'; kill $JOY_PID $TELEOP_PID 2>/dev/null; sudo pkill xboxdrv; exit 0" INT
wait