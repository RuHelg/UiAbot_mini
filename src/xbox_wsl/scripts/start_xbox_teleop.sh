#!/bin/bash
# Auto-configure and start Xbox controller for ROS2

echo "========================================="
echo "Xbox Controller Teleop Setup"
echo "========================================="
echo ""

# Start xboxdrv in background
echo "1. Starting xboxdrv..."
sudo pkill xboxdrv 2>/dev/null
sleep 1
sudo xboxdrv --detach-kernel-driver --silent &
XBOXDRV_PID=$!
sleep 2

# Fix permissions
echo "2. Fixing permissions..."
sudo chmod 666 /dev/input/event0 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Permissions fixed"
else
    echo "   ✗ Failed to fix permissions (is xboxdrv running?)"
    kill $XBOXDRV_PID 2>/dev/null
    exit 1
fi

# Source workspace
echo "3. Sourcing workspace..."
source ~/ros2_ws/install/setup.bash
echo "   ✓ Workspace sourced"

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

# Start teleop_twist_joy node with config file (use absolute path and set node name)
echo "5. Starting teleop_twist_joy node..."
TELEOP_PARAMS_FILE="$HOME/ros2_ws/src/xbox_wsl/config/xbox_teleop.yaml"
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