#!/bin/bash
# Auto-configure and start Xbox controller for ROS2

echo "========================================="
echo "Xbox Controller Teleop Setup"
echo "========================================="
echo ""

# Fix permissions
echo "1. Fixing permissions..."
sudo chmod 666 /dev/input/event0 2>/dev/null
if [ $? -eq 0 ]; then
    echo "   ✓ Permissions fixed"
else
    echo "   ✗ Failed to fix permissions (is xboxdrv running?)"
    exit 1
fi

# Source workspace
echo "2. Sourcing workspace..."
source ~/ros2_ws/install/setup.bash
echo "   ✓ Workspace sourced"

# Start joy node in background
echo "3. Starting xbox_joy_node..."
~/ros2_ws/src/xbox_joy_node.py &
JOY_PID=$!
sleep 2

if ps -p $JOY_PID > /dev/null; then
    echo "   ✓ Joy node started (PID: $JOY_PID)"
else
    echo "   ✗ Joy node failed to start"
    exit 1
fi

# Wait for teleop_twist_joy_node to be available
echo "4. Waiting for teleop_twist_joy_node..."
for i in {1..10}; do
    if ros2 node list 2>/dev/null | grep -q teleop_twist_joy_node; then
        echo "   ✓ Teleop node found"
        break
    fi
    if [ $i -eq 10 ]; then
        echo "   ✗ Teleop node not found. Is your robot launch file running?"
        echo "   Start it with: ros2 launch uiabot_mini_bringup uiabot_mini.launch.py"
        kill $JOY_PID
        exit 1
    fi
    sleep 1
done

# Configure teleop parameters
echo "5. Configuring teleop parameters..."
ros2 param set /teleop_twist_joy_node require_enable_button false >/dev/null 2>&1
ros2 param set /teleop_twist_joy_node axis_linear.x 1 >/dev/null 2>&1
ros2 param set /teleop_twist_joy_node axis_angular.yaw 0 >/dev/null 2>&1
ros2 param set /teleop_twist_joy_node scale_linear.x 0.35 >/dev/null 2>&1
ros2 param set /teleop_twist_joy_node scale_angular.yaw 3.14 >/dev/null 2>&1
echo "   ✓ Parameters configured"

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
trap "echo ''; echo 'Stopping joy node...'; kill $JOY_PID; exit 0" INT
wait $JOY_PID
