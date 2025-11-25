#!/bin/bash
# Start Xbox controller driver for WSL2
# This must stay running in the background for the controller to work

# Kill any existing xboxdrv instances
sudo pkill xboxdrv 2>/dev/null
sleep 1

# Fix permissions on event device (needed until you logout/login)
sudo chmod 666 /dev/input/event0 2>/dev/null

echo "========================================="
echo "Starting Xbox Controller Driver for ROS2"
echo "========================================="
echo ""
echo "This process must stay running!"
echo "Press Ctrl+C to stop the controller"
echo ""

# Start xboxdrv in foreground mode
# This creates /dev/input/js0 which ROS2 joy node needs
exec sudo xboxdrv --detach-kernel-driver
