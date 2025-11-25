# Xbox Controller Setup for WSL2 - Quick Guide

## The Problem
WSL2's kernel lacks the `joydev` module, so `/dev/input/js0` doesn't work even though `xboxdrv` says it creates it.

## The Solution
Use a custom Python node (`xbox_joy_node.py`) that reads directly from `/dev/input/event0`.

## One-Time Setup

### 1. Install Dependencies
```bash
sudo apt-get update
sudo apt-get install -y xboxdrv python3-evdev ros-jazzy-teleop-twist-joy
```

### 2. Make Script Executable
```bash
chmod +x ~/ros2_ws/src/xbox_joy_node.py
chmod +x ~/ros2_ws/start_xbox_controller.sh
```

### 3. Add User to Input Group (Optional - for permissions)
```bash
sudo usermod -a -G input $USER
# Then logout and login for this to take effect
```

## Every Time You Use the Controller

### Step 1: Attach USB in Windows (PowerShell as Admin)
```powershell
# Check the BUSID (usually 1-5 for Xbox controller)
usbipd list

# Attach it to WSL
usbipd attach --wsl --busid=1-5
```

### Step 2: Start xboxdrv (Terminal 1)
```bash
cd ~/ros2_ws
./start_xbox_controller.sh
```
**Keep this running!** It creates `/dev/input/event0`

### Step 3: Fix Permissions (Terminal 2)
```bash
sudo chmod 666 /dev/input/event0
```
*Note: Skip this if you've logged out/in after adding yourself to input group*

### Step 4: Start the Custom Joy Node (Terminal 2)
```bash
source ~/ros2_ws/install/setup.bash
~/ros2_ws/src/xbox_joy_node.py
```
**Keep this running!** It publishes to `/joy` topic

### Step 5: Set Teleop Parameters (Terminal 3)
```bash
source ~/ros2_ws/install/setup.bash

# Configure the teleop node with correct axis mapping
ros2 param set /teleop_twist_joy_node require_enable_button false
ros2 param set /teleop_twist_joy_node axis_linear.x 1
ros2 param set /teleop_twist_joy_node axis_angular.yaw 0
ros2 param set /teleop_twist_joy_node scale_linear.x 0.35
ros2 param set /teleop_twist_joy_node scale_angular.yaw 3.14
```

### Step 6: Launch Your Robot (Terminal 4)
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 launch uiabot_mini_bringup uiabot_mini.launch.py
# or for Gazebo:
ros2 launch uiabot_mini_gazebo uiabot_mini_gazebo_launch.py rviz:=true
```

## Test the Controller
```bash
# In a new terminal
source ~/ros2_ws/install/setup.bash

# Test 1: Check joy messages
ros2 topic echo /joy
# Move the joystick - you should see axes values changing (especially axes[0] and axes[1])

# Test 2: Check cmd_vel
ros2 topic echo /cmd_vel
# Move the left stick - you should see linear.x and angular.z changing
```

## Axis Mapping (Xbox 360 Controller)
- **Axis 0**: Left Stick X (left/right) → Controls rotation (angular.z)
- **Axis 1**: Left Stick Y (up/down) → Controls forward/backward (linear.x)
- Axis 2: Right Stick X
- Axis 3: Right Stick Y

## Quick Start Script (Alternative to Manual Steps)

Create a script to automate steps 3-5:

```bash
#!/bin/bash
# ~/ros2_ws/start_xbox_teleop.sh

# Fix permissions
sudo chmod 666 /dev/input/event0

# Source workspace
source ~/ros2_ws/install/setup.bash

# Start joy node in background
~/ros2_ws/src/xbox_joy_node.py &
JOY_PID=$!

# Wait for nodes to start
sleep 2

# Configure teleop parameters
ros2 param set /teleop_twist_joy_node require_enable_button false
ros2 param set /teleop_twist_joy_node axis_linear.x 1
ros2 param set /teleop_twist_joy_node axis_angular.yaw 0  
ros2 param set /teleop_twist_joy_node scale_linear.x 0.35
ros2 param set /teleop_twist_joy_node scale_angular.yaw 3.14

echo "Xbox controller ready! Move the left stick to control the robot."
echo "Press Ctrl+C to stop"

# Wait for Ctrl+C
wait $JOY_PID
```

Then just run: `./start_xbox_teleop.sh`

## Troubleshooting

### No /dev/input/event0
- Make sure xboxdrv is running (check Terminal 1)
- Check: `ls -la /dev/input/`
- Verify USB is attached: `lsusb | grep Xbox`

### Permission denied on /dev/input/event0
```bash
sudo chmod 666 /dev/input/event0
```

### Controller not detected in Windows
Re-attach in Windows PowerShell (Admin):
```powershell
usbipd list
usbipd attach --wsl --busid=1-5
```

### No cmd_vel messages
1. Check joy is publishing: `ros2 topic echo /joy` (move stick, see values change)
2. Check teleop node is running: `ros2 node list | grep teleop`
3. Verify parameters are set correctly:
   ```bash
   ros2 param get /teleop_twist_joy_node require_enable_button  # Should be: false
   ros2 param get /teleop_twist_joy_node axis_linear.x          # Should be: 1
   ros2 param get /teleop_twist_joy_node axis_angular.yaw       # Should be: 0
   ```

### After WSL restart
You'll need to:
1. Re-attach USB in Windows: `usbipd attach --wsl --busid=1-5`
2. Restart xboxdrv: `./start_xbox_controller.sh` (Terminal 1)
3. Fix permissions: `sudo chmod 666 /dev/input/event0` (Terminal 2)
4. Restart joy node: `~/ros2_ws/src/xbox_joy_node.py` (Terminal 2)
5. Set teleop parameters (Terminal 3)
6. Launch robot (Terminal 4)
