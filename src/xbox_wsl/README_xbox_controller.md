# Xbox Controller Setup for WSL2 + ROS2

⚠️ **See [XBOX_CONTROLLER_SETUP.md](XBOX_CONTROLLER_SETUP.md) for the complete, tested setup guide.**

## Quick Summary

WSL2's kernel lacks joystick drivers, so we use:
1. **xboxdrv** - Creates `/dev/input/event0` from the USB controller
2. **xbox_joy_node.py** - Custom Python node that reads event0 and publishes to `/joy`
3. **teleop_twist_joy** - Converts `/joy` to `/cmd_vel` (built-in ROS2 package)

## Quick Start (After Initial Setup)

```bash
# Terminal 1: Start Xbox controller (all-in-one: xboxdrv + joy + teleop)
./src/xbox_wsl/scripts/start_xbox_teleop.sh

# Terminal 2: Launch your robot
source install/setup.bash
ros2 launch uiabot_mini_bringup uiabot_mini.launch.py
```

See the full guide for first-time setup and troubleshooting.
