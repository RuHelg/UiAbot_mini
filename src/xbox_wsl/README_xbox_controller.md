## AI-Generated Code Notice

This package (`xbox_wsl`) was generated entirely with the assistance of AI-based code generation tools. As a result, the current maintainers do **not** claim original authorship of the underlying source code or its design.

The package has been reviewed, adapted, and tested for use within the UiAbot-mini project, but large parts of the implementation may be derived from patterns or examples present in the AI models’ training data.

If you recognize any part of this code as originating from your own work (e.g. a tutorial, blog post, open-source project, or other material):

- Please open an issue in this repository, or  
- Contact the maintainers (see the contact information in this repository)

so that proper credit can be given, or if appropriate, so that the relevant material can be replaced or removed.

Use this package at your own discretion.


# Xbox Controller Setup for WSL2 + ROS2

**See [XBOX_CONTROLLER_SETUP.md](XBOX_CONTROLLER_SETUP.md) for the complete, tested setup guide.**

## Quick Summary

WSL2's kernel lacks joystick drivers, so we use:
1. **xboxdrv** - Creates `/dev/input/event0` from the USB controller
2. **xbox_joy_node.py** - Custom Python node that reads event0 and publishes to `/joy`
3. **teleop_twist_joy** - Converts `/joy` to `/cmd_vel` (built-in ROS2 package)

## Quick Start (After Initial Setup)

```bash
# Terminal 1: Start Xbox controller (all-in-one: xboxdrv + joy + teleop)
source install/setup.bash &&
./src/xbox_wsl/scripts/start_xbox_teleop.sh

# Terminal 2: Launch your robot
source install/setup.bash
ros2 launch uiabot_mini_bringup uiabot_mini.launch.py
```

See the full guide for first-time setup and troubleshooting.
