# UIABot Mini - ROS2 Workspace

ROS2 Jazzy workspace for the UIABot Mini differential drive robot.

## Overview

This workspace contains all software for the UIABot Mini autonomous mobile robot platform, including:
- Robot model and TF configuration (URDF)
- Sensor drivers (RPLidar A1, BNO055 IMU)
- Motor control and serial communication
- Sensor fusion with Extended Kalman Filter
- Launch files and configuration

## Hardware

- **Platform**: Differential drive robot
- **Computer**: Running ROS2 Jazzy on Ubuntu 24.04
- **LiDAR**: RPLidar A1 (USB, /dev/ttyUSB0)
- **IMU**: BNO055 9-DOF (I2C)
- **Motor Controller**: Serial communication (/dev/ttyUSB1)

## Workspace Structure

```
ws/
├── src/
│   ├── uiabot_bringup/       # Main launch package (⭐ START HERE)
│   │   ├── launch/           # Launch files
│   │   ├── config/           # Configuration files (EKF, etc.)
│   │   └── urdf/             # Robot model
│   ├── bno055/               # BNO055 IMU driver
│   ├── rplidar_ros/          # RPLidar A1 driver
│   └── serial_communication/ # Custom motor control package
├── build/                    # Build artifacts (gitignored)
├── install/                  # Installation outputs
└── log/                      # Build and runtime logs
```

## Quick Start

### 1. Build the Workspace

```bash
cd ~/ros2/ws
colcon build
source install/setup.bash
```

### 2. Launch the Robot

```bash
ros2 launch uiabot_bringup bringup.launch.py
```

This starts:
- Robot state publisher (URDF transforms)
- EKF sensor fusion (IMU orientation)
- Motor control node (serial communication)
- Wheel state publisher (joint states)
- RPLidar A1 driver
- BNO055 IMU driver
- Static transform publishers

### 3. Control the Robot

```bash
# Keyboard teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Or publish directly to cmd_vel
ros2 topic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.2}"
```

### 4. Visualize in RViz2

```bash
rviz2
```

Add displays for:
- RobotModel
- TF
- LaserScan (topic: `/scan`)
- Odometry (topic: `/odometry/filtered`)

## Packages

## Packages

### uiabot_bringup 🚀
Main launch package containing robot configuration and bringup files.
- **Location**: `src/uiabot_bringup/`
- **Type**: Launch package (ament_cmake)
- **Purpose**: System integration and launch orchestration
- [Package README](src/uiabot_bringup/README.md)

### serial_communication
Custom package for motor control via serial interface.
- **Location**: `src/serial_communication/`
- **Type**: Python package
- **Nodes**: 
  - `teleop_to_serial` - Converts cmd_vel to motor commands
  - `wheel_state_publisher` - Publishes joint states from wheel velocities

### bno055
BNO055 9-DOF IMU sensor driver (I2C).
- **Location**: `src/bno055/`
- **Type**: Python package
- **Topics**: `/bno055/imu` (sensor_msgs/Imu)

### rplidar_ros
RPLidar A1 360° LiDAR sensor driver.
- **Location**: `src/rplidar_ros/`
- **Type**: C++ package (ament_cmake)
- **Topics**: `/scan` (sensor_msgs/LaserScan)

## Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/scan` | sensor_msgs/LaserScan | LiDAR scan data |
| `/bno055/imu` | sensor_msgs/Imu | IMU measurements |
| `/joint_states` | sensor_msgs/JointState | Wheel joint positions |
| `/odometry/filtered` | nav_msgs/Odometry | EKF fused odometry |
| `/wheel_r`, `/wheel_l` | std_msgs/Float32 | Individual wheel velocities |

## TF Tree

```
map (EKF publishes map→base_link transform)
 └─ base_link (Robot base frame with IMU absolute orientation)
     ├─ wheel_left_link (URDF continuous joint)
     ├─ wheel_right_link (URDF continuous joint)
     ├─ laser (static transform: 49.66mm, 0, 166.5mm)
     └─ bno055 (URDF fixed joint: 71.75mm, 58.75mm, 110.35mm, 180° yaw)
```

**Note**: This robot does not use an `odom` frame since it lacks wheel odometry. The EKF publishes directly from `map` to `base_link` using absolute IMU orientation.

## Configuration

### Robot Dimensions
- **Laser Position**: (49.66mm, 0, 166.5mm) from base_link
- **IMU Position**: (71.75mm, 58.75mm, 110.35mm) from base_link with 180° Z rotation
- **Wheel Separation**: 197.5mm (center to center)

### EKF Configuration
Located in `src/uiabot_bringup/config/ekf.yaml`:
- **Frequency**: 30 Hz
- **Frame Configuration**: Publishes `map` → `base_link` directly (no odom frame)
- **Inputs**: IMU absolute orientation (roll, pitch, yaw) only
- **Disabled**: Linear acceleration and angular velocity integration (to prevent drift)
- **IMU Variance**: High trust in orientation (0.001 for all axes)
- **Output**: Fused odometry on `/odometry/filtered`

## Development

### Building Specific Packages

```bash
# Build only the bringup package
colcon build --packages-select uiabot_bringup

# Build with dependencies
colcon build --packages-up-to uiabot_bringup
```

### Debugging

```bash
# Check TF tree
ros2 run tf2_tools view_frames

# Monitor topics
ros2 topic list
ros2 topic echo /bno055/imu

# Check node info
ros2 node list
ros2 node info /ekf_filter_node
```

## Troubleshooting

### IMU Calibration
The BNO055 IMU requires calibration on first use. Check calibration status:
```bash
ros2 topic echo /bno055/calib_status
```

Calibration levels (0-3, where 3 is fully calibrated):
- **sys**: System calibration
- **gyro**: Gyroscope calibration (wave sensor in air)
- **accel**: Accelerometer calibration (place in 6 orientations)
- **mag**: Magnetometer calibration (move in figure-8 pattern)

**Target**: gyro=3, accel=3, mag=3 for best performance.

Calibration offsets are saved in `src/bno055/bno055/params/bno055_params_i2c.yaml` and persist across restarts.

### Expected IMU Behavior
- **Drift**: Some yaw drift over time is normal without wheel odometry
- **Orientation**: Roll and pitch should be stable when stationary
- **RViz2**: Use `map` as the fixed frame for proper visualization

### Serial Port Permissions
```bash
sudo usermod -aG dialout $USER
# Log out and back in
```

### USB Device Detection
```bash
# Check if devices are detected
ls -l /dev/ttyUSB*

# Should show:
# /dev/ttyUSB0 (RPLidar)
# /dev/ttyUSB1 (Motor controller)
```

### I2C IMU Connection
```bash
# Check I2C devices
sudo i2cdetect -y 1

# Should show BNO055 at address 0x28
```

## Dependencies

- ROS2 Jazzy
- Python 3.12
- `robot_state_publisher`
- `robot_localization`
- `tf2_ros`
- `ament_cmake`
- `pyserial`

## Git Repository

- **Host**: GitLab
- **Group**: mas514-a25/group6
- **Project**: uiabot-ros

## Team

Gruppe 6 - MAS514 Autumn 2025
University of Agder (UiA)

## License

MIT

---

**Status**: ✅ Production Ready

For detailed package documentation, see [uiabot_bringup/README.md](src/uiabot_bringup/README.md)

## Suggestions for a good README

Every project is different, so consider which of these sections apply to yours. The sections used in the template are suggestions for most open source projects. Also keep in mind that while a README can be too long and detailed, too long is better than too short. If you think your README is too long, consider utilizing another form of documentation rather than cutting out information.

## Name
Choose a self-explaining name for your project.

## Description
Let people know what your project can do specifically. Provide context and add a link to any reference visitors might be unfamiliar with. A list of Features or a Background subsection can also be added here. If there are alternatives to your project, this is a good place to list differentiating factors.

## Badges
On some READMEs, you may see small images that convey metadata, such as whether or not all the tests are passing for the project. You can use Shields to add some to your README. Many services also have instructions for adding a badge.

## Visuals
Depending on what you are making, it can be a good idea to include screenshots or even a video (you'll frequently see GIFs rather than actual videos). Tools like ttygif can help, but check out Asciinema for a more sophisticated method.

## Installation
Within a particular ecosystem, there may be a common way of installing things, such as using Yarn, NuGet, or Homebrew. However, consider the possibility that whoever is reading your README is a novice and would like more guidance. Listing specific steps helps remove ambiguity and gets people to using your project as quickly as possible. If it only runs in a specific context like a particular programming language version or operating system or has dependencies that have to be installed manually, also add a Requirements subsection.

## Usage
Use examples liberally, and show the expected output if you can. It's helpful to have inline the smallest example of usage that you can demonstrate, while providing links to more sophisticated examples if they are too long to reasonably include in the README.

## Support
Tell people where they can go to for help. It can be any combination of an issue tracker, a chat room, an email address, etc.

## Roadmap
If you have ideas for releases in the future, it is a good idea to list them in the README.

## Contributing
State if you are open to contributions and what your requirements are for accepting them.

For people who want to make changes to your project, it's helpful to have some documentation on how to get started. Perhaps there is a script that they should run or some environment variables that they need to set. Make these steps explicit. These instructions could also be useful to your future self.

You can also document commands to lint the code or run tests. These steps help to ensure high code quality and reduce the likelihood that the changes inadvertently break something. Having instructions for running tests is especially helpful if it requires external setup, such as starting a Selenium server for testing in a browser.

## Authors and acknowledgment
Show your appreciation to those who have contributed to the project.

## License
For open source projects, say how it is licensed.

## Project status
If you have run out of energy or time for your project, put a note at the top of the README saying that development has slowed down or stopped completely. Someone may choose to fork your project or volunteer to step in as a maintainer or owner, allowing your project to keep going. You can also make an explicit request for maintainers.
