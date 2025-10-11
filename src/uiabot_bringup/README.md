# UIABot Mini Bringup Package

Launch files and configuration for the UIABot Mini differential drive robot.

## Overview

This package contains everything needed to bring up the UIABot Mini robot system:
- Robot URDF model
- Sensor configurations (RPLidar A1, BNO055 IMU)
- EKF sensor fusion setup
- TF tree configuration

## Hardware

- **Base**: Differential drive robot
- **LiDAR**: RPLidar A1 (USB, /dev/ttyUSB0)
- **IMU**: BNO055 9-DOF (I2C)
- **Motor Controller**: Serial communication (/dev/ttyUSB1)

## Launch Files

### bringup.launch.py

Main launch file that starts all robot systems:

```bash
ros2 launch uiabot_bringup bringup.launch.py
```

This launches:
- `robot_state_publisher` - Publishes URDF transforms
- `ekf_filter_node` - Fuses IMU data for orientation tracking
- `teleop_to_serial_node` - Converts cmd_vel to motor commands
- `wheel_state_publisher_node` - Publishes wheel joint states
- `rplidar_node` - RPLidar A1 driver
- `bno055` - BNO055 IMU driver
- Static transforms for sensor frames

## Configuration Files

### config/ekf.yaml

Extended Kalman Filter configuration for sensor fusion. Currently configured to:
- Fuse IMU orientation (roll, pitch, yaw)
- Fuse IMU angular velocities
- Publish odom→base_link transform at 30 Hz

### urdf/uiabot_mini.urdf

Robot model including:
- Base link geometry
- Left and right wheel joints (continuous)
- IMU sensor frame
- Wheel positions and axes

## Frame Configuration

### Position Parameters (in meters)

**Laser (RPLidar A1):**
- X: 0.0
- Y: 166.5mm
- Z: 49.66mm
- Rotation: 90° pitch (X-axis)

**IMU (BNO055):**
- Defined in URDF at (71.75mm, 58.75mm, 110.35mm)
- Rotation: 90° pitch (X-axis)

**Map→Odom Offset:**
- Z: -222.85mm (grid alignment)
- Rotation: 90° about Z-axis

## TF Tree

```
map
 └─ odom (90° Z rotation, -222.85mm Z offset)
     └─ base_link (EKF publishes this transform with IMU orientation)
         ├─ wheel_left_link (URDF joint)
         ├─ wheel_right_link (URDF joint)
         ├─ laser (static transform)
         └─ bno055 (URDF joint)
```

## Dependencies

- `robot_state_publisher`
- `robot_localization`
- `tf2_ros`
- `rplidar_ros`
- `bno055`
- `serial_communication` (custom package)

## Building

```bash
cd ~/ros2/ws
colcon build --packages-select uiabot_bringup
source install/setup.bash
```

## Usage

1. Ensure hardware connections:
   - RPLidar A1 on /dev/ttyUSB0
   - Motor controller on /dev/ttyUSB1
   - BNO055 IMU on I2C bus

2. Launch the system:
   ```bash
   ros2 launch uiabot_bringup bringup.launch.py
   ```

3. Control the robot:
   ```bash
   ros2 run teleop_twist_keyboard teleop_twist_keyboard
   ```

4. Visualize in RViz2:
   ```bash
   rviz2
   ```

## Topics

- `/cmd_vel` - Velocity commands (geometry_msgs/Twist)
- `/scan` - LiDAR data (sensor_msgs/LaserScan)
- `/bno055/imu` - IMU data (sensor_msgs/Imu)
- `/wheel_r`, `/wheel_l` - Individual wheel velocities
- `/joint_states` - Robot joint states
- `/odometry/filtered` - EKF filtered odometry

## Maintainer

Gruppe 6 <gruppe-6@uia.no>

## License

MIT
