# axis_reliability

A ROS 2 package for robust axis navigation with fallback and recovery state machine.

## Build Instructions

```bash
colcon build --packages-select axis_reliability
```

## Run Instructions

```bash
ros2 launch axis_reliability axis_demo.launch.py
```

## Topic Interfaces

| Topic             | Type                      | Direction   | QoS         | Notes                |
|-------------------|---------------------------|-------------|-------------|----------------------|
| /fix              | sensor_msgs/NavSatFix     | Subscribe   | 10, best_effort | GPS fix input        |
| /imu/data         | sensor_msgs/Imu           | Subscribe   | 50, best_effort | IMU data             |
| /odom             | nav_msgs/Odometry         | Subscribe   | 20, best_effort | Odometry             |
| /cmd_vel_nav      | geometry_msgs/Twist       | Subscribe   | 10, reliable    | Nav velocity input   |
| /cmd_vel          | geometry_msgs/Twist       | Publish     | 10, reliable    | Output velocity      |
| /axis/status      | std_msgs/String           | Publish     | 10, reliable    | State machine status |

## Dependencies

- rclpy
- sensor_msgs
- nav_msgs
- geometry_msgs
- std_msgs
- tf_transformations
