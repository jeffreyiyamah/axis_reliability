# axis_reliability

**ROS 2 Package for Reliable Navigation Under GPS Dropout**

`axis_reliability` monitors GPS, IMU, and odometry data to maintain stable navigation when GPS becomes unreliable. It detects GPS loss, switches to dead-reckoning (FALLBACK) using IMU and velocity commands, and transitions back to GPS-based control (RECOVERY) once GPS returns.

---

## Installation

### Prerequisites

- **ROS 2 Humble** installed and sourced
- `colcon` build tools
- Python 3
- (Optional) `turtlebot3_gazebo` if you want to use the included simulation world

### Clone & Build

```bash
cd ~/ros2_ws/src
git clone https://github.com/jeffreyiyamah/axis_reliability.git

cd ~/ros2_ws
colcon build --packages-select axis_reliability
source install/setup.zsh
```

---

## How to Run

### Option 1 — Run the Node Directly

If your robot already provides `/fix`, `/imu`, `/odom`, and `/cmd_vel_nav`:

```bash
ros2 run axis_reliability axis_reliability
```

Or run with parameters:

```bash
ros2 launch axis_reliability axis.launch.py
```

This loads parameters from:

```
config/axis_reliability.yaml
```

---

## Simulation (with Fake GPS)

The `sim.launch.py` file launches:
- TurtleBot3 + orchard world in Gazebo
- `axis_reliability` node
- `odom_to_fix.py` (converts `/odom` → `/fix`)

### Run Everything in One Go

```bash
ros2 launch axis_reliability sim.launch.py
```

This will:
- Start Gazebo
- Run your reliability node
- Simulate GPS messages from odometry

### Run Components Manually

**Terminal 1: Launch Gazebo world**

```bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py world:=~/ros2_ws/src/orchard_world/worlds/orchard.world
```

**Terminal 2: Run fake GPS node**

```bash
python3 ~/ros2_ws/src/axis_reliability/fake_sensors/odom_to_fix.py
```

**Terminal 3: Start your reliability node**

```bash
ros2 launch axis_reliability axis.launch.py
```

---

## Running Without Fake GPS (Field Test)

For field tests or indoor robots without GPS:

1. Robot provides:
   - `/imu/data` (`sensor_msgs/Imu`)
   - `/odom` (`nav_msgs/Odometry`)
   - `/cmd_vel_nav` from navigation stack
   - `/cmd_vel` for actual motion

2. You run:

```bash
ros2 launch axis_reliability axis.launch.py
```

3. If no GPS is available, mock it with:

```bash
python3 ~/ros2_ws/src/axis_reliability/fake_sensors/odom_to_fix.py
```

This creates a `/fix` topic from odometry so your node behaves as if GPS were present.

---

## Topics Overview

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/fix` | `sensor_msgs/NavSatFix` | Subscribed | GPS fix (real or fake) |
| `/imu` | `sensor_msgs/Imu` | Subscribed | Orientation and heading |
| `/odom` | `nav_msgs/Odometry` | Subscribed | Odometry data |
| `/cmd_vel_nav` | `geometry_msgs/Twist` | Subscribed | Navigation velocity commands |
| `/cmd_vel` | `geometry_msgs/Twist` | Published | Velocity output (possibly filtered) |
| `/axis/status` | `std_msgs/String` | Published | `NORMAL`, `FALLBACK`, or `RECOVERY` |

---

## Parameters

Defaults are specified in `config/axis_reliability.yaml`:

| Name | Default | Description |
|------|---------|-------------|
| `n_bad_gps_threshold` | 10 | Consecutive bad/stale GPS samples to enter FALLBACK |
| `m_good_gps_threshold` | 5 | Consecutive good samples to start RECOVERY |
| `gps_age_max_s` | 2.0 | Max allowed GPS message age (seconds) |
| `fallback_linear_velocity` | 0.3 | Linear velocity (m/s) in FALLBACK mode |
| `fallback_max_duration_s` | 30.0 | Hard timeout in FALLBACK before ABORT |
| `heading_correction_kp` | 0.5 | Proportional gain to hold heading |
| `max_angular_correction` | 0.1 | Angular velocity clamp (rad/s) in FALLBACK |
| `recovery_blend_duration_s` | 2.5 | Duration (seconds) to blend FALLBACK→NAV |
| `enable_csv_logging` | true | Enable CSV log output |
| `log_directory` | `/tmp/axis_logs` | Path for CSV logs |

---
## Field Test Checklist

- Robot publishes `/imu/data`, `/odom`, and `/cmd_vel_nav`
- Base controller listens on `/cmd_vel`
- GPS or fake GPS publishes `/fix`
- ROS 2 Humble sourced on both machines
- Network or ROS 2 domain configured if running across machines
- Verify topics with `ros2 topic list`
- Check node state:

```bash
ros2 topic echo /axis/status
```

---

## Repo Structure

```
axis_reliability/
├── axis_reliability/          # Node logic
│   ├── axis_reliability_node.py
│   ├── analyze_bag.py
│   └── generate_test_bag.py
├── config/
│   └── axis_reliability.yaml  # Parameters
├── fake_sensors/
│   └── odom_to_fix.py         # Fake GPS from odometry
├── launch/
│   ├── axis.launch.py         # Node only
│   └── sim.launch.py          # Node + Gazebo + fake GPS
├── tests/
│   └── test_state_machine.py
├── README.md
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Troubleshooting

**Gazebo world not found:** Edit `sim.launch.py` to point to the correct orchard world path.

**No `/fix` topic:** Run the fake GPS script manually:

```bash
python3 fake_sensors/odom_to_fix.py
```

**No `/axis/status` updates:** Check that `/fix` and `/imu` are publishing messages.

**GPS marked invalid:** Ensure your fake GPS uses `status: 2` (RTK-fixed). Adjust in `odom_to_fix.py` if needed.

## Logs & Data

**CSV logs:**  
Default path → `/tmp/axis_logs/axis_reliability_log.csv`  

Copy to your machine:
```bash
scp <user>@<robot_ip>:/tmp/axis_logs/axis_reliability_log.csv .
Record topics (optional):

bash
Copy code
ros2 bag record /fix /imu /odom /cmd_vel /axis/status
Copy bag:

bash
Copy code
scp -r <user>@<robot_ip>:~/rosbags/<bag_folder> .