#!/usr/bin/env python3
"""
Analyze a rosbag2 recording and generate metrics + plots.

Usage:
  ros2 run axis_reliability analyze_bag /path/to/rosbag2_dir
"""
import sys
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import matplotlib.pyplot as plt
import numpy as np

def extract_topics(bag_path):
    """Extract /axis/status and /cmd_vel from bag"""
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = ConverterOptions('', '')
    reader.open(storage_options, converter_options)
    timestamps = []
    states = []
    velocities_x = []
    velocities_z = []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == '/axis/status':
            msg = deserialize_message(data, String)
            timestamps.append(t / 1e9)
            states.append(msg.data)
        elif topic == '/cmd_vel':
            msg = deserialize_message(data, Twist)
            velocities_x.append(msg.linear.x)
            velocities_z.append(msg.angular.z)
    return np.array(timestamps), states, np.array(velocities_x), np.array(velocities_z)

def plot_results(timestamps, states, vel_x, vel_z):
    """Generate 2-subplot figure"""
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
    state_map = {'NORMAL': 0, 'FALLBACK': 1, 'RECOVERY': 2}
    state_values = [state_map.get(s, -1) for s in states]
    colors = ['green' if s == 0 else 'red' if s == 1 else 'yellow' for s in state_values]
    ax1.scatter(timestamps, state_values, c=colors, s=50, alpha=0.7)
    ax1.set_yticks([0, 1, 2])
    ax1.set_yticklabels(['NORMAL', 'FALLBACK', 'RECOVERY'])
    ax1.set_ylabel('System State')
    ax1.set_title('AXIS Reliability Module - State Transitions')
    ax1.grid(True, alpha=0.3)
    ax2.plot(timestamps, vel_x, label='linear.x (m/s)', linewidth=2)
    ax2.plot(timestamps, vel_z, label='angular.z (rad/s)', linewidth=2, alpha=0.7)
    ax2.set_xlabel('Time (seconds)')
    ax2.set_ylabel('Velocity')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig('axis_analysis.png', dpi=150)
    print("Plot saved to axis_analysis.png")

def compute_metrics(states, vel_x):
    """Compute summary statistics"""
    normal_count = states.count('NORMAL')
    fallback_count = states.count('FALLBACK')
    recovery_count = states.count('RECOVERY')
    total = len(states)
    print("\n" + "="*60)
    print("AXIS RELIABILITY ANALYSIS")
    print("="*60)
    print(f"Total samples: {total}")
    print(f"\nState breakdown:")
    print(f"  NORMAL:   {normal_count:5d} ({100*normal_count/total:5.1f}%)")
    print(f"  FALLBACK: {fallback_count:5d} ({100*fallback_count/total:5.1f}%)")
    print(f"  RECOVERY: {recovery_count:5d} ({100*recovery_count/total:5.1f}%)")
    print(f"\nVelocity stats:")
    print(f"  Mean linear.x: {np.mean(vel_x):.3f} m/s")
    print(f"  Max linear.x:  {np.max(vel_x):.3f} m/s")
    print(f"  Min linear.x:  {np.min(vel_x):.3f} m/s")
    print("="*60)

def main(args=None):
    if len(sys.argv) < 2:
        print("Usage: ros2 run axis_reliability analyze_bag <bag_path>")
        sys.exit(1)
    
    bag_path = sys.argv[1]
    timestamps, states, vel_x, vel_z = extract_topics(bag_path)
    compute_metrics(states, vel_x)
    plot_results(timestamps, states, vel_x, vel_z)

if __name__ == '__main__':
    main()
