#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import time
import argparse
import subprocess
from rclpy.time import Time
from builtin_interfaces.msg import Time as TimeMsg

class TestDataGenerator(Node):
    def __init__(self, record: bool = False):
        super().__init__('test_data_generator')
        self.fix_pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.cmd_nav_pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.sim_time = 0.0
        self.heading = 0.0
        self.record = record
        self.bag_proc = None
        if self.record:
            self.bag_proc = subprocess.Popen([
                'ros2', 'bag', 'record', '-a', '-o', 'test_run'
            ])
            self.get_logger().info('Recording rosbag2 to test_run/')

    def timer_callback(self):
        now = self.get_clock().now()
        self.sim_time = (now - self.start_time).nanoseconds / 1e9
        if self.sim_time > 30.0:
            self.get_logger().info('Test scenario complete.')
            if self.bag_proc:
                self.bag_proc.terminate()
            rclpy.shutdown()
            return
        # GPS
        fix = NavSatFix()
        fix.header.stamp = now.to_msg()
        fix.header.frame_id = 'gps'
        fix.latitude = 37.0
        fix.longitude = -122.0
        fix.altitude = 10.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        fix.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        fix.status.service = 1
        if self.sim_time < 10.0 or self.sim_time >= 25.0:
            fix.status.status = 2  # RTK Fixed
            # Simulate fresh GPS
            fix.header.stamp = (now - rclpy.duration.Duration(seconds=0.1)).to_msg()
        else:
            fix.status.status = 0  # No fix
            # Simulate stale GPS
            age = min(self.sim_time - 10.0, 15.0)
            fix.header.stamp = (now - rclpy.duration.Duration(seconds=age)).to_msg()
        self.fix_pub.publish(fix)
        # IMU
        imu = Imu()
        imu.header.stamp = now.to_msg()
        imu.header.frame_id = 'imu'
        # Constant heading with drift
        self.heading += 0.001 * 0.1  # 0.001 rad/s drift
        q = self.yaw_to_quaternion(self.heading)
        imu.orientation.x = q[0]
        imu.orientation.y = q[1]
        imu.orientation.z = q[2]
        imu.orientation.w = q[3]
        self.imu_pub.publish(imu)
        # Odom
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.twist.twist.linear.x = 0.5
        self.odom_pub.publish(odom)
        # Nav command
        nav_cmd = Twist()
        nav_cmd.linear.x = 0.5
        nav_cmd.angular.z = 0.0
        self.cmd_nav_pub.publish(nav_cmd)

    @staticmethod
    def yaw_to_quaternion(yaw):
        # Returns [x, y, z, w]
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        return [0.0, 0.0, sy, cy]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--record', action='store_true', help='Record rosbag2 during test')
    args = parser.parse_args()
    rclpy.init()
    node = TestDataGenerator(record=args.record)
    try:
        rclpy.spin(node)
    finally:
        if node.bag_proc:
            node.bag_proc.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
