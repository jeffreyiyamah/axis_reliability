#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

LAT0 = 42.4440      # origin latitude
LON0 = -76.5019     # origin longitude
R_EARTH = 6378137.0 # meters

class OdomToFix(Node):
    def __init__(self):
        super().__init__('odom_to_fix')
        self.origin_lat = self.declare_parameter('origin_lat', LAT0).get_parameter_value().double_value
        self.origin_lon = self.declare_parameter('origin_lon', LON0).get_parameter_value().double_value
        self.dropout_after = self.declare_parameter('dropout_after_sec', 0.0).get_parameter_value().double_value
        self.start_time = time.time()

        self.pub = self.create_publisher(NavSatFix, '/fix', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.cb, 10)

        # Timer publishes even if no /odom is received (good for testing)
        self.timer = self.create_timer(0.5, self.publish_fake_fix)

        self.get_logger().info(
            f'Publishing /fix from /odom (origin={self.origin_lat:.6f},{self.origin_lon:.6f})'
            + (f' with dropout after {self.dropout_after:.1f}s' if self.dropout_after>0 else '')
        )

        self.last_x = 0.0
        self.last_y = 0.0
        self._dropout_active = False

    def cb(self, msg: Odometry):
        self.last_x = msg.pose.pose.position.x
        self.last_y = msg.pose.pose.position.y

    def publish_fake_fix(self):
        elapsed = time.time() - self.start_time

        # Simulate GPS dropout if requested
        if self.dropout_after > 0:
            if self.dropout_after <= elapsed < self.dropout_after + 10.0:
                if not self._dropout_active:
                    self.get_logger().warn(f'GPS DROPOUT ACTIVE ({elapsed:.1f}s)')
                    self._dropout_active = True
                return
            elif elapsed >= self.dropout_after + 10.0 and self._dropout_active:
                self.get_logger().info(f'GPS RECOVERED ({elapsed:.1f}s)')
                self._dropout_active = False

        # Convert x,y to lat/lon offset
        dlat = (self.last_y / R_EARTH) * (180.0 / math.pi)
        dlon = (self.last_x / (R_EARTH * math.cos(math.radians(self.origin_lat)))) * (180.0 / math.pi)

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self.origin_lat + dlat
        fix.longitude = self.origin_lon + dlon
        fix.altitude = 0.0
        fix.position_covariance = [2.0, 0.0, 0.0,
                                   0.0, 2.0, 0.0,
                                   0.0, 0.0, 4.0]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED

        self.pub.publish(fix)

def main():
    rclpy.init()
    node = OdomToFix()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
