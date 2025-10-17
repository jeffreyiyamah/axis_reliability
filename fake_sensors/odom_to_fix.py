#!/usr/bin/env python3
import math, time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus

LAT0 = 42.4440      # set your origin (deg)
LON0 = -76.5019     # set your origin (deg)
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
        self.timer = self.create_timer(0.2, lambda: None)  # ~5 Hz
        self.get_logger().info(
          f'Publishing /fix from /odom (origin={self.origin_lat:.6f},{self.origin_lon:.6f})'
          + (f' with dropout after {self.dropout_after:.1f}s' if self.dropout_after>0 else '')
        )

    def cb(self, msg: Odometry):
        elapsed = time.time() - self.start_time
        
        if self.dropout_after > 0:
            dropout_end = self.dropout_after + 10.0
            if self.dropout_after <= elapsed < dropout_end:
                if not hasattr(self, '_dropout_logged'):
                    self.get_logger().warn(f'GPS DROPOUT ACTIVE ({elapsed:.1f}s)')
                    self._dropout_logged = True
                return
            elif elapsed >= dropout_end and not hasattr(self, '_recovery_logged'):
                self.get_logger().info(f'GPS RECOVERED ({elapsed:.1f}s)')
                self._recovery_logged = True
                self._dropout_logged = False
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        dlat = (y / R_EARTH) * (180.0 / math.pi)
        dlon = (x / (R_EARTH * math.cos(math.radians(self.origin_lat)))) * (180.0 / math.pi)

        fix = NavSatFix()
        fix.header = msg.header
        fix.status.status = 2
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = self.origin_lat + dlat
        fix.longitude = self.origin_lon + dlon
        fix.altitude = 0.0
        fix.position_covariance = [
            2.0, 0.0, 0.0,
            0.0, 2.0, 0.0,
            0.0, 0.0, 4.0
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        fix.header.frame_id = 'gps_link'

        self.pub.publish(fix)

def main():
    rclpy.init()
    rclpy.spin(OdomToFix())
    rclpy.shutdown()

if __name__ == '__main__':
    main()

