import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from transforms3d.euler import quat2euler
from rclpy.time import Time
import math
import os
import csv
from typing import Optional
from axis_reliability.axis_metrics import AxisMetrics


class AxisBaselineNode(Node):
    def __init__(self):
        super().__init__('axis_baseline')

        # Parameters (same structure as reliability)
        self.declare_parameter('gps_age_max_s', 2.0)
        self.declare_parameter('enable_csv_logging', True)
        self.declare_parameter('log_directory', '/tmp/axis_logs')

        self.gps_age_max_s = self.get_parameter('gps_age_max_s').value
        self.enable_csv_logging = self.get_parameter('enable_csv_logging').value
        self.log_directory = self.get_parameter('log_directory').value

        # State tracking
        self.last_gps_msg: Optional[NavSatFix] = None
        self.last_gps_time: Optional[Time] = None
        self.last_imu_msg: Optional[Imu] = None
        self.last_odom_msg: Optional[Odometry] = None
        self.gps_lost_time: Optional[Time] = None
        self.gps_recovered_time: Optional[Time] = None
        self.gps_status = "OK"
        self._shutdown_requested = False

        self.declare_parameter('baseline_info_period_s', 5.0)
        self.declare_parameter('stale_warn_period_s', 1.0)
        self.info_period = self.get_parameter('baseline_info_period_s').value
        self.stale_period = self.get_parameter('stale_warn_period_s').value

        # Heartbeat clocks
        self._last_info_log_time: Optional[Time] = None
        self._last_stale_log_time: Optional[Time] = None

        # Track last nav cmd for v in logs
        self.last_nav_cmd: Optional[Twist] = None
        self.create_subscription(
            Twist, '/cmd_vel_nav', self.cmd_vel_nav_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Metrics
        self.metrics = AxisMetrics(self.log_directory)
        self.metrics.start_baseline(self.get_clock().now())

        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.status_pub = self.create_publisher(
            String,
            '/axis/status',
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriptions
        self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

        # CSV logging
        os.makedirs(self.log_directory, exist_ok=True)
        self.csv_file = None
        self.csv_writer = None
        if self.enable_csv_logging:
            csv_path = os.path.join(self.log_directory, 'axis_baseline_log.csv')
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'state', 'gps_status', 'linear_x', 'angular_z', 'heading_error'])

        # Timer
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("AxisBaselineNode initialized — GPS-only monitoring active")

    # --- Callbacks ---
    def gps_callback(self, msg: NavSatFix):
        self.last_gps_msg = msg
        self.last_gps_time = self.get_clock().now()

    def imu_callback(self, msg: Imu):
        self.last_imu_msg = msg

    def odom_callback(self, msg: Odometry):
        self.last_odom_msg = msg

    # --- Control loop ---
    def control_loop(self):
        if not self.last_gps_time:
            return
        if getattr(self, "_shutdown_requested", False):
            return
        


        now = self.get_clock().now()
        gps_age = float('inf')
        if self.last_gps_time:
            gps_age = (now - self.last_gps_time).nanoseconds / 1e9

        # Evaluate GPS freshness
        is_stale = (gps_age > self.gps_age_max_s)

        # Edge transitions (one-shot logs)
        if is_stale and self.gps_status != "STALE":
            self.gps_status = "STALE"
            self.gps_lost_time = now
            self.get_logger().warn(f"GPS LOST — robot halted (age={self._fmt_age(gps_age)})")
            self.publish_zero_velocity()

        if (not is_stale) and self.gps_status in ("STALE", "UNKNOWN"):
            prev_status = self.gps_status
            self.gps_status = "OK"
            self.gps_recovered_time = now
            downtime = 0.0
            if prev_status == "STALE" and self.gps_lost_time:
                downtime = (self.gps_recovered_time - self.gps_lost_time).nanoseconds / 1e9
                self.metrics.mark_recovery(now, downtime)
            self.get_logger().info(f"GPS RECOVERED after {downtime:.2f}s")

        # HEARTBEAT LOGS (throttled)
        if not is_stale:
            # NORMAL heartbeat
            if (self._last_info_log_time is None or
                (now - self._last_info_log_time).nanoseconds / 1e9 >= self.info_period):
                self._last_info_log_time = now
                gps_status_field = self.last_gps_msg.status.status if self.last_gps_msg else -1
                lat = getattr(self.last_gps_msg, "latitude", float('nan')) if self.last_gps_msg else float('nan')
                lon = getattr(self.last_gps_msg, "longitude", float('nan')) if self.last_gps_msg else float('nan')
                v = self.last_nav_cmd.linear.x if self.last_nav_cmd else 0.0
                self.get_logger().info(
                    f"NORMAL: GPS OK (status={gps_status_field}, age={gps_age:.2f}s) | "
                    f"pos: {lat:.6f}, {lon:.6f} | v={v:.2f}"
                )
        else:
            # STALE heartbeat
            if (self._last_stale_log_time is None or
                (now - self._last_stale_log_time).nanoseconds / 1e9 >= self.stale_period):
                self._last_stale_log_time = now
                self.get_logger().warn(
                    f"GPS STALE: age={self._fmt_age(gps_age)} | holding at v=0.00"
                )

        # Publish status + zero cmd (baseline never drives)
        status = String(); status.data = "BASELINE"
        self.status_pub.publish(status)
        cmd = Twist()  # always zero
        self.cmd_vel_pub.publish(cmd)

        if self.enable_csv_logging:
            self.log_to_csv(cmd, 0.0)


    def publish_zero_velocity(self):
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def log_to_csv(self, cmd: Twist, heading_error: float):
        if not self.csv_writer:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        gps_status = self.last_gps_msg.status.status if self.last_gps_msg else -1
        self.csv_writer.writerow([now, "BASELINE", gps_status, cmd.linear.x, cmd.angular.z, heading_error])
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file:
            self.csv_file.close()
        super().destroy_node()

    def cmd_vel_nav_callback(self, msg: Twist):
        self.last_nav_cmd = msg

    def _fmt_age(self, age: float) -> str:
        return "∞" if math.isinf(age) else f"{age:.2f}s"

def main(args=None):
    rclpy.init(args=args)
    node = AxisBaselineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._shutdown_requested = True
        try:
            node.timer.cancel()
        except Exception:
            pass

        node.get_logger().info("Shutting down — finalizing baseline metrics...")

        end = node.get_clock().now()
        node.metrics.finish(end, 'BASELINE', abort_reason='UserInterrupt')

        try:
            node.metrics.print_summary()        # or: print(node.metrics.summary_text())
        except Exception:
            pass
    finally:
        # Close files and destroy node (no ROS logs past this point)
        if node.csv_file:
            node.csv_file.close()
        node.destroy_node()

        # Make shutdown idempotent (launch may already have called it)
        try:
            rclpy.shutdown()
        except Exception:
            pass



if __name__ == '__main__':
    main()
