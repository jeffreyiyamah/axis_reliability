import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from transforms3d.euler import quat2euler
from rclpy.time import Time
import math
from sensor_msgs.msg import NavSatStatus
import os
from axis_reliability.axis_metrics import AxisMetrics
import csv
from typing import Optional

class SystemState(Enum):
    NORMAL = "NORMAL"
    FALLBACK = "FALLBACK"
    RECOVERY = "RECOVERY"

class AxisReliabilityNode(Node):
    def __init__(self) -> None:
        super().__init__('axis_reliability')

        # Declare parameters (load from YAML)
        self.declare_parameter('n_bad_gps_threshold', 10)
        self.declare_parameter('m_good_gps_threshold', 5)
        self.declare_parameter('gps_age_max_s', 2.0)
        self.declare_parameter('fallback_linear_velocity', 0.3)
        self.declare_parameter('fallback_max_duration_s', 30.0)
        self.declare_parameter('heading_correction_kp', 0.5)
        self.declare_parameter('max_angular_correction', 0.1)
        self.declare_parameter('recovery_blend_duration_s', 2.5)
        self.declare_parameter('enable_csv_logging', True)
        self.declare_parameter('log_directory', '/tmp/axis_logs')

        # State
        self.state = SystemState.NORMAL

        # Message storage
        self.last_gps_msg: Optional[NavSatFix] = None
        self.last_gps_time: Optional[Time] = None
        self.last_imu_msg: Optional[Imu] = None
        self.last_imu_time: Optional[Time] = None
        self.last_odom_msg: Optional[Odometry] = None
        self.last_odom_time: Optional[Time] = None
        self.last_nav_cmd: Optional[Twist] = None

        # State machine counters
        self.bad_gps_count = 0
        self.good_gps_count = 0

        # Timing
        self.fallback_start_time: Optional[Time] = None
        self.recovery_start_time: Optional[Time] = None

        # Last known state
        self.last_known_heading = 0.0

        # Load parameters
        self.n_bad_gps_threshold = self.get_parameter('n_bad_gps_threshold').value
        self.m_good_gps_threshold = self.get_parameter('m_good_gps_threshold').value
        self.gps_age_max_s = self.get_parameter('gps_age_max_s').value
        self.fallback_linear_velocity = self.get_parameter('fallback_linear_velocity').value
        self.fallback_max_duration_s = self.get_parameter('fallback_max_duration_s').value
        self.heading_correction_kp = self.get_parameter('heading_correction_kp').value
        self.max_angular_correction = self.get_parameter('max_angular_correction').value
        self.recovery_blend_duration_s = self.get_parameter('recovery_blend_duration_s').value
        self.enable_csv_logging = self.get_parameter('enable_csv_logging').value
        self.log_directory = self.get_parameter('log_directory').value

        self.metrics = AxisMetrics(self.log_directory)
        self._shutdown_requested = False



        # CSV logging
        self.csv_writer = None
        self.csv_file = None
        if self.enable_csv_logging:
            os.makedirs(self.log_directory, exist_ok=True)
            csv_path = os.path.join(self.log_directory, 'axis_reliability_log.csv')
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow([
                'timestamp', 'state', 'gps_status', 'linear_x', 'angular_z', 'heading_error'
            ])

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
        self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_nav_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

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

        # Control loop timer (10 Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info('AxisReliabilityNode initialized')


    def _enter_fallback(self, now):
        self.metrics.start_fallback(now)
        self.state = SystemState.FALLBACK
        self.fallback_start_time = now
        self.last_known_heading = self.get_heading_from_imu()
        lat = getattr(self.last_gps_msg, "latitude", float("nan"))
        lon = getattr(self.last_gps_msg, "longitude", float("nan"))
        self.get_logger().info(
            f"STATE: NORMAL → FALLBACK (dead reckoning engaged) | "
            f"last_fix: lat={lat:.6f}, lon={lon:.6f}, heading={self.last_known_heading:.3f}"
        )

    def gps_callback(self, msg: NavSatFix) -> None:
        self.last_gps_msg = msg
        self.last_gps_time = self.get_clock().now()
        gps_valid = self.is_gps_valid(msg)
        if gps_valid:
            self.good_gps_count += 1
            self.bad_gps_count = 0
        else:
            self.bad_gps_count += 1
            self.good_gps_count = 0
            self.get_logger().warn(f"GPS INVALID: status={msg.status.status}, age={self.get_gps_age(msg):.2f}s")
        # State transitions
        if self.state == SystemState.NORMAL:
            if self.bad_gps_count >= self.n_bad_gps_threshold:
                # Transition to FALLBACK
                self.last_known_heading = self.get_heading_from_imu()
                self.fallback_start_time = self.get_clock().now()
                self.good_gps_count = 0
                self.state = SystemState.FALLBACK
                self.get_logger().info(
                    f"STATE: NORMAL → FALLBACK | Last GPS: lat={msg.latitude}, lon={msg.longitude}, heading={self.last_known_heading:.3f}"
                )
        elif self.state == SystemState.FALLBACK:
            if self.good_gps_count >= self.m_good_gps_threshold:
                self.recovery_start_time = self.get_clock().now()
                self.bad_gps_count = 0
                self.state = SystemState.RECOVERY
                if self.fallback_start_time:
                    downtime_s = (self.recovery_start_time - self.fallback_start_time).nanoseconds / 1e9
                else:
                    downtime_s = 0.0
                self.metrics.mark_recovery(self.recovery_start_time, downtime_s)

                duration = (self.recovery_start_time - self.fallback_start_time).nanoseconds / 1e9 if self.fallback_start_time else 0.0
                self.get_logger().info(f"STATE: FALLBACK → RECOVERY | Duration: {duration:.2f}s")
               

    def imu_callback(self, msg: Imu) -> None:
        self.last_imu_msg = msg
        self.last_imu_time = self.get_clock().now()

    def odom_callback(self, msg: Odometry) -> None:
        self.last_odom_msg = msg
        self.last_odom_time = self.get_clock().now()

    def cmd_vel_nav_callback(self, msg: Twist) -> None:
        self.last_nav_cmd = msg

    def is_gps_valid(self, msg: NavSatFix) -> bool:
        valid_statuses = {
            NavSatStatus.STATUS_FIX,       # 0
            NavSatStatus.STATUS_SBAS_FIX,  # 1
            NavSatStatus.STATUS_GBAS_FIX   # 2
        }
        if msg.status.status not in valid_statuses:
            return False
        # Rule 2: message age
        age = self.get_gps_age(msg)
        if age > self.gps_age_max_s:
            return False
        # Future: use position_covariance for HDOP-like checks
        return True

    def get_gps_age(self, msg: NavSatFix) -> float:
        now = self.get_clock().now()
        msg_time = Time.from_msg(msg.header.stamp)
        return (now - msg_time).nanoseconds / 1e9

    def control_loop(self) -> None:
        if getattr(self, "_shutdown_requested", False):
            return

        now = self.get_clock().now()
        
        # Safety aborts
        if self.should_abort():
            self.publish_zero_velocity()
            return
        
        # --- GPS staleness check ---
        gps_age = float("inf")
        if self.last_gps_time:
            gps_age = (now - self.last_gps_time).nanoseconds / 1e9
        
        if self.state == SystemState.NORMAL:
            if gps_age > self.gps_age_max_s:
                self.bad_gps_count += 1
                self.good_gps_count = 0
                if self.bad_gps_count == self.n_bad_gps_threshold - 1:
                    self.metrics.mark_degrade(now)
                # Throttled warning
                if not hasattr(self, "_last_gps_warn_time"):
                    self._last_gps_warn_time = now
                if (now - self._last_gps_warn_time).nanoseconds / 1e9 > 0.5:
                    self._last_gps_warn_time = now
                    self.get_logger().warn(
                        f"GPS stale: age={gps_age:.2f}s "
                        f"(bad={self.bad_gps_count}/{self.n_bad_gps_threshold})"
                    )
            else:
                self.bad_gps_count = 0
            
            # Threshold reached -> fallback
            if self.bad_gps_count >= self.n_bad_gps_threshold:
                self._enter_fallback(now)
        
        # --- State machine ---
        if self.state == SystemState.NORMAL:
            cmd = self.last_nav_cmd if self.last_nav_cmd else Twist()
            heading_error = 0.0

            if not hasattr(self, "_last_normal_info_time"):
                self._last_normal_info_time = now
            if (now - self._last_normal_info_time).nanoseconds / 1e9 > 1.0:
                self._last_normal_info_time = now
                gps_status = self.last_gps_msg.status.status if self.last_gps_msg else -1
                lat = self.last_gps_msg.latitude if self.last_gps_msg else float('nan')
                lon = self.last_gps_msg.longitude if self.last_gps_msg else float('nan')
                v = self.last_nav_cmd.linear.x if self.last_nav_cmd else 0.0
                self.get_logger().info(
                    f"NORMAL: GPS OK (status={gps_status}, age={gps_age:.2f}s) | "
                    f"pos: {lat:.6f}, {lon:.6f} | v={v:.2f}"
                )

            
        elif self.state == SystemState.FALLBACK:
            cmd = self.compute_fallback_command()
            heading_error = self.compute_heading_error()

            if self.fallback_start_time:
                dur = (now - self.fallback_start_time).nanoseconds / 1e9
                remaining = self.fallback_max_duration_s - dur
                warn_start = 0.8 * self.fallback_max_duration_s

                # --- Log heartbeat until warn phase ---
                if dur < warn_start:
                    if not hasattr(self, "_last_fb_info_time"):
                        self._last_fb_info_time = now
                    if (now - self._last_fb_info_time).nanoseconds / 1e9 > 1.0:
                        self._last_fb_info_time = now
                        self.get_logger().info(
                            f"FALLBACK active: v={cmd.linear.x:.2f} rad/s={cmd.angular.z:.2f} "

                        )

                # --- Countdown after 80% ---
                elif dur >= warn_start and remaining > 0:
                    secs_left = int(remaining)
                    if secs_left != getattr(self, "_last_warn_secs", None):
                        self.get_logger().warn(
                            f"Robot aborting in: {secs_left:02d}.0s "
                            f"(elapsed {dur:.1f}/{self.fallback_max_duration_s:.1f}s)"
                        )
                        self._last_warn_secs = secs_left

                # --- Final abort ---
                if dur > self.fallback_max_duration_s:
                    if not getattr(self, "_abort_logged", False):
                        self.get_logger().error(
                            f"ABORT: Fallback timeout at ({dur:.2f}s) — Robot stopping safely."
                        )
                        self.metrics.finish(now, 'ABORT', abort_reason='Timeout')
                        self.publish_zero_velocity()
                        self._abort_logged = True
                        self._shutdown_requested = True
                        try:
                            self.timer.cancel()
                        except Exception:
                            pass
                    return


                    
            
        elif self.state == SystemState.RECOVERY:
            cmd = self.compute_recovery_command()
            heading_error = self.compute_heading_error()
            

            # RECOVERY → NORMAL
            if self.recovery_start_time and (now - self.recovery_start_time).nanoseconds / 1e9 >= self.recovery_blend_duration_s:
                self.state = SystemState.NORMAL
                self.get_logger().info("STATE: RECOVERY → NORMAL | Blend complete")
        else:
            cmd = Twist()
            heading_error = 0.0
        
        # Publish
        self.cmd_vel_pub.publish(cmd)
        
        # Publish status
        status = String()
        status.data = self.state.value
        self.status_pub.publish(status)
        
        # Log to CSV
        if self.enable_csv_logging:
            self.log_to_csv(cmd, heading_error)

    def should_abort(self) -> bool:
        now = self.get_clock().now()
        # Check sensor staleness
        if self.last_imu_msg and self.last_imu_time and (now - self.last_imu_time).nanoseconds / 1e9 > 1.0:
            self.get_logger().error("IMU data stale - ABORTING")
            self.metrics.finish(now, 'ABORT', abort_reason='IMU stale')
            return True

        if self.last_odom_msg and self.last_odom_time and (now - self.last_odom_time).nanoseconds / 1e9 > 1.0:
            self.get_logger().error("Odom data stale - ABORTING")
            self.metrics.finish(now, 'ABORT', abort_reason='Odom stale')
            return True

        return False


    def publish_zero_velocity(self) -> None:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def compute_fallback_command(self) -> Twist:
        """Dead reckoning mode"""
        cmd = Twist()
        if not self.last_imu_msg:
            return cmd
        current_heading = self.get_heading_from_imu()
        heading_error = self.normalize_angle(self.last_known_heading - current_heading)
        angular_correction = self.heading_correction_kp * heading_error
        angular_correction = max(-self.max_angular_correction, min(self.max_angular_correction, angular_correction))
        cmd.linear.x = self.fallback_linear_velocity
        cmd.angular.z = angular_correction
        return cmd

    def compute_heading_error(self) -> float:
        if not self.last_imu_msg:
            return 0.0
        current_heading = self.get_heading_from_imu()
        return self.normalize_angle(self.last_known_heading - current_heading)

    def compute_recovery_command(self) -> Twist:
        """Blend fallback → nav command"""
        cmd = Twist()
        if not self.recovery_start_time or not self.last_nav_cmd or not self.last_imu_msg:
            return cmd
        elapsed = (self.get_clock().now() - self.recovery_start_time).nanoseconds / 1e9
        weight = min(1.0, elapsed / self.recovery_blend_duration_s)
        fallback_cmd = self.compute_fallback_command()
        nav_cmd = self.last_nav_cmd
        cmd.linear.x = (1 - weight) * fallback_cmd.linear.x + weight * nav_cmd.linear.x
        cmd.angular.z = (1 - weight) * fallback_cmd.angular.z + weight * nav_cmd.angular.z
        return cmd

    def normalize_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def get_heading_from_imu(self) -> float:
        """Extract yaw from quaternion"""
        if not self.last_imu_msg:
            return 0.0
        q = self.last_imu_msg.orientation
        # transforms3d expects [w, x, y, z]
        roll, pitch, yaw = quat2euler([q.w, q.x, q.y, q.z])
        return yaw

    def log_to_csv(self, cmd: Twist, heading_error: float) -> None:
        if not self.csv_writer:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        gps_status = self.last_gps_msg.status.status if self.last_gps_msg else -1
        self.csv_writer.writerow([
            now,
            self.state.value,
            gps_status,
            cmd.linear.x,
            cmd.angular.z,
            heading_error
        ])
        self.csv_file.flush()
    

def main(args=None):
    rclpy.init(args=args)
    node = AxisReliabilityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # Stop any periodic logging before shutdown
        node._shutdown_requested = True
        try:
            node.timer.cancel()
        except Exception:
            pass

        # Last log via ROS logger (still valid here)
        node.get_logger().info("Shutting down — finalizing metrics...")

        # Do summary/metrics BEFORE destroying the node / shutdown
        end = node.get_clock().now()
        if node.state == SystemState.FALLBACK:
            node.metrics.finish(end, 'ABORT', abort_reason='UserInterrupt')
        elif node.state == SystemState.RECOVERY:
            node.metrics.finish(end, 'RECOVERY')
        else:
            node.metrics.finish(end, 'CLEAN_EXIT')
    finally:
        # Close files and destroy node (no more ROS logging past this point)
        if node.csv_file:
            node.csv_file.close()
        node.destroy_node()

        # Make shutdown idempotent; ignore if already shut down
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()