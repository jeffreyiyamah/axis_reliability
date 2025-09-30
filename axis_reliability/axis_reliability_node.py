import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from transforms3d.euler import quat2euler

class SystemState(Enum):
    NORMAL = "NORMAL"
    FALLBACK = "FALLBACK"
    RECOVERY = "RECOVERY"


class AxisReliabilityNode(Node):
    def __init__(self):
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

        # Subscriptions
        self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.create_subscription(
            Imu,
            '/imu/data',
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
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info('AxisReliabilityNode initialized')

    def gps_callback(self, msg):
        # TODO: store message
        pass

    def imu_callback(self, msg):
        # TODO: store message
        pass

    def odom_callback(self, msg):
        # TODO: store message
        pass

    def cmd_vel_nav_callback(self, msg):
        # TODO: store message
        pass

    def control_loop(self):
        # TODO: state machine + velocity publishing
        pass


def main(args=None):
    rclpy.init(args=args)
    node = AxisReliabilityNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()