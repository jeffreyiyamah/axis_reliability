import unittest
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import time

class TestAxisStateMachine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_axis_state_machine')
        self.status_msgs = []
        self.cmd_vel_msgs = []
        self.status_sub = self.node.create_subscription(
            String, '/axis/status', self.status_callback, 10)
        self.cmd_vel_sub = self.node.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.gps_pub = self.node.create_publisher(NavSatFix, '/fix', 10)
        self.node.get_logger().info('Test node setup complete')

    def tearDown(self):
        self.node.destroy_node()

    def status_callback(self, msg):
        self.status_msgs.append((self.node.get_clock().now().nanoseconds, msg.data))

    def cmd_vel_callback(self, msg):
        self.cmd_vel_msgs.append((self.node.get_clock().now().nanoseconds, msg.linear.x, msg.angular.z))

    def test_dropout_triggers_fallback(self):
        """Test NORMAL → FALLBACK after N_bad threshold"""
        # TODO: Publish bad GPS, spin, check /axis/status for transition
        pass

    def test_fallback_maintains_velocity(self):
        """Test /cmd_vel.linear.x > 0 during FALLBACK"""
        # TODO: Publish bad GPS, spin, check /cmd_vel.linear.x
        pass

    def test_recovery_blends_smoothly(self):
        """Test no velocity jumps during RECOVERY"""
        # TODO: Simulate dropout and recovery, check /cmd_vel.linear.x acceleration
        pass

if __name__ == '__main__':
    unittest.main()



