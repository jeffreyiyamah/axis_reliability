from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('axis_reliability'),
        'config',
        'axis_reliability.yaml'  # reuse same config
    )

    return LaunchDescription([
        Node(
            package='axis_reliability',
            executable='axis_baseline',
            name='axis_baseline',
            output='screen',
            parameters=[config],
            emulate_tty=True,
        )
    ])
