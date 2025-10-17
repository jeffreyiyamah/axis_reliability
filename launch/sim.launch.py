from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

os.environ['TURTLEBOT3_MODEL'] = 'burger'

def generate_launch_description():
    home = os.path.expanduser("~")

    # --- Gazebo world (TurtleBot3 + orchard) ---
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.environ['CONDA_PREFIX'], '/share/turtlebot3_gazebo/launch/turtlebot3_world.launch.py']
        ),
        launch_arguments={'world': f"{home}/ros2_ws/src/orchard_world/worlds/orchard.world"}.items(),
    )

    # --- Axis Reliability Node ---
    axis_node = Node(
        package='axis_reliability',
        executable='axis_reliability',
        name='axis_reliability',
        output='screen',
        remappings=[
            ('/cmd_vel_nav', '/cmd_vel'),
            ('/cmd_vel_out', '/cmd_vel'),
        ]
    )

    # --- Odom → Fix Fake GPS Node ---
    odom_to_fix = ExecuteProcess(
        cmd=['python3', os.path.expanduser('~/ros2_ws/fake_sensors/odom_to_fix.py')],
        output='screen'
    )

    return LaunchDescription([
        gazebo_launch,
        axis_node,
        odom_to_fix
    ])
