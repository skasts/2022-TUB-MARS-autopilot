import os
import launch

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    ekf_cmd = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[PathJoinSubstitution([FindPackageShare("mars_robot"), "config", "ekf.yaml"])],
    )

    ld.add_action(ekf_cmd)

    return ld