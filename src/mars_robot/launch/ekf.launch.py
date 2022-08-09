# This starts the robot localization package that runs a Kalman filter to fuse the pose estimates
# from the slam and the drive plugin. See `èkf.yaml` for config information. This could be extended
# by other pose estimates like one from an IMU sensor.

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