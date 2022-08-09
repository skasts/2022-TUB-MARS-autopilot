# This spawns a robot entity of the MARS robot into a running Gazebo simulation

import launch
import math

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')

    declare_name_cmd = DeclareLaunchArgument('name',default_value='mars_robot')
    declare_x_cmd = DeclareLaunchArgument('x',default_value='0.0')
    declare_y_cmd = DeclareLaunchArgument('y',default_value='0.0')
    declare_z_cmd = DeclareLaunchArgument('z',default_value='1.0')
    declare_yaw_cmd = DeclareLaunchArgument('yaw',default_value=str(math.pi))

    mars_robot_dir = get_package_share_directory('mars_robot')
    # Spawn mars_robot entity
    spawn_mars_robot_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        # remappings = [('/rp_lidar/out', '/scan')],
        arguments=['-entity', name, '-x', x, '-y', y, '-z', z, '-Y', yaw,
                   '-file', mars_robot_dir + '/urdf/mars_robot.sdf']
    )

    ld.add_action(declare_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)

    ld.add_action(spawn_mars_robot_cmd)

    return ld