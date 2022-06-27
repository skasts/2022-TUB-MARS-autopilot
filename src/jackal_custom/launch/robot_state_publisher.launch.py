import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    declare_use_sim_time_cmd = DeclareLaunchArgument(name="use_sim_time", default_value="True")
    
    urdf = os.path.join(
        get_package_share_directory("jackal_custom"), "urdf/jackal.urdf"
    )

    state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[urdf],
    )

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(state_publisher_cmd)

    return ld