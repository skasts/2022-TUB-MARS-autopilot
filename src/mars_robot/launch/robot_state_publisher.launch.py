# This starts a robot state publisher for the MARS robot (using the describing .xacro)

import launch

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time", default="False")
    declare_use_sim_time_cmd = DeclareLaunchArgument(name="use_sim_time", default_value="False")
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mars_robot"), "urdf", "mars_robot.urdf.xacro"]
                # [FindPackageShare("diffbot_description"), "urdf", "diffbot_system.urdf.xacro"]
            ),
        ]
    )

    state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'robot_description': robot_description_content}]
    )

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(state_publisher_cmd)

    return ld