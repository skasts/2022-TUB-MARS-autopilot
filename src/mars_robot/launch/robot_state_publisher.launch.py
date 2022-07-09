import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    declare_use_sim_time_cmd = DeclareLaunchArgument(name="use_sim_time", default_value="True")
    
    urdf = os.path.join(get_package_share_directory("mars_robot"), "urdf/mars_robot.urdf")
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 
                     'robot_description': robot_desc}]
        # arguments=[urdf],
    )

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(state_publisher_cmd)

    return ld

    # TODO: # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare("diffbot_description"), "urdf", "diffbot_system.urdf.xacro"]
    #         ),
    #     ]
    # )
