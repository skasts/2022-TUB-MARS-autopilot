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

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("mars_robot"), "urdf", "mars_robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_state_pub_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    diffbot_diff_drive_controller = os.path.join(get_package_share_directory("mars_robot"),"config/diff_drive_controller.yaml")
    controller_manager_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, diffbot_diff_drive_controller],
        remappings=[
            ("/diffbot_base_controller/cmd_vel_unstamped", "/cmd_vel"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["diffbot_base_controller"],
        output="screen",
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    ld.add_action(controller_manager_cmd)
    ld.add_action(robot_state_pub_cmd)
    ld.add_action(spawn_jsb_controller)
    ld.add_action(spawn_dd_controller)
    
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
