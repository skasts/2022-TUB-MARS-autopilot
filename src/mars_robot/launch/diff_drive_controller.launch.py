# This starts a robot state publisher for the MARS robot (using the describing .xacro), a controller
# with a differential drive plugin (cmd_vel -> speeds left and right) and a joint state broadcaster
# plugin (transformations for wheels according to the set speed)

import os
import launch

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
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
                # [FindPackageShare("diffbot_description"), "urdf", "diffbot_system.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    diff_drive_controller = os.path.join(get_package_share_directory("mars_robot"),"config/diff_drive_controller.yaml")

    robot_state_pub_cmd = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    controller_manager_cmd = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, diff_drive_controller],
        remappings=[
            ("/base_controller/cmd_vel_unstamped", "/cmd_vel"),
            ("/odom", "/base_controller/odom"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
    )

    spawn_dd_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["base_controller"],
        output="screen",
    )

    spawn_jsb_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    ld.add_action(robot_state_pub_cmd)
    ld.add_action(controller_manager_cmd)
    ld.add_action(spawn_dd_controller)
    ld.add_action(spawn_jsb_controller)

    return ld