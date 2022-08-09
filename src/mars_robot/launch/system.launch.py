# This launch file starts all current parts of the system: The diff_drive_controller, slam, ekf and 
# RViz. This is done by including the individual launch files from this folder.

import os
import launch

from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    start_rviz = LaunchConfiguration("start_rviz")
    start_rviz_cmd = DeclareLaunchArgument("start_rviz",default_value="True")

    diff_drive_controller_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mars_robot"), "launch", "diff_drive_controller.launch.py")),
        launch_arguments={"start_rviz": "True"}.items())

    orb_slam2_mono_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mars_robot"), "launch", "orb_slam2_mono.launch.py")),
        launch_arguments={}.items())

    ekf_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("mars_robot"), "launch", "ekf.launch.py")),
        launch_arguments={}.items())

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("mars_robot"), "config", "default_view.rviz"]
    )

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )
    # Timed only with humble?
    # rviz_cmd_timed = TimerAction(
    #     period = 3.0,
    #     actions=[rviz_cmd]
    # )

    ld.add_action(start_rviz_cmd)

    ld.add_action(diff_drive_controller_cmd)
    ld.add_action(orb_slam2_mono_cmd)
    ld.add_action(ekf_cmd)
    ld.add_action(rviz_cmd)

    return ld