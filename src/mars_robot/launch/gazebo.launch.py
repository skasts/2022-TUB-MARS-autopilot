# This starts a Gazebo simulation with the Virtual Maize Field world and spawns the MARS robot into
# that world

import os
import launch
import math

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    # Create launch configuration variables
    headless = LaunchConfiguration('headless')

    # Declare the launch arguments
    declare_headless_cmd = DeclareLaunchArgument('headless',default_value='False',
        description='Whether to execute gzclient')
   
    # Start Robot Field Event Gazebo Simulation
    gazebo_simulation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory("virtual_maize_field") + "/launch/simulation.launch.py"),
        launch_arguments={"headless": headless}.items())

    # Spawn MARS robot
    spawn_robot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(get_package_share_directory("mars_robot") + "/launch/spawn_robot.launch.py"),
        launch_arguments={

        }.items())

    # Arguments
    ld.add_action(declare_headless_cmd)

    # Included launch files
    ld.add_action(gazebo_simulation_cmd)
    ld.add_action(spawn_robot_cmd)

    return ld