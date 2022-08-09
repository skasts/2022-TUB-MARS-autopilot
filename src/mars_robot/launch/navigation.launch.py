# This launches the Nav2 navigation stack and RViz with a navigation view. The parameters for Nav2
# are set in params.yaml but are in desperate need of being tuned. Some of the parameters for i.e.
# the global and local costmap do not make sense at the moment.

import os
import launch
import math

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    # Get launch directories
    autopilot_dir = get_package_share_directory('autopilot')

    # Create launch configuration variables
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    
    start_rviz_cmd = DeclareLaunchArgument("start_rviz",default_value="True")
    declare_use_sim_time_cmd = DeclareLaunchArgument(name="use_sim_time", default_value="False")
    declare_params_file_cmd = DeclareLaunchArgument('params_file',default_value=os.path.join(autopilot_dir, 'config', 'params.yaml'))
    declare_bt_xml_file_cmd = DeclareLaunchArgument('bt_xml_file',default_value=os.path.join(autopilot_dir, 'config', 'navigate_w_replanning_time.xml'))
    
    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument('map_subscribe_transient_local',default_value='True')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'default_bt_xml_filename': bt_xml_file,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    # Rewrite params YAML to reflect given parameters
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    # (Nav2-) Nodes to be controlled by lifecycle manager 
    lifecycle_nodes = ['controller_server',
                       'planner_server',
                       'recoveries_server',
                       'bt_navigator',
                       'waypoint_follower']

    # Nav2 controller, does local planning
    nav2_controller_cmd = Node(
        package='nav2_controller',
        executable='controller_server',
        output='screen',
        parameters=[configured_params])

    # Nav2 planner, does global planning
    nav2_planner_cmd = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params])

    # Nav2 recoveries, defines what to do when navigation fails
    nav2_recoveries_cmd = Node(
        package='nav2_recoveries',
        executable='recoveries_server',
        name='recoveries_server',
        output='screen',
        parameters=[configured_params])

    # Nav2 behavior tree navigator. The behavior tree does the mission planning
    nav2_bt_navigator_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params])

    # Nav2 waypoint follower. I.e. we use this when passing goal poses via RViz
    nav2_waypoint_follower_cmd = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params])

    # Nav2 lifecycle manager that manages the nodes lifecycle (startup, activation, ...)
    nav2_lifecycle_manager_cmd = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}])

    rviz_config_file = PathJoinSubstitution([FindPackageShare("mars_robot"), "config", "nav_view.rviz"])

    rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    ############################
    ## Declare launch options ##
    ############################

    # Arguments
    ld.add_action(declare_bt_xml_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(start_rviz_cmd)

    # Nav2 nodes
    ld.add_action(nav2_controller_cmd)
    ld.add_action(nav2_planner_cmd)
    ld.add_action(nav2_recoveries_cmd)
    ld.add_action(nav2_bt_navigator_cmd)
    ld.add_action(nav2_waypoint_follower_cmd)
    ld.add_action(nav2_lifecycle_manager_cmd)

    # RViz
    ld.add_action(rviz_cmd)

    return ld