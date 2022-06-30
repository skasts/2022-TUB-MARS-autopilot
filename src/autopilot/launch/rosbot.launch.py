import os
import launch
import math

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Create launch description object
    ld = launch.LaunchDescription()

    # Get launch directories
    autopilot_dir = get_package_share_directory('autopilot')
    gazebo_ros_dir = get_package_share_directory('gazebo_ros')
    slam_toolbox_dir = get_package_share_directory('slam_toolbox')

    # Define path to urdf file and read contents
    urdf = os.path.join(autopilot_dir, 'models', 'rosbot.urdf')
    with open(urdf, 'r') as infp:
        robot_desc = infp.read()

    ###############
    ## Arguments ##
    ###############

    # Create launch configuration variables
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    name = LaunchConfiguration('name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    params_file = LaunchConfiguration('params_file')
    bt_xml_file = LaunchConfiguration('bt_xml_file')
    simulation = LaunchConfiguration('simulation') # This also sets use_sim_time for all subsequent launch files
    autostart = LaunchConfiguration('autostart')
    map_subscribe_transient_local = LaunchConfiguration('map_subscribe_transient_local')
    use_rviz = LaunchConfiguration('use_rviz')

    # Declare the launch arguments
    declare_gazebo_world_cmd = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(autopilot_dir, 'worlds', 'my_willowgarage.world'),
        description='SDF world file')

    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='True',
        description='Whether to execute gzclient')

    declare_name_cmd = DeclareLaunchArgument(
        'name',
        default_value='robot',
        description='The name of the model to spawn')

    declare_x_cmd = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='X-value to spawn to')

    declare_y_cmd = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Y-value to spawn to')
    
    declare_z_cmd = DeclareLaunchArgument(
        'z',
        default_value='0.03',
        description='Z-value to spawn to')

    declare_yaw_cmd = DeclareLaunchArgument(
        'yaw',
        default_value=str(math.pi),
        description='Yaw-value to spawn to')

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(autopilot_dir, 'config', 'default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(autopilot_dir, 'config', 'params_rosbot.yaml'),
        description='Full path to the nav2 params file')

    declare_bt_xml_file_cmd = DeclareLaunchArgument(
        'bt_xml_file',
        default_value=os.path.join(autopilot_dir, 'behavior_trees', 'navigate_w_replanning_time.xml'),
        description='Full path to the behavior tree xml file')

    declare_simulation_cmd = DeclareLaunchArgument(
        'simulation',
        default_value='True',
        description='Launch simulation')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='True',
        description='Automatically startup the nav2 stack')

    declare_map_subscribe_transient_local_cmd = DeclareLaunchArgument(
        'map_subscribe_transient_local',
        default_value='True',
        description='Whether to set the map subscriber QoS to transient local')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether or not to launch rviz')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        # Substitute scan_topic depending on 'simulation' parameter. It has to be
        # "/rp_lidar/out" for simulation and "/scan" for real robot. Depending on
        # the node it is accessed via 'scan_topic' or 'scan.topic'
        'scan_topic': PythonExpression(['"/rp_lidar/out" if "True" == "', simulation, '" else "/scan"']),
        'scan.topic': PythonExpression(['"/rp_lidar/out" if "True" == "', simulation, '" else "/scan"']),
        'use_sim_time': simulation,
        'default_bt_xml_filename': bt_xml_file,
        'autostart': autostart,
        'map_subscribe_transient_local': map_subscribe_transient_local}

    # Rewrite params YAML to reflect given parameters
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites=param_substitutions,
        convert_types=True)

    #####################
    ## Specify actions ##
    #####################

    # Publish coordinate transformations
    tf1_cmd = Node(
        condition=IfCondition(simulation),
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '-1.5707', '0', '-1.5707', 'camera_link', 'camera_depth_frame'])

    tf2_cmd = Node(
        condition=IfCondition(simulation),
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['-0.03', '0', '0.11', '0', '0', '0', 'base_link', 'camera_link'])

    tf3_cmd = Node(
        condition=IfCondition(simulation),
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0.07', '0', '0', '0', 'base_link', 'laser'])

    # Gazebo server
    # This executes 'gzserver install/rosbot_config/share/rosbot_config/worlds/my_willowgarage.world 
    # --verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so -s libgazebo_ros_force_system.so'
    gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzserver.launch.py')),
        condition=IfCondition(simulation),
        launch_arguments={'verbose': 'True'}.items())

    # Gazebo client
    gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_dir, 'launch', 'gzclient.launch.py')),
        condition=UnlessCondition(headless))

    # Spawn rosbot
    spawn_rosbot_cmd = Node(
        condition=IfCondition(simulation),
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        remappings = [('/rp_lidar/out', '/scan')],
        arguments=['-entity', name, '-x', x, '-y', y, '-z', z, '-Y', yaw,
                   '-file', autopilot_dir + '/models/rosbot.sdf'])

    # Robot description
    robot_description_cmd = Node(
        condition=IfCondition(simulation),
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True, 
                     'robot_description': robot_desc}],
        arguments=[urdf])

    # Slam
    start_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(slam_toolbox_dir, 'launch', 'online_sync_launch.py')),
        launch_arguments={'params_file': configured_params}.items())

    # RViz
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    # -------------------------------- Nav2 stuff ----------------------------------

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
        parameters=[{'use_sim_time': simulation},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}])

    ############################
    ## Declare launch options ##
    ############################

    # Arguments
    ld.add_action(declare_bt_xml_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_simulation_cmd)
    ld.add_action(declare_gazebo_world_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_name_cmd)
    ld.add_action(declare_x_cmd)
    ld.add_action(declare_y_cmd)
    ld.add_action(declare_z_cmd)
    ld.add_action(declare_yaw_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_map_subscribe_transient_local_cmd)
    ld.add_action(declare_use_rviz_cmd)

    # ROSBot transformations
    ld.add_action(tf1_cmd)
    ld.add_action(tf2_cmd)
    ld.add_action(tf3_cmd)

    # Included launch files
    ld.add_action(gazebo_client_cmd)
    ld.add_action(gazebo_server_cmd)
    ld.add_action(spawn_rosbot_cmd)
    ld.add_action(robot_description_cmd)
    ld.add_action(start_slam_toolbox_cmd)

    # Nav2 nodes
    # ld.add_action(nav2_controller_cmd)
    # ld.add_action(nav2_planner_cmd)
    # ld.add_action(nav2_recoveries_cmd)
    # ld.add_action(nav2_bt_navigator_cmd)
    # ld.add_action(nav2_waypoint_follower_cmd)
    # ld.add_action(nav2_lifecycle_manager_cmd)

    # RViz
    ld.add_action(rviz_cmd) 

    return ld