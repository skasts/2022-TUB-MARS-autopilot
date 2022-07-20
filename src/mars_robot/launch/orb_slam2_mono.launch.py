import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Create launch description object
    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    voc_file = LaunchConfiguration("voc_file")
    camera_ini_file = LaunchConfiguration("camera_ini_file")

    use_sim_time_cmd = DeclareLaunchArgument("use_sim_time",default_value="false")
    params_file_cmd = DeclareLaunchArgument("params_file",default_value=os.path.join(get_package_share_directory("mars_robot"), "config", "slam.yaml"))
    voc_file_cmd = DeclareLaunchArgument("voc_file",default_value=os.path.join(get_package_share_directory("orb_slam2_ros"), "orb_slam2", "Vocabulary", "ORBvoc.txt"))
    camera_ini_file_cmd = DeclareLaunchArgument("camera_ini_file",default_value=os.path.join(get_package_share_directory("mars_robot"),"config", "camera_info_640x480.ini"))

    camera_driver_cmd = Node(
        package='opencv_cam',
        executable='opencv_cam_main',
        name='camera_driver_node',
        output='screen',
        remappings=[
            ("/image_raw", "/camera/image_raw"),
            ("/camera_info", "/camera/camera_info"),
        ],
        parameters=[{
            "width": 640,
            "height": 480,
            "file": False,
            "filename": "video_maisfeld.mp4", # Only used when file set to true
            "camera_frame_id": "camera_link",
            "camera_info_path": camera_ini_file
            },
        ],
    )

    covariance_adder_cmd = Node(
        package='covariance_adder',
        executable='covariance_adder',
        name='covariance_adder_node',
        output='screen',
        parameters=[params_file],
    )

    slam_cmd = Node(
        package="orb_slam2_ros",
        executable="orb_slam2_ros_mono",
        name="orb_slam2_mono",
        parameters=[
            params_file,
            {"voc_file": voc_file,
            "use_sim_time": use_sim_time},
        ],
        output="screen"
    )
    
    ld.add_action(use_sim_time_cmd)
    ld.add_action(voc_file_cmd)
    ld.add_action(params_file_cmd)
    ld.add_action(camera_ini_file_cmd)

    ld.add_action(camera_driver_cmd)
    ld.add_action(covariance_adder_cmd)
    ld.add_action(slam_cmd)

    return ld
