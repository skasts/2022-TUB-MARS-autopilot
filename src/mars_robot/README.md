# Mars robot

This package is the core of the 2022-TUB-MARS-autopilot project. It consits of a description of the
MARS robot with links, joints and corresponding Gazebo information for simulation. Also, it has 
several launch files to start up the different components of the system. It comes with configuration
files for all the parts of the system. 

## Launch files
Use the following launch files to start parts of the system. The usage is 
```
ros2 launch mars_robot <name_launch_file>
```
- `robot_state_publisher.launch.py` starts a robot state publisher for the MARS robot 
(as decribed in *urdf/mars_robot.urdf.xacro*)
- `diff_drive_controller.launch.py` starts the robot state publisher together with a controller with 
a differential drive plugin (cmd_vel -> speeds left and right) and a joint state broadcaster plugin 
(transformations for wheels according to the set speed)
- `orb_slam2_mono.launch.py` starts the Slam algorithm (ORB-Slam2) for a mono camera, a OpenCV-ROS2 
camera driver and the covariance adder node. The camera driver is needed to get the camera images as 
ROS msgs, the covariance adder adds covariance information to the pose estimate we get from the 
slam. The covariance information is needed by the kalman filter to be able to fuse the slam's pose 
estimate with the odom pose estimate from the drive plugin. See *slam.yaml* for configs. 
*camera_info_640x480.ini* is the calibration file for the used camera, you need to do this for your 
particular camera for better results. You can tweak the config for the camera driver right within 
the launch file (resolution, fps, video from file, ...)
- `ekf.launch.py` starts the robot localization package that runs a Kalman filter to fuse the pose 
estimates from the slam and the drive plugin. See *ekf.yaml* for config information. This could be
extended by other pose estimates like one from an IMU sensor.
- `system.launch.py` starts all above mentioned parts of the system together by including the 
individual launch files. This does not start navigation, as the parameters still need to be tuned.
- `navigation.launch.py` starts the Nav2 navigation stack and RViz with a navigation view. The 
parameters for Nav2 are set in *params.yaml* but are in **desperate need of being tuned**. Some of 
the parameters for i.e. the global and local costmap do not make sense at the moment.
- `gazebo.launch.py` starts a Gazebo simulation with the Virtual Maize Field world and spawns the 
MARS robot into that world. This requires you to have generated a maize field world before. You can 
do this with `ros2 run virtual_maize_field generate_world fre22_task_navigation_mini --show_map`. 
Edit the corresponding `fre22_task_navigation_mini` config in its package to change the world's 
parameters.
