# 2022-TUB-MARS-autopilot

This is our code repository for the MARS project in 2022. 

## Installation

### Docker
Check [this side](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) for the installation. Choose the installation using the repositories.
Add your used to the docker group. This lets you run docker commands without sudo
```
sudo usermod -aG docker $USER
```

### Commands
colcon build --symlink-install
. install/local_setup.bash
ros2 run virtual_maize_field generate_world fre22_task_navigation_mini --show_map
pkill gzserver
ros2 launch virtual_maize_field simulation.launch.py headless:='True' verbose:='True'
ros2 launch jackal_custom robot_state_publisher.launch.py
ros2 launch jackal_custom spawn_robot.launch.py
ros2 launch gazebo_ros gzclient.launch.py use_sim_time:='True' verbose:='True'