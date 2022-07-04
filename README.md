# 2022-TUB-MARS-autopilot

This repository contains our code for the autopilot B group from the MARS project course in summer semester 2022 at the TU Berlin. The workspace is meant to run inside a Docker container. It is also preconfigured to enable developing from inside of this container.

## Installation

### Docker
Check [this side](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) for the installation. Choose the installation using the repositories.
Add your used to the docker group. This lets you run docker commands without sudo. You might need to reboot afterwards.
```
sudo usermod -aG docker $USER
```
Check if you are able to run docker without sudo. Check, whether executing `docker ps` works.

### Clone and build workspace
Clone this workspace. To be able to do so, you need to have created a SSH-key and added it to you Github account, see [here](https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent#generating-a-new-ssh-key).
```
git clone git@github.com:skasts/2022-TUB-MARS-autopilot.git
```
Clone external repositories into src.
```
cd src/
vcs import < ros2.repos
cd ..
```
Build workspace. This might take ~20 minutes, depending on your computer. Every time you change a package, it has to be rebuilt (unless only changing python files). Just use this command again, unchanged packages will not be built again.
```
colcon build --symlink-install
```
Source workspace. This has to be done in every terminal that we use.
```
. install/local_setup.bash
```

## Usage of container
It follow some commands on running parts of the system. Remember to `. install/local_setup.bash` in each terminal before running one of the commands.

### Generate Maize world
If you want to use a maize field world for Gazebo, you need to generate one. This only needs to be done once. You can close the map viewer afterwards, that is only for visualization.
```
ros2 run virtual_maize_field generate_world fre22_task_navigation_mini --show_map
```
### Run whole autopilot in simulation
To launch the full autopilot in Gazebo with Slam Toolbox and Navigation, use this overall launch file.
```
ros2 launch autopilot autopilot.launch.py
```

### Run parts of system separately
Of course, you can also only run parts of the system. 
#### Gazebo simulation with Maize world
This runs the Gazebo simulation (Server and GUI).
```
ros2 launch virtual_maize_field simulation.launch.py verbose:='True'
```
By adding `headless:='True'` to the command, you start Gazebo without GUI. This can reduce computational load. If needed, you can start the GUI later with `ros2 launch gazebo_ros gzclient.launch.py use_sim_time:='True' verbose:='True'`.
#### Publish Jackal's transformations
```
ros2 launch jackal_custom robot_state_publisher.launch.py
```
#### Spawn Jackal robot to Gazebo
```
ros2 launch jackal_custom spawn_robot.launch.py
```
#### Run RViz
This starts RViz with a preconfigured view.
```
rviz2 -d src/autopilot/config/default_view.rviz
```

#### Run ORB-Slam3 (example, standalone)
Preparation: Change line 83 in `Examples/Monocular/mono_euroc.cc` to `ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::MONOCULAR, true);` (was false before). This activates the viewers. Also, the example requires the EuRoC dataset (V101) to be extracted into `/workspaces/2022-TUB-MARS-autopilot/`.
```
cd /ORB_SLAM3
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml ../workspaces/2022-TUB-MARS-autopilot/ Examples/Monocular/EuRoC_TimeStamps/V101.txt 
```

## Debug information

#### Other useful commands

##### Kill all old Gazebo processes
Often it happens, that Gazebo does not terminate completely after shutting it down. Then, if you start it again, you get an error like 'there is already an instance existing'. To kill any old Gazebo process, use the following.
```
pkill gzserver
```
##### Restart ROS2 daemon
If you experience unwanted behavior with ROS, i.e. no nodes found with `ros2 node list`, stop and start the daemon again.
```
ros2 daemon stop; ros2 daemon start
```
##### Get all transformations
The following creates a PDF ('frames.pdf') with all current TFs. 
```
ros2 run tf2_tools view_frames.py
```
##### Delete a model from Gazebo
To delete an entity from Gazebo, use this.
```
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity "{name: 'robot'}"
```
#### Control robot from keyboard
The robot received velocity commands of the topic `/cmd_vel`. You can directly publish onto this topic with a keyboard controller package.
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
#### Configure git in container if you want to commit / push
If you want to commit and push to the repository, git in the container needs to know who you are. Tell it with the following commands.
```
git config --global user.name "Simon Kast"
git config --global user.email 57326251+skasts@users.noreply.github.com
```