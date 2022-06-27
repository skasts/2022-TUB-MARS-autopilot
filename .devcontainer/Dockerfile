# FROM althack/ros2:foxy-dev 
FROM althack/ros2:foxy-gazebo-nvidia AS althack

# Install additional packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
   # Requirements for virtual maize field
   python3-jinja2 \
   python3-markupsafe \
   python3-shapely \
   python3-opencv \
   # Requirements for navigation2
   ros-foxy-test-msgs \
   ros-foxy-behaviortree-cpp-v3 \
   libsuitesparse-dev \
   libceres-dev \
   ros-foxy-joint-state-publisher \
   lcov \
   python3-zmq \
   libgraphicsmagick++1-dev \
   graphicsmagick-libmagick-dev-compat \
   ros-foxy-ompl \
   # Requirements for Robot state publisher
   ros-foxy-xacro \
   # Requirements for Jackal Robot model
   ros-foxy-velodyne-description \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

FROM althack AS w_requirements

# Source Gazebo's setup file
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/ros/.bashrc
ENV DEBIAN_FRONTEND=

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc