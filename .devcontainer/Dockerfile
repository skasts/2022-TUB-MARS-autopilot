# Author: Simon Kast
#
# Use Ubuntu 20.04 as base image
FROM ubuntu:20.04 AS base

ENV DEBIAN_FRONTEND=noninteractive

# Install language
RUN apt-get update && apt-get install -y \
  locales \
  && locale-gen en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
  && rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# Install timezone
RUN ln -fs /usr/share/zoneinfo/UTC /etc/localtime \
  && export DEBIAN_FRONTEND=noninteractive \
  && apt-get update \
  && apt-get install -y tzdata \
  && dpkg-reconfigure --frontend noninteractive tzdata \
  && rm -rf /var/lib/apt/lists/*

# ##################################### Base ROS2 image ############################################
# Install ROS2
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
  && curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt-get update && apt-get install -y \
    ros-foxy-ros-base \
    python3-argcomplete \
  && rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV ROS_DISTRO=foxy
ENV AMENT_PREFIX_PATH=/opt/ros/foxy
ENV COLCON_PREFIX_PATH=/opt/ros/foxy
ENV LD_LIBRARY_PATH=/opt/ros/foxy/lib
ENV PATH=/opt/ros/foxy/bin:$PATH
ENV PYTHONPATH=/opt/ros/foxy/lib/python3.8/site-packages
ENV ROS_PYTHON_VERSION=3
ENV ROS_VERSION=2
ENV DEBIAN_FRONTEND=

# ###################################### Dev ROS2 image ############################################
FROM base AS dev

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  gdb \
  git \
  pylint3 \
  python3-argcomplete \
  python3-colcon-common-extensions \
  python3-pip \
  python3-rosdep \
  python3-vcstool \
  vim \
  wget \
  # Install ros distro testing packages
  ros-foxy-ament-lint \
  ros-foxy-launch-testing \
  ros-foxy-launch-testing-ament-cmake \
  ros-foxy-launch-testing-ros \
  python3-autopep8 \
  && rm -rf /var/lib/apt/lists/* \
  && rosdep init || echo "rosdep already initialized" \
  # Update pydocstyle
  && pip install --upgrade pydocstyle

ARG USERNAME=ros
ARG USER_UID=1000
ARG USER_GID=$USER_UID
# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
  && useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME \
  # [Optional] Add sudo support for the non-root user
  && apt-get update \
  && apt-get install -y sudo \
  && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME\
  && chmod 0440 /etc/sudoers.d/$USERNAME \
  # Cleanup
  && rm -rf /var/lib/apt/lists/* \
  && echo "source /usr/share/bash-completion/completions/git" >> /home/$USERNAME/.bashrc \
  && echo "if [ -f /opt/ros/${ROS_DISTRO}/setup.bash ]; then source /opt/ros/${ROS_DISTRO}/setup.bash; fi" >> /home/$USERNAME/.bashrc
ENV DEBIAN_FRONTEND=

# ##################################### Full ROS2 image ############################################
FROM dev AS full

ENV DEBIAN_FRONTEND=noninteractive
# Install the full release
RUN apt-get update && apt-get install -y \
  ros-foxy-desktop \
  && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=

# ########################################## Gazebo ################################################
# ################
# Uncomment below to also install GAZEBO into the image
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update && apt-get install -y \
#   ros-foxy-gazebo* \
#   && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=
# # Source Gazebo's setup file
# RUN echo "source /usr/share/gazebo/setup.sh" >> /home/ros/.bashrc
# ENV DEBIAN_FRONTEND=

# ########################################### CUDA #################################################
# Uncomment below if you want to use CUDA. This requires NVIDIA graphics on the machine that runs 
# the image. Exposes the nvidia driver to allow opengl and installs dependencies for glvnd and X11.
# RUN apt-get update \
#  && apt-get install -y -qq --no-install-recommends \
#   libglvnd0 \
#   libgl1 \
#   libglx0 \
#   libegl1 \
#   libxext6 \
#   libx11-6
# # Env vars for the nvidia-container-runtime.
# ENV NVIDIA_VISIBLE_DEVICES all
# ENV NVIDIA_DRIVER_CAPABILITIES graphics,utility,compute
# ENV QT_X11_NO_MITSHM 1

# ################################## Specific to our project #######################################
# Install additional packages
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update \
   && apt-get -y install --no-install-recommends \
  #  # Requirements for virtual maize field
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
   # General deps:
   software-properties-common \
   # orb_slam2_ros reqs:
   ros-foxy-image-common \
   # opencv_cam reqs:
   ros-foxy-image-pipeline \
   # robot_localization reqs:
   libgeographic-dev \
   ros-foxy-diagnostic-updater \
   ros-foxy-geographic-msgs \
   # ros2 control
   ros-foxy-ros2-control \
   ros-foxy-ros2-controllers \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# Change ROS_DOMAIN_ID to a number [0,100] to separate you ROS2 systems from other ROS2 applications
# running on the same network. 0 is default.
# ARG WORKSPACE
# RUN echo "export ROS_DOMAIN_ID=1" >> /home/ros/.bashrc

# ######################################## ORB-Slam3 ###############################################
# The following is needed if you want to use ORB-SLAM3. This needs to be built by source and has
# several dependencies as Pangolin and OpenCV. Note, that we did not integrate ORB-Slam3 as a ROS2
# node. So you would only be able to run it standalone.
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends \
#    # Pangolin deps:
#    libc++-10-dev \
#    libc++-dev \
#    libc++1-10 \
#    libc++abi1-10 \
#    libpython3.9 \
#    libpython3.9-dev \
#    libpython3.9-minimal \
#    libpython3.9-stdlib \
#    ninja-build \
#    python3.9 \
#    python3.9-dev \
#    python3.9-minimal \
#    wayland-protocols \
#    # OpenCV deps:
#    build-essential \
#    cmake \
#    git \
#    pkg-config \
#    libgtk-3-dev \
#    libavcodec-dev \
#    libavformat-dev \
#    libswscale-dev \
#    libv4l-dev \
#    libxvidcore-dev \
#    libx264-dev \
#    libjpeg-dev \
#    libpng-dev \
#    libtiff-dev \
#    gfortran \
#    openexr \
#    libatlas-base-dev \
#    python3-dev \
#    python3-numpy \
#    libtbb2 \
#    libtbb-dev \
#    libdc1394-22-dev \
#    libopenexr-dev \
#    libgstreamer-plugins-base1.0-dev \
#    libgstreamer1.0-dev \
#    # ORB-Slam3 deps:
#    libpython2.7-dev \
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog

# # Build OpenCV
# RUN git clone https://github.com/opencv/opencv.git && \
#     cd opencv && \
#     # git checkout 4.4.0 && \
#     mkdir build && cd build && \
#     cmake -D CMAKE_BUILD_TYPE=RELEASE \
#     -D CMAKE_INSTALL_PREFIX=/usr/local \
#     -D INSTALL_C_EXAMPLES=OFF \
#     -D INSTALL_PYTHON_EXAMPLES=OFF \
#     -D BUILD_PERF_TESTS=OFF \
#     -D BUILD_DOCS=OFF \
#     -D BUILD_EXAMPLES=OFF .. && \
#     make -j8 && make install && \
#     cd / && rm -rf opencv && \
#     ldconfig

# # Build Pangolin
# RUN git clone https://github.com/stevenlovegrove/Pangolin && \
#     cd Pangolin && mkdir build && cd build && \
#     cmake .. && make -j$nproc && make install && \
#     cd / && rm -rf Pangolin && \
#     ldconfig

# # Build ORB-Slam3
# RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3 && \
#     cd ORB_SLAM3 && \
#     sed -i 's/++11/++14/g' CMakeLists.txt && \
#     ./build.sh