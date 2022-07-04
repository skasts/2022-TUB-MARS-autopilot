# Default
FROM althack/ros2:foxy-gazebo AS althack
# If you have a nvidia GPU with Cuda installed
# FROM althack/ros2:foxy-gazebo-nvidia AS althack 

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
   # Requirements for Slam Toolbox
   python3-rospkg \
   # General deps:
   software-properties-common \
   # Pangolin deps:
   libc++-10-dev \
   libc++-dev \
   libc++1-10 \
   libc++abi1-10 \
   libpython3.9 \
   libpython3.9-dev \
   libpython3.9-minimal \
   libpython3.9-stdlib \
   ninja-build \
   python3.9 \
   python3.9-dev \
   python3.9-minimal \
   wayland-protocols \
   # OpenCV deps:
   build-essential \
   cmake \
   git \
   pkg-config \
   libgtk-3-dev \
   libavcodec-dev \
   libavformat-dev \
   libswscale-dev \
   libv4l-dev \
   libxvidcore-dev \
   libx264-dev \
   libjpeg-dev \
   libpng-dev \
   libtiff-dev \
   gfortran \
   openexr \
   libatlas-base-dev \
   python3-dev \
   python3-numpy \
   libtbb2 \
   libtbb-dev \
   libdc1394-22-dev \
   libopenexr-dev \
   libgstreamer-plugins-base1.0-dev \
   libgstreamer1.0-dev \
   # ORB-Slam3 deps:
   libpython2.7-dev \
   # Clean up
   && apt-get autoremove -y \
   && apt-get clean -y \
   && rm -rf /var/lib/apt/lists/*
ENV DEBIAN_FRONTEND=dialog

FROM althack AS w_requirements

# Build OpenCV
RUN git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    # git checkout 4.4.0 && \
    mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=RELEASE \
    -D CMAKE_INSTALL_PREFIX=/usr/local \
    -D INSTALL_C_EXAMPLES=OFF \
    -D INSTALL_PYTHON_EXAMPLES=OFF \
    -D BUILD_PERF_TESTS=OFF \
    -D BUILD_DOCS=OFF \
    -D BUILD_EXAMPLES=OFF .. && \
    make -j8 && make install && \
    cd / && rm -rf opencv && \
    ldconfig

# Build Pangolin
RUN git clone https://github.com/stevenlovegrove/Pangolin && \
    cd Pangolin && mkdir build && cd build && \
    cmake .. && make -j$nproc && make install && \
    cd / && rm -rf Pangolin && \
    ldconfig

# Build ORB-Slam3
RUN git clone https://github.com/UZ-SLAMLab/ORB_SLAM3 && \
    cd ORB_SLAM3 && \
    sed -i 's/++11/++14/g' CMakeLists.txt && \
    ./build.sh

# Source Gazebo's setup file
RUN echo "source /usr/share/gazebo/setup.sh" >> /home/ros/.bashrc
ENV DEBIAN_FRONTEND=

# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc

# # Install realsense2 driver
# RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
#     || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
#     add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u && \
#     apt-get update && \
#     apt-get -y install --no-install-recommends \
#     librealsense2-dkms \
#     librealsense2-utils \
#     librealsense2-dev \
#     # Clean up
#     && apt-get autoremove -y \
#     && apt-get clean -y \
#     && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog