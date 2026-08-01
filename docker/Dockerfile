FROM robotis/ros:jazzy-ros-base-librealsense AS librealsense

# Use the base ROS2 Jazzy image
FROM ros:jazzy-ros-base AS open-manipulator

COPY --from=librealsense /opt/librealsense /usr/local/
COPY --from=librealsense /usr/local/lib/python3.12/dist-packages/pyrealsense2 /usr/local/lib/python3.12/dist-packages/pyrealsense2
COPY --from=librealsense /usr/src/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/
COPY --from=librealsense /usr/src/librealsense/config/99-realsense-d4xx-mipi-dfu.rules /etc/udev/rules.d/
ENV PYTHONPATH=/usr/local/lib

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

# Install realsense dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libusb-1.0-0-dev \
    libglfw3-dev \
    ros-${ROS_DISTRO}-image-transport-plugins \
    ros-${ROS_DISTRO}-usb-cam \
    vim \
    nano \
    && rm -rf /var/lib/apt/lists/*

ENV COLCON_WS=/root/ros2_ws
ENV DEPS_WS=/opt/open_manipulator_deps_ws
ENV ROS_DOMAIN_ID=30
WORKDIR ${COLCON_WS}

RUN mkdir -p ${DEPS_WS}/src && \
    cd ${DEPS_WS}/src && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_hardware_interface.git && \
    cd dynamixel_hardware_interface && git checkout 3375b3d0c99c5bcdab11b2dc15c89a2923e2bca2 && cd .. && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/dynamixel_interfaces.git && \
    cd dynamixel_interfaces && git checkout 3392e1214c62c7ee9e05ccc811e3e829c063bae8 && cd .. && \
    git clone -b jazzy https://github.com/ROBOTIS-GIT/DynamixelSDK.git && \
    cd DynamixelSDK && git checkout 69b30fcab2fb1200b9b7aa884350352393f79552 && cd .. && \
    git clone -b noble-jazzy-amd64 https://github.com/AndrejOrsula/pymoveit2.git && \
    cd pymoveit2 && git checkout 4bde2ac4b230c5600b31f5d7fd2cd13b2c39f6d0 && cd .. && \
    git clone -b r/4.56.4 https://github.com/IntelRealSense/realsense-ros.git && \
    cd realsense-ros && git checkout cbd8b8487d4c0126bb590822db9469414496e62a

RUN cd ${DEPS_WS} && \
    apt-get update && \
    apt-get install -y ./src/pymoveit2/ros-jazzy-pymoveit2_*.deb && \
    apt-get install -y --no-install-recommends \
    libfcl-dev \
    qt5-qmake \
    qtbase5-dev \
    ros-${ROS_DISTRO}-angles \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-controller-interface \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-dynamixel-hardware-interface \
    ros-${ROS_DISTRO}-generate-parameter-library \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-moveit-configs-utils \
    ros-${ROS_DISTRO}-moveit-core \
    ros-${ROS_DISTRO}-moveit-kinematics \
    ros-${ROS_DISTRO}-moveit-planners \
    ros-${ROS_DISTRO}-moveit-ros-move-group \
    ros-${ROS_DISTRO}-moveit-ros-planning \
    ros-${ROS_DISTRO}-moveit-ros-planning-interface \
    ros-${ROS_DISTRO}-moveit-ros-visualization \
    ros-${ROS_DISTRO}-moveit-ros-warehouse \
    ros-${ROS_DISTRO}-moveit-servo \
    ros-${ROS_DISTRO}-moveit-setup-assistant \
    ros-${ROS_DISTRO}-moveit-simple-controller-manager \
    ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-ros-gz-image \
    ros-${ROS_DISTRO}-ros-gz-sim \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-rsl \
    ros-${ROS_DISTRO}-rviz-common \
    ros-${ROS_DISTRO}-rviz-default-plugins \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tl-expected \
    ros-${ROS_DISTRO}-warehouse-ros-sqlite && \
    rosdep update && \
    rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys="librealsense2" -y && \
    rm -rf /var/lib/apt/lists/*

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd ${DEPS_WS} && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN mkdir -p ${COLCON_WS}/src && \
    git config --global --add safe.directory ${COLCON_WS}/src/open_manipulator

# Build the course workspace into the image. Compose mounts the checkout back at
# this same path, so students can edit files without setting up another workspace.
COPY . ${COLCON_WS}/src/open_manipulator

RUN bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source ${DEPS_WS}/install/setup.bash && \
    cd ${COLCON_WS} && \
    rosdep install -i --from-path src --rosdistro ${ROS_DISTRO} \
    --skip-keys='librealsense2' -y && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release"

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc  && \
    echo "source ${DEPS_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "[ -f ${COLCON_WS}/install/setup.bash ] && source ${COLCON_WS}/install/setup.bash" >> ~/.bashrc  && \
    echo "alias cb='colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'" >> ~/.bashrc && \
    echo 'export HISTFILE=/root/.open_manipulator_history/bash_history' >> ~/.bashrc && \
    echo 'export HISTSIZE=10000' >> ~/.bashrc && \
    echo 'export HISTFILESIZE=20000' >> ~/.bashrc && \
    echo 'export HISTCONTROL=ignoreboth' >> ~/.bashrc && \
    echo 'shopt -s histappend' >> ~/.bashrc && \
    echo 'export PROMPT_COMMAND="history -a; history -n"' >> ~/.bashrc

CMD ["bash"]
