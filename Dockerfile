FROM ros:jazzy-ros-core
ENV ROS_DISTRO=jazzy
ENV WS=/home/ros2-workspace


RUN apt-get update -y && apt-get install -y \
    ros-${ROS_DISTRO}-mavros \
    ros-${ROS_DISTRO}-mavros-extras \
    ros-${ROS_DISTRO}-mavros-msgs \
    python3-pip \
    python3-rosdep \
    python3-colcon-common-extensions \
    build-essential \
    ros-jazzy-ros2bag \
    ros-jazzy-rosbag2-storage-default-plugins \
    libegl1 \
    libgl1 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xfixes0 \
    libxcb-xinerama0 \
    libxkbcommon-x11-0 \
    libxcb-cursor0 \
    libxcb-shape0 \
    libxcb-xinput0 \
    qt6-base-dev \
    make

RUN wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
RUN bash ./install_geographiclib_datasets.sh

RUN pip install pymavlink==2.4.49 pyside6==6.10.1 --break-system-packages

RUN apt-get clean

WORKDIR ${WS}
COPY ./src ${WS}/src

WORKDIR ${WS}
RUN rosdep init && rosdep update
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash \
    && rosdep install --from-paths src --ignore-src -r -y \
    && colcon build --symlink-install"

RUN /bin/bash -c "echo \"source /opt/ros/${ROS_DISTRO}/setup.bash\" >> ~/.bashrc"
RUN /bin/bash -c "echo \"source ${WS}/install/setup.bash\" >> ~/.bashrc"

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
