FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers \
    ros-jazzy-foxglove-bridge \
    ros-jazzy-topic-tools \
    python3-colcon-common-extensions \
    python3-pytest \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /amr_ws/bags /amr_ws/maps /root/.gz/fuel

# explore_lite is NOT in Jazzy apt — built from source in the workspace
ENV ROS_DOMAIN_ID=42
WORKDIR /amr_ws
