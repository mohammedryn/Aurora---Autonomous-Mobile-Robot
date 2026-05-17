#!/bin/bash
# Run once on RPi5 Ubuntu 24.04 as ubuntu user
set -e

echo "--- Installing ROS2 Jazzy ---"
sudo apt-get update && sudo apt-get install -y software-properties-common curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list

sudo apt-get update && sudo apt-get install -y \
    ros-jazzy-ros-base \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-robot-localization \
    ros-jazzy-imu-tools \
    ros-jazzy-ros2-controllers \
    ros-jazzy-ros2-control \
    ros-jazzy-foxglove-bridge \
    python3-colcon-common-extensions \
    python3-rosdep

echo "--- Workspace setup ---"
mkdir -p ~/amr_ws/src
cd ~/amr_ws
rosdep init 2>/dev/null || true
rosdep update

echo "--- udev rules ---"
sudo cp ~/amr/scripts/udev/99-amr.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

echo "--- ROS2 sourcing ---"
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/amr_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

echo "Setup complete. Reboot recommended."
