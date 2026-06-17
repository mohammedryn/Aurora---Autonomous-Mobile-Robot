#!/bin/bash
set -e

xhost +local:docker 2>/dev/null || true

docker run -it --rm \
  --env DISPLAY="${DISPLAY}" \
  --env ROS_DOMAIN_ID=42 \
  --env GZ_SIM_RESOURCE_PATH="/root/.gz/fuel/models" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --device /dev/dri \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  -v "${HOME}/.gz/fuel":/root/.gz/fuel \
  --network host \
  amr_sim:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    source /amr_ws/install/setup.bash && \
    ros2 launch amr_bringup sim.launch.py
  "
