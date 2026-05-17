#!/bin/bash
set -e
docker run -it --rm \
  --env DISPLAY=$DISPLAY \
  --env ROS_DOMAIN_ID=42 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$HOME/amr/ros2_ws":/amr_ws \
  --network host \
  amr_sim:latest \
  ros2 launch amr_bringup sim.launch.py
