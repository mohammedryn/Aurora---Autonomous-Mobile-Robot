#!/bin/bash
set -e
RPI="ubuntu@amr.local"

echo "--- Syncing source ---"
rsync -avz --delete ~/amr/ros2_ws/src/ $RPI:~/amr_ws/src/

echo "--- Building on RPi5 ---"
ssh $RPI "
  source /opt/ros/jazzy/setup.bash &&
  cd ~/amr_ws &&
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release \
    2>&1 | tail -30
"

echo "--- Restarting service ---"
ssh $RPI "sudo systemctl restart amr.service 2>/dev/null || echo 'Service not yet installed'"
echo "Deploy complete."
