#!/bin/bash
# One-command Aurora simulation launcher.
# Builds the Docker image if not present, then launches the sim.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

echo "======================================"
echo "  Aurora Simulation Launcher"
echo "======================================"

# Build image if needed
if ! docker image inspect amr_sim:latest >/dev/null 2>&1; then
    echo "[INFO] Building amr_sim:latest Docker image..."
    docker build -t amr_sim:latest "${PROJECT_ROOT}/docker"
fi

# Build the ROS2 workspace inside the container
echo "[INFO] Building ROS2 workspace..."
docker run --rm \
  -v "${HOME}/AMR/ros2_ws":/amr_ws \
  amr_sim:latest \
  bash -c "
    source /opt/ros/jazzy/setup.bash && \
    cd /amr_ws && \
    colcon build --symlink-install 2>&1 | tail -5
  "

echo "[INFO] Launching Aurora simulation..."
echo "[INFO] Gazebo + RViz2 will open shortly."
echo "[INFO] Aurora will start exploring automatically after ~25s."
echo ""
"${PROJECT_ROOT}/docker/run_sim.sh"
