#!/bin/bash
# AMR Demo Launcher — runs full demo stack in a single tmux session
# Usage: bash ~/AMR/scripts/demo.sh

SESSION="amr_demo"
SETUP="source /opt/ros/jazzy/setup.bash && source ~/AMR/ros2_ws/install/setup.bash"
DISP="${DISPLAY:-:1}"
RES=$(xdpyinfo -display "$DISP" 2>/dev/null | grep dimensions | awk '{print $2}' || echo "1920x1080")
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
BAG="$HOME/AMR/demo_bags/demo_$TIMESTAMP"
VID="$HOME/AMR/demo_videos/demo_$TIMESTAMP.mp4"

mkdir -p "$HOME/AMR/demo_bags" "$HOME/AMR/demo_videos"

# Kill any existing session
tmux kill-session -t $SESSION 2>/dev/null

# ── Window 0: MAIN (stack top | bag bottom-left | control bottom-right) ──
tmux new-session -d -s $SESSION -x 220 -y 55 -n "main"

# Top pane: exploration stack
tmux send-keys -t "$SESSION:0.0" \
  "echo '>>> STACK starting...' && sleep 2 && $SETUP && ros2 launch amr_bringup explore_map.launch.py" Enter

# Bottom-left pane: bag recording (waits 10s for stack to come up)
tmux split-window -v -t "$SESSION:0" -p 30
tmux send-keys -t "$SESSION:0.1" \
  "echo '>>> BAG waiting 10s for stack...' && sleep 10 && $SETUP && ros2 bag record -o $BAG /scan /odom /map /map_metadata /tf /tf_static /cmd_vel_safe /cmd_vel /explore/status /amr/status /imu/data_raw" Enter

# Bottom-right pane: control (go_home, stop — user types here)
tmux split-window -h -t "$SESSION:0.1"
tmux send-keys -t "$SESSION:0.2" \
  "$SETUP && echo '=== CONTROL PANE ===' && echo 'When map looks good, run:' && echo \"  ros2 topic pub --once /amr/command std_msgs/msg/String \\\"data: 'go_home'\\\"\"" Enter

# ── Window 1: RECORD (screen capture) ──
tmux new-window -t $SESSION -n "record"
tmux send-keys -t "$SESSION:1" \
  "echo '>>> SCREEN RECORD starts in 5s (Ctrl+C to stop)' && sleep 5 && ffmpeg -f x11grab -r 30 -s $RES -i ${DISP}.0 -c:v libx264 -preset ultrafast -crf 18 $VID && echo '>>> Saved to $VID'" Enter

# ── Window 2: RVIZ2 ──
tmux new-window -t $SESSION -n "rviz2"
tmux send-keys -t "$SESSION:2" \
  "echo '>>> RViz2 starts in 8s...' && sleep 8 && $SETUP && rviz2 -d ~/AMR/demo.rviz 2>/dev/null || ($SETUP && rviz2)" Enter

# Focus back on main window, control pane
tmux select-window -t "$SESSION:0"
tmux select-pane  -t "$SESSION:0.2"

echo ""
echo "  Aurora AMR Demo Session"
echo "  ─────────────────────────────────"
echo "  Window 0 [main]   — stack + bag + control"
echo "  Window 1 [record] — screen recording → $VID"
echo "  Window 2 [rviz2]  — RViz2 visualization"
echo ""
echo "  Switch windows: Ctrl+B then 0 / 1 / 2"
echo "  Stop recording:  go to window 1, Ctrl+C"
echo ""

tmux attach-session -t $SESSION
