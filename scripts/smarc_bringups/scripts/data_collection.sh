#!/bin/bash

SESSION="ros2_session"

# Kill existing session if it exists
tmux kill-session -t $SESSION 2>/dev/null

# Ask about bag recording
read -p "Do you want to record a ROS2 bag (all topics)? [y/N]: " RECORD_BAG

# Create a new tmux session (detached), first window for sonar
tmux new-session -d -s $SESSION -n "sonar"

# Window 0: Waterlinked Sonar
tmux send-keys -t $SESSION:0 \
  "echo '=== Waterlinked Sonar 3D15 ===' && ros2 launch waterlinked_sonar_3d15 sonar_3d15.launch.py" \
  Enter

# Window 1: Septentrio GNSS Driver
tmux new-window -t $SESSION -n "gnss"
tmux send-keys -t $SESSION:1 \
  "echo '=== SBG GNSS Driver ===' && ros2 launch asv_ros2_lib all_driver_launch.xml" \
  Enter

# Window 3 (optional): ROS2 Bag recording
if [[ "$RECORD_BAG" =~ ^[Yy]$ ]]; then
  BAG_DIR="$HOME/bags"
  BAG_NAME="$BAG_DIR/ros2_bag_$(date +%Y%m%d_%H%M%S)"
  mkdir -p "$BAG_DIR"
  tmux new-window -t $SESSION -n "bag"
  tmux send-keys -t $SESSION:3 \
    "echo '=== Recording bag: $BAG_NAME ===' && ros2 bag record -a -o $BAG_NAME" \
    Enter
  echo "Bag recording started: $BAG_NAME"
else
  echo "Skipping bag recording."
fi

# Attach to the session
echo ""
echo "Attaching to tmux session '$SESSION'..."
echo "  Switch windows : Ctrl+b then 0/1/2/3"
echo "  Detach         : Ctrl+b then d"
echo "  Kill session   : tmux kill-session -t $SESSION"
echo ""

tmux attach-session -t $SESSION
