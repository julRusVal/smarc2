#!/bin/bash

# $1 is robot index (0, 1, ...)
IDX="$1"
while [[ -z "$IDX" ]]; do
    read -rp "Enter floatsam index (e.g. 0, 1): " IDX
done

if ! [[ "$IDX" =~ ^[0-9]+$ ]]; then
    echo "Error: IDX must be an integer"
    exit 1
fi

ROBOT_NAME="floatsam_usv_${IDX}"
SESSION="${ROBOT_NAME}_bringup"

SIM_TRUE=true
USE_SIM_TIME="$SIM_TRUE"
NUM_ROBOTS=2

# Set ROS domain ID based on robot index
export ROS_DOMAIN_ID="$IDX"

# --- Runtime settings ---
AGENT_TYPE=subsurface
PULSE_RATE=20.0
CONTEXT=tuper
BT_LOG_MODE=compact
DOMAIN=surface

if [[ "$SIM_TRUE" == "true" ]]; then
    REALSIM=simulation
else
    REALSIM=real
fi

command -v tmux >/dev/null 2>&1 || { echo "tmux not installed"; exit 1; }

# Reset stale session if present
if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

# --- Vehicle health publisher ---
tmux -2 new-session -d -s "$SESSION" -n "vehicle_health"
tmux select-window -t "$SESSION:0"
tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}'" C-m

# --- MQTT bridge ---
tmux new-window -t "$SESSION:1" -n "mqtt_bridge"
tmux select-window -t "$SESSION:1"
tmux send-keys "ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$DOMAIN realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m

# --- Topic bridge for floatsam ---
tmux new-window -t "$SESSION:2" -n "topic_bridge"
tmux select-window -t "$SESSION:2"
tmux send-keys "ros2 launch floatsam_topic_bridge floatsam_bridge.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_of_robots:=$NUM_ROBOTS" C-m

# --- Controllers ---
tmux new-window -t "$SESSION:3" -n "controllers"
tmux select-window -t "$SESSION:3"
tmux split-window -v -t "$SESSION:3.0"
tmux select-layout -t "$SESSION:3" tiled
tmux select-pane -t "$SESSION:3.0"
tmux send-keys "ros2 launch floatsam_controllers floatsam_controllers_launch.py robot_name:=$ROBOT_NAME" C-m
tmux select-pane -t "$SESSION:3.1"
tmux send-keys "sleep 4 && ros2 launch floatsam_controllers rvo_launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_robots:=$NUM_ROBOTS" C-m

# --- move_to, move_to_path, loiter ---
tmux new-window -t "$SESSION:4" -n "move_to_actions"
tmux select-window -t "$SESSION:4"
tmux split-window -h -t "$SESSION:4.0"
tmux split-window -v -t "$SESSION:4.0"

tmux select-pane -t "$SESSION:4.0"
tmux send-keys "sleep 5 && ros2 launch floatsam_move_to floatsam_move_to.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:4.1"
tmux send-keys "sleep 5 && ros2 launch floatsam_move_to_path floatsam_move_to_path.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:4.2"
tmux send-keys "sleep 5 && ros2 run floatsam_loiter floatsam_loiter_action_server --ros-args -r __ns:=/$ROBOT_NAME -p robot_name:=$ROBOT_NAME -p loiter_move_to_speed:=fast -p use_sim:=$SIM_TRUE" C-m

# --- loiter_heading, move_to_bidirectional ---
tmux new-window -t "$SESSION:5" -n "heading_bidir"
tmux select-window -t "$SESSION:5"
tmux split-window -h -t "$SESSION:5.0"

tmux select-pane -t "$SESSION:5.0"
tmux send-keys "sleep 5 && ros2 launch floatsam_loiter_heading floatsam_loiter_heading.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:5.1"
tmux send-keys "sleep 5 && ros2 launch floatsam_move_to_bidirectional floatsam_move_to_bidirectional.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

# --- Go_to_formation_rvo ---
tmux new-window -t "$SESSION:6" -n "go_to_formation_rvo"
tmux select-window -t "$SESSION:6"
tmux send-keys "sleep 5 && ros2 launch floatsam_go_to_formation_rvo floatsam_go_to_formation_rvo.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

# --- Go_in_formation ---
tmux new-window -t "$SESSION:7" -n "go_in_formation"
tmux select-window -t "$SESSION:7"
tmux send-keys "sleep 4 && ros2 launch floatsam_go_in_formation floatsam_go_in_formation.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_robots:=$NUM_ROBOTS" C-m

# --- Behavior tree ---
tmux new-window -t "$SESSION:8" -n "bt"
tmux select-window -t "$SESSION:8"
tmux send-keys "ros2 launch wasp_bt wasp_bt.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME bt_log_mode:=$BT_LOG_MODE" C-m

# --- GPS ---
tmux new-window -t "$SESSION:9" -n "gps"
tmux select-window -t "$SESSION:9"
tmux split-window -h -t "$SESSION:9.0"
tmux select-pane -t "$SESSION:9.0"
tmux send-keys "ros2 launch septentrio_gnss_driver rover.launch.py ns:=$ROBOT_NAME" C-m
tmux select-pane -t "$SESSION:9.1"
tmux send-keys "str2str -in ntrip://cinnmon@gmail.com:none@rtk2go.com:2101/Tranholmen -out serial://ttyACM1:115200" C-m

# --- Logging ---
tmux new-window -t "$SESSION:10" -n "logging"
tmux select-window -t "$SESSION:10"

# Set default window and attach/switch
tmux select-window -t "$SESSION:7"
if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux -2 attach-session -t "$SESSION"
fi
