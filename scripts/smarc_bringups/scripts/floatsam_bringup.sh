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

SIM_TRUE=false
USE_SIM_TIME="$SIM_TRUE"
NUM_ROBOTS=2

export ROS_DOMAIN_ID="$IDX"

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

if tmux has-session -t "$SESSION" 2>/dev/null; then
    tmux kill-session -t "$SESSION"
fi

tmux -2 new-session -d -s "$SESSION" -n "vehicle_health"
tmux select-window -t "$SESSION:0"
tmux send-keys "ros2 topic pub -r 1 /$ROBOT_NAME/smarc/vehicle_health std_msgs/msg/Int8 '{data: 0}'" C-m

tmux new-window -t "$SESSION:1" -n "mqtt_bridge"
tmux select-window -t "$SESSION:1"
tmux send-keys "ros2 launch str_json_mqtt_bridge waraps_bridge.launch broker_addr:=20.240.40.232 broker_port:=1884 robot_name:=$ROBOT_NAME domain:=$DOMAIN realsim:=$REALSIM use_sim_time:=$USE_SIM_TIME context:=$CONTEXT" C-m

tmux new-window -t "$SESSION:2" -n "topic_bridge"
tmux select-window -t "$SESSION:2"
tmux send-keys "ros2 launch floatsam_topic_bridge floatsam_bridge.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_of_robots:=$NUM_ROBOTS" C-m

tmux new-window -t "$SESSION:3" -n "localization"
tmux select-window -t "$SESSION:3"
tmux send-keys "sleep 8 && ros2 launch usv_localization_bringup localization.launch.py robot_name:=$ROBOT_NAME" C-m

tmux new-window -t "$SESSION:4" -n "controllers"
tmux select-window -t "$SESSION:4"
tmux split-window -v -t "$SESSION:4.0"
tmux select-layout -t "$SESSION:4" tiled
tmux select-pane -t "$SESSION:4.0"
tmux send-keys "ros2 launch floatsam_controllers floatsam_controllers_launch.py robot_name:=$ROBOT_NAME" C-m
tmux select-pane -t "$SESSION:4.1"
tmux send-keys "sleep 4 && ros2 launch floatsam_controllers rvo_launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_robots:=$NUM_ROBOTS" C-m

tmux new-window -t "$SESSION:5" -n "move_to_actions"
tmux select-window -t "$SESSION:5"
tmux split-window -h -t "$SESSION:5.0"
tmux split-window -v -t "$SESSION:5.0"
tmux select-pane -t "$SESSION:5.0"
tmux send-keys "sleep 5 && ros2 launch floatsam_move_to floatsam_move_to.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:5.1"
tmux send-keys "sleep 5 && ros2 launch floatsam_move_to_path floatsam_move_to_path.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

tmux new-window -t "$SESSION:6" -n "heading_bidir"
tmux select-window -t "$SESSION:6"
tmux split-window -h -t "$SESSION:6.0"
tmux select-pane -t "$SESSION:6.0"
tmux send-keys "sleep 5 && ros2 launch floatsam_loiter_heading floatsam_loiter_heading.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:6.1"
tmux send-keys "sleep 5 && ros2 launch floatsam_move_to_bidirectional floatsam_move_to_bidirectional.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

tmux new-window -t "$SESSION:7" -n "go_to_formation_rvo"
tmux select-window -t "$SESSION:7"
tmux send-keys "sleep 5 && ros2 launch floatsam_go_to_formation_rvo floatsam_go_to_formation_rvo.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

tmux new-window -t "$SESSION:8" -n "go_in_formation"
tmux select-window -t "$SESSION:8"
tmux send-keys "sleep 4 && ros2 launch floatsam_go_in_formation floatsam_go_in_formation.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_robots:=$NUM_ROBOTS" C-m

tmux new-window -t "$SESSION:9" -n "bt"
tmux select-window -t "$SESSION:9"
tmux send-keys "ros2 launch wasp_bt wasp_bt.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME bt_log_mode:=$BT_LOG_MODE" C-m

# RTK: Septentrio driver + NTRIP aux client (usv_rtk_bringup)
tmux new-window -t "$SESSION:10" -n "gps"
tmux select-window -t "$SESSION:10"
tmux split-window -h -t "$SESSION:10.0"
tmux select-pane -t "$SESSION:10.0"
tmux send-keys "sleep 5 && ros2 launch usv_rtk_bringup rtk.launch.py" C-m
tmux select-pane -t "$SESSION:10.1"
tmux send-keys "sleep 30 && ros2 run usv_rtk_bringup ntrip_aux_client --release-port" C-m

tmux new-window -t "$SESSION:11" -n "logging"
tmux select-window -t "$SESSION:11"

tmux select-window -t "$SESSION:10"
if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux -2 attach-session -t "$SESSION"
fi
