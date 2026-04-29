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

# --- Domain isolation + Public Square bridge config ---
PUBLIC_DOMAIN=111
NUM_ROBOTS=2

if (( IDX >= NUM_ROBOTS )); then
    echo "Error: IDX ${IDX} is out of range for NUM_ROBOTS=${NUM_ROBOTS}"
    exit 1
fi

if (( IDX == PUBLIC_DOMAIN )); then
    echo "Error: IDX must not be equal to PUBLIC_DOMAIN (${PUBLIC_DOMAIN})"
    exit 1
fi

export ROS_DOMAIN_ID="$IDX"

BRIDGE_YAML="/tmp/${ROBOT_NAME}_bridge.yaml"

ACTIONS=(
    move_to
    loiter
    move_path
    loiter_heading
    go_to_formation
    go_to_formation_rvo
)

echo "[domain_bridge] generating ${BRIDGE_YAML} (robot_domain=${IDX}, public_domain=${PUBLIC_DOMAIN})"

cat > "${BRIDGE_YAML}" <<EOF2
name: ${ROBOT_NAME}_public_square_bridge
from_domain: ${IDX}
to_domain: ${PUBLIC_DOMAIN}
topics:
  /${ROBOT_NAME}/smarc/odom:
    type: nav_msgs/msg/Odometry
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}

  /${ROBOT_NAME}/loiter_heading_fb:
    type: smarc_msgs/msg/FloatStamped
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}

  /${ROBOT_NAME}/waraps/action_server_heartbeat:
    type: std_msgs/msg/String
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}
EOF2

# Action topics (status + feedback) robot -> public
for ACTION_NAME in "${ACTIONS[@]}"; do
    cat >> "${BRIDGE_YAML}" <<EOF2
  /${ROBOT_NAME}/${ACTION_NAME}/_action/status:
    type: action_msgs/msg/GoalStatusArray
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}

  /${ROBOT_NAME}/${ACTION_NAME}/_action/feedback:
    type: smarc_msgs/action/BaseAction_FeedbackMessage
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}
EOF2
done

# Bridge TF both ways so each robot can resolve peer frames for RVO.
cat >> "${BRIDGE_YAML}" <<EOF2
  /tf:
    type: tf2_msgs/msg/TFMessage
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}
    bidirectional: true

  /tf_static:
    type: tf2_msgs/msg/TFMessage
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}
    bidirectional: true
    qos:
      durability: transient_local
      reliability: reliable
      history: keep_last
      depth: 1
EOF2

# Pull peer telemetry from public domain
for PEER_IDX in $(seq 0 $((NUM_ROBOTS - 1))); do
    if [[ "${PEER_IDX}" -eq "${IDX}" ]]; then
        continue
    fi

    PEER_NAME="floatsam_usv_${PEER_IDX}"

    cat >> "${BRIDGE_YAML}" <<EOF2
  /${PEER_NAME}/smarc/odom:
    type: nav_msgs/msg/Odometry
    from_domain: ${PUBLIC_DOMAIN}
    to_domain: ${IDX}

  /${PEER_NAME}/loiter_heading_fb:
    type: smarc_msgs/msg/FloatStamped
    from_domain: ${PUBLIC_DOMAIN}
    to_domain: ${IDX}
EOF2
done

cat >> "${BRIDGE_YAML}" <<EOF2
services:
EOF2

# Action services exposed to public domain (base station sends goals/cancel/result requests)
for ACTION_NAME in "${ACTIONS[@]}"; do
    cat >> "${BRIDGE_YAML}" <<EOF2
  /${ROBOT_NAME}/${ACTION_NAME}/_action/send_goal:
    type: smarc_msgs/action/BaseAction_SendGoal
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}

  /${ROBOT_NAME}/${ACTION_NAME}/_action/cancel_goal:
    type: action_msgs/srv/CancelGoal
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}

  /${ROBOT_NAME}/${ACTION_NAME}/_action/get_result:
    type: smarc_msgs/action/BaseAction_GetResult
    from_domain: ${IDX}
    to_domain: ${PUBLIC_DOMAIN}
EOF2
done

echo "[domain_bridge] config ready: ${BRIDGE_YAML}"

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
tmux send-keys "ros2 launch floatsam_topic_bridge floatsam_bridge.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

# --- Controllers ---
tmux new-window -t "$SESSION:3" -n "controllers"
tmux select-window -t "$SESSION:3"
tmux split-window -v -t "$SESSION:3.0"
tmux select-layout -t "$SESSION:3" tiled
tmux select-pane -t "$SESSION:3.0"
tmux send-keys "ros2 launch floatsam_controllers floatsam_controllers_launch.py robot_name:=$ROBOT_NAME" C-m
tmux select-pane -t "$SESSION:3.1"
tmux send-keys "ros2 launch floatsam_controllers rvo_launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE num_robots:=$NUM_ROBOTS" C-m

# --- Servers / action servers ---
tmux new-window -t "$SESSION:4" -n "servers"
tmux select-window -t "$SESSION:4"
tmux split-window -h -t "$SESSION:4.0"
tmux split-window -v -t "$SESSION:4.1"
tmux split-window -v -t "$SESSION:4.0"

tmux select-pane -t "$SESSION:4.0"
tmux send-keys "ros2 launch floatsam_move_to floatsam_move_to.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:4.1"
tmux send-keys "ros2 run floatsam_loiter floatsam_loiter_action_server --ros-args -r __ns:=/$ROBOT_NAME -p robot_name:=$ROBOT_NAME -p loiter_move_to_speed:=fast -p use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:4.2"
tmux send-keys "ros2 launch floatsam_move_to_path floatsam_move_to_path.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m
tmux select-pane -t "$SESSION:4.3"
tmux send-keys "ros2 launch floatsam_loiter_heading floatsam_loiter_heading.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

# --- Go_to_formation ---
tmux new-window -t "$SESSION:5" -n "go_to_formation"
tmux select-window -t "$SESSION:5"
tmux send-keys "ros2 launch floatsam_go_to_formation floatsam_go_to_formation.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

# --- Go_to_formation_rvo ---
tmux new-window -t "$SESSION:6" -n "go_to_formation_rvo"
tmux select-window -t "$SESSION:6"
tmux send-keys "ros2 launch floatsam_go_to_formation_rvo floatsam_go_to_formation_rvo.launch.py robot_name:=$ROBOT_NAME use_sim:=$SIM_TRUE" C-m

# --- Behavior tree ---
tmux new-window -t "$SESSION:7" -n "bt"
tmux select-window -t "$SESSION:7"
tmux send-keys "ros2 launch wasp_bt wasp_bt.launch robot_name:=$ROBOT_NAME agent_type:=$AGENT_TYPE pulse_rate:=$PULSE_RATE use_sim_time:=$USE_SIM_TIME bt_log_mode:=$BT_LOG_MODE" C-m

# --- GPS ---
tmux new-window -t "$SESSION:8" -n "gps"
tmux select-window -t "$SESSION:8"
tmux split-window -h -t "$SESSION:8.0"
tmux select-pane -t "$SESSION:8.0"
tmux send-keys "ros2 launch septentrio_gnss_driver rover.launch.py ns:=$ROBOT_NAME" C-m
tmux select-pane -t "$SESSION:8.1"
tmux send-keys "str2str -in ntrip://cinnamon@gmail.com:none@rtk2go.com:2101/Tranholmen -out serial://gps_rtk_1:115200" C-m

# --- Domain bridge ---
tmux new-window -t "$SESSION:9" -n "domain_bridge"
tmux select-window -t "$SESSION:9"
tmux send-keys "ros2 run domain_bridge domain_bridge $BRIDGE_YAML" C-m

# --- Logging ---
tmux new-window -t "$SESSION:10" -n "logging"
tmux select-window -t "$SESSION:10"

# Set default window and attach/switch
tmux select-window -t "$SESSION:6"
if [[ -n "${TMUX:-}" ]]; then
    tmux switch-client -t "$SESSION"
else
    tmux -2 attach-session -t "$SESSION"
fi
