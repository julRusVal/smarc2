import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, ParameterDescriptor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from smarc_action_base.gentler_action_server import GentlerActionServer

from floatsam_controllers.floatsam_common import FloatSam
from floatsam_go_in_formation.PathParameterizer import PathParameterizer
from floatsam_go_in_formation.PathSmoothing import PathSmoother

import time
import traceback
import json
import paho.mqtt.client as mqtt

from std_msgs.msg import Bool, Float32
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from smarc_msgs.action import BaseAction
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics


from scipy.optimize import linear_sum_assignment
import numpy as np
import math 


class FloatsamGoInFormationAction():
    def __init__(self, node:Node):
        self._node : Node = node
        

        self.odom_in_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # The carrot advance step (self._ds) MUST be derived from this same value,
        # since _loop_inner is what actually advances the shared progress.
        self._loop_frequency = 10

        self._as = GentlerActionServer(
            node, 
            'go_in_formation',
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=self._loop_frequency
        )

        self.declare_node_parameters()
        self.get_node_parameters()
        self._node.get_logger().info(f'self._yawrate_p_gain = {self._yawrate_p_gain}')
        self._node.get_logger().info(f'self._yawrate_p_gain = {self._yawrate_d_gain}')
        self._floatsam = FloatSam(self._node, self._this_robot_name, use_sim=self._use_sim)


        self._robot_positions = {}
        self._odom_subscribers = {}

        self.create_subscriptions()
        self.create_node_publishers()

        # MQTT bridge for real-hardware inter-robot coordination
        self._mqtt_client = None
        self._mqtt_connected = False
        self._mqtt_broker_ip = '20.240.40.232'
        self._mqtt_broker_port = 1884
        self._mqtt_client_id = f'go_in_formation_{self._this_robot_name}'
        if not self._use_sim:
            self._setup_mqtt_client()

        self._param_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_clients()

        self.MAP_FRAME : str = self._this_robot_name + '/map'
        self._floatsam = FloatSam(node, self._this_robot_name, use_sim=self._use_sim)

    def declare_node_parameters(self) -> None:
        int_desc = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER)
        double_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        string_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        bool_desc = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)

        self._node.declare_parameter("use_sim", True, bool_desc)
        self._node.declare_parameter("robot_name", 'floatsam_usv', string_desc)
        self._node.declare_parameter("num_robots", 2, int_desc)
        self._node.declare_parameter('update_rate', 20.0, double_desc)
        self._node.declare_parameter('carrot_speed', 1.0, double_desc)
        self._node.declare_parameter('move_to_trigger', 2.0, double_desc)
        self._node.declare_parameter('catch_up_gain', 1.0, double_desc)
        self._node.declare_parameter('catch_up_distance', 1.0, double_desc)
        self._node.declare_parameter('look_a_head_distance', 3.0, double_desc)


        self._node.declare_parameter("yaw_p_gain", 0.6, double_desc)
        self._node.declare_parameter("yaw_i_gain", 0.0, double_desc)
        self._node.declare_parameter("yaw_d_gain", 0.2, double_desc)
        self._node.declare_parameter("yaw_threshold", 0.5, double_desc)

        self._node.declare_parameter("yawrate_p_gain", 300.0, double_desc)
        self._node.declare_parameter("yawrate_i_gain", 0.0, double_desc)
        self._node.declare_parameter("yawrate_d_gain", 30.0, double_desc)

        self._node.declare_parameter("velocity_p_gain", 500.0, double_desc)
        self._node.declare_parameter("velocity_i_gain", 10.0, double_desc)
        self._node.declare_parameter("velocity_d_gain", 0.0, double_desc)

    def get_node_parameters(self) -> None:
        self._use_sim = self._node.get_parameter('use_sim').get_parameter_value().bool_value
        self._this_robot_name = self._node.get_parameter('robot_name').get_parameter_value().string_value
        self._num_robots = self._node.get_parameter('num_robots').get_parameter_value().integer_value
        self._update_rate = self._node.get_parameter('update_rate').get_parameter_value().double_value
        self._carrot_speed = self._node.get_parameter('carrot_speed').get_parameter_value().double_value
        self._move_to_trigger = self._node.get_parameter('move_to_trigger').get_parameter_value().double_value
        self._catch_up_gain = self._node.get_parameter('catch_up_gain').get_parameter_value().double_value
        self._catch_up_distance = self._node.get_parameter('catch_up_distance').get_parameter_value().double_value
        self._look_a_head_distance = self._node.get_parameter('look_a_head_distance').get_parameter_value().double_value

        

        self._robot_ids        = range(self._num_robots)
        self._robot_base_name  = '_'.join(self._this_robot_name.split('_')[:-1])
        self._others_arrived_flag = False
        # ds is the shared progress increment per control tick, in MASTER arc length
        # (metres). It must use the actual loop rate, not update_rate, or the carrot
        # moves at the wrong speed (e.g. update_rate=20 with a 10 Hz loop = half speed).
        self._ds = self._carrot_speed / self._loop_frequency
        self.distance_error = 0.0

        self._yaw_p_gain = self._node.get_parameter('yaw_p_gain').get_parameter_value().double_value
        self._yaw_i_gain = self._node.get_parameter('yaw_i_gain').get_parameter_value().double_value
        self._yaw_d_gain = self._node.get_parameter('yaw_d_gain').get_parameter_value().double_value
        self._yaw_threshold = self._node.get_parameter('yaw_threshold').get_parameter_value().double_value

        self._yawrate_p_gain = self._node.get_parameter('yawrate_p_gain').get_parameter_value().double_value
        self._yawrate_i_gain = self._node.get_parameter('yawrate_i_gain').get_parameter_value().double_value
        self._yawrate_d_gain = self._node.get_parameter('yawrate_d_gain').get_parameter_value().double_value

        self._velocity_p_gain = self._node.get_parameter('velocity_p_gain').get_parameter_value().double_value
        self._velocity_i_gain = self._node.get_parameter('velocity_i_gain').get_parameter_value().double_value
        self._velocity_d_gain = self._node.get_parameter('velocity_d_gain').get_parameter_value().double_value

    # ------------------------------------------------------------------
    # MQTT bridge (real hardware only)
    # ------------------------------------------------------------------

    def _setup_mqtt_client(self) -> None:
        try:
            self._mqtt_client = mqtt.Client(client_id=self._mqtt_client_id)
            self._mqtt_client.on_connect = self._mqtt_on_connect
            self._mqtt_client.on_disconnect = self._mqtt_on_disconnect
            self._mqtt_client.on_message = self._mqtt_on_message
            self._mqtt_client.connect(self._mqtt_broker_ip, self._mqtt_broker_port, keepalive=60)
            self._mqtt_client.loop_start()
            self._node.get_logger().info(
                f'MQTT: Connecting to {self._mqtt_broker_ip}:{self._mqtt_broker_port}...'
            )
        except Exception as e:
            self._node.get_logger().error(f'MQTT setup failed: {e}')
            self._mqtt_client = None

    def _mqtt_on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self._node.get_logger().info('MQTT: Connected.')
            self._mqtt_subscribe_to_peers()
        else:
            self._node.get_logger().error(f'MQTT: Connection failed with code {rc}')

    def _mqtt_on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        if rc != 0:
            self._node.get_logger().warn(f'MQTT: Unexpected disconnection (code {rc})')

    def _mqtt_on_message(self, client, userdata, msg):
        try:
            topic_parts = msg.topic.split('/')
            if len(topic_parts) < 2:
                return
            robot_name = topic_parts[0]
            topic_suffix = '/'.join(topic_parts[1:])
            payload = json.loads(msg.payload.decode('utf-8'))

            if topic_suffix == 'formation_error':
                float_msg = FloatStamped()
                float_msg.data = float(payload['data'])
                self._peer_error_cb(float_msg, robot_name)

            elif topic_suffix == 'mission_ready':
                bool_msg = Bool()
                bool_msg.data = bool(payload['data'])
                self._peer_ready_cb(bool_msg, robot_name)

        except Exception as e:
            self._node.get_logger().error(
                f'MQTT: Failed to parse message from {msg.topic}: {e}',
                throttle_duration_sec=5.0
            )

    def _mqtt_subscribe_to_peers(self) -> None:
        for robot_id in self._robot_ids:
            peer_name = f'{self._robot_base_name}_{robot_id}'
            if peer_name == self._this_robot_name:
                continue
            for suffix in ('formation_error', 'mission_ready'):
                topic = f'{peer_name}/{suffix}'
                self._mqtt_client.subscribe(topic)
                self._node.get_logger().info(f'MQTT: Subscribed to {topic}')

    def _mqtt_publish_formation_error(self, error_value: float, stamp) -> None:
        if self._mqtt_client is None or not self._mqtt_connected:
            return
        try:
            payload = json.dumps({
                'data': error_value,
                'stamp': {'sec': stamp.sec, 'nsec': stamp.nanosec}
            })
            self._mqtt_client.publish(
                f'{self._this_robot_name}/formation_error', payload, qos=0
            )
        except Exception as e:
            self._node.get_logger().error(
                f'MQTT: Failed to publish formation_error: {e}', throttle_duration_sec=5.0
            )

    def _mqtt_publish_mission_ready(self, ready: bool) -> None:
        if self._mqtt_client is None or not self._mqtt_connected:
            return
        try:
            payload = json.dumps({'data': ready})
            self._mqtt_client.publish(
                f'{self._this_robot_name}/mission_ready', payload, qos=0
            )
        except Exception as e:
            self._node.get_logger().error(
                f'MQTT: Failed to publish mission_ready: {e}', throttle_duration_sec=5.0
            )

    # ------------------------------------------------------------------

    def create_clients(self) -> None:
        captain_node_name = f'/{self._this_robot_name}/captain'
        self._get_params_client = self._node.create_client(
            GetParameters, f'{captain_node_name}/get_parameters', callback_group=self._param_cb_group)
            
        self._set_params_client = self._node.create_client(
            SetParameters, f'{captain_node_name}/set_parameters', callback_group=self._param_cb_group)
            
        self._saved_background_parameters = None

    def _read_captain_parameters(self) -> list:
        """Reads the current PID parameters from the Captain node."""
        if not self._get_params_client.service_is_ready():
            self._node.get_logger().warning("GetParameters service not ready!")
            return None

        req = GetParameters.Request()
        req.names = [
            'yaw_p_gain', 'yaw_i_gain', 'yaw_d_gain', 'yaw_threshold',
            'yawrate_p_gain', 'yawrate_i_gain', 'yawrate_d_gain',
            'velocity_p_gain', 'velocity_i_gain', 'velocity_d_gain'
        ]
        
        future = self._get_params_client.call_async(req)

        while not future.done():
            time.sleep(0.01)
            
        response = future.result()
        
        saved_params = []
        for name, value in zip(req.names, response.values):
            p = Parameter()
            p.name = name
            p.value = value
            saved_params.append(p)
            
        return saved_params

    def _write_captain_parameters(self, param_list: list):
        """Sends a list of Parameters to update the Captain node."""
        if not self._set_params_client.service_is_ready():
            self._node.get_logger().warning("SetParameters service not ready!")
            return

        req = SetParameters.Request()
        req.parameters = param_list
        
        future = self._set_params_client.call_async(req)
        while not future.done():
            time.sleep(0.01)
            
        if future.result().results[0].successful:
            self._node.get_logger().info("Successfully pushed parameters to Captain.")

    def _odometry_subscriptions(self):
            for robot_id in self._robot_ids:
                odom_topic = f'/{self._robot_base_name}_{robot_id}/smarc/odom_in_map'
                subscriber = self._node.create_subscription(
                    Odometry,
                    odom_topic,
                    lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                    self.odom_in_map_qos
                )
                self._odom_subscribers[robot_id] = subscriber
                self._node.get_logger().info(f'Subscribed to {odom_topic}')

    def _odom_callback(self, msg: Odometry, robot_id: int):
        """Update position and velocity for a robot from its odometry topic."""
        robot_name = f'{self._robot_base_name}_{robot_id}'

        pose_in_global = PoseStamped()
        pose_in_global.header = msg.header
        pose_in_global.pose   = msg.pose.pose

        self._robot_positions[robot_name]  = pose_in_global

    def _peer_error_cb(self, msg: FloatStamped, robot_name: str):
        self._peer_errors[robot_name] = msg.data

    def _peers_error_subscriptions(self) -> None:
        self._peer_errors = {}
        for robot_id in self._robot_ids:
            robot_name = f'{self._robot_base_name}_{robot_id}'
            
            if robot_name != self._this_robot_name:
                self._peer_errors[robot_name] = None 
                
                topic_name = f'/{robot_name}/formation_error'
                
                self._node.create_subscription(
                    FloatStamped, 
                    topic_name, 
                    lambda msg, rn=robot_name: self._peer_error_cb(msg, rn), 
                    self._best_effort_qos 
                )

    def _peers_ready_subscriptions(self) -> None:
        self._peer_ready = {}
        for robot_id in self._robot_ids:
            robot_name = f'{self._robot_base_name}_{robot_id}'
            if robot_name != self._this_robot_name:
                self._peer_ready[robot_name] = False
                topic_name = f'/{robot_name}/mission_ready'
                self._node.create_subscription(
                    Bool,
                    topic_name,
                    lambda msg, rn=robot_name: self._peer_ready_cb(msg, rn),
                    self._best_effort_qos
                )

    def _peer_ready_cb(self, msg: Bool, robot_name: str) -> None:
        self._peer_ready[robot_name] = msg.data

    def create_node_publishers(self) -> None:
        self._move_on_place_publisher = self._node.create_publisher(Bool, 'move_on_place', 1)

        self._yaw_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.YAW_SETPOINT, self._best_effort_qos)
        
        self._speed_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, self._best_effort_qos)
        
        self._error_publisher = self._node.create_publisher(FloatStamped, 'formation_error', self._best_effort_qos)
        self.thruster_port_pub = self._node.create_publisher(Float32,FloatsamTopics.THRUSTER_PORT_CMD, 1)
        self.thruster_strb_pub = self._node.create_publisher(Float32, FloatsamTopics.THRUSTER_STRB_CMD, 1)
        
        self._ready_publisher = self._node.create_publisher(Bool, 'mission_ready', self._best_effort_qos)

    def _heading_callback(self, msg:Float32) ->None:
        self._heading = msg.data

    def create_subscriptions(self) -> None:
        self._odometry_subscriptions()
        self._peers_error_subscriptions()
        self._peers_ready_subscriptions()
        heading_topic = f'/{self._this_robot_name}/smarc/heading'
        self._heading_subscriber = self._node.create_subscription(
            Float32, 
            heading_topic, 
            self._heading_callback, 
            10
        )


#     Goal Structure :
#    {
#        'desired_speed' : 2.0,
#        'track' : 
#        
#            
#                {'latitude': 59.00, 'longitude': 18.00} # Inner list element 0
#                {'latitude': 59.01, 'longitude': 18.01} # Inner list element 1
#            
#   
#    }
#           


# Functions for GentlerActionServer

    def _on_goal_received(self, goal_request: dict) -> bool:
        self._node.get_logger().info(f'Goal received: {goal_request}')
        
        try:
            self._desired_speed = float(goal_request.get('desired_speed', 2.0))

            raw_track = goal_request.get('track', None)
            
            if not isinstance(raw_track, list) or len(raw_track) < 3:
                self._node.get_logger().error(f'The track mist be a list of least 3 waypoints')
                return False 
            
            self._track_in_map = []

            for wp_idx, wp in enumerate(raw_track):
                gp = GeoPoint()
                gp.latitude = float(wp['latitude'])
                gp.longitude = float(wp['longitude'])
                gp.altitude = 0.0
                try:
                    map_pose = self._floatsam.convert_geopoint_to_map_pose_stamped(gp)
                    self._track_in_map.append(map_pose)
                except Exception as tf_error:
                    self._node.get_logger().error(f"TF Error on WP {wp_idx}: {tf_error}")
                    return False
            
            self._node.get_logger().info(f"Successfully converted {len(self._track_in_map)} tracks into map frame.")

            self._saved_background_parameters = self._read_captain_parameters()

            move_to_params = []
            param_dict = {
                "yaw_p_gain": self._yaw_p_gain,
                "yaw_i_gain": self._yaw_i_gain,
                "yaw_d_gain": self._yaw_d_gain,
                "yaw_threshold": self._yaw_threshold,
                "yawrate_p_gain": self._yawrate_p_gain,
                "yawrate_i_gain": self._yawrate_i_gain,
                "yawrate_d_gain": self._yawrate_d_gain,
                "velocity_p_gain": self._velocity_p_gain, 
                "velocity_i_gain": self._velocity_i_gain, 
                "velocity_d_gain": self._velocity_d_gain
            }

            for name, val in param_dict.items():
                p = Parameter()
                p.name = name
                p.value.type = ParameterType.PARAMETER_DOUBLE
                p.value.double_value = float(val)
                move_to_params.append(p)

            self._write_captain_parameters(move_to_params)

            return True 
        except Exception as e:
            self._node.get_logger().error(f"Failed to parse goal: {e}")
            return False
  
    def _CheckAllRobotsHavePosition(self, time_out : float = 5) -> bool:
        required_robot_count = len(self._tracks_in_map)
        start_time = time.time()

        while (time.time() - start_time) < time_out:
            if len(self._robot_positions) == required_robot_count:
                break

            self._node.get_logger().info(
            f'Waiting for odometry... (Got {len(self._robot_positions)}/{required_robot_count})', 
            throttle_duration_sec=2.0
            )
            time.sleep(0.5)

        if len(self._robot_positions) != required_robot_count:
            self._node.get_logger().error(
            f'Error while waiting for odometry... (Got {len(self._robot_positions)}/{required_robot_count}) after the time out.', 
            throttle_duration_sec=2.0
            )
            return False 
        
        return True 
    
    def compute_distance(self, robot_position:PoseStamped, track_position:PoseStamped) -> float:
        rx = robot_position.pose.position.x
        ry = robot_position.pose.position.y
        tx = track_position.pose.position.x
        ty = track_position.pose.position.y
        return (rx - tx)**2 + (ry - ty)**2
    
    def compute_cost_matrix(self, tracks_point:dict[str, PoseStamped], robot_positions:dict[str, PoseStamped], robot_names:list[str]) -> np.ndarray:
        size = len(robot_names)
        cost_matrix = np.zeros((size, size))
        for i, name in enumerate(robot_names):
            for j in range(size):
                cost_matrix[i][j] = self.compute_distance(robot_position=robot_positions[name], track_position=tracks_point[f'track_{j}'])
        return cost_matrix

    def _HungarianAssignment(self) -> bool:
        if not self._CheckAllRobotsHavePosition():
            self._node.get_logger().error(f'Not all the robot positions are available.')
            return False
        
        try:
            self._node.get_logger().info(f'Starting Hungarian Assignment...')
            self._assigned_tracks = {}
            robot_names = sorted(self._robot_positions.keys())

            tracks_point = {}
            for idx, track in enumerate(self._tracks_in_map):
                tracks_point[f'track_{idx}'] = track[0]

            cost_matrix = self.compute_cost_matrix(tracks_point, self._robot_positions, robot_names)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)
            for i in range(len(row_ind)):
                    robot_key = robot_names[row_ind[i]]
                    task_idx = col_ind[i]
                    self._assigned_tracks[robot_key] = task_idx
                    self._node.get_logger().info(f'{robot_key} assigned to goal_{task_idx}')
            
            self._this_robot_track_idx = self._assigned_tracks[self._this_robot_name]
            self._this_robot_waypoints = self._tracks_in_map[self._this_robot_track_idx]
            return True 
        
        except Exception as e:
            self._node.get_logger().error(f"Exception: {e}")
            return False

    def _prepare_loop(self) -> None: 
        self._node.get_logger().info('Preparing loop.')

        self._path_smoother = PathSmoother(master_track_ps=self._track_in_map)

        # 2. Get formation width parameter dynamically (defaulting to 4.0 if not declared)
        formation_width = 4.0 

        max_offset = float(formation_width)

        # 4. Run the curvature check loop against the full width
        master_x, master_y, u_fine, tck = self._path_smoother.smooth_track(
            num_points=100, 
            initial_smoothing=2.0,
            max_offset=max_offset,
            safety_margin=1.0,
            num=15 
        )

        # 5. Extract shared progress coordinate
        master_s = self._path_smoother.master_arclength(master_x, master_y)

        # 6. Generate the master-anchored parallel tracks
        generated_tracks_raw = self._path_smoother.compute_dynamic_tracks(
            master_x, master_y, u_fine, tck, 
            num_robots=self._num_robots,
            formation_width=formation_width 
        )

        # 7. Package coordinates back into PoseStamped structures
        self._tracks_in_map = []
        for track_list in generated_tracks_raw:
            pose_list = []
            for point in track_list:
                ps = PoseStamped()
                ps.pose.position.x = point[0]
                ps.pose.position.y = point[1]
                pose_list.append(ps)
            self._tracks_in_map.append(pose_list)

        # 8. Run Hungarian Task Assignment
        if not self._HungarianAssignment():
            self._node.get_logger().error('Assignment Failed. Aborting loop preparation')
            return 

        # 9. Initialize parameterizer with this agent's allocated track
        self._path_parametrizer = PathParameterizer(
            self._this_robot_waypoints, master_s, self._look_a_head_distance)
        self._move_to_pending = False
        self._node.get_logger().info('Loop correctly prepared.')

    def _publish_references(self, desired_speed: float, desired_heading: float)-> None:
        yaw_msg = FloatStamped()
        speed_msg = FloatStamped()
        move_on_place_msg = Bool()

        move_on_place_msg.data = False

        now = self._node.get_clock().now().to_msg()
        yaw_msg.header.stamp = now
        speed_msg.header.stamp = now

        yaw_msg.data = desired_heading
        speed_msg.data = desired_speed

        self._yaw_reference_publisher.publish(yaw_msg)
        self._speed_reference_publisher.publish(speed_msg)
        #self._move_on_place_publisher.publish(move_on_place_msg)

    def _everyone_following(self) -> bool:
        required_robot_count = self._num_robots - 1 

        if len(self._robot_positions) -1 != required_robot_count:
            self._node.get_logger().error(
            f'Error while waiting for peer error... (Got {len(self._peer_errors)}/{required_robot_count}) after the time out.', throttle_duration_sec=2.0)
            return False 
        

        for robot_name in self._peer_errors:
            if self._peer_errors[robot_name] is None:
                self._node.get_logger().info(f'Initializing the errors', throttle_duration_sec=2.0)
                return False
            if self._peer_errors[robot_name] > self._catch_up_distance:
                self._node.get_logger().info(f'{robot_name} is not catching up the carrot, its distance is: {self._peer_errors[robot_name]}', throttle_duration_sec=2.0)
                return False
        
        return True

    def _is_mission_complete(self) -> bool:
        if not self._path_parametrizer.is_at_end:
            return False
        if self.distance_error >= self._catch_up_distance:
            return False
        for robot_name, ready in self._peer_ready.items():
            if not ready:
                return False
        return True

    def _loop_inner(self):

        is_formation_moving = self._everyone_following() and self.distance_error < self._catch_up_distance

        if is_formation_moving:
            self._node.get_logger().info('Everyone is following the carrot, advancing the carrot', throttle_duration_sec=2.0)
            self._path_parametrizer.advance_carrot(self._ds)
        else:
            self._node.get_logger().info('Stopping the carrot for this step', throttle_duration_sec=5.0)
            pass 

        main_carrot_position, lookahead_carrot = self._path_parametrizer.get_carrots()
        main_carrot_x = main_carrot_position[0]
        main_carrot_y = main_carrot_position[1]
        lookahead_carrot_x = lookahead_carrot[0]
        lookahead_carrot_y = lookahead_carrot[1]


        robot_x = self._robot_positions[self._this_robot_name].pose.position.x
        robot_y = self._robot_positions[self._this_robot_name].pose.position.y
                
        self.distance_error = math.hypot(main_carrot_x - robot_x, main_carrot_y - robot_y)

        #self._node.get_logger().info(f'robot_x:{robot_x}, robot_y:{robot_y} and main_carrot_x:{main_carrot_x}, main_carrot_y:{main_carrot_y}', throttle_duration_sec=2.0)
        #self._node.get_logger().info(f'The distace from the carrot is:{self.distance_error}',throttle_duration_sec=3.0)
        desired_heading = math.atan2(main_carrot_y - robot_y, main_carrot_x - robot_x)
        v_desired = self._carrot_speed + self.distance_error * self._catch_up_gain

        max_safe_speed = 1.0
        v_command = min(max_safe_speed, v_desired)

        own_ready = self._path_parametrizer.is_at_end and self.distance_error < self._catch_up_distance
        ready_msg = Bool()
        ready_msg.data = bool(own_ready)
        self._ready_publisher.publish(ready_msg)

        distacne_error_msg = FloatStamped()
        distacne_error_msg.header.stamp = self._node.get_clock().now().to_msg()
        distacne_error_msg.data = self.distance_error
        self._error_publisher.publish(distacne_error_msg)

        if not self._use_sim:
            self._mqtt_publish_formation_error(self.distance_error, distacne_error_msg.header.stamp)
            self._mqtt_publish_mission_ready(bool(own_ready))


        if not is_formation_moving and self.distance_error < self._catch_up_distance:
            self._node.get_logger().info('Carrot stopped and I caught up. Idling thrusters!', throttle_duration_sec=2.0)
            thruster_port_msg = Float32()
            thruster_strb_msg = Float32()
            thruster_port_msg.data = 0.0
            thruster_strb_msg.data = 0.0
            self.thruster_port_pub.publish(thruster_port_msg)
            self.thruster_strb_pub.publish(thruster_strb_msg) 
        else:
            self._publish_references(v_command, desired_heading)
            #des_deg = desired_heading * 180 / math.pi
            #if des_deg < 0:
            #    des_deg += 360
            #error_heading = self._heading - des_deg
            #self._node.get_logger().info(f'des_deg:{des_deg}', throttle_duration_sec=2.0)
            #self._node.get_logger().info(f'self._heading:{self._heading}', throttle_duration_sec=2.0)
#
            #self._node.get_logger().info(f'error_heading:{error_heading}', throttle_duration_sec=2.0)
        
        if self._is_mission_complete():
            self._node.get_logger().info('Mission complete. All robots reached end of tracks.')
            return True
        
    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info("Cancel requested, stopping...")
        self._goal_in_map = None
        if self._saved_background_parameters:
            self._write_captain_parameters(self._saved_background_parameters)
        return True

    def _give_feedback(self) -> str:
        return 'feedback'


def main(args=None):
    rclpy.init(args=args)
    node = Node("floatsam_go_in_formation_action_server")

    go_in_formation_action = FloatsamGoInFormationAction(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()