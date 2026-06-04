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

import time
import traceback

from std_msgs.msg import Bool
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

        self._as = GentlerActionServer(
            node, 
            'go_in_formation',
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=10
        )

        self.declare_node_parameters()
        self.get_node_parameters()
        self._floatsam = FloatSam(self._node, self._this_robot_name, use_sim=self._use_sim)


        self._robot_positions = {}
        self._odom_subscribers = {}

        self.create_subscriptions()
        self.create_node_publishers()

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

        self._robot_ids        = range(self._num_robots)
        self._robot_base_name  = '_'.join(self._this_robot_name.split('_')[:-1])
        self._others_arrived_flag = False
        self._ds = self._carrot_speed * self._update_rate

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
                self._peer_errors[robot_name] = 0.0 
                
                topic_name = f'/{robot_name}/formation_error'
                
                self._node.create_subscription(
                    FloatStamped, 
                    topic_name, 
                    lambda msg, rn=robot_name: self._peer_error_cb(msg, rn), 
                    self._best_effort_qos 
                )

    def create_node_publishers(self) -> None:
        self._move_on_place_publisher = self._node.create_publisher(Bool, 'move_on_place', 1)
        self._best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._yaw_reference_publisher = self._node.create_publisher(
            FloatStamped, FloatsamTopics.YAW_SETPOINT, self._best_effort_qos)
        
        self._speed_reference_publisher = self._node.create_publisher(
            FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, self._best_effort_qos)
        
        self._error_publisher = self._node.create_publisher(
            FloatStamped, 'formation_error', self._best_effort_qos)

    def create_subscriptions(self) -> None:
        self._odometry_subscriptions()
        self._peers_error_subscriptions()

      


#     Goal Structure :
#    {
#        'desired_speed' : 2.0,
#        'tracks' : [
#            #Track 0 (outer list element 0)
#            [
#                {'latitude': 59.00, 'longitude': 18.00} # Inner list element 0
#                {'latitude': 59.01, 'longitude': 18.01} # Inner list element 1
#            ], 
#            #Track 1 (outer list element 1)
#            [
#                {'latitude': 59.00, 'longitude': 18.00} # Inner list element 0
#                {'latitude': 59.01, 'longitude': 18.01} # Inner list element 1
#            ]
#        ]
#    }
#           


# Functions for GentlerActionServer

    def _on_goal_received(self, goal_request: dict) -> bool:
        self._node.get_logger().info(f'Goal received: {goal_request}')
        
        try:
            self._desired_speed = float(goal_request.get('desired_speed', 2.0))

            raw_tracks = goal_request.get('tracks', None)

            if raw_tracks is None or not isinstance(raw_tracks, list) or len(raw_tracks) == 0:
                self._node.get_logger().error("Invalid or missing 'tracks' list.")
                return False
            
            self._tracks_in_map = []

            for track_idx, track in enumerate(raw_tracks):
                if not isinstance(track, list) or len(track)<2:
                    self._node.get_logger().error(f"Track {track_idx} must be a list of at least 2 waypoints.")
                    return False
                
                map_waypoints_for_this_track = []

                for wp_idx, wp in enumerate(track):
                    gp = GeoPoint()
                    gp.latitude = float(wp['latitude'])
                    gp.longitude = float(wp['longitude'])
                    gp.altitude = 0.0

                    try:
                        map_pose = self._floatsam.convert_geopoint_to_map_pose_stamped(gp)
                        map_waypoints_for_this_track.append(map_pose)
                    except Exception as tf_error:
                        self._node.get_logger().error(f"TF Error on Track {track_idx}, WP {wp_idx}: {tf_error}")
                        return False 
                
                self._tracks_in_map.append(map_waypoints_for_this_track)
            
            self._node.get_logger().info(f"Successfully converted {len(self._tracks_in_map)} tracks into map frame.")

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
            throttle_duration_sec=1.0
            )
            time.sleep(0.5)

        if len(self._robot_positions) != required_robot_count:
            self._node.get_logger().error(
            f'Error while waiting for odometry... (Got {len(self._robot_positions)}/{required_robot_count}) after the time out.', 
            throttle_duration_sec=1.0
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
        self._node.get_logger().info('Preapering loop.')
        if not self._HungarianAssignment():
            self._node.get_logger().error('Assignment Failed. Aborting loop preparation')
            return 
        
        self._path_parametrizer = PathParameterizer(self._this_robot_waypoints)
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
        self._move_on_place_publisher.publish(move_on_place_msg)

        self._node.get_logger().info(f'Publishing desired speed: {desired_speed} and desired_heading{desired_heading}', throttle_duration_sec=1.0)

    def _everyone_following(self, time_out: float = 5.0) -> bool:
        required_robot_count = len(self._tracks_in_map)
        start_time = time.time()

        while (time.time() - start_time) < time_out:
            if len(self._peer_errors) == required_robot_count:
                break

            self._node.get_logger().info(
            f'Waiting for peers error... (Got {len(self._peer_errors)}/{required_robot_count})', 
            throttle_duration_sec=1.0
            )
            time.sleep(0.5)

        if len(self._robot_positions) != required_robot_count:
            self._node.get_logger().error(
            f'Error while waiting for peer error... (Got {len(self._peer_errors)}/{required_robot_count}) after the time out.', 
            throttle_duration_sec=1.0
            )
            return False 
        
        
        
        return True

    def _loop_inner(self):
        self._path_parametrizer.advance_carrot(self._ds)
        main_carrot_position, lookahead_carrot = self._path_parametrizer.get_carrots()
        main_carrot_x = main_carrot_position[0]
        main_carrot_y = main_carrot_position[1]
        lookahead_carrot_x = lookahead_carrot[0]
        lookahead_carrot_y = lookahead_carrot[1]


        robot_x = self._robot_positions[self._this_robot_name].pose.position.x
        robot_y = self._robot_positions[self._this_robot_name].pose.position.y
                
        distance_error = math.hypot(main_carrot_x - robot_x, main_carrot_y - robot_y)
        desired_heading = math.atan2(lookahead_carrot_y - robot_y, lookahead_carrot_x - robot_x)
        v_desired = self._carrot_speed + distance_error * self._catch_up_gain
 
        self._publish_references(v_desired, desired_heading)
        


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