#!/usr/bin/python

import numpy as np
import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


import traceback

from floatsam_controllers.floatsam_common import FloatSam

from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from geometry_msgs.msg import  PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String


from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_action_base.gentler_action_server import GentlerActionServer
import time

class MoveToPathActionFloatSam():
    def __init__(self,
                 node: Node):
        self._node : Node = node
        
        self.declare_node_parameters()
        self.get_node_parameters()
        

        self.MAP_FRAME : str = self._robot_name + '/map'
        self._floatsam = FloatSam(node, self._robot_name, use_sim=self._use_sim)
        
        self.create_node_subscribers()
        self.create_node_publishers()
        self.initialise_node_attributes()

        self._as = GentlerActionServer(
            node,
            'move_path',
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=10
        )

        self._node.get_logger().info(f'FloatSam move_to server initialized for robot: {self._robot_name}')

    def declare_node_parameters(self) -> None:

        double_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        string_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        bool_desc = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)

        self._node.declare_parameter('use_sim', True, bool_desc)
        self._node.declare_parameter('robot_name', 'floatsam_usv', string_desc)
        self._node.declare_parameter('default_goal_tolerance', 1.0, double_desc)
        self._node.declare_parameter('default_speed_threshold',10.0, double_desc)

        self._node.declare_parameter('yaw_p_gain', 0.3, double_desc)
        self._node.declare_parameter('yaw_i_gain', 0.0, double_desc)
        self._node.declare_parameter('yaw_d_gain', 0.1, double_desc)
        self._node.declare_parameter('yaw_threshold', 0.5, double_desc)

        self._node.declare_parameter('yawrate_p_gain', 300.0, double_desc)
        self._node.declare_parameter('yawrate_i_gain', 0.0, double_desc)
        self._node.declare_parameter('yawrate_d_gain', 30.0, double_desc)

        self._node.declare_parameter('velocity_p_gain', 500.0, double_desc)
        self._node.declare_parameter('velocity_i_gain', 10.0, double_desc)
        self._node.declare_parameter('velocity_d_gain', 0.0, double_desc)

    def get_node_parameters(self) -> None:
        self._use_sim = self._node.get_parameter('use_sim').get_parameter_value().bool_value
        self._robot_name = self._node.get_parameter('robot_name').get_parameter_value().string_value

        self.yaw_p_gain = self._node.get_parameter('yaw_p_gain').get_parameter_value().double_value
        self.yaw_i_gain = self._node.get_parameter('yaw_i_gain').get_parameter_value().double_value
        self.yaw_d_gain = self._node.get_parameter('yaw_d_gain').get_parameter_value().double_value
        self.yaw_threshold = self._node.get_parameter('yaw_threshold').get_parameter_value().double_value

        self.yawrate_p_gain = self._node.get_parameter('yawrate_p_gain').get_parameter_value().double_value
        self.yawrate_i_gain = self._node.get_parameter('yawrate_i_gain').get_parameter_value().double_value
        self.yawrate_d_gain = self._node.get_parameter('yawrate_d_gain').get_parameter_value().double_value

        self.velocity_p_gain = self._node.get_parameter('velocity_p_gain').get_parameter_value().double_value
        self.velocity_i_gain = self._node.get_parameter('velocity_i_gain').get_parameter_value().double_value
        self.velocity_d_gain = self._node.get_parameter('velocity_d_gain').get_parameter_value().double_value

        self._default_goal_tolerance = self._node.get_parameter('default_goal_tolerance').get_parameter_value().double_value
        self._default_speed_threshold = self._node.get_parameter('default_speed_threshold').get_parameter_value().double_value

    def create_node_publishers(self) -> None:
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._yaw_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.YAW_SETPOINT, best_effort_qos)
        self._speed_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, best_effort_qos)

    def create_node_subscribers(self) -> None:
        self._param_cb_group = MutuallyExclusiveCallbackGroup()
        captain_node_name = f'/{self._robot_name}/captain'

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

    def initialise_node_attributes(self):
        self._initial_pos_deadline = int(self._node.get_clock().now().nanoseconds * 1e-9) + 5
        self._initial_pos_timer = self._node.create_timer(0.5, self._check_initial_position)
        self.index=0
        self._decelerating : bool = False
        self._desired_speed : float = 0.0

    @property
    def now_stamp(self):
        return self._node.get_clock().now().to_msg()
    
    @property
    def now_time(self):
        return self.now_stamp.sec + self.now_stamp.nanosec * 1e-9
    
    def log(self, msg: str):
        self._node.get_logger().info(msg)

    def _check_initial_position(self):
        '''Timer callback: print the first floatsam position received from odom_gt (or timeout).'''
        if self._floatsam.floatsam_in_map is not None:
            p = self._floatsam.floatsam_in_map.pose.position
            self._node.get_logger().info(f'Floatsam position from odom_gt: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]')
            try:
                self._initial_pos_timer.cancel()
            except Exception:
                pass
        else:
            now = int(self._node.get_clock().now().nanoseconds * 1e-9)
            if now > self._initial_pos_deadline:
                self._node.get_logger().warning('Timed out waiting for floatsam position from odom_gt')
                try:
                    self._initial_pos_timer.cancel()
                except Exception:
                    pass

    def _on_goal_received(self, goal_request: dict) -> bool:
        
        self._node.get_logger().info(f'Goal request received: {goal_request}')

        self._saved_background_parameters = self._read_captain_parameters()

        move_to_params = []
        param_dict = {
            "yaw_p_gain": self.yaw_p_gain,
            "yaw_i_gain": self.yaw_i_gain,
            "yaw_d_gain": self.yaw_d_gain,
            "yaw_threshold": self.yaw_threshold,
            "yawrate_p_gain": self.yawrate_p_gain,
            "yawrate_i_gain": self.yawrate_i_gain,
            "yawrate_d_gain": self.yawrate_d_gain,
            "velocity_p_gain": self.velocity_p_gain, 
            "velocity_i_gain": self.velocity_i_gain, 
            "velocity_d_gain": self.velocity_d_gain
        }
        
        for name, val in param_dict.items():
            p = Parameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(val)
            move_to_params.append(p)
            
        self._write_captain_parameters(move_to_params)

        try:
            try:
                self._goal_speed = goal_request.get('speed', 2.0)
                if self._goal_speed == 'standard':
                    self._goal_speed = 2.0
                elif self._goal_speed == 'slow':
                    self._goal_speed = 1.0
                elif self._goal_speed == 'fast':
                    self._goal_speed = 5.0
                else:
                    self._goal_speed = float(self._goal_speed)
            except:
                self._node.get_logger().info(f'no valid speed, default to 2.0')
                self._goal_speed = 2.0

            self._goal_in_map=[]
            self._goal_tolerance=[]
            self.index = 0 

            waypoints = goal_request['waypoints']
            if not isinstance(waypoints, list):
                waypoints = [waypoints]

            self._constant_speed = bool(goal_request.get('constant_speed', False))
            self._node.get_logger().info(f'Constant speed mode: {self._constant_speed}')

            for i in range(len(waypoints)):
                self._node.get_logger().info(f'waypoint {i}: {waypoints[i]}')

                gp : GeoPoint = GeoPoint()
                gp.latitude = float(waypoints[i]['latitude'])
                gp.longitude = float(waypoints[i]['longitude'])

                pose_converted = self._floatsam.convert_geopoint_to_map_pose_stamped(gp)
                self._goal_in_map.append(pose_converted)

                tol = float(waypoints[i].get('tolerance', self._default_goal_tolerance))
                self._goal_tolerance.append(tol)

                pos = pose_converted.pose.position
                self._node.get_logger().info(f'Received goal in map: [{pos.x:.2f},{pos.y:.2f},{pos.z:.2f}], tolerance: {tol}, speed: {self._goal_speed}')

            return True
        
        except:
            self._node.get_logger().error('Failed to parse goal request')
            traceback.print_exc()
            return False

    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info('Cancel requested, stopping...')
        self._goal_in_map = None

        if self._saved_background_parameters:
            self._write_captain_parameters(self._saved_background_parameters)
        return True

    def _prepare_loop(self) -> None:
        self._distance_remaining = None
        self._desired_speed = 0.0
        self._decelerating = False
        self.index = 0
        return

    def _loop_inner(self) -> bool|None:
        if self._goal_in_map is None or not self._goal_in_map:
            self._node.get_logger().info('No goal set, failing...')
            if self._saved_background_parameters:
                self._write_captain_parameters(self._saved_background_parameters)
            return False

        if self._floatsam.floatsam_in_map is None:
            self._node.get_logger().info('No floatsam position available yet, waiting...')
            return None
        
        if self.index >= len(self._goal_in_map):
            self._node.get_logger().info('All waypoints reached! SUCCESS.')
            if self._saved_background_parameters:
                self._write_captain_parameters(self._saved_background_parameters)
            return True 

        i = self.index

        goal_position = np.array([self._goal_in_map[i].pose.position.x,
                                  self._goal_in_map[i].pose.position.y])
        
        self_position = np.array([self._floatsam.floatsam_in_map.pose.position.x,
                                  self._floatsam.floatsam_in_map.pose.position.y])

        goal_error = goal_position - self_position
        goal_error_mag = np.linalg.norm(goal_error)
        self._distance_remaining = float(goal_error_mag)

        if self._distance_remaining <= self._goal_tolerance[i]:
            self._node.get_logger().info(f'Reached waypoint {i} within tolerance {self._goal_tolerance[i]}m')
            self.index += 1 
            return None 

        is_last_waypoint = (i == len(self._goal_in_map) - 1)

        if (self._distance_remaining <= self._default_speed_threshold) and is_last_waypoint and not self._constant_speed:
            self._desired_speed = (self._distance_remaining / self._default_speed_threshold) * self._goal_speed
            self._decelerating = True
        else:
            self._desired_speed = self._goal_speed
            self._decelerating = False
            
        error_heading = float(np.arctan2(goal_error[1], goal_error[0]))
        
        speed = float(self._desired_speed)
        
        yaw_msg = FloatStamped()
        speed_msg = FloatStamped()
        now = self._node.get_clock().now().to_msg()
        yaw_msg.header.stamp = now
        yaw_msg.data = error_heading
        speed_msg.header.stamp = now
        speed_msg.data = speed
        self._yaw_reference_publisher.publish(yaw_msg)
        self._speed_reference_publisher.publish(speed_msg)

        return None

    def _give_feedback(self) -> str:
        if self._distance_remaining is not None and self._goal_in_map:
            safe_index = min(self.index, len(self._goal_in_map) - 1)
            feedback = {
                'wp_index': safe_index + 1,
                'wp_total': len(self._goal_in_map),
                'distance_remaining': round(self._distance_remaining, 3),
                'tolerance': round(self._goal_tolerance[safe_index], 3),
                'desired_speed': round(self._desired_speed, 3),
                'decelerating': self._decelerating,
            }
        else:
            feedback = {
                'decelerating': False,
                'desired_speed': 0.0,
            }
        return json.dumps(feedback)
        

def main(args=None):
    rclpy.init(args=args)

    node = Node('floatsam_move_to_path_action_server')
    move_to_path_action = MoveToPathActionFloatSam(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()
