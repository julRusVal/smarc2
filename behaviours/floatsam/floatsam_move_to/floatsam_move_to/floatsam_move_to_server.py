#!/usr/bin/python

import numpy as np
import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType, ParameterDescriptor
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


import traceback

from floatsam_controllers.floatsam_common import FloatSam

from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from floatsam_msgs.srv import GetSafeVelocity
from geometry_msgs.msg import  PointStamped, PoseStamped
from geographic_msgs.msg import GeoPoint
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from std_msgs.msg import String
from std_msgs.msg import Bool



from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_action_base.gentler_action_server import GentlerActionServer
import time

class MoveToActionFloatSam():
    def __init__(self,
                 node: Node):
        self._node : Node = node

        self.rvo_invervents=0

        self.declare_node_parameters()
        self.get_node_parameters()

        self._client_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_node_publishers()
        self.create_subscriptions()
        
        self._param_cb_group = MutuallyExclusiveCallbackGroup()
        self.create_clients()

        self.MAP_FRAME : str = self._robot_name + '/map'
        self._floatsam = FloatSam(node, self._robot_name, use_sim=self._use_sim)
        
        self._node.get_logger().info(f"FloatSam move_to server initialized for robot: {self._robot_name}")  

        self._as = GentlerActionServer(
            node,
            "move_to",
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=10
        )

        self._initial_pos_deadline = int(self._node.get_clock().now().nanoseconds * 1e-9) + 5
        self._initial_pos_timer = self._node.create_timer(0.5, self._check_initial_position)

    def declare_node_parameters(self) -> None:
        double_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        string_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        bool_desc = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)

        self._node.declare_parameter("use_sim", True, bool_desc)
        self._node.declare_parameter("robot_name", 'floatsam_usv', string_desc)

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

        self._node.declare_parameter("goal_tolerance", 1.5, double_desc)
        self._node.declare_parameter("speed_threshold", 10.0, double_desc)
    
    def get_node_parameters(self) -> None:
        self._use_sim = self._node.get_parameter('use_sim').get_parameter_value().bool_value
        self._robot_name = self._node.get_parameter('robot_name').get_parameter_value().string_value

        self._move_to_yaw_p_gain = self._node.get_parameter('yaw_p_gain').get_parameter_value().double_value
        self._move_to_yaw_i_gain = self._node.get_parameter('yaw_i_gain').get_parameter_value().double_value
        self._move_to_yaw_d_gain = self._node.get_parameter('yaw_d_gain').get_parameter_value().double_value
        self._move_to_yaw_threshold = self._node.get_parameter('yaw_threshold').get_parameter_value().double_value

        self._move_to_yawrate_p_gain = self._node.get_parameter('yawrate_p_gain').get_parameter_value().double_value
        self._move_to_yawrate_i_gain = self._node.get_parameter('yawrate_i_gain').get_parameter_value().double_value
        self._move_to_yawrate_d_gain = self._node.get_parameter('yawrate_d_gain').get_parameter_value().double_value

        self._move_to_velocity_p_gain = self._node.get_parameter('velocity_p_gain').get_parameter_value().double_value
        self._move_to_velocity_i_gain = self._node.get_parameter('velocity_i_gain').get_parameter_value().double_value
        self._move_to_velocity_d_gain = self._node.get_parameter('velocity_d_gain').get_parameter_value().double_value

        self._default_goal_tolerance = self._node.get_parameter('goal_tolerance').get_parameter_value().double_value
        self._default_speed_threshold = self._node.get_parameter('speed_threshold').get_parameter_value().double_value

    def create_node_publishers(self) -> None:
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._yaw_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.YAW_SETPOINT, best_effort_qos)
        self._speed_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, best_effort_qos)

    def create_subscriptions(self) -> None:
        
        self._move_on_place_publisher = self._node.create_publisher(Bool, 'move_on_place', 1)
        self._rvo_client = self._node.create_client(GetSafeVelocity, 'get_safe_velocity', callback_group=self._client_cb_group)
        self._captain_parameters_publisher = self._node.create_publisher(String, 'captain_parameters',10)

    def create_clients(self) -> None:
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
    
    @property
    def now_stamp(self):
        return self._node.get_clock().now().to_msg()
    
    @property
    def now_time(self):
        return self.now_stamp.sec + self.now_stamp.nanosec * 1e-9
    
    def log(self, msg: str):
        self._node.get_logger().info(msg)

    def _check_initial_position(self):
        """Timer callback: print the first floatsam position received from odom_gt (or timeout)."""
        if self._floatsam.floatsam_in_map is not None:
            p = self._floatsam.floatsam_in_map.pose.position
            self._node.get_logger().info(f"Floatsam position from odom_gt: [{p.x:.2f}, {p.y:.2f}, {p.z:.2f}]")
            try:
                self._initial_pos_timer.cancel()
            except Exception:
                pass
        else:
            now = int(self._node.get_clock().now().nanoseconds * 1e-9)
            if now > self._initial_pos_deadline:
                self._node.get_logger().warning("Timed out waiting for floatsam position from odom_gt")
                try:
                    self._initial_pos_timer.cancel()
                except Exception:
                    pass

    def _on_goal_received(self, goal_request: dict) -> bool:
        
        self._node.get_logger().info(f"Goal request received: {goal_request}")

        self._saved_background_parameters = self._read_captain_parameters()

        move_to_params = []
        param_dict = {
            "yaw_p_gain": self._move_to_yaw_p_gain,
            "yaw_i_gain": self._move_to_yaw_i_gain,
            "yaw_d_gain": self._move_to_yaw_d_gain,
            "yaw_threshold": self._move_to_yaw_threshold,
            "yawrate_p_gain": self._move_to_yawrate_p_gain,
            "yawrate_i_gain": self._move_to_yawrate_i_gain,
            "yawrate_d_gain": self._move_to_yawrate_d_gain,
            "velocity_p_gain": self._move_to_velocity_p_gain, 
            "velocity_i_gain": self._move_to_velocity_i_gain, 
            "velocity_d_gain": self._move_to_velocity_d_gain
        }
        
        for name, val in param_dict.items():
            p = Parameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(val)
            move_to_params.append(p)
            
        self._write_captain_parameters(move_to_params)

        try:
            gp : GeoPoint = GeoPoint()
            gp.latitude = goal_request['waypoint']['latitude']
            gp.longitude = goal_request['waypoint']['longitude']

            self._goal_in_map = self._floatsam.convert_geopoint_to_map_pose_stamped(gp)

            self._goal_tolerance = float(goal_request['waypoint']['tolerance'])
            self._node.get_logger().info(f"Goal tolerance: {self._goal_tolerance}")


            try:
                self._goal_speed = goal_request['speed']
                if self._goal_speed == "standard":
                    self._goal_speed = 2.0  

                elif self._goal_speed == "slow":
                    self._goal_speed = 1.0  

                elif self._goal_speed == "fast":
                    self._goal_speed = 5.0 
                
                else:
                    self._goal_speed = 2.0  

            except Exception as e:
                self._node.get_logger().warning(f"No valid speed specified, using default: {e}")
                self._goal_speed = 2.0

            
            self._constant_speed = bool(goal_request.get('constant_speed', False))
            self._node.get_logger().info(f"Constant speed mode: {self._constant_speed}")


            pos = self._goal_in_map.pose.position
            
            self._node.get_logger().info(f"Received goal in map: [{pos.x:.2f},{pos.y:.2f},{pos.z:.2f}], tolerance: {self._goal_tolerance}, speed: {self._goal_speed}")
            
            return True
        
        except Exception as e:
            self._node.get_logger().error(f"Failed to parse goal request: {e}")
            traceback.print_exc()
            return False

    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info("Cancel requested, stopping...")
        self._goal_in_map = None
        if self._saved_background_parameters:
            self._write_captain_parameters(self._saved_background_parameters)
        return True

    def _prepare_loop(self) -> None:
        self._distance_remaining = None
        return

    def _loop_inner(self) -> bool|None:
        if self._goal_in_map is None:
            self._node.get_logger().info("No goal set, failing...", throttle_duration_sec=0.5)
            if self._saved_background_parameters:
                self._write_captain_parameters(self._saved_background_parameters)
            return False

        if self._goal_tolerance is None:
            self._node.get_logger().info("No goal tolerance set, failing...", throttle_duration_sec=0.5)
            if self._saved_background_parameters:
                self._write_captain_parameters(self._saved_background_parameters)
            return False

        if self._floatsam.floatsam_in_map is None:
            self._node.get_logger().info("No floatsam position available yet, waiting...", throttle_duration_sec=0.5)
            return None
        
        goal_position = np.array([self._goal_in_map.pose.position.x,
                                  self._goal_in_map.pose.position.y])
        
        self_position = np.array([self._floatsam.floatsam_in_map.pose.position.x,
                                  self._floatsam.floatsam_in_map.pose.position.y])
        
        goal_error = goal_position - self_position
        goal_error_mag = np.linalg.norm(goal_error)
        self._distance_remaining = float(goal_error_mag)

        if self._distance_remaining <= self._goal_tolerance:
            self._node.get_logger().info(f"Reached goal within tolerance {self._goal_tolerance}m", throttle_duration_sec=0.5)
            if self._saved_background_parameters:
                self._write_captain_parameters(self._saved_background_parameters)
            self._node.get_logger().info(f'RVO acted:{self.rvo_invervents}')
            return True
        
        if self._distance_remaining <= self._default_speed_threshold and not self._constant_speed:
            self._desired_speed = (self._distance_remaining / self._default_speed_threshold) * self._goal_speed
            self._node.get_logger().info(f"Slowing down, new speed: {self._desired_speed:.2f}", throttle_duration_sec=0.5)
        else:
            self._desired_speed = self._goal_speed
    
        error_heading = float(np.arctan2(goal_error[1], goal_error[0]))
        
        self._node.get_logger().info(f"The distance remaining is {self._distance_remaining:.2f} m", throttle_duration_sec=1.0)
        speed = float(self._desired_speed)
        
        rvo_request = GetSafeVelocity.Request()
        rvo_request.robot_id = self._robot_name
        rvo_request.pref_velocity = [speed * np.cos(error_heading), speed * np.sin(error_heading)]
        rvo_request.goal_position = [float(goal_position[0]), float(goal_position[1])]

        move_on_place_msg = Bool()
        move_on_place_msg.data = True 

        if self._rvo_client.service_is_ready():
            future = self._rvo_client.call_async(rvo_request)
            deadline = time.time() + 0.5
            while not future.done() and time.time() < deadline:
                time.sleep(0.01)

            if not future.done():
                self._node.get_logger().warning('RVO service call timed out, skipping publish', throttle_duration_sec=0.5)
                return None

            rvo_response = future.result()
            if not rvo_response.success:
                self._node.get_logger().warning('RVO service returned success=False, skipping publish', throttle_duration_sec=0.5)
                return None

            safe_speed = rvo_response.safe_velocity[0]
            safe_angle = rvo_response.safe_velocity[1]
            if rvo_response.change == True:
                self._node.get_logger().warning(f'RVO changed the velocity', throttle_duration_sec=0.5)
                self.rvo_invervents += 1
                move_on_place_msg.data = False
        else:
            self._node.get_logger().warning('RVO service not available, using preferred velocity directly', throttle_duration_sec=1.0)
            safe_speed = speed
            safe_angle = error_heading

        yaw_msg = FloatStamped()
        speed_msg = FloatStamped()
        now = self._node.get_clock().now().to_msg()
        yaw_msg.header.stamp = now
        yaw_msg.data = safe_angle
        speed_msg.header.stamp = now
        speed_msg.data = safe_speed
        self._yaw_reference_publisher.publish(yaw_msg)
        self._speed_reference_publisher.publish(speed_msg)
        self._move_on_place_publisher.publish(move_on_place_msg)

        angle_msg = FloatStamped()
        angle_msg.header.stamp = now
        angle_msg.data = 0.5  
        
        return None

    def _give_feedback(self) -> str:
        if self._distance_remaining is not None:
            return f"Distance remaining: {self._distance_remaining:.2f} (tolerance: {self._goal_tolerance:.2f}m)"
        else:
            return "No distance remaining info"
                
            

def main(args=None):
    rclpy.init(args=args)
    node = Node("floatsam_move_to_action_server")

    move_to_action = MoveToActionFloatSam(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()