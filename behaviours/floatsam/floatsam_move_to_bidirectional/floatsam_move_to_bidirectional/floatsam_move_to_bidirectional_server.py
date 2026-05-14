#!/usr/bin/python

import numpy as np
import rclpy
import json
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration

import traceback

from floatsam_controllers.floatsam_common import FloatSam

from smarc_msgs.msg import FloatStamped
from smarc_msgs.msg import Topics as SmarcTopics
from smarc_control_msgs.msg import Topics as ControlTopics
from floatsam_msgs.msg import Topics as FloatsamTopics
from std_msgs.msg import Float32
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

class MoveToActionFloatSam():
    def __init__(self,
                 node: Node):
        self._node : Node = node
        
        self._node.declare_parameter('use_sim', True)
        self._use_sim = self._node.get_parameter('use_sim').get_parameter_value().bool_value
        
        self._robot_name : str = self._node.get_parameter('robot_name').value
        
        self.MAP_FRAME : str = self._robot_name + '/map'
        self._floatsam = FloatSam(node, self._robot_name, use_sim=self._use_sim)
        
        self._node.get_logger().info(f"FloatSam move_to server initialized for robot: {self._robot_name}")

        self._default_goal_tolerance = 1  
        self._default_speed_threshold = 10   

        self.declare_node_parameters()

        self._loiter_yaw_p_gain = str(self._node.get_parameter('yaw_p_gain').value)
        self._loiter_yaw_i_gain = str(self._node.get_parameter('yaw_i_gain').value)
        self._loiter_yaw_d_gain = str(self._node.get_parameter('yaw_d_gain').value)
        self._loiter_yaw_threshold = str(self._node.get_parameter('yaw_threshold').value)

        self._loiter_yawrate_p_gain = str(self._node.get_parameter('yawrate_p_gain').value)
        self._loiter_yawrate_i_gain = str(self._node.get_parameter('yawrate_i_gain').value)
        self._loiter_yawrate_d_gain = str(self._node.get_parameter('yawrate_d_gain').value)

        self._loiter_velocity_p_gain = str(self._node.get_parameter('velocity_p_gain').value)
        self._loiter_velocity_i_gain = str(self._node.get_parameter('velocity_i_gain').value)
        self._loiter_velocity_d_gain = str(self._node.get_parameter('velocity_d_gain').value)

        self._yaw_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.YAW_SETPOINT, 10)

        self._speed_reference_publisher = self._node.create_publisher(FloatStamped, FloatsamTopics.VELOCITY_SETPOINT, 10)

        self._move_on_place_publisher = self._node.create_publisher(Bool, 'move_on_place', 1)

        self._captain_parameters_publisher = self._node.create_publisher(
            String, 
            'captain_parameters',
            10
        )

        self.yaw_measurement = 0.0
        self.last_yaw_meas_time = 0.0
        
        self._node.create_subscription(Float32, ControlTopics.CONTROL_YAW_TOPIC,
                                       self.yaw_meas_cb, 1)
        
        self._as = GentlerActionServer(
            node,
            "move_to_bidirectional",
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
        self._node.declare_parameter("yaw_p_gain", 0.3)
        self._node.declare_parameter("yaw_i_gain", 0.0)
        self._node.declare_parameter("yaw_d_gain", 0.1)
        self._node.declare_parameter("yaw_threshold", 0.5)

        self._node.declare_parameter("yawrate_p_gain", 300.0)
        self._node.declare_parameter("yawrate_i_gain", 0.0)
        self._node.declare_parameter("yawrate_d_gain", 30.0)

        self._node.declare_parameter("velocity_p_gain", 500.0)
        self._node.declare_parameter("velocity_i_gain", 10.0)
        self._node.declare_parameter("velocity_d_gain", 0.0)
    
    
    @property
    def now_stamp(self):
        return self._node.get_clock().now().to_msg()
    
    @property
    def now_time(self):
        return self.now_stamp.sec + self.now_stamp.nanosec * 1e-9
    
    def log(self, msg: str):
        self._node.get_logger().info(msg)
    
    def yaw_meas_cb(self, msg):
        self.last_yaw_meas_time = int(self._node.get_clock().now().nanoseconds * 1e-9)
        self.yaw_measurement = msg.data

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
        return True

    def _prepare_loop(self) -> None:
        self._distance_remaining = None
        return

    def _loop_inner(self) -> bool|None:
        if self._goal_in_map is None:
            self._node.get_logger().info("No goal set, failing...")
            return False

        if self._goal_tolerance is None:
            self._node.get_logger().info("No goal tolerance set, failing...")
            return False

        if self._floatsam.floatsam_in_map is None:
            self._node.get_logger().info("No floatsam position available yet, waiting...")
            return None
        
        goal_position = np.array([self._goal_in_map.pose.position.x,
                                  self._goal_in_map.pose.position.y])
        
        self_position = np.array([self._floatsam.floatsam_in_map.pose.position.x,
                                  self._floatsam.floatsam_in_map.pose.position.y])

    
        self._node.get_logger().info(f"Current position: [{self_position[0]:.2f}, {self_position[1]:.2f}]")
        self._node.get_logger().info(f"Goal position:    [{goal_position[0]:.2f}, {goal_position[1]:.2f}]")
        
        goal_error = goal_position - self_position
        goal_error_mag = np.linalg.norm(goal_error)
        self._distance_remaining = float(goal_error_mag)

        if self._distance_remaining <= self._goal_tolerance:
            self._node.get_logger().info(f"Reached goal within tolerance {self._goal_tolerance}m")
            return True
        
        if self._distance_remaining <= self._default_speed_threshold and not self._constant_speed:
            self._desired_speed = (self._distance_remaining / self._default_speed_threshold) * self._goal_speed
            self._node.get_logger().info(f"Slowing down, new speed: {self._desired_speed:.2f}")
        else:
            self._desired_speed = self._goal_speed
    
        
        error_heading = float(np.arctan2(goal_error[1], goal_error[0]))
        
        # Check if going backwards would be more efficient
        heading_diff = error_heading - self.yaw_measurement
        # Normalize heading_diff to [-pi, pi]
        while heading_diff > np.pi:
            heading_diff -= 2 * np.pi
        while heading_diff < -np.pi:
            heading_diff += 2 * np.pi
        
        speed = float(self._desired_speed)
        
        if abs(heading_diff) > np.pi / 2:
            # Goal is more than 90 degrees away, go backwards instead
            error_heading = error_heading + np.pi
            # Normalize error_heading to [-pi, pi]
            while error_heading > np.pi:
                error_heading -= 2 * np.pi
            while error_heading < -np.pi:
                error_heading += 2 * np.pi
            speed = -speed
            self._node.get_logger().info(f"Goal is behind, reversing: heading={error_heading:.2f}, speed={speed:.2f}")
        
        self._node.get_logger().info(f"The distance remaining is {self._distance_remaining:.2f} m")
        self._node.get_logger().info(f"The desired speed is {self._desired_speed:.2f} m/s")
        
        safe_speed = speed
        safe_angle = error_heading
        
        move_on_place_msg = Bool()
        move_on_place_msg.data = True

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
        self._publish_captain_parametrs()
        
        return None

    def _give_feedback(self) -> str:
        if self._distance_remaining is not None:
            return f"Distance remaining: {self._distance_remaining:.2f} (tolerance: {self._goal_tolerance:.2f}m)"
        else:
            return "No distance remaining info"
        
    def _publish_captain_parametrs(self):
        """It publish the message containing the parameters for captain node"""
        parameters = {
            "yaw_p_gain" : self._loiter_yaw_p_gain,
            "yaw_i_gain" : self._loiter_yaw_i_gain,
            "yaw_d_gain" : self._loiter_yaw_d_gain,
            "yaw_threshold" : self._loiter_yaw_threshold,
            "yawrate_p_gain" : self._loiter_yawrate_p_gain,
            "yawrate_i_gain" : self._loiter_yawrate_i_gain,
            "yawrate_d_gain" : self._loiter_yaw_d_gain,
            "velocity_p_gain" : self._loiter_velocity_p_gain, 
            "velocity_i_gain" : self._loiter_velocity_i_gain, 
            "velocity_d_gain" : self._loiter_velocity_d_gain
        }
        msg = String()
        msg.data = json.dumps(parameters)
        self._captain_parameters_publisher.publish(msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = Node("floatsam_move_to_action_server")
    node.declare_parameter('robot_name', 'floatsam_usv')

    move_to_action = MoveToActionFloatSam(node)
    executor = MultiThreadedExecutor()
    rclpy.spin(node, executor=executor)
    node.destroy_node()
    rclpy.shutdown()