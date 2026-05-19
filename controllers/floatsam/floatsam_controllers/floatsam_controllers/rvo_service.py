import numpy as np
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from .floatsam_common import FloatSam
from floatsam_msgs.srv import GetSafeVelocity

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy



class RVOservice(Node):
    def __init__(self):
        super().__init__("rvo_service_node")
        self.logger = self.get_logger()
        self.srv = self.create_service(GetSafeVelocity, 'get_safe_velocity', self.compute_safe_velocity_callback)
        self.get_logger().info('RVO Safe Velocity Service is ready.')
        self.declare_node_parameters()
        self.get_node_parameters()
        self._floatsam = FloatSam(self, self.this_robot_name, use_sim=self.use_sim)
        self.odom_in_map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self._odom_subscribers = {}
        self._robot_positions = {}
        self._robot_velocities = {}
        self._odometry_subscriptions()

        self.effective_safety_margin = 2 * self.safety_margin

    def compute_safe_velocity_callback(self, request, response):
        """
        request fields expected:
            robot_id        : str
            pref_velocity   : [vx, vy]   (desired Cartesian velocity)
            goal_position   : [gx, gy]   (current navigation goal in map frame)
                              If not provided / zero-length, goal-aware costs are skipped.
        """
        self.this_robot_id = request.robot_id
        self.pref_velocity = request.pref_velocity

        this_robot_position = self._robot_positions[self.this_robot_name].pose.position
        this_robot_velocity = self._robot_velocities[self.this_robot_name]
        self.this_robot_position = np.array([this_robot_position.x, this_robot_position.y])
        self.this_robot_velocity = np.array([this_robot_velocity.x, this_robot_velocity.y])

        goal = None
        goal_direction_angle = None
        if hasattr(request, 'goal_position') and len(request.goal_position) == 2:
            goal = np.array(request.goal_position, dtype=float)
            to_goal = goal - self.this_robot_position
            dist_to_goal = np.linalg.norm(to_goal)
            if dist_to_goal > 1e-3:
                goal_direction_angle = np.arctan2(to_goal[1], to_goal[0])

        pref_velocity_vec = np.array(self.pref_velocity, dtype=float)

        speed_samples = np.arange(0.0, self.max_speed + self.speed_step, self.speed_step)
        velocity_samples = self._build_velocity_samples(speed_samples, goal_direction_angle)

        pref_is_safe = self._velocity_is_safe(pref_velocity_vec)

        if pref_is_safe:
            pref_speed = float(np.linalg.norm(pref_velocity_vec))
            pref_angle = float(np.arctan2(pref_velocity_vec[1], pref_velocity_vec[0]))
            response.safe_velocity = [pref_speed, pref_angle]
            response.success = True
            response.change = False
            self.get_logger().info('Preferred velocity is safe – no change needed.', throttle_duration_sec=1.0)
            return response

        self.get_logger().info('Preferred velocity is NOT safe – searching for alternative.', throttle_duration_sec=1.0)

        best_velocity = None
        best_cost = np.inf

        for v_polar in velocity_samples:
            speed, angle = v_polar
            v_cart = np.array([speed * np.cos(angle), speed * np.sin(angle)])

            if not self._velocity_is_safe(v_cart):
                continue

            cost = self._velocity_cost(v_cart, pref_velocity_vec, goal)
            if cost < best_cost:
                best_cost = cost
                best_velocity = v_polar

        if best_velocity is None:
            response.success = False
            self.get_logger().warn('No safe velocity found – sending failure.')
            return response

        self.get_logger().info(
            f'Safe velocity found: speed={best_velocity[0]:.2f}  angle={np.degrees(best_velocity[1]):.1f}°', throttle_duration_sec=1.0)
        response.safe_velocity = list(best_velocity)
        response.success = True
        response.change = True
        return response


    def _velocity_cost(self, v_cart, pref_velocity_vec, goal):
        """
        Combined cost that balances three objectives:

        1. Deviation from preferred velocity   – keeps behaviour smooth.
        2. Goal-progress reward                – ensures the robot still moves
                                                 toward its destination after
                                                 an avoidance manoeuvre.
        3. Low-speed penalty                   – prevents the robot from
                                                 "solving" avoidance by stopping.
        """
        deviation_cost = np.linalg.norm(pref_velocity_vec - v_cart)

        progress_cost = 0.0
        if goal is not None:
            to_goal = goal - self.this_robot_position
            to_goal_dist = np.linalg.norm(to_goal)
            if to_goal_dist > 1e-3:
                to_goal_unit = to_goal / to_goal_dist
                progress = np.dot(v_cart, to_goal_unit)   
                progress_cost = -progress                  

        # 3. Low-speed penalty
        speed = np.linalg.norm(v_cart)
        stop_penalty = self.stop_penalty if speed < self.min_useful_speed else 0.0

        return (self.w_deviation * deviation_cost
                + self.w_goal    * progress_cost
                + stop_penalty)


    def _velocity_is_safe(self, v_cart):
        """Return True if v_cart does not enter any VO cone."""
        for idx in self.robot_ids:
            robot_name = f'{self.robot_base_name}_{idx}'
            if robot_name == self.this_robot_name:
                continue
            if robot_name not in self._robot_velocities:
                continue
            rv = self._robot_velocities[robot_name]
            robot_velocity = np.array([rv.x, rv.y])
            v_apex = (self.this_robot_velocity + robot_velocity) / 2.0
            if self.is_in_cone(idx, v_apex, v_cart):
                return False
        return True


    def is_in_cone(self, idx, v_apex, projected_velocity):
        robot_name = f'{self.robot_base_name}_{idx}'
        if robot_name not in self._robot_positions:
            return False

        position = self._robot_positions[robot_name].pose.position
        position = np.array([position.x, position.y])

        rp = position - self.this_robot_position
        distance = np.linalg.norm(rp)

        if distance < 1e-6:
            return True

        relative_velocity = projected_velocity - v_apex
        speed_rel = np.linalg.norm(relative_velocity)

        if speed_rel < 1e-9:
            return False

        ratio = np.clip(self.effective_safety_margin / distance, -1.0, 1.0)
        alpha = np.arcsin(ratio)

        theta = self.compute_angle(rp, relative_velocity)

        if theta >= alpha:
            return False

        approach_speed = speed_rel * np.cos(theta)
        if approach_speed <= 0.0:
            return False

        distance_to_edge = distance - self.effective_safety_margin
        if distance_to_edge <= 0:
            return True

        time_to_collision = distance_to_edge / approach_speed
        return time_to_collision < self.time_horizon


    def compute_angle(self, vector1, vector2):
        norm1 = np.linalg.norm(vector1)
        norm2 = np.linalg.norm(vector2)
        if norm1 < 1e-9 or norm2 < 1e-9:
            return 0.0
        cos_theta = np.clip(np.dot(vector1, vector2) / (norm1 * norm2), -1.0, 1.0)
        return np.arccos(cos_theta)



    def _build_velocity_samples(self, speed_samples, goal_direction_angle=None):
        """
        Build a set of (speed, angle) samples.

        Strategy
        --------
        * A coarse uniform grid covers the full circle so the robot is never
          completely stuck.
        * If a goal direction is known, a denser fan of samples is added in
          the half-space facing the goal (±90°).  This biases the search
          toward solutions that maintain progress.
        * Speed 0 is included so the robot can stop if truly necessary, but
          the cost function penalises it.
        """
        # Coarse uniform coverage
        coarse_angles = np.linspace(0.0, 2.0 * np.pi, self.num_coarse_angles, endpoint=False)

        if goal_direction_angle is not None:
            # Dense fan toward goal
            fine_angles = np.linspace(
                goal_direction_angle - np.pi / 2.0,
                goal_direction_angle + np.pi / 2.0,
                self.num_fine_angles
            )
            all_angles = np.concatenate([coarse_angles, fine_angles])
        else:
            all_angles = coarse_angles

        samples = [
            (float(speed), float(angle))
            for speed in speed_samples
            for angle in all_angles
        ]
        return samples

    

    def declare_node_parameters(self):
        double_desc = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        string_desc = ParameterDescriptor(type=ParameterType.PARAMETER_STRING)
        bool_desc   = ParameterDescriptor(type=ParameterType.PARAMETER_BOOL)
        int_desc    = ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER)

        self.declare_parameter("robot_name",        "floatsam_usv_0", string_desc)
        self.declare_parameter("use_sim",            False,            bool_desc)
        self.declare_parameter("time_horizon",       0.5,              double_desc)
        self.declare_parameter("safety_margin",      10.0,              double_desc)
        self.declare_parameter("max_speed",          3.0,              double_desc)
        self.declare_parameter("speed_step",         0.25,             double_desc)
        self.declare_parameter("update_rate",        0.0,              double_desc)
        self.declare_parameter("num_robots",         2,                int_desc)

        self.declare_parameter("num_coarse_angles",  60,               int_desc)
        self.declare_parameter("num_fine_angles",    60,               int_desc)

        self.declare_parameter("w_deviation",        1.0,              double_desc)
        self.declare_parameter("w_goal",             1.5,              double_desc)
        self.declare_parameter("stop_penalty",       2.0,              double_desc)
        self.declare_parameter("min_useful_speed",   0.1,              double_desc)

    def get_node_parameters(self):
        gp = self.get_parameter

        self.this_robot_name  = gp("robot_name").get_parameter_value().string_value
        self.use_sim          = gp("use_sim").get_parameter_value().bool_value
        self.update_rate      = gp("update_rate").get_parameter_value().double_value
        self.safety_margin    = gp("safety_margin").get_parameter_value().double_value
        self.max_speed        = gp("max_speed").get_parameter_value().double_value
        self.speed_step       = gp("speed_step").get_parameter_value().double_value
        self.time_horizon     = gp("time_horizon").get_parameter_value().double_value
        self.num_robot        = gp("num_robots").get_parameter_value().integer_value
        self.robot_ids        = range(self.num_robot)
        self.robot_base_name  = '_'.join(self.this_robot_name.split('_')[:-1])

        # Sampling
        self.num_coarse_angles = gp("num_coarse_angles").get_parameter_value().integer_value
        self.num_fine_angles   = gp("num_fine_angles").get_parameter_value().integer_value

        # Cost weights
        self.w_deviation      = gp("w_deviation").get_parameter_value().double_value
        self.w_goal           = gp("w_goal").get_parameter_value().double_value
        self.stop_penalty     = gp("stop_penalty").get_parameter_value().double_value
        self.min_useful_speed = gp("min_useful_speed").get_parameter_value().double_value

    # -----------------------------------------------------------------------
    # Odometry subscriptions & callbacks
    # -----------------------------------------------------------------------

    def _odometry_subscriptions(self):
        for robot_id in self.robot_ids:
            odom_topic = f'/{self.robot_base_name}_{robot_id}/smarc/odom_in_map'
            subscriber = self.create_subscription(
                Odometry,
                odom_topic,
                lambda msg, rid=robot_id: self._odom_callback(msg, rid),
                self.odom_in_map_qos
            )
            self._odom_subscribers[robot_id] = subscriber
            self.get_logger().info(f'Subscribed to {odom_topic}')

    def _odom_callback(self, msg: Odometry, robot_id: int):
        """Update position and velocity for a robot from its odometry topic."""
        robot_name = f'{self.robot_base_name}_{robot_id}'

        pose_in_global = PoseStamped()
        pose_in_global.header = msg.header
        pose_in_global.pose   = msg.pose.pose

        self._robot_positions[robot_name]  = pose_in_global
        self._robot_velocities[robot_name] = msg.twist.twist.linear


def main(args=None):
    rclpy.init(args=args)
    node = RVOservice()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()