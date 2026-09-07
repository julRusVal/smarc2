import py_trees
import numpy as np
import json
from scipy.optimize import linear_sum_assignment
from rclpy.action import ActionClient
from smarc_msgs.action import BaseAction
from smarc_msgs.msg import FloatStamped
from floatsam_msgs.msg import Topics as FloatsamTopics
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String


class CollisionPlaceholder():
    def __init__(self):
        self.counter = 0
        self.is_colliding = False

class HaveGoal(py_trees.behaviour.Behaviour):
    """Check if a goal has been assigned to this agent."""
    
    def __init__(self, name="HaveGoal"):
        super().__init__(name)
        self.node = None  
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("formation_points", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        if not hasattr(self.blackboard, 'formation_points') or self.blackboard.formation_points is None:
            self.node.get_logger().info(f"{self.name}: No formation points received")
            return py_trees.common.Status.FAILURE
        
        if not hasattr(self.blackboard, 'robot_assignments') or self.blackboard.robot_assignments is None:
            self.node.get_logger().info(f"{self.name}: No robot assignments yet")
            return py_trees.common.Status.FAILURE
        
        if len(self.blackboard.robot_assignments) == 0:
            self.node.get_logger().info(f"{self.name}: Robot assignments dictionary is empty")
            return py_trees.common.Status.FAILURE
            
        self.node.get_logger().info(f"{self.name}: Have goal - {len(self.blackboard.robot_assignments)} robots assigned")
        return py_trees.common.Status.SUCCESS

class HungarianAssignment(py_trees.behaviour.Behaviour):
    """Perform Hungarian algorithm to assign targets to agents."""
    
    def __init__(self, name="HungarianAssignment"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("formation_points", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions", access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.WRITE)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        if not hasattr(self.blackboard, 'formation_points'):
            self.node.get_logger().info(f"{self.name}: The formation points are not available yet")
            return py_trees.common.Status.RUNNING
        if not hasattr(self.blackboard, 'robot_positions'):
            self.node.get_logger().info(f"{self.name}: The robot positions are not available yet")
            return py_trees.common.Status.RUNNING

        try: 
            self.node.get_logger().info(f"{self.name}: Running assignment algorithm...")
            robot_names = sorted(self.blackboard.robot_positions.keys())
            cost_matrix = self.compute_cost_matrix(self.blackboard.formation_points, self.blackboard.robot_positions, robot_names)
            row_ind, col_ind = linear_sum_assignment(cost_matrix)

            for i in range(len(row_ind)):
                robot_key = robot_names[row_ind[i]]
                task_idx = col_ind[i]
                self.node.get_logger().info(f"{self.name}: {robot_key} assigned to goal_{task_idx}")
                self.blackboard.robot_assignments[robot_key] = f'goal_{task_idx}'

            return py_trees.common.Status.SUCCESS
        
        except Exception as e:
            self.node.get_logger().error(f"{self.name}: Exception: {e}")
            return py_trees.common.Status.FAILURE

    def compute_cost_matrix(self, formation_points, robot_positions, robot_names):
        size = len(robot_names)
        cost_matrix = np.zeros((size, size))
        for i, name in enumerate(robot_names):
            for j in range(size):
                cost_matrix[i][j] = self.compute_distance(robot_position=robot_positions[name], goal_position=formation_points[f'goal_{j}'])
        return cost_matrix
    
    def compute_distance(self, robot_position, goal_position):
        rx = robot_position.pose.position.x
        ry = robot_position.pose.position.y
        return (rx - goal_position[0])**2 + (ry - goal_position[1])**2

class ArrivalCheck(py_trees.behaviour.Behaviour):
    """Check if agent has arrived at assigned target position."""
    
    def __init__(self, name="ArrivalCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name", access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_arrived_flag", access=py_trees.common.Access.READ)
        self.this_robot_name = self.blackboard.this_robot_name

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        self.node.get_logger().info(f"{self.name}: Update begin")

        if not hasattr(self.blackboard, 'this_robot_arrived_flag') or self.blackboard.this_robot_arrived_flag == False:
            self.node.get_logger().info(f"{self.this_robot_name}: Is not arrived yet")
            return py_trees.common.Status.FAILURE
        
        self.node.get_logger().info(f"{self.this_robot_name}: Is arrived")
        return py_trees.common.Status.SUCCESS

class MoveToClient(py_trees.behaviour.Behaviour):
    """Action client that calls the move_to action server."""

    def __init__(self, name="MoveToClient"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_assignments",               access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_positions",                 access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points",                access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points_latlon",         access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_name",                 access=py_trees.common.Access.READ)
        self.blackboard.register_key("max_velocity",                    access=py_trees.common.Access.READ)
        self.blackboard.register_key("this_robot_arrived_flag",         access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("last_point_tolerance_move_path",  access=py_trees.common.Access.READ)

        self._action_client = None
        self.blackboard.this_robot_arrived_flag = False

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self._floatsam = kwargs['node']._floatsam
        self._action_client = ActionClient(self.node, BaseAction, 'move_to')
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        self._send_goal_future  = None
        self._get_result_future = None
        self._goal_handle       = None
        self._goal_accepted     = False
        self._goal_done         = False
        self._goal_succeeded    = False

    def update(self):
        self.node.get_logger().info(f"{self.name}: Update begin")

        if self._send_goal_future is None:
            my_name     = self.blackboard.this_robot_name
            assignments = self.blackboard.robot_assignments
            if my_name not in assignments:
                self.node.get_logger().warn(f"{self.name}: No assignment for {my_name}")
                return py_trees.common.Status.FAILURE

            my_goal_key  = assignments[my_name]
            final_latlon = self.blackboard.formation_points_latlon.get(my_goal_key)
            if final_latlon is None:
                self.node.get_logger().warn(f"{self.name}: No lat/lon data for {my_goal_key}")
                return py_trees.common.Status.FAILURE

            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn(f"{self.name}: move_to action server not available")
                return py_trees.common.Status.FAILURE

            goal_dict = {
                'waypoint': {
                    'latitude':  final_latlon['latitude'],
                    'longitude': final_latlon['longitude'],
                    'tolerance': self.blackboard.last_point_tolerance_move_path
                },
                'speed': self.blackboard.max_velocity
            }
            goal_msg = BaseAction.Goal()
            goal_msg.goal.data = json.dumps(goal_dict)

            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self._feedback_cb
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
            self.node.get_logger().info(f"{self.name}: Goal sent to move_to ({my_goal_key})")
            return py_trees.common.Status.RUNNING

        if not self._goal_accepted:
            self.node.get_logger().info(f"{self.name}: Waiting for goal acceptance...")
            if self._goal_done:
                self.node.get_logger().info(f"{self.name}: Goal was rejected by the server")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING

        if self._goal_done:
            self.node.get_logger().info(
                f"{self.name}: Goal finished: {'SUCCEEDED' if self._goal_succeeded else 'FAILED'}"
            )
            return (
                py_trees.common.Status.SUCCESS
                if self._goal_succeeded
                else py_trees.common.Status.FAILURE
            )

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.node.get_logger().info(f"{self.name}: Terminate called with new_status={new_status}")
        if self._goal_handle is not None and not self._goal_done:
            self._goal_handle.cancel_goal_async()

    def _feedback_cb(self, feedback_msg):
        try:
            fb = json.loads(feedback_msg.feedback.feedback.data)
            self.node.get_logger().info(f"{self.name}: Feedback - {fb}")
        except Exception:
            pass

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name}: Goal rejected by move_to server")
            self._goal_done = True
            return
        self._goal_handle   = goal_handle
        self._goal_accepted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.node.get_logger().info(f"{self.name}: Goal result received")
        result = future.result().result
        self._goal_succeeded = result.success
        self._goal_done      = True
        if self._goal_succeeded:
            self.blackboard.this_robot_arrived_flag = True


class AllArrivalCheck(py_trees.behaviour.Behaviour):
    """
    Check if ALL robots have reached their assigned formation goals.

    For each robot the Euclidean distance between its current position
    (from robot_positions, already in map frame) and its assigned goal
    (from formation_points, also in map frame) is compared against
    arrival_tolerance (default 1.0 m).  Returns SUCCESS only when
    every robot is within tolerance.
    """

    def __init__(self, name="AllArrivalCheck"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("robot_positions",   access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments", access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points",  access=py_trees.common.Access.READ)
        self.blackboard.register_key("arrival_tolerance", access=py_trees.common.Access.READ)

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def update(self):
        robot_positions  = self.blackboard.robot_positions
        robot_assignments = self.blackboard.robot_assignments
        formation_points = self.blackboard.formation_points
        tolerance        = self.blackboard.arrival_tolerance  # metres

        if not robot_assignments:
            self.node.get_logger().info(f"{self.name}: No assignments yet.")
            return py_trees.common.Status.FAILURE

        for robot_name, goal_key in robot_assignments.items():
            # --- position ---
            pose = robot_positions.get(robot_name)
            if pose is None:
                self.node.get_logger().info(
                    f"{self.name}: No position for {robot_name} yet.")
                return py_trees.common.Status.FAILURE

            robot_xy = np.array([pose.pose.position.x, pose.pose.position.y])

            # --- goal ---
            goal_xy = formation_points.get(goal_key)
            if goal_xy is None:
                self.node.get_logger().info(
                    f"{self.name}: No formation point for {goal_key}.")
                return py_trees.common.Status.FAILURE

            goal_xy = np.array(goal_xy)

            # --- distance check ---
            distance = float(np.linalg.norm(robot_xy - goal_xy))
            self.node.get_logger().info(
                f"{self.name}: {robot_name} → {goal_key}  dist={distance:.2f}m  tol={tolerance:.2f}m")

            if distance > tolerance:
                self.node.get_logger().info(
                    f"{self.name}: {robot_name} has NOT arrived yet ({distance:.2f}m > {tolerance:.2f}m).")
                return py_trees.common.Status.FAILURE

        self.node.get_logger().info(
            f"{self.name}: All robots have arrived within {tolerance:.2f}m!")
        return py_trees.common.Status.SUCCESS


class LoiterWithHeadingClient(py_trees.behaviour.Behaviour):
    """Action client to call loiter_with_heading action server."""
    
    def __init__(self, name="LoiterWithHeadingClient"):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("this_robot_name",         access=py_trees.common.Access.READ)
        self.blackboard.register_key("robot_assignments",       access=py_trees.common.Access.READ)
        self.blackboard.register_key("formation_points_latlon", access=py_trees.common.Access.READ)
        self._action_client = None

    def setup(self, **kwargs):
        self.node = kwargs['node']
        self._action_client = ActionClient(self.node, BaseAction, 'loiter_heading')
        self.node.get_logger().info(f"{self.name}: Setup complete.")

    def initialise(self):
        self._send_goal_future  = None
        self._get_result_future = None
        self._goal_handle       = None
        self._goal_accepted     = False
        self._goal_done         = False
        self._goal_succeeded    = False

    def update(self):
        self.node.get_logger().info(f"{self.name}: Update begin")
        
        if self._send_goal_future is None:
            my_name     = self.blackboard.this_robot_name
            assignments = self.blackboard.robot_assignments
            
            if my_name not in assignments:
                self.node.get_logger().warn(f"{self.name}: No assignment for {my_name}")
                return py_trees.common.Status.FAILURE
            
            my_goal_key = assignments[my_name]
            
            formation_points_latlon = self.blackboard.formation_points_latlon
            if my_goal_key not in formation_points_latlon:
                self.node.get_logger().warn(f"{self.name}: No formation point data for {my_goal_key}")
                return py_trees.common.Status.FAILURE
            
            goal_data = formation_points_latlon[my_goal_key]
            heading   = goal_data.get('heading', 0.0)
            
            if not self._action_client.wait_for_server(timeout_sec=2.0):
                self.node.get_logger().warn(f"{self.name}: loiter_heading action server not available")
                return py_trees.common.Status.FAILURE
            
            goal_dict = {'duration': 400, 'heading': heading}
            goal_msg  = BaseAction.Goal()
            goal_msg.goal.data = json.dumps(goal_dict)
            
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self._feedback_cb
            )
            self._send_goal_future.add_done_callback(self._goal_response_cb)
            self.node.get_logger().info(
                f"{self.name}: Goal sent to loiter_heading (duration=400s, heading={heading}°)")
            return py_trees.common.Status.RUNNING
        
        if not self._goal_accepted:
            self.node.get_logger().info(f"{self.name}: Waiting for goal acceptance...")
            if self._goal_done:
                self.node.get_logger().info(f"{self.name}: Goal was rejected by the server")
                return py_trees.common.Status.FAILURE
            return py_trees.common.Status.RUNNING
        
        if self._goal_done:
            self.node.get_logger().info(
                f"{self.name}: Goal finished: {'SUCCEEDED' if self._goal_succeeded else 'FAILED'}")
            return (
                py_trees.common.Status.SUCCESS
                if self._goal_succeeded
                else py_trees.common.Status.FAILURE
            )
        
        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.node.get_logger().info(f"{self.name}: Terminate called with new_status={new_status}")
        if self._goal_handle is not None and not self._goal_done:
            self._goal_handle.cancel_goal_async()

    def _feedback_cb(self, feedback_msg):
        try:
            feedback_str = feedback_msg.feedback.feedback.data
            feedback = json.loads(feedback_str)
            self.node.get_logger().info(
                f"{self.name}: Feedback - Position reached: {feedback.get('position_reached', False)}, "
                f"Heading reached: {feedback.get('heading_reached', False)}, "
                f"Distance: {feedback.get('distance_from_center', 0.0):.2f}m, "
                f"Heading error: {feedback.get('heading_error', 0.0):.1f}°"
            )
        except Exception as e:
            self.node.get_logger().warn(f"{self.name}: Failed to parse feedback: {e}")

    def _goal_response_cb(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error(f"{self.name}: Goal rejected by loiter_heading server")
            self._goal_done = True
            return
        self._goal_handle   = goal_handle
        self._goal_accepted = True
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self.node.get_logger().info(f"{self.name}: Goal result received")
        result = future.result().result
        self._goal_succeeded = result.success
        self._goal_done      = True