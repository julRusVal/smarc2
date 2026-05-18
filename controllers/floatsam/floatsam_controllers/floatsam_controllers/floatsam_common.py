#!/usr/bin/python

from rclpy.node import Node
from rclpy.time import Time, Duration
import numpy as np 
from geometry_msgs.msg import PointStamped, Vector3Stamped, Twist
from tf2_geometry_msgs import do_transform_point, do_transform_vector3

from geometry_msgs.msg import PointStamped, PoseStamped, TransformStamped, Twist, Quaternion
from geographic_msgs.msg import GeoPoint
from nav_msgs.msg import Odometry

from tf2_geometry_msgs import do_transform_pose_stamped
from tf2_ros import Buffer, TransformListener

from smarc_utilities.georef_utils import convert_latlon_to_utm, convert_utm_to_latlon


class FloatSam():
    def __init__(self,
                 node: Node,
                 robot_name: str,
                 use_sim: bool = False):
        
        self._node : Node = node
        self._floatsam_in_map : None | PoseStamped = None
        self.use_sim = use_sim


        if self.use_sim:
            self.GLOBAL_MAP_FRAME: str = 'unity_origin' 
            self.LOCAL_MAP_FRAME: str  = 'unity_origin' 
        else:
            self.GLOBAL_MAP_FRAME: str = 'map'
            self.LOCAL_MAP_FRAME: str  = "map" 

        self._tf_buffer : Buffer = Buffer()
        self._tf_listener : TransformListener = TransformListener(self._tf_buffer, self._node, spin_thread=True)

        self.robot_name = robot_name
        odom_topic = f"/{robot_name}/smarc/odom"
        self._node.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self._node.get_logger().info(f"[FloatSam] Subscribed to odometry: {odom_topic}")
        
    def _odom_cb(self, msg_odom: Odometry):
        """Keep this specific robot's pose updated in its LOCAL map for standard behaviors."""
        floatsam_in_odom = PoseStamped()
        floatsam_in_odom.header = msg_odom.header
        floatsam_in_odom.pose = msg_odom.pose.pose
        
        try:
            # LIVE LOOKUP: Source -> Local Map
            odom_to_map_tf = self._tf_buffer.lookup_transform(
                self.LOCAL_MAP_FRAME, 
                msg_odom.header.frame_id, 
                Time() 
            )
            self._floatsam_in_map = do_transform_pose_stamped(floatsam_in_odom, odom_to_map_tf)
        except Exception as e:
            self._node.get_logger().warn(
                f"Waiting for TF: {msg_odom.header.frame_id} -> {self.LOCAL_MAP_FRAME}...", 
                throttle_duration_sec=2.0
            )

    @property
    def floatsam_in_map(self) -> PoseStamped|None:
        return self._floatsam_in_map    

    def convert_geopoint_to_map_pose_stamped(self, gp: GeoPoint) -> PoseStamped:
        in_utm : PointStamped = convert_latlon_to_utm(gp)
        in_utm_pose : PoseStamped = PoseStamped()
        in_utm_pose.header = in_utm.header
        in_utm_pose.pose.position = in_utm.point
        in_utm_pose.pose.position.z = gp.altitude  

        source_frame = in_utm.header.frame_id
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=self.LOCAL_MAP_FRAME,
                source_frame=source_frame,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
        except Exception as e:
            try:
                tf = self._tf_buffer.lookup_transform(
                    target_frame=self.LOCAL_MAP_FRAME,
                    source_frame='utm_34_V',
                    time=Time(seconds=0),
                    timeout=Duration(seconds=1)
                )
            except Exception as e2:
                err_msg = (
                    f"Failed to find a transform from any UTM frame to '{self.LOCAL_MAP_FRAME}'. "
                    f"Tried '{source_frame}' and 'utm'."
                )
                self._node.get_logger().error(err_msg)
                raise

        in_map = do_transform_pose_stamped(in_utm_pose, tf)
        in_map.pose.position.z = gp.altitude  
        return in_map

    def convert_map_point_to_geopoint(self, x: float, y: float, z: float = 0.0) -> GeoPoint:
        in_map = PoseStamped()
        in_map.header.frame_id = self.LOCAL_MAP_FRAME
        in_map.pose.position.x = float(x)
        in_map.pose.position.y = float(y)
        in_map.pose.position.z = float(z)

        if not hasattr(self, '_utm_frame_cache') or self._utm_frame_cache is None:
            candidates = ['utm_33_V', 'utm', 'utm_34_V'] + [f'utm_{i}' for i in range(1, 61)]
            source_frame = None
            for candidate in candidates:
                try:
                    self._tf_buffer.lookup_transform(
                        target_frame=candidate,
                        source_frame=self.LOCAL_MAP_FRAME,
                        time=Time(seconds=0),
                        timeout=Duration(seconds=0)
                    )
                    source_frame = candidate
                    break  
                except Exception:
                    continue
            if source_frame is None:
                raise RuntimeError(f"Could not find a TF from '{self.LOCAL_MAP_FRAME}' to any valid UTM frame.")
            self._utm_frame_cache = source_frame

        try:
            tf_inv = self._tf_buffer.lookup_transform(
                target_frame=self._utm_frame_cache,
                source_frame=self.LOCAL_MAP_FRAME,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
        except Exception as e:
            self._utm_frame_cache = None  
            raise RuntimeError(f"Failed to transform map to {self._utm_frame_cache}: {e}")

        in_utm = do_transform_pose_stamped(in_map, tf_inv)
        in_utm.header.frame_id = self._utm_frame_cache
        return convert_utm_to_latlon(in_utm)


    def convert_odom_point_to_geopoint(self, x: float, y: float, z: float = 0.0) -> GeoPoint:
        in_odom = PoseStamped()
        if self.use_sim:
            in_odom.header.frame_id = "unity_origin"  
        else:
            in_odom.header.frame_id = f"{self.robot_name}/odom"  
        in_odom.pose.position.x = float(x)
        in_odom.pose.position.y = float(y)
        in_odom.pose.position.z = float(z)

        try:
            odom_to_map_tf = self._tf_buffer.lookup_transform(
                target_frame=self.LOCAL_MAP_FRAME,
                source_frame=in_odom.header.frame_id,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
        except Exception as e:
            raise RuntimeError(
                f"Failed to transform '{in_odom.header.frame_id}' -> '{self.LOCAL_MAP_FRAME}': {e}"
            )

        in_map = do_transform_pose_stamped(in_odom, odom_to_map_tf)
        in_map.header.frame_id = self.LOCAL_MAP_FRAME

        # Then reuse map -> geopoint logic
        return self.convert_map_point_to_geopoint(
            in_map.pose.position.x,
            in_map.pose.position.y,
            in_map.pose.position.z
        )
      

    def convert_body_to_map_twist(self, twist_body: Twist, orientation: Quaternion) -> Twist:
        """
        Converts velocity from the local body frame (e.g., base_link) 
        to the global map frame (e.g., unity_origin).
        
        :param twist_body: The Twist message containing surge/sway velocities.
        :param orientation: The pose quaternion [x, y, z, w] representing the vehicle's current attitude.
        """
        # Extract linear velocity vector (Surge, Sway, Heave)
        v_body = np.array([twist_body.linear.x, twist_body.linear.y, twist_body.linear.z])
        
        # Extract quaternion
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        
        # Apply forward rotation: Body -> Map
        rotation = R.from_quat(quat)
        v_map = rotation.apply(v_body)
        
        twist_map = Twist()
        twist_map.linear.x = v_map[0]  # Global X (East)
        twist_map.linear.y = v_map[1]  # Global Y (North)
        twist_map.linear.z = v_map[2]  # Global Z (Up)
        
        # Angular velocity (roll/pitch/yaw rates) is typically treated as frame-independent
        # when dealing with basic kinematics, so we pass it through directly.
        twist_map.angular = twist_body.angular
        
        return twist_map

    def convert_map_to_body_twist(self, twist_map: Twist, orientation: Quaternion) -> Twist:
        """
        Converts velocity from the global map frame (e.g., unity_origin) 
        back into the local body frame (e.g., base_link).
        """
        # Extract linear velocity vector (East, North, Up)
        v_map = np.array([twist_map.linear.x, twist_map.linear.y, twist_map.linear.z])
        
        # Extract quaternion
        quat = [orientation.x, orientation.y, orientation.z, orientation.w]
        
    
        rotation = R.from_quat(quat)
        v_body = rotation.inv().apply(v_map)
        
        twist_body = Twist()
        twist_body.linear.x = v_body[0]  
        twist_body.linear.y = v_body[1] 
        twist_body.linear.z = v_body[2]  
        
        twist_body.angular = twist_map.angular
        
        return twist_body

    def convert_odom_point_to_map_point(self, x: float, y: float, z: float = 0.0) -> PointStamped:
        """Transforms a coordinate from the Odom frame to the Local Map frame.
        If TF tree is not ready, returns point in original frame until it becomes available."""
        in_odom = PointStamped()
        in_odom.header.frame_id = self.GLOBAL_MAP_FRAME if self.use_sim else f"{self.robot_name}/odom"
        in_odom.point.x = float(x)
        in_odom.point.y = float(y)
        in_odom.point.z = float(z)

        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=self.LOCAL_MAP_FRAME,
                source_frame=in_odom.header.frame_id,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
            return do_transform_point(in_odom, tf)
        except Exception as e:
            # TF tree not ready yet; return point in original frame as fallback
            self._node.get_logger().debug(f"TF lookup not ready (Odom->Map): {e}", throttle_duration_sec=5.0)
            return in_odom
    def convert_map_point_to_odom_point(self, x: float, y: float, z: float = 0.0) -> PointStamped:
        """Transforms a coordinate from the Local Map frame to the Odom frame.
        If TF tree is not ready, returns point in original frame until it becomes available."""
        in_map = PointStamped()
        in_map.header.frame_id = self.LOCAL_MAP_FRAME
        in_map.point.x = float(x)
        in_map.point.y = float(y)
        in_map.point.z = float(z)

        target_odom_frame = self.GLOBAL_MAP_FRAME if self.use_sim else f"{self.robot_name}/odom"

        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=target_odom_frame,
                source_frame=in_map.header.frame_id,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
            return do_transform_point(in_map, tf)
        except Exception as e:
            # TF tree not ready yet; return point in original frame as fallback
            self._node.get_logger().debug(f"TF lookup not ready (Map->Odom): {e}", throttle_duration_sec=5.0)
            return in_map

    def convert_point_frame_to_frame(self, point_x: float, point_y: float, point_z: float, 
                                     source_frame: str, target_frame: str) -> PointStamped:
        """
        Generalized point transformation between any two frames using TF2.
        If TF tree is not ready, returns point in original frame until it becomes available.
        """
        in_point = PointStamped()
        in_point.header.frame_id = source_frame
        in_point.point.x = float(point_x)
        in_point.point.y = float(point_y)
        in_point.point.z = float(point_z)

        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
            return do_transform_point(in_point, tf)
        except Exception as e:
            # TF tree not ready yet; return point in original frame as fallback
            self._node.get_logger().debug(f"TF lookup not ready (Point {source_frame}->{target_frame}): {e}", throttle_duration_sec=5.0)
            return in_point

    def convert_twist_frame_to_frame(self, twist_in: Twist, source_frame: str, target_frame: str) -> Twist:
        """
        Generalized velocity transformation using TF2. 
        Because velocities are vectors (not points), TF2 will cleanly apply 
        ONLY the rotation from the tree, ignoring RTK map translations!
        If TF tree is not ready, returns twist unchanged until it becomes available.
        """
        try:
            # Get the latest transform between the frames
            tf = self._tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=source_frame,
                time=Time(seconds=0),
                timeout=Duration(seconds=1)
            )
        except Exception as e:
            # TF tree not ready yet; return twist unchanged as fallback
            self._node.get_logger().debug(f"TF lookup not ready (Twist {source_frame}->{target_frame}): {e}", throttle_duration_sec=5.0)
            return twist_in

        lin_vec = Vector3Stamped()
        lin_vec.vector = twist_in.linear
        lin_rotated = do_transform_vector3(lin_vec, tf)

        ang_vec = Vector3Stamped()
        ang_vec.vector = twist_in.angular
        ang_rotated = do_transform_vector3(ang_vec, tf)

        twist_out = Twist()
        twist_out.linear = lin_rotated.vector
        twist_out.angular = ang_rotated.vector
        
        return twist_out

    def convert_body_to_map_twist(self, twist_body: Twist) -> Twist:
        source = f"{self.robot_name}/base_link"
        target = self.LOCAL_MAP_FRAME
        return self.convert_twist_frame_to_frame(twist_body, source, target)

    def convert_map_to_body_twist(self, twist_map: Twist) -> Twist:
        source = self.LOCAL_MAP_FRAME
        target = f"{self.robot_name}/base_link"
        return self.convert_twist_frame_to_frame(twist_map, source, target)