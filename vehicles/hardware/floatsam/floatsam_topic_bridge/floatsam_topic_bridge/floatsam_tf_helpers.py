# /vehicles/hardware/floatsam/floatsam_topic_bridge/floatsam_topic_bridge/floatsam_tf_helpers.py

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

class FloatSamTransforms:
    """Helper class for Floatsam transformation operations"""
    
    @staticmethod
    def apply_lever_arm_compensation(px4_x, px4_y, px4_z, yaw_rad, 
                                      antenna_offset_x, antenna_offset_y, antenna_offset_z):
        """
        Apply GPS antenna lever arm compensation.
        
        Args:
            px4_x, px4_y, px4_z: Position from PX4 EKF (ENU frame)
            yaw_rad: Heading in radians
            antenna_offset_x/y/z: Antenna offset in body frame (body-forward, body-right, body-down)
            
        Returns:
            (compensated_x, compensated_y, compensated_z)
        """
        # Create quaternion from yaw (rotation around Z axis in ENU)
        quat = [math.cos(yaw_rad / 2.0), 0.0, 0.0, math.sin(yaw_rad / 2.0)]  # w, x, y, z
        rot = R.from_quat(quat)
        
        # Antenna offset in body frame (NED convention from PX4)
        antenna_offset_body = [antenna_offset_y, antenna_offset_x, -antenna_offset_z]
        
        # Rotate to ENU frame
        antenna_offset_enu = rot.apply(antenna_offset_body)
        
        return (
            px4_x + antenna_offset_enu[0],
            px4_y + antenna_offset_enu[1],
            px4_z + antenna_offset_enu[2]
        )
    
    @staticmethod
    def create_static_tf_transform(stamp, parent_frame, child_frame, 
                                    offset_x, offset_y, offset_z,
                                    quat_w=1.0, quat_x=0.0, quat_y=0.0, quat_z=0.0):
        """
        Create a static TransformStamped.
        
        Args:
            stamp: ROS time stamp
            parent_frame: Parent frame ID
            child_frame: Child frame ID
            offset_x/y/z: Translation
            quat_w/x/y/z: Quaternion (default: identity)
            
        Returns:
            TransformStamped message
        """
        from geometry_msgs.msg import TransformStamped
        
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = child_frame
        
        t.transform.translation.x = float(offset_x)
        t.transform.translation.y = float(offset_y)
        t.transform.translation.z = float(offset_z)
        
        t.transform.rotation.w = float(quat_w)
        t.transform.rotation.x = float(quat_x)
        t.transform.rotation.y = float(quat_y)
        t.transform.rotation.z = float(quat_z)
        
        return t