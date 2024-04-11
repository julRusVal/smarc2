#!/usr/bin/python

import numpy as np
from geodesy import utm

import rclpy
from rclpy import time
from rclpy.node import Node
import message_filters

import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

from geometry_msgs.msg import Quaternion, TransformStamped
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

try:
    from .helpers.ros_helpers import rcl_time_to_secs, rcl_time_to_stamp, ros_time_to_secs
except ImportError:
    from helpers.ros_helpers import rcl_time_to_secs, rcl_time_to_stamp, ros_time_to_secs


class PublishGPSPose(Node):

    def __init__(self):
        super().__init__("gps_node")

        # ===== Declare parameters =====
        self.declare_node_parameters()

        # ===== Get parameters =====
        self.gps_topic = self.get_parameter('gps_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.utm_frame = self.get_parameter('utm_frame').value
        self.gps_frame = self.get_parameter('gps_frame').value
        self.gps_odom_top = self.get_parameter('gps_odom_topic').value

        # GPS odom in UTM frame
        # ROS1
        # self.gps_sam_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.sam_gps_cb)
        # self.gps_sam_pub = rospy.Publisher(self.gps_odom_top, Odometry, queue_size=10)

        self.gps_sam_sub = self.create_subscription(msg_type=NavSatFix, topic=self.gps_topic,
                                                    callback=self.sam_gps_cb, qos_profile=10)

        self.gps_sam_pub = self.create_publisher(msg_type=Odometry, topic=self.gps_odom_top,
                                                 qos_profile=10)

        # Broadcast UTM to map frame
        self.tf_buffer = Buffer()
        self.listener = TransformListener(self.tf_buffer, self)
        self.static_tf_bc = tf2_ros.StaticTransformBroadcaster(self)

        # Auxiliar subs and pubs for floatsam
        # ROS1
        # self.gps_prt_sub = message_filters.Subscriber("/sam/core/gps/prt", NavSatFix)
        # self.gps_stb_sub = message_filters.Subscriber("/sam/core/gps/stb", NavSatFix)

        self.gps_prt_sub = message_filters.Subscriber(self, NavSatFix, "/sam/core/gps/prt")
        self.gps_stb_sub = message_filters.Subscriber(self, NavSatFix, "/sam/core/gps/stb")
        self.ts = message_filters.ApproximateTimeSynchronizer([self.gps_prt_sub, self.gps_stb_sub],
                                                              20, slop=20.0, allow_headerless=False)
        self.ts.registerCallback(self.gps_callback)

        # ROS1
        # self.odom_pub = rospy.Publisher('gps_odom', Odometry, queue_size=10)
        # self.gps_prt_pub = rospy.Publisher('gps_odom_prt', Odometry, queue_size=10)
        # self.gps_stb_pub = rospy.Publisher('gps_odom_stb', Odometry, queue_size=10)

        self.odom_pub = self.create_publisher(msg_type=Odometry, topic='gps_odom', qos_profile=10)
        self.gps_prt_pub = self.create_publisher(msg_type=Odometry, topic='gps_odom_prt', qos_profile=10)
        self.gps_stb_pub = self.create_publisher(msg_type=Odometry, topic='gps_odom_stb', qos_profile=10)

    def declare_node_parameters(self):
        """
        Declare the parameters of the node
        """

        """
        self.gps_topic = rospy.get_param('~gps_topic', '/sam/core/gps')
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.utm_frame = rospy.get_param('~utm_frame', 'utm')
        self.gps_frame = rospy.get_param('~gps_frame', 'sam/gps_link')
        self.gps_odom_top = rospy.get_param('~gps_odom_topic', 'gps_odom_sam')
        
        """
        default_robot_name = 'sam0'
        self.declare_parameter('gps_topic', f'/{default_robot_name}/core/gps')
        self.declare_parameter('gps_topic', f'/{default_robot_name}/core/gps')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('utm_frame', 'utm')
        self.declare_parameter('gps_frame', f'{default_robot_name}/gps_link')
        self.declare_parameter('gps_odom_topic', 'gps_odom_sam')

    def sam_gps_cb(self, sam_gps):

        if sam_gps.status.status != -1:

            utm_sam = utm.fromLatLong(sam_gps.latitude, sam_gps.longitude)
            rot = [0., 0., 0., 1.]

            try:
                # TODO check that the frame order is correct
                # Not sure what the argument order is in ros 1
                # ROS1
                # (world_trans, world_rot) = self.listener.lookupTransform(self.utm_frame,
                #                                                          self.map_frame,
                #                                                          rospy.Time(0))

                (world_trans, world_rot) = self.tf_buffer.lookup_transform(target_frame=self.map_frame,
                                                                           source_frame=self.utm_frame,
                                                                           time=rclpy.time.Time())

            except (LookupException, ConnectivityException):
                self.get_logger().info(f"GPS node: broadcasting transform {self.utm_frame} to {self.map_frame}")
                transformStamped = TransformStamped()
                transformStamped.transform.translation.x = utm_sam.easting
                transformStamped.transform.translation.y = utm_sam.northing
                transformStamped.transform.translation.z = 0.
                transformStamped.transform.rotation.x = rot[0]
                transformStamped.transform.rotation.y = rot[1]
                transformStamped.transform.rotation.z = rot[2]
                transformStamped.transform.rotation.w = rot[3]
                transformStamped.header.frame_id = self.utm_frame
                transformStamped.child_frame_id = self.map_frame
                transformStamped.header.stamp = rcl_time_to_stamp(self.get_clock().now())
                self.static_tf_bc.sendTransform(transformStamped)

                return

            # For SAM GPS
            odom_msg = Odometry()
            odom_msg.header.stamp = rcl_time_to_stamp(self.get_clock().now())
            odom_msg.header.frame_id = self.utm_frame
            odom_msg.child_frame_id = self.gps_frame
            odom_msg.pose.covariance = [0.] * 36
            odom_msg.pose.pose.position.x = utm_sam.easting
            odom_msg.pose.pose.position.y = utm_sam.northing
            odom_msg.pose.pose.position.z = 0.
            odom_msg.pose.pose.orientation.x = rot[0]
            odom_msg.pose.pose.orientation.y = rot[1]
            odom_msg.pose.pose.orientation.z = rot[2]
            odom_msg.pose.pose.orientation.w = rot[3]
            self.gps_sam_pub.publish(odom_msg)

    def gps_callback(self, prt_msg, stb_msg):

        if prt_msg.status.status == -1:
            return

        # lat long to UTM
        utm_prt = utm.fromLatLong(prt_msg.latitude, prt_msg.longitude)
        utm_stb = utm.fromLatLong(stb_msg.latitude, stb_msg.longitude)
        # prt - stb
        diff = np.array([utm_prt.northing, utm_prt.easting]) - np.array([utm_stb.northing, utm_stb.easting])
        # mid point
        utm_mid = diff / 2. + np.array([utm_stb.northing, utm_stb.easting])
        heading = np.arctan2(diff[1], diff[0]) - np.pi / 2.0

        # TODO: flip easting northing where required :D
        #### Auxiliar ones for floatsam
        rot = [0., 0., 0., 1.]
        odom_msg = Odometry()
        odom_msg.header.stamp = rcl_time_to_stamp(self.get_clock().now())
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.child_frame_id = self.gps_frame
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.x = utm_mid[0]
        odom_msg.pose.pose.position.y = utm_mid[1]
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.pose.orientation.x = rot[0]
        odom_msg.pose.pose.orientation.y = rot[1]
        odom_msg.pose.pose.orientation.z = rot[2]
        odom_msg.pose.pose.orientation.w = rot[3]
        self.odom_pub.publish(odom_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = rcl_time_to_stamp(self.get_clock().now())
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.child_frame_id = self.gps_frame
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.x = utm_prt.northing
        odom_msg.pose.pose.position.y = utm_prt.easting
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.pose.orientation.x = rot[0]
        odom_msg.pose.pose.orientation.y = rot[1]
        odom_msg.pose.pose.orientation.z = rot[2]
        odom_msg.pose.pose.orientation.w = rot[3]
        self.gps_prt_pub.publish(odom_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = rcl_time_to_stamp(self.get_clock().now())
        odom_msg.header.frame_id = self.utm_frame
        odom_msg.child_frame_id = self.gps_frame
        odom_msg.pose.covariance = [0.] * 36
        odom_msg.pose.pose.position.x = utm_stb.northing
        odom_msg.pose.pose.position.y = utm_stb.easting
        odom_msg.pose.pose.position.z = 0.
        odom_msg.pose.pose.orientation.x = rot[0]
        odom_msg.pose.pose.orientation.y = rot[1]
        odom_msg.pose.pose.orientation.z = rot[2]
        odom_msg.pose.pose.orientation.w = rot[3]
        self.gps_stb_pub.publish(odom_msg)


def main(args=None):
    rclpy.init(args=args)
    gps_node = PublishGPSPose()
    try:
        rclpy.spin(gps_node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
