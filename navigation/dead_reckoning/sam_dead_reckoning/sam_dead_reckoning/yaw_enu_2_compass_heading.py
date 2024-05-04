#!/usr/bin/python

# General
import math

# ROS
from rclpy.node import Node
import rclpy

# Transforms
from tf_transformations import euler_from_quaternion

# Messages
# from sbg_driver.msg import SbgEkfEuler ##
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

# Topics
from sam_msgs.msg import Topics as SamTopics
from dead_reckoning_msgs.msg import Topics as DRTopics


def yaw_enu_2_compass_heading(yaw: float) -> float:
    """
    Calculates the compass heading from the yaw.

    :param yaw: yaw in radians w.r.t. to the ENU frame
    :return: compass_heading in degrees, 0-360
    """

    # Convert input yaw to degrees
    yaw_deg = yaw * (180 / math.pi)

    # Convert yaw (ENU) to heading (NED)
    heading = 90. - yaw_deg

    # Bound to 0 - 360
    compass_heading = heading % 360.

    return compass_heading


class YawEnu2CompassHeading(Node):
    """
    This node will convert the yaw (enu) to compass heading in degrees.
    The yaw can be provided by either the SBG IMU or by dead reckoning.
    Reporting the dead reckoning is the default.
    """

    def __init__(self, namespace=None):
        super().__init__("yaw_enu_2_compass_heading", namespace=namespace)
        self.get_logger().info("Starting node defined in yaw_enu_2_compass_heading.py")

        # Set source via parameter
        self.declare_parameter("convert_dr", True)
        self.convert_dr = self.get_parameter("convert_dr").value

        # Set topics according to source
        # Input: a yaw topic
        # Output: corresponding compass heading topic
        if self.convert_dr:
            self.yaw_topic = DRTopics.DR_YAW_TOPIC
            self.compass_heading_topic = DRTopics.DR_COMPASS_HEADING_TOPIC
        else:
            self.yaw_topic = SamTopics.SBG_IMU_YAW_TOPIC
            self.compass_heading_topic = SamTopics.SBG_IMU_COMPASS_HEADING_TOPIC

        self.create_subscription(msg_type=Float64, topic=self.yaw_topic,
                                 callback=self.yaw_callback, qos_profile=10)

        self.compass_heading_pub = self.create_publisher(msg_type=Float64, topic=self.compass_heading_topic,
                                                         qos_profile=10)

    def yaw_callback(self, yaw_msg):
        yaw_enu = yaw_msg.data

        # Convert to
        compass_heading = yaw_enu_2_compass_heading(yaw_enu)

        if self.convert_dr_yaw:
            self.get_logger().info(f"DR compass heading (deg): {compass_heading}")
        else:
            self.get_logger().info(f"SBG compass heading (deg): {compass_heading}")

        # Construct messages and publish
        compass_heading_msg = Float64()
        compass_heading_msg.data = compass_heading
        self.compass_heading_pub.publish(compass_heading_msg)


def main(args=None, namespace=None):
    rclpy.init(args=args)
    compass_heading_node = YawEnu2CompassHeading(namespace=namespace)
    try:
        rclpy.spin(compass_heading_node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    default_namespace = "sam0"
    main(namespace=default_namespace)
