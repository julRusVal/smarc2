#!/usr/bin/python

# General

# ROS
from rclpy.node import Node
import rclpy

# Transforms
from tf_transformations import euler_from_quaternion

# Messages
# from sbg_driver.msg import SbgEkfEuler ##
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

# Topics
from sam_msgs.msg import Topics as SamTopics
from dead_reckoning_msgs.msg import Topics as DRTopics


class Quat2rpy(Node):
    """
    This node will convert the quaternion orientation to roll, pitch, and yaw.
    The orientation can be provided by either the SBG IMU or by dead reckoning.
    Reporting the dead reckoning is the default.
    Parameters:
        - convert_dr: Selects the DR odometery as the source to convert to rpy
        - verbose: If true, prints the roll, pitch, and yaw of the selected source
    """

    def __init__(self, namespace=None):
        super().__init__("quat_2_rpy_enu", namespace=namespace)
        self.get_logger().info("Starting node defined in quat_2_rpy_enu.py")

        # Parameters
        # Set source via parameter
        self.declare_parameter("convert_dr_quat", True)
        self.convert_dr = self.get_parameter("convert_dr_yaw").value

        # Set verboseness
        self.declare_parameter("verbose", False)
        self.verbose = self.get_parameter("verbose").value

        # Set the corresponding topics
        if self.convert_dr:
            self.sbg_topic = SamTopics.SBG_IMU_TOPIC
            self.create_subscription(msg_type=Imu, topic=self.sbg_topic,
                                     callback=self.sbg_imu_callback, qos_profile=10)

            # Output topic
            self.roll_topic = SamTopics.SBG_IMU_ROLL_TOPIC
            self.pitch_topic = SamTopics.SBG_IMU_PITCH_TOPIC
            self.yaw_topic = SamTopics.SBG_IMU_YAW_TOPIC
        else:
            self.dr_topic = DRTopics.DR_ODOM_TOPIC
            self.create_subscription(msg_type=Odometry, topic=self.dr_topic,
                                     callback=self.dr_odom_callback, qos_profile=10)

            # output topics
            self.roll_topic = DRTopics.DR_ROLL_TOPIC
            self.pitch_topic = DRTopics.DR_PITCH_TOPIC
            self.yaw_topic = DRTopics.DR_YAW_TOPIC

        # Publishers
        self.roll_pub = self.create_publisher(msg_type=Float64, topic=self.roll_topic, qos_profile=10)

        self.pitch_pub = self.create_publisher(msg_type=Float64, topic=self.pitch_topic, qos_profile=10)

        self.yaw_pub = self.create_publisher(msg_type=Float64, topic=self.yaw_topic, qos_profile=10)

    def sbg_imu_callback(self, sbg_msg):
        quat = sbg_msg.orientation
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        if self.verbose:
            self.get_logger().info(f"SBG roll: {roll:.2f} - pitch: {pitch:.2f} - yaw: {yaw:.2f}")

        self.publish_rpy(roll, pitch, yaw)

    def dr_odom_callback(self, dr_msg):
        quat = dr_msg.orientation
        euler = euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])

        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]

        if self.verbose:
            self.get_logger().info(f"SBG roll: {roll:.2f} - pitch: {pitch:.2f} - yaw: {yaw:.2f}")

        self.publish_rpy(roll, pitch, yaw)

    def publish_rpy(self, roll: float, pitch: float, yaw: float):
        # Construct messages and publish
        roll_msg = Float64()
        roll_msg.data = roll
        self.roll_pub.publish(roll_msg)

        pitch_msg = Float64()
        pitch_msg.data = pitch
        self.pitch_pub.publish(pitch_msg)

        yaw_msg = Float64()
        yaw_msg.data = yaw
        self.yaw_pub.publish(yaw_msg)


def main(args=None, namespace=None):
    rclpy.init(args=args)
    rpy_node = Quat2rpy(namespace=namespace)
    try:
        rclpy.spin(rpy_node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    default_namespace = "sam0"
    main(namespace=default_namespace)
