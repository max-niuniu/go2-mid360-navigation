#!/usr/bin/env python3
"""Expose FAST-LIO odometry as Nav2's planar odom interface.

This node deliberately has no Gazebo input.  It derives ``/odom`` and the
``odom -> base_footprint`` TF only from FAST-LIO's ``/Odometry`` estimate.
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def yaw_from_quaternion(quaternion):
    """Return the planar yaw component of a ROS quaternion."""
    sin_yaw = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cos_yaw = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(sin_yaw, cos_yaw)


class FastLioOdomBridge(Node):
    def __init__(self):
        super().__init__("fast_lio_odom_bridge")

        self.declare_parameter("source_topic", "/Odometry")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_footprint")

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        source_topic = self.get_parameter("source_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.publisher = self.create_publisher(Odometry, odom_topic, 20)
        self.broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, source_topic, self.on_odometry, 20)

        self.get_logger().info(
            f"Bridging {source_topic} to {odom_topic} and "
            f"{self.odom_frame} -> {self.base_frame} (no Gazebo truth)"
        )

    def on_odometry(self, source):
        yaw = yaw_from_quaternion(source.pose.pose.orientation)
        half_yaw = 0.5 * yaw

        odom = Odometry()
        odom.header = source.header
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = source.pose.pose.position.x
        odom.pose.pose.position.y = source.pose.pose.position.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = math.sin(half_yaw)
        odom.pose.pose.orientation.w = math.cos(half_yaw)
        odom.twist = source.twist

        # FAST-LIO's covariance is not always populated.  Give Nav2 a finite,
        # conservative planar covariance rather than forwarding all-zero data.
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.08
        self.publisher.publish(odom)

        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = odom.pose.pose.position.x
        transform.transform.translation.y = odom.pose.pose.position.y
        transform.transform.rotation.z = odom.pose.pose.orientation.z
        transform.transform.rotation.w = odom.pose.pose.orientation.w
        self.broadcaster.sendTransform(transform)


def main():
    rclpy.init()
    node = FastLioOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
