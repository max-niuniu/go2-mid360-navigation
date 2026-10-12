#!/usr/bin/env python3
"""Wait for localization/TF or Nav2 activation before advancing bringup."""
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from std_srvs.srv import Trigger


class ReadinessWaiter(Node):
    def __init__(self):
        super().__init__("wait_for_ready")
        self.declare_parameter("timeout_sec", 120.0)
        self.declare_parameter("stage", "localization")
        self.stage = self.get_parameter("stage").value
        if self.stage not in ("localization", "navigation"):
            raise ValueError("stage must be localization or navigation")
        self.lifecycle_clients = []
        self.futures = {}
        if self.stage == "navigation":
            self.lifecycle_clients = [self.create_client(Trigger, f"/{name}/is_active")
                            for name in ("lifecycle_manager_map", "lifecycle_manager_navigation")]
        self.buffer = Buffer(node=self)
        self.listener = TransformListener(self.buffer, self)
        self.pose = None
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped, "/localization/pose", self.on_pose, 10
        )

    def on_pose(self, message):
        self.pose = message

    def ready(self):
        if self.stage == "navigation":
            for client in self.lifecycle_clients:
                future = self.futures.get(client)
                if future is None and client.service_is_ready():
                    self.futures[client] = client.call_async(Trigger.Request())
                elif future is not None and future.done():
                    if future.exception() is not None or not future.result().success:
                        del self.futures[client]
            return all(
                client in self.futures and self.futures[client].done()
                and self.futures[client].exception() is None
                and self.futures[client].result().success for client in self.lifecycle_clients
            )
        now = self.get_clock().now().nanoseconds / 1e9
        if now <= 0 or self.pose is None or self.pose.header.frame_id != "map":
            return False
        stamps = [self.pose.header.stamp]
        try:
            for target, source in [("map", "odom"), ("odom", "base_footprint"),
                                   ("map", "base_footprint")]:
                transform = self.buffer.lookup_transform(target, source, rclpy.time.Time())
                stamps.append(transform.header.stamp)
        except TransformException:
            return False
        # Reject stale messages, including when simulation time moves backwards.
        return all(-0.2 <= now - (s.sec + s.nanosec / 1e9) <= 2.0 for s in stamps)


def main():
    rclpy.init()
    node = ReadinessWaiter()
    deadline = time.monotonic() + node.get_parameter("timeout_sec").value
    ready_since = None
    label = "Localization and TF" if node.stage == "localization" else "Nav2 lifecycle managers"
    node.get_logger().info(f"Waiting for {label}...")
    result = 1
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            if node.ready():
                if ready_since is None:
                    ready_since = now
                if now - ready_since >= 1.0:
                    node.get_logger().info(f"{label} are ready.")
                    result = 0
                    break
            else:
                ready_since = None
        if rclpy.ok() and result:
            node.get_logger().error(f"Timed out waiting for {label}; bringup was not advanced.")
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
