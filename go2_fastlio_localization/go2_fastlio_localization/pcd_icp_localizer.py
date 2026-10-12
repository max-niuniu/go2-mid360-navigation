#!/usr/bin/env python3
"""Relocalize FAST-LIO registered clouds in an existing PCD map."""

import math
import os
import time
from collections import deque

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster


def load_binary_pcd(path):
    header = {}

    with open(path, "rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise RuntimeError("PCD header is incomplete")

            text = line.decode("ascii").strip()
            if not text or text.startswith("#"):
                continue

            parts = text.split()
            header[parts[0].upper()] = parts[1:]

            if parts[0].upper() == "DATA":
                break

        if header["DATA"][0].lower() != "binary":
            raise RuntimeError("Only binary PCD files are supported")

        fields = header["FIELDS"]
        sizes = [int(value) for value in header["SIZE"]]
        types = header["TYPE"]
        counts = [int(value) for value in header.get("COUNT", ["1"] * len(fields))]
        point_count = int(header["POINTS"][0])

        format_map = {
            ("F", 4): "<f4",
            ("F", 8): "<f8",
            ("I", 1): "<i1",
            ("I", 2): "<i2",
            ("I", 4): "<i4",
            ("U", 1): "<u1",
            ("U", 2): "<u2",
            ("U", 4): "<u4",
        }

        offsets = []
        formats = []
        offset = 0

        for field_type, field_size, count in zip(types, sizes, counts):
            offsets.append(offset)
            base_format = format_map[(field_type, field_size)]
            formats.append(base_format if count == 1 else (base_format, count))
            offset += field_size * count

        dtype = np.dtype(
            {
                "names": fields,
                "formats": formats,
                "offsets": offsets,
                "itemsize": offset,
            }
        )

        raw = stream.read(point_count * offset)
        cloud = np.frombuffer(raw, dtype=dtype, count=point_count)

    points = np.column_stack(
        (
            cloud["x"].astype(np.float64),
            cloud["y"].astype(np.float64),
            cloud["z"].astype(np.float64),
        )
    )

    return points[np.isfinite(points).all(axis=1)]


def cloud_to_numpy(message):
    offsets = {field.name: field.offset for field in message.fields}

    if not all(name in offsets for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)

    endian = ">" if message.is_bigendian else "<"
    count = message.width * message.height

    dtype = np.dtype(
        {
            "names": ["x", "y", "z"],
            "formats": [endian + "f4"] * 3,
            "offsets": [offsets["x"], offsets["y"], offsets["z"]],
            "itemsize": message.point_step,
        }
    )

    cloud = np.frombuffer(message.data, dtype=dtype, count=count)

    points = np.column_stack(
        (
            cloud["x"].astype(np.float64),
            cloud["y"].astype(np.float64),
            cloud["z"].astype(np.float64),
        )
    )

    return points[np.isfinite(points).all(axis=1)]


def voxel_downsample(points, voxel_size):
    if len(points) == 0:
        return points

    keys = np.floor(points / voxel_size).astype(np.int32)
    _, indices = np.unique(keys, axis=0, return_index=True)
    return points[indices]


def pose_to_matrix(pose):
    transform = np.eye(4)
    transform[:3, :3] = Rotation.from_quat(
        [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ]
    ).as_matrix()

    transform[:3, 3] = [
        pose.position.x,
        pose.position.y,
        pose.position.z,
    ]

    return transform


def transform_points(points, transform):
    return points @ transform[:3, :3].T + transform[:3, 3]


def rigid_transform(source, target):
    source_center = source.mean(axis=0)
    target_center = target.mean(axis=0)

    source_zero = source - source_center
    target_zero = target - target_center

    u, _, vt = np.linalg.svd(source_zero.T @ target_zero)
    rotation = vt.T @ u.T

    if np.linalg.det(rotation) < 0.0:
        vt[-1, :] *= -1.0
        rotation = vt.T @ u.T

    translation = target_center - rotation @ source_center

    result = np.eye(4)
    result[:3, :3] = rotation
    result[:3, 3] = translation
    return result


def run_icp(source, target_tree, initial_transform, max_distance):
    transform = initial_transform.copy()
    inlier_count = 0
    rmse = math.inf

    for _ in range(35):
        transformed = transform_points(source, transform)
        distances, indices = target_tree.query(
            transformed,
            distance_upper_bound=max_distance,
        )

        valid = np.isfinite(distances)

        if np.count_nonzero(valid) < 60:
            return transform, math.inf, int(np.count_nonzero(valid))

        source_matches = transformed[valid]
        target_matches = target_tree.data[indices[valid]]

        delta = rigid_transform(source_matches, target_matches)
        transform = delta @ transform

        inlier_count = len(source_matches)
        rmse = float(np.sqrt(np.mean(distances[valid] ** 2)))

        translation_change = np.linalg.norm(delta[:3, 3])
        rotation_change = Rotation.from_matrix(delta[:3, :3]).magnitude()

        if translation_change < 0.001 and rotation_change < 0.001:
            break

    return transform, rmse, inlier_count


class PcdIcpLocalizer(Node):
    def __init__(self):
        super().__init__("pcd_icp_localizer")

        self.declare_parameter(
            "map_file",
            os.path.join(
                get_package_share_directory("go2_fastlio_localization"),
                "..", "..", "..", "..",
                "maps", "go2_workshop", "map.pcd",
            ),
        )
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("local_frame", "camera_init")
        self.declare_parameter("body_frame", "body")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("map_voxel_size", 0.15)
        self.declare_parameter("scan_voxel_size", 0.10)
        self.declare_parameter("max_correspondence_distance", 1.0)
        self.declare_parameter("localization_period", 1.0)
        self.declare_parameter("maximum_accepted_rmse", 0.45)
        self.declare_parameter("scan_history_size", 15)
        self.declare_parameter("auto_initialize", False)
        self.declare_parameter("auto_initialize_delay", 10.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.map_frame = self.get_parameter("map_frame").value
        self.local_frame = self.get_parameter("local_frame").value
        self.body_frame = self.get_parameter("body_frame").value
        self.odom_frame = self.get_parameter("odom_frame").value

        map_file = self.get_parameter("map_file").value
        map_voxel = float(self.get_parameter("map_voxel_size").value)

        self.scan_voxel = float(
            self.get_parameter("scan_voxel_size").value
        )
        self.max_distance = float(
            self.get_parameter("max_correspondence_distance").value
        )
        self.localization_period = float(
            self.get_parameter("localization_period").value
        )
        self.maximum_rmse = float(
            self.get_parameter("maximum_accepted_rmse").value
        )
        self.scan_history = deque(
            maxlen=int(self.get_parameter("scan_history_size").value)
        )
        self.auto_initial_pending = bool(
            self.get_parameter("auto_initialize").value
        )
        self.auto_initial_not_before = time.monotonic() + float(
            self.get_parameter("auto_initialize_delay").value
        )
        self.initial_x = float(self.get_parameter("initial_x").value)
        self.initial_y = float(self.get_parameter("initial_y").value)
        self.initial_yaw = float(
            self.get_parameter("initial_yaw").value
        )

        self.get_logger().info(f"Loading PCD map: {map_file}")
        self.map_points = voxel_downsample(
            load_binary_pcd(map_file),
            map_voxel,
        )
        self.map_tree = cKDTree(self.map_points)

        self.get_logger().info(
            f"Loaded {len(self.map_points)} downsampled map points"
        )

        self.map_to_local = None
        self.local_to_body = None
        self.odom_to_footprint = None
        self.footprint_to_base = None
        self.map_to_odom = None
        self.bridge_ready_reported = False
        self.last_localization_time = 0.0

        map_qos = QoSProfile(depth=1)
        map_qos.reliability = ReliabilityPolicy.RELIABLE
        map_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.map_publisher = self.create_publisher(
            PointCloud2,
            "/localization/map_cloud",
            map_qos,
        )
        self.aligned_publisher = self.create_publisher(
            PointCloud2,
            "/localization/aligned_cloud",
            qos_profile_sensor_data,
        )
        self.pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            "/localization/pose",
            10,
        )

        self.create_subscription(
            Odometry,
            "/Odometry",
            self.on_odometry,
            20,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self.on_navigation_odometry,
            20,
        )
        self.create_subscription(
            Odometry,
            "/odom/local",
            self.on_base_odometry,
            20,
        )
        self.create_subscription(
            PointCloud2,
            "/cloud_registered",
            self.on_cloud,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/initialpose",
            self.on_initial_pose,
            10,
        )

        self.broadcaster = TransformBroadcaster(self)
        self.create_timer(0.05, self.broadcast_transform)
        self.create_timer(2.0, self.publish_map)
        self.create_timer(0.5, self.try_auto_initialize)

        if self.auto_initial_pending:
            self.get_logger().info(
                "Waiting for /Odometry before automatic initial pose"
            )
        else:
            self.get_logger().info(
                "Waiting for /Odometry and RViz /initialpose"
            )

    def on_odometry(self, message):
        self.local_to_body = pose_to_matrix(message.pose.pose)

    def on_navigation_odometry(self, message):
        """Store odom -> base_footprint from the FAST-LIO odom bridge."""
        self.odom_to_footprint = pose_to_matrix(message.pose.pose)

    def on_base_odometry(self, message):
        """Store base_footprint -> base_link, including the body height."""
        self.footprint_to_base = pose_to_matrix(message.pose.pose)

    def try_auto_initialize(self):
        """Seed ICP once FAST-LIO odometry is ready.

        The configured pose is a map-frame estimate, not Gazebo ground truth.
        ICP still corrects it against the live registered point cloud.
        """
        if (
            not self.auto_initial_pending
            or self.local_to_body is None
            or time.monotonic() < self.auto_initial_not_before
        ):
            return

        message = PoseWithCovarianceStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.map_frame
        message.pose.pose.position.x = self.initial_x
        message.pose.pose.position.y = self.initial_y
        message.pose.pose.orientation.z = math.sin(0.5 * self.initial_yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * self.initial_yaw)

        self.auto_initial_pending = False
        self.on_initial_pose(message)
        self.get_logger().info(
            "Automatic initial pose sent: "
            f"x={self.initial_x:.3f}, y={self.initial_y:.3f}, "
            f"yaw={self.initial_yaw:.3f} rad"
        )

    def on_initial_pose(self, message):
        if self.local_to_body is None:
            self.get_logger().warning(
                "Cannot initialize: /Odometry has not arrived"
            )
            return

        self.auto_initial_pending = False
        map_to_body_guess = pose_to_matrix(message.pose.pose)
        self.map_to_local = map_to_body_guess @ np.linalg.inv(
            self.local_to_body
        )
        self.scan_history.clear()

        self.get_logger().info(
            "Initial pose received; ICP localization enabled"
        )

    def on_cloud(self, message):
        if self.map_to_local is None or self.local_to_body is None:
            return

        current_scan = cloud_to_numpy(message)
        if len(current_scan) > 0:
            self.scan_history.append(current_scan)

        now = time.monotonic()
        if now - self.last_localization_time < self.localization_period:
            return
        self.last_localization_time = now

        if not self.scan_history:
            return

        source = voxel_downsample(
            np.concatenate(tuple(self.scan_history), axis=0),
            self.scan_voxel,
        )

        if len(source) < 60:
            self.get_logger().warning(
                f"Too few scan points: {len(source)}"
            )
            return

        result, rmse, inliers = run_icp(
            source,
            self.map_tree,
            self.map_to_local,
            self.max_distance,
        )

        if not math.isfinite(rmse) or rmse > self.maximum_rmse:
            self.get_logger().warning(
                f"ICP rejected: rmse={rmse:.3f}, inliers={inliers}"
            )
            return

        self.map_to_local = result
        self.update_map_to_odom()

        aligned = transform_points(source, self.map_to_local)
        header = Header()
        header.stamp = message.header.stamp
        header.frame_id = self.map_frame

        aligned_message = point_cloud2.create_cloud_xyz32(
            header,
            aligned.astype(np.float32),
        )
        self.aligned_publisher.publish(aligned_message)

        self.publish_body_pose(message.header.stamp)

        self.get_logger().info(
            f"ICP accepted: rmse={rmse:.3f}, inliers={inliers}"
        )

    def update_map_to_odom(self):
        """Bridge FAST-LIO localization into the standard Nav2 TF tree.

        FAST-LIO estimates camera_init -> body.  In this robot model the
        FAST-LIO body origin is coincident with base_link.  CHAMP publishes
        base_footprint -> base_link on /odom/local, while the sensor-only
        bridge publishes odom -> base_footprint from FAST-LIO.  Both offsets
        are compensated before publishing map -> odom.
        """
        if (
            self.map_to_local is None
            or self.local_to_body is None
            or self.odom_to_footprint is None
            or self.footprint_to_base is None
        ):
            return

        map_to_base = self.map_to_local @ self.local_to_body
        map_to_footprint = map_to_base @ np.linalg.inv(
            self.footprint_to_base
        )
        self.map_to_odom = map_to_footprint @ np.linalg.inv(
            self.odom_to_footprint
        )

        # Nav2 uses a planar REP-105 map/odom chain.  FAST-LIO and the debug
        # camera_init branch remain fully 3D, while roll and pitch are kept
        # out of map -> odom so the 2D costmaps stay horizontal.
        yaw = Rotation.from_matrix(
            self.map_to_odom[:3, :3]
        ).as_euler("xyz")[2]
        self.map_to_odom[:3, :3] = Rotation.from_euler(
            "z", yaw
        ).as_matrix()

        if not self.bridge_ready_reported:
            self.get_logger().info(
                "Nav2 TF bridge ready: map -> odom"
            )
            self.bridge_ready_reported = True

    def publish_map(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.map_frame

        message = point_cloud2.create_cloud_xyz32(
            header,
            self.map_points.astype(np.float32),
        )
        self.map_publisher.publish(message)

    def publish_body_pose(self, stamp):
        map_to_body = self.map_to_local @ self.local_to_body
        quaternion = Rotation.from_matrix(
            map_to_body[:3, :3]
        ).as_quat()

        message = PoseWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.map_frame

        message.pose.pose.position.x = float(map_to_body[0, 3])
        message.pose.pose.position.y = float(map_to_body[1, 3])
        message.pose.pose.position.z = float(map_to_body[2, 3])
        message.pose.pose.orientation.x = float(quaternion[0])
        message.pose.pose.orientation.y = float(quaternion[1])
        message.pose.pose.orientation.z = float(quaternion[2])
        message.pose.pose.orientation.w = float(quaternion[3])

        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[14] = 0.09
        message.pose.covariance[21] = 0.02
        message.pose.covariance[28] = 0.02
        message.pose.covariance[35] = 0.04

        self.pose_publisher.publish(message)

    def broadcast_transform(self):
        if self.map_to_local is None and self.map_to_odom is None:
            return

        stamp = self.get_clock().now().to_msg()
        transforms = []

        if self.map_to_local is not None:
            transforms.append(
                self.matrix_to_transform(
                    self.map_to_local,
                    self.map_frame,
                    self.local_frame,
                    stamp,
                )
            )

        if self.map_to_odom is not None:
            transforms.append(
                self.matrix_to_transform(
                    self.map_to_odom,
                    self.map_frame,
                    self.odom_frame,
                    stamp,
                )
            )

        self.broadcaster.sendTransform(transforms)

    @staticmethod
    def matrix_to_transform(matrix, parent_frame, child_frame, stamp):
        quaternion = Rotation.from_matrix(matrix[:3, :3]).as_quat()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(matrix[0, 3])
        transform.transform.translation.y = float(matrix[1, 3])
        transform.transform.translation.z = float(matrix[2, 3])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        return transform


def main():
    rclpy.init()
    node = PcdIcpLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
