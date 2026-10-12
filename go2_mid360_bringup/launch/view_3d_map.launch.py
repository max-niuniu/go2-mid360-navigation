import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    fast_lio_share = get_package_share_directory("fast_lio")
    go2_share = get_package_share_directory("go2_mid360_bringup")
    workspace_dir = os.path.abspath(
        os.path.join(go2_share, "..", "..", "..", "..")
    )
    map_file = LaunchConfiguration("map_file")

    cloud_publisher = Node(
        package="pcl_ros",
        executable="pcd_to_pointcloud",
        name="mid360_3d_map_publisher",
        output="screen",
        parameters=[{"file_name": map_file, "publish_rate": 1.0}],
        remappings=[("cloud_pcd", "cloud_registered")],
    )

    map_frame = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="mid360_3d_map_frame",
        arguments=["0", "0", "0", "0", "0", "0", "camera_init", "base_link"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="mid360_3d_map_rviz",
        output="screen",
        arguments=["-d", os.path.join(fast_lio_share, "rviz", "fastlio.rviz")],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map_file", default_value=os.path.join(workspace_dir, "maps", "go2_workshop", "map.pcd")
            ),
            cloud_publisher,
            map_frame,
            rviz,
        ]
    )
