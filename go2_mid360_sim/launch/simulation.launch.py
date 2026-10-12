import os
import socket

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def check_gazebo_port(_context):
    # Run only when launching, so --show-args remains a read-only inspection.
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as connection:
        connection.settimeout(0.5)
        if connection.connect_ex(("127.0.0.1", 11345)) == 0:
            raise RuntimeError(
                "Gazebo port 11345 is already in use. Stop the previous "
                "simulation in its launch terminal with Ctrl+C, wait for it "
                "to exit, then retry. Check remaining servers with: "
                "ps -C gzserver -o pid,ppid,args"
            )
    return []


def generate_launch_description():

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_mid360_config"
    ).find("go2_mid360_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_mid360_description"
    ).find("go2_mid360_description")
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_mid360.xacro")
    sim_share = get_package_share_directory("go2_mid360_sim")
    default_world_path = os.path.join(sim_share, "worlds/go2_workshop.world")
    default_rviz_path = os.path.join(sim_share, "config/mid360.rviz")
    default_scan_params_path = os.path.join(
        sim_share, "config/mid360_to_scan.yaml"
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_scan_params = DeclareLaunchArgument(
        "scan_params_file",
        default_value=default_scan_params_path,
        description="PointCloud2-to-LaserScan parameter file",
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )

    declare_gui = DeclareLaunchArgument(
        "gui", default_value="true", description="Use gui"
    )
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="0.0")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="0.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.35")
    declare_world_init_heading = DeclareLaunchArgument(
        "world_init_heading", default_value="0.0"
    )

    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "rviz_path": default_rviz_path,
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            # The localization package publishes odom -> base_footprint
            # from FAST-LIO; CHAMP must not publish a competing TF.
            "publish_odom_tf": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_share,
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "gui": LaunchConfiguration("gui"),
        }.items(),
    )

    # The Livox Gazebo plugin publishes PointCloud2 on /livox/lidar.  Nav2
    # still needs a planar LaserScan, so derive it from the plugin cloud.
    cloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="mid360_cloud_to_scan",
        output="screen",
        parameters=[LaunchConfiguration("scan_params_file")],
        remappings=[("cloud_in", "/livox/lidar"), ("scan", "/scan")],
    )

    from launch.actions import SetEnvironmentVariable
    environment = [SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
                   SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
                   SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
                   SetEnvironmentVariable("ALSOFT_DRIVERS", "null")]
    return LaunchDescription(
        [*environment,
            declare_use_sim_time,
            declare_rviz,
            declare_scan_params,
            declare_robot_name,
            declare_lite,
            declare_gazebo_world,
            declare_gui,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            OpaqueFunction(function=check_gazebo_port),
            bringup_ld,
            gazebo_ld,
            cloud_to_scan,

        ]
    )
