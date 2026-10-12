import os

import yaml
from scipy.spatial.transform import Rotation

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    go2_share = get_package_share_directory("go2_fastlio_localization")
    workspace_dir = os.path.abspath(
        os.path.join(go2_share, "..", "..", "..", "..")
    )

    sim_share = get_package_share_directory("go2_mid360_sim")
    localization_share = get_package_share_directory("go2_fastlio_localization")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("show_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_output = LaunchConfiguration("map_output")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "rviz": "false",
            "use_sim_time": use_sim_time,
        }.items(),
    )

    # FAST-LIO does not read its LiDAR-to-IMU extrinsics from TF.
    # Share the exact same mounting configuration as the robot Xacro.
    # The current imu_joint/floating_base joints are identity transforms,
    # so base_link and imu_link coincide (see go2_mid360_description).
    mount_path = os.path.join(
        get_package_share_directory("go2_mid360_description"), "config", "mid360_mount.yaml"
    )
    with open(mount_path, encoding="utf-8") as stream:
        mount = yaml.safe_load(stream)
    lidar_to_imu_rotation = Rotation.from_euler(
        "xyz", mount["rpy_degrees"], degrees=True
    ).as_matrix().ravel().tolist()

    fast_lio = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_mapping",
        output="screen",
        parameters=[
            os.path.join(go2_share, "config", "fast_lio_mid360_sim.yaml"),
            {"use_sim_time": use_sim_time, "map_file_path": map_output,
             "mapping.extrinsic_T": [float(value) for value in mount["xyz"]],
             "mapping.extrinsic_R": lidar_to_imu_rotation,
             "pcd_save.pcd_save_en": ParameterValue(
                 LaunchConfiguration("save_pcd"), value_type=bool)},
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="fast_lio_rviz",
        output="screen",
        arguments=["-d", os.path.join(go2_share, "config", "mapping.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(show_rviz),
    )

    from launch.actions import SetEnvironmentVariable
    environment = [SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
                   SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
                   SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
                   SetEnvironmentVariable("ALSOFT_DRIVERS", "null")]
    return LaunchDescription(
        [*environment,
            DeclareLaunchArgument(
                "world",
                default_value=os.path.join(sim_share, "worlds", "go2_workshop.world"),
            ),
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("save_pcd", default_value="true"),
            DeclareLaunchArgument(
                "map_output",
                default_value=os.path.join(
                    workspace_dir, "maps", "go2_workshop", "map.pcd"
                ),
            ),
            simulation,
            # Allow the robot to stand and settle before IMU initialization;
            # starting during that transient can bias gravity and height.
            TimerAction(period=35.0, actions=[fast_lio]),
            TimerAction(period=12.0, actions=[rviz]),
        ]
    )
