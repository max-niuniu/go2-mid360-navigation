import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    RegisterEventHandler,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.logging import get_logger
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    go2_share = get_package_share_directory("go2_mid360_bringup")
    nav2_share = get_package_share_directory("nav2_bringup")
    workspace_dir = os.path.abspath(
        os.path.join(go2_share, "..", "..", "..", "..")
    )

    sim_share = get_package_share_directory("go2_mid360_sim")
    localization_share = get_package_share_directory("go2_fastlio_localization")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    show_rviz = LaunchConfiguration("show_rviz")
    nav_rviz = LaunchConfiguration("nav_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    pcd_map = LaunchConfiguration("pcd_map")
    nav_map = LaunchConfiguration("nav_map")
    params_file = LaunchConfiguration("params_file")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    auto_initial_delay = LaunchConfiguration("auto_initial_delay")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, "launch", "localization.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "mapping_rviz": "false",
            "use_sim_time": use_sim_time,
            "map_file": pcd_map,
            "auto_initial_pose": auto_initial_pose,
            "auto_initial_delay": auto_initial_delay,
            "initial_x": initial_x,
            "initial_y": initial_y,
            "initial_yaw": initial_yaw,
        }.items(),
    )

    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[params_file, {"use_sim_time": use_sim_time, "yaml_filename": nav_map}],
    )
    map_lifecycle = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "autostart": True,
            "node_names": ["map_server"],
        }],
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "autostart": "true",
            "use_composition": "False",
            "params_file": params_file,
        }.items(),
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="nav2_rviz",
        output="screen",
        arguments=["-d", os.path.join(go2_share, "rviz", "navigation.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(nav_rviz),
    )

    localization_waiter = Node(
        package="go2_mid360_bringup",
        executable="wait_for_ready.py",
        name="wait_for_localization",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    navigation_waiter = Node(
        package="go2_mid360_bringup",
        executable="wait_for_ready.py",
        name="wait_for_nav2",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "stage": "navigation"}],
        condition=IfCondition(nav_rviz),
    )

    def start_rviz_when_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            get_logger(__name__).error("Nav2 not active; RViz was not started. Check navigation logs.")
            return []
        return [rviz]

    def start_when_ready(event, context):
        if context.is_shutdown:
            return []
        if event.returncode != 0:
            get_logger(__name__).error("Localization/TF not ready; Nav2 was not started. Check localization logs.")
            return []
        actions = [navigation]
        if IfCondition(auto_initial_pose).evaluate(context):
            actions.append(navigation_waiter)
        return actions

    start_navigation = RegisterEventHandler(OnProcessExit(
        target_action=localization_waiter, on_exit=start_when_ready,
    ))
    start_rviz = RegisterEventHandler(OnProcessExit(
        target_action=navigation_waiter, on_exit=start_rviz_when_ready,
    ))
    # Manual initialization needs RViz's 2D Pose Estimate before TF is available.
    manual_rviz = GroupAction(condition=UnlessCondition(auto_initial_pose), actions=[rviz])

    from launch.actions import SetEnvironmentVariable
    environment = [SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),
                   SetEnvironmentVariable("GAZEBO_MASTER_URI", "http://127.0.0.1:11345"),
                   SetEnvironmentVariable("GAZEBO_IP", "127.0.0.1"),
                   SetEnvironmentVariable("ALSOFT_DRIVERS", "null")]
    return LaunchDescription([*environment,
        DeclareLaunchArgument(
            "world",
            default_value=os.path.join(sim_share, "worlds", "go2_workshop.world"),
        ),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("show_rviz", default_value="true"),
        # Capture the public option before nested FAST-LIO launches reuse and
        # overwrite their own `show_rviz` launch configuration.
        SetLaunchConfiguration("nav_rviz", show_rviz),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("auto_initial_pose", default_value="true"),
        DeclareLaunchArgument("auto_initial_delay", default_value="10.0"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "pcd_map",
            default_value=os.path.join(
                workspace_dir, "maps", "go2_workshop", "map.pcd"
            ),
        ),
        DeclareLaunchArgument(
            "nav_map",
            default_value=os.path.join(
                workspace_dir, "maps", "go2_workshop", "map.yaml"
            ),
        ),
        DeclareLaunchArgument(
            "params_file",
            default_value=os.path.join(go2_share, "config", "autonomy", "fast_lio_nav2.yaml"),
        ),
        localization,
        TimerAction(period=16.0, actions=[map_server, map_lifecycle]),
        start_navigation,
        start_rviz,
        localization_waiter,
        manual_rviz,
    ])
