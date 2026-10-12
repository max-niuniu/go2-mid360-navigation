import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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
    mapping_rviz = LaunchConfiguration("mapping_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    auto_initial_pose = LaunchConfiguration("auto_initial_pose")
    auto_initial_delay = LaunchConfiguration("auto_initial_delay")
    initial_x = LaunchConfiguration("initial_x")
    initial_y = LaunchConfiguration("initial_y")
    initial_yaw = LaunchConfiguration("initial_yaw")

    fast_lio_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_share, "launch", "mapping.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "show_rviz": mapping_rviz,
            "use_sim_time": use_sim_time,
            "save_pcd": "false",
            # Keep even an explicitly requested map-save service away from
            # the reference PCD used by ICP.
            "map_output": os.path.join(workspace_dir, "maps", "localization_session.pcd"),
        }.items(),
    )

    fast_lio_odom_bridge = Node(
        package="go2_fastlio_localization",
        executable="fast_lio_odom_bridge",
        name="fast_lio_odom_bridge",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    localizer = Node(
        package="go2_fastlio_localization",
        executable="pcd_icp_localizer",
        name="pcd_icp_localizer",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "map_file": map_file,
                "auto_initialize": ParameterValue(
                    auto_initial_pose, value_type=bool
                ),
                "auto_initialize_delay": ParameterValue(
                    auto_initial_delay, value_type=float
                ),
                "initial_x": ParameterValue(initial_x, value_type=float),
                "initial_y": ParameterValue(initial_y, value_type=float),
                "initial_yaw": ParameterValue(initial_yaw, value_type=float),
            }
        ],
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
                default_value=os.path.join(
                    sim_share, "worlds", "go2_workshop.world"
                ),
            ),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("show_rviz", default_value="true"),
            DeclareLaunchArgument("mapping_rviz", default_value=show_rviz),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("auto_initial_pose", default_value="true"),
            DeclareLaunchArgument("auto_initial_delay", default_value="10.0"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_yaw", default_value="0.0"),
            DeclareLaunchArgument(
                "map_file",
                default_value=os.path.join(
                    workspace_dir, "maps", "go2_workshop", "map.pcd"
                ),
            ),
            fast_lio_stack,
            fast_lio_odom_bridge,
            TimerAction(period=14.0, actions=[localizer]),
        ]
    )
