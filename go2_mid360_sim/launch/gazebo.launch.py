"""Gazebo processes and Go2 spawning; included by simulation.launch.py."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, Shutdown, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    sim_share = get_package_share_directory("go2_mid360_sim")
    config_share = get_package_share_directory("go2_mid360_config")
    arguments = [
        DeclareLaunchArgument("robot_name", default_value="go2"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument(
            "world", default_value=os.path.join(sim_share, "worlds/go2_workshop.world")
        ),
        DeclareLaunchArgument("world_init_x", default_value="0.0"),
        DeclareLaunchArgument("world_init_y", default_value="0.0"),
        DeclareLaunchArgument("world_init_z", default_value="0.50"),
        DeclareLaunchArgument("world_init_heading", default_value="0.0"),
        DeclareLaunchArgument(
            "contact_sensor", default_value="false",
            description="Run the optional CHAMP foot-contact helper",
        ),
    ]

    server = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose",
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
            LaunchConfiguration("world"),
            "--ros-args", "--params-file",
            os.path.join(sim_share, "config/gazebo.yaml"),
        ],
        output="screen",
        on_exit=[Shutdown(reason="Gazebo server exited; stopping the simulation.")],
    )
    client = ExecuteProcess(
        cmd=["gzclient"],
        condition=IfCondition(LaunchConfiguration("gui")),
        output="screen",
    )
    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", LaunchConfiguration("robot_name"),
            "-topic", "/robot_description",
            "-robot_namespace", "",
            "-x", LaunchConfiguration("world_init_x"),
            "-y", LaunchConfiguration("world_init_y"),
            "-z", LaunchConfiguration("world_init_z"),
            "-R", "0", "-P", "0",
            "-Y", LaunchConfiguration("world_init_heading"),
        ],
    )
    # Allow the world to load before inserting the robot.
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_robot])
    # gazebo_ros2_control creates the manager after the robot is spawned.
    controllers = Node(
        package="controller_manager",
        executable="spawner",
        name="go2_controller_spawner",
        output="screen",
        arguments=[
            "joint_states_controller", "joint_group_effort_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )
    contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        output="screen",
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            os.path.join(config_share, "config/links/links.yaml"),
        ],
        condition=IfCondition(LaunchConfiguration("contact_sensor")),
    )
    return LaunchDescription([
        *arguments, server, client, delayed_spawn, controllers, contact_sensor,
    ])
