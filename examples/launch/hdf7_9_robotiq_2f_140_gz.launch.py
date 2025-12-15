#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the overall LaunchDescription with declared arguments."""
    description_pkg = get_package_share_directory("hdr_description")
    resource_path = os.path.dirname(description_pkg)

    existing_gz_path = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    if existing_gz_path:
        gz_resource_path = f"{existing_gz_path}:{resource_path}"
    else:
        gz_resource_path = resource_path

    declared_arguments = [
        SetEnvironmentVariable(
            name="GZ_SIM_RESOURCE_PATH",
            value=gz_resource_path
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for visualization."
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    """Set up all nodes and configurations for the Gazebo simulation."""
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_pkg = get_package_share_directory("hdr_description")
    simulation_pkg = get_package_share_directory("hdr_simulation_gz")

    xacro_file = os.path.join(
        description_pkg, "examples", "hdf7_9_robotiq_2f_140.urdf.xacro"
    )
    controllers_file = os.path.join(
        simulation_pkg, "examples", "config", "hdf7_9_robotiq_2f_140_controllers.yaml"
    )
    rviz_config_file = os.path.join(
        description_pkg, "rviz", "display_robot.rviz"
    )

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", xacro_file,
        " use_sim:=true",
        " use_mock_hardware:=false",
        " name:=hdf7_9_robotiq_2f_140",
        " controllers_file:=", controllers_file,
    ])

    robot_description = {"robot_description": robot_description_content}

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string", robot_description_content,
            "-name", "hdf7_9_robotiq_2f_140",
            "-allow_renaming", "true",
        ],
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "gripper_controller",
            "--controller-manager", "/controller_manager",
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawn_entity,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_joint_trajectory_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    delayed_gripper_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_trajectory_controller_spawner,
            on_exit=[gripper_controller_spawner],
        )
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(launch_rviz),
    )

    delayed_rviz = TimerAction(
        period=3.0,
        actions=[rviz_node],
    )

    return [
        robot_state_publisher_node,
        gz_sim_launch,
        gz_spawn_entity,
        gz_sim_bridge,
        delayed_joint_state_broadcaster,
        delayed_joint_trajectory_controller,
        delayed_gripper_controller,
        delayed_rviz,
    ]
