#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    TimerAction,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
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
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_model",
            default_value="ha006b",
            choices=['ha006b', 'hdf7_9', 'hdf8_8', 'hdr50_22', 'hdr220_26', 'hh020'],
            description="HDR robot model to use.",
        ),
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="Enable Ignition Gazebo simulation hardware plugin.",
        ),
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="hdr_simulation_gz",
            description="Name of the package providing controllers configuration.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="hdr_controllers.yaml",
            description="YAML file name defining the ROS2 controllers to load.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="hdr_description",
            description="Description package with robot URDF files. Usually the argument is not set, enabling use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="hdr.urdf.xacro",
            description="URDF/XACRO file to use for the robot description.",
        ),
        DeclareLaunchArgument(
            "initial_positions_file",
            default_value="initial_positions.yaml",
            description="The initial positions file for each robot is located in its corresponding moveit_config package.",
        ),
        DeclareLaunchArgument(
            'kinematics_file',
            default_value="kinematics.yaml",
            description="YAML file name defining the robot kinematics.",
        ),
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 for visualization of the robot state."
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])

def launch_setup(context, *args, **kwargs):
    """Set up all nodes and configurations for the Gazebo simulation."""
    use_sim = LaunchConfiguration("use_sim")
    robot_model = LaunchConfiguration("robot_model").perform(context)
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    kinematics_file = LaunchConfiguration("kinematics_file")
    launch_rviz = LaunchConfiguration("launch_rviz")
    
    hdr_hardware_interface_share = get_package_share_directory('hdr_hardware_interface')
    moveit_package_name = f'{robot_model}_moveit_config'

    controllers_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )
    initial_positions = PathJoinSubstitution(
        [FindPackageShare(moveit_package_name), "config", initial_positions_file]
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(description_package), "rviz", "display_robot.rviz"]
    )
    
    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                hdr_hardware_interface_share,
                'launch',
                'ros2_control.launch.py',
            )
        ),
        launch_arguments={
            'robot_model': robot_model,
            'use_sim': use_sim,
            'initial_positions_file': initial_positions,
            'controllers_config_package': runtime_config_package,
            'controllers_file': controllers_file,
            'kinematics_file': kinematics_file,
        }.items(),
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(description_package), "urdf", description_file]
            ),
            " use_sim:=", use_sim,
            " robot_model:=", robot_model,
            " name:=", "hdr",
            " hdr_ros2_control:=", controllers_file,
            " initial_positions_file:=", initial_positions,
        ]
    )

    SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=PathJoinSubstitution([FindPackageShare("hdr_description")]),
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        parameters=[{'use_sim_time': use_sim}],
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--yaw', '0.0', '--pitch', '0.0', '--roll',
            '0.0', '--frame-id', 'world', '--child-frame-id', 'base_link'
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{'use_sim_time': use_sim}],
        condition=IfCondition(launch_rviz),
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "hdr",
            "-allow_renaming",
            "true",
        ],
    )

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": "-r -v 4 empty.sdf"}.items(),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",],
        output="screen",
    )
    
    delayed_rviz_node = TimerAction(
        period=2.0,
        actions=[rviz_node]
    )

    return [
        static_tf_node,
        gz_spawn_entity,
        gz_sim_launch,
        gz_sim_bridge,
        ros2_control_launch,
        delayed_rviz_node,
    ]