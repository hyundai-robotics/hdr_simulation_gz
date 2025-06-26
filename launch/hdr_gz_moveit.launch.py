#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    OpaqueFunction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, 
    PathJoinSubstitution
)

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Declare launch arguments and set up HDR simulation + MoveIt."""
    declared_arguments = [
        DeclareLaunchArgument(
            'robot_model',
            default_value='ha006b',
            choices=['ha006b', 'hdf7_9', 'hdf8_8', 'hdr50_22', 'hdr220_26', 'hh020'],
            description="HDR robot model to use.",
        ),
        DeclareLaunchArgument(
            'use_sim',
            default_value='true',
            description="Enable Ignition Gazebo simulation hardware plugin.",
        ),
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='hdr_simulation_gz',
            description="Name of the package providing controller, initial positions, and kinematics configurations.",
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value='hdr_controllers.yaml',
            description='YAML file name defining the ROS2 controllers to load.',
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='hdr_description',
            description="Description package with robot URDF files. Usually the argument is not set, enabling use of a custom description.",
        ),
        DeclareLaunchArgument(
            'description_file',
            default_value='hdr.urdf.xacro',
            description='URDF/XACRO file to use for the robot description.',
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
    ]

    return LaunchDescription(declared_arguments + [
            OpaqueFunction(function=launch_setup)
    ])

def launch_setup(context, *args, **kwargs):
    """Set up HDR simulation and MoveIt includes."""
    robot_model = LaunchConfiguration('robot_model').perform(context)
    use_sim = LaunchConfiguration('use_sim')
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    controllers_file = LaunchConfiguration('controllers_file')
    description_package = LaunchConfiguration('description_package')
    description_file = LaunchConfiguration('description_file')
    initial_positions_file = LaunchConfiguration("initial_positions_file")
    kinematics_file = LaunchConfiguration("kinematics_file")
    
    moveit_package_name = f'{robot_model}_moveit_config'
    hdr_moveit_config_share = get_package_share_directory(moveit_package_name)
    
    initial_positions = PathJoinSubstitution(
        [FindPackageShare(moveit_package_name), "config", initial_positions_file]
    )

    hdr_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare(runtime_config_package),
             '/launch/hdr_gz_spawn.launch.py']
        ),
        launch_arguments={
            'robot_model': robot_model,
            'use_sim': use_sim,
            'description_package': description_package,
            'description_file': description_file,
            'initial_positions_file': initial_positions,
            'controllers_config_package': runtime_config_package,
            'controllers_file': controllers_file,
            'kinematics_file': kinematics_file,
            'launch_rviz': 'false',
        }.items(),
    )

    move_group_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                hdr_moveit_config_share,
                'launch',
                'move_group.launch.py',
            )
        ),
        launch_arguments={
            'robot_model': robot_model,
            "use_sim": use_sim,
        }.items(),
    )

    return [hdr_control_launch, move_group_launch]
