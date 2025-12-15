#!/usr/bin/env python3
import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    TimerAction,
)
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz2 with MoveIt configuration.",
        ),
    ]
    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    launch_rviz = LaunchConfiguration("launch_rviz")

    description_pkg = get_package_share_directory("hdr_description")
    simulation_pkg = get_package_share_directory("hdr_simulation_gz")

    xacro_file = os.path.join(description_pkg, "examples", "hdf7_9_onrobot_rg6.urdf.xacro")
    controllers_file = os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6_controllers.yaml")
    srdf_file = os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6.srdf")

    robot_description_content = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]),
        " ", xacro_file,
        " use_sim:=true",
        " use_mock_hardware:=false",
        " name:=hdf7_9_onrobot_rg6",
        " controllers_file:=", controllers_file,
    ])

    robot_description = {"robot_description": robot_description_content}

    with open(srdf_file, "r") as f:
        robot_description_semantic_content = f.read()
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}
    publish_robot_description_semantic = {"publish_robot_description_semantic": True}

    robot_description_kinematics_yaml = load_yaml_file(
        os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6_kinematics.yaml")
    )
    robot_description_kinematics = {"robot_description_kinematics": robot_description_kinematics_yaml}

    joint_limits_yaml = load_yaml_file(
        os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6_joint_limits.yaml")
    )
    pilz_cartesian_limits_yaml = load_yaml_file(
        os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6_pilz_cartesian_limits.yaml")
    )
    robot_description_planning = {
        "robot_description_planning": {
            **joint_limits_yaml,
            **pilz_cartesian_limits_yaml,
        }
    }

    planning_pipeline_config = {
        "default_planning_pipeline": "ompl",
        "planning_pipelines": ["pilz", "ompl"],
        "pilz": {
            "planning_plugin": "pilz_industrial_motion_planner/CommandPlanner",
        },
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
        },
    }

    for planner_name, config_file in [
        ("pilz", "hdf7_9_onrobot_rg6_pilz_planning.yaml"),
        ("ompl", "hdf7_9_onrobot_rg6_ompl_planning.yaml"),
    ]:
        planner_yaml = load_yaml_file(os.path.join(simulation_pkg, "examples", "config", config_file))
        if planner_yaml:
            planning_pipeline_config[planner_name].update(planner_yaml)

    moveit_controllers_yaml = load_yaml_file(
        os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6_moveit_controllers.yaml")
    )
    moveit_controllers = {
        "moveit_simple_controller_manager": moveit_controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
        "trajectory_execution.execution_duration_monitoring": False,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "move_interactive_markers": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            publish_robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    rviz_config_file = os.path.join(simulation_pkg, "examples", "config", "hdf7_9_onrobot_rg6_moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config_file] if os.path.exists(rviz_config_file) else [],
        parameters=[
            robot_description,
            robot_description_semantic,
            planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
            {"use_sim_time": True},
        ],
    )

    delay_rviz_node = TimerAction(
        period=3.0,
        actions=[rviz_node],
    )

    nodes = [move_group_node]
    if launch_rviz.perform(context).lower() == "true":
        nodes.append(delay_rviz_node)

    return nodes


def load_yaml_file(file_path: str):
    try:
        with open(file_path, "r") as f:
            return yaml.safe_load(f)
    except (OSError, yaml.YAMLError):
        return None
