from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def launch_setup(context, *args, **kwargs):
    # Initialize Arguments
    hdr_type = LaunchConfiguration("hdr_type")
    safety_limits = LaunchConfiguration("safety_limits")
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Include HDR Control Launch
    hdr_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("hdr_simulation"), "/launch/hdr_sim_control.launch.py"]
        ),
        launch_arguments={
            "hdr_type": hdr_type,
            "safety_limits": safety_limits,
            "runtime_config_package": runtime_config_package,
            "controllers_file": controllers_file,
            "description_package": description_package,
            "description_file": description_file,
            "prefix": prefix,
            "launch_rviz": "false",
        }.items(),
    )

    # Include HDR MoveIt Launch
    hdr_moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("hdr_moveit_config"), "/launch/hdr_moveit.launch.py"]
        ),
        launch_arguments={
            "hdr_type": hdr_type,
            "safety_limits": safety_limits,
            "description_package": description_package,
            "description_file": description_file,
            "moveit_config_package": moveit_config_package,
            "moveit_config_file": moveit_config_file,
            "prefix": prefix,
            "use_sim_time": "true",
            "launch_rviz": "true",
            "use_fake_hardware": use_fake_hardware,
        }.items(),
    )

    return [hdr_control_launch, hdr_moveit_launch]

def generate_launch_description():
    declared_arguments = [
        # HDR specific arguments
        DeclareLaunchArgument(
            "hdr_type",
            default_value="hs220_02",
            description="Type/series of used HDR robot.",
            choices=["ha006b", "hh7", "hh020", "hs220_02", "hx400"],
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Indicate whether robot is running with fake hardware mirroring command to its states.",
        ),
        DeclareLaunchArgument(
            "safety_limits",
            default_value="false",
            description="Enables the safety limits controller if true.",
        ),
        # General arguments
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="hdr_simulation",
            description="Package with the controller's configuration in 'config' folder. Usually the argument is not set, enabling use of a custom setup.",
        ),
        DeclareLaunchArgument(
            "controllers_file",
            default_value="hdr_controllers.yaml",
            description="YAML file with the controllers configuration.",
        ),
        DeclareLaunchArgument(
            "description_package",
            default_value="hdr_description",
            description="Description package with robot URDF/XACRO files. Usually the argument is not set, enabling use of a custom description.",
        ),
        DeclareLaunchArgument(
            "description_file",
            default_value="hdr.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="hdr_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument is not set, enabling use of a custom moveit config.",
        ),
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="hdr.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        ),
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for multi-robot setup. If changed, joint names in the controllers' configuration must also be updated.",
        ),
    ]

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
