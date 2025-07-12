"""
Launch:
- robot_state_publisher with flexible clock, URDF path, and control configuration

This script is intended to be used by both simulation and real world launch files to generate and publish the robot
URDF string. Control is enabled when a valid path to a controller YAML config is supplied; leave as empty/default 
to disable control. 
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DESCRIPTION_PACKAGE_NAME = "diff_drive_robot_description"
URDF_DEFAULT_PATH = os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), "urdf", "robot.urdf.xacro")


def generate_launch_description():
    # Make use_sim_time argument mandatory to avoid unexpected time behaviour
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        description=(
            "Specify whether robot_state_publisher node subscribes to /clock ('true') to sync with Gazebo time or "
            "uses OS wall-clock ('false') to sync with real world time"
        )
    )
    # Optional arguments
    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_path",
        description=(
            "Absolute path to robot URDF file containing properties, macros, links, joints, plugins, and physics "
            "parameters. Defaults to URDF defined in {} package".format(DESCRIPTION_PACKAGE_NAME)
        ),
        default_value=URDF_DEFAULT_PATH
    )
    controller_config_arg = DeclareLaunchArgument(
        name="controller_config_path",
        description=(
            "Absolute path to YAML file containing parameters for controller manager, joint state broadcaster, and "
            "controller. Default value of '' disables control"
        ),
        default_value=""
    )
    hardware_type_arg = DeclareLaunchArgument(
        name="hardware_type",
        description=(
            "Type of hardware ('real' or 'simulated') being used, which determines whether the real or Gazebo hardware "
            "interface plugin is used. Default value of '' disables control"
        ),
        default_value=""
    )

    # Assume urdf_path empty string means default should be used. This is to provide URDF path flexibility to other launch
    # scripts while only allowing this script to define the default
    urdf_path = PythonExpression(
        ["'", URDF_DEFAULT_PATH, "' if '", LaunchConfiguration("urdf_path"), "' == '' else '", LaunchConfiguration("urdf_path"), "'"]
    )

    # Construct xacro command using urdf and controller config paths to generate robot description string
    robot_description_cmd = Command([
        "xacro ",
        urdf_path,
        " controller_config_path:=",
        LaunchConfiguration("controller_config_path"),
        " hardware_type:=",
        LaunchConfiguration("hardware_type")
    ])

    # Publish robot description on /robot_description topic
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description_cmd,
            "use_sim_time": PythonExpression(["True if '", LaunchConfiguration("use_sim_time"), "' == 'true' else False"])
        }]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        urdf_path_arg,
        controller_config_arg,
        hardware_type_arg,
        robot_state_publisher
    ])
