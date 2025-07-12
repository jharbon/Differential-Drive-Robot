"""
Launch:
- publish_description.launch.py to include robot_state_publisher node.
- joint_state_publisher_gui.
- RViz2 GUI with custom display configuration.

This script simply launches all the nodes needed to describe a robot, display it in RViz2, and enable a user to
control its joint states with a separate GUI.

If the use_sim_time is set to "true", it is assumed that a robot_state_publisher node process has already been
created by the simulation launch script, and hence it is not created here to avoid conflict.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import UnlessCondition
from launch_ros.actions import Node

DESCRIPTION_PACKAGE_NAME = "diff_drive_robot_description"
PUBLISH_DESCRIPTION_REL_PATH = os.path.join("launch", "publish_description.launch.py")
DISPLAYS_CONFIG_REL_PATH = os.path.join("config", "displays_config.rviz")


def generate_launch_description():
    # Mandatory to avoid unexpected time behaviour
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
        default_value=""
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

    # Publish robot description with robot_state_publisher node when use_sim_time is False.
    # Otherwise, assume that simulation launch has created robot_state_publisher
    publish_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), PUBLISH_DESCRIPTION_REL_PATH)
        ),
        launch_arguments=[
            ("use_sim_time", LaunchConfiguration("use_sim_time")),
            ("urdf_path", LaunchConfiguration("urdf_path")),  
            ("controller_config_path", LaunchConfiguration("controller_config_path")),
            ("hardware_type", LaunchConfiguration("hardware_type"))
        ],
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")) 
    )

    use_sim_time_bool = PythonExpression(["True if '", LaunchConfiguration("use_sim_time"), "' == 'true' else False"])

    # Enable manipulation of movable joints only when robot is NOT being simulated
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=UnlessCondition(LaunchConfiguration("use_sim_time")) 
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output="screen",
        arguments=["-d", os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), DISPLAYS_CONFIG_REL_PATH)],
        parameters=[{
            "use_sim_time": use_sim_time_bool
        }] 
    )

    return LaunchDescription([
        use_sim_time_arg,
        urdf_path_arg,
        controller_config_arg,
        hardware_type_arg,
        publish_description,
        joint_state_publisher_gui,
        rviz
    ])
