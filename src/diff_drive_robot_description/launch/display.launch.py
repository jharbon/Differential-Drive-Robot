import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

"""
Launch:
- robot_state_publisher with custom URDF file
- joint_state_publisher_gui
- RViz2 GUI with custom display configuration
"""

PACKAGE_NAME = "diff_drive_robot_description"
URDF_NAME = "robot.urdf.xacro"
DISPLAYS_CONFIG_REL_PATH = os.path.join("config", "displays_config.rviz")

def generate_launch_description():
    package_share_dir = get_package_share_directory(PACKAGE_NAME)
    urdf_path = os.path.join(package_share_dir, "urdf", URDF_NAME)
    robot_description = Command(["xacro ", urdf_path])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}]
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen"
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output="screen",
        arguments=["-d", os.path.join(package_share_dir, DISPLAYS_CONFIG_REL_PATH)]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
