import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

"""
Launch:
- robot_state_publisher with custom URDF file
- Empty world in Gazebo using gz_sim.launch.py from ros_gz_sim
- Robot model in Gazebo using ros_gz_sim and custom URDF content
- ROS-Gazebo bridge with gz.msgs.Clock -> rosgraph_msgs/msg/Clock conversion published in /clock
"""

PACKAGE_NAME = "diff_drive_robot_description"
URDF_NAME = "robot.urdf.xacro"

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory(PACKAGE_NAME), "urdf", URDF_NAME)
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": Command(["xacro ", urdf_path]),
            "use_sim_time": use_sim_time
            }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", [" -v 4 empty.sdf"])
        ]
    )

    gazebo_spawn_model = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", "diff_drive_robot"
        ]
    )

    ros_gazebo_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"  # Convert gz.msgs.Clock -> rosgraph_msgs/msg/Clock and publish in /clock so that ROS 2 nodes can subscribe and determine the simulation time
        ]
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        gazebo_spawn_model,
        ros_gazebo_bridge
    ])
