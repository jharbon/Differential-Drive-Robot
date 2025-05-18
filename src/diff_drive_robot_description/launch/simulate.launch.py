import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

"""
Launch:
- robot_state_publisher with URDF from description package and launch arg for specifying controllers YAML config (this switches on control)
- Gazebo simulation using gz_sim.launch.py (from ros_gz_sim) with max verbosity, empty world, and launch arg to start the simulation either running or paused
- Robot model in Gazebo using create (from ros_gz_sim) with URDF read from /robot_description topic
- ROS-Gazebo bridge with gz.msgs.Clock -> rosgraph_msgs/msg/Clock conversion published in /clock
"""

DESCRIPTION_PACKAGE_NAME = "diff_drive_robot_description"
URDF_NAME = "robot.urdf.xacro"

def generate_launch_description():
    urdf_path = os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), "urdf", URDF_NAME)
    # Provide option to configure and use controllers in sim 
    # Default sim has no control to allow basic test of URDF, world etc...
    controllers_config_arg = DeclareLaunchArgument(
        name="controllers_config_path",
        description="Absolute path to YAML file containing parameters for controller manager, joint state broadcaster, and controllers. Control only used when path is non-empty",
        default_value=""
    )
    robot_description_cmd = Command([
        "xacro ",
        urdf_path,
        " controllers_config_path:=",
        LaunchConfiguration("controllers_config_path")
    ])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_cmd,
            "use_sim_time": LaunchConfiguration("use_sim_time", default="true")
            }]
    )

    # Provide option to auto run sim and create correct flag for gz sim CLI 
    run_sim_arg = DeclareLaunchArgument(
        name="run_sim",
        description="Start the simulation running (true) or paused (false)",
        default_value="false")
    run_sim_flag = PythonExpression(["'-r' if '", LaunchConfiguration("run_sim"), "' == 'true' else ''"])
    gz_args = PythonExpression(["'", run_sim_flag, " -v 4 empty.sdf'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")),
        launch_arguments=[
            # Pass arguments to underlying gz sim CLI
            ("gz_args", gz_args)
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
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"  # Convert gz.msgs.Clock -> rosgraph_msgs/msg/Clock and publish in /clock to allow nodes to subscribe and determine simulation time
        ]
    )

    return LaunchDescription([
        controllers_config_arg,
        robot_state_publisher,
        run_sim_arg,
        gazebo,
        gazebo_spawn_model,
        ros_gazebo_bridge
    ])
