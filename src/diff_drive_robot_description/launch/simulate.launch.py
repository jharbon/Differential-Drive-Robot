"""
Launch:
- Gazebo simulation with max verbosity, empty world, and option to start the simulation running or paused.
- Robot model in Gazebo where URDF is read from /robot_description topic.
- ROS-Gazebo bridge with gz.msgs.Clock -> rosgraph_msgs/msg/Clock conversion published in /clock.

This script simply launches all the nodes needed to describe and simulate a robot. 
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

DESCRIPTION_PACKAGE_NAME = "diff_drive_robot_description"
PUBLISH_DESCRIPTION_REL_PATH = os.path.join("launch", "publish_description.launch.py")
GZ_SIM_PACKAGE_NAME = "ros_gz_sim"
MODEL_SPAWN_NAME = "diff_drive_robot"
GZ_BRIDGE_PACKAGE_NAME = "ros_gz_bridge"


def generate_launch_description():
    # Optional URDF argument.
    # Want to define the default URDF path only in the publish description launch
    # script and make the urdf launch arg here essentially optional
    urdf_path_arg = DeclareLaunchArgument(
        name="urdf_path",
        description=(
            "Absolute path to robot URDF file containing properties, macros, links, joints, plugins, and physics "
            "parameters. Defaults to URDF defined in {} package".format(DESCRIPTION_PACKAGE_NAME)
        ),
        default_value=""
    )
    # Optional arguments to enable and configure controller
    controller_config_arg = DeclareLaunchArgument(
        name="controller_config_path",
        description=(
            "Absolute path to YAML file containing parameters for controller manager, joint state broadcaster, and "
            "controller. Default value of '' disables control"
        ),
        default_value="" 
    )
    # Provide option to auto run sim and create correct flag for gz sim CLI 
    run_sim_arg = DeclareLaunchArgument(
        name="run_sim",
        description="Start the simulation running (true) or paused (false)",
        default_value="false"
    )

    # Publish robot description using robot_state_publisher node
    publish_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), PUBLISH_DESCRIPTION_REL_PATH)
        ),
        launch_arguments=[
            ("use_sim_time", "true"),  
            ("urdf_path", LaunchConfiguration("urdf_path")),
            ("controller_config_path", LaunchConfiguration("controller_config_path")),
            ("hardware_type", "simulated")  # Hardware type should always have this value for simulation
        ]
    )
    
    run_sim_flag = PythonExpression(["'-r' if '", LaunchConfiguration("run_sim"), "' == 'true' else ''"])
    gz_args = PythonExpression(["'", run_sim_flag, " -v 4 empty.sdf'"])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(GZ_SIM_PACKAGE_NAME), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[
            ("gz_args", gz_args)  # Pass arguments to underlying gz sim CLI
        ]
    )

    gazebo_spawn_model = Node(
        package=GZ_SIM_PACKAGE_NAME,
        executable="create",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", MODEL_SPAWN_NAME
        ],
        parameters=[{
            "use_sim_time": True
        }]
    )

    ros_gazebo_bridge = Node(
        package=GZ_BRIDGE_PACKAGE_NAME,
        executable="parameter_bridge",
        output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"  # Convert gz.msgs.Clock -> rosgraph_msgs/msg/Clock and publish in /clock 
        ],
        parameters=[{
            "use_sim_time": True
        }]
    )

    return LaunchDescription([
        urdf_path_arg,
        controller_config_arg,
        run_sim_arg,
        publish_description,
        gazebo,
        gazebo_spawn_model,
        ros_gazebo_bridge
    ])
