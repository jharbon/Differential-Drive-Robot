"""
Launch:
- Controller manager if real hard
- Joint state broadcaster from ros2_control with parameters from YAML config
- Differential drive controller from ros2_control with parameters from YAML config
- twist_to_twist_stamped node 

twist_to_twist_stamped subscribes to /cmd_vel, converts Twist messages into TwistStamped messages,
and publishes them in /diff_drive_controller/cmd_vel.

Using this setup, velocity commands from a user are ultimately sent to a (real or simulated) hardware
interface, enabling control of the robot using teleop_twist_keyboard or teleop_twist_joystick.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch.conditions import UnlessCondition

CONTROL_PACKAGE_NAME = "custom_control"
CONTROLLER_CONFIG_REL_PATH = "config/controller.yaml"
ROBOT_DESCRIPTION_TOPIC_NAME = "/robot_description"
TWIST_STAMPED_PUB_NAME = "twist_to_twist_stamped" 

def generate_launch_description():
    control_package_share_path = get_package_share_directory(CONTROL_PACKAGE_NAME)
    controller_config_default_path = os.path.join(control_package_share_path, CONTROLLER_CONFIG_REL_PATH)

    # Mandatory argument to avoid unexpected time behaviour
    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        description=(
            "Specify whether nodes subscribe to /clock ('true') to sync with Gazebo time or "
            "use OS wall-clock ('false') to sync with real world time"
        )
    )
    # Optional argument
    controller_config_arg = DeclareLaunchArgument(
        name="controller_config_path",
        description=(
            "Absolute path to YAML file containing parameters for joint state broadcaster and diff drive controller"
        ),
        default_value=controller_config_default_path
    )

    use_sim_time_bool = PythonExpression(["True if '", LaunchConfiguration("use_sim_time"), "' == 'true' else False"])

    # Instantiate controller manager here ONLY IF use_sim_time is False, which signals that we are launching
    # control of real hardware. Gazebo loads the controller manager internally when its ros2_control hardware
    # interface plugin is loaded
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_bool,
            "robot_description": ROBOT_DESCRIPTION_TOPIC_NAME  # Assume robot_state_publisher is publishing URDF
        }],
        condition=UnlessCondition(use_sim_time_bool) 
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--param-file", LaunchConfiguration("controller_config_path")
        ],
        parameters=[{
            "use_sim_time": use_sim_time_bool
        }]    
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=[
            "diff_drive_controller",
            "--param-file", LaunchConfiguration("controller_config_path")
        ],
        parameters=[{
            "use_sim_time": use_sim_time_bool
        }]
    )

    twist_to_twist_stamped = Node(
        package=CONTROL_PACKAGE_NAME,
        executable=TWIST_STAMPED_PUB_NAME,
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time_bool
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        controller_config_arg,
        controller_manager,
        joint_state_broadcaster_spawner,
        controller_spawner,
        twist_to_twist_stamped
    ])
