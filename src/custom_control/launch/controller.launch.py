import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

"""
Launch:
- Joint state broadcaster with parameters from YAML config
- Differential drive controller from ros2_control with parameters from YAML config
- twist_to_twist_stamped node 

twist_to_twist_stamped subscribes to /cmd_vel, converts the user-commanded Twist
messages into TwistStamped messages, and publishes them in /diff_drive_controller/cmd_vel.
The velocity command from the user will ultimately be sent to the hardware interface and 
hence allow the user to control the robot (in simulation or real world) using teleop_twist_keyboard
or teleop_twist_joystick
"""

CONTROL_PACKAGE_NAME = "custom_control"
CONTROLLERS_CONFIG_REL_PATH = "config/controllers.yaml"
TWIST_STAMPED_PUB_NAME = "twist_to_twist_stamped" 

def generate_launch_description():
    control_package_share_path = get_package_share_directory(CONTROL_PACKAGE_NAME)
    controllers_config_default_path = os.path.join(control_package_share_path, CONTROLLERS_CONFIG_REL_PATH)
    controllers_config_arg = DeclareLaunchArgument(
        name="controllers_config_path",
        description="Absolute path to YAML file containing parameters for controller manager, joint state broadcaster, and controllers",
        default_value=controllers_config_default_path
    )
    controllers_config_path = LaunchConfiguration("controllers_config_path")

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file", controllers_config_path
        ],
        output="screen"
    )

    controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--param-file", controllers_config_path
        ],
        output="screen"
    )

    twist_to_twist_stamped = Node(
        package=CONTROL_PACKAGE_NAME,
        executable=TWIST_STAMPED_PUB_NAME,
        output="screen"
    )

    return LaunchDescription([
        controllers_config_arg,
        joint_state_broadcaster_spawner,
        controller_spawner,
        twist_to_twist_stamped
    ])
