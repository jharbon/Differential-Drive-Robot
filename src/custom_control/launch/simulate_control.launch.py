import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

"""
Launch:
- Argument to specify controllers YAML config
- controller.launch.py with controllers YAML config  
- simulate.launch.py with controllers YAML config and simulation running from start 

This is the top-level launch file intended for simulating controllers
"""

CONTROL_PACKAGE_NAME = "custom_control"
CONTROLLERS_CONFIG_REL_PATH = "config/controllers.yaml"
CONTROLLER_LAUNCH_REL_PATH = "launch/controller.launch.py"
DESCRIPTION_PACKAGE_NAME = "diff_drive_robot_description"
SIMULATION_LAUNCH_REL_PATH = "launch/simulate.launch.py"

def generate_launch_description():
    control_package_share_path = get_package_share_directory(CONTROL_PACKAGE_NAME)
    controllers_config_default_path = os.path.join(control_package_share_path, CONTROLLERS_CONFIG_REL_PATH)
    controllers_config_arg = DeclareLaunchArgument(
        name="controllers_config_path",
        description="Absolute path to YAML file containing parameters for controller manager, joint state broadcaster, and controllers",
        default_value=controllers_config_default_path
    )
    controllers_config_path = LaunchConfiguration("controllers_config_path")

    # Launch controller first to avoid issues with Gazebo control plugins
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_package_share_path, CONTROLLER_LAUNCH_REL_PATH)),
        launch_arguments=[
            ("controllers_config_path", controllers_config_path)
        ]                                 
    )

    # Launch simulation and start it running to expose the hardware interface to controller
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), SIMULATION_LAUNCH_REL_PATH)),
        launch_arguments=[
            ("controllers_config_path", controllers_config_path),
            ("run_sim", "true")
        ]
    )

    return LaunchDescription([
        controllers_config_arg,
        controller,
        simulation
    ])
