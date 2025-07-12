"""
Launch:
- controller.launch.py with nodes synced to simulation time and controller YAML config path from this script. 
- simulate.launch.py with simulation running from the start and controller YAML config path from this script. 

This is the top-level launch file which includes the control and simulation launch scripts to enable control
of robot during simulation. The control nodes are created first to avoid errors when the Gazebo hardware
interface plugin is loaded. 

The same controller YAML config is passed to both control and simulation launch to ensure consistency when 
the URDF control blocks with the Gazebo hardware interface plugin are enabled.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

CONTROL_PACKAGE_NAME = "custom_control"
CONTROLLER_CONFIG_REL_PATH = "config/controller.yaml"
CONTROLLER_LAUNCH_REL_PATH = "launch/controller.launch.py"
DESCRIPTION_PACKAGE_NAME = "diff_drive_robot_description"
SIMULATION_LAUNCH_REL_PATH = "launch/simulate.launch.py"

def generate_launch_description():
    control_package_share_path = get_package_share_directory(CONTROL_PACKAGE_NAME)
    controller_config_default_path = os.path.join(control_package_share_path, CONTROLLER_CONFIG_REL_PATH)

    # Optional argument
    controller_config_arg = DeclareLaunchArgument(
        name="controller_config_path",
        description=(
            "Absolute path to YAML file containing parameters for controller manager, joint "
            " state broadcaster, and controller"
        ),
        default_value=controller_config_default_path
    )

    # Launch controller first to avoid issues with Gazebo control plugins
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(control_package_share_path, CONTROLLER_LAUNCH_REL_PATH)),
        launch_arguments=[
            ("use_sim_time", "true"),  # All controller nodes should sync with Gazebo time
            ("controller_config_path", LaunchConfiguration("controller_config_path"))
        ]                                 
    )

    # Launch simulation and start it running to expose the hardware interface to controller
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory(DESCRIPTION_PACKAGE_NAME), SIMULATION_LAUNCH_REL_PATH)),
        launch_arguments=[
            ("controller_config_path", LaunchConfiguration("controller_config_path")),
            ("run_sim", "true")
        ]
    )

    return LaunchDescription([
        controller_config_arg,
        controller,
        simulation
    ])
