#ifndef GZ_ROS2_PID__PID_SYSTEM_INTERFACE
#define GZ_ROS2_PID__PID_SYSTEM_INTERFACE

#include "gz_ros2_control/gz_system.hpp"

//#include <control_toolbox/pid.hpp>
#include <control_toolbox/control_toolbox/pid.hpp>

#include <unordered_map>
#include <string>

namespace gz_ros2_pid {

class GazeboPidSimSystem 
    : public gz_ros2_control::GazeboSimSystem {

public:
    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces
    ) override;

    // Keep almost all functionality from base write() and implement PID velocity control
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) override;

private:
    std::unordered_map<std::string, control_toolbox::Pid> velocity_controllers;

    void update_joint_effort(sim::Entity& joint_entity, double effort);
};

}  // namespace gz_ros2_pid

#endif
