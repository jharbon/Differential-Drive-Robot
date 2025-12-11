#ifndef GZ_ROS2_PID__PID_SYSTEM_INTERFACE
#define GZ_ROS2_PID__PID_SYSTEM_INTERFACE

#include <gz_ros2_control/gz_system.hpp>

#include <control_toolbox/control_toolbox/pid.hpp>

#include <vector>
#include <unordered_map>
#include <string>

namespace gz_ros2_pid {

class GazeboPidSimSystem : public gz_ros2_control::GazeboSimSystem {
public:
    // Overload base class method 
    bool initSim(
        rclcpp::Node::SharedPtr& model_nh,
        std::map<std::string, sim::Entity>& joints,
        const hardware_interface::HardwareInfo& hardware_info,
        sim::EntityComponentManager& _ecm,
        unsigned int update_rate,
        rclcpp::Node::SharedPtr& config_node
    ) override;

    hardware_interface::return_type perform_command_mode_switch(
        const std::vector<std::string> & start_interfaces,
        const std::vector<std::string> & stop_interfaces
    ) override;

    hardware_interface::return_type read(
        const rclcpp::Time &,
        const rclcpp::Duration &
    ) override;

    // Keep almost all functionality from base write() and implement PID velocity control
    hardware_interface::return_type write(
        const rclcpp::Time& time,
        const rclcpp::Duration& period
    ) override;

private:
    std::unordered_map<std::string, control_toolbox::Pid> velocity_controllers;
    rclcpp::Node::SharedPtr config_node;
    control_toolbox::Pid::Gains gains;  // Controllers have the same PID config 
    double effort_clamp;
    double deadband_threshold;

    void reset_controllers();
    void set_controllers_gains();
    void update_joint_effort(sim::Entity& joint_entity, double effort);
};

}  // namespace gz_ros2_pid

#endif
