#ifndef GZ_ROS2_PID__GZ_ROS2_PID_PLUGIN_HPP
#define GZ_ROS2_PID__GZ_ROS2_PID_PLUGIN_HPP

// Vendor packages resolve include paths to include major versions
#include <gz/sim/System.hh>  

#include <gz_ros2_control/gz_ros2_control_plugin.hpp>

namespace sim = gz::sim;

namespace gz_ros2_pid {

class PidConfigNode : public rclcpp::Node {
public:
    PidConfigNode();
    ~PidConfigNode() override = default;
};

class GZPidResourceManager : public gz_ros2_control::GZResourceManager {
public:
    GZPidResourceManager(
        rclcpp::Node::SharedPtr & node,
        sim::EntityComponentManager & ecm,
        std::map<std::string, sim::Entity> enabledJoints,
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor
    );
    // Override method to use argument list required by gz_ros2_pid::GazeboPidSimSystem::initSim
    // Used in base class load_and_initialize_components method
    bool init_hardware_interface_sim(
        std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface>& gz_pid_sim_system,
        const hardware_interface::HardwareInfo& hardware_info,
        unsigned int update_rate
    ) override;

private:
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor;
    std::thread executor_node_thread;
    rclcpp::Node::SharedPtr config_node;
};

class GazeboRos2PidPlugin : public gz_ros2_control::GazeboSimROS2ControlPlugin {
public:
    GazeboRos2PidPlugin();
    ~GazeboRos2PidPlugin() override = default;

    void Configure(
        const sim::Entity & _entity,
        const std::shared_ptr<const sdf::Element> & _sdf,
        sim::EntityComponentManager & _ecm,
        sim::EventManager & _eventMgr
    ) override;

private:
    std::unique_ptr<hardware_interface::ResourceManager> create_resource_manager(
        sim::EntityComponentManager& _ecm,
        std::map<std::string, sim::Entity>& enabledJoints
    ) override;
};
}  // namespace gz_ros2_pid

#endif  // GZ_ROS2_PID__GZ_ROS2_PID_PLUGIN_HPP
