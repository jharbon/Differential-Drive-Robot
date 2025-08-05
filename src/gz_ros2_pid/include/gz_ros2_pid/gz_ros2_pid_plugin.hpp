#ifndef GZ_ROS2_PID__GZ_ROS2_PID_PLUGIN_HPP
#define GZ_ROS2_PID__GZ_ROS2_PID_PLUGIN_HPP

// Vendor packages resolve include paths to include major versions
#include <gz/sim/System.hh>  

#include <gz_ros2_control/gz_ros2_control_plugin.hpp>

namespace sim = gz::sim;

namespace gz_ros2_pid {

class GazeboRos2PidPlugin : 
    public gz_ros2_control::GazeboSimROS2ControlPlugin {

public:
    GazeboRos2PidPlugin();
    ~GazeboRos2PidPlugin() override = default;

    void Configure(
        const sim::Entity & _entity,
        const std::shared_ptr<const sdf::Element> & _sdf,
        sim::EntityComponentManager & _ecm,
        sim::EventManager & _eventMgr
    ) override;
};
}  // namespace gz_ros2_pid

#endif  // GZ_ROS2_PID__GZ_ROS2_PID_PLUGIN_HPP
