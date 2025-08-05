#include <rclcpp/rclcpp.hpp>
// Vendor packages resolve include paths to include major versions
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>  

#include <gz_ros2_pid/gz_ros2_pid_plugin.hpp>

namespace sim = gz::sim;

namespace gz_ros2_pid {

GazeboRos2PidPlugin::GazeboRos2PidPlugin() : gz_ros2_control::GazeboSimROS2ControlPlugin() {}

void GazeboRos2PidPlugin::Configure(
    const sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    sim::EntityComponentManager & _ecm,
    sim::EventManager & _eventMgr
) {
    // Call base method to load plugin correctly 
    gz_ros2_control::GazeboSimROS2ControlPlugin::Configure(
        _entity,
        _sdf,
        _ecm,
        _eventMgr
    );

    RCLCPP_INFO(
        this->get_node()->get_logger(),
        "GazeboRos2PidPlugin loaded successfully"
    );
}

}  // namespace gz_ros2_pid

// Register plugin and specify standard interfaces
GZ_ADD_PLUGIN(
    gz_ros2_pid::GazeboRos2PidPlugin,
    sim::System,
    gz_ros2_pid::GazeboRos2PidPlugin::ISystemConfigure,
    gz_ros2_pid::GazeboRos2PidPlugin::ISystemPreUpdate,
    gz_ros2_pid::GazeboRos2PidPlugin::ISystemPostUpdate
)
