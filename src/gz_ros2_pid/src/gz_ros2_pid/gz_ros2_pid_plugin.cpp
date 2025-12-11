#include <rclcpp/rclcpp.hpp>
// Vendor packages resolve include paths to include major versions
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>  

#include <gz_ros2_pid/pid_constants.hpp>
#include <gz_ros2_pid/gz_ros2_pid_plugin.hpp>

namespace sim = gz::sim;

namespace gz_ros2_pid {

PidConfigNode::PidConfigNode() : Node{"gz_ros2_pid", "/"} {
    // Declare PID config parameters and check value of each straight after to determine whether default 
    // value is being used; log message to terminal
    rcl_interfaces::msg::ParameterDescriptor default_descriptor;
    default_descriptor.floating_point_range.resize(1);
    // Define continuous range from 0 to very large positive number
    default_descriptor.floating_point_range[0].from_value = 0.0;
    default_descriptor.floating_point_range[0].to_value = std::numeric_limits<double>::max();  // Use max double as practical infinity
    default_descriptor.floating_point_range[0].step = 0.0;
    // Proportional gain
    default_descriptor.description = "Proportional gain";
    this->declare_parameter(
        pid_constants::P_GAIN_NAME,
        rclcpp::ParameterValue(pid_constants::P_GAIN),
        default_descriptor
    );
    // Integral gain
    default_descriptor.description = "Integral gain";
    this->declare_parameter(
        pid_constants::I_GAIN_NAME,
        rclcpp::ParameterValue(pid_constants::I_GAIN),
        default_descriptor
    );
    // Derivative gain
    default_descriptor.description = "Derivative gain";
    this->declare_parameter(
        pid_constants::D_GAIN_NAME,
        rclcpp::ParameterValue(pid_constants::D_GAIN),
        default_descriptor
    );
    // Integral error maximum
    default_descriptor.description = "Maximum integral error clamp applied before multiplication with gain";
    this->declare_parameter(
        pid_constants::I_MAX_NAME,
        rclcpp::ParameterValue(pid_constants::I_MAX),
        default_descriptor
    );
    // Integral error minimum
    rcl_interfaces::msg::ParameterDescriptor i_min_descriptor;
    i_min_descriptor.floating_point_range.resize(1);
    i_min_descriptor.description = "Minimum integral error clamp applied before multiplication with gain";
    // Define continuous range from very large negative number to 0 
    i_min_descriptor.floating_point_range[0].from_value = std::numeric_limits<double>::lowest(); // Use lowest double as practical negative infinity
    i_min_descriptor.floating_point_range[0].to_value = 0.0;  
    i_min_descriptor.floating_point_range[0].step = 0.0;
    this->declare_parameter(
        pid_constants::I_MIN_NAME,
        rclcpp::ParameterValue(pid_constants::I_MIN),
        i_min_descriptor
    );

    default_descriptor.description = "Clamp effort (PID output) with maximum absolute value";
    this->declare_parameter(
        pid_constants::EFFORT_CLAMP_NAME,
        rclcpp::ParameterValue(pid_constants::MAX_ABS_EFFORT),
        default_descriptor
    );

    default_descriptor.description = "Effort set to zero when absolute value of error is below threshold";
    this->declare_parameter(
        pid_constants::DEADBAND_THRESHOLD_NAME,
        rclcpp::ParameterValue(pid_constants::DEADBAND_THRESHOLD),
        default_descriptor
    );
}

GZPidResourceManager::GZPidResourceManager(
        rclcpp::Node::SharedPtr& node,
        sim::EntityComponentManager& ecm,
        std::map<std::string, sim::Entity> enabledJoints,
        rclcpp::executors::MultiThreadedExecutor::SharedPtr executor
) : gz_ros2_control::GZResourceManager(node, ecm, enabledJoints), executor{executor} {
    // Create and spin PID config node
    this->config_node = std::make_shared<PidConfigNode>();
    this->executor->add_node(this->config_node);
}

bool GZPidResourceManager::init_hardware_interface_sim(
        std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface>& gz_pid_sim_system,
        const hardware_interface::HardwareInfo& hardware_info,
        unsigned int update_rate
) {
    return gz_pid_sim_system->initSim(    
        this->node_,
        this->enabledJoints_,
        hardware_info,
        *this->ecm_,
        update_rate,
        this->config_node
    );
}

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

std::unique_ptr<hardware_interface::ResourceManager> GazeboRos2PidPlugin::create_resource_manager(
    sim::EntityComponentManager& _ecm,
    std::map<std::string, sim::Entity>& enabledJoints
) {
    // Use modified GZResourceManager (defined under gz_ros2_pid namespace) and pass executor
    // to spin additional node
    return std::make_unique<gz_ros2_pid::GZPidResourceManager>(
        this->get_node(),
        _ecm,
        enabledJoints,
        this->get_executor()
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
