#include <gz_ros2_pid/pid_constants.hpp>
#include <gz_ros2_pid/gz_pid_system.hpp>
#include <gz_ros2_pid/gz_ros2_pid_plugin.hpp>

#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>

#include <functional>
#include <iomanip>

namespace gz_ros2_pid {

bool GazeboPidSimSystem::initSim(
        rclcpp::Node::SharedPtr& model_nh,
        std::map<std::string, sim::Entity>& joints,
        const hardware_interface::HardwareInfo& hardware_info,
        sim::EntityComponentManager& _ecm,
        unsigned int update_rate,
        rclcpp::Node::SharedPtr& config_node
) {
    // Call base class initSim method using scope resolution
    if (!gz_ros2_control::GazeboSimSystem::initSim(model_nh, joints, hardware_info, _ecm, update_rate)) {
        return false;
    }

    this->config_node = config_node;

    // Read parameters from PID config node to initialize config members 
    // Use parameter name constants from 
    this->gains = control_toolbox::Pid::Gains(
        config_node->get_parameter(pid_constants::P_GAIN_NAME).as_double(),
        config_node->get_parameter(pid_constants::I_GAIN_NAME).as_double(),
        config_node->get_parameter(pid_constants::D_GAIN_NAME).as_double(),
        config_node->get_parameter(pid_constants::I_MAX_NAME).as_double(),
        config_node->get_parameter(pid_constants::I_MIN_NAME).as_double()
    );
    this->effort_clamp = config_node->get_parameter(pid_constants::EFFORT_CLAMP_NAME).as_double();
    this->deadband_threshold = config_node->get_parameter(pid_constants::DEADBAND_THRESHOLD_NAME).as_double();

    return true;
}

hardware_interface::return_type GazeboPidSimSystem::perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces
) {
    for (unsigned int j = 0; j < this->get_num_joints(); j++) {
        std::string joint_name = this->get_joint(j).name;
        for (const std::string & interface_name : stop_interfaces) {
            // Clear joint control method bits corresponding to stop interfaces
            if (interface_name == (joint_name + "/" + hardware_interface::HW_IF_POSITION)) {
                this->get_joint(j).joint_control_method &=
                    static_cast<ControlMethod_>(VELOCITY & EFFORT);
            }
            else if (interface_name == (joint_name + "/" + hardware_interface::HW_IF_VELOCITY)) {  // NOLINT
                this->get_joint(j).joint_control_method &=
                    static_cast<ControlMethod_>(POSITION & EFFORT);
                this->reset_controllers();
            }
            else if (interface_name == (joint_name + "/" + hardware_interface::HW_IF_EFFORT)) {  // NOLINT
                this->get_joint(j).joint_control_method &=
                    static_cast<ControlMethod_>(POSITION & VELOCITY);
            }
        }

        // Set joint control method bits corresponding to start interfaces
        for (const std::string & interface_name : start_interfaces) {
            if (interface_name == (joint_name + "/" + hardware_interface::HW_IF_POSITION)) {
                this->get_joint(j).joint_control_method |= POSITION;
            } 
            else if (interface_name == (joint_name + "/" + hardware_interface::HW_IF_VELOCITY)) {  // NOLINT
                this->get_joint(j).joint_control_method |= VELOCITY;
                if (this->velocity_controllers.find(joint_name) == this->velocity_controllers.end()) {
                    // Key with joint name does not exist yet - create PID
                    std::string anti_windup_str = (pid_constants::ANTI_WINDUP) ? "true" : "false";
                    RCLCPP_INFO_STREAM(
                        this->get_node()->get_logger(),
                        std::fixed << std::setprecision(2)  // 2 decimal places for doubles
                        << "Creating PID controller with gains (P, I, D) = ("
                        << this->gains.p_gain_
                        << ", " << this->gains.i_gain_
                        << ", " << this->gains.d_gain_
                        << ")"
                        << " and integral windup constraints"
                        << " (I_MAX, I_MIN, ANTI_WINDUP) = ("
                        << this->gains.i_max_ << ", "
                        << this->gains.i_min_ << ", "
                        << anti_windup_str
                        << ")"
                        << " for joint '" << joint_name << "'..." 
                    );
                    control_toolbox::Pid pid;
                    pid.initialize(
                        this->gains.p_gain_,
                        this->gains.i_gain_,
                        this->gains.d_gain_,
                        this->gains.i_max_,
                        this->gains.i_min_,
                        pid_constants::ANTI_WINDUP
                    );
                    this->velocity_controllers[joint_name] = pid;
                    RCLCPP_INFO_STREAM(
                        this->get_node()->get_logger(),
                        "PID controller created for joint '" << joint_name << "'"
                    );
                }
            }
            else if (interface_name == (joint_name + "/" + hardware_interface::HW_IF_EFFORT)) {  // NOLINT
                this->get_joint(j).joint_control_method |= EFFORT;
            }
        }
    }    

    if (!this->velocity_controllers.empty()) {
        // At least one PID controller has been initialized 
        RCLCPP_INFO_STREAM(
            this->get_node()->get_logger(),
            std::fixed << std::setprecision(2)  // 2 decimal places for doubles
            << "PID controllers are constrained as follows:\n"
            << "Maximum absolute effort applied to joint = " << this->effort_clamp << "Nm\n"
            << "Deadband absolute error threshold = " << this->deadband_threshold << "rad/s"            
        );
    }

    return hardware_interface::return_type::OK;
}

void GazeboPidSimSystem::reset_controllers() {
    RCLCPP_INFO_STREAM(
        this->get_node()->get_logger(),
        "Resetting error history of all PID controllers"
    );
    for (auto& key_value_pair : this->velocity_controllers) {
        key_value_pair.second.reset();  // Clear error history 
    }
}

void GazeboPidSimSystem::set_controllers_gains() {
    for (auto& key_value_pair : this->velocity_controllers) {
        key_value_pair.second.set_gains(this->gains);
    };
}

hardware_interface::return_type GazeboPidSimSystem::read(
  const rclcpp::Time & time,
  const rclcpp::Duration & duration
) {
    control_toolbox::Pid::Gains old_gains = this->gains;
    double old_effort_clamp = this->effort_clamp;
    double old_deadband_threshold = this->deadband_threshold;
    // Compare current PID config to node config parameters; modify where necessary 
    if (this->gains.p_gain_ != config_node->get_parameter(pid_constants::P_GAIN_NAME).as_double()) {
        this->gains.p_gain_ = config_node->get_parameter(pid_constants::P_GAIN_NAME).as_double();
    }
    if (this->gains.i_gain_ != config_node->get_parameter(pid_constants::I_GAIN_NAME).as_double()) {
        this->gains.i_gain_ = config_node->get_parameter(pid_constants::I_GAIN_NAME).as_double();
    }
    if (this->gains.d_gain_ != config_node->get_parameter(pid_constants::D_GAIN_NAME).as_double()) {
        this->gains.d_gain_ = config_node->get_parameter(pid_constants::D_GAIN_NAME).as_double();
    }
    if (this->gains.i_max_ != config_node->get_parameter(pid_constants::I_MAX_NAME).as_double()) {
        this->gains.i_max_ = config_node->get_parameter(pid_constants::I_MAX_NAME).as_double();
    }
    if (this->gains.i_min_ != config_node->get_parameter(pid_constants::I_MIN_NAME).as_double()) {
        this->gains.i_min_ = config_node->get_parameter(pid_constants::I_MIN_NAME).as_double();
    }
    if (this->effort_clamp != config_node->get_parameter(pid_constants::EFFORT_CLAMP_NAME).as_double()) {
        this->effort_clamp = config_node->get_parameter(pid_constants::EFFORT_CLAMP_NAME).as_double();
    }
    if (this->deadband_threshold != config_node->get_parameter(pid_constants::DEADBAND_THRESHOLD_NAME).as_double()) {
        this->deadband_threshold = config_node->get_parameter(pid_constants::DEADBAND_THRESHOLD_NAME).as_double();
    }

    bool modified = false;
    if (this->gains.p_gain_ != old_gains.p_gain_ ||
        this->gains.i_gain_ != old_gains.i_gain_ ||
        this->gains.d_gain_ != old_gains.d_gain_
    ) {
        RCLCPP_INFO_STREAM(
            this->get_node()->get_logger(),
            std::fixed << std::setprecision(2)  // 2 decimal places for doubles
            << "Modifying config (P, I, D, I_MAX, I_MIN) of all PID controllers as follows:"
            << " (" << old_gains.p_gain_ << ", " << old_gains.i_gain_ << ", " << old_gains.d_gain_ << ", "
            << old_gains.i_max_ << ", " << old_gains.i_min_ << ")"
            << " ->"
            << " (" << this->gains.p_gain_ << ", " << this->gains.i_gain_ << ", " << this->gains.d_gain_ << ", "
            << this->gains.i_max_ << ", " << this->gains.i_min_ << ")"
        );
        this->set_controllers_gains();
        modified = true;
    }

    if (this->effort_clamp != old_effort_clamp ||
        this->deadband_threshold != old_deadband_threshold
    ) {
        RCLCPP_INFO_STREAM(
            this->get_node()->get_logger(),
            std::fixed << std::setprecision(2)  // 2 decimal places for doubles
            << "Modifying PID constraints (EFFORT_CLAMP, DEADBAND_THRESHOLD) as follows:"
            << " (" << old_effort_clamp << ", " << old_deadband_threshold << ")"
            << " ->"
            << " (" << this->effort_clamp << ", " << this->deadband_threshold << ")"
        );
        modified = true;
    }

    if (modified) {
        this->reset_controllers();
    }

    // Call base class method to read hardware states; return value signals whether or not this was successful
    return gz_ros2_control::GazeboSimSystem::read(time, duration);
}

hardware_interface::return_type GazeboPidSimSystem::write(
    const rclcpp::Time&,
    const rclcpp::Duration& period
) {
    for (unsigned int i = 0; i < this->get_num_joints(); ++i) {
        if (this->get_joint(i).sim_joint == sim::kNullEntity) {
            continue;
        }

        if (this->get_joint(i).joint_control_method & VELOCITY) {
            if (!this->get_ecm()->Component<sim::components::JointVelocityCmd>(
                this->get_joint(i).sim_joint))
            {
            this->get_ecm()->CreateComponent(
                this->get_joint(i).sim_joint,
                sim::components::JointVelocityCmd({0}));
            } 
            else {
                // Get reference to PID for given joint
                auto& pid = this->velocity_controllers[this->get_joint(i).name];
                // Get (setpoint - actual) error in velocity
                double error = this->get_joint(i).joint_velocity_cmd - this->get_joint(i).joint_velocity;
                double effort;
                if (std::abs(error) < this->deadband_threshold) {
                    // Set effort to zero to avoid amplification of noise and small numerical inaccuracies in simulation
                    effort = 0.0;
                }
                else {
                    // Update PID error and compute effort signal 
                    effort = pid.compute_command(error, period.seconds());
                }
                // Clamp effort to avoid unrealistic torques
                if (effort > this->effort_clamp) {
                    effort = this->effort_clamp;
                }
                else if (effort < -this->effort_clamp) {
                    effort = -this->effort_clamp;
                }
                
                this->update_joint_effort(this->get_joint(i).sim_joint, effort);
            }
        } else if (this->get_joint(i).joint_control_method & POSITION) {
            // Get error in position
            double error;
            error = (this->get_joint(i).joint_position -
            this->get_joint(i).joint_position_cmd) * this->get_update_rate();

            // Calculate target velcity
            double target_vel = -this->get_pos_prop_gain() * error;

            auto vel =
            this->get_ecm()->Component<sim::components::JointVelocityCmd>(
            this->get_joint(i).sim_joint);

            if (vel == nullptr) {
            this->get_ecm()->CreateComponent(
                this->get_joint(i).sim_joint,
                sim::components::JointVelocityCmd({target_vel}));
            } else if (!vel->Data().empty()) {
            vel->Data()[0] = target_vel;
            }
        } else if (this->get_joint(i).joint_control_method & EFFORT) {
            this->update_joint_effort(
                this->get_joint(i).sim_joint,
                this->get_joint(i).joint_effort_cmd
            );
        } else if (this->get_joint(i).is_actuated && this->get_hold_joints()) {
            // Fallback case is a velocity command of zero (only for actuated joints)
            double target_vel = 0.0;
            auto vel =
            this->get_ecm()->Component<sim::components::JointVelocityCmd>(
            this->get_joint(i).sim_joint);

            if (vel == nullptr) {
            this->get_ecm()->CreateComponent(
                this->get_joint(i).sim_joint,
                sim::components::JointVelocityCmd({target_vel}));
            } else if (!vel->Data().empty()) {
            vel->Data()[0] = target_vel;
            } else if (!vel->Data().empty()) {
            vel->Data()[0] = target_vel;
            }
        }
    }

    // set values of all mimic joints with respect to mimicked joint
    for (const auto & mimic_joint : this->info_.mimic_joints) {
        // Get the joint position
        double position_mimicked_joint =
            this->get_ecm()->Component<sim::components::JointPosition>(
            this->get_joint(mimic_joint.mimicked_joint_index).sim_joint)->Data()[0];

        double position_mimic_joint =
            this->get_ecm()->Component<sim::components::JointPosition>(
            this->get_joint(mimic_joint.joint_index).sim_joint)->Data()[0];

        double position_error =
            position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

        double velocity_sp = (-1.0) * position_error * this->get_update_rate();

        auto vel =
            this->get_ecm()->Component<sim::components::JointVelocityCmd>(
            this->get_joint(mimic_joint.joint_index).sim_joint);

        if (vel == nullptr) {
            this->get_ecm()->CreateComponent(
            this->get_joint(mimic_joint.joint_index).sim_joint,
            sim::components::JointVelocityCmd({velocity_sp}));
        } else if (!vel->Data().empty()) {
            vel->Data()[0] = velocity_sp;
        }
    }

    return hardware_interface::return_type::OK;
}

void GazeboPidSimSystem::update_joint_effort(sim::Entity& joint_entity, const double effort) {
    if (!this->get_ecm()->Component<sim::components::JointForceCmd>(joint_entity)) {
        this->get_ecm()->CreateComponent(
            joint_entity,
            sim::components::JointForceCmd({effort})
        );
    } 
    else {
        const auto effort_cmd_ptr =
            this->get_ecm()->Component<sim::components::JointForceCmd>(joint_entity);
        // Set effort command value
        effort_cmd_ptr->Data()[0] = effort; // Mutate vector in-place
    }
}

}  // namespace gz_ros2_pid

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
    gz_ros2_pid::GazeboPidSimSystem,
    gz_ros2_control::GazeboSimSystemInterface // Gazebo looks for implementation of this class
)
