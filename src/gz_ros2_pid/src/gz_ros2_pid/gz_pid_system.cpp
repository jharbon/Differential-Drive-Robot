#include "gz_ros2_pid/gz_pid_system.hpp"

#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>

#include <iomanip>

// PID gains
const double P_GAIN = 1.0;
const double I_GAIN = 0.5;
const double D_GAIN = 0.0;
// Limit integral windup effects
const double I_MIN = -0.1; 
const double I_MAX = 0.1;
const bool ANTI_WINDUP = false;  // Constrain integral contribution to control output (false) or constrain integral error (true) 
// Constrain effort from PID output
const double MAX_ABS_EFFORT = 0.32;
// Ignore error when its absolute value is lower than defined threshold 
const double DEADBAND_THRESHOLD = 0.03;

namespace gz_ros2_pid {

hardware_interface::return_type
GazeboPidSimSystem::perform_command_mode_switch(
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
                // Reset PID controllers
                for (auto& key_value_pair : this->velocity_controllers) {
                    key_value_pair.second.reset();  // Clear error history 
                }
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
                    std::string anti_windup_str = (ANTI_WINDUP) ? "true" : "false";
                    RCLCPP_INFO_STREAM(
                        this->get_node()->get_logger(),
                        std::fixed << std::setprecision(2)  // 2 decimal places for doubles
                        << "Creating PID controller with gains (P, I, D) = ("
                        << P_GAIN
                        << ", " << I_GAIN
                        << ", " << D_GAIN
                        << ")"
                        << " and integral windup constants"
                        << " (I_MIN, I_MAX, ANTI_WINDUP) = ("
                        << I_MIN << ", "
                        << I_MAX << ", "
                        << anti_windup_str
                        << ")"
                        << " for joint '" << joint_name << "'..." 
                    );
                    control_toolbox::Pid pid;
                    pid.initialize(P_GAIN, I_GAIN, D_GAIN, I_MAX, I_MIN, ANTI_WINDUP);
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
            << "All PID controllers have been constrained as follows:\n"
            << "Maximum absolute effort applied to joint = " << MAX_ABS_EFFORT << "Nm\n"
            << "Deadband absolute error threshold = " << DEADBAND_THRESHOLD << "rad/s"            
        );
    }

    return hardware_interface::return_type::OK;
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
                if (std::abs(error) < DEADBAND_THRESHOLD) {
                    // Set effort to zero to avoid amplification of noise and small numerical inaccuracies in simulation
                    effort = 0.0;
                }
                else {
                    // Update PID error and compute effort signal 
                    effort = pid.compute_command(error, period.seconds());
                }
                // Clamp effort to avoid unrealistic torques
                if (effort > MAX_ABS_EFFORT) {
                    effort = MAX_ABS_EFFORT;
                }
                else if (effort < -MAX_ABS_EFFORT) {
                    effort = -MAX_ABS_EFFORT;
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
