#include <gz/sim8/gz/sim/System.hh>
#include <gz/sim8/gz/sim/Model.hh>
#include <gz/sim8/gz/sim/components/JointVelocityCmd.hh>
#include <gz/plugin2/gz/plugin/Register.hh>

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <geometry_msgs/msg/twist.hpp>

#include <string>
#include <stdexcept>

// Names of parameters which should be set in the URDF
const std::string LEFT_WHEEL_JOINT_PARAM = "left_wheel_joint_name";
const std::string RIGHT_WHEEL_JOINT_PARAM = "right_wheel_joint_name";
const std::string WHEEL_RADIUS_PARAM = "wheel_radius";
const std::string WHEELS_SEPARATION_PARAM = "wheels_separation";

namespace custom {
    class DiffDrivePlugin 
        : public gz::sim::System,  
          public gz::sim::ISystemConfigure,
          public gz::sim::ISystemPreUpdate,
          public gz::sim::ISystemPostUpdate {

            public:
              void Configure(const gz::sim::Entity& _entity,
                             const std::shared_ptr<const sdf::Element>& _sdf,
                             gz::sim::EntityComponentManager& _ecm,
                             gz::sim::EventManager&) override {

                // Make sure that ROS 2 has been initialized 
                if (!rclcpp::ok()) {
                  RCLCPP_INFO(rclcpp::get_logger("DiffDrivePlugin"), "Initializing ROS 2...");
                  rclcpp::init(0, nullptr);
                }

                auto model = gz::sim::Model(_entity);
                this->model_name = model.Name(_ecm);
                RCLCPP_INFO(rclcpp::get_logger("DiffDrivePlugin"), "Loading DiffDrivePlugin for '%s'...", this->model_name.c_str());

                // Get wheel joint names, wheel radius, and wheels separation from parsed URDF parameters
                std::string left_wheel_joint_name, right_wheel_joint_name;
                if (_sdf->HasElement(LEFT_WHEEL_JOINT_PARAM) && _sdf->HasElement(RIGHT_WHEEL_JOINT_PARAM)
                    && _sdf->HasElement(WHEEL_RADIUS_PARAM) && _sdf->HasElement(WHEELS_SEPARATION_PARAM)) {

                  left_wheel_joint_name = _sdf->Get<std::string>(LEFT_WHEEL_JOINT_PARAM);
                  right_wheel_joint_name = _sdf->Get<std::string>(RIGHT_WHEEL_JOINT_PARAM);
                  this->wheel_radius = _sdf->Get<double>(WHEEL_RADIUS_PARAM);
                  this->wheels_separation = _sdf->Get<double>(WHEELS_SEPARATION_PARAM);
                }
                else {
                  RCLCPP_FATAL(rclcpp::get_logger("DiffDrivePlugin"), "Parameters '%s', '%s', '%s', and '%s' must all be set in URDF",
                               LEFT_WHEEL_JOINT_PARAM.c_str(), RIGHT_WHEEL_JOINT_PARAM.c_str(),
                               WHEEL_RADIUS_PARAM.c_str(), WHEELS_SEPARATION_PARAM.c_str());
                  throw std::runtime_error("Missing URDF plugin parameters");
                }

                RCLCPP_DEBUG(rclcpp::get_logger("DiffDrivePlugin"), "Left wheel joint name parameter: %s", left_wheel_joint_name.c_str());
                RCLCPP_DEBUG(rclcpp::get_logger("DiffDrivePlugin"), "Right wheel joint name parameter: %s", right_wheel_joint_name.c_str()); 
                RCLCPP_DEBUG(rclcpp::get_logger("DiffDrivePlugin"), "Wheel radius parameter: %.4f", this->wheel_radius); 
                RCLCPP_DEBUG(rclcpp::get_logger("DiffDrivePlugin"), "Wheels separation parameter: %.4f", this->wheels_separation); 
                
                // Get wheel joints
                this->left_wheel_joint = model.JointByName(_ecm, left_wheel_joint_name);
                this->right_wheel_joint = model.JointByName(_ecm, right_wheel_joint_name);

                // Make sure that the ECM found valid joint entities 
                if (!this->left_wheel_joint) {
                  RCLCPP_FATAL(rclcpp::get_logger("DiffDrivePlugin"), "Cannot find left wheel joint with name '%s'", left_wheel_joint_name.c_str());
                  throw std::runtime_error("Missing left wheel joint");
                }
                else if (!this->right_wheel_joint) {
                  RCLCPP_FATAL(rclcpp::get_logger("DiffDrivePlugin"), "Cannot find right wheel joint with name '%s'", right_wheel_joint_name.c_str());
                  throw std::runtime_error("Missing right wheel joint");
                }

                // Create components for setting joint velocities in update steps
                _ecm.CreateComponent(this->left_wheel_joint, gz::sim::components::JointVelocityCmd({0.0}));
                _ecm.CreateComponent(this->right_wheel_joint, gz::sim::components::JointVelocityCmd({0.0}));

                // Create node and subscribe to /vel_cmd for user kinematics input
                this->node = std::make_shared<rclcpp::Node>("diff_drive_plugin_node");
                this->sub_cmd_vel = this->node->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&DiffDrivePlugin::vel_cmd_callback, this, std::placeholders::_1));

                // Create an executor and thread (separate to Gazebo) to spin node
                this->executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
                this->executor->add_node(this->node);
                this->ros_thread = std::thread([this]() {
                  this->executor->spin();
                });
              }  

              void PreUpdate(const gz::sim::UpdateInfo&,
                             gz::sim::EntityComponentManager& _ecm) override {
                auto* left_vel_cmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->left_wheel_joint);
                auto* right_vel_cmd = _ecm.Component<gz::sim::components::JointVelocityCmd>(this->right_wheel_joint);

                left_vel_cmd->Data()[0] = this->left_angular_vel;
                right_vel_cmd->Data()[0] = this->right_angular_vel;
              }

              void PostUpdate(const gz::sim::UpdateInfo &,
                              const gz::sim::EntityComponentManager &) override {}

              ~DiffDrivePlugin() {
                RCLCPP_INFO(rclcpp::get_logger("DiffDrivePlugin"), "Destroying custom DiffDrivePlugin for '%s'...", this->model_name.c_str());

                // Instruct the executor to stop if still spinning
                if (this->executor) {
                  this->executor->cancel();
                }

                // Wait for thread to shut down
                if (this->ros_thread.joinable()) {
                  this->ros_thread.join();
                }
              }

            private:
              std::string model_name;
              gz::sim::Entity left_wheel_joint;
              gz::sim::Entity right_wheel_joint;
              double wheel_radius;  // m
              double wheels_separation;  // m
              double left_angular_vel, right_angular_vel;  // rad/s

              rclcpp::Node::SharedPtr node;
              rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel;
              rclcpp::executors::SingleThreadedExecutor::SharedPtr executor;
              std::thread ros_thread;

              void vel_cmd_callback(const geometry_msgs::msg::Twist msg) {
                RCLCPP_DEBUG(rclcpp::get_logger("DiffDrivePlugin"), "Received velocity command [linear, angular] = [%.4f, %.4f]", msg.linear.x, msg.angular.z);
                // Use differential drive kinematics to determine left and right wheel angular velocity required for the given base linear and angular velocities
                this->left_angular_vel = (msg.linear.x - (this->wheels_separation * msg.angular.z) / 2) / this->wheel_radius;
                this->right_angular_vel = (msg.linear.x + (this->wheels_separation * msg.angular.z) / 2) / this->wheel_radius;
              }
    };
}

GZ_ADD_PLUGIN(
  custom::DiffDrivePlugin,
  gz::sim::System,
  gz::sim::ISystemConfigure,
  gz::sim::ISystemPreUpdate,
  gz::sim::ISystemPostUpdate
)

GZ_ADD_PLUGIN_ALIAS(custom::DiffDrivePlugin, "custom::DiffDrivePlugin")