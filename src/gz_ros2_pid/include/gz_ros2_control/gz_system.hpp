// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#ifndef GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_
#define GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gz_ros2_control/gz_system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <gz/msgs/imu.pb.h>
#include <gz/msgs/wrench.pb.h>

#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/transport/Node.hh>

#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;
};

class ForceTorqueData
{
public:
  /// \brief force torque sensor's name.
  std::string name{};

  /// \brief force torque sensor's topic name.
  std::string topicName{};

  /// \brief handles to the force torque from within Gazebo
  sim::Entity sim_ft_sensors_ = sim::kNullEntity;

  /// \brief An array per FT
  std::array<double, 6> ft_sensor_data_;

  /// \brief callback to get the Force Torque topic values
  void OnForceTorque(const GZ_MSGS_NAMESPACE Wrench & _msg);
};

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg);
};

namespace gz_ros2_control
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GazeboSimSystemPrivate
{
public:
  GazeboSimSystemPrivate() = default;

  ~GazeboSimSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus.
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief vector with the force torque sensors.
  std::vector<std::shared_ptr<ForceTorqueData>> ft_sensors_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  unsigned int update_rate;

  /// \brief Gazebo communication node.
  GZ_TRANSPORT_NAMESPACE Node node;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;

  // Should hold the joints if no control_mode is active
  bool hold_joints_ = true;
};

// These class must inherit `gz_ros2_control::GazeboSimSystemInterface` which implements a
// simulated `ros2_control` `hardware_interface::SystemInterface`.

class GazeboSimSystem : public GazeboSimSystemInterface
{
public:
  // Documentation Inherited
  CallbackReturn on_init(const hardware_interface::HardwareInfo & system_info)
  override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  // Documentation Inherited
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // Documentation Inherited
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  // Documentation Inherited
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  // Documentation Inherited
  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Documentation Inherited
  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  // Documentation Inherited
  bool initSim(
    rclcpp::Node::SharedPtr & model_nh,
    std::map<std::string, sim::Entity> & joints,
    const hardware_interface::HardwareInfo & hardware_info,
    sim::EntityComponentManager & _ecm,
    unsigned int update_rate) override;

protected:
  sim::EntityComponentManager * get_ecm(); 
  size_t get_n_dof() const;
  jointData& get_joint(int i);
  unsigned int get_num_joints() const;
  int get_update_rate() const;
  rclcpp::Node::SharedPtr get_node() const;
  double get_pos_prop_gain() const;
  bool get_hold_joints() const;


private:
  // Register a sensor (for now just IMUs)
  // \param[in] hardware_info hardware information where the data of
  // the sensors is extract.
  void registerSensors(
    const hardware_interface::HardwareInfo & hardware_info);

  /// \brief Private data class
  std::unique_ptr<GazeboSimSystemPrivate> dataPtr;
};

}  // namespace gz_ros2_control

// for backward compatibility
namespace ign_ros2_control
{
using IgnitionSystem = gz_ros2_control::GazeboSimSystem;
}  // namespace ign_ros2_control

#endif  // GZ_ROS2_CONTROL__GZ_SYSTEM_HPP_
