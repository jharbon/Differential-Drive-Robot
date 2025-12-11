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

#ifndef GZ_ROS2_CONTROL__GZ_ROS2_CONTROL_PLUGIN_HPP_
#define GZ_ROS2_CONTROL__GZ_ROS2_CONTROL_PLUGIN_HPP_

#include <memory>

#include <gz/sim/System.hh>
#include <rclcpp/rclcpp.hpp>

#include <hardware_interface/resource_manager.hpp>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

#include <pluginlib/class_loader.hpp>

#include "gz_ros2_control/gz_system.hpp"

namespace sim = gz::sim;

namespace gz_ros2_control
{

class GZResourceManager : public hardware_interface::ResourceManager
{
public:
  GZResourceManager(
    rclcpp::Node::SharedPtr & node,
    sim::EntityComponentManager & ecm,
    std::map<std::string, sim::Entity> enabledJoints);
  
  GZResourceManager(const GZResourceManager &) = delete;

  // Called from Controller Manager when robot description is initialized from callback
  bool load_and_initialize_components(
    const std::string & urdf,
    unsigned int update_rate) override;

  // Refactored load_and_initialize_components to initialize hardware interface here.
  // Enables GZResourceManager subclass to easily call initSim method with different
  // arguments
  virtual bool init_hardware_interface_sim(
    std::unique_ptr<gz_ros2_control::GazeboSimSystemInterface>& gz_sim_system,
    const hardware_interface::HardwareInfo& hardware_info,
    unsigned int update_rate
  );

  //std::shared_ptr<rclcpp::Node>& get_node();
  //sim::EntityComponentManager* get_ecm();
  //std::map<std::string, sim::Entity>& get_enabled_joints();

protected:
  std::shared_ptr<rclcpp::Node> node_;
  sim::EntityComponentManager * ecm_;
  std::map<std::string, sim::Entity> enabledJoints_;

private:
  /// \brief Interface loader
  pluginlib::ClassLoader<gz_ros2_control::GazeboSimSystemInterface> gz_system_loader_;

  rclcpp::Logger logger_;
};

// Forward declarations.
class GazeboSimROS2ControlPluginPrivate;

class GazeboSimROS2ControlPlugin
  : public sim::System,
  public sim::ISystemConfigure,
  public sim::ISystemPreUpdate,
  public sim::ISystemPostUpdate
{
public:
  /// \brief Constructor
  GazeboSimROS2ControlPlugin();

  /// \brief Destructor
  ~GazeboSimROS2ControlPlugin() override;

  // Documentation inherited
  void Configure(
    const sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    sim::EntityComponentManager & _ecm,
    sim::EventManager & _eventMgr) override;

  // Documentation inherited
  void PreUpdate(
    const sim::UpdateInfo & _info,
    sim::EntityComponentManager & _ecm) override;

  void PostUpdate(
    const sim::UpdateInfo & _info,
    const sim::EntityComponentManager & _ecm) override;

protected:
  // Enable subclasses to access gz_ros_control node for logging and node
  // executor to add new nodes
  std::shared_ptr<rclcpp::Node>& get_node();
  rclcpp::executors::MultiThreadedExecutor::SharedPtr get_executor() const;
  // Enable subclass to override creation of resource manager to use 
  // modified GZResourceManager
  virtual std::unique_ptr<hardware_interface::ResourceManager> create_resource_manager(
    sim::EntityComponentManager& _ecm,
    std::map<std::string, sim::Entity>& enabledJoints);

private:
  /// \brief Private data pointer.
  std::unique_ptr<GazeboSimROS2ControlPluginPrivate> dataPtr;
};
}  // namespace gz_ros2_control

#endif  // GZ_ROS2_CONTROL__GZ_ROS2_CONTROL_PLUGIN_HPP_
