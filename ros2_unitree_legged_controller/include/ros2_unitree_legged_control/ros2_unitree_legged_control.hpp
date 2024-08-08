// Copyright 2020 PAL Robotics S.L.
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

#ifndef ROS2_UNITREE_LEGGED_CONTROL__JOINT_GROUP_EFFORT_CONTROLLER_HPP_
#define ROS2_UNITREE_LEGGED_CONTROL__JOINT_GROUP_EFFORT_CONTROLLER_HPP_

#include <string>

#include "controller_interface/controller_interface.hpp"
#include "controller_interface/helpers.hpp"
#include "ros2_unitree_legged_control/visibility_control.h"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"   // Command type
#include "ros2_unitree_legged_msgs/msg/motor_state.hpp"   // State type
#include "ros2_unitree_legged_control_parameters.hpp"

#include "control_toolbox/pid.hpp"
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition.hpp>
#include "std_msgs/msg/float64.hpp"
#include "ros2_unitree_legged_control/unitree_joint_control_tool.h"
#include <urdf/model.h>

#define PMSM      (0x0A)
#define BRAKE     (0x00)
#define PosStopF  (2.146E+9f)
#define VelStopF  (16000.0f)

namespace ros2_unitree_legged_control

{
  using CmdType = ros2_unitree_legged_msgs::msg::MotorCmd;      // Command received (MotorCmd)
  using StateType = ros2_unitree_legged_msgs::msg::MotorState;  // State send (MotorState)
/**
 * \brief Forward command controller for a set of effort controlled joints (linear or angular).
 *
 * This class forwards the commanded efforts down to a set of joints.
 *
 * \param joints Names of the joints to control.
 *
 * Subscribes to:
 * - \b command (std_msgs::msg::Float64MultiArray) : The effort commands to apply.
 */
class UnitreeLeggedController : public controller_interface::ControllerInterface
{
public:
  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  UnitreeLeggedController();

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  ~UnitreeLeggedController() = default;

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  
  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::CallbackReturn on_deactivate
  (const rclcpp_lifecycle::State & previous_state) override;

  ROS2_UNITREE_LEGGED_CONTROL_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  // Unitree specific
  bool isHip, isCalf, isThigh;
  CmdType lastCmd;
  StateType lastState;
  ServoCmd servoCmd;

  void positionLimits(double &position);
  void velocityLimits(double &velocity);
  void effortLimits(double &effort);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup);
  void getGains(double &p, double &i, double &d, double &i_max, double &i_min);
  void publishState();
  void setCmdCallback(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg);

protected:
  /**
   * Derived controllers have to declare parameters in this method.
   * Error handling does not have to be done. It is done in `on_init`-method of this class.
   */
  void declare_parameters();

  /**
   * Derived controllers have to read parameters in this method and set `command_interface_types_`
   * variable. The variable is then used to propagate the command interface configuration to
   * controller manager. The method is called from `on_configure`-method of this class.
   *
   * It is expected that error handling of exceptions is done.
   *
   * \returns controller_interface::CallbackReturn::SUCCESS if parameters are successfully read and
   * their values are allowed, controller_interface::CallbackReturn::ERROR otherwise.
   */
  controller_interface::CallbackReturn read_parameters();

  urdf::Model model_;
  std::string urdf_string_;

  std::string joint_name_;
  int joint_type_; // enumize later
  std::vector<std::string> joint_names_;
  urdf::JointConstSharedPtr joint_;
  std::string state_interface_name_;

  std::vector<std::string> command_interface_types_;

  realtime_tools::RealtimeBuffer<CmdType> rt_command_;
  rclcpp::Subscription<CmdType>::SharedPtr joints_command_subscriber_;
  control_toolbox::Pid pid_controller_;

  std::shared_ptr<ros2_unitree_legged_control::ParamListener> param_listener_;
  ros2_unitree_legged_control::Params params_;

  // boost::scoped_ptr<realtime_tools::RealtimePublisher<StateType>> rt_controller_state_publisher_;
  std::shared_ptr<realtime_tools::RealtimePublisher<StateType>> rt_controller_state_publisher_;
  rclcpp::Publisher<StateType>::SharedPtr state_publisher_;

};

}  // namespace ros2_unitree_legged_control

#endif  // ROS2_UNITREE_LEGGED_CONTROL__JOINT_GROUP_EFFORT_CONTROLLER_HPP_
