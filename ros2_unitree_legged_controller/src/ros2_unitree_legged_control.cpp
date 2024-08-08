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

#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ros2_unitree_legged_control/ros2_unitree_legged_control.hpp"
#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"

namespace ros2_unitree_legged_control
{
UnitreeLeggedController::UnitreeLeggedController()
: controller_interface::ControllerInterface(),
  joints_command_subscriber_(nullptr), rt_controller_state_publisher_(nullptr), state_publisher_(nullptr)
{
  // memset(&lastCmd, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorCmd));
  // memset(&lastState, 0, sizeof(ros2_unitree_legged_msgs::msg::MotorState));
  lastCmd = ros2_unitree_legged_msgs::msg::MotorCmd();
  lastState = ros2_unitree_legged_msgs::msg::MotorState();
  memset(&servoCmd, 0, sizeof(ServoCmd));
}

controller_interface::CallbackReturn UnitreeLeggedController::on_init()
{

  try
  {
    declare_parameters();

    // Initialize variables and pointers
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

void UnitreeLeggedController::declare_parameters()
{
  param_listener_ = std::make_shared<ParamListener>(get_node());
}

controller_interface::CallbackReturn UnitreeLeggedController::read_parameters(){

  // Read joint name here, used in state and interface configuration
  // Custom param struct is generated accodring to yaml file under ros2_unitree_legged_control_parameters.hpp

  if (!param_listener_)
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init. param_listener does not exist");
      return controller_interface::CallbackReturn::ERROR;
    }
    params_ = param_listener_->get_params();

    if (params_.joint_name.empty())
    {
      RCLCPP_ERROR(get_node()->get_logger(), "'joint_name' parameter was empty");
      return controller_interface::CallbackReturn::ERROR;
    }

    if(get_node()->get_parameter("robot_description", urdf_string_)){
      if (!model_.initString(urdf_string_)){
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse URDF string");
        return controller_interface::CallbackReturn::ERROR; 
      }
    }

    joint_name_ = params_.joint_name;
    joint_ = model_.getJoint(joint_name_);

    // if(joint_name_.find("hip") != std::string::npos){
    //   joint_type_ = 0;
    // }
    // else if(joint_name_.find("thigh") != std::string::npos){
    //   joint_type_ = 1;
    // }
    // else if(joint_name_.find("calf") != std::string::npos){
    //   joint_type_ = 2;
    // }

    RCLCPP_INFO_STREAM(get_node()->get_logger(), "Configured joint: " << joint_name_ << "with type: " << joint_type_);

    return controller_interface::CallbackReturn::SUCCESS;
}

void UnitreeLeggedController::setCmdCallback(const ros2_unitree_legged_msgs::msg::MotorCmd::SharedPtr msg){
  lastCmd.mode = msg->mode;
  lastCmd.q = msg->q;
  lastCmd.kp = msg->kp;
  lastCmd.dq = msg->dq;
  lastCmd.kd = msg->kd;
  lastCmd.tau = msg->tau;

  rt_command_.writeFromNonRT(lastCmd); 
}

controller_interface::CallbackReturn UnitreeLeggedController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  auto ret = read_parameters();
  if (ret != controller_interface::CallbackReturn::SUCCESS)
  {
    return ret;
  }

  // command callback
  joints_command_subscriber_ = get_node()->create_subscription<CmdType>(
    "~/command", rclcpp::SystemDefaultsQoS(), std::bind(&UnitreeLeggedController::setCmdCallback, this, std::placeholders::_1));

  state_publisher_ = get_node()->create_publisher<StateType>(
    "~/state", rclcpp::SystemDefaultsQoS()
  );

  // rt_controller_state_publisher_.reset(
  //   new realtime_tools::RealtimePublisher<StateType>(state_publisher_)
  // );
  rt_controller_state_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<StateType>>(state_publisher_);

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
UnitreeLeggedController::command_interface_configuration() const
{
  // controller_interface::InterfaceConfiguration command_interfaces_config;
  // command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // command_interfaces_config.names = command_interface_types_;
  std::vector<std::string> joint_names;
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);

  return {controller_interface::interface_configuration_type::INDIVIDUAL, joint_names};

}

controller_interface::InterfaceConfiguration UnitreeLeggedController::state_interface_configuration()
  const
{ 
  std::vector<std::string> joint_names;
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_POSITION);
  joint_names.push_back(joint_name_ + "/" + hardware_interface::HW_IF_EFFORT);
  return controller_interface::InterfaceConfiguration{controller_interface::interface_configuration_type::INDIVIDUAL, joint_names};
}

controller_interface::CallbackReturn UnitreeLeggedController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{

  // TODO
  //  check if we have all resources defined in the "points" parameter
  //  also verify that we *only* have the resources defined in the "points" parameter
  // ATTENTION(destogl): Shouldn't we use ordered interface all the time?
  // std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
  //   ordered_interfaces;
  // if (
  //   !controller_interface::get_ordered_interfaces(
  //     command_interfaces_, command_interface_types_, std::string(""), ordered_interfaces) ||
  //   command_interface_types_.size() != ordered_interfaces.size())
  // {
  //   RCLCPP_ERROR(
  //     get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
  //     command_interface_types_.size(), ordered_interfaces.size());
  //   return controller_interface::CallbackReturn::ERROR;
  // }

  // reset command buffer if a command came through callback when controller was inactive
  // rt_command_ = realtime_tools::RealtimeBuffer<CmdType>();
  RCLCPP_INFO(get_node()->get_logger(), "accessing state interface");
  double init_pose = state_interfaces_[0].get_value();
  // Runtime error prone?
  RCLCPP_INFO(get_node()->get_logger(), "accessing state interface successful");
  lastCmd.q = (float) init_pose;
  lastState.q = (float) init_pose;
  lastCmd.dq = 0;
  lastState.dq = 0;
  lastCmd.tau = 0;
  lastState.tau_est = 0;
  // rt_command_.initRT(lastCmd);

  RCLCPP_INFO(get_node()->get_logger(), "activate successful");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn UnitreeLeggedController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // reset command buffer
  rt_command_ = realtime_tools::RealtimeBuffer<CmdType>();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type UnitreeLeggedController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // lastCmd = *(*rt_command_.readFromRT());
  auto joint_commands = rt_command_.readFromRT();

  // // no command received yet. This is only okay for feedforward as it does not provide a feedback
  if (!joint_commands)
  {
    publishState();
    return controller_interface::return_type::OK;
  }

  lastCmd = *(joint_commands);
  double currentPos, currentVel, calcTorque;

  if(lastCmd.mode == PMSM) {
    servoCmd.pos = lastCmd.q;
    positionLimits(servoCmd.pos);
    servoCmd.posStiffness = lastCmd.kp;
    if(fabs(lastCmd.q - PosStopF) < 0.00001){
        servoCmd.posStiffness = 0;
    }
    servoCmd.vel = lastCmd.dq;
    velocityLimits(servoCmd.vel);
    servoCmd.velStiffness = lastCmd.kd;
    if(fabs(lastCmd.dq - VelStopF) < 0.00001){
        servoCmd.velStiffness = 0;
    }
    servoCmd.torque = lastCmd.tau;
    effortLimits(servoCmd.torque);
  }
  if(lastCmd.mode == BRAKE) {
    servoCmd.posStiffness = 0;
    servoCmd.vel = 0;
    servoCmd.velStiffness = 20;
    servoCmd.torque = 0;
    effortLimits(servoCmd.torque);
  }

  currentPos = state_interfaces_[0].get_value();    // Position state interface
  currentVel = computeVel(currentPos, (double)lastState.q, (double)lastState.dq, period.seconds());
  calcTorque = computeTorque(currentPos, currentVel, servoCmd);

  // if(joint_name_ == "RL_calf_joint"){
  //   RCLCPP_INFO_STREAM(get_node()->get_logger(), "i: pos: " << currentPos << " state_q: " << (double)lastState.q << " state_dq: " << (double)lastState.dq << " period " << period.seconds() << " cmd_q: " << servoCmd.pos << " cmd_dq: " << servoCmd.vel << " cmd_tau: " << servoCmd.torque  << " cmd_ps: " <<  servoCmd.posStiffness << " cmd_vs: " <<  servoCmd.velStiffness << " o: vel: " << currentVel << " t: " << calcTorque);
  // }      
  // calcTorque = -5.0;
  // effortLimits(calcTorque);
  
  assert (command_interfaces_[0].get_interface_name() == hardware_interface::HW_IF_EFFORT);
  assert (state_interfaces_[0].get_interface_name() == hardware_interface::HW_IF_POSITION);
  assert (state_interfaces_[1].get_interface_name() == hardware_interface::HW_IF_EFFORT);

  command_interfaces_[0].set_value(calcTorque); // Send effort to joint

  lastState.q = (float) currentPos;
  lastState.dq = (float) currentVel;
  lastState.tau_est = (float) state_interfaces_[1].get_value(); // May cause issues? -> Possibly causing issues. Also check (float) conversion issues

  publishState();

  // This is specific to Float64MultiArray (all calls with data)
  // if ((*joint_commands)->data.size() != command_interfaces_.size())
  // {
  //   RCLCPP_ERROR_THROTTLE(
  //     get_node()->get_logger(), *(get_node()->get_clock()), 1000,
  //     "command size (%zu) does not match number of interfaces (%zu)",
  //     (*joint_commands)->data.size(), command_interfaces_.size());
  //   return controller_interface::return_type::ERROR;
  // }

  // for (auto index = 0ul; index < command_interfaces_.size(); ++index)
  // {
  //   // Unitree controller logic starts

  //   // command_interfaces_[index].set_value((*joint_commands)->data[index]);
  //   command_interfaces_[index].set_value((*joint_commands)->data[index]);
  // }

  return controller_interface::return_type::OK;

}

void UnitreeLeggedController::publishState(){
  if (rt_controller_state_publisher_ && rt_controller_state_publisher_->trylock()) {
    rt_controller_state_publisher_->msg_.q = lastState.q;
    rt_controller_state_publisher_->msg_.dq = lastState.dq;
    rt_controller_state_publisher_->msg_.tau_est = lastState.tau_est;
    rt_controller_state_publisher_->unlockAndPublish();    
  }
}

void UnitreeLeggedController::getGains(double &p, double &i, double &d, double &i_max, double &i_min){
  bool dummy;
  pid_controller_.getGains(p,i,d,i_max,i_min,dummy);
}

 void UnitreeLeggedController::getGains(double &p, double &i, double &d, double &i_max, double &i_min, bool &antiwindup)
{
  pid_controller_.getGains(p,i,d,i_max,i_min,antiwindup);
}

void UnitreeLeggedController::positionLimits(double &position){
  clamp(position, joint_->limits->lower, joint_->limits->upper);
  // switch (joint_type_){
  //   double upper, lower;
  //   case 0:
  //   // hip
  //     lower = -0.863, upper = 0.863;
  //     clamp(position, lower, upper);
  //     break;

  //   case 1:
  //   // thigh
  //     lower = -0.686, upper = 4.501;
  //     clamp(position, lower, upper);
  //     break;

  //   case 2:
  //   // calf
  //     lower = -2.818, upper = -0.888;
  //     clamp(position, lower, upper);
  //     break;

  //   default:
  //     break;
  // }
}

void UnitreeLeggedController::velocityLimits(double &velocity){
  clamp(velocity, -joint_->limits->velocity, joint_->limits->velocity);
  // switch (joint_type_){
  //   case 0:
  //   // hip
  //     lower = -30.1, upper = 30.1;
  //     clamp(velocity, lower, upper);
  //     break;

  //   case 1:
  //   // thigh
  //     lower = -30.1, upper = 30.1;
  //     clamp(velocity, lower, upper);
  //     break;

  //   case 2:
  //   // calf
  //     lower = -20.06, upper = 20.06;
  //     clamp(velocity, lower, upper);
  //     break;

  //   default:
  //     break;
  // }
}

void UnitreeLeggedController::effortLimits(double &effort){
  clamp(effort, -joint_->limits->effort, joint_->limits->effort);
  // double upper, lower;
  // switch (joint_type_){
  //   case 0:
  //   // hip
  //     lower = -23.7, upper = 23.7;
  //     clamp(effort, lower, upper);
  //     break;

  //   case 1:
  //   // thigh
  //     lower = -23.7, upper = 23.7;
  //     clamp(effort, lower, upper);
  //     break;

  //   case 2:
  //   // calf
  //     lower = -35.5, upper = 35.5;
  //     clamp(effort, lower, upper);
  //     break;

  //   default:
  //     break;
  // }
}

}  // namespace effort_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ros2_unitree_legged_control::UnitreeLeggedController, controller_interface::ControllerInterface)