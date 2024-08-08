/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#ifndef IOROS_H
#define IOROS_H

#include "ros/ros.h"
#include "interface/IOInterface.h"
#include "unitree_legged_msgs/LowCmd.h"
#include "unitree_legged_msgs/LowState.h"
#include "unitree_legged_msgs/MotorCmd.h"
#include "unitree_legged_msgs/MotorState.h"
#include <sensor_msgs/Imu.h>
#include <string>

class IOROS : public IOInterface{
public:
IOROS();
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
ros::NodeHandle _nm;
ros::Subscriber _servo_sub[12], _imu_sub;
ros::Publisher _servo_pub[12];
unitree_legged_msgs::LowCmd _lowCmd;
unitree_legged_msgs::LowState _lowState;
std::string _robot_name;

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::Imu & msg);

void FRhipCallback(const unitree_legged_msgs::MotorState& msg);
void FRthighCallback(const unitree_legged_msgs::MotorState& msg);
void FRcalfCallback(const unitree_legged_msgs::MotorState& msg);

void FLhipCallback(const unitree_legged_msgs::MotorState& msg);
void FLthighCallback(const unitree_legged_msgs::MotorState& msg);
void FLcalfCallback(const unitree_legged_msgs::MotorState& msg);

void RRhipCallback(const unitree_legged_msgs::MotorState& msg);
void RRthighCallback(const unitree_legged_msgs::MotorState& msg);
void RRcalfCallback(const unitree_legged_msgs::MotorState& msg);

void RLhipCallback(const unitree_legged_msgs::MotorState& msg);
void RLthighCallback(const unitree_legged_msgs::MotorState& msg);
void RLcalfCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS

#ifdef COMPILE_WITH_ROS2_MB

#ifndef IOROS_H
#define IOROS_H

#include "rclcpp/rclcpp.hpp"
#include "interface/IOInterface.h"
#include "ros2_unitree_legged_msgs/msg/low_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/low_state.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_cmd.hpp"
#include "ros2_unitree_legged_msgs/msg/motor_state.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <string>
#include <vector>

class IOROS : public IOInterface{
public:
IOROS(rclcpp::Node::SharedPtr node_ptr);
~IOROS();
void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);

private:
static void RosShutDown(int sig);
void sendCmd(const LowlevelCmd *cmd);
void recvState(LowlevelState *state);
rclcpp::Node::SharedPtr _nm;
rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_sub;
rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _joint_state_sub;
std::vector<rclcpp::Subscription<ros2_unitree_legged_msgs::msg::MotorState>::SharedPtr> _servo_sub;
std::vector<rclcpp::Publisher<ros2_unitree_legged_msgs::msg::MotorCmd>::SharedPtr> _servo_pub;
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr _joint_cmd_pub;
ros2_unitree_legged_msgs::msg::LowCmd _lowCmd;
ros2_unitree_legged_msgs::msg::LowState _lowState;
std::string _robot_name;
std::unordered_map<std::string, int> joint_index_map;
std::thread executor_thread;

sensor_msgs::msg::JointState _joint_state;
std_msgs::msg::Float64MultiArray _joint_cmd;

//repeated functions for multi-thread
void initRecv();
void initSend();

//Callback functions for ROS
void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

// Functionality of these callbacks is overriden in ROS 2 interface
void FRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void FRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void FRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

void FLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void FLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void FLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

void RRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void RRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void RRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

void RLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void RLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);
void RLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg);

// Callback functions for ROS 2 interface
void initializeJointIndexMap();
void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);

};

#endif  // IOROS_H

#endif  // COMPILE_WITH_ROS2_MB