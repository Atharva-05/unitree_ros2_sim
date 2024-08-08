/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_ROS

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>

void RosShutDown(int sig){
	ROS_INFO("ROS interface shutting down!");
	ros::shutdown();
}

IOROS::IOROS():IOInterface(){
    std::cout << "The control interface for ROS 2 Gazebo simulation" << std::endl;
    ros::param::get("/robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // start subscriber
    initRecv();
    ros::AsyncSpinner subSpinner(1); // one threads
    subSpinner.start();
    usleep(300000);     //wait for subscribers start
    // initialize publisher
    initSend();   

    signal(SIGINT, RosShutDown);

    cmdPanel = new KeyBoard();
}

IOROS::~IOROS(){
    delete cmdPanel;
    ros::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    for(int i(0); i < 12; ++i){
        _lowCmd.motorCmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motorCmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motorCmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motorCmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motorCmd[i].Kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motorCmd[i].Kp = lowCmd->motorCmd[i].Kp;
    }
    for(int m(0); m < 12; ++m){
        _servo_pub[m].publish(_lowCmd.motorCmd[m]);
    }
    ros::spinOnce();
}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
        state->motorState[i].ddq = _lowState.motorState[i].ddq;
        state->motorState[i].tauEst = _lowState.motorState[i].tauEst;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend(){
    _servo_pub[0] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_hip_controller/command", 1);
    _servo_pub[1] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_thigh_controller/command", 1);
    _servo_pub[2] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FR_calf_controller/command", 1);
    _servo_pub[3] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_hip_controller/command", 1);
    _servo_pub[4] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_thigh_controller/command", 1);
    _servo_pub[5] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/FL_calf_controller/command", 1);
    _servo_pub[6] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_hip_controller/command", 1);
    _servo_pub[7] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_thigh_controller/command", 1);
    _servo_pub[8] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RR_calf_controller/command", 1);
    _servo_pub[9] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_hip_controller/command", 1);
    _servo_pub[10] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_thigh_controller/command", 1);
    _servo_pub[11] = _nm.advertise<unitree_legged_msgs::MotorCmd>("/" + _robot_name + "_gazebo/RL_calf_controller/command", 1);
}

void IOROS::initRecv(){
    _imu_sub = _nm.subscribe("/trunk_imu", 1, &IOROS::imuCallback, this);
    _servo_sub[0] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_hip_controller/state", 1, &IOROS::FRhipCallback, this);
    _servo_sub[1] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_thigh_controller/state", 1, &IOROS::FRthighCallback, this);
    _servo_sub[2] = _nm.subscribe("/" + _robot_name + "_gazebo/FR_calf_controller/state", 1, &IOROS::FRcalfCallback, this);
    _servo_sub[3] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_hip_controller/state", 1, &IOROS::FLhipCallback, this);
    _servo_sub[4] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_thigh_controller/state", 1, &IOROS::FLthighCallback, this);
    _servo_sub[5] = _nm.subscribe("/" + _robot_name + "_gazebo/FL_calf_controller/state", 1, &IOROS::FLcalfCallback, this);
    _servo_sub[6] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_hip_controller/state", 1, &IOROS::RRhipCallback, this);
    _servo_sub[7] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_thigh_controller/state", 1, &IOROS::RRthighCallback, this);
    _servo_sub[8] = _nm.subscribe("/" + _robot_name + "_gazebo/RR_calf_controller/state", 1, &IOROS::RRcalfCallback, this);
    _servo_sub[9] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_hip_controller/state", 1, &IOROS::RLhipCallback, this);
    _servo_sub[10] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_thigh_controller/state", 1, &IOROS::RLthighCallback, this);
    _servo_sub[11] = _nm.subscribe("/" + _robot_name + "_gazebo/RL_calf_controller/state", 1, &IOROS::RLcalfCallback, this);
}

void IOROS::imuCallback(const sensor_msgs::Imu & msg)
{ 
    _lowState.imu.quaternion[0] = msg.orientation.w;
    _lowState.imu.quaternion[1] = msg.orientation.x;
    _lowState.imu.quaternion[2] = msg.orientation.y;
    _lowState.imu.quaternion[3] = msg.orientation.z;

    _lowState.imu.gyroscope[0] = msg.angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg.angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg.angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg.linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg.linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg.linear_acceleration.z;
}

void IOROS::FRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[0].mode = msg.mode;
    _lowState.motorState[0].q = msg.q;
    _lowState.motorState[0].dq = msg.dq;
    _lowState.motorState[0].tauEst = msg.tauEst;
}

void IOROS::FRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[1].mode = msg.mode;
    _lowState.motorState[1].q = msg.q;
    _lowState.motorState[1].dq = msg.dq;
    _lowState.motorState[1].tauEst = msg.tauEst;
}

void IOROS::FRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[2].mode = msg.mode;
    _lowState.motorState[2].q = msg.q;
    _lowState.motorState[2].dq = msg.dq;
    _lowState.motorState[2].tauEst = msg.tauEst;
}

void IOROS::FLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[3].mode = msg.mode;
    _lowState.motorState[3].q = msg.q;
    _lowState.motorState[3].dq = msg.dq;
    _lowState.motorState[3].tauEst = msg.tauEst;
}

void IOROS::FLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[4].mode = msg.mode;
    _lowState.motorState[4].q = msg.q;
    _lowState.motorState[4].dq = msg.dq;
    _lowState.motorState[4].tauEst = msg.tauEst;
}

void IOROS::FLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[5].mode = msg.mode;
    _lowState.motorState[5].q = msg.q;
    _lowState.motorState[5].dq = msg.dq;
    _lowState.motorState[5].tauEst = msg.tauEst;
}

void IOROS::RRhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[6].mode = msg.mode;
    _lowState.motorState[6].q = msg.q;
    _lowState.motorState[6].dq = msg.dq;
    _lowState.motorState[6].tauEst = msg.tauEst;
}

void IOROS::RRthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[7].mode = msg.mode;
    _lowState.motorState[7].q = msg.q;
    _lowState.motorState[7].dq = msg.dq;
    _lowState.motorState[7].tauEst = msg.tauEst;
}

void IOROS::RRcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[8].mode = msg.mode;
    _lowState.motorState[8].q = msg.q;
    _lowState.motorState[8].dq = msg.dq;
    _lowState.motorState[8].tauEst = msg.tauEst;
}

void IOROS::RLhipCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[9].mode = msg.mode;
    _lowState.motorState[9].q = msg.q;
    _lowState.motorState[9].dq = msg.dq;
    _lowState.motorState[9].tauEst = msg.tauEst;
}

void IOROS::RLthighCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[10].mode = msg.mode;
    _lowState.motorState[10].q = msg.q;
    _lowState.motorState[10].dq = msg.dq;
    _lowState.motorState[10].tauEst = msg.tauEst;
}

void IOROS::RLcalfCallback(const unitree_legged_msgs::MotorState& msg)
{
    _lowState.motorState[11].mode = msg.mode;
    _lowState.motorState[11].q = msg.q;
    _lowState.motorState[11].dq = msg.dq;
    _lowState.motorState[11].tauEst = msg.tauEst;
}

#endif  // COMPILE_WITH_ROS

// ROS 2 usage

#ifdef COMPILE_WITH_ROS2_MB

#include "interface/IOROS.h"
#include "interface/KeyBoard.h"
#include <iostream>
#include <unistd.h>
#include <csignal>
#include <string.h>
#include <thread>
#include <memory>

void IOROS::RosShutDown(int sig){

	// RCLCPP_INFO_ONCE(_nm->get_logger(), "ROS 2 interface shutting down!");
    std::cout << "ROS 2 interface shutting down!" << std::endl;
    // executor_thread.join();
	rclcpp::shutdown();
}

// void IOROS::initializeJointIndexMap(){
//     joint_index_map["FL_hip_joint"] = 0;
//     joint_index_map["FL_thigh_joint"] = 1;         
//     joint_index_map["FL_calf_joint"] = 2;
//     joint_index_map["FR_hip_joint"] = 3;
//     joint_index_map["FR_thigh_joint"] = 4;
//     joint_index_map["FR_calf_joint"] = 5;         
//     joint_index_map["RL_hip_joint"] = 6;
//     joint_index_map["RL_thigh_joint"] = 7;         
//     joint_index_map["RL_calf_joint"] = 8;
//     joint_index_map["RR_hip_joint"] = 9;          
//     joint_index_map["RR_thigh_joint"] = 10;        
//     joint_index_map["RR_calf_joint"] = 11;
// }

IOROS::IOROS(rclcpp::Node::SharedPtr node_ptr) : IOInterface(){
    _nm = node_ptr;
    
    std::cout << "The control interface for ROS 2 Gazebo simulation" << std::endl;
    _nm->declare_parameter("robot_name", "go2");
    _nm->get_parameter("robot_name", _robot_name);
    std::cout << "robot_name: " << _robot_name << std::endl;

    // initialize map
    // initializeJointIndexMap();

    // _joint_cmd = std_msgs::msg::Float64MultiArray();
    // for(int i = 0; i < 12; i++){
    //     _joint_cmd.data.push_back(0.0);
    // }
    
    // start subscriber
    initRecv();
    std::cout << "init recv" << std::endl;

    // ros::AsyncSpinner subSpinner(1); // one threads
    // subSpinner.start();

    // ROS 2 equivalent
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 1
    );
    
    executor->add_node(_nm);
    executor_thread = std::thread([executor] (){
        executor->spin();
    });
    executor_thread.detach();
    std::cout << "executor thread detached and running" << std::endl;

    usleep(300000);     // wait for subscribers start

    initSend();   

    signal(SIGINT, IOROS::RosShutDown);

    cmdPanel = new KeyBoard();
    
}

IOROS::~IOROS(){
    delete cmdPanel;
    rclcpp::shutdown();
}

void IOROS::sendRecv(const LowlevelCmd *cmd, LowlevelState *state){
    sendCmd(cmd);
    recvState(state);

    state->userCmd = cmdPanel->getUserCmd();
    state->userValue = cmdPanel->getUserValue();
}

void IOROS::sendCmd(const LowlevelCmd *lowCmd){
    
    for(int i(0); i < 12; ++i){
        _lowCmd.motor_cmd[i].mode = lowCmd->motorCmd[i].mode;
        _lowCmd.motor_cmd[i].q = lowCmd->motorCmd[i].q;
        _lowCmd.motor_cmd[i].dq = lowCmd->motorCmd[i].dq;
        _lowCmd.motor_cmd[i].tau = lowCmd->motorCmd[i].tau;
        _lowCmd.motor_cmd[i].kd = lowCmd->motorCmd[i].Kd;
        _lowCmd.motor_cmd[i].kp = lowCmd->motorCmd[i].Kp;

    }
    for(int m(0); m < 12; ++m){
        _servo_pub[m]->publish(_lowCmd.motor_cmd[m]);
    }
    // _joint_cmd_pub->publish(_joint_cmd);
    // rclcpp::spin_once();


}

void IOROS::recvState(LowlevelState *state){
    for(int i(0); i < 12; ++i){
        state->motorState[i].q = _lowState.motor_state[i].q;
        state->motorState[i].dq = _lowState.motor_state[i].dq;
        state->motorState[i].ddq = _lowState.motor_state[i].ddq;
        state->motorState[i].tauEst = _lowState.motor_state[i].tau_est;
    }
    for(int i(0); i < 3; ++i){
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];
}

void IOROS::initSend(){
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/FR_hip_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/FR_thigh_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/FR_calf_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/FL_hip_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/FL_thigh_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/FL_calf_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/RR_hip_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/RR_thigh_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/RR_calf_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/RL_hip_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/RL_thigh_controller/command", 1));
    _servo_pub.push_back(_nm->create_publisher<ros2_unitree_legged_msgs::msg::MotorCmd>("/RL_calf_controller/command", 1));
    // _joint_cmd_pub = _nm->create_publisher<std_msgs::msg::Float64MultiArray>("/joint_group_position_controller/commands", 1);
}

void IOROS::initRecv(){
    _imu_sub = _nm->create_subscription<sensor_msgs::msg::Imu>("/imu_plugin/out", 1, std::bind(&IOROS::imuCallback, this, std::placeholders::_1));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/FR_hip_controller/state", 1, std::bind(&IOROS::FRhipCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/FR_thigh_controller/state", 1, std::bind(&IOROS::FRthighCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/FR_calf_controller/state", 1, std::bind(&IOROS::FRcalfCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/FL_hip_controller/state", 1, std::bind(&IOROS::FLhipCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/FL_thigh_controller/state", 1, std::bind(&IOROS::FLthighCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/FL_calf_controller/state", 1, std::bind(&IOROS::FLcalfCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/RR_hip_controller/state", 1, std::bind(&IOROS::RRhipCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/RR_thigh_controller/state", 1, std::bind(&IOROS::RRthighCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/RR_calf_controller/state", 1, std::bind(&IOROS::RRcalfCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/RL_hip_controller/state", 1, std::bind(&IOROS::RLhipCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/RL_thigh_controller/state", 1, std::bind(&IOROS::RLthighCallback, this, std::placeholders::_1)));
    _servo_sub.push_back(_nm->create_subscription<ros2_unitree_legged_msgs::msg::MotorState>("/RL_calf_controller/state", 1, std::bind(&IOROS::RLcalfCallback, this, std::placeholders::_1)));
    // _joint_state_sub = _nm->create_subscription<sensor_msgs::msg::JointState>("/joint_states", 1, std::bind(&IOROS::jointStateCallback, this, std::placeholders::_1));
}

void IOROS::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    _lowState.imu.quaternion[0] = msg->orientation.w;
    _lowState.imu.quaternion[1] = msg->orientation.x;
    _lowState.imu.quaternion[2] = msg->orientation.y;
    _lowState.imu.quaternion[3] = msg->orientation.z;

    _lowState.imu.gyroscope[0] = msg->angular_velocity.x;
    _lowState.imu.gyroscope[1] = msg->angular_velocity.y;
    _lowState.imu.gyroscope[2] = msg->angular_velocity.z;
    
    _lowState.imu.accelerometer[0] = msg->linear_acceleration.x;
    _lowState.imu.accelerometer[1] = msg->linear_acceleration.y;
    _lowState.imu.accelerometer[2] = msg->linear_acceleration.z;
}


void IOROS::FRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[0].mode = msg->mode;
    _lowState.motor_state[0].q = msg->q;
    _lowState.motor_state[0].dq = msg->dq;
    _lowState.motor_state[0].tau_est = msg->tau_est;
}

void IOROS::FRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[1].mode = msg->mode;
    _lowState.motor_state[1].q = msg->q;
    _lowState.motor_state[1].dq = msg->dq;
    _lowState.motor_state[1].tau_est = msg->tau_est;
}

void IOROS::FRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[2].mode = msg->mode;
    _lowState.motor_state[2].q = msg->q;
    _lowState.motor_state[2].dq = msg->dq;
    _lowState.motor_state[2].tau_est = msg->tau_est;
}

void IOROS::FLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[3].mode = msg->mode;
    _lowState.motor_state[3].q = msg->q;
    _lowState.motor_state[3].dq = msg->dq;
    _lowState.motor_state[3].tau_est = msg->tau_est;
}

void IOROS::FLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[4].mode = msg->mode;
    _lowState.motor_state[4].q = msg->q;
    _lowState.motor_state[4].dq = msg->dq;
    _lowState.motor_state[4].tau_est = msg->tau_est;
}

void IOROS::FLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[5].mode = msg->mode;
    _lowState.motor_state[5].q = msg->q;
    _lowState.motor_state[5].dq = msg->dq;
    _lowState.motor_state[5].tau_est = msg->tau_est;
}

void IOROS::RRhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[6].mode = msg->mode;
    _lowState.motor_state[6].q = msg->q;
    _lowState.motor_state[6].dq = msg->dq;
    _lowState.motor_state[6].tau_est = msg->tau_est;
}

void IOROS::RRthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[7].mode = msg->mode;
    _lowState.motor_state[7].q = msg->q;
    _lowState.motor_state[7].dq = msg->dq;
    _lowState.motor_state[7].tau_est = msg->tau_est;
}

void IOROS::RRcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[8].mode = msg->mode;
    _lowState.motor_state[8].q = msg->q;
    _lowState.motor_state[8].dq = msg->dq;
    _lowState.motor_state[8].tau_est = msg->tau_est;
}

void IOROS::RLhipCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[9].mode = msg->mode;
    _lowState.motor_state[9].q = msg->q;
    _lowState.motor_state[9].dq = msg->dq;
    _lowState.motor_state[9].tau_est = msg->tau_est;
}

void IOROS::RLthighCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[10].mode = msg->mode;
    _lowState.motor_state[10].q = msg->q;
    _lowState.motor_state[10].dq = msg->dq;
    _lowState.motor_state[10].tau_est = msg->tau_est;
}

void IOROS::RLcalfCallback(const ros2_unitree_legged_msgs::msg::MotorState::SharedPtr msg)
{
    _lowState.motor_state[11].mode = msg->mode;
    _lowState.motor_state[11].q = msg->q;
    _lowState.motor_state[11].dq = msg->dq;
    _lowState.motor_state[11].tau_est = msg->tau_est;
}


#endif  // COMPILE_WITH_ROS2_MB