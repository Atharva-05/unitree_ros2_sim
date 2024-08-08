/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#ifdef COMPILE_WITH_MOVE_BASE

#include "FSM/State_move_base.h"

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    initRecv();
    
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
    ros::spinOnce();
}

void State_move_base::twistCallback(const geometry_msgs::Twist& msg){
    _vx = msg.linear.x;
    _vy = msg.linear.y;
    _wz = msg.angular.z;
}

void State_move_base::initRecv(){
    _cmdSub = _nm.subscribe("/cmd_vel", 1, &State_move_base::twistCallback, this);
}

#endif  // COMPILE_WITH_MOVE_BASE

#ifdef COMPILE_WITH_ROS2_MB

#include "FSM/State_move_base.h"

State_move_base::State_move_base(CtrlComponents *ctrlComp)
    :State_Trotting(ctrlComp){
    _stateName = FSMStateName::MOVE_BASE;
    _stateNameString = "move_base";
    _nm = rclcpp::Node::make_shared("state_mb");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
        rclcpp::ExecutorOptions(), 1
    );
    executor->add_node(_nm);
    executor_thread = std::thread([executor] (){
        executor->spin();
    });
    executor_thread.detach();
    initRecv();
}

FSMStateName State_move_base::checkChange(){
    if(_lowState->userCmd == UserCommand::L2_B){
        return FSMStateName::PASSIVE;
    }
    else if(_lowState->userCmd == UserCommand::L2_A){
        return FSMStateName::FIXEDSTAND;
    }
    else{
        return FSMStateName::MOVE_BASE;
    }
}

void State_move_base::getUserCmd(){
    setHighCmd(_vx, _vy, _wz);
}

void State_move_base::twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg){
    _vx = msg->linear.x;
    _vy = msg->linear.y;
    _wz = msg->angular.z;
}

void State_move_base::initRecv(){
    std::cout << "Initialized cmd vel sub" << std::endl;
    _cmdSub = _nm->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&State_move_base::twistCallback, this, std::placeholders::_1));
}

#endif  // COMPILE_WITH_ROS2_MB