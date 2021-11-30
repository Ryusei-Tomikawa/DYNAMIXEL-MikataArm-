﻿/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "../include/open_manipulator_libs/open_manipulator.h"

OpenManipulator::OpenManipulator()
{}
OpenManipulator::~OpenManipulator()
{
  delete kinematics_;
  delete actuator_;
  delete tool_;
  for(uint8_t index = 0; index < CUSTOM_TRAJECTORY_SIZE; index++)
    delete custom_trajectory_[index];
}

void OpenManipulator::initOpenManipulator(bool using_actual_robot_state, STRING usb_port, STRING baud_rate, float control_loop_time)
{
  /*****************************************************************************
  ** Initialize Manipulator Parameter 
  *****************************************************************************/
  addWorld("world",   // world name
           "joint1"); // child name

  addJoint("joint1", // my name
           "world",  // parent name
           "joint2", // child name
           math::vector3(0.0, 0.0, 0.0515), // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Z_AXIS, // axis of rotation
           11,     // actuator id
           M_PI*0.916,   // max joint limit (3.14 rad)
           -M_PI*0.916); // min joint limit (-3.14 rad)

  addJoint("joint2", // my name
           "joint1", // parent name
           "joint3", // child name
           math::vector3(0.0, 0.0, 0.05475), // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS, // axis of rotation
           12,     // actuator id
           M_PI*0.5965,   // max joint limit (3.14 rad)
           -M_PI*0.6679); // min joint limit (-3.14 rad)

  addJoint("joint3", // my name
           "joint2", // parent name
           "joint4", // child name
           math::vector3(0.024, 0.0, 0.1684), // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS, // axis of rotation
           13,     // actuator id
           M_PI*0.448,   // max joint limit (3.14 rad)
           -M_PI*0.5735); // min joint limit (-3.14 rad)

  addJoint("joint4", // my name
           "joint3", // parent name
           "joint5",   // child name
           math::vector3(0.062, 0.0, 0.0), // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           X_AXIS, // axis of rotation
           14,     // actuator id
           M_PI*0.994,   // max joint limit (3.14 rad)
           -M_PI*0.994); // min joint limit (-3.14 rad)

  addJoint("joint5", // my name
           "joint4", // parent name
           "joint6",   // child name
           math::vector3(0.088, 0.0, 0.0), // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           Y_AXIS, // axis of rotation
           15,     // actuator id
           M_PI*0.7073,   // max joint limit (3.14 rad)
           -M_PI*0.5735); // min joint limit (-3.14 rad)

  addJoint("joint6", // my name
           "joint5", // parent name
           "gripper",   // child name
           math::vector3(0.062, 0.0, 0.0), // relative position
           math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
           X_AXIS, // axis of rotation
           16,     // actuator id
           M_PI*0.994,   // max joint limit (3.14 rad)
           -M_PI*0.994); // min joint limit (-3.14 rad)

  addTool("gripper",   // my name
          "joint6", // parent name
          math::vector3(0.08375, 0.0, 0.0), // relative position
          math::convertRPYToRotationMatrix(0.0, 0.0, 0.0), // relative orientation
          17,     // actuator id
          1.06,   // max gripper limit
          -0.72, // min gripper limit
          -1.0);  // Coefficient


  /*****************************************************************************
  ** Initialize Kinematics 
  *****************************************************************************/
  kinematics_ = new kinematics::SolverCustomizedforOMChain();
//  kinematics_ = new kinematics::SolverUsingCRAndSRPositionOnlyJacobian();
  addKinematics(kinematics_);

  if(using_actual_robot_state)
  {
    /*****************************************************************************
    ** Initialize Joint Actuator
    *****************************************************************************/
    // actuator_ = new dynamixel::JointDynamixel();
    actuator_ = new dynamixel::JointDynamixelProfileControl(control_loop_time);
    
    // Set communication arguments
    STRING dxl_comm_arg[2] = {usb_port, baud_rate};
    void *p_dxl_comm_arg = &dxl_comm_arg;

    // Set joint actuator id
    std::vector<uint8_t> jointDxlId;
    jointDxlId.push_back(11);
    jointDxlId.push_back(12);
    jointDxlId.push_back(13);
    jointDxlId.push_back(14);
    jointDxlId.push_back(15);
    jointDxlId.push_back(16);
    addJointActuator(JOINT_DYNAMIXEL, actuator_, jointDxlId, p_dxl_comm_arg);

    // Set joint actuator control mode
    STRING joint_dxl_mode_arg = "position_mode";
    void *p_joint_dxl_mode_arg = &joint_dxl_mode_arg;
    setJointActuatorMode(JOINT_DYNAMIXEL, jointDxlId, p_joint_dxl_mode_arg);


    /*****************************************************************************
    ** Initialize Tool Actuator
    *****************************************************************************/
    tool_ = new dynamixel::GripperDynamixel();

    uint8_t gripperDxlId = 17;
    addToolActuator(TOOL_DYNAMIXEL, tool_, gripperDxlId, p_dxl_comm_arg);

    // Set gripper actuator control mode
    STRING gripper_dxl_mode_arg = "current_based_position_mode";
    void *p_gripper_dxl_mode_arg = &gripper_dxl_mode_arg;
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_mode_arg);

    // Set gripper actuator parameter
    STRING gripper_dxl_opt_arg[2];
    void *p_gripper_dxl_opt_arg = &gripper_dxl_opt_arg;
    gripper_dxl_opt_arg[0] = "Profile_Acceleration";
    gripper_dxl_opt_arg[1] = "20";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    gripper_dxl_opt_arg[0] = "Profile_Velocity";
    gripper_dxl_opt_arg[1] = "200";
    setToolActuatorMode(TOOL_DYNAMIXEL, p_gripper_dxl_opt_arg);

    // Enable All Actuators 
    enableAllActuator();

    // Receive current angles from all actuators 
    receiveAllJointActuatorValue();
    receiveAllToolActuatorValue();
  }

  /*****************************************************************************
  ** Initialize Custom Trajectory
  *****************************************************************************/
  custom_trajectory_[0] = new custom_trajectory::Line();
  custom_trajectory_[1] = new custom_trajectory::Circle();
  custom_trajectory_[2] = new custom_trajectory::Rhombus();
  custom_trajectory_[3] = new custom_trajectory::Heart();

  addCustomTrajectory(CUSTOM_TRAJECTORY_LINE, custom_trajectory_[0]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, custom_trajectory_[1]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, custom_trajectory_[2]);
  addCustomTrajectory(CUSTOM_TRAJECTORY_HEART, custom_trajectory_[3]);
}

void OpenManipulator::processOpenManipulator(double present_time)
{
  JointWaypoint goal_joint_value = getJointGoalValueFromTrajectory(present_time);
  JointWaypoint goal_tool_value  = getToolGoalValue();

  receiveAllJointActuatorValue();
  receiveAllToolActuatorValue();
  if(goal_joint_value.size() != 0) sendAllJointActuatorValue(goal_joint_value);
  if(goal_tool_value.size() != 0) sendAllToolActuatorValue(goal_tool_value);
  solveForwardKinematics();
}
