/*******************************************************************************
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

#ifndef OPEN_MANIPULATOR_CONTROLLER_H
#define OPEN_MANIPULATOR_CONTROLLER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <boost/thread.hpp>
#include <unistd.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"
#include "open_manipulator_msgs/SetDrawingTrajectory.h"
#include "open_manipulator_msgs/SetActuatorState.h"
#include "open_manipulator_msgs/OpenManipulatorState.h"

#include "open_manipulator_libs/open_manipulator.h"

namespace open_manipulator_controller
{

class OpenManipulatorController
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  // ROS Parameters
  bool using_platform_;
  bool using_moveit_;
  double control_period_;

  // ROS Publisher
  ros::Publisher open_manipulator_states_pub_;
  std::vector<ros::Publisher> open_manipulator_kinematics_pose_pub_;
  ros::Publisher open_manipulator_joint_states_pub_;
  ros::Publisher open_manipulator_controller_state_pub_;
  std::vector<ros::Publisher> gazebo_goal_joint_position_pub_;

  // ROS Subscribers
  ros::Subscriber open_manipulator_option_sub_;

  // ROS Action Server
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> follow_joint_trajectory_server_;
  control_msgs::FollowJointTrajectoryFeedback follow_joint_trajectory_feedback_;
  control_msgs::FollowJointTrajectoryResult follow_joint_trajectory_result_;

  // ROS Service Server
  ros::ServiceServer goal_joint_space_path_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_pose_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_position_server_;
  ros::ServiceServer goal_joint_space_path_to_kinematics_orientation_server_;
  ros::ServiceServer goal_task_space_path_server_;
  ros::ServiceServer goal_task_space_path_position_only_server_;
  ros::ServiceServer goal_task_space_path_orientation_only_server_;
  ros::ServiceServer goal_joint_space_path_from_present_server_;
  ros::ServiceServer goal_task_space_path_from_present_position_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_orientation_only_server_;
  ros::ServiceServer goal_task_space_path_from_present_server_;
  ros::ServiceServer goal_tool_control_server_;
  ros::ServiceServer set_actuator_state_server_;
  ros::ServiceServer goal_drawing_trajectory_server_;

  // MoveIt! interface
  moveit::planning_interface::MoveGroupInterface* move_group_;
  trajectory_msgs::JointTrajectory joint_trajectory_;

  // Thread parameter
  pthread_t timer_thread_;
  pthread_attr_t attr_;

  // Related robotis_manipulator
  OpenManipulator open_manipulator_;

  // flag parameter
  bool timer_thread_state_;
  bool moveit_plan_state_;

 public:

  OpenManipulatorController(std::string usb_port, std::string baud_rate);
  ~OpenManipulatorController();

  void publishCallback(const ros::TimerEvent&);

  void initPublisher();
  void initSubscriber();

  void initServer();

  void openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg);

  double getControlPeriod(void){return control_period_;}

  void goalFollowJointTrajectoryCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal);

  bool goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                  open_manipulator_msgs::SetJointPosition::Response &res);

  bool goalJointSpacePathToKinematicsPoseCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                  open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsPositionCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                  open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathToKinematicsOrientationCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                  open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                 open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathPositionOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                             open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathOrientationOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalJointSpacePathFromPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                             open_manipulator_msgs::SetJointPosition::Response &res);

  bool goalTaskSpacePathFromPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                            open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentPositionOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                        open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalTaskSpacePathFromPresentOrientationOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                           open_manipulator_msgs::SetKinematicsPose::Response &res);

  bool goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                               open_manipulator_msgs::SetJointPosition::Response &res);

  bool setActuatorStateCallback(open_manipulator_msgs::SetActuatorState::Request  &req,
                                open_manipulator_msgs::SetActuatorState::Response &res);

  bool goalDrawingTrajectoryCallback(open_manipulator_msgs::SetDrawingTrajectory::Request  &req,
                                     open_manipulator_msgs::SetDrawingTrajectory::Response &res);

  void startTimerThread();
  static void *timerThread(void *param);

  void moveitPublishGoal(uint32_t step_cnt, double* time_from_start);
  void moveitPublishFeedback();
  void moveitTimer(double present_time);
  void process(double time);

  void publishOpenManipulatorStates();
  void publishKinematicsPose();
  void publishStates();
  void publishGazeboCommand();

  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::JointPosition msg);
  bool calcPlannedPath(const std::string planning_group, open_manipulator_msgs::KinematicsPose msg);

};
}

#endif //OPEN_MANIPULATOR_CONTROLLER_H
