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

#include <ros/ros.h>
#include "open_manipulator_controller/open_manipulator_controller.h"

using namespace open_manipulator_controller;

OpenManipulatorController::OpenManipulatorController(std::string usb_port, std::string baud_rate)
    :node_handle_(""),
     priv_node_handle_("~"),
     timer_thread_state_(false),
     moveit_plan_state_(false),
     using_platform_(true),
     control_period_(0.010f),
     follow_joint_trajectory_server_(priv_node_handle_, "arm_controller/follow_joint_trajectory",
                                     boost::bind(&OpenManipulatorController::goalFollowJointTrajectoryCallback, this, _1),
                                     false)
{
  control_period_       = priv_node_handle_.param<double>("control_period", 0.010f);
  using_platform_       = priv_node_handle_.param<bool>("using_platform", false);
  std::string planning_group_name = priv_node_handle_.param<std::string>("planning_group_name", "arm");

  ROS_INFO("using_platform:=%d", using_platform_);

  open_manipulator_.initOpenManipulator(using_platform_, usb_port, baud_rate, control_period_);

  if (using_platform_ == true)        
  {
    log::info("Succeeded to init " + priv_node_handle_.getNamespace());
  }
  else if (using_platform_ == false)  
    log::info("Ready to simulate " +  priv_node_handle_.getNamespace() + " on Gazebo");

}

OpenManipulatorController::~OpenManipulatorController()
{
  timer_thread_state_ = false;
  pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
  log::info("Shutdown the OpenManipulator");
  open_manipulator_.disableAllActuator();
  ros::shutdown();
}

void OpenManipulatorController::startTimerThread()
{
  ////////////////////////////////////////////////////////////////////
  /// Use this when you want to increase the priority of threads.
  ////////////////////////////////////////////////////////////////////
  //  pthread_attr_t attr_;
  //  int error;
  //  struct sched_param param;
  //  pthread_attr_init(&attr_);

  //  error = pthread_attr_setschedpolicy(&attr_, SCHED_RR);
  //  if (error != 0)   log::error("pthread_attr_setschedpolicy error = ", (double)error);
  //  error = pthread_attr_setinheritsched(&attr_, PTHREAD_EXPLICIT_SCHED);
  //  if (error != 0)   log::error("pthread_attr_setinheritsched error = ", (double)error);

  //  memset(&param, 0, sizeof(param));
  //  param.sched_priority = 31;    // RT
  //  error = pthread_attr_setschedparam(&attr_, &param);
  //  if (error != 0)   log::error("pthread_attr_setschedparam error = ", (double)error);

  //  if ((error = pthread_create(&this->timer_thread_, &attr_, this->timerThread, this)) != 0)
  //  {
  //    log::error("Creating timer thread failed!!", (double)error);
  //    exit(-1);
  //  }
  // timer_thread_state_ = true;
  ////////////////////////////////////////////////////////////////////

  int error;
  if ((error = pthread_create(&this->timer_thread_, NULL, this->timerThread, this)) != 0)
  {
    log::error("Creating timer thread failed!!", (double)error);
    exit(-1);
  }
  timer_thread_state_ = true;
}

void *OpenManipulatorController::timerThread(void *param)
{
  OpenManipulatorController *controller = (OpenManipulatorController *) param;
  static struct timespec next_time;
  static struct timespec curr_time;

  clock_gettime(CLOCK_MONOTONIC, &next_time);

  while(controller->timer_thread_state_)
  {
    next_time.tv_sec += (next_time.tv_nsec + ((int)(controller->getControlPeriod() * 1000)) * 1000000) / 1000000000;
    next_time.tv_nsec = (next_time.tv_nsec + ((int)(controller->getControlPeriod() * 1000)) * 1000000) % 1000000000;

    double time = next_time.tv_sec + (next_time.tv_nsec*0.000000001);
    controller->process(time);

    clock_gettime(CLOCK_MONOTONIC, &curr_time);
    /////
    double delta_nsec = controller->getControlPeriod() - ((next_time.tv_sec - curr_time.tv_sec) + ((double)(next_time.tv_nsec - curr_time.tv_nsec)*0.000000001));
    //log::info("control time : ", controller->getControlPeriod() - delta_nsec);
    if(delta_nsec > controller->getControlPeriod())
    {
      //log::warn("Over the control time : ", delta_nsec);
      next_time = curr_time;
    }
    else
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, NULL);
    /////
  }
  return 0;
}

void OpenManipulatorController::initPublisher()
{

  // ros message publisher
  auto opm_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  for (auto const& name:opm_tools_name)
  {
    ros::Publisher pb;
    pb = priv_node_handle_.advertise<open_manipulator_msgs::KinematicsPose>(name + "/kinematics_pose", 10);
    open_manipulator_kinematics_pose_pub_.push_back(pb);
  }
  open_manipulator_states_pub_ = priv_node_handle_.advertise<open_manipulator_msgs::OpenManipulatorState>("states", 10);

  if(using_platform_ == true)
  {
    // open_manipulator_joint_states_pub_ = priv_node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
    open_manipulator_joint_states_pub_ = node_handle_.advertise<sensor_msgs::JointState>("joint_states", 10);
    open_manipulator_controller_state_pub_ = priv_node_handle_.advertise<control_msgs::JointTrajectoryControllerState>("arm_controller/state", 10);
  }
  else
  {
    auto gazebo_joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
    gazebo_joints_name.reserve(gazebo_joints_name.size() + opm_tools_name.size());
    gazebo_joints_name.insert(gazebo_joints_name.end(), opm_tools_name.begin(), opm_tools_name.end());

    for (auto const& name:gazebo_joints_name)
    {
      ros::Publisher pb;
      pb = priv_node_handle_.advertise<std_msgs::Float64>(name + "_position/command", 10);
      gazebo_goal_joint_position_pub_.push_back(pb);
    }
  }
}
void OpenManipulatorController::initSubscriber()
{
  // ros message subscriber
  open_manipulator_option_sub_ = priv_node_handle_.subscribe("option", 10, &OpenManipulatorController::openManipulatorOptionCallback, this);
}

void OpenManipulatorController::initServer()
{
  follow_joint_trajectory_server_.start();

  goal_joint_space_path_server_                     = priv_node_handle_.advertiseService("goal_joint_space_path", &OpenManipulatorController::goalJointSpacePathCallback, this);
  goal_joint_space_path_to_kinematics_pose_server_  = priv_node_handle_.advertiseService("goal_joint_space_path_to_kinematics_pose", &OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback, this);
  goal_joint_space_path_to_kinematics_position_server_  = priv_node_handle_.advertiseService("goal_joint_space_path_to_kinematics_position", &OpenManipulatorController::goalJointSpacePathToKinematicsPositionCallback, this);
  goal_joint_space_path_to_kinematics_orientation_server_  = priv_node_handle_.advertiseService("goal_joint_space_path_to_kinematics_orientation", &OpenManipulatorController::goalJointSpacePathToKinematicsOrientationCallback, this);

  goal_task_space_path_server_                  = priv_node_handle_.advertiseService("goal_task_space_path", &OpenManipulatorController::goalTaskSpacePathCallback, this);
  goal_task_space_path_position_only_server_    = priv_node_handle_.advertiseService("goal_task_space_path_position_only", &OpenManipulatorController::goalTaskSpacePathPositionOnlyCallback, this);
  goal_task_space_path_orientation_only_server_ = priv_node_handle_.advertiseService("goal_task_space_path_orientation_only", &OpenManipulatorController::goalTaskSpacePathOrientationOnlyCallback, this);

  goal_joint_space_path_from_present_server_      = priv_node_handle_.advertiseService("goal_joint_space_path_from_present", &OpenManipulatorController::goalJointSpacePathFromPresentCallback, this);

  goal_task_space_path_from_present_server_                   = priv_node_handle_.advertiseService("goal_task_space_path_from_present", &OpenManipulatorController::goalTaskSpacePathFromPresentCallback, this);
  goal_task_space_path_from_present_position_only_server_     = priv_node_handle_.advertiseService("goal_task_space_path_from_present_position_only", &OpenManipulatorController::goalTaskSpacePathFromPresentPositionOnlyCallback, this);
  goal_task_space_path_from_present_orientation_only_server_  = priv_node_handle_.advertiseService("goal_task_space_path_from_present_orientation_only", &OpenManipulatorController::goalTaskSpacePathFromPresentOrientationOnlyCallback, this);

  goal_tool_control_server_         = priv_node_handle_.advertiseService("goal_tool_control", &OpenManipulatorController::goalToolControlCallback, this);
  set_actuator_state_server_        = priv_node_handle_.advertiseService("set_actuator_state", &OpenManipulatorController::setActuatorStateCallback, this);
  goal_drawing_trajectory_server_   = priv_node_handle_.advertiseService("goal_drawing_trajectory", &OpenManipulatorController::goalDrawingTrajectoryCallback, this);
}

void OpenManipulatorController::openManipulatorOptionCallback(const std_msgs::String::ConstPtr &msg)
{
  if(msg->data == "print_open_manipulator_setting")
    open_manipulator_.printManipulatorSetting();
}

void OpenManipulatorController::goalFollowJointTrajectoryCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
  {
    ROS_DEBUG("goalFollowJointTrajectoryCallback");

    if (!follow_joint_trajectory_feedback_.joint_names.size())
    {
      // ControllerState is not published
      // e.g. during gazebo simulation
      follow_joint_trajectory_feedback_.joint_names = goal->trajectory.joint_names;
    }
    else
    {
      // Check joint_names
      if(goal->trajectory.joint_names.size() != follow_joint_trajectory_feedback_.joint_names.size())
        {
          log::error("Sizes mismatch at goalFollowJointTrajectoryCallback!");
          ROS_ERROR("Aborting Trajectory Execution!");
          follow_joint_trajectory_server_.setAborted();
          return;
        }
      for(uint8_t i = 0; i < goal->trajectory.joint_names.size(); i ++)
        {
          if(goal->trajectory.joint_names.at(i) != follow_joint_trajectory_feedback_.joint_names.at(i))
            {
              log::error("Joint names mismatch at goalFollowJointTrajectoryCallback!");
              ROS_ERROR("Aborting Trajectory Execution!");
              follow_joint_trajectory_server_.setAborted();
              return;
            }
        }
    }

    joint_trajectory_ = goal->trajectory;
    moveit_plan_state_ = true;

    ros::Rate loop_rate(100);
    while(follow_joint_trajectory_server_.isActive() && ros::ok())
    {
      if(follow_joint_trajectory_server_.isPreemptRequested())
      {
        ROS_ERROR("Trajectory Execution Interrupted!");
        follow_joint_trajectory_server_.setPreempted();
        moveit_plan_state_ = false;
        return;
      }
      loop_rate.sleep();
    }
  }

bool OpenManipulatorController::goalJointSpacePathCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                                           open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  open_manipulator_.makeJointTrajectory(target_angle, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsPoseCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                           open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeJointTrajectory(req.end_effector_name, target_pose, req.path_time);
  
  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsPositionCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                               open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.makeJointTrajectory(req.end_effector_name, target_pose.position, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathToKinematicsOrientationCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                                  open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeJointTrajectory(req.end_effector_name, target_pose.orientation, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                          open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);
  open_manipulator_.makeTaskTrajectory(req.end_effector_name, target_pose, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathPositionOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                      open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.makeTaskTrajectory(req.end_effector_name, position, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathOrientationOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                         open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectory(req.end_effector_name, orientation, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalJointSpacePathFromPresentCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                                                      open_manipulator_msgs::SetJointPosition::Response &res)
{
  std::vector <double> target_angle;

  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    target_angle.push_back(req.joint_position.position.at(i));

  open_manipulator_.makeJointTrajectoryFromPresentPosition(target_angle, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                     open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  KinematicPose target_pose;
  target_pose.position[0] = req.kinematics_pose.pose.position.x;
  target_pose.position[1] = req.kinematics_pose.pose.position.y;
  target_pose.position[2] = req.kinematics_pose.pose.position.z;

  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                       req.kinematics_pose.pose.orientation.x,
                       req.kinematics_pose.pose.orientation.y,
                       req.kinematics_pose.pose.orientation.z);

  target_pose.orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req.planning_group, target_pose, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentPositionOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                                 open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Vector3d position;
  position[0] = req.kinematics_pose.pose.position.x;
  position[1] = req.kinematics_pose.pose.position.y;
  position[2] = req.kinematics_pose.pose.position.z;

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req.planning_group, position, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalTaskSpacePathFromPresentOrientationOnlyCallback(open_manipulator_msgs::SetKinematicsPose::Request  &req,
                                                                                    open_manipulator_msgs::SetKinematicsPose::Response &res)
{
  Eigen::Quaterniond q(req.kinematics_pose.pose.orientation.w,
                        req.kinematics_pose.pose.orientation.x,
                        req.kinematics_pose.pose.orientation.y,
                        req.kinematics_pose.pose.orientation.z);

  Eigen::Matrix3d orientation = math::convertQuaternionToRotationMatrix(q);

  open_manipulator_.makeTaskTrajectoryFromPresentPose(req.planning_group, orientation, req.path_time);

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalToolControlCallback(open_manipulator_msgs::SetJointPosition::Request  &req,
                                                        open_manipulator_msgs::SetJointPosition::Response &res)
{
  for(int i = 0; i < req.joint_position.joint_name.size(); i ++)
    open_manipulator_.makeToolTrajectory(req.joint_position.joint_name.at(i), req.joint_position.position.at(i));

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::setActuatorStateCallback(open_manipulator_msgs::SetActuatorState::Request  &req,
                                                         open_manipulator_msgs::SetActuatorState::Response &res)
{
  if(req.set_actuator_state == true) // enable actuators
  {
    log::println("Wait a second for actuator enable", "GREEN");
    timer_thread_state_ = false;
    pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
    open_manipulator_.enableAllActuator();
    startTimerThread();
  }
  else // disable actuators
  {
    log::println("Wait a second for actuator disable", "GREEN");
    timer_thread_state_ = false;
    pthread_join(timer_thread_, NULL); // Wait for the thread associated with thread_p to complete
    open_manipulator_.disableAllActuator();
    startTimerThread();
  }

  res.is_planned = true;
  return true;
}

bool OpenManipulatorController::goalDrawingTrajectoryCallback(open_manipulator_msgs::SetDrawingTrajectory::Request  &req,
                                                              open_manipulator_msgs::SetDrawingTrajectory::Response &res)
{
  try
  {
    if(req.drawing_trajectory_name == "circle")
    {
      double draw_circle_arg[3];
      draw_circle_arg[0] = req.param[0];  // radius (m)
      draw_circle_arg[1] = req.param[1];  // revolution (rev)
      draw_circle_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_circle_arg = &draw_circle_arg;

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_CIRCLE, req.end_effector_name, p_draw_circle_arg, req.path_time);
    }
    else if(req.drawing_trajectory_name == "line")
    {
      TaskWaypoint draw_line_arg;
      draw_line_arg.kinematic.position(0) = req.param[0]; // x axis (m)
      draw_line_arg.kinematic.position(1) = req.param[1]; // y axis (m)
      draw_line_arg.kinematic.position(2) = req.param[2]; // z axis (m)
      void *p_draw_line_arg = &draw_line_arg;
      
      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_LINE, req.end_effector_name, p_draw_line_arg, req.path_time);
    }
    else if(req.drawing_trajectory_name == "rhombus")
    {
      double draw_rhombus_arg[3];
      draw_rhombus_arg[0] = req.param[0];  // radius (m)
      draw_rhombus_arg[1] = req.param[1];  // revolution (rev)
      draw_rhombus_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_rhombus_arg = &draw_rhombus_arg;

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_RHOMBUS, req.end_effector_name, p_draw_rhombus_arg, req.path_time);
    }
    else if(req.drawing_trajectory_name == "heart")
    {
      double draw_heart_arg[3];
      draw_heart_arg[0] = req.param[0];  // radius (m)
      draw_heart_arg[1] = req.param[1];  // revolution (rev)
      draw_heart_arg[2] = req.param[2];  // start angle position (rad)
      void* p_draw_heart_arg = &draw_heart_arg;

      open_manipulator_.makeCustomTrajectory(CUSTOM_TRAJECTORY_HEART, req.end_effector_name, p_draw_heart_arg, req.path_time);
    }
    res.is_planned = true;
    return true;
  }
  catch ( ros::Exception &e )
  {
    log::error("Creation the custom trajectory is failed!");
  }
  return true;
}

void OpenManipulatorController::publishOpenManipulatorStates()
{
  open_manipulator_msgs::OpenManipulatorState msg;
  if(open_manipulator_.getMovingState())
    msg.open_manipulator_moving_state = msg.IS_MOVING;
  else
    msg.open_manipulator_moving_state = msg.STOPPED;

  if(open_manipulator_.getActuatorEnabledState(JOINT_DYNAMIXEL))
    msg.open_manipulator_actuator_state = msg.ACTUATOR_ENABLED;
  else
    msg.open_manipulator_actuator_state = msg.ACTUATOR_DISABLED;

  open_manipulator_states_pub_.publish(msg);
}

void OpenManipulatorController::publishKinematicsPose()
{
  open_manipulator_msgs::KinematicsPose msg;
  auto opm_tools_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  uint8_t index = 0;
  for (auto const& tools:opm_tools_name)
  {
    KinematicPose pose = open_manipulator_.getKinematicPose(tools);
    msg.pose.position.x = pose.position[0];
    msg.pose.position.y = pose.position[1];
    msg.pose.position.z = pose.position[2];
    Eigen::Quaterniond orientation = math::convertRotationMatrixToQuaternion(pose.orientation);
    msg.pose.orientation.w = orientation.w();
    msg.pose.orientation.x = orientation.x();
    msg.pose.orientation.y = orientation.y();
    msg.pose.orientation.z = orientation.z();

    open_manipulator_kinematics_pose_pub_.at(index).publish(msg);
    index++;
  }
}

void OpenManipulatorController::publishStates()
{
  sensor_msgs::JointState joint_state_msg;
  control_msgs::JointTrajectoryControllerState controller_state_msg;
  trajectory_msgs::JointTrajectoryPoint actual;
  trajectory_msgs::JointTrajectoryPoint error;
  ros::Time stamp = ros::Time::now();

  auto joints_name = open_manipulator_.getManipulator()->getAllActiveJointComponentName();
  auto tool_name = open_manipulator_.getManipulator()->getAllToolComponentName();

  auto joint_value = open_manipulator_.getAllActiveJointValue();
  auto tool_value = open_manipulator_.getAllToolValue();

  // Update joint values
  for(uint8_t i = 0; i < joints_name.size(); i ++)
  {
    joint_state_msg.name.push_back(joints_name.at(i));

    actual.positions.push_back(joint_value.at(i).position);
    actual.velocities.push_back(joint_value.at(i).velocity);
    actual.effort.push_back(joint_value.at(i).effort);
    actual.accelerations.push_back(joint_value.at(i).acceleration);
  }

  // Calculate error
  for(uint8_t i = 0; i < follow_joint_trajectory_feedback_.desired.positions.size(); i ++)
    error.positions.push_back(follow_joint_trajectory_feedback_.desired.positions.at(i) -
                              joint_value.at(i).position);
  for(uint8_t i = 0; i < follow_joint_trajectory_feedback_.desired.velocities.size(); i ++)
    error.velocities.push_back(follow_joint_trajectory_feedback_.desired.velocities.at(i) -
                               joint_value.at(i).velocity);
  for(uint8_t i = 0; i < follow_joint_trajectory_feedback_.desired.accelerations.size(); i ++)
    error.accelerations.push_back(follow_joint_trajectory_feedback_.desired.accelerations.at(i) -
                                  joint_value.at(i).acceleration);
  // for(uint8_t i = 0; i < follow_joint_trajectory_feedback_.desired.effort.size(); i ++)
  //   error.effort.push_back(follow_joint_trajectory_feedback_.desired.effort.at(i) -
  //                          joint_value.at(i).effort);

  // Update Feedback message
  follow_joint_trajectory_feedback_.joint_names = joint_state_msg.name;
  follow_joint_trajectory_feedback_.actual = actual;
  follow_joint_trajectory_feedback_.error = error;

  // Prepare JointState message
  joint_state_msg.header.stamp = stamp;
  joint_state_msg.position = actual.positions;
  joint_state_msg.velocity = actual.velocities;
  joint_state_msg.effort = actual.effort;

  for(uint8_t i = 0; i < tool_name.size(); i ++)
  {
    joint_state_msg.name.push_back(tool_name.at(i));

    joint_state_msg.position.push_back(tool_value.at(i).position);
    joint_state_msg.velocity.push_back(0.0f);
    joint_state_msg.effort.push_back(0.0f);
  }

  // Prepare JointTrajectoryControllerState message
  controller_state_msg.header.stamp = stamp;
  controller_state_msg.joint_names = follow_joint_trajectory_feedback_.joint_names;
  controller_state_msg.desired = follow_joint_trajectory_feedback_.desired;
  controller_state_msg.actual = follow_joint_trajectory_feedback_.actual;
  controller_state_msg.error = follow_joint_trajectory_feedback_.error;

  // ROS_INFO("joint_state.posi[0]:=%3f",joint_state_msg.position[0]);
  // ROS_INFO("joint_state.posi[1]:=%3f",joint_state_msg.position[1]);
  // ROS_INFO("joint_state.posi[2]:=%3f",joint_state_msg.position[2]);


  // Publish messages
  open_manipulator_joint_states_pub_.publish(joint_state_msg);
  open_manipulator_controller_state_pub_.publish(controller_state_msg);
}

void OpenManipulatorController::publishGazeboCommand()
{
  JointWaypoint joint_value = open_manipulator_.getAllActiveJointValue();
  JointWaypoint tool_value = open_manipulator_.getAllToolValue();

  for(uint8_t i = 0; i < joint_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = joint_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(i).publish(msg);
  }

  for(uint8_t i = 0; i < tool_value.size(); i ++)
  {
    std_msgs::Float64 msg;
    msg.data = tool_value.at(i).position;

    gazebo_goal_joint_position_pub_.at(joint_value.size() + i).publish(msg);
  }
}

void OpenManipulatorController::publishCallback(const ros::TimerEvent&)
{
  if (using_platform_ == true)  publishStates();
  else  publishGazeboCommand();

  publishOpenManipulatorStates();
  publishKinematicsPose();
}

void OpenManipulatorController::moveitPublishGoal(uint32_t step_cnt, double* time_from_start)
{
  JointWaypoint target;

  for(uint8_t i = 0; i < joint_trajectory_.points[step_cnt].positions.size(); i++)
  {
    JointValue temp;
    temp.position = joint_trajectory_.points[step_cnt].positions.at(i);
    if(joint_trajectory_.points[step_cnt].velocities.size())
      temp.velocity = joint_trajectory_.points[step_cnt].velocities.at(i);
    if(joint_trajectory_.points[step_cnt].accelerations.size())
      temp.acceleration = joint_trajectory_.points[step_cnt].accelerations.at(i);
    // if(joint_trajectory_.points[step_cnt].effort.size())
    //   temp.effort = joint_trajectory_.points[step_cnt].effort.at(i);
  target.push_back(temp);
  }

  follow_joint_trajectory_feedback_.desired = joint_trajectory_.points[step_cnt];
  double req_time = joint_trajectory_.points[step_cnt].time_from_start.toSec();
  double path_time = req_time - *time_from_start;

  if (path_time > 0.0f)
    open_manipulator_.makeJointTrajectory(target, path_time);

  *time_from_start = req_time;
}

void OpenManipulatorController::moveitPublishFeedback()
{
  follow_joint_trajectory_server_.publishFeedback(follow_joint_trajectory_feedback_);
}


void OpenManipulatorController::moveitTimer(double present_time)
{
  static double start_time = 0.0f;
  static double time_from_start = 0.0f;
  static uint32_t step_cnt = 0;

  if (moveit_plan_state_ == true)
  {
    if (present_time > start_time + time_from_start)
    {
      uint32_t all_time_steps = joint_trajectory_.points.size();
      if (step_cnt >= all_time_steps)
      {
        if(follow_joint_trajectory_server_.isActive())
          follow_joint_trajectory_server_.setSucceeded(follow_joint_trajectory_result_);
        moveit_plan_state_ = false;
        ROS_DEBUG("Finished Trajectory Execution!");
      }
      else
      {
        moveitPublishGoal(step_cnt, &time_from_start);
        step_cnt++;
      }
    }
    moveitPublishFeedback();
  }
  else
  {
    start_time = present_time;
    time_from_start = 0.0f;
    step_cnt = 0;
  }
}

void OpenManipulatorController::process(double time)
{
  moveitTimer(time);
  open_manipulator_.processOpenManipulator(time);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "open_manipulator_controller");
  ros::NodeHandle node_handle("");

  std::string usb_port = "/dev/ttyUSB0";
  std::string baud_rate = "1000000";

  if (argc < 3)
  {
    log::error("Please set '-port_name' and  '-baud_rate' arguments for connected Dynamixels");
    return 0;
  }
  else
  {
    usb_port = argv[1];
    baud_rate = argv[2];
  }

  OpenManipulatorController om_controller(usb_port, baud_rate);

  om_controller.initPublisher();
  om_controller.initSubscriber();
  om_controller.initServer();

  om_controller.startTimerThread();

  ros::Timer publish_timer = node_handle.createTimer(ros::Duration(om_controller.getControlPeriod()), &OpenManipulatorController::publishCallback, &om_controller);

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
