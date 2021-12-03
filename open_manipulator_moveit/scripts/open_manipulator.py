#!/usr/bin/env python

import sys, math, copy
import rospy, tf, geometry_msgs.msg

import moveit_commander 
#import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler

def euler_to_quaternion(role, pitch, yaw):
	q = quaternion_from_euler(role, pitch, yaw)
	return Quaternion(q[0], q[1], q[2], q[3])

def main():

    # configuration for moveit
    robot = moveit_commander.RobotCommander()

    # group name is "arm" and "gripper"
    print("robot group:", robot.get_group_names())
    #group name
    arm = moveit_commander.MoveGroupCommander("arm")
    print("robot current state:", arm.get_current_joint_values())
    #joint name
    print ("")
    # ['world_fixed', 'joint1', 'joint2', 'joint3', 'joint4', 'end_effector_joint']
    print("arm joint name:", robot.get_joint_names("arm"))
    print ("")

    arm.set_pose_reference_frame("world")

    rospy.loginfo("Start InitPose")
    joint_goal = arm.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[1] = -1.18
    joint_goal[2] = 0.80
    joint_goal[4] = 0.568 
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(joint_goal, wait=True)

    # wait
    rospy.sleep(3.0)

    rospy.loginfo("Start SecondPose")
    joint_goal = arm.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[0] = -0.327
    joint_goal[1] = 0.65
    joint_goal[2] = -0.84
    joint_goal[4] = 2.086
    joint_goal[5] = -0.33
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(joint_goal, wait=True)

    # wait
    rospy.sleep(3.0)

    rospy.loginfo("Back InitPose")
    joint_goal = arm.get_current_joint_values()
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    joint_goal[1] = -1.18
    joint_goal[2] = 0.80
    joint_goal[4] = 0.568 
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(joint_goal, wait=True)

    print("Finish!")

if __name__ == '__main__':

  rospy.init_node( "mikata-arm", anonymous=True )

  try:
      main()
  except (rospy.ROSInterruptException):
      pass


    
    
    

