#!/usr/bin/env python

import sys, math, copy
import rospy, tf, geometry_msgs.msg

import moveit_commander 
#import MoveGroupCommander, RobotCommander
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from open_manipulator_msgs.msg import JointPosition
from open_manipulator_msgs.srv import SetJointPosition


rospy.wait_for_service('/mikata_arm/goal_tool_control')

gripper = rospy.ServiceProxy('/mikata_arm/goal_tool_control', SetJointPosition)
jp = JointPosition()
jp.position = []
jp.joint_name = []

def euler_to_quaternion(role, pitch, yaw):
	q = quaternion_from_euler(role, pitch, yaw)
	return Quaternion(q[0], q[1], q[2], q[3])


# Max 0.7
def gripper_open(posi):

  jp.joint_name.append('gripper')
  jp.position.append(posi)
  res = gripper('', jp, 0.0)
  print('open')

# Min -0.7
def gripper_close(posi):

  jp.joint_name.append('gripper')
  jp.position.append(posi)
  res = gripper('', jp, 0.0)
  print('close')



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
    print("arm joint name:", robot.get_joint_names("arm"))
    print ("")

    arm.set_pose_reference_frame("world")

    rospy.loginfo("Gripper Init")
    gripper_open(0.7)

    rospy.sleep(3)

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
    joint_goal[5] = -0.0015339808305725455
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(joint_goal, wait=True)

    rospy.sleep(3)

    gripper_close(0.7)

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

  rospy.init_node( "manipulation", anonymous=True )

  try:
      main()
  except (rospy.ROSInterruptException):
      pass


    
    
    

