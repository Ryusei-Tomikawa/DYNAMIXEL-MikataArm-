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
    # ['gripper', 'gripper_sub']

    arm.set_pose_reference_frame("world")

    # pose 1
    rospy.loginfo("Start Pose 1")
    joint_goal = arm.get_current_joint_values()
    print("size:=", len(joint_goal))
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(joint_goal, wait=True)

    print("robot current state:", arm.get_current_joint_values())
    #joint name
    print ("")
    # ['world_fixed', 'joint1', 'joint2', 'joint3', 'joint4', 'end_effector_joint']
    print("arm joint name:", robot.get_joint_names("arm"))
    print ("")

    end_effector_link = arm.get_end_effector_link()
    arm.set_end_effector_link(end_effector_link)
    pose_home = PoseStamped()
    pose_home.header.frame_id = "world"
    pose_home.pose.position.x = 0.212149804964
    pose_home.pose.position.y = 0.0107562461967
    pose_home.pose.position.z = 0.157262164164
    pose_home.pose.orientation.x = -0.0101956468087
    pose_home.pose.orientation.y = 0.379710360295
    pose_home.pose.orientation.z = 0.0248296548147
    pose_home.pose.orientation.w = 0.924715945203
    arm.set_joint_value_target(pose_home, True)
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(wait=True)
    rospy.sleep(1.0)
    rospy.loginfo("Goal Pose 1")

    print("robot current state:", arm.get_current_joint_values())
    #joint name
    print ("")
    # ['world_fixed', 'joint1', 'joint2', 'joint3', 'joint4', 'end_effector_joint']
    print("arm joint name:", robot.get_joint_names("arm"))
    print ("")

     # pose 1
    rospy.loginfo("Start Pose 1")
    joint_goal = arm.get_current_joint_values()
    print("size:=", len(joint_goal))
    for i in range(0,len(joint_goal)):
      joint_goal[i] = 0
    arm.set_max_velocity_scaling_factor(0.2)
    arm.go(joint_goal, wait=True)

    print("robot current state:", arm.get_current_joint_values())
    #joint name
    print ("")
    # ['world_fixed', 'joint1', 'joint2', 'joint3', 'joint4', 'end_effector_joint']
    print("arm joint name:", robot.get_joint_names("arm"))
    print ("")

    # # Gripper Close Pose
    # # Pose 2 
    # rospy.loginfo("Start Pose 2")
    # arm.set_end_effector_link("end_effector_link")
    # pose_target_2 = PoseStamped()
    # pose_target_2.header.frame_id = "world"
    # pose_target_2.pose.orientation= euler_to_quaternion(0.0, math.pi/2.0, 0.0)
    # pose_target_2.pose.position.x =  0.18
    # pose_target_2.pose.position.y =  0.0
    # pose_target_2.pose.position.z =  0.10
    # arm.set_joint_value_target(pose_target_2, True)
    # arm.go(wait=True)
    # rospy.sleep(1.5)
    # rospy.loginfo("Goal Pose 2")
    
    # # wait
    # rospy.sleep(2.0)
    
    #  # Pose 3 Z:+0.05[m]
    # rospy.loginfo( "Starting Pose 3 Z:-0.05[m]")
    # pose_target_2.pose.position.z += 0.07
    # arm.set_joint_value_target( pose_target_2, True )
    # arm.go(wait=True) 

    #  # wait
    # rospy.sleep(2.0)

    print("Finish!")

if __name__ == '__main__':

    node_name = "Gripper"
    rospy.init_node( node_name, anonymous=True )

    try:
        main()
    except (rospy.ROSInterruptException):
        pass


    
    
    

