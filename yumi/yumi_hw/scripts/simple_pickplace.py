#!/usr/bin/env python

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import actionlib
import geometry_msgs
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from yumi_hw.srv import YumiGrasp


def simple_pickplace():
  ## First initialize moveit_commander and rospy.
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('simple_pickplace', anonymous=True)

  ## This interface can be used to plan and execute motions on robotleft.
  robotleft_group = moveit_commander.MoveGroupCommander("left_arm")
  ## MoveGroup Commander Object for robot2.
  robotright_group = moveit_commander.MoveGroupCommander("right_arm")
  ## MoveGroup Commander Object for both_arm.
  #robotboth_group = moveit_commander.MoveGroupCommander("both_arm")

  ## Action clients to the ExecuteTrajectory action server.
  robotleft_client = actionlib.SimpleActionClient('execute_trajectory',
    moveit_msgs.msg.ExecuteTrajectoryAction)
  robotleft_client.wait_for_server()
  rospy.loginfo('Execute Trajectory server is available for robot_leftarm')
  robotright_client = actionlib.SimpleActionClient('execute_trajectory',
    moveit_msgs.msg.ExecuteTrajectoryAction)
  robotright_client.wait_for_server()
  rospy.loginfo('Execute Trajectory server is available for robot_rightarm')


  robotleft_group.set_named_target("LeftHome")

  ## Plan to the desired joint-space goal using the default planner (RRTConnect).
  robotleft_plan_home = robotleft_group.plan()
  ## Create a goal message object for the action server.
  robotleft_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  ## Update the trajectory in the goal message.
  robotleft_goal.trajectory = robotleft_plan_home

  ## Send the goal to the action server.
  robotleft_client.send_goal(robotleft_goal)
  robotleft_client.wait_for_result()

  robotright_group.set_named_target("RightHome")

  ## Plan to the desired joint-space goal using the default planner (RRTConnect).
  robotright_plan_home = robotright_group.plan()
  ## Create a goal message object for the action server.
  robotright_goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
  ## Update the trajectory in the goal message.
  robotright_goal.trajectory = robotright_plan_home

  ## Send the goal to the action server.
  robotright_client.send_goal(robotright_goal)
  robotright_client.wait_for_result()

def gripper_close(x):
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('/yumi/yumi_gripper/do_grasp')
    try:
        service_proxy = rospy.ServiceProxy("/yumi/yumi_gripper/do_grasp", YumiGrasp)
        resp = service_proxy(x) #gripper_id is the ID of your gripper, 1 or 2
        rospy.sleep(1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def gripper_open(x):
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service('/yumi/yumi_gripper/release_grasp')
    try:
        service_proxy = rospy.ServiceProxy("/yumi/yumi_gripper/release_grasp", YumiGrasp)
        resp = service_proxy(x) #gripper_id is the ID of your gripper, 1 or 2
        rospy.sleep(1)
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

if __name__=='__main__':
  try:
      gripper_close(1)
      gripper_open(2)
  except rospy.ROSInterruptException:
      pass
