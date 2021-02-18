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


def simpletrajectory():
  moveit_commander.roscpp_initialize(sys.argv)
  rospy.init_node('simpletrajectory', anonymous=True)

  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  robotleft_group = moveit_commander.MoveGroupCommander("left_arm")
  robotright_group = moveit_commander.MoveGroupCommander("right_arm")

  display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)

  planning_frame = robotleft_group.get_planning_frame()
  print "============ Reference frame: %s" % planning_frame

  eef_link = robotleft_group.get_end_effector_link()
  print "============ End effector: %s" % eef_link

  group_names = robot.get_group_names()
  print "============ Robot Groups:", robot.get_group_names()

  print "============ Printing robot state"
  print robot.get_current_state()
  print ""

  current_pose = robotleft_group.get_current_pose()
  rospy.sleep(0.5)
  current_pose = robotleft_group.get_current_pose()

  pose_goal = geometry_msgs.msg.Pose()
  pose_goal.position.x = -0.1
  pose_goal.position.y = -0.1
  pose_goal.position.z = 0.3
  pose_goal.orientation = copy.deepcopy(current_pose.pose.orientation)
  robotleft_group.set_pose_target(pose_goal)

  plan = robotleft_group.go(wait=True)
  robotleft_group.stop()
  robotleft_group.clear_pose_targets()

  robotleft_group.execute(plan, wait=True)





if __name__=='__main__':
  try:
    simpletrajectory()
  except rospy.ROSInterruptException:
    pass
