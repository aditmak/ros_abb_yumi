'''  robotleft_group.set_named_target("LeftHome")

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
  robotright_client.wait_for_result()''' 
