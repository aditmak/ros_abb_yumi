Node [/yumi_moveit_demo]
Publications:
 * /attached_collision_object [moveit_msgs/AttachedCollisionObject]
 * /collision_object [moveit_msgs/CollisionObject]
 * /move_group/display_planned_path [moveit_msgs/DisplayTrajectory]
 * /rosout [rosgraph_msgs/Log]
 * /yumi/gripper_l_effort_cmd [std_msgs/Float64]
 * /yumi/gripper_r_effort_cmd [std_msgs/Float64]

Subscriptions: None

Services:
 * /yumi_moveit_demo/get_loggers
 * /yumi_moveit_demo/set_logger_level


contacting node http://abbpc-System-Product-Name:46083/ ...
Pid: 14741
Connections:
 * topic: /collision_object
    * to: /move_group
    * direction: outbound
    * transport: TCPROS
 * topic: /yumi/gripper_r_effort_cmd
    * to: /yumi/yumi_gripper
    * direction: outbound
    * transport: TCPROS
 * topic: /yumi/gripper_l_effort_cmd
    * to: /yumi/yumi_gripper
    * direction: outbound
    * transport: TCPROS
 * topic: /rosout
    * to: /rosout
    * direction: outbound
    * transport: TCPROS
 * topic: /attached_collision_object
    * to: /move_group
    * direction: outbound
    * transport: TCPROS
 * topic: /move_group/display_planned_path
    * to: /rviz_abbpc_System_Product_Name_12143_1293641632203576528
    * direction: outbound
    * transport: TCPROS
