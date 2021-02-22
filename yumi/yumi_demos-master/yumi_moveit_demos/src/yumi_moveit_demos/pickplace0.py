#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import yumi_moveit_utils0 as yumi
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Empty
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf2_ros
import tf2_geometry_msgs


def close_grippers(arm):
    """Closes the grippers.

    Closes the grippers with an effort of 15 and then relaxes the effort to 0.

    :param arm: The side to be closed (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, 15.0)
    yumi.gripper_effort(arm, 0.0)

def open_grippers(arm):
    """Opens the grippers.

    Opens the grippers with an effort of -15 and then relaxes the effort to 0.

    :param arm: The side to be opened (moveit_utils LEFT or RIGHT)
    :type arm: int
    :returns: Nothing
    :rtype: None
    """
    yumi.gripper_effort(arm, -15.0)
    yumi.gripper_effort(arm, 0.0)


def move_and_grasp(arm, pose_ee, grip_effort):
    try:
        yumi.traverse_path([pose_ee], arm, 10)
    except Exception:
        if (arm == yumi.LEFT):
            yumi.plan_and_move(yumi.group_l, pose_ee)
        elif (arm == yumi.RIGHT):
            yumi.plan_and_move(yumi.group_r, pose_ee)

    if (grip_effort <= 20 and grip_effort >= -20):
        yumi.gripper_effort(arm, grip_effort)
    else:
        print("The gripper effort values should be in the range [-20, 20]")

def trackCallback(data):
    object_pose1 = geometry_msgs.msg.TransformStamped()
    if data.markers:
        if(data.markers[-1].id == 0):
            while True:
              try:
                object_pose1 = tf_buffer.lookup_transform("yumi_base_link", "ar_marker_0",rospy.Time(0), rospy.Duration(1))
                break
              except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            object_pose = geometry_msgs.msg.Pose()
            object_pose.position.x = object_pose1.transform.translation.x
            object_pose.position.y = object_pose1.transform.translation.y
            object_pose.position.z = object_pose1.transform.translation.z
            object_pose.orientation.x = object_pose1.transform.rotation.x
            object_pose.orientation.y = object_pose1.transform.rotation.y
            object_pose.orientation.z = object_pose1.transform.rotation.z
            object_pose.orientation.w = object_pose1.transform.rotation.w
            rospy.loginfo("Data received {}".format(object_pose))
            move_and_grasp(yumi.LEFT, object_pose, -5)
        else:
            rospy.loginfo("Target FOUND but not #0")
    else:
        rospy.loginfo("Target LOST")
    '''if data.markers:
        if(data.markers[-1].id == 0):
            #rospy.loginfo("Data received {}".format(info.pose.pose.position))
            #artag_baselink_transformed = tf_buffer.transform(info.pose)
            #m
            object_pose = geometry_msgs.msg.PoseStamped()
            object_pose.header= data.markers[-1].header
            object_pose.header.frame_id = "camera_link"
            object_pose.pose = data.markers[-1].pose.pose
            while True:
              try:
                object_world_pose = tf_buffer.transform(object_pose, "yumi_base_link")
                #rospy.loginfo("Inside")
                break
              except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
            rospy.loginfo("Data received {}".format(object_world_pose))
            move_and_grasp(yumi.LEFT, object_world_pose.pose, -5)
            tf_buffer.clear()
        else:
            rospy.loginfo("Target FOUND but not #0")
    else:
        rospy.loginfo("Target LOST")
        '''


def run():
    """Starts the node

    Runs to start the node and initialize everthing. Runs forever via Spin()

    :returns: Nothing
    :rtype: None
    """
    #Start by connecting to ROS and MoveIt!
    yumi.init_Moveit()

    # Print current joint angles
    yumi.print_current_joint_states(yumi.RIGHT)
    yumi.print_current_joint_states(yumi.LEFT)

    # Reset YuMi joints to "home" position
    #yumi.reset_pose()
    #
    #move_and_grasp(yumi.LEFT, pose_ee, -10)
    #ar_track_alvar for tags

    rospy.loginfo("Waiting for ar_pose_marker topic...")
    rospy.wait_for_message('ar_pose_marker', AlvarMarkers)
    rospy.Subscriber("ar_pose_marker", AlvarMarkers , trackCallback)



    # Drive YuMi end effectors to a desired position (pose_ee), and perform a grasping task with a given effort (grip_effort)
    # Gripper effort: opening if negative, closing if positive, static if zero
    '''
    pose_ee = [0.3, 0.5, 0.2, 0, 3.14, 3.14]
    grip_effort = -10.0
    move_and_grasp(yumi.LEFT, pose_ee, grip_effort)

    pose_ee = [0.3, 0.5, 0.06, 0, 3.14, 3.14]
    grip_effort = 10.0
    move_and_grasp(yumi.LEFT, pose_ee, grip_effort)

    pose_ee = [0.3, 0.5, 0.2, 0, 3.14, 3.14]
    grip_effort = 10.0
    move_and_grasp(yumi.LEFT, pose_ee, grip_effort)

    #close_grippers(yumi.LEFT)
    open_grippers(yumi.RIGHT)
    '''
#    pose_ee = [0.3, 0, 0.2, 0, 3.14, 3.14]
#    grip_effort = 10.0
#    move_and_grasp(yumi.LEFT, pose_ee, grip_effort)

#    pose_ee = [0.3, 0.3, 0.3, 0, 3.14, 3.14]
#    grip_effort = 10.0
#    move_and_grasp(yumi.LEFT, pose_ee, grip_effort)


#    yumi.reset_pose()

    rospy.spin()




if __name__ == '__main__':
    try:
        rospy.init_node('yumi_moveit_demo', anonymous=True)
        # Declate a TF buffer globally.
        tf_buffer = tf2_ros.Buffer(rospy.Duration(0.001))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        run()

    	print "####################################     Program finished     ####################################"
    except rospy.ROSInterruptException:
        pass
