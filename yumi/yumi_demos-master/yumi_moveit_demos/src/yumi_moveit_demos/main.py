#!/usr/bin/env python


import sys
import copy
import rospy
import moveit_commander
import yumi_moveit_utils as yumi
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
    object_pose_left = geometry_msgs.msg.TransformStamped()
    object_pose_right = geometry_msgs.msg.TransformStamped()
    if data.markers:
        #Need to figure out good way to switch betweem markers. For example, if left marker is gone, it should directly
        #switch to right marker and right arm but currently that does not happen!
        #Also change the ID number if you change the marker number for both arms
        if(data.markers[-1].id == 0):
            while True:
              try:
                object_pose_left = tf_buffer.lookup_transform("yumi_base_link", "follow_left",rospy.Time(0), rospy.Duration(1))
                break
              except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            object_pose_l = geometry_msgs.msg.Pose()
            object_pose_l.position.x = object_pose_left.transform.translation.x
            object_pose_l.position.y = object_pose_left.transform.translation.y
            object_pose_l.position.z = object_pose_left.transform.translation.z
            object_pose_l.orientation.x = object_pose_left.transform.rotation.x
            object_pose_l.orientation.y = object_pose_left.transform.rotation.y
            object_pose_l.orientation.z = object_pose_left.transform.rotation.z
            object_pose_l.orientation.w = object_pose_left.transform.rotation.w
            rospy.loginfo("Data received {}".format(object_pose_l))
            if(object_pose_l.position.y > -0.1):
                move_and_grasp(yumi.LEFT, object_pose_l, -5)
                rate.sleep()
            else:
                rospy.loginfo("YUMI LEFT ARM cannot reach the position")


        if(data.markers[-1].id == 3):
            while True:
              try:
                object_pose_right = tf_buffer.lookup_transform("yumi_base_link", "follow_right",rospy.Time(0), rospy.Duration(1))
                break
              except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

            object_pose_r = geometry_msgs.msg.Pose()
            object_pose_r.position.x = object_pose_right.transform.translation.x
            object_pose_r.position.y = object_pose_right.transform.translation.y
            object_pose_r.position.z = object_pose_right.transform.translation.z
            object_pose_r.orientation.x = object_pose_right.transform.rotation.x
            object_pose_r.orientation.y = object_pose_right.transform.rotation.y
            object_pose_r.orientation.z = object_pose_right.transform.rotation.z
            object_pose_r.orientation.w = object_pose_right.transform.rotation.w
            rospy.loginfo("Data received {}".format(object_pose_r))

            if(object_pose_r.position.y < 0.01):
                move_and_grasp(yumi.RIGHT, object_pose_r, -5)
                rate.sleep()
            else:
                rospy.loginfo("YUMI RIGHT ARM cannot reach the position")
    else:
        rospy.loginfo("Target LOST")


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
        tf_buffer = tf2_ros.Buffer(rospy.Duration(0))
        tf_listener = tf2_ros.TransformListener(tf_buffer)
        rate = rospy.Rate(10.0)
        run()

    	print "####################################     Program finished     ####################################"
    except rospy.ROSInterruptException:
        pass
