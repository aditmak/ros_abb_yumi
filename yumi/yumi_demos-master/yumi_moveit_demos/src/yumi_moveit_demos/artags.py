#!/usr/bin/env python

import sys
import rospy
from geometry_msgs.msg import PoseStamped
from ar_track_alvar_msgs.msg import AlvarMarkers
import yumi_moveit_utils as ab


def trackCallback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s" , data.data)
    PoseStamped target_position;
    if not data.markers.empty:
        target_position.header = data.markers[0].header;
        target_position.pose = data.markers[0].pose.pose;
        rospy.loginfo("Data received and transfered!")
    else:
        rospy.loginfo("Target LOST")







def tracker():
    rospy.init_node("tracker", anonymous=True)

    rospy.Subscriber("ar_pose_marker", ar_track_alvar_msgs , trackCallback)

    rospy.spin()


if __name == '__main__':
    try:
        tracker()
    except  rospy.ROSInterruptException:
        rospy.loginfo("Tracker Node Terminated")
