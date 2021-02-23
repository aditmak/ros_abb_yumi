#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ar_track_alvar_msgs/AlvarMarkers.h"


void trackerCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr &data){
  geometry_msgs::PoseStamped target_position;
  if(!data->markers.empty() && data->markers[0].id == 0){
    target_position.header = data->markers[0].header;
    target_position.pose = data->markers[0].pose.pose;
    ROS_INFO("the data is [%f] and frame_id is [%d]", data->markers[0].pose.pose.position.x,
                                                      data->markers[0].id);
  }
  else{
    ROS_INFO("Target Not Found! Waiting for Target!");
  }
}

int main(int argc, char **argv){
  ros::init(argc, argv, "tracker");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("ar_pose_marker", 1000, trackerCallback);
  ros::spin();
  return 0;
}
