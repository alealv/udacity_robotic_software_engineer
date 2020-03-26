#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class Picker {

public:
  Picker() : ac("move_base", true) {

    marker_sub = n.subscribe("/visualization_marker", 10, &Picker::check_marker, this);

    // while (!ac.waitForServer(ros::Duration(5.0))) {
    //   ROS_INFO("Waiting for the move_base action server to come up");
    // }
  }

private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber marker_sub;
  std::string state;
  move_base_msgs::MoveBaseGoal goal;
  MoveBaseClient ac;

  void check_marker(const visualization_msgs::Marker marker) {

    // set up the frame parameters and define a position and orientation for the robot to reach
    goal.target_pose.header.frame_id = "base_footprint";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose = marker.pose;

    // Send the first goal position and orientation for the robot to reach
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
  }
};

int main(int argc, char **argv) {

  // Initialize the pick_objects node
  ros::init(argc, argv, "pick_objects");

  Picker pck;

  // Handle ROS communication events
  ros::spin();

  return 0;
}