#include <cstdio>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <string>
#include <visualization_msgs/Marker.h>

class Marker {

public:
  Marker() {

    marker_sub = n.subscribe("/odom", 10, &Marker::foo, this);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);

    marker.header.frame_id = "base_footprint";       // frame ID
    marker.header.stamp = ros::Time::now();          // timestamp
    marker.ns = "basic_shapes";                      // namespace
    marker.id = 0;                                   // id
    marker.type = visualization_msgs::Marker::CUBE;  // type
    marker.action = visualization_msgs::Marker::ADD; // Action. Options: ADD, DELETE, *DELETEALL (ROS Indigo)

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 3;
    marker.pose.position.y = 3;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // // Publish the marker
    // while (marker_pub.getNumSubscribers() < 1) {
    //   if (!ros::ok()) {
    //     return;
    //   }
    //   ROS_WARN_ONCE("Please create a subscriber to the marker");
    //   sleep(1);
    // }

    marker_pub.publish(marker);
  }

private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;
  ros::Subscriber marker_sub;
  std::string state;
  visualization_msgs::Marker marker;

  void foo(const nav_msgs::Odometry odom) {

    // marker_pub.publish(marker);

    // Delete marker from pickup zone
    if (state == "down" && marker.pose.position.x == odom.pose.pose.position.x && marker.pose.position.y == odom.pose.pose.position.y && marker.pose.position.z == odom.pose.pose.position.z) {
      // Wait 5 seconds
      sleep(5);

      // Set drop-off position
      marker.pose.position.x = 3;
      marker.pose.position.y = 0;
      marker.pose.position.z = 0;

      // Hide marker
      marker.action = visualization_msgs::Marker::DELETE;

      ROS_INFO("hiding marker");
      marker_pub.publish(marker);

      // Update state
      state = "up";
    }

    // Show marker in drop zone
    if (state == "up" && marker.pose.position.x == odom.pose.pose.position.x && marker.pose.position.y == odom.pose.pose.position.y && marker.pose.position.z == odom.pose.pose.position.z) {
      marker.action = visualization_msgs::Marker::ADD;

      ROS_INFO("showing marker");
      marker_pub.publish(marker);

      // Update state
      state = "finished";
    }
  }
};

int main(int argc, char **argv) {

  ros::init(argc, argv, "basic_shapes");

  Marker marker;

  // Handle ROS communication events
  ros::spin();

  return 0;
}