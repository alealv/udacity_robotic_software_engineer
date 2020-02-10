#include "ball_chaser/DriveToTarget.h"
#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include <std_msgs/Float64.h>

// ROS::Publisher motor commands;
ros::Publisher cmd_vel_publisher;

class DriveRobot {
public:
  DriveRobot() {
    // Define a safe_move service with a handle_safe_move_request callback function
    motor_command_service = n.advertiseService("/ball_chaser/command_robot", &DriveRobot::handle_drive_request, this);

    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist on the robot actuation topic with a publishing queue size of 10
    cmd_vel_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ROS_INFO("Ready to send drive commands");
  };

  // ~DriveRobot();

private:
  ros::NodeHandle n;
  ros::ServiceServer motor_command_service;
  ros::Publisher cmd_vel_publisher;

  // This callback function executes whenever a safe_move service is requested
  bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res) {
    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;

    // Set wheel velocities, forward [0.5, 0.0]
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish angles to drive the robot
    cmd_vel_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "Linear x: " + std::to_string(motor_command.linear.x) + "\nAngular z: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);
  }
};

int main(int argc, char **argv) {
  // Initialize a ROS node
  ros::init(argc, argv, "drive_bot");

  DriveRobot drive_bot;

  // TODO: Handle ROS communication events
  ros::spin();

  return 0;
}