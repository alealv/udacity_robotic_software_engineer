#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"
#include <cstdio>
#include <math.h>
#include <sensor_msgs/Image.h>

class ImageProc {

public:
  // Constructor
  ImageProc() {

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    sub1 = n.subscribe("/camera/rgb/image_raw", 10, &ImageProc::process_image_callback, this);
  }

private:
  ros::NodeHandle n;
  ros::ServiceClient client;
  ros::Subscriber sub1;

  // This callback function continuously executes and reads the image data
  void process_image_callback(const sensor_msgs::Image img) {

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int left = 0, mid = 0, right = 0;

    for (int row = 0; row < img.height; row++) {

      for (int col = 0; col < img.width; col++) {

        // Get pixel position
        int pixel = (row * img.step + col * 3);

        // Check tath RGB have 255
        if (img.data[pixel] == 255 && img.data[pixel + 1] == 255 && img.data[pixel + 2] == 255) {

          if (col <= img.width / 3.0)
            left++;
          if (col > img.width / 3.0 && col < img.width * 2.0 / 3.0)
            mid++;
          if (col >= img.width * 2.0 / 3.0)
            right++;
        }
      }
    }

    // Move if any of the counters are greater than 0
    float lin_x = left || mid || right ? 1 : 0;

    // Set the angle proportional to the difference between right and left
    float ang_z = 0;
    if (lin_x)
      ang_z = M_PI_4 * (right - left) / (float)(left + mid + right);

    // Move robot
    drive_robot(lin_x, ang_z);
  }

  // This function calls the command_robot service to drive the robot in the specified direction
  void drive_robot(float lin_x, float ang_z) {
    // TODO: Request a service and pass the velocities to it to drive the robot

    // Request centered joint angles [1.57, 1.57]
    ball_chaser::DriveToTarget msg;
    msg.request.linear_x = lin_x;
    msg.request.angular_z = ang_z;

    if (!client.call(msg))
      ROS_ERROR("Failed to call service command_robot");
  }
};

int main(int argc, char **argv) {
  // Initialize the process_image node and create a handle to it
  ros::init(argc, argv, "process_image");

  ImageProc image_proc;

  // Handle ROS communication events
  ros::spin();

  return 0;
}