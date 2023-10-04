#include "robot_gui/robot_gui_backend.h"
#include <cstdint>
#include <ros/ros.h>

#include <opencv2/opencv.hpp>
#define CVUI_IMPLEMENTATION
#include "robot_gui/cvui.h"

#define WINDOW_NAME "robot_gui_node"

int main(int argc, char **argv) {
  // initialize ros
  ros::init(argc, argv, "robot_gui_node");
  ros::NodeHandle node;

  // initialize gui backend
  RobotGUI robot_obj = RobotGUI(&node);

  // init a OpenCV window and tell cvui to use it.
  cv::Mat frame = cv::Mat(640, 520, CV_8UC3);
  cv::namedWindow(WINDOW_NAME);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {

    // create frame background
    frame = cv::Scalar(49, 52, 49);

    // screen header and quite button
    cvui::text(frame, 210, 15, "Robot Controller");
    if (cvui::button(frame, 445, 10, 64, 25, "&Quit")) {
      break;
    }

    // general info area
    cvui::window(frame, 10, 40, 500, 150, "Topic: /robot_info");
    int nexline = 0;
    for (auto value : robot_obj.get_robot_info_packet()) {
      cvui::text(frame, 20, 65 + nexline, value);
      nexline += 15;
    }

    // current velocities
    cvui::text(frame, 10, 200, "Robot current velocity:");
    cvui::window(frame, 10, 220, 245, 50, "Linear Velocity");
    cvui::text(frame, 20, 249, robot_obj.get_cmd_vel_linear_x());
    cvui::window(frame, 265, 220, 245, 50, "Angular Velocity");
    cvui::text(frame, 275, 249, robot_obj.get_cmd_vel_angular_z());

    // robot position
    cvui::text(frame, 10, 280, "Estimated robot position based on odometry:");
    cvui::window(frame, 10, 300, 160, 50, "Linear X");
    cvui::text(frame, 20, 329, robot_obj.get_odom_x());
    cvui::window(frame, 180, 300, 160, 50, "Linear Y");
    cvui::text(frame, 200, 329, robot_obj.get_odom_y());
    cvui::window(frame, 350, 300, 160, 50, "Linear Z");
    cvui::text(frame, 370, 329, robot_obj.get_odom_z());

    // distance travelled service
    cvui::text(frame, 10, 360, "Distance Travelled:");
    cvui::window(frame, 10, 380, 160, 50, "Distance (in meter):");
    cvui::text(frame, 20, 409, robot_obj.get_distance());
    if (cvui::button(frame, 180, 380, 160, 50, "Call Service")) {
      robot_obj.service_client_get_distance();
    }
    if (cvui::button(frame, 350, 380, 160, 50, "Reset")) {
      robot_obj.service_client_reset_distance();
    }

    // teleoperation buttons
    cvui::text(frame, 10, 440, "Robot joystick:");
    if (cvui::button(frame, 180, 460, 160, 50, "Forward")) {
      robot_obj.publisher_call_linear(0.2, 1);
    }
    if (cvui::button(frame, 10, 520, 160, 50, "Left")) {
      robot_obj.publisher_call_angular(0.2, 1);
    }
    if (cvui::button(frame, 180, 520, 160, 50, "Stop")) {
      robot_obj.publisher_call_halt();
    }
    if (cvui::button(frame, 350, 520, 160, 50, "Right")) {
      robot_obj.publisher_call_angular(0.2, -1);
    }
    if (cvui::button(frame, 180, 580, 160, 50, "Backward")) {
      robot_obj.publisher_call_linear(0.2, -1);
    }

    // update cvui and show everything on the screen
    cvui::update();
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }

    // publish and spin as a single-threaded node
    robot_obj.publisher_cmd_vel_callback();
    ros::spinOnce();
  }

  return 0;
}
