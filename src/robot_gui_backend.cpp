#include "robot_gui/robot_gui_backend.h"
#include <geometry_msgs/Twist.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <vector>

RobotGUI::RobotGUI(ros::NodeHandle *node)
    : cmd_vel_linear_x(0), cmd_vel_angular_z(0), linear_x_prime(0), angular_z_prime(0), odom_x(0), odom_y(0), odom_z(0) {
  // ros object
  this->subscriber_info = node->subscribe("/robot_info", 1000, &RobotGUI::subscriber_info_callback, this);
  this->subscriber_odom = node->subscribe("/odom", 1000, &RobotGUI::subscriber_odom_callback, this);
  this->subscriber_cmd_vel = node->subscribe("/cmd_vel", 1000, &RobotGUI::subscriber_cmd_vel_callback, this);
  this->publisher_cmd_vel = node->advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  this->client_distance = node->serviceClient<std_srvs::Trigger>("/get_distance");
  this->client_reset_distance = node->serviceClient<std_srvs::Trigger>("/reset_distance");
  // node acknowledgement
  ROS_INFO("The node backend started successfully.");
}

// general info area
void RobotGUI::subscriber_info_callback(const robot_info::Packet::ConstPtr &msg) {
  std::vector<std::string> values(8);
  values[0] = msg->info_1;
  values[1] = msg->info_2;
  values[2] = msg->info_3;
  values[3] = msg->info_4;
  values[4] = msg->info_5;
  values[5] = msg->info_6;
  values[6] = msg->info_7;
  values[7] = msg->info_8;
  this->robot_info = values;
}

std::vector<std::string> RobotGUI::get_robot_info_packet() {
  // no conversion required as by default string
  return this->robot_info;
}

// robot position
void RobotGUI::subscriber_odom_callback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  this->odom_x = msg->pose.pose.position.x;
  this->odom_y = msg->pose.pose.position.y;
  this->odom_z = msg->pose.pose.position.z;
}

std::string RobotGUI::get_odom_x() {
  std::string value = std::to_string(this->odom_x);
  return value;
}

std::string RobotGUI::get_odom_y() {
  std::string value = std::to_string(this->odom_y);
  return value;
}

std::string RobotGUI::get_odom_z() {
  std::string value = std::to_string(this->odom_z);
  return value;
}

// current velocities
void RobotGUI::subscriber_cmd_vel_callback(
    const geometry_msgs::Twist::ConstPtr &msg) {
  this->cmd_vel_linear_x = msg->linear.x;
  this->cmd_vel_angular_z = msg->angular.z;
}

void RobotGUI::publisher_cmd_vel_callback() {
  geometry_msgs::Twist velocity;
  velocity.linear.x = this->linear_x_prime;
  velocity.angular.z = this->angular_z_prime;
  this->publisher_cmd_vel.publish(velocity);
}

std::string RobotGUI::get_cmd_vel_linear_x() {
  std::string value = std::to_string(this->cmd_vel_linear_x) + " m/sec";
  return value;
}

std::string RobotGUI::get_cmd_vel_angular_z() {
  std::string value = std::to_string(this->cmd_vel_angular_z) + " rad/sec";
  return value;
}

// teleoperation buttons
void RobotGUI::publisher_call_linear(double linear_x, double direction) {
  // publishing done by publisher_cmd_vel_callback()
  this->linear_x_prime = this->cmd_vel_linear_x + direction * linear_x;
  this->angular_z_prime = this->cmd_vel_angular_z;
}

void RobotGUI::publisher_call_angular(double angular_z, double direction) {
  // publishing done by publisher_cmd_vel_callback()
  geometry_msgs::Twist velocity;
  this->linear_x_prime = this->cmd_vel_linear_x;
  this->angular_z_prime = this->cmd_vel_angular_z + direction * angular_z;
}

void RobotGUI::publisher_call_halt() {
  // publishing done by publisher_cmd_vel_callback()
  this->linear_x_prime = 0;
  this->angular_z_prime = 0;
}

// distance travelled service
void RobotGUI::service_client_get_distance() {
  std_srvs::Trigger service;
  if (this->client_distance.call(service)) {
    this->distance = service.response.message.c_str();
    ROS_INFO("The service /get_distance successful.");
  } else {
    ROS_ERROR("The service /get_distance failed.");
  }
}

void RobotGUI::service_client_reset_distance() {
  std_srvs::Trigger service;
  if (this->client_reset_distance.call(service)) {
    this->distance = service.response.message.c_str();
    ROS_INFO("The service /reset_distance successful.");
  } else {
    ROS_ERROR("The service /reset_distance failed.");
  }
}

std::string RobotGUI::get_distance() {
  // no conversion required as by default string
  return this->distance;
}
