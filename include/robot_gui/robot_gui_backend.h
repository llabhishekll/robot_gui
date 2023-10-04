#ifndef ROBOT_GUI_H
#define ROBOT_GUI_H

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <robot_info/Packet.h>
#include <ros/ros.h>
#include <string>
#include <vector>

class RobotGUI {
private:
  // member variables
  std::vector<std::string> robot_info;
  double cmd_vel_linear_x;
  double cmd_vel_angular_z;
  double linear_x_prime;
  double angular_z_prime;
  double odom_x;
  double odom_y;
  double odom_z;
  std::string distance;

  // ros objects
  ros::Subscriber subscriber_info;
  ros::Subscriber subscriber_odom;
  ros::Subscriber subscriber_cmd_vel;
  ros::Publisher publisher_cmd_vel;
  ros::ServiceClient client_distance;
  ros::ServiceClient client_reset_distance;

public:
  // constructor
  RobotGUI(ros::NodeHandle *node);

  // general info area
  void subscriber_info_callback(const robot_info::Packet::ConstPtr &msg);
  std::vector<std::string> get_robot_info_packet();

  // robot position
  void subscriber_odom_callback(const nav_msgs::Odometry::ConstPtr &msg);
  std::string get_odom_x();
  std::string get_odom_y();
  std::string get_odom_z();

  // current velocities
  void subscriber_cmd_vel_callback(const geometry_msgs::Twist::ConstPtr &msg);
  void publisher_cmd_vel_callback();
  std::string get_cmd_vel_linear_x();
  std::string get_cmd_vel_angular_z();

  // teleoperation buttons
  void publisher_call_linear(double linear_x, double direction);
  void publisher_call_angular(double angular_z, double direction);
  void publisher_call_halt();

  // distance travelled service
  void service_client_get_distance();
  void service_client_reset_distance();
  std::string get_distance();
};

#endif
