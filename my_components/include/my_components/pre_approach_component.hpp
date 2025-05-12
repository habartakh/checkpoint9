#ifndef PRE_APPROACH_COMPONENT_HPP
#define PRE_APPROACH_COMPONENT_HPP

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace my_components {
class PreApproach: public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit PreApproach(const rclcpp::NodeOptions &options);

protected:
  double normalize_angle(double);
  void timer_callback();
  void first_step_pre_approach();
  void second_step_pre_approach(double);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr);
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr);

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  double front_distance = 0.0;
  double current_heading = 0.0;
  double start_heading = 0.0; // heading before rotation of the robot
  double obstacle;
  int degrees;

  geometry_msgs::msg::Twist twist_cmd;

  enum class State {
    FIRST_STEP,
    FIRST_STEP_DONE,
    SECOND_STEP,
    SECOND_STEP_DONE
  };
  State state;
};

} // namespace my_components

#endif