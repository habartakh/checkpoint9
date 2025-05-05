#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

float obstacle = 0.0;
int degrees = 0;

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach_node") {

    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PreApproachNode::timer_callback, this));

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PreApproachNode::scan_callback, this, _1));

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PreApproachNode::odom_callback, this, _1));
  }

private:
  void timer_callback() { publisher_->publish(); }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) const {

    RCLCPP_INFO(this->get_logger(), "I heard scan data");
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {

    RCLCPP_INFO(this->get_logger(), "I heard odometry data");
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachNode>());
  rclcpp::shutdown();
  return 0;
}