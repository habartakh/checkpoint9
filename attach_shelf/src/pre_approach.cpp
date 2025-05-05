#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PreApproachNode : public rclcpp::Node {
public:
  PreApproachNode() : Node("pre_approach_node") {

    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0);
    getting_params();
    // RCLCPP_INFO(this->get_logger(), "Obstacle parameter is : %f", obstacle);

    cmd_vel_publisher =
        this->create_publisher<geometry_msgs::msg::Twist>("robot/cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PreApproachNode::timer_callback, this));

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PreApproachNode::scan_callback, this, _1));

    // odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
    //     "odom", 10, std::bind(&PreApproachNode::odom_callback, this, _1));
  }

private:
  void getting_params() {
    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<double>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
  }

  void timer_callback() {
    geometry_msgs::msg::Twist twist_cmd;

    // first advance the robot to the desired location
    //  right in front of the wall
    if (std::abs(front_distance - obstacle) > 0.1) {
      RCLCPP_INFO(this->get_logger(), "The front distance is: %f ",
                  front_distance);
      RCLCPP_INFO(this->get_logger(), "Full speed ahead!!");
      twist_cmd.linear.x = 0.2;
      twist_cmd.angular.z = 0.0;
    }

    else {
      RCLCPP_INFO(this->get_logger(), "Facing wall, stopping...");
      twist_cmd.linear.x = 0.0;
      twist_cmd.angular.z = 0.0;
    }

    cmd_vel_publisher->publish(twist_cmd);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    if (!msg->ranges.empty()) {

      auto ranges_middle_iterator =
          msg->ranges.begin() + (msg->ranges.size() / 2); // At the number 540

      // For a more robust computation of the front distance, we will use the
      // average of the distances of the 40 front rays of the laser sensor
      std::vector<double> front_ranges_copy(40);
      std::replace_copy_if(
          ranges_middle_iterator - 20, ranges_middle_iterator + 20,
          front_ranges_copy.begin(), [](double x) { return (std::isinf(x)); },
          0.0);

      front_distance = std::accumulate(front_ranges_copy.begin(),
                                       front_ranges_copy.end(), 0.0) /
                       front_ranges_copy.size();

      // front_distance = msg->ranges[540];

      //   RCLCPP_INFO(this->get_logger(), "The front distance is: %f ",
      //             front_distance);
    }
  }

  //   void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const {

  //     RCLCPP_INFO(this->get_logger(), "I heard odometry data");
  //   }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;

  double front_distance = 0.0;
  double obstacle;
  int degrees;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PreApproachNode>());
  rclcpp::shutdown();
  return 0;
}