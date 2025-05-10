#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>
#include <string>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using GoToLoading = attach_shelf::srv::GoToLoading;

class PreApproachNodeV2 : public rclcpp::Node {
public:
  PreApproachNodeV2() : Node("pre_approach_v2_node") {

    this->declare_parameter("obstacle", 0.0);
    this->declare_parameter("degrees", 0);
    this->declare_parameter("final_approach", false);
    getting_params();

    state = State::FIRST_STEP; // init state of robot

    // We use MutuallyExclusive groups to manage the execution of the callbacks
    // Thus, each callback will be executed in a separate thread
    timer_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    scan_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions options1;
    options1.callback_group = scan_callback_group_;
    rclcpp::SubscriptionOptions options2;
    options2.callback_group = odom_callback_group_;

    cmd_vel_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diffbot_base_controller/cmd_vel_unstamped", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PreApproachNodeV2::timer_callback, this),
        timer_callback_group_);

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&PreApproachNodeV2::scan_callback, this, _1),
        options1);

    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&PreApproachNodeV2::odom_callback, this, _1),
        options2);

    approach_shelf_client = this->create_client<GoToLoading>("approach_shelf");
  }

private:
  void getting_params() {
    obstacle =
        this->get_parameter("obstacle").get_parameter_value().get<double>();
    degrees = this->get_parameter("degrees").get_parameter_value().get<int>();
    final_approach =
        this->get_parameter("final_approach").get_parameter_value().get<bool>();
  }

  // normalize angles to range [-pi, pi]
  double normalize_angle(double angle) {
    double normalized_angle = angle;
    while (angle > M_PI) {
      normalized_angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
      normalized_angle += 2 * M_PI;
    }
    return normalized_angle;
  }

  bool is_service_done() const { return this->service_done_; }

  void timer_callback() {
    switch (state) {
    case State::FIRST_STEP:
      first_step_pre_approach();
      break;

    case State::FIRST_STEP_DONE:
      start_heading = current_heading;
      state = State::SECOND_STEP;
      break;

    case State::SECOND_STEP:
      second_step_pre_approach(start_heading);
      break;

    case State::SECOND_STEP_DONE:
      RCLCPP_INFO_ONCE(this->get_logger(), "Pre-approach phase is finished!");
      state = State::ATTACH_TO_SHELF;
      break;

    // After the pre_approach is done, call the service
    case State::ATTACH_TO_SHELF:
      if (!service_called_) {
        RCLCPP_INFO(this->get_logger(), "Send Async Request");
        send_async_request();
      }
      break;
    }
  }

  // The first  step of the preapproach consists of advancing the robot to the
  // desired location "obstacle" meters away from the wall
  void first_step_pre_approach() {

    if (std::abs(front_distance - obstacle) > 0.05) {
      RCLCPP_INFO_ONCE(this->get_logger(),
                       "First step of the pre-approach : Moving forward...");
      twist_cmd.linear.x = 0.25;
      twist_cmd.angular.z = 0.0;
    } else {
      state = State::FIRST_STEP_DONE;
      twist_cmd.linear.x = 0.0;
      twist_cmd.angular.z = 0.0;
    }

    cmd_vel_publisher->publish(twist_cmd);
  }

  // The second step of the preapproach consists of turning the robot to the
  // desired heading provided by the user in the arguments
  void second_step_pre_approach(double starting_angle) {

    double target_angle =
        normalize_angle(starting_angle + ((double)degrees * M_PI / 180));
    double error_angle = current_heading - target_angle;

    if (std::abs(error_angle) > 0.05) {

      twist_cmd.linear.x = 0.0;
      twist_cmd.angular.z = (error_angle > 0 ? -0.3 : 0.3);
      RCLCPP_INFO_ONCE(this->get_logger(),
                       "Second step of the pre-approach: Turning ...");

    } else {
      twist_cmd.linear.x = 0.0;
      twist_cmd.angular.z = 0.0;
      state = State::SECOND_STEP_DONE;
    }
    cmd_vel_publisher->publish(twist_cmd);
  }

  // This callback computes the distance to the nearest obstacle
  // in front of the robot using the lidar rays
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
    }
  }

  // This callback provides the current heading of the robot
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // Convert odometry orientation quaternion to euler angles
    tf2::Quaternion quat_tf;
    geometry_msgs::msg::Quaternion quat_msg = msg->pose.pose.orientation;
    tf2::fromMsg(quat_msg, quat_tf);
    double roll{}, pitch{}, yaw{};
    tf2::Matrix3x3 m(quat_tf);
    m.getRPY(roll, pitch, yaw);

    current_heading = yaw; // in rads
  }

  void send_async_request() {
    while (!approach_shelf_client->wait_for_service(1s)) {

      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }

    auto request = std::make_shared<GoToLoading::Request>();
    request->attach_to_shelf = final_approach;
    auto result_future = approach_shelf_client->async_send_request(
        request, std::bind(&PreApproachNodeV2::response_callback, this,
                           std::placeholders::_1));
    service_called_ = true;

    // Now check for the response after a timeout of 1 second
    auto status = result_future.wait_for(1s);

    if (status != std::future_status::ready) {

      RCLCPP_WARN(this->get_logger(), "Response not ready yet.");
    }
  }

  // will be called when the service returns a response
  void response_callback(rclcpp::Client<GoToLoading>::SharedFuture future) {
    // Get response value
    auto response = future.get();
    std::string response_string = response->complete ? "True" : "False";
    RCLCPP_INFO(this->get_logger(), "Response: %s ", response_string.c_str());
    service_done_ = true;
    rclcpp::shutdown();
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Client<GoToLoading>::SharedPtr approach_shelf_client;

  rclcpp::CallbackGroup::SharedPtr timer_callback_group_;
  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

  double front_distance = 0.0;
  double current_heading = 0.0;
  double start_heading = 0.0; // heading before rotation of the robot
  double obstacle;
  int degrees;
  bool final_approach;

  geometry_msgs::msg::Twist twist_cmd;

  enum class State {
    FIRST_STEP,
    FIRST_STEP_DONE,
    SECOND_STEP,
    SECOND_STEP_DONE,
    ATTACH_TO_SHELF
  };
  State state;

  bool service_done_ = false;
  bool service_called_ = false;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<PreApproachNodeV2> pre_approach_node =
      std::make_shared<PreApproachNodeV2>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(pre_approach_node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}