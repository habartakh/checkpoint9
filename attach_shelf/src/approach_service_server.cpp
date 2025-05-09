#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2/transform_datatypes.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <numeric>

using namespace std::chrono_literals;
using GoToLoading = attach_shelf::srv::GoToLoading;
using std::placeholders::_1;
using std::placeholders::_2;

class ApproachShelfServer : public rclcpp::Node {
public:
  ApproachShelfServer() : Node("service_stop") {

    this->declare_parameter("final_approach", false);
    final_approach =
        this->get_parameter("final_approach").get_parameter_value().get<bool>();

    srv_ = create_service<GoToLoading>(
        "approach_shelf",
        std::bind(&ApproachShelfServer::service_callback, this, _1, _2));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "diffbot_base_controller/cmd_vel_unstamped", 10);

    timer_ = this->create_wall_timer(
        500ms, std::bind(&ApproachShelfServer::control_loop, this));
    // timer_->cancel(); // Cancel the timer till we start the service

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ApproachShelfServer::scan_callback, this, _1));

    // Initialize the transform broadcaster
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Initialize the tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    published_cart_frame = false;
  }

private:
  bool final_approach;
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<int> shelf_laser_indexes;
  int leg_2_first_index;
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
  float angle_increment;
  bool two_legs_detected = false;
  bool broadcast_cart_tf = false;
  bool published_cart_frame = false;

  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response) {

    request->attach_to_shelf = final_approach; // set the request

    if (request->attach_to_shelf) {
      RCLCPP_INFO(this->get_logger(), " Initiating final approach procedure.");
      timer_->reset(); // starts the timer and execute control loop
    }

    else {
      timer_->cancel(); // stop the timer
      RCLCPP_INFO(this->get_logger(),
                  " Not initiating final approach procedure.");
    }
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    angle_increment = msg->angle_increment;
    laser_scan_msg = msg;
  }

  void control_loop() {

    if (published_cart_frame == false) {
      shelf_leg_detection();
      publish_cart_frame_tf();
      RCLCPP_INFO(this->get_logger(), " Published cart_frame ONCE!!!!!! ");
      published_cart_frame = true;
    }

    // approach_cart_legs_center();
  }

  void shelf_leg_detection() {

    // store all the indexes of the rays having intensities >= 8000
    // that correspond to the shelf legs inside a vector
    for (auto it = laser_scan_msg->intensities.begin();
         it != laser_scan_msg->intensities.end(); ++it) {
      if (*it >= 8000) {
        int shelf_index = it - laser_scan_msg->intensities.begin();
        shelf_laser_indexes.push_back(shelf_index);
      }
    }

    // find the number of the shelf legs detected
    // if two legs were detected, the intensity indexes won't be consecutive
    auto it = std::adjacent_find(shelf_laser_indexes.begin(),
                                 shelf_laser_indexes.end(),
                                 [](int x, int y) { return y != x + 1; });
    leg_2_first_index = it - shelf_laser_indexes.begin();

    // if the two legs were detected
    if (it != shelf_laser_indexes.end()) {
      two_legs_detected = true;
    }
  }

  // Publish a transform named cart_frame to the center point between both legs
  void publish_cart_frame_tf() {

    // Compute the coordinates of the point in the middle of the 2 legs
    // First, compute projection of both legs on x & y axes
    int middle_ray_index = (int)laser_scan_msg->intensities.size() / 2;

    int leg_1_index = shelf_laser_indexes[0];
    double leg1_angle = (leg_1_index - middle_ray_index) * angle_increment;
    double leg1_x = -laser_scan_msg->ranges[leg_1_index] * std::sin(leg1_angle);
    double leg1_y = -laser_scan_msg->ranges[leg_1_index] * std::cos(leg1_angle);

    int leg_2_index = shelf_laser_indexes.back();
    double leg2_angle = (leg_2_index - middle_ray_index) * angle_increment;
    double leg2_x = laser_scan_msg->ranges[leg_2_index] * std::cos(leg2_angle);
    double leg2_y = laser_scan_msg->ranges[leg_2_index] * std::sin(leg2_angle);

    double middle_point_x = (leg1_x + leg2_x) / 2.0;
    double middle_point_y = (leg1_y + leg2_y) / 2.0;

    std::cout << "middle_ray_index : " << middle_ray_index << std::endl;
    std::cout << "leg_1_index : " << leg_1_index << std::endl;
    std::cout << "leg_2_index : " << leg_2_index << std::endl;
    std::cout << "leg2_angle : " << leg2_angle << std::endl;
    std::cout << "middle_point_x : " << middle_point_x << std::endl;
    std::cout << "middle_point_y : " << middle_point_y << std::endl;

    /*********************************************************************************************/

    // To get the tf between odom and the cart frame :
    // First, compute the pose of the cart_frame origin relative to laser_frame
    // Then, get the coordinates of the cart_frame origin relative to odom_frame
    geometry_msgs::msg::PoseStamped laser_pose;
    laser_pose.header.frame_id = "robot_front_laser_base_link";
    laser_pose.header.stamp = this->get_clock()->now();
    laser_pose.pose.position.x = middle_point_x;
    laser_pose.pose.position.y = middle_point_y;
    laser_pose.pose.position.z = 0.0;
    laser_pose.pose.orientation.w = 1.0;

    geometry_msgs::msg::PoseStamped odom_pose;
    geometry_msgs::msg::TransformStamped odom_to_laser_tf;
    try {
      odom_to_laser_tf = tf_buffer_->lookupTransform(
          "odom", "robot_front_laser_base_link", tf2::TimePointZero);
      tf2::doTransform(laser_pose, odom_pose, odom_to_laser_tf);
    }

    catch (tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
      return;
    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "odom";
    t.child_frame_id = "cart_frame";
    t.transform.translation.x = odom_pose.pose.position.x - 0.07;
    t.transform.translation.y = odom_pose.pose.position.y;
    t.transform.translation.z = 0.0;

    // Orientation
    tf2::Quaternion q;
    q.setRPY(0, 0, -1.57);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_static_broadcaster_->sendTransform(t);
  }

  void approach_cart_legs_center() {
    std::string fromFrameRel = "cart_frame";
    std::string toFrameRel = "robot_base_footprint";
    geometry_msgs::msg::TransformStamped t;

    // Look up for the transformation between target_frame and robot frames
    // and send velocity commands for robot to reach target_frame
    try {
      t = tf_buffer_->lookupTransform(toFrameRel, fromFrameRel,
                                      tf2::TimePointZero);
    } catch (const tf2::TransformException &ex) {
      RCLCPP_INFO(this->get_logger(), "Could not transform %s to %s: %s",
                  toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
      return;
    }

    auto x = t.transform.translation.x;
    auto y = t.transform.translation.y;

    float error_distance = std::sqrt(x * x + y * y);
    float error_yaw = std::atan2(y, x);
    RCLCPP_INFO(this->get_logger(), "Error distance: %.4f", error_distance);
    RCLCPP_INFO(this->get_logger(), "Error yaw: %.4f", error_yaw);

    geometry_msgs::msg::Twist msg;
    if (error_distance > 0.05) {
      // msg.angular.z = -0.5 * error_yaw;
      msg.angular.z = 0;
      msg.linear.x = std::min(1.0 * error_distance, 0.1);
      RCLCPP_INFO(this->get_logger(), "Z angular: %.3f", msg.angular.z);
      RCLCPP_INFO(this->get_logger(), "X linear: %.3f", msg.linear.x);
    } else {
      msg.angular.z = 0;
      msg.linear.x = 0;
      RCLCPP_INFO(this->get_logger(), "Final approach completed.");
    }

    publisher_->publish(msg);

    // static const double scaleRotationRate = 1.0;
    // // msg.angular.z = scaleRotationRate *
    // //                 atan2(t.transform.translation.y,
    // //                 t.transform.translation.x);
    // msg.angular.z = 0.0;

    // static const double scaleForwardSpeed = 0.5;
    // msg.linear.x = scaleForwardSpeed * sqrt(pow(t.transform.translation.x, 2)
    // +
    //                                         pow(t.transform.translation.y,
    //                                         2));
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachShelfServer>());
  rclcpp::shutdown();
  return 0;
}
