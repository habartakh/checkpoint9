#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/exceptions.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"

#include <algorithm>
#include <cmath>
#include <memory>

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

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ApproachShelfServer::scan_callback, this, _1));

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Initialize the tf listener
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  }

private:
  bool final_approach;
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<int> shelf_laser_indexes;
  float angle_increment;
  bool two_legs_detected = false;
  bool broadcast_cart_tf = false;

  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response) {

    request->attach_to_shelf = final_approach; // set the request
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    angle_increment = msg->angle_increment;
    shelf_leg_detection(msg->intensities);
    publish_cart_frame_tf(msg->ranges);
    // approach_middle_shelf();
  }

  void shelf_leg_detection(std::vector<float> &intensities_vector) {
    // store all the indexes of the rays having intensities >= 8000
    // that corresponf to the shelf legs inside a vector
    for (auto it = intensities_vector.begin(); it != intensities_vector.end();
         ++it) {
      if (*it >= 8000) {
        int shelf_index = it - intensities_vector.begin();
        shelf_laser_indexes.push_back(shelf_index);
      }
    }

    // find the number of the shelf legs detected
    // if two legs were detected, the intensity indexes won't be consecutive
    auto it = std::adjacent_find(shelf_laser_indexes.begin(),
                                 shelf_laser_indexes.end(),
                                 [](int x, int y) { return y != x + 1; });

    // if the two legs were detected
    if (it != shelf_laser_indexes.end()) {
      two_legs_detected = true;
    }
  }

  // Publish a transform named cart_frame to the center point between both legs
  void publish_cart_frame_tf(std::vector<float> &ranges_vector) {

    geometry_msgs::msg::TransformStamped t;

    // Read message content and assign it to
    // corresponding tf variables
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "robot_base_footprint";
    t.child_frame_id = "cart_frame";

    // Compute the distance of the robot to the middle point between the two
    // shelf legs
    float angle_shelf_legs =
        (shelf_laser_indexes.back() - shelf_laser_indexes[0] + 1) *
        angle_increment;

    float distance_to_shelf =
        ranges_vector[shelf_laser_indexes[0]] * std::cos(angle_shelf_legs / 2);
    // std::cout << "distance_to_shelf : " << distance_to_shelf << std::endl;

    // Turtle only exists in 2D, thus we get x and y translation
    // coordinates from the message and set the z coordinate to 0
    t.transform.translation.x = distance_to_shelf;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.0;

    // Orientation does not matter as much as translation
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    // Send the transformation
    tf_broadcaster_->sendTransform(t);
  }

  
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachShelfServer>());
  rclcpp::shutdown();
  return 0;
}
