#ifndef ATTACH_SERVER_COMPONENT_HPP
#define ATTACH_SERVER_COMPONENT_HPP

#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"


namespace my_components {
class AttachServer : public rclcpp::Node {

public:
  COMPOSITION_PUBLIC
  explicit AttachServer(const rclcpp::NodeOptions &options);

protected:
  void service_callback(const std::shared_ptr<GoToLoading::Request>,
                        const std::shared_ptr<GoToLoading::Response>);
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr);
  void shelf_leg_detection();
  void publish_cart_frame_tf();
  void move_cart_legs_center();
  void move_under_cart();
  void lift_shelf();

private:
  rclcpp::CallbackGroup::SharedPtr srv_cbg;
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lift_shelf_pub;

  rclcpp::CallbackGroup::SharedPtr shelf_detection_cp;
  rclcpp::CallbackGroup::SharedPtr move_cart_cp;
  rclcpp::TimerBase::SharedPtr shelf_detection_timer;
  rclcpp::TimerBase::SharedPtr move_cart_center_timer;

  rclcpp::CallbackGroup::SharedPtr scan_callback_group_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

  std::vector<int> shelf_laser_indexes;
  int leg_2_first_index;
  sensor_msgs::msg::LaserScan::SharedPtr laser_scan_msg;
  float angle_increment;
  bool two_legs_detected = false;
  bool published_cart_frame = false;
  bool service_complete = false;
  bool reached_final_position = false;
  bool leg_detection_complete = false;
};
} // namespace my_components

#endif