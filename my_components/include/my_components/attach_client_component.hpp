#ifndef ATTACH_CLIENT_COMPONENT_HPP_
#define ATTACH_CLIENT_COMPONENT_HPP_

#include "attach_shelf/srv/go_to_loading.hpp"
#include "my_components/visibility_control.h"
#include "rclcpp/rclcpp.hpp"

using GoToLoading = attach_shelf::srv::GoToLoading;

namespace my_components {

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC
  explicit AttachClient(const rclcpp::NodeOptions &options);

protected:
  void on_timer();

private:
  rclcpp::Client<GoToLoading>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace my_components

#endif // COMPOSITION__CLIENT_COMPONENT_HPP_