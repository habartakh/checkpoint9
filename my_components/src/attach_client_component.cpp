#include "my_components/attach_client_component.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using GoToLoading = attach_shelf::srv::GoToLoading;
using ServiceResponseFuture = rclcpp::Client<GoToLoading>::SharedFuture;

namespace my_components {
AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node("attach_client_component", options) {

  client_ = create_client<GoToLoading>("approach_shelf");
  timer_ = create_wall_timer(2s, std::bind(&AttachClient::on_timer, this));
}

void AttachClient::on_timer() {

  if (!client_->wait_for_service(1s)) {

    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(),
                   "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  auto request = std::make_shared<GoToLoading::Request>();
  request->attach_to_shelf = true;

  auto response_received_callback = [this](ServiceResponseFuture future) {
    auto status = future.wait_for(1s);

    if (status == std::future_status::ready) {
      auto response = future.get();
      std::string response_string = response->complete ? "True" : "False";
      RCLCPP_INFO(this->get_logger(), "Service returned the response: %s ",
                  response_string.c_str());
    }

    else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  };

  auto future_result =
      client_->async_send_request(request, response_received_callback);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)