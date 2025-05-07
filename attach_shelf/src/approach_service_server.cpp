#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <algorithm>
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
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    scan_sub = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 10, std::bind(&ApproachShelfServer::scan_callback, this, _1));
  }

private:
  bool final_approach;
  rclcpp::Service<GoToLoading>::SharedPtr srv_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub;

  bool two_legs_detected = false;

  void service_callback(const std::shared_ptr<GoToLoading::Request> request,
                        const std::shared_ptr<GoToLoading::Response> response) {

    request->attach_to_shelf = final_approach; // set the request
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    shelf_leg_detection(msg->intensities);

    // std::cout << "laser scan intensities size : " << msg->intensities.size()
    //           << std::endl;
  }

  void shelf_leg_detection(std::vector<float> intensities_vector) {

    // strore all the indexes of the rays having intensities >= 8000
    // inside the shelf_laser_indexes vector
    std::vector<int> shelf_laser_indexes;
    for (auto it = intensities_vector.begin(); it != intensities_vector.end();
         ++it) {
      if (*it >= 8000) {
        int shelf_index = it - intensities_vector.begin();
        shelf_laser_indexes.push_back(shelf_index);
        std::cout << "shelf_index = " << shelf_index
                  << ", intensity_value = " << *it << std::endl;
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

      //   std::cout << "*it = " << *it << std::endl;
      // rclcpp::shutdown();
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachShelfServer>());
  rclcpp::shutdown();
  return 0;
}
