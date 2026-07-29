#include <chrono>
#include <string>

#include "custom_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

class DirectionService : public rclcpp::Node {
public:
  using GetDirection = custom_interfaces::srv::GetDirection;

  DirectionService() : Node("direction_srv_server_node") {
    // Create a service that will handle status queries
    std::string name_service = "/direction_service";
    service_ = this->create_service<GetDirection>(
        name_service,
        [this](const std::shared_ptr<GetDirection::Request> request,
               std::shared_ptr<GetDirection::Response> response) {
          direction_callback(request, response);
        });

    RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                name_service.c_str());
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr service_;

  void direction_callback(const std::shared_ptr<GetDirection::Request> request,
                          std::shared_ptr<GetDirection::Response> response) {}
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}