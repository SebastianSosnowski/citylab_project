#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("Patrol_bot") {
    RCLCPP_INFO(this->get_logger(), "Patrol_bot is alive...");
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}