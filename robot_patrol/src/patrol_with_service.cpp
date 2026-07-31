#include <chrono>
#include <string>

#include "custom_interfaces/srv/get_direction.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace std::chrono_literals;
using GetDirection = custom_interfaces::srv::GetDirection;

class TestServiceClient : public rclcpp::Node {
public:
  TestServiceClient() : Node("test_service_client_node") {
    // Subscribe to Laser Topic
    auto qos_laser =
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos_laser,
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          this->laserscan_callback(msg);
        });
    // Init command Publisher
    command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);
    // Create the Service Client object
    std::string name_service = "/direction_service";
    client_ = this->create_client<GetDirection>(name_service);

    // Wait for the service to be available (checks every second)
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Interrupted while waiting for the service. Exiting.");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service %s not available, waiting again...",
                  name_service.c_str());
    }
    // Create timer for periodic service call
    auto timer_period = std::chrono::milliseconds(500);
    timer_ =
        this->create_wall_timer(timer_period, [this] { call_service_timer(); });
    RCLCPP_INFO(this->get_logger(), "Service Test Ready...");
  }

  void send_request(const std::shared_ptr<GetDirection::Request> request) {
    client_->async_send_request(
        request, [this](rclcpp::Client<GetDirection>::SharedFuture future) {
          try {
            const auto response = future.get();

            RCLCPP_INFO(this->get_logger(), "Direction to move: %s",
                        response->direction.c_str());

            publish_command(response->direction);
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s",
                         e.what());
          }
        });
  }

private:
  rclcpp::Client<GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  sensor_msgs::msg::LaserScan laser_data_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  bool laser_data_received_{false};
  bool request_in_progress_{false};

  void
  laserscan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    laser_data_ = *msg;
    laser_data_received_ = true;
  }
  void call_service_timer() {
    if (!laser_data_received_ || request_in_progress_) {
      return;
    }
    auto request = std::make_shared<GetDirection::Request>();
    request->laser_data = laser_data_;
    send_request(request);
  }

  void publish_command(const std::string &direction) {
    geometry_msgs::msg::Twist cmd;

    if (direction == "forward") {
      cmd.linear.x = 0.1;
    } else if (direction == "left") {
      cmd.linear.x = 0.1;
      cmd.angular.z = 0.5;
    } else if (direction == "right") {
      cmd.linear.x = 0.1;
      cmd.angular.z = -0.5;
    } else {
      RCLCPP_WARN(this->get_logger(), "Unknown direction: %s. Stopping robot.",
                  direction.c_str());
    }

    command_publisher_->publish(cmd);
  }
};

int main(int argc, char **argv) {
  // Initialize the ROS communication
  rclcpp::init(argc, argv);

  // Declare the node constructor
  auto client = std::make_shared<TestServiceClient>();

  rclcpp::spin(client);

  // Shutdown the ROS communication
  rclcpp::shutdown();
  return 0;
}