#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

enum class MoveDirection { FORWARD, LEFT, RIGHT };

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("Patrol_bot") {
    // Subscribe to Laser Topic
    auto qos_laser =
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::Reliable);
    subscriber_laser_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/fastbot_1/scan", qos_laser,
        [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
          this->laserscan_callback(msg);
        });
    // Init command Publisher
    command_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/fastbot_1/cmd_vel", 10);
    auto timer_period = std::chrono::milliseconds(100);
    timer_ =
        this->create_wall_timer(timer_period, [this] { timer_callback(); });
    RCLCPP_INFO(this->get_logger(), "Patrol_bot is alive...");
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  MoveDirection move_direction_ = MoveDirection::FORWARD;

  void timer_callback() {
    auto cmd = geometry_msgs::msg::Twist();
    if (move_direction_ == MoveDirection::FORWARD) {
      cmd.linear.x = 0.1;
    } else if (move_direction_ == MoveDirection::LEFT) {
      cmd.linear.x = 0.05;
      cmd.angular.z = 0.5;
    } else {
      cmd.linear.x = 0.05;
      cmd.angular.z = -0.5;
    }
    command_publisher_->publish(cmd);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      subscriber_laser_;
  std::map<std::string, std::pair<int, int>> sectors_ = {
      {"Front_Right", {189, 199}}, // +20 deg
      {"Front_Left", {0, 10}},     // -20 deg
      {"Left", {11, 50}},          // -90 deg
      {"Right", {150, 188}},       // +90 deg
  };
  double FRONT_THRESHOLD = 0.35;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    std::map<std::string, double> min_distances;
    std::map<std::string, double> max_distances;

    for (const auto &sector : this->sectors_) {
      min_distances[sector.first] = get_min_distance(msg, sector.second);

      max_distances[sector.first] = get_max_distance(msg, sector.second);
    }

    if (_front_detected(min_distances)) {
      choose_safest_direction(max_distances);
    }
  }

  double get_min_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                          std::pair<int, int> sector) {
    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = sector.first; i <= sector.second; i++) {
      double scan = msg->ranges[i];

      _is_valid(scan, msg);

      min_distance = std::min(min_distance, scan);
    }

    return min_distance;
  }

  double get_max_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                          std::pair<int, int> sector) {
    double max_distance = std::numeric_limits<double>::lowest();

    for (int i = sector.first; i <= sector.second; i++) {
      double scan = msg->ranges[i];

      _is_valid(scan, msg);

      max_distance = std::max(max_distance, scan);
    }

    return max_distance;
  }
  bool _is_valid(double scan,
                 const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!std::isfinite(scan))
      return false;
    if (scan < msg->range_min || scan > msg->range_max)
      return false;

    return true;
  }

  bool _front_detected(const std::map<std::string, double> &min_distances) {
    return (min_distances.at("Front_Left") < FRONT_THRESHOLD or
            min_distances.at("Front_Right") < FRONT_THRESHOLD);
  }

  void
  choose_safest_direction(const std::map<std::string, double> &max_distances) {
    double left_max =
        std::max(max_distances.at("Front_Left"), max_distances.at("Left"));

    double right_max =
        std::max(max_distances.at("Front_Right"), max_distances.at("Right"));

    std::string safest_sector;

    if (left_max > right_max) {
      move_direction_ = MoveDirection::LEFT;

      if (max_distances.at("Front_Left") > max_distances.at("Left")) {
        safest_sector = "Front_Left";
      } else {
        safest_sector = "Left";
      }
    } else {
      move_direction_ = MoveDirection::RIGHT;

      if (max_distances.at("Front_Right") > max_distances.at("Right")) {
        safest_sector = "Front_Right";
      } else {
        safest_sector = "Right";
      }
    }

    RCLCPP_INFO(this->get_logger(),
                "Safest direction: %s | Best sector: %s | "
                "Front_Left: %.2f m | Left: %.2f m | "
                "Front_Right: %.2f m | Right: %.2f m",
                move_direction_ == MoveDirection::LEFT ? "LEFT" : "RIGHT",
                safest_sector.c_str(), max_distances.at("Front_Left"),
                max_distances.at("Left"), max_distances.at("Front_Right"),
                max_distances.at("Right"));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}