#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>

enum class MoveDirection { FORWARD, LEFT, RIGHT };
enum class Sector { FrontLeft, FrontRight, Left, Right };

class Patrol : public rclcpp::Node {
public:
  Patrol() : Node("Patrol_bot") {
    // Subscribe to Laser Topic
    auto qos_laser =
        rclcpp::QoS(10).reliability(rclcpp::ReliabilityPolicy::BestEffort);
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
  std::map<Sector, std::pair<int, int>> sectors_;
  double FRONT_THRESHOLD = 0.35;

  void laserscan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    std::map<Sector, double> min_distances;
    std::map<Sector, double> max_distances;

    calculate_sectors(msg);

    for (const auto &sector : this->sectors_) {
      min_distances[sector.first] = get_min_distance(msg, sector.second);

      max_distances[sector.first] = get_max_distance(msg, sector.second);
    }

    if (_front_detected(min_distances)) {
      choose_safest_direction(max_distances);
    } else {
      move_direction_ = MoveDirection::FORWARD;
    }
  }

  // calculate sectors indicies based on number of rays, angle min and
  // increment.
  void calculate_sectors(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    const double DEG_TO_RAD = M_PI / 180.0;
    const double deg_20 = 20.0 * DEG_TO_RAD;
    const double deg_90 = 90.0 * DEG_TO_RAD;
    const double deg_360 = 2.0 * M_PI;

    sectors_[Sector::FrontLeft] = {_angle_to_index(0.0, msg),
                                   _angle_to_index(deg_20, msg)};
    sectors_[Sector::Left] = {_angle_to_index(deg_20, msg) + 1,
                              _angle_to_index(deg_90, msg)};
    sectors_[Sector::FrontRight] = {_angle_to_index(deg_360 - deg_20, msg),
                                    static_cast<int>(msg->ranges.size()) - 1};
    sectors_[Sector::Right] = {_angle_to_index(deg_360 - deg_90, msg),
                               _angle_to_index(deg_360 - deg_20, msg) - 1};
  }

  int _angle_to_index(double angle,
                      const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    int index = static_cast<int>(
        std::round((angle - msg->angle_min) / msg->angle_increment));

    const int max_index = static_cast<int>(msg->ranges.size()) - 1;

    return std::clamp(index, 0, max_index);
  }

  double get_min_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                          std::pair<int, int> sector) {
    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = sector.first; i <= sector.second; i++) {
      double scan = msg->ranges[i];

      if (_is_valid(scan, msg)) {
        min_distance = std::min(min_distance, scan);
      }
    }

    return min_distance;
  }

  double get_max_distance(const sensor_msgs::msg::LaserScan::SharedPtr msg,
                          std::pair<int, int> sector) {
    double max_distance = std::numeric_limits<double>::lowest();

    for (int i = sector.first; i <= sector.second; i++) {
      double scan = msg->ranges[i];

      if (_is_valid(scan, msg)) {
        max_distance = std::max(max_distance, scan);
      }
    }

    return max_distance;
  }
  [[nodiscard]] bool
  _is_valid(double scan, const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    if (!std::isfinite(scan))
      return false;
    if (scan < msg->range_min || scan > msg->range_max)
      return false;

    return true;
  }

  bool _front_detected(const std::map<Sector, double> &min_distances) {
    return (min_distances.at(Sector::FrontLeft) < FRONT_THRESHOLD or
            min_distances.at(Sector::FrontRight) < FRONT_THRESHOLD);
  }

  void choose_safest_direction(const std::map<Sector, double> &max_distances) {
    double left_max = std::max(max_distances.at(Sector::FrontLeft),
                               max_distances.at(Sector::Left));

    double right_max = std::max(max_distances.at(Sector::FrontRight),
                                max_distances.at(Sector::Right));

    Sector safest_sector;

    if (left_max > right_max) {
      move_direction_ = MoveDirection::LEFT;

      if (max_distances.at(Sector::FrontLeft) >
          max_distances.at(Sector::Left)) {
        safest_sector = Sector::FrontLeft;
      } else {
        safest_sector = Sector::Left;
      }
    } else {
      move_direction_ = MoveDirection::RIGHT;

      if (max_distances.at(Sector::FrontRight) >
          max_distances.at(Sector::Right)) {
        safest_sector = Sector::FrontRight;
      } else {
        safest_sector = Sector::Right;
      }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "Safest direction: %s | Best sector: %s | "
        "Front_Left: %.2f m | Left: %.2f m | "
        "Front_Right: %.2f m | Right: %.2f m",
        move_direction_ == MoveDirection::LEFT ? "LEFT" : "RIGHT",
        _sector_to_string(safest_sector), max_distances.at(Sector::FrontLeft),
        max_distances.at(Sector::Left), max_distances.at(Sector::FrontRight),
        max_distances.at(Sector::Right));
  }

  const char *_sector_to_string(Sector sector) {
    switch (sector) {
    case Sector::FrontLeft:
      return "Front_Left";

    case Sector::FrontRight:
      return "Front_Right";

    case Sector::Left:
      return "Left";

    case Sector::Right:
      return "Right";
    }

    return "Unknown";
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Patrol>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}