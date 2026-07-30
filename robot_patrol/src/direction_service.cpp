#include <chrono>
#include <limits>
#include <numeric>
#include <string>

#include "custom_interfaces/srv/get_direction.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <rclcpp/rclcpp.hpp>

enum class Sector { FrontLeft, FrontRight, Left, Right };

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
  std::map<Sector, std::pair<int, int>> sectors_;
  double FRONT_THRESHOLD = 0.35;

  void direction_callback(const std::shared_ptr<GetDirection::Request> request,
                          std::shared_ptr<GetDirection::Response> response) {

    std::map<Sector, double> min_distances_front;
    std::map<Sector, double> max_distances;
    const sensor_msgs::msg::LaserScan &laser_data = request->laser_data;

    calculate_sectors(laser_data);

    min_distances_front[Sector::FrontLeft] =
        get_min_distance(laser_data, sectors_.at(Sector::FrontLeft));
    min_distances_front[Sector::FrontRight] =
        get_min_distance(laser_data, sectors_.at(Sector::FrontRight));

    if (_front_detected(min_distances_front)) {
      response->direction = choose_safest_direction(laser_data);
    } else {
      response->direction = "forward";
    }
  }

  // calculate sectors indicies based on number of rays, angle min and
  // increment.
  void calculate_sectors(const sensor_msgs::msg::LaserScan &msg) {
    const double DEG_TO_RAD = M_PI / 180.0;
    const double deg_30 = 30.0 * DEG_TO_RAD;
    const double deg_90 = 90.0 * DEG_TO_RAD;
    const double deg_360 = 2.0 * M_PI;

    sectors_[Sector::FrontLeft] = {_angle_to_index(0.0, msg),
                                   _angle_to_index(deg_30, msg)};
    sectors_[Sector::Left] = {_angle_to_index(deg_30, msg) + 1,
                              _angle_to_index(deg_90, msg)};
    sectors_[Sector::FrontRight] = {_angle_to_index(deg_360 - deg_30, msg),
                                    static_cast<int>(msg.ranges.size()) - 1};
    sectors_[Sector::Right] = {_angle_to_index(deg_360 - deg_90, msg),
                               _angle_to_index(deg_360 - deg_30, msg) - 1};
  }

  int _angle_to_index(double angle, const sensor_msgs::msg::LaserScan &msg) {
    int index = static_cast<int>(
        std::round((angle - msg.angle_min) / msg.angle_increment));

    const int max_index = static_cast<int>(msg.ranges.size()) - 1;

    return std::clamp(index, 0, max_index);
  }
  double get_min_distance(const sensor_msgs::msg::LaserScan &msg,
                          std::pair<int, int> sector) {
    double min_distance = std::numeric_limits<double>::infinity();

    for (int i = sector.first; i <= sector.second; i++) {
      double scan = msg.ranges[i];

      if (_is_valid(scan, msg)) {
        min_distance = std::min(min_distance, scan);
      }
    }

    return min_distance;
  }

  [[nodiscard]] bool _is_valid(double scan,
                               const sensor_msgs::msg::LaserScan &msg) {
    if (!std::isfinite(scan))
      return false;
    if (scan < msg.range_min || scan > msg.range_max)
      return false;

    return true;
  }

  bool _front_detected(const std::map<Sector, double> &min_distances) {
    return (min_distances.at(Sector::FrontLeft) < FRONT_THRESHOLD or
            min_distances.at(Sector::FrontRight) < FRONT_THRESHOLD);
  }

  std::string choose_safest_direction(const sensor_msgs::msg::LaserScan &msg) {

    auto f = [this, &msg](double acc, double scan) {
      return _is_valid(scan, msg) ? acc + scan : acc;
    };
    double sum_left = std::accumulate(
        msg.ranges.begin() + sectors_.at(Sector::Left).first,
        msg.ranges.begin() + sectors_.at(Sector::Left).second, 0.0, f);
    double sum_right = std::accumulate(
        msg.ranges.begin() + sectors_.at(Sector::Right).first,
        msg.ranges.begin() + sectors_.at(Sector::Right).second, 0.0, f);

    if (sum_left > sum_right) {
      return "left";
    }
    return "right";
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
  auto node = std::make_shared<DirectionService>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}