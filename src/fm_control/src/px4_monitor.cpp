#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::VehicleStatus;
using px4_msgs::msg::VehicleLocalPosition;

class Px4Monitor : public rclcpp::Node
{
public:
  Px4Monitor() : Node("px4_monitor")
  {
    // QoS compatible avec les topics de sortie de PX4
    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().transient_local();

    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", qos,
      [this](const VehicleStatus::SharedPtr msg) { status_ = *msg; });

    position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) { position_ = *msg; });

    timer_ = create_wall_timer(500ms, [this]() { print_state(); });
    RCLCPP_INFO(get_logger(), "px4_monitor démarré");
  }

private:
  void print_state()
  {
    if (!status_ || !position_) {
      RCLCPP_WARN(get_logger(), "En attente des données PX4...");
      return;
    }

    const bool armed = status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
    const float altitude = -position_->z;                       // NED : z vers le bas
    const float heading_deg = position_->heading * 180.0f / static_cast<float>(M_PI);
    const bool valid = position_->xy_valid && position_->z_valid;

    RCLCPP_INFO(get_logger(),
      "%s | mode: %-9s | N: %6.2f m  E: %6.2f m  alt: %5.2f m | cap: %6.1f deg%s",
      armed ? "ARME   " : "DESARME",
      nav_state_name(status_->nav_state).c_str(),
      position_->x, position_->y, altitude, heading_deg,
      valid ? "" : "  (position invalide)");
  }

  static std::string nav_state_name(uint8_t state)
  {
    switch (state) {
      case VehicleStatus::NAVIGATION_STATE_MANUAL:       return "Manual";
      case VehicleStatus::NAVIGATION_STATE_ALTCTL:       return "Altitude";
      case VehicleStatus::NAVIGATION_STATE_POSCTL:       return "Position";
      case VehicleStatus::NAVIGATION_STATE_AUTO_MISSION: return "Mission";
      case VehicleStatus::NAVIGATION_STATE_AUTO_LOITER:  return "Hold";
      case VehicleStatus::NAVIGATION_STATE_AUTO_RTL:     return "Return";
      case VehicleStatus::NAVIGATION_STATE_AUTO_TAKEOFF: return "Takeoff";
      case VehicleStatus::NAVIGATION_STATE_AUTO_LAND:    return "Land";
      case VehicleStatus::NAVIGATION_STATE_OFFBOARD:     return "Offboard";
      default:                                           return "Autre(" + std::to_string(state) + ")";
    }
  }

  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::optional<VehicleStatus> status_;
  std::optional<VehicleLocalPosition> position_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Px4Monitor>());
  rclcpp::shutdown();
  return 0;
}