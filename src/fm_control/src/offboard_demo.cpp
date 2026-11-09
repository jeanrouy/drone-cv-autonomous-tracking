#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;

class OffboardDemo : public rclcpp::Node
{
public:
  OffboardDemo() : Node("offboard_demo")
  {
    offboard_mode_pub_ = create_publisher<OffboardControlMode>(
      "/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>(
      "/fmu/in/trajectory_setpoint", 10);

    timer_ = create_wall_timer(50ms, [this]() { on_timer(); });   // 20 Hz
    RCLCPP_INFO(get_logger(), "offboard_demo v1 : heartbeat + consigne (0, 0, -2) a 20 Hz");
  }

private:
  void on_timer()
  {
    publish_heartbeat();
    publish_position_setpoint(0.0f, 0.0f, -2.0f);   // NED : 2 m au-dessus du point de depart
  }

  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  void publish_heartbeat()
  {
    OffboardControlMode msg{};
    msg.position = true;          // on pilote au niveau "position" de la cascade
    msg.timestamp = now_us();
    offboard_mode_pub_->publish(msg);
  }

  void publish_position_setpoint(float north, float east, float down)
  {
    const float nan = std::nanf("");
    TrajectorySetpoint msg{};
    msg.position = {north, east, down};
    msg.velocity = {nan, nan, nan};       // non controle
    msg.acceleration = {nan, nan, nan};   // non controle
    msg.yaw = 0.0f;                       // cap vers le nord
    msg.yawspeed = nan;
    msg.timestamp = now_us();
    setpoint_pub_->publish(msg);
  }

  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardDemo>());
  rclcpp::shutdown();
  return 0;
}