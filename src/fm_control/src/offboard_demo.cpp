#include <chrono>
#include <cmath>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleStatus;

class OffboardDemo : public rclcpp::Node
{
public:
  OffboardDemo() : Node("offboard_demo")
  {
    offboard_mode_pub_ = create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
    setpoint_pub_ = create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
    command_pub_ = create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

    auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().transient_local();
    status_sub_ = create_subscription<VehicleStatus>(
      "/fmu/out/vehicle_status_v4", qos,
      [this](const VehicleStatus::SharedPtr msg) { status_ = *msg; });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack_v1", qos,
      [this](const VehicleCommandAck::SharedPtr msg) { on_ack(*msg); });

    timer_ = create_wall_timer(50ms, [this]() { on_timer(); });   // 20 Hz
    RCLCPP_INFO(get_logger(), "offboard_demo v2 demarre");
  }

private:
  enum class State { Warmup, RequestOffboard, RequestArm, Flying, Aborted };

  static constexpr int kWarmupTicks = 20;   // 1 s de consignes avant la bascule
  static constexpr int kRetryTicks = 20;    // une tentative par seconde
  static constexpr int kMaxAttempts = 5;

  // ---------- Boucle principale (20 Hz) ----------
  void on_timer()
  {
    if (state_ == State::Aborted) {
      return;   // on n'emet plus rien : PX4 ne peut plus repasser en Offboard sur nos consignes
    }

    publish_heartbeat();
    publish_position_setpoint(0.0f, 0.0f, -2.0f);
    ++ticks_in_state_;

    switch (state_) {
      case State::Warmup:
        if (status_ && ticks_in_state_ >= kWarmupTicks) {
          change_state(State::RequestOffboard);
        }
        break;

      case State::RequestOffboard:
        if (is_offboard()) {
          change_state(State::RequestArm);
        } else if (should_send_attempt()) {
          send_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0f, 6.0f);
        }
        break;

      case State::RequestArm:
        if (!is_offboard()) {
          abort("le mode a quitte Offboard avant l'armement");
        } else if (is_armed()) {
          change_state(State::Flying);
        } else if (should_send_attempt()) {
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case State::Flying:
        if (!is_offboard() || !is_armed()) {
          abort("PX4 a quitte Offboard ou a desarme : je rends la main");
        }
        break;

      case State::Aborted:
        break;
    }
  }

  // ---------- Machine a etats ----------
  void change_state(State next)
  {
    RCLCPP_INFO(get_logger(), "Etat : %s -> %s", state_name(state_), state_name(next));
    state_ = next;
    ticks_in_state_ = 0;
    attempts_ = 0;
  }

  void abort(const std::string & reason)
  {
    RCLCPP_ERROR(get_logger(), "ABANDON (%s) : %s", state_name(state_), reason.c_str());
    state_ = State::Aborted;
  }

  bool should_send_attempt()
  {
    // Premiere tentative au 1er tick de l'etat, puis une par seconde.
    if (ticks_in_state_ % kRetryTicks != 1) {
      return false;
    }
    if (attempts_ >= kMaxAttempts) {
      abort("aucun effet apres " + std::to_string(kMaxAttempts) + " tentatives");
      return false;
    }
    ++attempts_;
    return true;
  }

  bool is_offboard() const
  {
    return status_ && status_->nav_state == VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  bool is_armed() const
  {
    return status_ && status_->arming_state == VehicleStatus::ARMING_STATE_ARMED;
  }

  static const char * state_name(State s)
  {
    switch (s) {
      case State::Warmup:          return "Warmup";
      case State::RequestOffboard: return "RequestOffboard";
      case State::RequestArm:      return "RequestArm";
      case State::Flying:          return "Flying";
      case State::Aborted:         return "Aborted";
    }
    return "?";
  }

  // ---------- Accuses de reception ----------
  void on_ack(const VehicleCommandAck & ack)
  {
    RCLCPP_INFO(get_logger(), "ACK commande %u : %s", ack.command, result_name(ack.result));
  }

  static const char * result_name(uint8_t result)
  {
    switch (result) {
      case VehicleCommandAck::VEHICLE_CMD_RESULT_ACCEPTED:             return "ACCEPTED";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED: return "TEMPORARILY_REJECTED";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_DENIED:               return "DENIED";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_UNSUPPORTED:          return "UNSUPPORTED";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_FAILED:               return "FAILED";
      case VehicleCommandAck::VEHICLE_CMD_RESULT_IN_PROGRESS:          return "IN_PROGRESS";
      default:                                                         return "autre code";
    }
  }

  // ---------- Publications vers PX4 ----------
  uint64_t now_us()
  {
    return get_clock()->now().nanoseconds() / 1000;
  }

  void publish_heartbeat()
  {
    OffboardControlMode msg{};
    msg.position = true;
    msg.timestamp = now_us();
    offboard_mode_pub_->publish(msg);
  }

  void publish_position_setpoint(float north, float east, float down)
  {
    const float nan = std::nanf("");
    TrajectorySetpoint msg{};
    msg.position = {north, east, down};
    msg.velocity = {nan, nan, nan};
    msg.acceleration = {nan, nan, nan};
    msg.yaw = 0.0f;
    msg.yawspeed = nan;
    msg.timestamp = now_us();
    setpoint_pub_->publish(msg);
  }

  void send_command(uint32_t command, float param1 = 0.0f, float param2 = 0.0f)
  {
    VehicleCommand msg{};
    msg.command = command;
    msg.param1 = param1;
    msg.param2 = param2;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = now_us();
    command_pub_->publish(msg);
    RCLCPP_INFO(get_logger(), "Commande %u envoyee (tentative %d/%d)", command, attempts_, kMaxAttempts);
  }

  // ---------- Attributs ----------
  rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_mode_pub_;
  rclcpp::Publisher<TrajectorySetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<VehicleCommand>::SharedPtr command_pub_;
  rclcpp::Subscription<VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<VehicleStatus> status_;
  State state_ = State::Warmup;
  int ticks_in_state_ = 0;
  int attempts_ = 0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardDemo>());
  rclcpp::shutdown();
  return 0;
}