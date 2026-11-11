#include <chrono>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"

using namespace std::chrono_literals;
using px4_msgs::msg::OffboardControlMode;
using px4_msgs::msg::TrajectorySetpoint;
using px4_msgs::msg::VehicleCommand;
using px4_msgs::msg::VehicleCommandAck;
using px4_msgs::msg::VehicleLocalPosition;
using px4_msgs::msg::VehicleStatus;

// Point de mission, relatif au point de depart
struct Waypoint
{
  float north;    // m
  float east;     // m
  float up;       // m, altitude au-dessus du depart (positif vers le haut)
  double hold_s;  // temps de maintien une fois le point atteint
};

// Position dans le repere NED de PX4
struct Ned
{
  float north;
  float east;
  float down;
};

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
    position_sub_ = create_subscription<VehicleLocalPosition>(
      "/fmu/out/vehicle_local_position_v1", qos,
      [this](const VehicleLocalPosition::SharedPtr msg) { position_ = *msg; });
    ack_sub_ = create_subscription<VehicleCommandAck>(
      "/fmu/out/vehicle_command_ack_v1", qos,
      [this](const VehicleCommandAck::SharedPtr msg) { on_ack(*msg); });

    mission_ = {
      {0.0f, 0.0f, 2.0f, 10.0},   // decollage a 2 m, maintien 10 s
      {4.0f, 0.0f, 2.0f, 2.0},    // carre de 4 m
      {4.0f, 4.0f, 2.0f, 2.0},
      {0.0f, 4.0f, 2.0f, 2.0},
      {0.0f, 0.0f, 2.0f, 2.0},    // retour au-dessus du depart
    };

    timer_ = create_wall_timer(50ms, [this]() { on_timer(); });   // 20 Hz
    RCLCPP_INFO(get_logger(), "offboard_demo v3 demarre (%zu points de mission)", mission_.size());
  }

private:
  enum class State { Warmup, RequestOffboard, RequestArm, Mission, Landing, Done, Aborted };

  static constexpr int kWarmupTicks = 20;            // 1 s de consignes avant la bascule
  static constexpr int kRetryTicks = 20;             // une tentative par seconde
  static constexpr int kMaxAttempts = 5;
  static constexpr float kReachedTolerance = 0.3f;   // m
  static constexpr double kWaypointTimeout = 20.0;   // s pour atteindre un point

  // ---------- Boucle principale (20 Hz) ----------
  void on_timer()
  {
    if (should_stream()) {
      publish_heartbeat();
      publish_setpoint(mission_[wp_index_]);
    }
    ++ticks_in_state_;

    switch (state_) {
      case State::Warmup:
        if (!home_) {
          capture_home();
        } else if (status_ && ticks_in_state_ >= kWarmupTicks) {
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
          start_mission();
        } else if (should_send_attempt()) {
          send_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
        }
        break;

      case State::Mission:
        if (!is_offboard() || !is_armed()) {
          abort("PX4 a quitte Offboard ou a desarme : je rends la main");
        } else {
          update_mission();
        }
        break;

      case State::Landing:
        if (!is_armed()) {
          change_state(State::Done);
          RCLCPP_INFO(get_logger(), "Mission terminee : drone pose et desarme");
        } else if (!is_landing() && should_send_attempt()) {
          send_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        }
        break;

      case State::Done:
      case State::Aborted:
        break;
    }
  }

  // On emet le flux Offboard tant qu'on pilote, et pendant l'atterrissage
  // tant que PX4 n'a pas encore quitte l'Offboard (evite la course avec le failsafe).
  bool should_stream() const
  {
    if (!home_) {
      return false;
    }
    switch (state_) {
      case State::Warmup:
      case State::RequestOffboard:
      case State::RequestArm:
      case State::Mission:
        return true;
      case State::Landing:
        return is_offboard();
      case State::Done:
      case State::Aborted:
        return false;
    }
    return false;
  }

  // ---------- Mission ----------
  void capture_home()
  {
    if (position_ && position_->xy_valid && position_->z_valid) {
      home_ = Ned{position_->x, position_->y, position_->z};
      ticks_in_state_ = 0;   // la chauffe commence maintenant
      RCLCPP_INFO(get_logger(), "Point de depart : N %.2f  E %.2f  D %.2f",
        home_->north, home_->east, home_->down);
    }
  }

  void start_mission()
  {
    change_state(State::Mission);
    wp_index_ = 0;
    arrived_time_.reset();
    waypoint_start_time_ = get_clock()->now();
    RCLCPP_INFO(get_logger(), "Cap sur le point 1/%zu", mission_.size());
  }

  void update_mission()
  {
    const rclcpp::Time now = get_clock()->now();

    if (!arrived_time_) {
      // En route vers le point courant
      if (distance_to(mission_[wp_index_]) < kReachedTolerance) {
        arrived_time_ = now;
        RCLCPP_INFO(get_logger(), "Point %zu/%zu atteint", wp_index_ + 1, mission_.size());
      } else if ((now - waypoint_start_time_).seconds() > kWaypointTimeout) {
        abort("point " + std::to_string(wp_index_ + 1) + " non atteint en "
          + std::to_string(static_cast<int>(kWaypointTimeout)) + " s");
      }
      return;
    }

    // Maintien sur le point courant
    if ((now - *arrived_time_).seconds() >= mission_[wp_index_].hold_s) {
      if (wp_index_ + 1 < mission_.size()) {
        ++wp_index_;
        arrived_time_.reset();
        waypoint_start_time_ = now;
        RCLCPP_INFO(get_logger(), "Cap sur le point %zu/%zu", wp_index_ + 1, mission_.size());
      } else {
        change_state(State::Landing);
      }
    }
  }

  Ned to_ned(const Waypoint & wp) const
  {
    return Ned{home_->north + wp.north, home_->east + wp.east, home_->down - wp.up};
  }

  float distance_to(const Waypoint & wp) const
  {
    if (!position_ || !position_->xy_valid || !position_->z_valid) {
      return std::numeric_limits<float>::infinity();   // position non fiable : jamais "atteint"
    }
    const Ned target = to_ned(wp);
    const float dn = position_->x - target.north;
    const float de = position_->y - target.east;
    const float dd = position_->z - target.down;
    return std::sqrt(dn * dn + de * de + dd * dd);
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

  bool is_landing() const
  {
    return status_ && status_->nav_state == VehicleStatus::NAVIGATION_STATE_AUTO_LAND;
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
      case State::Mission:         return "Mission";
      case State::Landing:         return "Landing";
      case State::Done:            return "Done";
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

  void publish_setpoint(const Waypoint & wp)
  {
    const Ned target = to_ned(wp);
    const float nan = std::nanf("");
    TrajectorySetpoint msg{};
    msg.position = {target.north, target.east, target.down};
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
  rclcpp::Subscription<VehicleLocalPosition>::SharedPtr position_sub_;
  rclcpp::Subscription<VehicleCommandAck>::SharedPtr ack_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::optional<VehicleStatus> status_;
  std::optional<VehicleLocalPosition> position_;
  std::optional<Ned> home_;

  std::vector<Waypoint> mission_;
  size_t wp_index_ = 0;
  rclcpp::Time waypoint_start_time_;
  std::optional<rclcpp::Time> arrived_time_;

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