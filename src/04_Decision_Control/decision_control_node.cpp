#include <chrono>
#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_msgs/msg/perception_result.hpp"
#include "pbv_dcu_msgs/msg/fault_status.hpp"
#include "pbv_dcu_msgs/msg/state_mode.hpp"
#include "pbv_dcu_msgs/msg/safety_constraint.hpp"
#include "pbv_dcu_msgs/msg/control_command.hpp"
#include "decision_control_logic.h"

using namespace std::chrono_literals;

class DecisionControlNode : public rclcpp::Node
{
public:
  DecisionControlNode() : Node("pbv_04_decision_control")
  {
    cfg_.base_speed_mps = static_cast<float>(declare_parameter<double>("base_speed_mps", 0.8));
    cfg_.rpm_per_mps = static_cast<float>(declare_parameter<double>("rpm_per_mps", 220.0));
    cfg_.lane_offset_gain = static_cast<float>(declare_parameter<double>("lane_offset_gain", 0.75));
    cfg_.lane_heading_gain = static_cast<float>(declare_parameter<double>("lane_heading_gain", 0.45));
    pub_ = create_publisher<pbv_dcu_msgs::msg::ControlCommand>("/pbv/control/command", 10);
    perception_sub_ = create_subscription<pbv_dcu_msgs::msg::PerceptionResult>("/pbv/perception/result", 10, [this](pbv_dcu_msgs::msg::PerceptionResult::SharedPtr msg){ perception_ = *msg; have_perception_ = true; });
    fault_sub_ = create_subscription<pbv_dcu_msgs::msg::FaultStatus>("/pbv/fault/status", 10, [this](pbv_dcu_msgs::msg::FaultStatus::SharedPtr msg){ fault_ = *msg; have_fault_ = true; });
    state_sub_ = create_subscription<pbv_dcu_msgs::msg::StateMode>("/pbv/state/mode", 10, [this](pbv_dcu_msgs::msg::StateMode::SharedPtr msg){ state_ = *msg; have_state_ = true; });
    safety_sub_ = create_subscription<pbv_dcu_msgs::msg::SafetyConstraint>("/pbv/safety/constraint", 10, [this](pbv_dcu_msgs::msg::SafetyConstraint::SharedPtr msg){ safety_ = *msg; have_safety_ = true; });
    timer_ = create_wall_timer(20ms, std::bind(&DecisionControlNode::on_timer, this));
  }
private:
  void on_timer()
  {
    if (!have_perception_ || !have_fault_ || !have_state_ || !have_safety_) return;
    DcuPerceptionResult perception{};
    DcuFaultStatus fault{};
    DcuStateMode state{};
    DcuSafetyConstraint safety{};
    DcuControlCommand out{};

    perception.lane_valid = perception_.lane_valid ? 1u : 0u;
    perception.lane_offset_m = perception_.lane_offset_m;
    perception.lane_heading_rad = perception_.lane_heading_rad;
    fault.fail_safe = fault_.fail_safe ? 1u : 0u;
    fault.fault_bits = fault_.fault_bits;
    state.mode = state_.mode;
    safety.failsafe = safety_.failsafe ? 1u : 0u;
    safety.max_speed_mps = safety_.max_speed_mps;
    safety.max_steer_norm = safety_.max_steer_norm;
    decision_control_process(&cfg_, &perception, &fault, &state, &safety, &out);

    pbv_dcu_msgs::msg::ControlCommand ros;
    ros.header.stamp = this->now();
    ros.header.frame_id = "pbv_dcu";
    ros.target_rpm = out.target_rpm;
    ros.drive_state = out.drive_state;
    ros.fault_summary = out.fault_summary;
    ros.target_speed_mps = out.target_speed_mps;
    ros.steering_cmd = out.steering_cmd;
    ros.speed_level = out.speed_level;
    ros.enable = out.enable != 0u;
    ros.estop = out.estop != 0u;
    pub_->publish(ros);
  }
  DcuDecisionConfig cfg_{};
  bool have_perception_{false};
  bool have_fault_{false};
  bool have_state_{false};
  bool have_safety_{false};
  pbv_dcu_msgs::msg::PerceptionResult perception_{};
  pbv_dcu_msgs::msg::FaultStatus fault_{};
  pbv_dcu_msgs::msg::StateMode state_{};
  pbv_dcu_msgs::msg::SafetyConstraint safety_{};
  rclcpp::Subscription<pbv_dcu_msgs::msg::PerceptionResult>::SharedPtr perception_sub_;
  rclcpp::Subscription<pbv_dcu_msgs::msg::FaultStatus>::SharedPtr fault_sub_;
  rclcpp::Subscription<pbv_dcu_msgs::msg::StateMode>::SharedPtr state_sub_;
  rclcpp::Subscription<pbv_dcu_msgs::msg::SafetyConstraint>::SharedPtr safety_sub_;
  rclcpp::Publisher<pbv_dcu_msgs::msg::ControlCommand>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DecisionControlNode>());
  rclcpp::shutdown();
  return 0;
}
