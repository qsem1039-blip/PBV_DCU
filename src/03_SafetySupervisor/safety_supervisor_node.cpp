#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_msgs/msg/perception_result.hpp"
#include "pbv_dcu_msgs/msg/fault_status.hpp"
#include "pbv_dcu_msgs/msg/safety_constraint.hpp"
#include "safety_supervisor_logic.h"

class SafetySupervisorNode : public rclcpp::Node
{
public:
  SafetySupervisorNode() : Node("pbv_03_safety_supervisor")
  {
    cfg_.nominal_max_speed_mps = static_cast<float>(declare_parameter<double>("nominal_max_speed_mps", 1.0));
    cfg_.degraded_max_speed_mps = static_cast<float>(declare_parameter<double>("degraded_max_speed_mps", 0.35));
    cfg_.stop_distance_m = static_cast<float>(declare_parameter<double>("stop_distance_m", 1.0));
    cfg_.target_near_distance_m = static_cast<float>(declare_parameter<double>("target_near_distance_m", 5.0));
    pub_ = create_publisher<pbv_dcu_msgs::msg::SafetyConstraint>("/pbv/safety/constraint", 10);
    perception_sub_ = create_subscription<pbv_dcu_msgs::msg::PerceptionResult>(
      "/pbv/perception/result", 10, [this](pbv_dcu_msgs::msg::PerceptionResult::SharedPtr msg){ perception_ = *msg; have_perception_ = true; publish_if_ready(msg->header); });
    fault_sub_ = create_subscription<pbv_dcu_msgs::msg::FaultStatus>(
      "/pbv/fault/status", 10, [this](pbv_dcu_msgs::msg::FaultStatus::SharedPtr msg){ fault_ = *msg; have_fault_ = true; publish_if_ready(msg->header); });
  }
private:
  void publish_if_ready(const std_msgs::msg::Header & header)
  {
    if (!have_perception_ || !have_fault_) return;
    DcuPerceptionResult perception{};
    DcuFaultStatus fault{};
    DcuSafetyConstraint out{};
    perception.target_valid = perception_.target_valid ? 1u : 0u;
    perception.traffic_light_state = perception_.traffic_light_state;
    perception.traffic_light_distance_m = perception_.traffic_light_distance_m;
    fault.timeout_fault = fault_.timeout_fault ? 1u : 0u;
    fault.lane_fault = fault_.lane_fault ? 1u : 0u;
    fault.fail_safe = fault_.fail_safe ? 1u : 0u;
    safety_supervisor_process(&cfg_, &fault, &perception, &out);

    pbv_dcu_msgs::msg::SafetyConstraint ros;
    ros.header = header;
    ros.stale_data = out.stale_data != 0u;
    ros.red_light = out.red_light != 0u;
    ros.target_near = out.target_near != 0u;
    ros.degraded_lane = out.degraded_lane != 0u;
    ros.failsafe = out.failsafe != 0u;
    ros.stop_line = out.stop_line != 0u;
    ros.max_speed_mps = out.max_speed_mps;
    ros.max_steer_norm = out.max_steer_norm;
    ros.safety_bits = out.safety_bits;
    pub_->publish(ros);
  }
  DcuSafetyConfig cfg_{};
  bool have_perception_{false};
  bool have_fault_{false};
  pbv_dcu_msgs::msg::PerceptionResult perception_{};
  pbv_dcu_msgs::msg::FaultStatus fault_{};
  rclcpp::Subscription<pbv_dcu_msgs::msg::PerceptionResult>::SharedPtr perception_sub_;
  rclcpp::Subscription<pbv_dcu_msgs::msg::FaultStatus>::SharedPtr fault_sub_;
  rclcpp::Publisher<pbv_dcu_msgs::msg::SafetyConstraint>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisorNode>());
  rclcpp::shutdown();
  return 0;
}
