#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_msgs/msg/perception_result.hpp"
#include "pbv_dcu_msgs/msg/fault_status.hpp"
#include "pbv_dcu_msgs/msg/state_mode.hpp"
#include "state_manager_logic.h"

class StateManagerNode : public rclcpp::Node
{
public:
  StateManagerNode() : Node("pbv_02_state_manager")
  {
    cfg_.stop_distance_m = static_cast<float>(declare_parameter<double>("stop_distance_m", 1.0));
    cfg_.approach_distance_m = static_cast<float>(declare_parameter<double>("approach_distance_m", 5.0));
    pub_ = create_publisher<pbv_dcu_msgs::msg::StateMode>("/pbv/state/mode", 10);
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
    DcuStateMode out{};
    perception.target_valid = perception_.target_valid ? 1u : 0u;
    perception.traffic_light_state = perception_.traffic_light_state;
    perception.traffic_light_distance_m = perception_.traffic_light_distance_m;
    fault.fail_safe = fault_.fail_safe ? 1u : 0u;
    fault.lane_fault = fault_.lane_fault ? 1u : 0u;
    state_manager_process(&cfg_, &fault, &perception, &out);

    pbv_dcu_msgs::msg::StateMode ros;
    ros.header = header;
    ros.mode = out.mode;
    ros.init = out.init != 0u;
    ros.ready = out.ready != 0u;
    ros.cruise = out.cruise != 0u;
    ros.approach_stop = out.approach_stop != 0u;
    ros.stopped = out.stopped != 0u;
    ros.degraded = out.degraded != 0u;
    ros.failsafe = out.failsafe != 0u;
    pub_->publish(ros);
  }
  DcuStateConfig cfg_{};
  bool have_perception_{false};
  bool have_fault_{false};
  pbv_dcu_msgs::msg::PerceptionResult perception_{};
  pbv_dcu_msgs::msg::FaultStatus fault_{};
  rclcpp::Subscription<pbv_dcu_msgs::msg::PerceptionResult>::SharedPtr perception_sub_;
  rclcpp::Subscription<pbv_dcu_msgs::msg::FaultStatus>::SharedPtr fault_sub_;
  rclcpp::Publisher<pbv_dcu_msgs::msg::StateMode>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateManagerNode>());
  rclcpp::shutdown();
  return 0;
}
