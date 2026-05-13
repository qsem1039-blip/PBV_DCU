#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_msgs/msg/perception_result.hpp"
#include "pbv_dcu_msgs/msg/fault_status.hpp"
#include "fault_management_logic.h"

class FaultManagementNode : public rclcpp::Node
{
public:
  FaultManagementNode() : Node("pbv_01_fault_management")
  {
    cfg_.timeout_ms = static_cast<uint32_t>(declare_parameter<int>("timeout_ms", 200));
    cfg_.lane_min_confidence = static_cast<float>(declare_parameter<double>("lane_min_confidence", 0.45));
    pub_ = create_publisher<pbv_dcu_msgs::msg::FaultStatus>("/pbv/fault/status", 10);
    sub_ = create_subscription<pbv_dcu_msgs::msg::PerceptionResult>(
      "/pbv/perception/result", 10, std::bind(&FaultManagementNode::on_perception, this, std::placeholders::_1));
  }
private:
  uint32_t now_ms() const
  {
    return static_cast<uint32_t>(this->now().nanoseconds() / 1000000ULL);
  }
  void on_perception(const pbv_dcu_msgs::msg::PerceptionResult::SharedPtr msg)
  {
    DcuPerceptionResult in{};
    DcuFaultStatus out{};
    cfg_.now_ms = now_ms();
    in.timestamp_ms = msg->timestamp_ms;
    in.camera_ok = msg->camera_ok ? 1u : 0u;
    in.lane_valid = msg->lane_valid ? 1u : 0u;
    in.lane_confidence = msg->lane_confidence;
    in.target_valid = msg->target_valid ? 1u : 0u;
    in.traffic_light_state = msg->traffic_light_state;
    in.traffic_light_distance_m = msg->traffic_light_distance_m;
    fault_management_process(&cfg_, &mem_, &in, &out);

    pbv_dcu_msgs::msg::FaultStatus ros;
    ros.header = msg->header;
    ros.requested_gate = out.requested_gate != 0u;
    ros.sensor_health = out.sensor_health;
    ros.timeout_fault = out.timeout_fault != 0u;
    ros.lane_fault = out.lane_fault != 0u;
    ros.tps_fault = out.tps_fault != 0u;
    ros.fault_bits = out.fault_bits;
    ros.fail_safe = out.fail_safe != 0u;
    ros.stale_counter = out.stale_counter;
    ros.lost_counter = out.lost_counter;
    pub_->publish(ros);
  }
  DcuFaultConfig cfg_{};
  DcuFaultMemory mem_{};
  rclcpp::Subscription<pbv_dcu_msgs::msg::PerceptionResult>::SharedPtr sub_;
  rclcpp::Publisher<pbv_dcu_msgs::msg::FaultStatus>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FaultManagementNode>());
  rclcpp::shutdown();
  return 0;
}
