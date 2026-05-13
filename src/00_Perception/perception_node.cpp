#include <memory>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_msgs/msg/oakd_features.hpp"
#include "pbv_dcu_msgs/msg/perception_result.hpp"
#include "perception_logic.h"

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode() : Node("pbv_00_perception")
  {
    cfg_.lane_valid_threshold = static_cast<float>(declare_parameter<double>("lane_valid_threshold", 0.55));
    cfg_.target_valid_threshold = static_cast<float>(declare_parameter<double>("target_valid_threshold", 0.50));
    pub_ = create_publisher<pbv_dcu_msgs::msg::PerceptionResult>("/pbv/perception/result", 10);
    sub_ = create_subscription<pbv_dcu_msgs::msg::OakdFeatures>(
      "/oakd/features", 10, std::bind(&PerceptionNode::on_features, this, std::placeholders::_1));
  }
private:
  void on_features(const pbv_dcu_msgs::msg::OakdFeatures::SharedPtr msg)
  {
    DcuOakdFeatures in{};
    DcuPerceptionResult out{};
    in.camera_ok = msg->camera_ok ? 1u : 0u;
    in.timestamp_ms = msg->timestamp_ms;
    in.traffic_light_confidence = msg->traffic_light_confidence;
    in.lane_confidence = msg->lane_confidence;
    in.lane_offset_m = msg->lane_offset_m;
    in.lane_heading_rad = msg->lane_heading_rad;
    in.traffic_light_state = msg->traffic_light_state;
    in.traffic_light_distance_m = msg->traffic_light_distance_m;
    perception_process(&cfg_, &in, &out);

    pbv_dcu_msgs::msg::PerceptionResult ros;
    ros.header = msg->header;
    ros.timestamp_ms = out.timestamp_ms;
    ros.camera_ok = out.camera_ok != 0u;
    ros.lane_valid = out.lane_valid != 0u;
    ros.lane_confidence = out.lane_confidence;
    ros.lane_offset_m = out.lane_offset_m;
    ros.lane_heading_rad = out.lane_heading_rad;
    ros.target_valid = out.target_valid != 0u;
    ros.traffic_light_state = out.traffic_light_state;
    ros.traffic_light_confidence = out.traffic_light_confidence;
    ros.traffic_light_distance_m = out.traffic_light_distance_m;
    pub_->publish(ros);
  }
  DcuPerceptionConfig cfg_{};
  rclcpp::Subscription<pbv_dcu_msgs::msg::OakdFeatures>::SharedPtr sub_;
  rclcpp::Publisher<pbv_dcu_msgs::msg::PerceptionResult>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PerceptionNode>());
  rclcpp::shutdown();
  return 0;
}
