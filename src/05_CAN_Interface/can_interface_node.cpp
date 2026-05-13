#include <array>
#include <chrono>
#include <cstring>
#include <memory>
#include <string>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <functional>
#include "rclcpp/rclcpp.hpp"
#include "pbv_dcu_msgs/msg/control_command.hpp"
#include "pbv_dcu_msgs/msg/aurix_diagnostic.hpp"
#include "can_interface_logic.h"

using namespace std::chrono_literals;

class CanInterfaceNode : public rclcpp::Node
{
public:
  CanInterfaceNode() : Node("pbv_05_can_interface")
  {
    can_if_ = declare_parameter<std::string>("can_interface", "can0");
    max_speed_mps_ = static_cast<float>(declare_parameter<double>("can_pack_max_speed_mps", 1.0));
    diag_pub_ = create_publisher<pbv_dcu_msgs::msg::AurixDiagnostic>("/pbv/aurix/diagnostic", 10);
    cmd_sub_ = create_subscription<pbv_dcu_msgs::msg::ControlCommand>(
      "/pbv/control/command", 10, std::bind(&CanInterfaceNode::on_command, this, std::placeholders::_1));
    open_socket();
    timer_ = create_wall_timer(10ms, std::bind(&CanInterfaceNode::poll_rx, this));
  }
  ~CanInterfaceNode() override
  {
    if (sock_ >= 0) close(sock_);
  }
private:
  void open_socket()
  {
    struct ifreq ifr;
    struct sockaddr_can addr;
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
    {
      RCLCPP_WARN(get_logger(), "SocketCAN open failed. Commands will not be transmitted until interface is available.");
      return;
    }
    std::memset(&ifr, 0, sizeof(ifr));
    std::strncpy(ifr.ifr_name, can_if_.c_str(), IFNAMSIZ - 1);
    if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
    {
      RCLCPP_WARN(get_logger(), "CAN interface %s not found", can_if_.c_str());
      close(sock_);
      sock_ = -1;
      return;
    }
    std::memset(&addr, 0, sizeof(addr));
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0)
    {
      RCLCPP_WARN(get_logger(), "CAN bind failed on %s", can_if_.c_str());
      close(sock_);
      sock_ = -1;
      return;
    }
    const int flags = fcntl(sock_, F_GETFL, 0);
    (void)fcntl(sock_, F_SETFL, flags | O_NONBLOCK);
  }

  void on_command(const pbv_dcu_msgs::msg::ControlCommand::SharedPtr msg)
  {
    if (sock_ < 0) return;
    DcuControlCommand cmd{};
    std::array<uint8_t, 8> data{};
    struct can_frame frame;
    cmd.target_rpm = msg->target_rpm;
    cmd.drive_state = msg->drive_state;
    cmd.fault_summary = msg->fault_summary;
    cmd.target_speed_mps = msg->target_speed_mps;
    cmd.steering_cmd = msg->steering_cmd;
    cmd.speed_level = msg->speed_level;
    cmd.enable = msg->enable ? 1u : 0u;
    cmd.estop = msg->estop ? 1u : 0u;
    const uint8_t seq = seq_++;

    can_interface_pack_drive_cmd(&cmd, max_speed_mps_, seq, data.data());
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = PBV_CAN_ID_DRIVE_CMD;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data.data(), 8);
    (void)write(sock_, &frame, sizeof(frame));

    can_interface_pack_steer_cmd(&cmd, seq, data.data());
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = PBV_CAN_ID_STEER_CMD;
    frame.can_dlc = 8;
    std::memcpy(frame.data, data.data(), 8);
    (void)write(sock_, &frame, sizeof(frame));
  }

  void poll_rx()
  {
    if (sock_ < 0) return;
    struct can_frame frame;
    while (read(sock_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame)))
    {
      const uint32_t id = frame.can_id & CAN_EFF_MASK;
      const uint8_t * d = frame.data;
      if (id == PBV_CAN_ID_SPEED_FB)
      {
        diag_.target_rpm = static_cast<float>(can_interface_get_s16_le(d, 0)) * 0.1f;
        diag_.raw_rpm = static_cast<float>(can_interface_get_s16_le(d, 2)) * 0.1f;
        diag_.avg_rpm = static_cast<float>(can_interface_get_s16_le(d, 4)) * 0.1f;
        diag_.feedback_mask = d[6];
        diag_.speed_level = d[7];
      }
      else if (id == PBV_CAN_ID_PID)
      {
        diag_.pid_error_rpm = static_cast<float>(can_interface_get_s16_le(d, 0)) * 0.1f;
        diag_.pid_trim_duty = can_interface_get_u16_le(d, 2);
        diag_.brake_duty = can_interface_get_u16_le(d, 4);
        diag_.brake_active = d[6] != 0u;
        diag_.stop_reason = d[7];
      }
      else if (id == PBV_CAN_ID_DUTY)
      {
        diag_.left_duty = can_interface_get_u16_le(d, 0);
        diag_.right_duty = can_interface_get_u16_le(d, 2);
        diag_.left_dir = static_cast<int8_t>(d[4]);
        diag_.right_dir = static_cast<int8_t>(d[5]);
        diag_.flags = d[6];
        diag_.buffer_count = d[7];
      }
      else if (id == PBV_CAN_ID_ENC)
      {
        diag_.enc0_count = can_interface_get_s16_le(d, 0);
        diag_.enc1_count = can_interface_get_s16_le(d, 2);
        diag_.enc2_count = can_interface_get_s16_le(d, 4);
        diag_.enc3_count = can_interface_get_s16_le(d, 6);
      }
      diag_.header.stamp = this->now();
      diag_.header.frame_id = "tc275";
      diag_pub_->publish(diag_);
    }
  }

  int sock_{-1};
  std::string can_if_;
  float max_speed_mps_{1.0f};
  uint8_t seq_{0u};
  pbv_dcu_msgs::msg::AurixDiagnostic diag_{};
  rclcpp::Subscription<pbv_dcu_msgs::msg::ControlCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<pbv_dcu_msgs::msg::AurixDiagnostic>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
