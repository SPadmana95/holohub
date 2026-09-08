#include <algorithm>
#include <cstddef>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class A2BAudioRos2Subscriber : public rclcpp::Node {
public:
  A2BAudioRos2Subscriber() : Node("a2b_audio_ros2_subscriber") {
    raw_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "a2b_audio/raw", 10, [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr message) {
          RCLCPP_INFO(get_logger(), "raw audio: %zu samples", message->data.size());
        });
    beam_subscription_ = create_subscription<std_msgs::msg::Float32MultiArray>(
        "a2b_audio/beamformed", 10,
        [this](const std_msgs::msg::Float32MultiArray::ConstSharedPtr message) {
          RCLCPP_INFO(get_logger(), "beamformed audio: %zu samples", message->data.size());
        });
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr raw_subscription_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr beam_subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2BAudioRos2Subscriber>());
  rclcpp::shutdown();
  return 0;
}
