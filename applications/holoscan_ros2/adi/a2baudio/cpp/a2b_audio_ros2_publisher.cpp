#include <cmath>
#include <chrono>
#include <cstddef>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class A2BAudioRos2Publisher : public rclcpp::Node {
public:
  A2BAudioRos2Publisher() : Node("a2b_audio_ros2_publisher"), phase_(0) {
    raw_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("a2b_audio/raw", 10);
    beam_publisher_ = create_publisher<std_msgs::msg::Float32MultiArray>("a2b_audio/beamformed", 10);
    timer_ = create_wall_timer(std::chrono::milliseconds(10), [this] { publish_audio(); });
  }

private:
  void publish_audio() {
    constexpr std::size_t samples = 480;
    constexpr std::size_t channels = 4;
    constexpr float sample_rate = 48000.0f;
    std_msgs::msg::Float32MultiArray raw;
    std_msgs::msg::Float32MultiArray beamformed;
    raw.layout.dim.resize(2);
    raw.layout.dim[0].label = "channels";
    raw.layout.dim[0].size = channels;
    raw.layout.dim[0].stride = channels * samples;
    raw.layout.dim[1].label = "samples";
    raw.layout.dim[1].size = samples;
    raw.layout.dim[1].stride = samples;
    beamformed.layout.dim = raw.layout.dim;
    beamformed.layout.dim[0].size = 1;
    beamformed.layout.dim[0].stride = samples;
    raw.data.reserve(channels * samples);
    beamformed.data.reserve(samples);
    for (std::size_t index = 0; index < samples; ++index) {
      const float value = std::sin(2.0f * static_cast<float>(M_PI) * 440.0f *
                                   static_cast<float>(phase_ + index) / sample_rate);
      for (std::size_t channel = 0; channel < channels; ++channel) raw.data.push_back(value);
      beamformed.data.push_back(value);
    }
    phase_ += samples;
    raw_publisher_->publish(raw);
    beam_publisher_->publish(beamformed);
  }

  std::size_t phase_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr raw_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr beam_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<A2BAudioRos2Publisher>());
  rclcpp::shutdown();
  return 0;
}
