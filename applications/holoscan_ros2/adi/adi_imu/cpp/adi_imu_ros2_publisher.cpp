#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include <holoscan/holoscan.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

#include "adi_imu_op.hpp"

class ImuRos2PublisherOp : public holoscan::Operator {
public:
  HOLOSCAN_OPERATOR_FORWARD_ARGS(ImuRos2PublisherOp)
  ImuRos2PublisherOp() = default;

  void setup(holoscan::OperatorSpec& spec) override {
    spec.input<std::string>("imu_data");
    spec.input<std::string>("temp_data");
  }

  void initialize() override {
    holoscan::Operator::initialize();
    imu_publisher_ = node_->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
    temperature_publisher_ =
        node_->create_publisher<sensor_msgs::msg::Temperature>("imu/temperature", 10);
  }

  void compute(holoscan::InputContext& input, holoscan::OutputContext&,
               holoscan::ExecutionContext&) override {
    const auto imu = input.receive<std::string>("imu_data");
    const auto temperature = input.receive<std::string>("temp_data");
    if (!imu || !temperature) return;
    const auto imu_fields = split(imu.value());
    const auto temperature_fields = split(temperature.value());
    if (imu_fields.size() != 7 || temperature_fields.size() != 2) return;

    const double stamp = std::stod(imu_fields[0]);
    sensor_msgs::msg::Imu message;
    message.header.stamp.sec = static_cast<int32_t>(stamp);
    message.header.stamp.nanosec =
        static_cast<uint32_t>((stamp - message.header.stamp.sec) * 1e9);
    message.header.frame_id = "imu_link";
    message.angular_velocity.x = std::stod(imu_fields[1]) * 0.0001;
    message.angular_velocity.y = std::stod(imu_fields[2]) * 0.0001;
    message.angular_velocity.z = std::stod(imu_fields[3]) * 0.0001;
    message.linear_acceleration.x = std::stod(imu_fields[4]) * 0.0001;
    message.linear_acceleration.y = std::stod(imu_fields[5]) * 0.0001;
    message.linear_acceleration.z = std::stod(imu_fields[6]) * 0.0001;
    message.orientation_covariance[0] = -1.0;
    imu_publisher_->publish(message);

    sensor_msgs::msg::Temperature temperature_message;
    temperature_message.header = message.header;
    temperature_message.temperature = std::stod(temperature_fields[1]) * 0.00565 + 25.0;
    temperature_publisher_->publish(temperature_message);
  }

  void set_node(const std::shared_ptr<rclcpp::Node>& node) { node_ = node; }

private:
  static std::vector<std::string> split(const std::string& value) {
    std::vector<std::string> fields;
    std::string field;
    for (const char character : value) {
      if (character == ',') {
        fields.push_back(field);
        field.clear();
      } else {
        field += character;
      }
    }
    fields.push_back(field);
    return fields;
  }

  std::shared_ptr<rclcpp::Node> node_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_publisher_;
};

class ImuRos2Application : public holoscan::Application {
public:
  explicit ImuRos2Application(std::shared_ptr<rclcpp::Node> node) : node_(std::move(node)) {}

  void compose() override {
    auto hardware = make_operator<hololink::ops::ImuHardwareOp>(
        "imu_hardware", holoscan::Arg("hsb_ip", std::string("192.168.0.2")),
        holoscan::Arg("sync_freq", 120), holoscan::Arg("output_freq", 240),
        holoscan::Arg("spi_port", 2), holoscan::Arg("spi_cs", 0),
        holoscan::Arg("spi_div", 10), holoscan::Arg("spi_cpol", 1),
        holoscan::Arg("spi_cpha", 1), holoscan::Arg("reset_pin", 9),
        holoscan::Arg("dr_pin", 8), holoscan::Arg("sync_pin", 13));
    auto publisher = make_operator<ImuRos2PublisherOp>("ros2_publisher");
    publisher->set_node(node_);
    add_flow(hardware, publisher, {{"imu_data", "imu_data"}, {"temp_data", "temp_data"}});
  }

private:
  std::shared_ptr<rclcpp::Node> node_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("adi_imu_ros2_publisher");
  ImuRos2Application application(node);
  application.run();
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
