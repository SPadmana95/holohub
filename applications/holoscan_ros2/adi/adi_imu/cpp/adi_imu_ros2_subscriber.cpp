#include <cstdio>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/temperature.hpp>

class ImuRos2Subscriber : public rclcpp::Node {
public:
  ImuRos2Subscriber() : Node("adi_imu_ros2_subscriber") {
    imu_subscription_ = create_subscription<sensor_msgs::msg::Imu>(
        "imu/data_raw", 10,
        [this](sensor_msgs::msg::Imu::ConstSharedPtr message) {
          RCLCPP_INFO(get_logger(), "IMU gyro=(%.4f, %.4f, %.4f) accel=(%.4f, %.4f, %.4f)",
                      message->angular_velocity.x, message->angular_velocity.y,
                      message->angular_velocity.z, message->linear_acceleration.x,
                      message->linear_acceleration.y, message->linear_acceleration.z);
        });
    temperature_subscription_ = create_subscription<sensor_msgs::msg::Temperature>(
        "imu/temperature", 10,
        [this](sensor_msgs::msg::Temperature::ConstSharedPtr message) {
          RCLCPP_INFO(get_logger(), "IMU temperature=%.3f C", message->temperature);
        });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::Temperature>::SharedPtr temperature_subscription_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuRos2Subscriber>());
  rclcpp::shutdown();
  return 0;
}
