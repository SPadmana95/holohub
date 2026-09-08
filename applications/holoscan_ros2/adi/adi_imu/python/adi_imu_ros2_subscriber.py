import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Temperature


class ImuRos2Subscriber(Node):
    def __init__(self):
        super().__init__("adi_imu_ros2_subscriber")
        self.create_subscription(Imu, "imu/data_raw", self.on_imu, 10)
        self.create_subscription(Temperature, "imu/temperature", self.on_temperature, 10)

    def on_imu(self, message):
        self.get_logger().info(
            "IMU gyro=(%.4f, %.4f, %.4f) accel=(%.4f, %.4f, %.4f)"
            % (
                message.angular_velocity.x,
                message.angular_velocity.y,
                message.angular_velocity.z,
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            )
        )

    def on_temperature(self, message):
        self.get_logger().info("IMU temperature=%.3f C" % message.temperature)


def main():
    rclpy.init()
    node = ImuRos2Subscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
