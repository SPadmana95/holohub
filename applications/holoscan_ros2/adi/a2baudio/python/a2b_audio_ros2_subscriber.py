import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


class A2BAudioRos2Subscriber(Node):
    def __init__(self):
        super().__init__("a2b_audio_ros2_subscriber")
        self.create_subscription(Float32MultiArray, "a2b_audio/raw", self.on_raw, 10)
        self.create_subscription(Float32MultiArray, "a2b_audio/beamformed", self.on_beamformed, 10)

    def on_raw(self, message):
        self.get_logger().info("raw audio: %d samples" % len(message.data))

    def on_beamformed(self, message):
        self.get_logger().info("beamformed audio: %d samples" % len(message.data))


def main():
    rclpy.init()
    node = A2BAudioRos2Subscriber()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
