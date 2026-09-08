import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, MultiArrayDimension


class A2BAudioRos2Publisher(Node):
    """ROS 2 transport example publishing raw and beamformed audio."""

    def __init__(self):
        super().__init__("a2b_audio_ros2_publisher")
        self.raw_pub = self.create_publisher(Float32MultiArray, "a2b_audio/raw", 10)
        self.beam_pub = self.create_publisher(Float32MultiArray, "a2b_audio/beamformed", 10)
        self.sample_rate = 48000
        self.channels = 4
        self.phase = 0
        self.timer = self.create_timer(0.01, self.publish_audio)

    def publish_audio(self):
        samples = 480
        raw = []
        beam = []
        for index in range(samples):
            value = math.sin(2.0 * math.pi * 440.0 * (self.phase + index) / self.sample_rate)
            raw.extend(value for _ in range(self.channels))
            beam.append(value)
        self.phase += samples
        self.raw_pub.publish(self.message(raw, self.channels, samples))
        self.beam_pub.publish(self.message(beam, 1, samples))

    @staticmethod
    def message(values, channels, samples):
        message = Float32MultiArray()
        message.layout.dim = [
            MultiArrayDimension(label="channels", size=channels, stride=channels * samples),
            MultiArrayDimension(label="samples", size=samples, stride=samples),
        ]
        message.data = values
        return message


def main():
    rclpy.init()
    node = A2BAudioRos2Publisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
